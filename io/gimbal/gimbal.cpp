#include "gimbal.hpp"

#include <cmath>
#include <cstring>
#include <vector>

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
Gimbal::Gimbal(const std::string & config_path)
: last_q_(Eigen::Quaterniond::Identity(), std::chrono::steady_clock::now())
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");

  // 下位机若未实现 CRC，可置 crc_check: false 关闭校验（缺省 true）
  if (yaml["crc_check"]) crc_check_ = yaml["crc_check"].as<bool>();
  if (!crc_check_)
    tools::logger()->warn(
      "[Gimbal] crc_check 已关闭，改用单位四元数做合理性校验 —— "
      "仅用于下位机固件尚未实现 CRC 时的联调，电控修好后请改回 true");

  try {
    serial_.setPort(com_port);
    serial_.open();
    configure_port();
  } catch (const std::exception & e) {
    // 启动时电控未上电/设备未枚举不致命：读线程会对未打开的端口持续重试，
    // 电控就绪后自动恢复（与协议文档"上电顺序"约定一致）
    tools::logger()->error(
      "[Gimbal] Failed to open serial: {} —— 将在后台持续重试，电控上电后自动恢复", e.what());
  }

  thread_ = std::thread(&Gimbal::read_thread, this);

  // 等待首帧 IMU 数据（电控必须先于视觉上电）。超时后不阻塞：
  // 模式保持 IDLE，q() 用 last_q_ 兜底，电控就绪后自动恢复。
  for (int i = 0; i < 300 && queue_.empty(); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  if (queue_.empty())
    tools::logger()->warn("[Gimbal] 3s 内未收到电控首帧，继续运行（请确认电控已上电）");
  else
    tools::logger()->info("[Gimbal] First q received.");
}

Gimbal::~Gimbal()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  serial_.close();
}

GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

void Gimbal::configure_port()
{
  // 每次 open() 后必须重新下发配置：重开/设备重枚举后的 tty 是内核默认 termios
  // （ICANON 按行缓冲、ISTRIP 剥第 7 位、IXON 吞掉 0x13/0x11、OPOST 把 0x0a 扩成
  // 0x0d0a），二进制帧会整体损坏 —— 这是此前重连后 96%+ 帧解析失败的根因。
  // setBaudrate 内部会调用 reconfigurePort() 把端口设为 raw/8N1/无流控。
  // USB CDC 虚拟串口（ttyACM）波特率无实义；若电控经 USB-UART 桥（ttyUSB）接入
  // 则必须与电控固件一致 —— 统一按 115200 设置。
  serial_.setBaudrate(115200);
  serial_.setTimeout(port_timeout_);
}

void Gimbal::write_frame()
{
  // 串口未打开（重连中/设备掉线）时直接丢弃本帧，读线程重连成功后自动恢复
  if (!serial_.isOpen()) return;

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    // 掉线瞬间 fd 已死但 isOpen 仍为 true 时会走到这里；限频告警避免刷屏
    auto now = std::chrono::steady_clock::now();
    if (now - last_tx_warn_ > std::chrono::seconds(1)) {
      last_tx_warn_ = now;
      tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
    }
  }
}

std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE:
      return "IDLE";
    case GimbalMode::AUTO_AIM:
      return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF:
      return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:
      return "BIG_BUFF";
    default:
      return "INVALID";
  }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  using Sample = std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>;

  Sample sample_a;
  // 队列为空（电控未上电/停发）时不阻塞，返回最近一次有效姿态
  if (!queue_.try_pop(sample_a)) {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::get<0>(last_q_);
  }

  while (true) {
    auto [q_a, t_a] = sample_a;

    Sample sample_b;
    if (!queue_.try_pop(sample_b)) {
      // 只剩一帧无法插值，直接返回
      std::lock_guard<std::mutex> lock(mutex_);
      last_q_ = sample_a;
      return q_a;
    }

    auto [q_b, t_b] = sample_b;
    // 必须是"新样本 - 旧样本"（delta_time(a,b)=a-b）。旧实现参数写反导致正常
    // 时序下 t_ab 恒为负、插值分支永远不执行：q() 实际一直返回队列里最新的
    // 姿态并把队列清空，世界系变换用的是 t_img+管线滞后 时刻的姿态，云台自身
    // 运动漏进解算形成正反馈，自瞄时云台剧烈震荡/发散。
    auto t_ab = tools::delta_time(t_b, t_a);
    if (t_ab <= 0) {  // 时间戳未推进（重复帧），丢弃旧帧继续，避免除零产生 NaN
      sample_a = sample_b;
      continue;
    }

    auto t_ac = tools::delta_time(t, t_a);
    // k 截断到 [0,1]：查询时刻 t 早于队列全部样本（如串口重连后队列刚被清空）时
    // 返回最旧的样本，绝不做外推 —— 外推会把云台角速度放大十几倍，直接把
    // 世界系变换污染成垃圾值（2026-10-12 联调实测:自瞄时云台剧烈乱抖的诱因之一）。
    auto k = std::clamp(t_ac / t_ab, 0.0, 1.0);
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();

    if (t <= t_b) {
      std::lock_guard<std::mutex> lock(mutex_);
      last_q_ = sample_b;
      return q_c;
    }

    // t > t_b：两帧都早于查询时刻，丢弃旧帧继续
    sample_a = sample_b;
  }
}

void Gimbal::send(const Command & command)
{
  // Command 只携带位置量，速度/加速度前馈置零；需要前馈请改用 8 参数版本
  send(command.control, command.shoot, command.yaw, 0, 0, command.pitch, 0, 0);
}

void Gimbal::send(io::VisionToGimbal VisionToGimbal)
{
  std::lock_guard<std::mutex> lock(tx_mutex_);

  // NaN/Inf 防护：姿态插值异常时宁发停止帧，不发电控无法执行的垃圾值
  if (!std::isfinite(VisionToGimbal.yaw) || !std::isfinite(VisionToGimbal.pitch)) {
    tools::logger()->warn("[Gimbal] Non-finite yaw/pitch, sending stop frame instead.");
    VisionToGimbal = {};  // 帧头 'SP' 由默认成员初始化保留，mode 归零
  }

  tx_data_.mode = VisionToGimbal.mode;
  tx_data_.yaw = VisionToGimbal.yaw;
  tx_data_.yaw_vel = VisionToGimbal.yaw_vel;
  tx_data_.yaw_acc = VisionToGimbal.yaw_acc;
  tx_data_.pitch = VisionToGimbal.pitch;
  tx_data_.pitch_vel = VisionToGimbal.pitch_vel;
  tx_data_.pitch_acc = VisionToGimbal.pitch_acc;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  write_frame();
}

void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  std::lock_guard<std::mutex> lock(tx_mutex_);

  // NaN/Inf 防护：姿态插值异常时宁发停止帧，不发电控无法执行的垃圾值
  if (!std::isfinite(yaw) || !std::isfinite(pitch)) {
    tools::logger()->warn("[Gimbal] Non-finite yaw/pitch, sending stop frame instead.");
    control = false;
    fire = false;
    yaw = yaw_vel = yaw_acc = pitch = pitch_vel = pitch_acc = 0;
  }

  tx_data_.mode = control ? (fire ? 2 : 1) : 0;
  tx_data_.yaw = yaw;
  tx_data_.yaw_vel = yaw_vel;
  tx_data_.yaw_acc = yaw_acc;
  tx_data_.pitch = pitch;
  tx_data_.pitch_vel = pitch_vel;
  tx_data_.pitch_acc = pitch_acc;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  write_frame();
}

void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");

  // 帧同步状态机：滑动找 'S''P' 帧头，再读 body，校验失败自动重同步。
  enum class State { LOOK_FOR_S, LOOK_FOR_P, READ_BODY } state = State::LOOK_FOR_S;
  uint8_t body[sizeof(GimbalToVision) - sizeof(rx_data_.head)];
  size_t body_size = 0;

  // 批量读缓冲：一次系统调用尽量取走内核缓冲里的全部字节。逐字节 read()
  // 实测只能消耗 ~200 B/s，跟不上电控 ~6 kB/s 连续流，tty 缓冲溢出丢字节后
  // 帧永远错位（此前 96%+ 帧判为无效的直接原因）。
  std::vector<uint8_t> buf(4096);

  int consecutive_read_errors = 0;  // “可读但读不到数据”异常连续计数，用于判设备掉线
  int idle_loops = 0;               // 连续读超时次数（每次最多 ~100ms），~3s 无数据触发重连

  auto reset_sync = [&]() {
    state = State::LOOK_FOR_S;
    body_size = 0;
  };

  // 完整收齐一帧 body 后调用：解码打点、校验、入队与状态更新
  auto accept_frame = [&]() {
    rx_data_.head[0] = 'S';
    rx_data_.head[1] = 'P';
    std::memcpy(
      reinterpret_cast<uint8_t *>(&rx_data_) + sizeof(rx_data_.head), body, sizeof(body));

    auto t = std::chrono::steady_clock::now();
    Eigen::Quaterniond q(rx_data_.q[0], rx_data_.q[1], rx_data_.q[2], rx_data_.q[3]);

    // 1Hz 接收解码日志（联调排查用：原样展示电控发来的帧内容与 CRC 对比，
    // 即使帧校验失败也打印，让电控能直接看到自己发的原始值）
    // packed 字段不能直接传给 fmt（无法绑定引用），先拷贝到局部变量
    static auto last_rx_log = std::chrono::steady_clock::time_point{};
    if (tools::delta_time(std::chrono::steady_clock::now(), last_rx_log) > 1.0) {
      const auto rx_mode = rx_data_.mode;
      const auto rx_yaw = rx_data_.yaw;
      const auto rx_pitch = rx_data_.pitch;
      const auto rx_bullet_speed = rx_data_.bullet_speed;
      const auto rx_bullet_count = rx_data_.bullet_count;
      const auto rx_crc = rx_data_.crc16;
      auto crc_calc = tools::get_crc16(
        reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_) - sizeof(rx_data_.crc16));
      tools::logger()->info(
        "[GimbalRx] mode={} q=({:.3f},{:.3f},{:.3f},{:.3f}) |q|²={:.1f} yaw={:.2f} "
        "pitch={:.2f} bullet_speed={:.2f} count={} crc_rx=0x{:04X} crc_calc=0x{:04X}",
        rx_mode, q.w(), q.x(), q.y(), q.z(), q.squaredNorm(),
        rx_yaw, rx_pitch, rx_bullet_speed, rx_bullet_count, rx_crc, crc_calc);
      // 同频次的 43 字节原始转储，供核对帧结构/字节序/偏移
      std::string hex;
      const auto * raw = reinterpret_cast<const uint8_t *>(&rx_data_);
      for (size_t i = 0; i < sizeof(rx_data_); ++i) hex += fmt::format("{:02X} ", raw[i]);
      tools::logger()->info("[GimbalRxRaw] {}", hex);
      last_rx_log = std::chrono::steady_clock::now();
    }

    if (crc_check_) {
      const bool ok = tools::check_crc16(
        reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_));
      record_crc(ok);  // 只有 crc_check 开启时才统计，关闭时 crc_rate() 恒返回 -1.0
      if (!ok) {
        tools::logger()->debug("[Gimbal] CRC16 check failed.");
        return;
      }
    } else if (std::abs(q.squaredNorm() - 1.0) > 1e-2) {
      // CRC 关闭时的替代校验：单位四元数约束（同 io/cboard.cpp 的 CAN 路径做法）
      tools::logger()->debug("[Gimbal] Invalid quaternion norm: {}", q.squaredNorm());
      return;
    }

    queue_.push({q, t});

    std::lock_guard<std::mutex> lock(mutex_);

    state_.yaw = rx_data_.yaw;
    state_.yaw_vel = rx_data_.yaw_vel;
    state_.pitch = rx_data_.pitch;
    state_.pitch_vel = rx_data_.pitch_vel;
    state_.bullet_speed = rx_data_.bullet_speed;
    state_.bullet_count = rx_data_.bullet_count;

    switch (rx_data_.mode) {
      case 0:
        mode_ = GimbalMode::IDLE;
        break;
      case 1:
        mode_ = GimbalMode::AUTO_AIM;
        break;
      case 2:
        mode_ = GimbalMode::SMALL_BUFF;
        break;
      case 3:
        mode_ = GimbalMode::BIG_BUFF;
        break;
      default:
        mode_ = GimbalMode::IDLE;
        tools::logger()->warn("[Gimbal] Invalid mode: {}", rx_data_.mode);
        break;
    }
  };

  while (!quit_) {
    size_t n = 0;
    try {
      n = serial_.read(buf.data(), buf.size());
    } catch (const std::exception &) {
      // 设备掉线（USB 重枚举）时 select 报可读但 read 拿不到数据，会连续抛异常。
      // 期间收到任何真实数据都会清零计数，因此只在“连续异常且无数据”时判定掉线。
      if (++consecutive_read_errors >= 20) {
        tools::logger()->warn("[Gimbal] Serial read errors, attempting to reconnect...");
        reconnect();
        consecutive_read_errors = 0;
        idle_loops = 0;
        reset_sync();
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));  // 防异常热循环
      }
      continue;
    }

    if (n == 0) {
      // 读超时（100ms 内无数据）。电控停发/掉电时不会永久阻塞，约 3s 触发重连。
      if (++idle_loops >= 30) {
        tools::logger()->warn("[Gimbal] No serial data for 3s, attempting to reconnect...");
        reconnect();
        idle_loops = 0;
        consecutive_read_errors = 0;
        reset_sync();
      }
      continue;
    }

    consecutive_read_errors = 0;
    idle_loops = 0;

    for (size_t i = 0; i < n; ++i) {
      uint8_t byte = buf[i];

      if (state == State::LOOK_FOR_S) {
        if (byte != 'S') continue;
        state = State::LOOK_FOR_P;
      } else if (state == State::LOOK_FOR_P) {
        if (byte == 'P') {
          state = State::READ_BODY;
          body_size = 0;
        } else if (byte != 'S') {
          state = State::LOOK_FOR_S;  // 'S' 后不是 'P'：从该字节重新开始找
        }
        // byte == 'S'：继续等 'P'
      } else {  // READ_BODY
        body[body_size++] = byte;
        if (body_size < sizeof(body)) continue;

        state = State::LOOK_FOR_S;
        accept_frame();
      }
    }
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

void Gimbal::record_crc(bool ok)
{
  ++crc_total_;
  if (ok) ++crc_good_;
  auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(crc_mutex_);
  crc_history_.emplace_back(now, ok);
  // 只保留最近 crc_window_ 秒内的记录
  while (!crc_history_.empty() && now - crc_history_.front().first > crc_window_) {
    crc_history_.pop_front();
  }
}

double Gimbal::crc_rate() const
{
  std::lock_guard<std::mutex> lock(crc_mutex_);
  if (crc_history_.empty()) return -1.0;
  uint64_t good = 0;
  for (const auto & [t, ok] : crc_history_) {
    (void)t;
    if (ok) ++good;
  }
  return 100.0 * static_cast<double>(good) / static_cast<double>(crc_history_.size());
}

void Gimbal::reconnect()
{
  // 持续重连直到成功或退出（电控 USB 约每 8s 重枚举一次，10 次上限的旧实现
  // 会在设备恢复前放弃，之后读超时也永远不再重试）。
  int attempt = 0;
  while (!quit_) {
    ++attempt;
    try {
      serial_.close();
    } catch (...) {
    }

    try {
      serial_.open();
      // 根因修复：重开后的 tty 是内核默认 termios，必须重新下发 raw/8N1 配置，
      // 否则 ICANON/ISTRIP/IXON/OPOST 会整体损坏二进制帧（此前重连后 96%+ 帧
      // 判为无效的根因）。
      configure_port();
      queue_.clear();
      tools::logger()->info("[Gimbal] Reconnected serial successfully (attempt {}).", attempt);
      return;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
    }

    // 前 10 次 500ms 快速重试（可覆盖一次完整重枚举周期），之后 2s 一次避免刷日志
    auto delay = attempt < 10 ? std::chrono::milliseconds(500) : std::chrono::seconds(2);
    std::this_thread::sleep_for(delay);
  }
}

}  // namespace io