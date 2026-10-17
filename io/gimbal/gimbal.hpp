#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>
#include <utility>

#include "io/command.hpp"
#include "serial/serial.h"
#include "tools/thread_safe_queue.hpp"

namespace io
{
struct __attribute__((packed)) GimbalToVision
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t mode;  // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
  float q[4];    // wxyz顺序
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;  // 子弹累计发送次数
  uint16_t crc16;
};

static_assert(sizeof(GimbalToVision) <= 64);

struct __attribute__((packed)) VisionToGimbal
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t mode;  // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  uint16_t crc16;
};

static_assert(sizeof(VisionToGimbal) <= 64);

enum class GimbalMode
{
  IDLE,        // 空闲
  AUTO_AIM,    // 自瞄
  SMALL_BUFF,  // 小符
  BIG_BUFF     // 大符
};

struct GimbalState
{
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;
};

class Gimbal
{
public:
  Gimbal(const std::string & config_path);

  ~Gimbal();

  GimbalMode mode() const;
  GimbalState state() const;
  std::string str(GimbalMode mode) const;
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

  void send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
    float pitch_acc);

  void send(io::VisionToGimbal VisionToGimbal);

  // 兼容 io::Command 的调用（CommandGener 按 send(Command) 调用）。
  // Command 只带位置量 yaw/pitch，故速度与加速度前馈置零。
  // 若需要速度/加速度前馈，请改用 Planner 产出的 Plan 并调用上面的 8 参数版本。
  void send(const Command & command);

  // CRC 正确率统计（自瞄串口链路质量）。
  // 返回最近 crc_window_ 秒内的 CRC 通过率 (0~100)；窗口内无数据或 crc_check 关闭时返回 -1.0。
  double crc_rate() const;
  uint64_t crc_total() const { return crc_total_.load(); }  // 启动以来累计校验帧数
  uint64_t crc_good() const { return crc_good_.load(); }    // 启动以来累计通过帧数
  bool crc_check_enabled() const { return crc_check_; }

private:
  serial::Serial serial_;

  std::thread thread_;
  std::atomic<bool> quit_ = false;
  mutable std::mutex mutex_;
  mutable std::mutex tx_mutex_;  // 保护 tx_data_：发送可能来自多个线程
  bool crc_check_ = true;        // 配置项 crc_check，默认开启

  // 串口读写超时：批量读时字节间 10ms 静默即返回（覆盖电控帧间隔 ~7ms），
  // 单次读调用最多等 100ms；写超时保持 100ms。
  serial::Timeout port_timeout_{10, 100, 0, 100, 0};
  // 写失败告警限频：掉线重连期间避免 500Hz 刷屏
  std::chrono::steady_clock::time_point last_tx_warn_{};

  GimbalToVision rx_data_;
  VisionToGimbal tx_data_;

  GimbalMode mode_ = GimbalMode::IDLE;
  GimbalState state_;
  tools::ThreadSafeQueue<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>>
    queue_{1000};
  // 最近一次有效姿态：队列为空（电控未上电/停发）时 q() 的兜底，避免主循环永久阻塞
  std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point> last_q_;

  // 每次 open() 后必须调用：重开/重枚举后的 tty 是内核默认 termios，二进制帧会被损坏
  void configure_port();
  // tx_data_ 填充完成后下发（含未打开/写失败处理）
  void write_frame();
  void read_thread();
  void reconnect();

  // CRC 统计（接收线程写、主线程读，各自加锁/原子）
  void record_crc(bool ok);
  std::chrono::seconds crc_window_{5};
  std::atomic<uint64_t> crc_total_{0};
  std::atomic<uint64_t> crc_good_{0};
  mutable std::mutex crc_mutex_;  // 保护 crc_history_
  std::deque<std::pair<std::chrono::steady_clock::time_point, bool>> crc_history_;
};

}  // namespace io

#endif  // IO__GIMBAL_HPP