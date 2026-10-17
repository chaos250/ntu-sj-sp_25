#include "cboard1.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <cerrno>
#include <system_error>
#include <algorithm>
#include <Eigen/Geometry>

#include "tools/logger.hpp"

namespace io {

CBoard1::CBoard1(const std::string& serial_port)
    : serial_port_(serial_port),
      last_q_(Eigen::Quaterniond::Identity()),
      rx_buffer_(512) {

    serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd_ < 0) {
        throw std::system_error(errno, std::generic_category(),
                                "Failed to open serial port: " + serial_port_);
    }

    if (!configureSerial()) {
        close(serial_fd_);
        throw std::runtime_error("Failed to configure serial port: " + serial_port_);
    }

    tools::logger()->info("[CBoard1] Serial port opened: {}", serial_port_);
}

CBoard1::~CBoard1() {
    if (serial_fd_ >= 0) {
        close(serial_fd_);
        tools::logger()->info("[CBoard1] Serial port closed.");
    }
}

bool CBoard1::configureSerial() {
    struct termios options;
    if (tcgetattr(serial_fd_, &options) != 0) return false;

    // 电控端 USART1 波特率 115200
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CRTSCTS;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_iflag &= ~(INLCR | ICRNL | IGNCR);
    options.c_oflag &= ~OPOST;

    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    if (tcsetattr(serial_fd_, TCSANOW, &options) != 0) return false;

    tcflush(serial_fd_, TCIOFLUSH);
    return true;
}

void CBoard1::send(const Command& command) const {
    SendPacket packet;

    packet.tracking = command.tracking;
    packet.id = command.id;
    packet.armors_num = command.armors_num;
    packet.reserved = 0;
    packet.x = command.target_x;
    packet.y = command.target_y;
    packet.z = command.target_z;
    packet.yaw = command.target_yaw;
    packet.vx = command.target_vx;
    packet.vy = command.target_vy;
    packet.vz = command.target_vz;
    packet.v_yaw = command.target_v_yaw;
    packet.r1 = command.r1;
    packet.r2 = command.r2;
    packet.dz = command.dz;

    uint16_t crc = calculateCRC16(reinterpret_cast<uint8_t*>(&packet),
                                  sizeof(packet) - sizeof(packet.checksum));
    packet.checksum = crc;

    ssize_t written = write(serial_fd_, &packet, sizeof(packet));
    if (written != static_cast<ssize_t>(sizeof(packet))) {
        tools::logger()->warn("[CBoard1] Send failed: {} of {}", written, sizeof(packet));
    } else {
        tools::logger()->info("[CBoard1] Sent {} bytes, tracking={} yaw={:.3f} pitch={:.3f}",
                      written, static_cast<int>(packet.tracking), command.yaw, command.pitch);
    }
}

void CBoard1::update() {
    uint8_t buf[256];
    ssize_t n = read(serial_fd_, buf, sizeof(buf));
    if (n > 0) {
        rx_buffer_.insert(rx_buffer_.end(), buf, buf + n);
    }
    parseBuffer();
}

void CBoard1::parseBuffer() {
    constexpr size_t FRAME_SIZE = sizeof(ReceivePacket);

    while (rx_buffer_.size() >= FRAME_SIZE) {
        // 1. 查找 header 0x5A
        auto it = std::find(rx_buffer_.begin(), rx_buffer_.end(), 0x5A);
        if (it == rx_buffer_.end()) {
            rx_buffer_.clear();
            return;
        }
        if (it != rx_buffer_.begin()) {
            rx_buffer_.erase(rx_buffer_.begin(), it);
        }
        if (rx_buffer_.size() < FRAME_SIZE) return;

        // 2. 拷贝一帧
        ReceivePacket packet;
        std::memcpy(&packet, rx_buffer_.data(), FRAME_SIZE);

        // 3. 验证 CRC16
        uint16_t calc_crc = calculateCRC16(rx_buffer_.data(), FRAME_SIZE - 2);
        if (calc_crc != packet.checksum) {
            recordCrc(false);
            rx_buffer_.erase(rx_buffer_.begin());
            continue;
        }
        recordCrc(true);

        // 4. 解析成功
        last_received_ = packet;
        mode = Mode::auto_aim;  // 临时：强制自瞄模式（联调用，正式比赛需删掉）

        // 欧拉角转四元数 (ZYX)
        Eigen::AngleAxisd yaw_a(packet.yaw, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd pitch_a(packet.pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd roll_a(packet.roll, Eigen::Vector3d::UnitX());
        last_q_ = (yaw_a * pitch_a * roll_a).normalized();

        // 限频日志（1秒一次）
        static auto last_log = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(now - last_log).count() >= 1.0) {
            tools::logger()->info("[CBoard1] yaw={:.3f} pitch={:.3f} roll={:.3f} color={}",
                                  static_cast<float>(packet.yaw),
                                  static_cast<float>(packet.pitch),
                                  static_cast<float>(packet.roll),
                                  packet.detect_color ? "blue" : "red");
            last_log = now;
        }

        // 5. 移除已处理帧
        rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + FRAME_SIZE);
    }

    if (rx_buffer_.size() > 1024) {
        rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + 512);
    }
}

void CBoard1::recordCrc(bool ok) {
    auto now = std::chrono::steady_clock::now();
    ++crc_total_;
    if (ok) ++crc_good_;
    crc_history_.emplace_back(now, ok);
    // 只保留最近 crc_window_ 秒内的记录
    while (!crc_history_.empty() &&
           now - crc_history_.front().first > crc_window_) {
        crc_history_.pop_front();
    }
}

double CBoard1::crc_rate() const {
    if (crc_history_.empty()) return -1.0;
    uint64_t good = 0;
    for (const auto& [t, ok] : crc_history_) {
        (void)t;
        if (ok) ++good;
    }
    return 100.0 * static_cast<double>(good) / static_cast<double>(crc_history_.size());
}
// ========== 与 Gimbal 兼容的接口实现 ==========

CBoard1Mode CBoard1::gimbal_mode() const {
    switch (mode) {
        case Mode::auto_aim: return CBoard1Mode::AUTO_AIM;
        case Mode::auto_buff: return CBoard1Mode::SMALL_BUFF;
        default: return CBoard1Mode::IDLE;
    }
}

CBoard1State CBoard1::state() const {
    CBoard1State s;
    s.yaw = last_received_.yaw;
    s.yaw_vel = yaw_vel_;
    s.pitch = last_received_.pitch;
    s.pitch_vel = pitch_vel_;
    s.bullet_speed = bullet_speed;
    s.bullet_count = bullet_count_;
    return s;
}

std::string CBoard1::str(CBoard1Mode mode) const {
    switch (mode) {
        case CBoard1Mode::IDLE: return "IDLE";
        case CBoard1Mode::AUTO_AIM: return "AUTO_AIM";
        case CBoard1Mode::SMALL_BUFF: return "SMALL_BUFF";
        case CBoard1Mode::BIG_BUFF: return "BIG_BUFF";
        default: return "UNKNOWN";
    }
}

Eigen::Quaterniond CBoard1::q(std::chrono::steady_clock::time_point t) {
    return imu_at(t);
}

void CBoard1::send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc,
    float pitch, float pitch_vel, float pitch_acc) {

    yaw_vel_ = yaw_vel;
    pitch_vel_ = pitch_vel;

    Command command;
    command.control = control;
    command.shoot = fire;
    command.yaw = yaw;
    command.pitch = pitch;
    command.tracking = control;

    send(command);
}


}  // namespace io
