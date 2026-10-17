#ifndef IO_CBOARD1_HPP
#define IO_CBOARD1_HPP

#include <string>
#include <chrono>
#include <deque>
#include <utility>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "io/command.hpp"
#include "io/protocol.hpp"

namespace io {

// 与 Gimbal 兼容的模式枚举
enum class CBoard1Mode {
    IDLE,
    AUTO_AIM,
    SMALL_BUFF,
    BIG_BUFF
};

// 与 Gimbal 兼容的状态结构
struct CBoard1State {
    float yaw;
    float yaw_vel;
    float pitch;
    float pitch_vel;
    float bullet_speed;
    uint16_t bullet_count;
};

class CBoard1 {
public:
    explicit CBoard1(const std::string& serial_port);
    ~CBoard1();

    // 原有接口
    void send(const Command& command) const;
    void update();

    // 与 Gimbal 兼容的接口
    CBoard1Mode gimbal_mode() const;
    CBoard1State state() const;
    std::string str(CBoard1Mode mode) const;
    Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

    void send(
        bool control, bool fire, float yaw, float yaw_vel, float yaw_acc,
        float pitch, float pitch_vel, float pitch_acc);

    // 原有访问器
    Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point) const {
        return last_q_.normalized();
    }

    Mode mode = Mode::idle;
    float bullet_speed = 0.0f;

    bool getRobotColor() const { return last_received_.detect_color == 0; }
    bool getResetTracker() const { return last_received_.reset_tracker; }
    float getRoll() const { return last_received_.roll; }
    float getPitch() const { return last_received_.pitch; }
    float getYaw() const { return last_received_.yaw; }

    // CRC 正确率统计（自瞄串口链路质量）
    // 返回最近 crc_window_ 秒内的 CRC 通过率 (0~100)，窗口内无数据时返回 -1.0
    double crc_rate() const;
    uint64_t crc_total() const { return crc_total_; }  // 启动以来累计校验帧数
    uint64_t crc_good() const { return crc_good_; }    // 启动以来累计通过帧数

private:
    bool configureSerial();
    void parseBuffer();
    void recordCrc(bool ok);

    int serial_fd_;
    std::string serial_port_;
    Eigen::Quaterniond last_q_;
    ReceivePacket last_received_;
    std::vector<uint8_t> rx_buffer_;

    // 用于 state() 的额外字段
    mutable float yaw_vel_ = 0.0f;
    mutable float pitch_vel_ = 0.0f;
    mutable uint16_t bullet_count_ = 0;

    // CRC 统计（滑动窗口 + 累计）
    std::chrono::seconds crc_window_{5};
    std::deque<std::pair<std::chrono::steady_clock::time_point, bool>> crc_history_;
    uint64_t crc_total_ = 0;
    uint64_t crc_good_ = 0;
};

}  // namespace io

#endif
