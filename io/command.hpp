// 文件路径: io/command.hpp
#ifndef IO_COMMAND_HPP
#define IO_COMMAND_HPP

#include <string>
#include <vector>

namespace io {

// 模式枚举
enum class Mode {
    idle = 0,
    auto_aim = 1,
    auto_buff = 2,
    small_buff = 3,
    big_buff = 4,
};

// 模式名称（用于日志输出）
extern const std::vector<std::string> MODES;

// 视觉发给电控的命令结构体
struct Command {
    bool control = false;          // 是否开启云台控制
    bool shoot = false;            // 是否开火
    float yaw = 0.0f;              // 目标 yaw 角度 (rad)
    float pitch = 0.0f;            // 目标 pitch 角度 (rad)
    float horizon_distance = 0.0f; // 水平距离 (m)

    // 扩展字段用于填充 SendPacket
    float target_x = 0.0f;
    float target_y = 0.0f;
    float target_z = 0.0f;
    float target_vx = 0.0f;
    float target_vy = 0.0f;
    float target_vz = 0.0f;
    float target_yaw = 0.0f;
    float target_v_yaw = 0.0f;
    float r1 = 0.12f;
    float r2 = 0.12f;
    float dz = 0.0f;
    bool tracking = false;
    uint8_t id = 0;
    uint8_t armors_num = 4;
};

} // namespace io

#endif // IO_COMMAND_HPP
