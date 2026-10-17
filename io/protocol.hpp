#ifndef PROTOCOL_HPP
#define PROTOCOL_HPP

#include <cstdint>
#include <cstddef>

// ========== 电控 → 视觉 (28 字节) ==========
// header(1) + flags(1) + roll(4) + pitch(4) + yaw(4) + aim_x(4) + aim_y(4) + aim_z(4) + CRC16(2)
struct ReceivePacket {
    uint8_t header = 0x5A;
    uint8_t detect_color : 1;   // 0=red, 1=blue
    bool reset_tracker : 1;
    uint8_t reserved : 6;
    float roll;
    float pitch;
    float yaw;
    float aim_x;
    float aim_y;
    float aim_z;
    uint16_t checksum = 0;
} __attribute__((packed));

// ========== 视觉 → 电控 (48 字节) ==========
// header(1) + flags(1) + x(4) + y(4) + z(4) + yaw(4)
// + vx(4) + vy(4) + vz(4) + v_yaw(4) + r1(4) + r2(4) + dz(4) + CRC16(2)
struct SendPacket {
    uint8_t header = 0xA5;
    uint8_t tracking : 1;
    uint8_t id : 3;
    uint8_t armors_num : 3;
    uint8_t reserved : 1;
    float x;
    float y;
    float z;
    float yaw;
    float vx;
    float vy;
    float vz;
    float v_yaw;
    float r1;
    float r2;
    float dz;
    uint16_t checksum = 0;
} __attribute__((packed));

// CRC16 (右移法, poly=0xA001)
uint16_t calculateCRC16(const uint8_t* data, size_t len);

#endif
