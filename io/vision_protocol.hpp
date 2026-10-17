#ifndef IO_VISION_PROTOCOL_HPP
#define IO_VISION_PROTOCOL_HPP

#include <cstdint>
#include <cstring>
#include <vector>

namespace io {

inline uint8_t calculateCRC8(const uint8_t* data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x80) crc = (crc << 1) ^ 0x31;
            else crc = crc << 1;
        }
    }
    return crc;
}

inline uint16_t calculateCRC16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else crc = crc >> 1;
        }
    }
    return crc;
}

inline void writeUint16LE(uint8_t* buf, uint16_t val) {
    buf[0] = static_cast<uint8_t>(val & 0xFF);
    buf[1] = static_cast<uint8_t>((val >> 8) & 0xFF);
}

inline uint16_t readUint16LE(const uint8_t* buf) {
    return static_cast<uint16_t>(buf[0]) | (static_cast<uint16_t>(buf[1]) << 8);
}

inline void writeFloatLE(uint8_t* buf, float val) {
    std::memcpy(buf, &val, 4);
}

inline float readFloatLE(const uint8_t* buf) {
    float val;
    std::memcpy(&val, buf, 4);
    return val;
}

constexpr uint8_t SOF = 0xA5;
constexpr uint16_t CMD_ID_VISION_TO_MCU = 0x0001;
constexpr uint16_t CMD_ID_MCU_TO_VISION = 0x0002;

inline std::vector<uint8_t> packVisionFrame(uint16_t flags, float pitch, float yaw) {
    constexpr size_t FRAME_LEN = 18;
    std::vector<uint8_t> frame(FRAME_LEN);
    frame[0] = SOF;
    writeUint16LE(&frame[1], 10);
    frame[3] = calculateCRC8(frame.data(), 3);
    writeUint16LE(&frame[4], CMD_ID_VISION_TO_MCU);
    writeUint16LE(&frame[6], flags);
    writeFloatLE(&frame[8], pitch);
    writeFloatLE(&frame[12], yaw);
    uint16_t crc16 = calculateCRC16(frame.data(), FRAME_LEN - 2);
    writeUint16LE(&frame[16], crc16);
    return frame;
}

struct MCUFrameData {
    uint16_t flags = 0;
    float yaw = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;
    bool valid = false;
};

inline MCUFrameData unpackMCUFrame(const uint8_t* frame, size_t len) {
    MCUFrameData result;
    if (len < 10) return result;
    if (frame[0] != SOF) return result;
    uint16_t data_length = readUint16LE(&frame[1]);
    size_t expected_len = data_length + 8;
    if (len != expected_len) return result;
    if (calculateCRC8(frame, 3) != frame[3]) return result;
    uint16_t crc16_calc = calculateCRC16(frame, len - 2);
    uint16_t crc16_recv = readUint16LE(&frame[len - 2]);
    if (crc16_calc != crc16_recv) return result;
    uint16_t cmd_id = readUint16LE(&frame[4]);
    if (cmd_id != CMD_ID_MCU_TO_VISION) return result;
    result.flags = readUint16LE(&frame[6]);
    result.yaw = readFloatLE(&frame[8]);
    result.pitch = readFloatLE(&frame[12]);
    result.roll = readFloatLE(&frame[16]);
    result.valid = true;
    return result;
}

}  // namespace io

#endif
