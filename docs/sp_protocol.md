# SP 串口协议(视觉 ↔ 电控)

联调用协议版本:2025-10-07。**双方必须按本文档实现,并当面核对文末"待确认项"。**

代码定义见 `io/gimbal/gimbal.hpp`(视觉侧唯一权威来源)。

## 物理层与通用约定

| 项 | 约定 |
|---|---|
| 接口 | USB CDC 虚拟串口(设备名如 `/dev/ttyACM0`);若经 USB-UART 桥接,波特率统一 **115200** |
| 字节序 | 全部**小端**(x86 与 Cortex-M 均为小端,无需转换) |
| 对齐 | 结构体 1 字节 packed,无填充 |
| CRC | 查表 CRC16,init `0xFFFF`,反射多项式 `0x1021`(与 DJI 官方视觉协议 vision_crc 相同),**覆盖 crc16 字段之前的所有字节**;CRC 结果 16 位按小端存放(低字节在前) |
| 频率 | 视觉 → 电控:500Hz(2ms 周期,位置指令);电控 → 视觉:建议 1kHz |

CRC 参考实现(位运算版,与视觉侧查表版完全等价):

```c
uint16_t crc16_update(uint16_t crc, uint8_t byte) {
  crc ^= byte;
  for (int i = 0; i < 8; i++)
    crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : crc >> 1;
  return crc;
}

// 用法:对帧头到 crc 字段之前的所有字节依次 update,初值 0xFFFF
uint16_t crc = 0xFFFF;
for (i = 0; i < len_before_crc; i++) crc = crc16_update(crc, frame[i]);
```

**测试向量:`"123456789"` → `0x6F91`;`"SP"` → `0x5BA2`。** 联调前两边各算一遍对表。

## 电控 → 视觉(`GimbalToVision`,共 43 字节)

| 偏移 | 字段 | 类型 | 说明 |
|---|---|---|---|
| 0 | head[0] | uint8 | 帧头 `'S'` = 0x53 |
| 1 | head[1] | uint8 | 帧头 `'P'` = 0x50 |
| 2 | mode | uint8 | 0 空闲 / 1 自瞄 / 2 小符 / 3 大符 |
| 3 | q[0..3] | float×4 | 云台姿态四元数,**顺序 w x y z** |
| 19 | yaw | float | 云台当前 yaw(rad) |
| 23 | yaw_vel | float | yaw 角速度(rad/s) |
| 27 | pitch | float | 云台当前 pitch(rad) |
| 31 | pitch_vel | float | pitch 角速度(rad/s) |
| 35 | bullet_speed | float | **弹速(m/s),必须回传真实值**;无效时视觉会用配置默认弹速并告警 |
| 39 | bullet_count | uint16 | 累计发弹数 |
| 41 | crc16 | uint16 | 覆盖偏移 0~40 的所有字节 |

C 定义:

```c
#pragma pack(push, 1)
typedef struct {
  uint8_t  head[2];        // {'S','P'}
  uint8_t  mode;           // 0 空闲 1 自瞄 2 小符 3 大符
  float    q[4];           // w x y z
  float    yaw;            // rad
  float    yaw_vel;        // rad/s
  float    pitch;          // rad
  float    pitch_vel;      // rad/s
  float    bullet_speed;   // m/s
  uint16_t bullet_count;
  uint16_t crc16;
} GimbalToVision;          // sizeof == 43
#pragma pack(pop)
```

## 视觉 → 电控(`VisionToGimbal`,共 29 字节)

| 偏移 | 字段 | 类型 | 说明 |
|---|---|---|---|
| 0 | head[0] | uint8 | 帧头 `'S'` = 0x53 |
| 1 | head[1] | uint8 | 帧头 `'P'` = 0x50 |
| 2 | mode | uint8 | **0 不控制 / 1 控制云台但不开火 / 2 控制云台且开火** |
| 3 | yaw | float | 目标 yaw(rad) |
| 7 | yaw_vel | float | yaw 角速度前馈(rad/s) |
| 11 | yaw_acc | float | yaw 角加速度前馈(rad/s²) |
| 15 | pitch | float | 目标 pitch(rad) |
| 19 | pitch_vel | float | pitch 角速度前馈(rad/s) |
| 23 | pitch_acc | float | pitch 角加速度前馈(rad/s²) |
| 27 | crc16 | uint16 | 覆盖偏移 0~26 的所有字节 |

C 定义:

```c
#pragma pack(push, 1)
typedef struct {
  uint8_t  head[2];        // {'S','P'}
  uint8_t  mode;           // 0 不控制 1 控制不开火 2 控制且开火
  float    yaw;            // rad
  float    yaw_vel;        // rad/s
  float    yaw_acc;        // rad/s^2
  float    pitch;          // rad
  float    pitch_vel;      // rad/s
  float    pitch_acc;      // rad/s^2
  uint16_t crc16;
} VisionToGimbal;          // sizeof == 29
#pragma pack(pop)
```

## 行为约定(联调前当面确认)

1. **yaw/pitch 符号**:视觉按"世界系 yaw = atan2(y,x) + 偏置,pitch 上为负"发送。请双方约定并实测:放一个偏左、偏上的静态目标,看云台转向是否正确。确认后视觉侧只调 `yaw_offset`/`pitch_offset`(configs/standard3.yaml)。**完整坐标系与符号约定见 [coordinate_conventions.md](coordinate_conventions.md),发给电控按第 6 节静态对表。**
2. **mode 语义**:电控发 1(自瞄)后视觉才开始检测与控制;视觉发 0(不控制)时电控的响应(保持/回中/停控)由电控决定,但要保证**不会执行视觉停止前最后一帧的旧指令**。视觉在断流/丢帧时会自动改发 mode 0 帧。
3. **弹速**:电控必须回传真实 bullet_speed;帧内弹速无效(<14 m/s)时视觉改用 `default_bullet_speed`(configs/standard3.yaml,当前 23.0,**联调前改为实际弹速**)并在日志告警。
4. **CRC**:电控固件实现本协议 CRC 并回传后,视觉侧把 `crc_check: false` 改回 `true`(standard3.yaml)。此前视觉用单位四元数模长做替代校验。
5. **上电顺序**:先电控后视觉(视觉启动后 3s 内未收到首帧会打 warn 继续运行,收到首帧后自动进入正常流程)。
