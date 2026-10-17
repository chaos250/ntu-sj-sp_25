# 视觉 → 电控 数据与坐标符号约定

给电控固件的数值语义速查。代码权威来源：`io/gimbal/gimbal.hpp`(帧结构)、`tasks/auto_aim/aimer.cpp`(yaw/pitch 计算)、`tasks/auto_aim/multithread/commandgener.cpp`(发送逻辑)、`tasks/auto_aim/solver.cpp`(坐标变换)、`configs/standard3.yaml`(偏置配置)。

> 核心一句话：**视觉发给电控的 yaw/pitch 是"绝对角度"，参考系 = 你们发来的四元数 q 所定义的世界系**。云台直接按绝对角闭环，不要做相对增量。

## 1. 视觉 → 电控 发送数据总览(29B 帧)

| 字段 | 类型 | 单位 | 语义 |
|---|---|---|---|
| head[2] | uint8×2 | — | 帧头 `'S' 'P'` |
| mode | uint8 | — | **0 不控制 / 1 控制云台但不开火 / 2 控制云台且开火** |
| yaw | float | rad | 世界系目标绝对方位角,**左正右负**,见第 2 节 |
| yaw_vel | float | rad/s | yaw 角速度前馈(当前主程序恒为 0,见第 6 节) |
| yaw_acc | float | rad/s² | yaw 角加速度前馈(当前主程序恒为 0) |
| pitch | float | rad | 世界系目标绝对俯仰角,**上负下正(负 = 抬枪)**,见第 3 节 |
| pitch_vel | float | rad/s | pitch 角速度前馈(当前主程序恒为 0) |
| pitch_acc | float | rad/s² | pitch 角加速度前馈(当前主程序恒为 0) |
| crc16 | uint16 | — | CRC16(见 sp_protocol.md),当前配置 `crc_check: false` 时电控可先忽略 |

发送频率:**~500Hz**(commandgener.cpp 每 2ms 发一帧)。帧结构、字节序、CRC 细节见 [sp_protocol.md](sp_protocol.md)。

## 2. yaw:左右怎么分

- **定义**:`yaw = atan2(y_world, x_world) + yaw_offset`(aimer.cpp:141)
- **值域**:[-π, π](atan2 天然输出,**没有** wrap 到 0~2π;电控做差时请处理 ±π 跳变)
- **正负**(以 q 参考系 x 前、y 左、z 上、右手系为例):

| 目标位置 | yaw 值 | 云台应做的动作 |
|---|---|---|
| 正前方 | ≈ 0 | 不动 |
| 偏**左** | **> 0** | 向左转(yaw 增大) |
| 偏**右** | **< 0** | 向右转 |

- **语义**:绝对角,不是相对当前云台角的增量。

## 3. pitch:上下怎么分

- **定义**:`pitch = -(弹道俯仰角 + pitch_offset)`,其中弹道俯仰角 = atan2(z_world, √(x²+y²))(水平面以上为正)(aimer.cpp:142)
- **符号语义**:

| 目标位置 | pitch 值 | 云台应做的动作 |
|---|---|---|
| 高于水平面 | **负** | 抬枪 |
| 低于水平面 | **正** | 压枪 |

- 即:**负值 = 抬枪,正值 = 压枪**。范围:弹道解算范围内,实际 |pitch| 通常 < 45°(0.78 rad)。
- 电控若 pitch 电机编码器正方向与此相反(很多板子下压为正),在固件里取反即可,不要怀疑数据错。

## 4. 参考系与计算链

```
世界系(绝对系)  ←—— 由电控发来的 q 定义
   ↑ q (R_gimbal2world, 把云台系向量转到世界系)
云台系(gimbal)
   ↑ R_camera2gimbal + t_camera2gimbal(相机安装外参,visual 配置里标定好的固定值)
相机系(OpenCV 右手系:x 右、y 下、z 前)
```

目标位置的计算链(代码顺序):

```
PnP 解算(相机系 xyz)
  → xyz_in_gimbal = R_camera2gimbal · xyz + t_camera2gimbal
  → xyz_in_world  = q · xyz_in_gimbal
  → yaw  = atan2(y_world, x_world) + yaw_offset
  → pitch = -(弹道俯仰角 + pitch_offset)
```

**世界系 x/y/z 轴指向由电控的 q 参考系决定**:q = (1,0,0,0)(单位四元数)时,世界系 = 云台零位系。如果你们 IMU 绝对系的 x 轴不是"正前方",差一个常值角,由视觉侧 `yaw_offset` 补偿(见第 7 节)。当前配置里 `R_gimbal2imubody` 为单位阵,即视觉认为"云台系 = IMU 体系";若实际安装有夹角,推荐电控直接在固件里把 q 转成"云台体→世界",或与视觉约定填 `R_gimbal2imubody`。

## 5. mode 语义与保护行为

| mode | 含义 | 触发条件(代码) |
|---|---|---|
| 0 | 不控制 | 停控帧:视觉未收到电控自瞄指令、目标丢失 > 0.2s、或 yaw/pitch 非有限值(NaN/Inf 防护,宁发停控帧不发电控无法执行的垃圾值) |
| 1 | 控制云台但不开火 | 有目标并跟踪,但 shooter 判定不满足开火条件 |
| 2 | 控制云台且开火 | 有目标 + `auto_fire: true` + 上一帧与当前 yaw 突变 < 2×tolerance + 云台当前 yaw 已跟上目标(误差 < tolerance,按目标距离选 first/second_tolerance) |

约定:电控收到 mode 0 后的响应(保持/回中/停控)由电控决定,但要保证**不会执行视觉停控前最后一帧的旧指令**。视觉在断流/丢帧时会自动改发 mode 0 帧。

## 6. 前馈字段(yaw_vel / yaw_acc / pitch_vel / pitch_acc)

- **当前正式程序 `mt_standard`(watchdog 默认)**:指令走 `io::Command` 路径,只带位置量,**这四个字段恒为 0**,电控忽略即可(gimbal.cpp:131-135)。
- **实验程序 `standard_mpc`(MPC 规划)**:由 Planner 输出非零 MPC 前馈(planner.cpp:88-93)。若将来正式启用前馈,其符号与 yaw/pitch 一致(绝对角速度/角加速度),单位 rad/s、rad/s²。

## 7. 偏置与配置(configs/standard3.yaml)

| 配置项 | 当前值 | 单位 | 用途 |
|---|---|---|---|
| `yaw_offset` | 2 | 度 | 整体 yaw 修正(装配误差、相机-云台安装角、世界系 x 轴不一致) |
| `pitch_offset` | 6.5 | 度 | 整体 pitch 修正(同上) |
| `default_bullet_speed` | 23.0 | m/s | 电控帧内弹速无效(<14 m/s)时的兜底弹速,**联调前改为实际弹速** |
| `left_yaw_offset` / `right_yaw_offset` | 未定义 | 度 | 可选,**双枪位**(哨兵)左右开火时的额外 yaw 偏移;步兵单枪不用(aimer.cpp:33-38) |

改完杀掉视觉程序(watchdog 自动拉起)即生效。**注意:offset 只能修"常值偏差",修不了"方向反"**。若实测云台转向和预期镜像相反,那是 q 参考系/电机正方向的问题,要改电控侧或 `R_gimbal2imubody`,不是调 offset。

## 8. 依赖的电控回传数据(43B 帧)

| 字段 | 约定 | 用途 |
|---|---|---|
| mode | 0 空闲 / 1 自瞄 / 2 小符 / 3 大符 | 电控发 1 后视觉才开始检测与控制 |
| q[4] | **顺序 w x y z**,模长必须 = 1(视觉校验,无效则沿用旧姿态) | 定义世界系,视觉按帧时间戳外推到检测时刻 |
| yaw / yaw_vel / pitch / pitch_vel | 云台当前绝对 yaw/pitch 及角速度,符号应与 q 参考系一致 | 监控与校验(请填真实值,勿留 0/垃圾) |
| bullet_speed | **弹速 m/s,必须回传真实值** | 弹道解算;帧内弹速 <14 m/s 时视觉改用 `default_bullet_speed` 并告警 |
| bullet_count | uint16 累计发弹数 | 统计 |

> **联调现状(2026-10-10 晚,视觉侧抓包)**:q 已修好(单位四元数 ✓),但帧尾部仍有三类问题,详见文末第 11 节。

## 9. 静态对表步骤(联调时当面做,一次说清)

前提:云台放零位,电控发 q(哪怕发单位四元数也可以开始对)。

| 操作 | 视觉发送预期 | 云台应做的动作 |
|---|---|---|
| 靶放**正前方** | yaw ≈ 0 | 不动 |
| 靶放**偏左** | yaw > 0 | 云台向左转(yaw 增大) |
| 靶放**偏右** | yaw < 0 | 云台向右转 |
| 靶**抬高** | pitch < 0 | 抬枪 |
| 靶**放低** | pitch > 0 | 压枪 |

- 左/右反了 → 电控检查 q 的参考系与电机正方向;
- 差固定角度 → 视觉侧调 `yaw_offset` / `pitch_offset`;
- 数值对、动作对 → 通过。

## 10. 快速速查表

| 数据 | 方向 | 类型 | 单位 | 语义 |
|---|---|---|---|---|
| q (w,x,y,z) | 电控→视觉 | float×4 | — | 云台→世界旋转,模长 1,顺序 wxyz |
| mode | 视觉→电控 | uint8 | — | 0 不控制 / 1 控制不开火 / 2 控制且开火 |
| yaw | 视觉→电控 | float | rad | 世界系绝对方位角,**左正右负**,[-π,π] |
| pitch | 视觉→电控 | float | rad | 世界系绝对俯仰角,**上负下正(负 = 抬枪)** |
| yaw_vel / yaw_acc / pitch_vel / pitch_acc | 视觉→电控 | float | rad/s, rad/s² | 前馈预留;当前正式程序恒为 0,忽略即可 |
| bullet_speed | 电控→视觉 | float | m/s | 必须回传真实弹速,<14 时视觉用配置默认值 |
| 帧率 | 视觉→电控 | — | — | ~500Hz(2ms 周期,位置指令) |

## 11. 联调现状:视觉侧抓到的电控帧问题(2026-10-10 晚)

视觉侧已上线 1Hz 接收解码日志(`[GimbalRx]` + `[GimbalRxRaw]`,含 43 字节原始十六进制转储),双方可实时对表。抓包结论(最近 100 帧统计):

| 类别 | 占比 | 现象 |
|---|---|---|
| 完全正确 | 27% | q 单位、bullet_speed=23.0、count=0、CRC 校验通过 |
| q 有效但尾部错位 | 33% | bullet_speed/bullet_count/crc 整体比协议位置**前移 1 字节**(视觉读到 bullet≈0、count 乱跳、CRC 失败) |
| 整帧乱码 | 40% | q 为天文数字(如 y=-1.5e6、z=-1.1e17),|q|² 达 1e60 级别 |

同内容的帧(23.0 的浮点字节 `00 00 B8 41`)在正确帧里位于偏移 35,在错位帧里位于偏移 34——**内容对、位置漂**,疑似电控从 CAN 报文 memcpy 组装帧时偏移错了 1 字节,或存在两个不同版本的结构体/两个发送任务共用一个串口。

电控侧排查清单:

1. 打印 `sizeof(帧结构体)`,必须 == 43;用 `offsetof` 逐字段核对:head@0、mode@2、q@3、yaw@19、yaw_vel@23、pitch@27、pitch_vel@31、bullet_speed@35、bullet_count@39、crc16@41(见 sp_protocol.md 第 2 节)。
2. CRC 用测试向量 `"SP"` → `0x5BA2` 验证(覆盖 crc 字段之前所有字节,小端存放)。
3. 确认串口上**只有一个任务**在发帧;若帧由 CAN 报文组装,核对 memcpy 源偏移(帧尾错位 1 字节的典型原因)。
4. 发送前 `memset` 清空缓冲,勿发送未初始化内存(整帧乱码帧的来源)。
5. yaw/pitch/yaw_vel/pitch_vel 需回传云台真实姿态(若云台静止停零位,当前 ≈0 属正常)。
