# 自瞄追踪实现与参数摘要

本文档总结了仓库中自瞄（auto_aim）追踪模块的实现要点、关键文件位置与常用调参项，便于快速理解与调试。

**实现流程概述**
- 检测（Detector）：从图像检测灯条/装甲板，输出 `Armor` 结构（4 点关键点、置信度等）。
- 求解（Solver）：对检测到的装甲板调用 `solvePnP`，得到三维位置/朝向，并做 yaw 优化。
- 跟踪（Tracker / Target）：
  - `Tracker` 负责目标选择、状态机（lost/detecting/tracking/temp_lost/switching）与调用 `Target` 的预测/更新。
  - `Target` 使用扩展卡尔曼滤波（EKF）估计目标（旋转中心、角速度、半径等），提供预测与装甲板匹配逻辑。
- 瞄准（Aimer）：基于 EKF 预测的目标位置选择瞄准点，迭代求解弹道飞行时间，输出 yaw/pitch 指令。
- 开火判决（Shooter）：根据角度误差与距离阈值判断是否触发开火（有防抖逻辑）。
- 下发电控：通过 CAN（CBoard/SocketCAN）或串口（Gimbal 协议）发送控制/状态帧。

**关键源码文件（快速跳转）**
- 检测/结构：[tasks/auto_aim/detector.cpp](tasks/auto_aim/detector.cpp), [tasks/auto_aim/armor.hpp](tasks/auto_aim/armor.hpp)
- 求解 / 重投影：[tasks/auto_aim/solver.cpp](tasks/auto_aim/solver.cpp), [tasks/auto_aim/solver.hpp](tasks/auto_aim/solver.hpp)
- 跟踪 / EKF：[tasks/auto_aim/tracker.cpp](tasks/auto_aim/tracker.cpp), [tasks/auto_aim/target.cpp](tasks/auto_aim/target.cpp)
- 瞄准 / 弹道：[tasks/auto_aim/aimer.cpp](tasks/auto_aim/aimer.cpp), [tools/trajectory.hpp](tools/trajectory.hpp)
- 开火判决：[tasks/auto_aim/shooter.cpp](tasks/auto_aim/shooter.cpp)
- 通信（CAN/串口）：[io/cboard.cpp](io/cboard.cpp), [io/socketcan.hpp](io/socketcan.hpp), [io/gimbal/gimbal.cpp](io/gimbal/gimbal.cpp)
- 配置示例：[configs/example.yaml](configs/example.yaml)

**主要配置项与含义（位于 configs/*.yaml）**
- Tracker / 跟踪
  - `min_detect_count`：连续检测到同一目标达到此帧数后进入 `tracking`（默认示例：5）。
  - `max_temp_lost_count`：临时丢失允许最大帧数（示例：15）。
- Aimer / 瞄准
  - `yaw_offset`：整体 yaw 偏置（度，代码中会转换为弧度）。
  - `pitch_offset`：整体 pitch 偏置（度）。
  - `comming_angle` / `leaving_angle`：用于旋转（小陀螺）时判断装甲板进入/离开射击范围（度）。
  - `high_speed_delay_time` / `low_speed_delay_time` / `decision_speed`：角速度阈值与对应的预测延时，用于决定预测 horizon。
- Shooter / 开火
  - `first_tolerance` / `second_tolerance`：近/远射击角度容差（度）。
  - `judge_distance`：近/远判定距离（米）。
  - `auto_fire`：是否启用自动开火（布尔）。
- 通信
  - `quaternion_canid`、`bullet_speed_canid`、`send_canid`：CAN ID 配置（见 `CBoard::read_yaml`）。
  - `can_interface`：CAN 设备名称（如 `can0`）。
  - `com_port`：云台串口设备（如 `/dev/gimbal`）。
- 标定/相机
  - `camera_matrix`、`distort_coeffs`、`R_camera2gimbal`、`t_camera2gimbal`：相机与云台/IMU 的外参与内参（影响 solvePnP 与重投影）。

**实现细节要点（便于调试）**
- 目标初始化：`Tracker::set_target` 根据装甲类型给 EKF 初始协方差 P0 赋不同值，控制收敛速度与鲁棒性。
- 预测噪声：`Target::predict` 中根据目标类型（outpost vs 普通）设置过程噪声 v1/v2，影响滤波器对加速度与角加速度的容忍度。
- 观测噪声：`Target::update_ypda` 动态计算测量噪声矩阵 R（根据角度差与距离调整），能提高在远/近与角度异常时的鲁棒性。
- 瞄准迭代：`Aimer::aim` 通过迭代预测目标在“未来 (预测延迟 + fly_time)”时刻的位置并重算弹道，直到飞行时间收敛。
- 弹道不可解判断：`tools::Trajectory` 提供 `unsolvable` 标志（当给定速度/距离/高度无法解出合适抬角时触发），此时放弃该次瞄准。

**调参建议（快速起步）**
1. 使用 `configs/example.yaml` 作为基线配置，先保证检测稳定性（`min_confidence`, ROI 等）。
2. 调整 `min_detect_count` 和 `max_temp_lost_count` 以适应相机帧率与检测抖动。
3. 调整 `yaw_offset` / `pitch_offset` 来补偿系统系统偏差（小步调整，单位为度）。
4. 若目标旋转（小陀螺）不稳定，适当缩小 `comming_angle` / 增大 `leaving_angle` 以避免频繁切换。
5. 若弹道频繁不可解，检查 `bullet_speed` 来源与 `tools/trajectory` 的参数；可临时提高子弹速度默认值以验证流程。

**下一步建议**
- 若需要，我可以把上述参数生成一份可编辑的 `docs/auto_aim_params.md` 表格，便于直接填入比赛机器；或把调参步骤写成一份短的调试清单。

---
文档生成于仓库 `docs/auto_aim_summary.md`。
