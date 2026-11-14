# sp_vision_25 代码框架逐模块分析

> 同济大学 SuperPower 战队 25 赛季视觉算法开源项目(自瞄 / 能量机关 / 哨兵)。
> 本文档对项目代码框架按「目录 → 模块 → 文件 → 类/函数 → 代码段」逐层说明**作用**、**逐段实现**、**优点**、**缺点/隐患**。
> 厂商 SDK 头文件(海康 `io/hikrobot/include`、迈德威视 `io/mindvision/include`、串口 `io/serial/include`)与捆绑的第三方库(Ceres、abseil-cpp、tinympc 内部实现)不做逐行展开,仅在相关小节说明其作用。

---

# 目录

1. [项目概述](#一项目概述)
2. [目录结构总览](#二目录结构总览)
3. [顶层构建系统 CMakeLists.txt](#三顶层构建系统-cmakeliststxt)
4. [启动与守护脚本](#四启动与守护脚本)
5. [tools 基础工具库](#五tools-基础工具库)
6. [io 设备/通信层](#六io-设备通信层)
7. [tasks/auto_aim 自动瞄准](#七tasksauto_aim-自动瞄准)
8. [tasks/auto_buff 能量机关](#八tasksauto_buff-能量机关)
9. [tasks/omniperception 哨兵感知](#九tasksomniperception-哨兵感知)
10. [src 主程序入口](#十src-主程序入口)
11. [calibration 标定工具](#十一calibration-标定工具)
12. [tests 测试程序](#十二tests-测试程序)
13. [configs 配置体系](#十三configs-配置体系)

---

## 一、项目概述

这是一个用于 RoboMaster 对抗赛的**视觉 + 火控**软件,运行在小电脑(如 NUC)上,通过相机采集图像,识别敌方装甲板/能量机关扇叶,进行目标跟踪、位姿解算、弹道补偿与开火决策,最后通过串口把控制指令下发给下位机(STM32)执行云台与发射机构动作。

核心算法链(自瞄):
```
相机取流 → 装甲板检测(OpenVINO YOLO 四点模型) → 数字分类 → 目标跟踪(EKF) → 
PnP 位姿解算 → 坐标变换/弹道解算 → 轨迹规划/开火决策 → 串口下发云台指令
```

技术栈:C++17、CMake、OpenCV、OpenVINO(神经网络推理)、Eigen(矩阵/姿态)、Ceres(最小二乘)、yaml-cpp(配置)、spdlog(日志)、fmt(格式化)、nlohmann_json、ROS2(仅哨兵导航接口,可选)。

---

## 二、目录结构总览

| 目录 | 职责 |
|------|------|
| `tools/` | 无外部依赖的基础工具库:数学/坐标变换、PID、扩展卡尔曼、RANSAC 正弦拟合、绘图、日志、录像、CRC、线程池、线程安全队列、YAML 读取 |
| `io/` | 设备与通信抽象:相机(海康/迈德威视/USB)、云台、IMU、串口协议、CAN、ROS2 桥接 |
| `tasks/auto_aim/` | 自瞄算法:检测、分类、跟踪、解算、瞄准、发射、投票、轨迹规划(MPC) |
| `tasks/auto_buff/` | 能量机关(大风车)击打算法 |
| `tasks/omniperception/` | 哨兵全向感知与决策 |
| `src/` | 各机器人对应的主程序 main 入口(步兵 standard、哨兵 sentry、无人机 uav) |
| `calibration/` | 相机内参标定、手眼标定、视频切帧等工具 |
| `tests/` | 各模块独立测试程序 |
| `configs/` | YAML 参数配置 |
| `scripts/` | 依赖安装、云台 udev 规则、云台监控等运维脚本 |
| `docs/` | 部署文档、坐标系约定、协议说明 |

依赖关系(自底向上):`tools` ← `io` ← `tasks/*` ← `src/*`(编译目标)。

---

## 三、顶层构建系统 CMakeLists.txt

文件:`CMakeLists.txt`(152 行)。下面逐段说明。

### 3.1 基础设置(`:1-16`)
```cmake
cmake_minimum_required(VERSION 3.16.3)
set(OpenVINO_DIR "/opt/intel/openvino_2025.4.0/runtime/cmake" CACHE PATH "Path to OpenVINO")
project(sp_vision)
find_program(CCACHE_PROGRAM ccache)
...
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
```
- **作用**:声明最低 CMake 版本、项目名、C++ 标准与构建类型,并可选启用 ccache 编译缓存。
- **逐段说明**:`find_program` 检测 ccache,存在才设置 `CMAKE_CXX_COMPILER_LAUNCHER`,避免未安装时 configure 直接失败。
- **优点**:ccache 检测做了容错;显式指定 C++17/Release。
- **缺点**:`CMAKE_BUILD_TYPE` 被硬编码为 Release,用户无法通过 `-DCMAKE_BUILD_TYPE=Debug` 覆盖(单值 set 可被命令行覆盖,但这里写死为默认,调试时需手动改)。第 4 行的 `OpenVINO_DIR` 设置是冗余的(后面 `:25-31` 又重新解析),且硬编码了绝对路径。

### 3.2 依赖查找(`:18-36`)
```cmake
find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
...
find_package(OpenVINO REQUIRED)
include_directories(...)
```
- **作用**:查找并引入所有第三方依赖。
- **逐段说明**:`25-31` 优先找 2025.4.0 的 OpenVINO,不存在则回退 2024.6.0,都没有则告警。`include_directories` 全局加入 Eigen/OpenCV/项目根目录头文件路径。
- **优点**:OpenVINO 双版本回退逻辑贴合"开发机 vs 小电脑"实际部署需求。
- **缺点**:`include_directories` 是全局、目录级(而非 target 级)的,污染所有目标的包含路径,现代 CMake 更推荐 `target_include_directories`。

### 3.3 子目录与库(`:39-43`)
```cmake
add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(tasks/auto_aim)
add_subdirectory(tasks/auto_buff)
add_subdirectory(tasks/omniperception)
```
- **作用**:按分层顺序编译各子模块。tools/io/tasks 用 `OBJECT`/`STATIC` 库,再被可执行文件链接。

### 3.4 src 可执行文件(`:45-66`)
- **作用**:声明各机器人主程序(`standard`、`mt_standard`、`standard_mpc`、`auto_buff_debug` 等),并链接对应库。
- **逐段说明**:大量被注释掉的目标(`mt_auto_aim_debug`、`uav`、`uav_debug`)说明这些主程序暂不参与编译;`auto_buff_debug*` 额外链接 `${CERES_LIBRARIES}`。
- **缺点**:`${CERES_LIBRARIES}` 这个变量在现代 Ceres 里通常为空(应使用 `Ceres::ceres` 目标),这里直接拼接可能导致链接失败或依赖传递不清晰;被注释代码散落,增加了维护噪音。

### 3.5 calibration 与 tests(`:68-123`)
- **作用**:声明标定工具和测试程序并链接依赖。
- **缺点**:每个测试/标定程序逐条 `target_link_libraries` 高度重复,可提炼成 helper 函数或 `INTERFACE` 库;链接项手写 `fmt::fmt yaml-cpp ...` 易漏(例如部分目标链接了 `yaml-cpp` 部分没有)。

### 3.6 ROS2 条件编译(`:125-151`)
- **作用**:用 `find_package(... QUIET)` 检测 ROS2 环境,存在才编译 `sentry*`、`publish_test` 等依赖 ROS 的目标。
- **优点**:无 ROS 环境时能正常编译其余代码,契合"无 ROS 依赖即可上手"的设计目标。
- **缺点**:`sentry` 相关可执行文件在无 ROS 时被整体跳过,若只改了这些文件可能长期不编译;ROS 包名 `sp_msgs` 依赖战队自定义消息包,外部用户需要额外获取。

---

## 四、启动与守护脚本

### 4.1 autostart.sh(23 行)
- **作用**:开机自启入口,由 `~/.config/autostart/sp_vision.desktop` 在登录后调用,在独立 `screen` 会话里启动 `watchdog.sh`。
- **逐段说明**:`:10` 用 `dirname "${BASH_SOURCE[0]}"` 定位脚本自身所在目录作为仓库根;`:12` `sleep 5` 等待相机/串口设备枚举完成;`:14-15` 进入根目录并建 `logs/`;`:17-22` 用 `screen -L -Logfile ... -dm bash watchdog.sh` 后台启动守护进程,并把屏幕日志落盘到带时间戳的文件。
- **优点**:`SCRIPT_DIR` 自定位使脚本可被拷到任意路径;`screen -L` 让崩溃现场可 `screen -r` 回溯。
- **缺点**:`sleep 5` 是魔法等待,设备枚举慢时会不够;`screen` 是额外运行时依赖。

### 4.2 watchdog.sh(51 行)
- **作用**:守护视觉程序,崩溃/退出后自动重启,并带指数退避避免设备未就绪时刷屏。
- **逐段说明**:
  - `:8` `set -u` 未定义变量报错,更健壮。
  - `:13-15` 三个环境变量 `VISION_BIN`/`VISION_CONFIG`/`RESTART_DELAY` 可覆盖默认值(`standard_mpc` + `standard3.yaml`)。
  - `:21` `fail_streak` 记录连续快速崩溃次数。
  - `:23-50` 主循环:先检查二进制存在/可执行(`:24`),不存在则 10s 后重试;否则记录启动时间、运行程序、拿到退出码 `:30-34`;运行超过 60s 视为健康清零退避(`:36-40`);`:43-46` 退避间隔 = `RESTART_DELAY * 2^n`,上限 60s;最后 sleep 后再循环。
- **优点**:指数退避 + 健康判定设计合理,既保证崩溃自愈又避免崩溃风暴;`tee -a` 同时输出到终端和日志。
- **缺点**:`run_secs` 用 `date +%s`(秒级)计算,程序在 60s 边界附近判定可能抖动;没有区分"正常退出"与"崩溃",退出码未细分处理。

### 4.3 .gitignore(14 行)
- **作用**:排除 `build/`、`logs/`、`records*/`、`imgs/`、`MvSdkLog/`、测试视频等生成物。
- **缺点**:第 1 行 `.vscode/` 被忽略,但仓库实际提交了 `.vscode/`(见 git 状态),说明该规则与仓库内容存在矛盾;`Camera/`、`testvideos/` 等路径为历史遗留。

---

## 五、tools 基础工具库

`tools/CMakeLists.txt` 将下列 13 个源文件编译为 `OBJECT` 库(仅编译、不归档,最终由各可执行文件直接链接目标文件),实现代码复用。

### 5.1 math_tools.hpp / math_tools.cpp(58 / 201 行)

**作用**:坐标系/角度/时间的数学工具箱,是整个项目坐标变换的基础。

#### 函数清单与逐段说明

- **`limit_rad(double angle)`**(`:8-13`)
  - 作用:把角度归一化到 `(-π, π]`。
  - 实现:两个 `while` 循环加减 `2π`。
  - 缺点:用 `while` 而非 `fmod`,输入是超大角度时循环次数多;边界 `-π` 用 `<=` 处理使其可能返回 `π`。

- **`eulers(Quaterniond q, int axis0, axis1, axis2, bool extrinsic)`**(`:15-74`)
  - 作用:任意轴序的四元数→欧拉角(支持内旋/外旋)。
  - 实现:`:17` 内旋时交换 axis0/axis2;`:20-22` 判断是否"proper"欧拉角(`i==k`)并计算符号;`:24-36` 从四元数系数构造中间变量;`:39-62` 用 `acos/atan2` 反解中间角,并处理奇点(gimbal lock)分支 `:52-61`;`:64` 归一化;`:66-69` 非 proper 时修正;`:71` 内旋时再交换回。
  - 优点:通用性强,可指定任意旋转轴序,参考文献实现(quaternion_to_euler)。
  - 缺点:算法可读性差,魔法计算多;奇点分支与 `is_proper` 逻辑容易用错;没有单元测试覆盖各轴序。

- **`eulers(Matrix3d R, ...)`**(`:76-80`)
  - 作用:旋转矩阵→欧拉角,内部转四元数后复用上一函数。
  - 缺点:多一次四元数转换,精度略有损失。

- **`rotation_matrix(const Vector3d & ypr)`**(`:82-101`)
  - 作用:yaw-pitch-roll(先 z 后 y 后 x,即 zyx)构造旋转矩阵。
  - 实现:`:84-86` 注意入参 `ypr` 里 `roll=ypr[2]`、`pitch=ypr[1]`、`yaw=ypr[0]`;`:94-98` 手写 3×3 矩阵。
  - 缺点:函数名与入参名不一致(参数叫 `ypr` 但顺序其实是 yaw/pitch/roll,语义易混);手写矩阵易与其它函数轴序不一致导致符号错误。

- **`xyz2ypd` / `ypd2xyz`**(`:103-110`、`:139-146`)
  - 作用:直角坐标 ↔ 球坐标(yaw、pitch、distance)互转。
  - 实现:`xyz2ypd` 用 `atan2(y,x)` 求 yaw、`atan2(z, sqrt(x²+y²))` 求 pitch、欧氏距离求 distance;`ypd2xyz` 是逆变换。
  - 优点:云台常用 yaw/pitch/distance 表示,与下位机通信字段直接对应。

- **`xyz2ypd_jacobian` / `ypd2xyz_jacobian`**(`:112-137`、`:148-177`)
  - 作用:上两个变换对自变量的雅可比矩阵,用于 EKF 量测模型的线性化。
  - 缺点:`xyz2ypd_jacobian` 里 `x==0 && y==0`(目标在正下方)时 `x*x+y*y=0` 会除零;纯解析求导易出错且无测试。

- **`delta_time(a, b)`**(`:179-184`)
  - 作用:`steady_clock` 时间点差值,返回秒(double)。

- **`get_abs_angle(v1, v2)`**(`:186-192`)
  - 作用:两向量夹角,范围 `[0, π]`,零向量时返回 0(防除零)。

- **`square`**(`:49-53`)
  - 作用:模板求平方。

- **`limit_min_max`**(`:194-201`)
  - 作用:数值限幅。与 `pid.cpp` 里的 `clip` 功能重复。

**优点**:坐标变换集中于一处,避免散落各处重复实现;提供雅可比,便于 EKF 融合。
**缺点**:角度轴序约定散落在注释里,函数签名不表达语义,极易误用;缺少针对奇异点/轴序的测试;部分解析导数有除零风险。

### 5.2 pid.hpp / pid.cpp(36 / 27 行)

**作用**:位置式 PID 控制器,用于云台角度跟踪。

- 构造参数(`:9-15`):`dt` 控制周期、`kp/ki/kd` 三参数、`max_out` 输出限幅、`max_iout` 积分限幅、`angular` 是否角度模式。
- 公开调试量 `pout/iout/dout`(`:17-19`):三项分量,便于 PlotJuggler 调参。
- **`calc(set, fdb)`**(`:14-25`)
  - 作用:计算控制量。
  - 逐段:`:16-17` 角度模式时误差与微分用 `limit_rad` 归一化到 `(-π,π]`,避免角度回绕导致错误大误差;`:18` 更新 `last_fdb_`;`:20` P 项;`:21` I 项做积分限幅 `clip`;`:22` D 项用反馈差分 `(last_fdb_-fdb)/dt`(而非误差差分),避免 set 突变时的微分冲击;`:24` 总输出限幅。
- **优点**:抗角度回绕、抗微分冲击、积分抗饱和,是面向云台控制的成熟实现;三项分量暴露便于调试。
- **缺点**:`clip` 定义在全局命名空间且与 `limit_min_max` 重复;D 项直接乘 `kd` 未做低通滤波,噪声大时微分项抖动;未实现输出变化率限制。

### 5.3 extended_kalman_filter.hpp / .cpp(56 / 97 行)

**作用**:通用扩展卡尔曼滤波(EKF),用于目标状态估计(装甲板/能量机关运动建模)。

- 成员 `x`(状态)、`P`(协方差)公开(`:14-15`)。
- 构造(`:19-22`):初始状态/协方差,以及自定义状态加法 `x_add`(默认向量加法;对角度分量可传带限幅的加法)。
- **`predict(F, Q)` / `predict(F, Q, f)`**(`:23-35`)
  - 作用:预测步。线性版内部 `f(x)=F*x`;非线性版先 `P = F P Fᵀ + Q` 再 `x = f(x)`。
- **`update(z, H, R)` / `update(z, H, R, h, z_subtract)`**(`:37-96`)
  - 作用:量测更新步。
  - 逐段:`:50` 卡尔曼增益 `K = P Hᵀ (H P Hᵀ + R)⁻¹`;`:54` 用 Joseph 形式更新后验协方差(数值更稳定,保证对称正定);`:56` 状态更新,支持自定义 `z_subtract`(角度残差限幅);`:58-95` 卡方检验(NIS/NEES)统计,`:70-71` 阈值按自由度查表(NIS 自由度 4→9.488,NEES 自由度 11→19.675);`:78-85` 用滑动窗口统计近 N 次 NIS 失败率,写入 `data` 供调试。
- **优点**:Joseph 形式协方差更新数值稳定;支持自定义加/减法以处理角度回绕;卡方检验 + 滑动窗口是工程上很实用的滤波质量在线监控,注释明确解释了曾误用 0.711 阈值导致误判的教训。
- **缺点**:`update` 内部假设 `z` 为 4 维、状态为 11 维(硬编码 `residual[0..3]` 和阈值自由度),通用性打折,一旦状态维度改变此处会越界或阈值失效;`I` 单位阵在构造时按初始状态维度确定,状态维度中途改变会出错。

### 5.4 ransac_sine_fitter.hpp / .cpp(51 / 105 行)

**作用**:用 RANSAC + 线性最小二乘拟合正弦 `A·sin(ωt+φ)+C`,用于能量机关扇叶的旋转运动建模。

- `Result`(`:15-22`):拟合结果 `A/omega/phi/C/inliers`。
- 构造(`:11-19`):迭代次数、内点阈值、角频率搜索范围、随机数种子 `std::random_device`。
- **`add_data(t, v)`**(`:21-25`)
  - 作用:加入 (时间, 观测值) 数据点;若与上一点时间间隔 > 5s 则清空(视为新一轮旋转),防止跨轮数据污染。
- **`fit()`**(`:27-66`)
  - 作用:执行拟合。不足 3 点直接返回;`:35-63` 每轮打乱索引取 3 个采样点、随机采样一个 ω、用 `fit_partial_model` 解线性最小二乘得 `A1/A2/C`,再由 `A=√(A1²+A2²)`、`φ=atan2(A2,A1)` 还原幅相,最后 `evaluate_inliers` 统计内点并保留最优;`:65` 数据超 150 点丢弃最旧点。
- **`fit_partial_model`**(`:68-89`)
  - 作用:固定 ω 时,把 `A·sin(ωt+φ)+C` 展开为 `A1·sin(ωt)+A2·cos(ωt)+C`,构造设计矩阵 `X`,用 `bdcSvd`(双对角 SVD)最小二乘求解,`try/catch` 保护奇异。
- **`evaluate_inliers`**(`:91-103`)
  - 作用:统计预测值与观测残差小于阈值的点数。
- **优点**:把非线性 ω 与线性幅相参数解耦——RANSAC 随机搜 ω,内部线性求解幅相,收敛快、对离群点鲁棒;SVD 求解数值稳定。
- **缺点**:ω 在全范围均匀随机采样,ω 范围较大时收敛慢、可能陷局部;`threshold_` 是绝对误差阈值,不同量级观测值需重调;`best_result_` 不随新一轮数据清空,`add_data` 清空数据后旧结果仍保留。

### 5.5 img_tools.hpp / .cpp(26 / 30 行)

**作用**:OpenCV 绘图封装,便于调试可视化。

- `draw_point`(`:5-8`):画实心圆。
- `draw_points(Point)`(`:10-15`):把点集包成单条 contour 用 `drawContours` 连线(比逐点 `line` 简洁)。
- `draw_points(Point2f)`(`:17-22`):浮点点集转 `Point` 后复用。
- `draw_text`(`:24-29`):`putText` 写文字。
- **优点**:统一绘图入口,调用简洁。
- **缺点**:浮点→整型截断(非四舍五入),绘制精度略低;封装过于薄,边际价值有限。

### 5.6 plotter.hpp / .cpp(28 / 28 行)

**作用**:通过 UDP 把 JSON 数据发到 PlotJuggler(曲线绘制工具)进行在线调试。

- 构造(`:9-16`):创建 `AF_INET` + `SOCK_DGRAM` 套接字,填目标地址(默认 127.0.0.1:9870)。
- `plot(json)`(`:20-27`):加锁后 `json.dump()` 序列化并 `sendto` 发送。
- **优点**:UDP 无连接、低开销,丢包不影响主流程;互斥锁保证多线程调用安全;便于"赛前/赛后数据回看"。
- **缺点**:构造不检查 `socket()` 返回值(失败时为 -1,后续 sendto 静默失败);UDP 无重传,高频发送会丢点;无时间戳自动附加,需调用方自带。

### 5.7 trajectory.hpp / .cpp(20 / 32 行)

**作用**:忽略空气阻力的弹道解算,给定初速、水平距离、高度差求发射俯仰角与飞行时间。

- 构造(`:9-31`):`a = g d²/(2v₀²)`、`b = -d`、`c = a + h`,由二次方程 `a·tan²θ + b·tanθ + c = 0` 解得两个 `tanθ`,取飞行时间更短的一支(`:29-30`);判别式 `delta < 0` 时置 `unsolvable`。
- **优点**:公式简洁,计算量小,适合实时火控。
- **缺点**:完全忽略空气阻力与弹道下降的多项式修正,远距离精度有限;`g` 硬编码为 9.7833(特定纬度重力加速度),非通用;未考虑发射机构本身的仰角偏置。

### 5.8 recorder.hpp / .cpp(45 / 81 行)

**作用**:类 rosbag 的录制器——把图像(视频)+ 四元数姿态(文本,含时间戳)异步落盘,用于赛后回放。

- `FrameData`(`:23-28`):一帧的图像、四元数、时间戳。
- 构造(`:13-24`):建 `records/` 目录,按系统时间命名 `.avi` 与 `.txt`;队列容量 1(`queue_(1)` 即满了丢弃)。
- **`record(img, q, timestamp)`**(`:58-70`):空图跳过;首次调用 `init`;按 `fps` 节流(距上一帧不足 `1/fps` 则丢弃);入队。
- **`init`**(`:72-79`):打开文本,创建 MJPG 编码的 `VideoWriter`,启动保存线程。
- **`save_to_file`**(`:38-56`):保存线程循环 `pop` 帧,空帧跳过;写视频;把"距起点秒数 + 四元数 wxyz"写入文本。
- 析构(`:26-36`):置停止标志,推一个空帧解除阻塞,join 保存线程,关闭文件。
- **优点**:生产者(主循环)与消费者(磁盘 IO)解耦,不阻塞视觉主线程;节流控制录屏体积。
- **缺点**:队列容量 1 且 `push` 满时静默丢弃(`ThreadSafeQueue` 默认行为),录制帧实际 fps 可能远低于设定值;析构推空帧依赖 `save_to_file` 对空帧的 `continue` 判断;`text_writer_` 输出顺序注释写"wxyz",实际 `xyzw[3]` 是 w、`xyzw[0..2]` 是 xyz,正确但易误解。

### 5.9 logger.hpp / .cpp(11 / 35 行)

**作用**:基于 spdlog 的统一日志封装(单例)。

- `set_logger()`(`:15-27`):按系统时间生成 `logs/*.log` 文件 sink(debug 级)+ 彩色控制台 sink(debug 级),`flush_on(info)`。
- `logger()`(`:29-33`):懒初始化,返回全局 `logger_`。
- **优点**:文件+控制台双输出、时间戳文件名、懒加载单例。
- **缺点**:全局单例 `logger_` 无线程安全的初始化保护(多线程首次同时调用 `logger()` 可能重复初始化);`logs/` 目录不存在时 `basic_file_sink_mt` 会创建失败;未提供按模块命名 logger。

### 5.10 crc.hpp / .cpp(22 / 90 行)

**作用**:CRC8 / CRC16 校验(查表法),用于与下位机串口通信的帧校验。

- `CRC8_TABLE`/`CRC16_TABLE`(`:4-46`):预计算 256 项查表。
- `get_crc8`(`:50-63`):`CRC8_INIT=0xff` 初始化,逐字节查表异或。
- `check_crc8`(`:65-68`):校验值在帧尾,`get_crc8(data, len-1)` 与末字节比较。
- `get_crc16`(`:70-83`):`CRC16_INIT=0xffff`,标准 CRC-16。
- `check_crc16`(`:85-89`):末两字节(高字节在前)与计算结果比较。
- **优点**:查表法高效;`check_*` 封装了"len 是否含校验"的约定,调用方不易错。
- **缺点**:CRC 多项式/初值无注释说明是哪种标准(CRC-8/MAXIM 或 CRC-16/CCITT),跨语言对齐协议时易踩坑;两个大表占用只读内存但无 `constexpr`(CRC8_INIT 有 `constexpr`,表是 `const`)。

### 5.11 exiter.hpp / .cpp(15 / 19 行)

**作用**:捕获 `SIGINT`(Ctrl-C)置退出标志,供主循环优雅退出。

- 构造(`:11-16`):`exiter_inited_` 保证单例(重复构造抛异常);注册 SIGINT 信号处理器设 `exit_ = true`。
- **优点**:单例约束避免多处注册信号处理器。
- **缺点**:信号处理器里只置 `bool`(非 `volatile`/`std::atomic`),严格讲存在未定义行为风险(实践中多数平台可用);`std::signal` 的 handler 中做复杂操作不安全,此处仅置位可接受但应声明 `volatile sig_atomic_t`。

### 5.12 thread_pool.hpp(186 行)

**作用**:通用线程池 + 帧序号有序队列 + YOLO 批量创建辅助。

- `Frame`(`:16-23`):一帧的 id、图像、时间戳、四元数、检测到的装甲板列表。
- `create_yolo11s/create_yolov8s`(`:25-43`):按配置批量创建 N 个 YOLO 推理器(多线程各自持有一个,避免 OpenVINO 推理器跨线程竞争)。
- `OrderedQueue`(`:45-121`):按帧 `id` 严格有序出队的并发队列。
  - `enqueue`(`:61-88`):id 小于当前期望 id 丢弃(迟到帧);等于则入主队列并推进,随后把 `buffer_` 里连续 id 的帧依次搬入;否则暂存到 `buffer_` 等待。
  - `dequeue/try_dequeue`(`:90-111`):阻塞/非阻塞取队首。
  - **优点**:保证多线程并行检测后结果按帧序消费,解决乱序问题。
  - **缺点**:`buffer_` 无上限,若某帧永久丢失,后续所有帧积压在 buffer 且 `current_id_` 停住,队列永久卡死(缺少"跳过缺失帧"的超时/上限策略)。
- `ThreadPool`(`:123-182`):经典实现——工作线程在条件变量上等待任务,`enqueue` 投递;析构置 stop 并唤醒、join 所有线程。
  - **缺点**:任务队列无界,提交速度超过处理速度时内存增长;异常在 `task()` 中抛出会终止工作线程且无捕获。

### 5.13 thread_safe_queue.hpp(152 行)

**作用**:模板化的线程安全队列,支持满时丢弃/回调、非阻塞、清空等。

- 模板参数 `PopWhenFull`(`:12`):满时是丢弃最旧帧还是触发回调丢弃新帧。
- `push`(`:22-37`):满时按 `PopWhenFull` 选择 `pop` 最旧或调用 `full_handler_` 后丢弃新帧。
- `try_push`(`:42-51`):满时返回 false,不丢帧也不静默,由调用方处理(注释解释了动机:入队失败要能还资源)。
- `try_pop/pop/front/back/empty/full/size/clear`(`:53-140`):各种带锁操作。
- **优点**:接口丰富,满策略可配置,`try_*` 系列满足"失败需反馈"场景;锁粒度细。
- **缺点**:`pop(T&)` 与 `front()` 在 `wait` 被唤醒后、队空时行为不一致(一个打印错误返回、一个返回默认值),竞态下仍有隐患;无 `max_size==0`(无界)模式;`clear` 的 `notify_all` 语义在无界等待者场景可能造成忙等。

### 5.14 yaml.hpp(32 行)

**作用**:yaml-cpp 的安全封装,加载配置并做错误处理。

- `load(path)`(`:10-21`):`YAML::LoadFile` 失败或解析异常时打日志并 `exit(1)`。
- `read<T>(yaml, key)`(`:23-29`):键不存在时打日志退出。
- **优点**:集中处理配置错误,避免每个读取点重复 try/catch;模板化读取简洁。
- **缺点**:错误处理直接 `exit(1)` 而非抛异常/返回错误码,不便于部分初始化失败的优雅降级;`read` 只检查 `yaml[key]` 是否为 null,类型不匹配时 `as<T>()` 仍会抛未捕获异常。

---


---

# IO 层分析

## 模块概述

`io/` 层是整个 RoboMaster 视觉系统（sp_vision_25）最底层的硬件抽象层，负责把相机、云台/电控、IMU、串口、CAN、USB、ROS2 导航等异构设备统一封装成上层 `tasks/`、`src/`、`auto_aim/` 等模块可直接调用的 C++ 对象。上层只看到"读一帧图像""拿一个姿态四元数""发一个云台指令""订阅一个敌方状态"这样的接口，不必关心底层是海康、迈德威视还是 V4L2 摄像头，也不必关心云台是走 CAN、走 UART 帧还是走 USB CDC 虚拟串口。

相机抽象分两条线。一条是正规的抽象工厂：`CameraBase` 纯虚接口定义 `read(cv::Mat&, time_point&)`，`Camera` 作为工厂根据 YAML 里的 `camera_name` 在 `MindVision`（迈德威视 SDK）与 `HikRobot`（海康 MVS SDK）之间二选一，两者都继承 `CameraBase`。另一条是独立成型的 `USBCamera`（OpenCV V4L2），它并不继承 `CameraBase`，因此无法被 `Camera` 工厂选中，只能被 `src/sentry*.cpp` 等入口直接实例化——这是本项目相机抽象不彻底的一处历史遗留。

云台/电控通信目前存在**三套并存的实现**，反映了项目从 CAN 方案向串口方案迁移的过渡状态：`CBoard` 走 Linux SocketCAN（把 `Command` 打包成 8 字节 `can_frame`，并顺带从 CAN 报文里解算 IMU 四元数与弹速）；`CBoard1` 走 UART 裸串口（`0x5A/0xA5` 帧头 + CRC16，`src/standard1.cpp` 使用）；`Gimbal` 走 USB CDC 虚拟串口（`'S''P'` 帧头 + CRC16，`src/standard_mpc.cpp`、`src/mt_standard.cpp` 使用）。三者都实现了 `send(Command)` 与 `q(timestamp)`/`imu_at(timestamp)` 这类同名接口，方便上层互换，但内部协议、字节序、CRC 多项式、姿态插值实现互不相同。

数据流向是典型的生产者-消费者模型：每个设备驱动内部都开一个专门的接收/取流线程，把原始数据（帧图像、四元数、CAN 报文、串口报文）解析后写入一个 `tools::ThreadSafeQueue`（`tools/thread_safe_queue.hpp`）；上层主循环按需 `pop`/`try_pop` 取最新数据。相机驱动（海康/迈德威视/USB）更是自带"守护线程"：取流线程一旦因掉线退出，守护线程会自动重开设备、甚至通过 libusb 对 USB 设备执行 `reset_device`，实现无人值守的自动恢复。

`command` 与 `protocol` 两个头文件是协议栈的"字典"：`io/command.hpp` 定义了视觉→电控的 `Command` 结构体（含 `control/shoot/yaw/pitch/horizon_distance` 以及扩展的 `target_x/y/z/vx/vy/vz/yaw/v_yaw/r1/r2/dz` 等字段，同时兼容 CAN 与串口两套发送方）和 `Mode` 枚举；`io/protocol.hpp` 定义了 28 字节 `ReceivePacket`（电控→视觉）与 48 字节 `SendPacket`（视觉→电控）两个 packed 结构体及 CRC16 实现。`io/vision_protocol.hpp` 则定义了另一套带 SOF/长度/CRC8/CMD_ID 的帧打包/解包函数，但经检索**当前全仓库没有任何文件引用它**，属于死代码。

整体上，`io` 层代码量不大（29 个文件，多数是单文件驱动），注释质量两极分化：`gimbal.cpp` 的注释记录了 2026-10-12 联调踩过的真实坑（插值参数写反、外推放大角速度、termios 重枚举丢配置导致 96% 帧解析失败），是最用心的一份；而 `protocol.hpp` 的 CRC 注释与实际表不一致、`cboard.cpp`/`dm_imu.cpp` 仍保留着 `gimbal.cpp` 已修复的插值除零隐患，说明代码是在多个版本间逐步迭代、尚未统一收敛的。

---

### 文件: io/command.cpp / io/command.hpp

总括：定义视觉系统的工作模式枚举、模式名称表，以及视觉发给电控的统一命令结构体 `Command`。

- **`enum class Mode`**（`command.hpp:11-17`）
  - **作用**：枚举工作模式 `idle=0 / auto_aim=1 / auto_buff=2 / small_buff=3 / big_buff=4`。
  - **逐行说明**：显式给每个枚举量赋了值，因为后续要被当作数组下标（`MODES[...]`）以及 CAN 报文里的字节（`Mode(frame.data[2])`）使用，数值必须与电控固件约定一致。
  - **优点**：`enum class` 强类型，避免与其它枚举隐式混用。
  - **缺点/隐患**：一旦电控端模式编号变动，这里不会告警，只能靠联调发现；且 `Mode` 值直接取自 CAN 报文字节（见 cboard），超出 0~4 会触发下标越界。

- **`extern const std::vector<std::string> MODES`**（`command.hpp:20`，定义于 `command.cpp:4-10`）
  - **作用**：模式枚举到中文/英文名字的映射表，用于日志输出。
  - **逐行说明**：`command.cpp` 里按 `Mode` 的数值顺序填入 `"idle","auto_aim","auto_buff","small_buff","big_buff"` 五个字符串。
  - **优点**：集中维护，避免各处散落魔法字符串。
  - **缺点/隐患**：`std::vector<std::string>` 用 `extern` 在头文件声明、cpp 定义，若有多处包含且链接不当可能出现多重定义；向量大小靠"人肉保证与 `Mode` 一致"，下标越界无保护。

- **`struct Command`**（`command.hpp:23-45`）
  - **作用**：视觉→电控的命令载体。前半段 `control/shoot/yaw/pitch/horizon_distance` 是 CAN（`CBoard::send`）使用的字段；后半段 `target_x/y/z/vx/vy/vz/yaw/v_yaw/r1/r2/dz/tracking/id/armors_num` 是串口 `SendPacket`（`CBoard1::send`）使用的扩展字段。
  - **逐行说明**：`:24-28` 是布尔控制位与位置量（`yaw/pitch` 单位 rad，`horizon_distance` 单位 m）；`:31-38` 是目标状态量（位置、速度、yaw 角、yaw 角速度）；`:39-40` `r1=r2=0.12` 是默认弹道半径；`:42-44` `tracking`、`id`、`armors_num=4` 对应 `SendPacket` 位域。
  - **优点**：一个结构体同时喂饱 CAN 与串口两套发送路径，上层调用方只用填 `Command`，不必关心底层协议。
  - **缺点/隐患**：结构体字段语义混杂——`yaw/pitch` 与 `target_yaw/target_x...` 代表的是两套完全不同的数据（一套是"直接发角度"，一套是"发目标状态"），调用方很容易只填一套而另一套保持默认 0 导致下位机收到全零帧（见 `CBoard1::send(8参数)` 的缺陷）。`r1/r2=0.12` 这种物理参数硬编码在结构体默认值里，一旦弹道参数改变就得改代码重编译。

---

### 文件: io/protocol.cpp / io/protocol.hpp

总括：定义串口自瞄链路的两个 packed 帧结构体（`ReceivePacket`/`SendPacket`）和它们共用的 CRC16 查表实现。

- **`struct ReceivePacket`**（`protocol.hpp:9-21`）
  - **作用**：电控→视觉的 28 字节帧（`header 0x5A` + flags 位域 + roll/pitch/yaw + aim_x/y/z 共 6 个 float + CRC16）。
  - **逐行说明**：`:10` `header=0x5A` 帧头；`:11-13` 三个位域压缩进 1 字节 flags：`detect_color`（0=红 1=蓝）、`reset_tracker`、`reserved:6`；`:14-19` 六个体位/瞄准量；`:20` `checksum`；`:21` `__attribute__((packed))` 取消对齐填充，保证内存布局恰好 28 字节。
  - **优点**：位域 + packed 精确控制线上字节，结构清晰。
  - **缺点/隐患**：packed 结构体直接 `memcpy` 到内存再用 `reinterpret_cast` 解析，依赖小端平台（x86/ARM 均小端，问题不大）；位域布局是编译器实现定义，跨编译器（尤其跨 GCC/Clang/MSVC）不能保证 `detect_color` 一定在 bit0，且电控端若用大端或不同位域顺序会静默错位。

- **`struct SendPacket`**（`protocol.hpp:26-44`）
  - **作用**：视觉→电控的 48 字节帧（`header 0xA5` + 位域 `tracking/id/armors_num/reserved` + 11 个 float + CRC16）。
  - **逐行说明**：`:27` `header=0xA5`；`:28-31` `tracking:1`、`id:3`、`armors_num:3`、`reserved:1`；`:32-42` 目标 x/y/z/yaw/vx/vy/vz/v_yaw/r1/r2/dz 共 11 个 float（44 字节）+2 字节 CRC = 48 字节；`:44` packed。
  - **优点**：与 `Command` 扩展字段一一对应，便于 `CBoard1::send` 直接搬运。
  - **缺点/隐患**：头文件注释里的字节计数写的是 48 字节，实测也对，但注释里只列到 12 项 float 会让人误数；`id:3` 只有 3 bit，最多表达 0~7 号装甲板，若装甲板 id 超过 7 会截断。

- **`static const uint16_t CRC16_TABLE[256]`**（`protocol.cpp:4-37`）
  - **作用**：CRC16 查表表，poly=0x1021、init=0xFFFF（即 CRC-16/XMODEM 反射形式）。
  - **逐行说明**：`crc = (crc >> 8) ^ TABLE[(crc ^ data[i]) & 0xFF]`（`:42`）是右移查表法的标准写法，对应"poly 0x1021、refin/refout=true、init 0xFFFF"。
  - **优点**：查表比逐位快。
  - **缺点/隐患**：**头文件 `protocol.hpp:46` 的注释写的是"右移法, poly=0xA001"，而实际表是 0x1021 反射表，注释与实现矛盾**。更严重的是本仓库存在**三个同名 `calculateCRC16`**：`protocol.cpp` 的全局函数（0x1021）、`vision_protocol.hpp` 的 `io::calculateCRC16`（0xA001 逐位 MODBUS）、以及 `tools/crc.hpp` 的 `tools::get_crc16`（供 gimbal/dm_imu 使用）。三者的多项式不同、结果不同，误用会产生"视觉算的 CRC 与电控永远对不上"的隐蔽故障。

- **`uint16_t calculateCRC16(const uint8_t*, size_t)`**（`protocol.cpp:39-45`）
  - **作用**：对 `data` 前 `len` 字节计算 CRC16，供 `CBoard1` 收发校验。
  - **逐行说明**：`:40` 初始 0xFFFF；`:41-43` 逐字节查表；`:44` 返回。
  - **优点**：实现简单正确（针对 0x1021）。
  - **缺点/隐患**：函数定义在**全局命名空间**（未包在 `namespace io`），与 `vision_protocol.hpp` 里 `namespace io` 下的 `inline` 同名函数共存，调用处 `using namespace io` 或 `io::` 前缀会静默选到不同的实现，属于易踩的命名陷阱。

---

### 文件: io/vision_protocol.hpp

总括：另一套带 SOF/长度/CRC8/CMD_ID 的帧协议工具（纯头文件 inline 实现），**全仓库无任何文件引用，是死代码**。

- **`inline uint8_t calculateCRC8`**（`:10-20`）
  - **作用**：CRC8，poly=0x31、init=0x00、MSB 先移。
  - **逐行说明**：`:11` 初始 0；`:13` 异或字节；`:14-17` 逐位：最高位为 1 时 `(crc<<1)^0x31`，否则左移。
  - **缺点/隐患**：与另外两个 CRC16 一样，命名空间/多项式混乱，未使用时问题不大，一旦将来被启用易与 `protocol.cpp` 混淆。

- **`inline uint16_t calculateCRC16`**（`:22-32`）
  - **作用**：CRC16/MODBUS，poly=0xA001、init=0xFFFF、LSB 先移（逐位版）。
  - **逐行说明**：`:24-30` 逐字节逐位，最低位为 1 时 `(crc>>1)^0xA001`。注意与 `protocol.cpp` 的 0x1021 查表版**结果不同**。

- **`writeUint16LE / readUint16LE / writeFloatLE / readFloatLE`**（`:34-51`）
  - **作用**：小端序 16 位整数与 float 的读写封装。
  - **逐行说明**：`writeFloatLE`/`readFloatLE` 直接用 `std::memcpy` 搬运 float 的 4 字节（`:43-51`），隐式假定主机即小端 IEEE754。

- **`SOF / CMD_ID_VISION_TO_MCU / CMD_ID_MCU_TO_VISION`**（`:53-55`）
  - **作用**：常量 `SOF=0xA5`、命令号 `0x0001/0x0002`。

- **`inline std::vector<uint8_t> packVisionFrame(uint16_t flags, float pitch, float yaw)`**（`:57-70`）
  - **作用**：打包一条视觉→电控的 18 字节帧。
  - **逐行说明**：`:60` `SOF`；`:61` `writeUint16LE(&frame[1], 10)` 写"数据长度"；`:62` 对前 3 字节算 CRC8 放到 `frame[3]`；`:63-66` 依次写 cmd_id、flags、pitch(offset 8)、yaw(offset 12)；`:67-68` 对前 16 字节算 CRC16 放到尾部。
  - **缺点/隐患**：**长度字段硬编码为 10，但实际载荷（cmd_id 2 + flags 2 + pitch 4 + yaw 4 = 12 字节）是 12，语义不符**；更关键的是 `packVisionFrame` 把 pitch 放 offset 8、yaw 放 offset 12，而下面 `unpackMCUFrame` 读的是 yaw 在 offset 8、pitch 在 offset 12——收发两个方向字段顺序相反，若两端期望对称则这是 bug（也可能是与某版固件的约定，但代码注释未说明）。

- **`struct MCUFrameData`**（`:72-78`）
  - **作用**：解包结果结构，含 `flags/yaw/pitch/roll/valid`。

- **`inline MCUFrameData unpackMCUFrame(const uint8_t* frame, size_t len)`**（`:80-99`）
  - **作用**：解包电控→视觉帧并校验。
  - **逐行说明**：`:82` `len<10` 直接返回 invalid；`:83` 校验 SOF；`:84-86` 读长度并断言 `len == data_length+8`；`:87` CRC8 校验；`:88-90` CRC16 校验；`:91-92` 校验 cmd_id；`:93-97` 依次读 flags、yaw(8)、pitch(12)、roll(16)，置 `valid=true`。
  - **优点**：校验链条完整（SOF→长度→CRC8→CRC16→cmd_id），失败即返回 invalid。
  - **缺点/隐患**：`len < 10` 的最小长度检查偏松（真正需要 len≥18），但因为有 `len != data_length+8` 兜底，实际不会越界读；不过 `data_length` 本身来自网络帧、若帧头伪装成 `0xA5` 且长度字段被篡改，`expected_len` 会被算成巨大值，仅靠 `len != expected_len` 拒绝，逻辑上安全但不够直观。

---

### 文件: io/camera.cpp / io/camera.hpp

总括：相机抽象接口 `CameraBase` 与工厂类 `Camera`，根据 YAML 选择海康或迈德威视相机。

- **`class CameraBase`**（`camera.hpp:11-16`）
  - **作用**：纯虚基类，定义统一取图接口 `read(cv::Mat&, time_point&)`，虚析构保证多态析构安全。
  - **优点**：标准抽象，`HikRobot`/`MindVision` 都继承它，工厂能统一管理。

- **`class Camera`**（`camera.hpp:18-26`，`camera.cpp:11-37`）
  - **作用**：相机工厂/门面，内部持有一个 `std::unique_ptr<CameraBase> camera_`。
  - **构造函数**（`camera.cpp:11-32`）逐行说明：`:13` 用 `tools::load` 读 YAML；`:14` 读 `camera_name`；`:15` 读 `exposure_ms`；`:17-21` 若为 `mindvision` 再读 `gamma`、`vid_pid`，构造 `MindVision(exposure_ms, gamma, vid_pid)`；`:23-27` 若为 `hikrobot` 再读 `gain`、`vid_pid`，构造 `HikRobot(exposure_ms, gain, vid_pid)`；`:29-31` 其它名字抛 `runtime_error`。
  - **`read`**（`camera.cpp:34-37`）：直接转发给 `camera_->read(img, timestamp)`。
  - **优点**：工厂模式集中封装配置解析与对象创建，上层拿到 `Camera` 即完成初始化；`unique_ptr` 自动管理生命周期。
  - **缺点/隐患**：只支持 mindvision/hikrobot 两种名字，**`USBCamera` 未接入此工厂**，导致 USB 相机只能在各 `src` 入口里裸用，抽象不统一；`exposure_ms` 用 `read<double>` 读取，若 YAML 里写成整数会因 yaml-cpp 类型不匹配抛异常；`vid_pid` 在工厂里被读出来传进两个相机，但两个相机各自再 parse 一遍，重复解析。

---

### 文件: io/cboard.cpp / io/cboard.hpp

总括：基于 SocketCAN 的电控通信类，负责把 `Command` 打包成 `can_frame` 发送，并从 CAN 报文解算 IMU 四元数与弹速/模式/射击模式。

- **`enum ShootMode` 与 `SHOOT_MODES`**（`cboard.hpp:23-28`）
  - **作用**：哨兵专有的射击模式 `left_shoot/right_shoot/both_shoot` 及其名字表。
  - **缺点/隐患**：用的是**非 scoped 的普通 `enum`**（不是 `enum class`），枚举量 `left_shoot` 等会泄漏到 `io` 命名空间，易与其它枚举冲突；名字表同样靠人肉与枚举对齐，越界无保护。

- **`class CBoard`**（`cboard.hpp:30-61`）
  - **成员变量**：`:33-36` 公开的 `double bullet_speed`、`Mode mode`、`ShootMode shoot_mode`、`double ft_angle`（摩擦轮角度）；`:44-49` 私有 `IMUData`（四元数+时间戳）、`queue_`、`can_`、`data_ahead_/data_behind_`；`:56` 三个 canid。
  - **构造函数**（`cboard.cpp:8-20`）逐行说明：`:9-13` 初始化 `mode=idle`、`shoot_mode=left_shoot`、`bullet_speed=0`、`queue_(5000)`、`can_(read_yaml(config_path), bind(&CBoard::callback,...))`；`:16-18` 阻塞 `pop` 两个 IMU 样本填充 `data_ahead_/data_behind_`；`:19` 打日志。
  - **`imu_at(timestamp)`**（`cboard.cpp:22-45`）逐行说明：`:24` 若 `data_behind_` 早于查询时刻，先把 `data_ahead_` 推进到 `data_behind_`；`:26-30` 循环 pop 直到找到时间戳晚于查询时刻的样本；`:32-42` 对前后两个四元数做 slerp 插值，`k = (t_c-t_a)/(t_b-t_a)`。
    - **优点**：思路正确（用前后两帧四元数 slerp 得到任意时刻姿态）。
    - **缺点/隐患**：**没有做 `t_ab<=0` 的除零防护、也没有把 `k` clamp 到 [0,1]**——这正是 `gimbal.cpp:137-151` 注释里明确记录、2026-10-12 已修复的那个"时间戳参数写反→t_ab 恒为负→插值分支永远不执行/外推放大角速度→云台震荡"的 bug，`CBoard` 这里仍是旧版实现，隐患保留；此外 `data_ahead_/data_behind_` 非线程安全，若 `imu_at` 被多线程并发调用会互相踩踏。
  - **`send(Command) const`**（`cboard.cpp:47-66`）逐行说明：`:49-51` 组 `can_frame`（`send_canid_`、`dlc=8`）；`:52-53` `control`/`shoot` 各占 1 字节；`:54-59` 把 `yaw/pitch/horizon_distance` 各乘 1e4 转 `int16` 后按**大端**拆分高低字节写入 `data[2..7]`；`:61-65` try/catch 写 CAN。
    - **优点**：异常被捕获并告警，不会因单次写失败崩溃。
    - **缺点/隐患**：`(int16_t)(command.yaw * 1e4) >> 8` 对负值做右移是**实现定义行为**（大多数平台算术右移恰好正确，但标准不保证）；`float * 1e4` 转 `int16` 存在溢出风险——`horizon_distance` 单位为米，5 m 时 5e4 已超 int16 最大值 32767，会静默回绕成负值；`yaw/pitch` 单位为 rad，±π×1e4≈±31415 尚在范围内，但没有任何范围校验。
  - **`callback(const can_frame&)`**（`cboard.cpp:68-103`）逐行说明：`:70` 取当前时间；`:72-84` 若 `can_id == quaternion_canid_`，解析 4 个 int16（大端 `data[0]<<8|data[1]` 等）除以 1e4 得到 x/y/z/w，`:78-81` 校验四元数模长（`|x²+y²+z²+w²-1|>1e-2` 则丢弃），`:83` 入队 `{w,x,y,z}`；`:86-101` 若 `can_id == bullet_speed_canid_`，解析弹速（int16/1e2）、`mode=Mode(data[2])`、`shoot_mode=ShootMode(data[3])`、`ft_angle`（int16/1e4），`:93-100` 用 `static` 时间戳限频到 1Hz 打日志。
    - **优点**：四元数模长校验能滤掉坏数据；1Hz 限频日志避免刷屏。
    - **缺点/隐患**：`MODES[static_cast<int>(mode)]` 与 `SHOOT_MODES[shoot_mode]` **下标未做范围检查**，若电控发来超范围的 mode/shoot_mode 会数组越界导致未定义行为/崩溃；`mode`、`shoot_mode`、`bullet_speed`、`ft_angle` 在 `callback`（CAN 接收线程）写、而主线程读，**没有任何锁或原子**，是数据竞争。
  - **`read_yaml(config_path)`**（`cboard.cpp:106-119`）逐行说明：`:110-112` 读三个 canid；`:114-116` 校验 `can_interface` 存在否则抛异常；`:118` 返回 `can_interface` 字符串供 `SocketCAN` 构造。
    - **缺点/隐患**：构造函数里 `can_` 的初始化实参 `read_yaml(config_path)` 先于成员 `queue_` 等完成，且 `callback` 会在 `SocketCAN` 构造过程中被后台线程触发——头文件注释 `cboard.hpp:51` 自己都标了"callback 的运行会早于 CBoard 构造函数的完成"，属于真实的初始化顺序竞争隐患。

---

### 文件: io/cboard1.cpp / io/cboard1.hpp

总括：基于 UART 裸串口的电控通信类（`src/standard1.cpp` 使用），走 `0x5A/0xA5` + CRC16 的 `ReceivePacket`/`SendPacket` 协议，并实现了一套与 `Gimbal` 兼容的接口。

- **`enum class CBoard1Mode`**（`cboard1.hpp:18-23`）与 **`struct CBoard1State`**（`:26-33`）
  - **作用**：模仿 `GimbalMode`/`GimbalState` 的模式与状态结构，便于上层在 `CBoard1` 与 `Gimbal` 间无缝切换。
  - **缺点/隐患**：`CBoard1State` 里 `yaw_vel/pitch_vel/bullet_speed/bullet_count` 等字段实际上几乎从未被真实数据填充（见下），属于"为兼容而设"的空壳。

- **`class CBoard1`**（`cboard1.hpp:35-95`，`cboard1.cpp`）
  - **构造函数**（`cboard1.cpp:16-33`）逐行说明：`:17-19` 初始化 `last_q_=Identity`、`rx_buffer_(512)`；`:21-25` `open(O_RDWR|O_NOCTTY|O_NDELAY)` 失败抛 `system_error`；`:27-30` `configureSerial()` 失败则 `close` 并抛异常；`:32` 打日志。
  - **析构函数**（`:35-40`）：`fd>=0` 则 `close`。
  - **`configureSerial()`**（`:42-69`）逐行说明：`:44` `tcgetattr` 失败返回 false；`:47-48` 波特率 115200；`:50-55` `CLOCAL|CREAD`、`CS8`、无校验、1 停止位、关闭硬件流控；`:57-60` 关闭 ICANON/ECHO/ISIG、IXON 软件流控、输入输出换行转换；`:62-63` `VMIN=0/VTIME=0` 非阻塞读；`:65` 生效；`:67` `tcflush` 清空缓冲。
    - **优点**：raw/8N1 配置正确，注释明确标注电控端 USART1 波特率 115200。
    - **缺点/隐患**：非阻塞读（VMIN=0/VTIME=0）意味着 `update()` 里的 `read` 可能返回 0，调用方必须反复轮询，但没有配套的重连/掉线检测逻辑（对比 `Gimbal` 有完整的重连状态机），串口掉线后 `CBoard1` 只会静默读不到数据。
  - **`send(const Command&) const`**（`:71-101`）逐行说明：`:74-88` 把 `Command` 的 `tracking/id/armors_num/target_x..dz` 逐个搬运进 `SendPacket`；`:90-92` 用 `calculateCRC16`（protocol.cpp 的 0x1021）对除 checksum 外的字节算 CRC 填回；`:94-100` `write` 串口，部分写入告警、全量写入打 info。
    - **缺点/隐患**：`:98-99` 的 info 日志打印 `command.yaw/command.pitch`，但这两个字段在 `SendPacket` 里根本不存在（发送的是 `target_*`），所以日志里 yaw/pitch 恒为 0，误导排查。
  - **`update()`**（`:103-110`）：每次读最多 256 字节追加到 `rx_buffer_`，再 `parseBuffer()`。
  - **`parseBuffer()`**（`:112-169`）逐行说明：`:115-125` 循环直到不足一帧，`std::find` 找 `0x5A` 帧头，找不到清空、找到则丢弃帧头前的垃圾字节；`:128-129` `memcpy` 一帧 `ReceivePacket`；`:132-137` 校验 CRC16，失败 `recordCrc(false)` 并**只删除 1 字节**重新同步；`:138` `recordCrc(true)`；`:141-142` 保存 `last_received_`，**`mode = Mode::auto_aim` 硬编码强制自瞄（注释自己说"联调用，正式比赛需删掉"）**；`:145-148` 欧拉角转四元数（ZYX：yaw*Z × pitch*Y × roll*X）并归一化存 `last_q_`；`:151-160` 1Hz 限频日志；`:163` 删除已处理帧；`:166-168` 缓冲超 1024 字节时裁掉前 512 字节防爆。
    - **优点**：CRC 失败只删 1 字节的逐字节重同步策略健壮；有缓冲区上限保护。
    - **缺点/隐患**：`mode = Mode::auto_aim` 硬编码是**明确的生产事故隐患**；用 `std::find` 找 `0x5A` 时，float 载荷里也完全可能出现 `0x5A`，会造成假帧头（虽然后续 CRC 能纠正，但会丢帧）；每次只删 1 字节重同步在长数据流下可能多次误判。
  - **`recordCrc(bool)` / `crc_rate()`**（`:171-191`）：滑动窗口（`crc_window_=5s`）统计 CRC 通过率，返回百分比或 `-1.0`。`crc_rate` 用 `int64_t` 计数遍历 deque。
    - **优点**：滑动窗口只保留近 5s，能反映实时链路质量。
    - **缺点/隐患**：`crc_rate()` 与 `recordCrc()` 之间没有锁（`crc_history_` 被写线程和读线程同时访问），**数据竞争**——对比 `gimbal.cpp` 的实现是有 `crc_mutex_` 的，这里偷工了。
  - **`gimbal_mode()/state()/str()/q()`**（`:194-225`）：兼容接口。`gimbal_mode` 把 `auto_buff` 映射成 `SMALL_BUFF`（映射本身有歧义）；`state()` 用 `last_received_.yaw/pitch` 填 yaw/pitch，但 `yaw_vel_/pitch_vel_/bullet_speed/bullet_count_` 是成员缓存值；`q(t)` 调 `imu_at(t)`。
    - **缺点/隐患**：`state()` 里的 `bullet_speed` 直接读成员 `bullet_speed`（默认 0，从未被 `parseBuffer` 更新），`bullet_count_` 也从不更新，所以 `state()` 返回的弹速/弹量恒为 0；`q(t)` 实际走 `imu_at(t)`（见下），但 `imu_at` 在此类里只是一个无插值的访问器，名不副实。
  - **`imu_at(timestamp) const`**（`cboard1.hpp:55-57`）：`return last_q_.normalized()`，**完全忽略 timestamp、不做 slerp**。
    - **缺点/隐患**：与 `CBoard::imu_at`、`DM_IMU::imu_at`、`Gimbal::q` 的"按时间插值"语义不一致，同一接口名在不同类里行为不同，上层若按 `Gimbal` 的语义调用会拿到滞后且无插值的姿态。
  - **`send(8参数)`**（`:227-242`）：`:231-232` 缓存 `yaw_vel_/pitch_vel_`；`:234-241` 组装 `Command`（只填 `control/shoot/yaw/pitch/tracking`）后调用 `send(command)`。
    - **缺点/隐患**：**`Command` 的 `target_x/y/z/vx/vy/vz/target_yaw/v_yaw/r1/r2/dz` 全部保持默认 0**，所以经这个 8 参数接口发送的 `SendPacket` 里目标位置/速度/弹道参数全是 0，只有 `tracking` 和 `control` 生效；`yaw_acc/pitch_acc` 参数被直接忽略。若上层期望这路也发完整目标状态，结果会静默错误。
  - **getter**（`cboard1.hpp:62-72`）：`getRobotColor()` 返回 `detect_color==0`（红），`getResetTracker/getRoll/getPitch/getYaw` 读 `last_received_`。
    - **缺点/隐患**：在收到第一帧前，`last_received_` 是默认值（`detect_color=0` 即红），`getRobotColor()` 会误报红色，调用方拿不到"未初始化"的区分。

---

### 文件: io/socketcan.hpp

总括：Linux SocketCAN 的 RAII 封装，内置 epoll 接收线程与守护重连线程。

- **`constexpr int MAX_EVENTS = 10`**（`:20`）：epoll 单次最多处理的事件数。

- **`class SocketCAN`**（`:24-155`）
  - **构造函数**（`:27-50`）逐行说明：`:28-33` 初始化 `interface_/socket_fd_=-1/epoll_fd_=-1/rx_handler_/quit_=false/ok_=false`；`:35` `try_open()`；`:38-49` 起守护线程：每 100ms 检查 `ok_`，若 `ok_` 则 continue，否则 join 读线程、`close()`、`try_open()` 重开。
  - **析构函数**（`:52-59`）：`quit_=true`，join 两个线程，`close()`。
  - **`write(can_frame*)`**（`:61-65`）：`::write` 失败抛 `runtime_error`。
  - **`open()`**（`:79-124`）逐行说明：`:81-82` `socket(PF_CAN, SOCK_RAW, CAN_RAW)`；`:84-87` `ioctl(SIOCGIFINDEX)` 拿接口索引；`:89-96` `bind` 到 `sockaddr_can`（失败则 `close(socket_fd_)` 后抛）；`:98-105` `epoll_create1` + `epoll_ctl ADD`；`:108-122` 起读线程：`ok_=true`，10us 睡眠循环，`read()` 抛异常则 `ok_=false` 并 break。
  - **`try_open()`**（`:126-133`）：`open()` 抛异常仅告警不抛出。
  - **`read()`**（`:135-146`）：`epoll_wait(…,2)` 超时 2ms，逐事件 `recv(MSG_DONTWAIT)` 到共享的 `frame_` 后调 `rx_handler_(frame_)`。
  - **`close()`**（`:148-154`）：`socket_fd_==-1` 直接返回，否则 `epoll_ctl DEL`、`close(epoll_fd_)`、`close(socket_fd_)`。
  - **优点**：把"打开→收发→掉线重连"全封装成对象，上层只传一个回调函数；重连逻辑集中在守护线程。
  - **缺点/隐患**：
    1. **`quit_`、`ok_` 是普通 `bool`，被多个线程读写而无原子/锁**，是典型的数据竞争（UB）。`hikrobot.hpp` 里同类标志用的是 `std::atomic<bool>`，这里没跟上。
    2. **`close()` 没有把 `socket_fd_`、`epoll_fd_` 重置回 -1**，导致 `close()` 可能被多次调用时对已关闭的 fd 再次 `close`（`epoll_ctl DEL` 已关闭的 fd 报错、`close` 复用后的 fd 会误关别的资源）；`open()` 里 bind 失败路径也只 `close(socket_fd_)` 不重置。
    3. `frame_` 是单块共享缓冲，`read()` 在一个 epoll 循环里多次 `recv` 会覆盖上一帧，虽然当前 `rx_handler_` 同步消费、按值/常量引用传递无碍，但很脆弱。
    4. `read()` 里 `recv` 返回 -1 就抛异常终止整个读线程，由守护线程重启，代价偏高。
    5. `write()` 不检查 socket 是否打开，直接 `::write` 一个已关闭 fd 会抛 `EBADF`，靠调用方（`CBoard::send`）捕获。

---

### 文件: io/usbcamera/usbcamera.cpp / io/usbcamera.hpp

总括：基于 OpenCV V4L2 的 USB 相机类（**不继承 `CameraBase`**），带取流线程与守护线程。

- **`struct CameraData`**（`usbcamera.hpp:23-27`）：`cv::Mat img` + 时间戳，作为队列元素。

- **`class USBCamera`**（`usbcamera.hpp:13-45`）
  - **构造函数**（`usbcamera.cpp:12-59`）逐行说明：`:13` 初始化 `quit_=false/ok_=false/queue_(1)/open_count_=0`；`:15-21` 读 YAML 的 `image_width/height`(double)、`usb_exposure/frame_rate/gamma/gain`(int)；`:22` `try_open()`；`:25-58` 守护线程：100ms 循环，`ok_` 则 continue；`open_count_>20` 则放弃（`quit_=true`、加锁 `close()`、join 取流线程、break）；否则 join 取流线程、加锁 `close()`、`try_open()`。
  - **析构函数**（`:61-71`）：`quit_=true`、加锁 `close()`、join 两个线程。
  - **`read()`（返回 cv::Mat）**（`:73-82`）：加锁，`!isOpened` 返回空 Mat，否则 `cap_ >> img_` 返回。
  - **`read(cv::Mat&, timestamp)`**（`:84-91`）：从 `queue_` 阻塞 `pop` 一帧。
  - **`open()`**（`:93-154`）逐行说明：`:95` 加锁；`:96-101` 打开 `"/dev/"+open_name_`（`CAP_V4L`）；`:102` 读 `sharpness_`；`:103-107` 设 FOURCC=MJPG、FPS、`AUTO_EXPOSURE=1`、GAMMA、GAIN；`:109-119` **根据 `sharpness_==2` 命名为 "left"、`==3` 命名为 "right"**，并分别设宽高与 `EXPOSURE=usb_exposure_`；`:121-123` 打日志；`:127-153` 起取流线程：50ms 初始睡眠，1ms 循环，加锁 `cap_.read(img)`，成功后 push `{img, timestamp}`，失败则 break、`ok_=false`。
  - **`try_open()`**（`:156-164`）：`open()` 后 `open_count_++`，异常仅告警。
  - **`close()`**（`:166-172`）：`isOpened` 则 `release()`。
  - **优点**：取流线程 + 守护线程 + 队列解耦，掉线自动重开，且 USB 相机（哨兵多机位）正是靠它承载的。
  - **缺点/隐患**：
    1. **用 `CAP_PROP_SHARPNESS` 的返回值（2 或 3）来识别左右相机**，是脆弱且隐晦的魔法数字——换个摄像头固件、不同曝光下 sharpness 值可能变化，导致 `device_name` 不赋值（空串）或识别错误。
    2. `:105` 设了 `AUTO_EXPOSURE=1`（开启自动曝光），`:113/:118` 又设 `EXPOSURE=usb_exposure_`（手动曝光），两者冲突，实际行为取决于驱动。
    3. `open_count_` 只在 `try_open` 里自增、成功后**从不归零**，所以即使相机一直是好的，累计 open 20 次后守护线程也会永久放弃（`>20` 判定用的是累计而非连续失败次数）。
    4. `quit_/ok_/open_count_` 等普通 bool/int 跨线程读写，无原子/锁（`cap_` 的访问有 `cap_mutex_` 保护，但标志位没有）。
    5. `read()`（返回 Mat）与取流线程都在 `cap_mutex_` 保护下竞争同一个 `cap_`，若两者同时被调用会互相抢帧、丢帧；而两个 `read` 重载语义不同（一个直接读 cap、一个读队列），易误用。
    6. `queue_(1)` 只保留 1 帧，取流速度与消费速度不匹配时旧帧被覆盖，时间戳可能滞后于真实取流时刻。
    7. `device_name` 是 public 成员且不在构造初始化列表里，`sharpness` 非 2/3 时保持空串，日志里出现无意义的 " opened"。

---

### 文件: io/hikrobot/hikrobot.cpp / io/hikrobot.hpp

总括：海康机器人（MVS SDK）工业相机的驱动，继承 `CameraBase`，带取流线程 + 守护线程 + libusb 硬复位。

- **`class HikRobot : public CameraBase`**（`hikrobot.hpp:16-52`）
  - **构造函数**（`hikrobot.cpp:11-36`）逐行说明：`:12` `exposure_us_ = exposure_ms*1e3`（毫秒→微秒）、`gain_`、`queue_(1)`、`daemon_quit_=false`、`vid_=pid_=-1`；`:14` `set_vid_pid`；`:15` `libusb_init`；`:17-35` 守护线程：先 `capture_start()`，100ms 循环里 `capturing_` 为真就 continue，否则 `capture_stop()→reset_usb()→capture_start()`。
  - **析构函数**（`:38-43`）：`daemon_quit_=true`、join 守护线程。
  - **`read()`**（`:45-52`）：从队列 pop。
  - **`capture_start()`**（`:54-189`）逐行说明：`:56-57` `capturing_=false`、`capture_quit_=false`；`:62-71` `MV_CC_EnumDevices(MV_USB_DEVICE)`，无设备返回；`:73-77` `MV_CC_CreateHandle(device_list.pDeviceInfo[0])`；`:79-83` `MV_CC_OpenDevice`；`:85-90` 关闭白平衡自动/曝光自动/增益自动，设曝光与增益，`MV_CC_SetFrameRate(150)`；`:93-104` 读取并打印曝光/增益支持范围；`:106-110` `MV_CC_StartGrabbing`；`:112-188` 取流线程：`capturing_=true`，1ms 循环里 `MV_CC_GetImageBuffer(handle_, &raw, 100)`，`MV_E_NODATA` 累计到 50 次（≈5s）才断，其它错误直接断；`:151-177` 把原始 Bayer 图用 OpenCV `cvtColor` + `type_map`（BayerGR/RG/GB/BG8→RGB）转成 RGB 后入队；`:179-183` `MV_CC_FreeImageBuffer`。
  - **`capture_stop()`**（`:191-215`）：`capture_quit_=true`、join 取流线程，然后 `MV_CC_StopGrabbing/CloseDevice/DestroyHandle`。
  - **`set_float_value / set_enum_value`**（`:217-239`）：包装 `MV_CC_SetFloatValue/SetEnumValue`，失败告警。
  - **`set_vid_pid`**（`:241-258`）：解析 `"vid:pid"` 十六进制字符串。
  - **`reset_usb()`**（`:260-277`）：`libusb_open_device_with_vid_pid` + `libusb_reset_device` + `libusb_close`，对 USB 设备硬复位（注释引用了 ralight/usb-reset）。
  - **优点**：取流超时采用"连续 50 次 NODATA 才判掉线"的容错策略（`:120-149`），避免 USB/虚拟机直通下的偶发超时误判掉线；`capturing_/capture_quit_/daemon_quit_` 都是 `std::atomic<bool>`，线程安全做得比 socketcan/usbcamera 好。
  - **缺点/隐患**：
    1. **`type_map.at(pixel_type)`（`:174`）在相机输出非 Bayer8（例如直接 BGR8、Mono8、YUV 等）时会抛 `std::out_of_range`，而取流线程没有 try/catch，异常会导致 `std::terminate` 直接崩溃**。注释掉的 `MV_CC_ConvertPixelType` 路径（`:165`）说明作者曾在 SDK 转换与 OpenCV 转换之间摇摆，现版本的 `type_map` 覆盖不全。
    2. `handle_` 是 `void*`，在守护线程（capture_start/stop）与取流线程之间共享，虽然 `capture_stop` 先 join 了取流线程，但 `capture_start` 前半段（CreateHandle/OpenDevice）在守护线程里做、取流线程随后才起，若中途异常，句柄状态可能不一致。
    3. `MV_CC_EnumDevices` 后固定取 `pDeviceInfo[0]`（第一台），**完全不按 `vid_pid` 过滤**——`vid_pid` 只用于 USB 复位，多相机场景会选错设备。
    4. `libusb_init` 之后全程没有 `libusb_exit`，资源泄漏（进程级一般无害，但不规范）。
    5. `MV_CC_SetFrameRate(150)`、`100`ms 超时、`50` 次阈值等均为魔法数字。
    6. `raw.pBufAddr` 构造的 `cv::Mat`（`:152`）在 `MV_CC_FreeImageBuffer`（`:179`）之前就已经被 `cvtColor` 复制成 `dst_image`，因此没有悬垂指针问题，但这依赖 `cvtColor` 一定拷贝，逻辑上较隐晦。

---

### 文件: io/mindvision/mindvision.cpp / io/mindvision.hpp

总括：迈德威视（MVSDK）相机的驱动，继承 `CameraBase`，同样带取流线程 + 守护线程 + libusb 硬复位。

- **`class MindVision : public CameraBase`**（`mindvision.hpp:14-42`）
  - **构造函数**（`mindvision.cpp:13-42`）逐行说明：`:14-21` `exposure_ms_`、`gamma_`、`handle_=-1`、`quit_=false`、`ok_=false`、`queue_(1)`、`vid_=pid_=-1`；`:23` `set_vid_pid`；`:24` `libusb_init`；`:26` `try_open()`；`:29-41` 守护线程：100ms 循环，`ok_` 则 continue，否则 join 取流线程、`close()`、`reset_usb()`、`try_open()`。
  - **析构函数**（`:44-51`）：`quit_=true`、join 两线程、`close()`。
  - **`read()`**（`:53-60`）：队列 pop。
  - **`open()`**（`:62-116`）逐行说明：`:64-68` `CameraSdkInit(1)`、`CameraEnumerateDevice`（`camera_num` 初值 1 即最多枚举 1 台）；`:70` 无相机抛异常；`:72-73` `CameraInit(&camera_info_list, -1, -1, &handle_)` 失败抛异常；`:75-77` `CameraGetCapability` 取最大分辨率 `width_/height_`；`:79-84` 关自动曝光、设曝光（`exposure_ms_*1e3` 微秒）、设伽马（`gamma_*1e2`）、输出 BGR8、连续触发、低帧率模式；`:86` `CameraPlay`；`:89-113` 取流线程：`ok_=true`，1ms 循环，每次 `new` 一个 `cv::Mat(height_,width_,CV_8UC3)`，`CameraGetImageBuffer(handle_,&head,&raw,100)`，失败告警 "Camera dropped!" 并 `ok_=false` break，成功则 `CameraImageProcess` 转 BGR 后 `CameraReleaseImageBuffer`、入队。
  - **`try_open()`**（`:118-125`）：`open()` 异常仅告警。
  - **`close()`**（`:127-131`）：`handle_==-1` 返回，否则 `CameraUnInit(handle_)`。
  - **`set_vid_pid / reset_usb`**（`:133-169`）：与 HikRobot 完全相同的实现。
  - **优点**：异常通过抛 `runtime_error` 由 `try_open` 兜住，初始化失败不至于崩溃；守护线程自动重连。
  - **缺点/隐患**：
    1. `quit_/ok_` 是普通 `bool` 跨线程读写，数据竞争（海康版用了 `atomic`，这里没跟上）。
    2. **`close()` 后 `handle_` 不重置为 -1**，重复 `close()` 会对已 uninit 的句柄再次 `CameraUnInit`，是双重释放/未定义行为隐患。
    3. 取流线程里**每 1ms 就 `new` 一个 `cv::Mat`**（`:97`），内存分配/释放极其频繁，是本层最明显的性能隐患（应复用缓冲）。
    4. `CameraSdkInit`/`CameraEnumerateDevice`/`CameraGetCapability`/`CameraPlay` 等的返回值大多未检查（只检查了 `CameraInit` 和无相机），SDK 调用失败会静默继续。
    5. `camera_num` 初值 1，多台迈德威视相机时只能枚举到 1 台；同样不按 `vid_pid` 选设备。
    6. `libusb_init` 后无 `libusb_exit`；`gamma_*1e2`、`exposure_ms_*1e3` 等换算系数是魔法数字。

---

### 文件: io/dm_imu/dm_imu.cpp / io/dm_imu.hpp

总括：DM 惯性测量单元（IMU）的串口驱动，读 `0x55 0xAA 0x01 0x01` 帧头的三子帧数据（加速度/角速度/欧拉角），转成四元数供上层查询。

- **`struct IMU_Receive_Frame`**（`dm_imu.hpp:19-50`）
  - **作用**：packed 结构体描述 57 字节接收帧，由 3 个 19 字节子帧组成。
  - **逐行说明**：每个子帧为 `FrameHeader(1)+flag(1)+slave_id(1)+reg(1)+3×uint32(12)+crc(2)+FrameEnd(1)=19` 字节；三个子帧分别对应加速度、角速度、欧拉角。
  - **优点**：packed 布局精确匹配线协议，直接 `memcpy` 解析。
  - **缺点/隐患**：`uint32_t` 字段里实际装的是 float（后续用 `reinterpret_cast` 强转），依赖小端与 IEEE754；`FrameEnd` 字节（0x66 之类）未校验。

- **`typedef struct IMU_Data`**（`:52-63`）：9 个 float 的解析结果（accx..yaw）。

- **`class DM_IMU`**（`:65-92`，`dm_imu.cpp`）
  - **构造函数**（`dm_imu.cpp:17-24`）：`:17` `queue_(5000)`；`:19` `init_serial()`；`:20` 起接收线程；`:21-22` 阻塞 pop 两个样本填充 `data_ahead_/data_behind_`；`:23` 打日志。
  - **析构函数**（`:26-35`）：`stop_thread_=true`、join、关串口。
  - **`init_serial()`**（`:37-58`）逐行说明：`:40` **`setPort("/dev/ttyACM0")` 硬编码**；`:41` 波特率 921600；`:42-46` 无流控、无校验、1 停止位、8 数据位、`simpleTimeout(20)`；`:48` `open()`；`:49` `usleep(1000000)` 1 秒；`:54-57` 捕获 `IOException` 后告警并 **`exit(0)` 直接终止整个进程**。
    - **优点**：串口参数齐全。
    - **缺点/隐患**：**设备路径与波特率硬编码，且打开失败直接 `exit(0)`**——作为库内组件，一个 IMU 掉线就杀掉整个视觉进程，是严重缺陷；`usleep(1s)` 的固定等待也是魔法延迟。
  - **`get_imu_data_thread()`**（`:60-104`）逐行说明：`:63-65` 未打开则告警；`:67` `read(4)` 读帧头前 4 字节；`:69-73` 校验 `0x55 0xAA 0x01 0x01`；`:74` 匹配则再读剩余 53 字节（`57-4`）；`:76-85` 三处分别用 `tools::get_crc16` 校验三个子帧，通过才解析对应 float；`:94-99` 欧拉角（度→弧度）转四元数（ZYX）归一化后入队；`:100-102` 不匹配则打 info 日志。
    - **优点**：三个子帧独立 CRC，坏一子帧不影响其它。
    - **缺点/隐患**：
      1. **`serial_.read` 返回值完全不检查**——`simpleTimeout(20)` 的读可能只返回 0~3 字节，代码仍按 4 字节对齐去匹配帧头，一旦流偏移 1~3 字节就**永远无法重新同步**（因为它每次只尝试 4 字节对齐的位置，不做字节级滑动）。
      2. 帧头不匹配时打 **`info` 级日志，每次循环都打**，在 921600 波特率下会疯狂刷屏日志。
      3. `M_PI`（`:95-97`）依赖 `<math.h>` 且非 C++ 标准宏，严格编译下可能未定义。
  - **`imu_at(timestamp)`**（`:106-129`）：与 `CBoard::imu_at` 完全相同的旧版 slerp 算法。
    - **缺点/隐患**：**同样的 `t_ab` 除零风险、无 `k` clamp、无重复时间戳防护**——即 `gimbal.cpp` 注释里已修复、但这里仍然存在的插值 bug；`data_ahead_/data_behind_` 非线程安全。

---

### 文件: io/gimbal/gimbal.cpp / io/gimbal.hpp

总括：基于 USB CDC 虚拟串口的云台驱动，`'S''P'` 帧头 + CRC16 协议，是本 io 层注释最详尽、鲁棒性最高的一份实现。

- **`struct GimbalToVision`**（`gimbal.hpp:21-33`）
  - **作用**：电控→视觉的 packed 帧：`head[2]='SP'`、`mode`、`q[4]`（wxyz 四元数）、`yaw/yaw_vel/pitch/pitch_vel/bullet_speed`（float）、`bullet_count`（uint16）、`crc16`。
  - **逐行说明**：`:35` `static_assert(sizeof<=64)` 兜底；实际 43 字节。
  - **优点**：`static_assert` 保证帧不超预算；字段注释清晰。

- **`struct VisionToGimbal`**（`:37-48`）
  - **作用**：视觉→电控帧：`head/mode/yaw/yaw_vel/yaw_acc/pitch/pitch_vel/pitch_acc/crc16`，33 字节，`:50` static_assert。
  - **逐行说明**：`:39` `mode` 语义为"0 不控制、1 控制不开火、2 控制且开火"。

- **`enum class GimbalMode / struct GimbalState`**（`:52-68`）：模式枚举与状态结构（yaw/yaw_vel/pitch/pitch_vel/bullet_speed/bullet_count）。

- **`class Gimbal`**（`:70-139`，`gimbal.cpp`）
  - **构造函数**（`gimbal.cpp:14-48`）逐行说明：`:15` `last_q_` 初始化为单位四元数；`:17-18` 读 `com_port`；`:21-25` 读 `crc_check`（缺省 true，可关闭）；`:27-36` 打开串口 + `configure_port()`，失败仅 error 日志不抛（读线程会持续重试）；`:38` 起 `read_thread`；`:42-47` 最多等 3s 首帧（300×10ms），超时告警不阻塞。
  - **析构函数**（`:50-55`）：`quit_=true`、join、`serial_.close()`。
  - **`mode()/state()`**（`:57-67`）：`mutex_` 加锁返回。
  - **`configure_port()`**（`:69-79`）：`setBaudrate(115200)`、`setTimeout(port_timeout_)`。注释（`:71-76`）详细解释了重枚举后 termios 恢复默认（ICANON/ISTRIP/IXON/OPOST）会毁坏二进制帧、必须重发配置。
  - **`write_frame()`**（`:81-96`）：未打开直接丢弃；`serial_.write(tx_data_)`，异常按 1s 限频告警。
  - **`str(GimbalMode)`**（`:98-112`）：枚举转字符串。
  - **`q(timestamp)`**（`:114-163`）逐行说明：`:120-123` `try_pop` 拿样本 a，队列空则加锁返回 `last_q_` 四元数；`:128-134` `try_pop` 样本 b，只剩一帧则存 `last_q_` 返回 a；`:141` `t_ab = delta_time(t_b, t_a)`；`:142-145` `t_ab<=0` 时丢弃旧帧继续（防除零 NaN）；`:147-151` `k = clamp(t_ac/t_ab, 0, 1)`，绝不外推；`:152` slerp；`:154-158` `t<=t_b` 时存 `last_q_=sample_b` 返回；`:160-161` 否则推进 sample_a。
    - **优点**：这是全层唯一把"时间戳除零、外推放大角速度、参数写反"都处理掉的插值实现，注释（`:138-150`）完整记录了 2026-10-12 联调踩坑过程，是质量标杆。
    - **缺点/隐患**：`last_q_` 仅在队列空/单帧分支被赋值，且赋值在 `mutex_` 保护下；但 `q()` 本身若被多线程并发调用，`sample_a/sample_b` 是局部变量、`last_q_` 有锁，基本安全，只是队列消费存在多消费者竞争语义（谁 pop 到算谁的）。
  - **`send(Command)`**（`:165-169`）：转发到 8 参数版，速度/加速度前馈置零。
  - **`send(VisionToGimbal)`**（`:171-192`）逐行说明：`:173` `tx_mutex_` 加锁；`:176-179` NaN/Inf 防护，非有限值则清零发停止帧；`:181-189` 填 `tx_data_` 并算 CRC16（`tools::get_crc16`）；`:191` `write_frame()`。
  - **`send(8参数)`**（`:194-219`）：`:198` 加锁；`:201-206` NaN 防护；`:208` `mode = control ? (fire?2:1) : 0`；`:209-218` 填字段、算 CRC、写帧。
    - **优点**：**NaN/Inf 防护**避免插值异常时把垃圾角度发给电控（相比 `CBoard` 无此防护，是明显改进）；`tx_mutex_` 保护多线程发送。
  - **`read_thread()`**（`:221-383`）逐行说明：`:226-228` 帧同步状态机（LOOK_FOR_S/LOOK_FOR_P/READ_BODY）；`:233` 4096 字节批量读缓冲（注释解释逐字节 read 只能 ~200B/s、跟不上 6kB/s 会丢字节）；`:235-236` 连续读错误与空闲计数；`:238-241` `reset_sync` lambda；`:244-322` `accept_frame` lambda：`:245-248` 重构帧（head + body），`:250-251` 取时间与四元数，`:256-277` 1Hz 解码日志（含原始 43 字节 hex 转储），`:279-291` CRC 校验或（关闭 CRC 时）单位四元数模长校验，`:293` 入队，`:295-321` 加锁更新 `state_/mode_`；`:324-380` 主循环：`serial_.read` 异常累计 20 次触发 `reconnect()`，`n==0` 累计 30 次（≈3s）触发重连，否则逐字节喂状态机。
  - **`record_crc / crc_rate`**（`:385-408`）：滑动窗口 CRC 统计，`crc_total_/crc_good_` 用 `std::atomic<uint64_t>`，`crc_history_` 用 `crc_mutex_` 保护（**比 `CBoard1` 的裸 deque 安全**）。
  - **`reconnect()`**（`:410-439`）：循环 close→open→configure_port→clear 队列，前 10 次 500ms 间隔、之后 2s，直到成功或退出。
  - **优点**：帧同步状态机、批量读、连续错误/空闲双重掉线判定、无上限重连、重开后重发 termios 配置、NaN 防护、CRC 统计加锁——**这是全 io 层最完整、最接近生产质量的设备驱动**，大量注释记录了真实踩坑与修复根因。
  - **缺点/隐患**：
    1. 帧同步只找 `'S''P'` 两字节，若 `q[4]` 等 float 数据里恰好出现 `0x53 0x50`（'S''P'）会假同步，靠后续 CRC 纠正（可接受但会丢帧）。
    2. `accept_frame` 里即使 CRC 失败也会先打印 1Hz 日志与 43 字节 hex 转储（`:256-277`），高干扰场景下日志量偏大。
    3. `mode_/state_` 与 `queue_` 分别用不同锁/机制保护，`mode()` 读到的模式与 `q()` 拿到的姿态并非同一时刻原子一致（跨多个对象读取时可能短暂不一致）。
    4. `port_timeout_` 等超时参数为经验魔法数字（注释有解释，但仍是调参得来的硬编码）。

---

### 文件: io/ros2/ros2.cpp / io/ros2/ros2.hpp

总括：ROS2 门面类，负责初始化 rclcpp、创建发布/订阅节点并各起一个 spin 线程。

- **`class ROS2`**（`ros2.hpp:9-42`，`ros2.cpp`）
  - **构造函数**（`ros2.cpp:4-15`）逐行说明：`:6` `rclcpp::init(0, nullptr)`；`:8` `make_shared<Publish2Nav>()`；`:10` `make_shared<Subscribe2Nav>()`；`:12` 起发布 spin 线程 `publish2nav_->start()`；`:14` 起订阅 spin 线程 `subscribe2nav_->start()`。
  - **析构函数**（`:17-22`）：`rclcpp::shutdown()` 后 join 两个线程。
  - **`publish(target_pos)`**（`:24`）：转发给 `publish2nav_->send_data`。
  - **`subscribe_enemy_status()/subscribe_autoaim_target()`**（`:26-34`）：转发给订阅节点。
  - **`create_publisher<T>()` 模板**（`ros2.hpp:22-34`）逐行说明：`:26` 新建一个 `rclcpp::Node`；`:28` 建 publisher；`:31` `std::thread([node]{ rclcpp::spin(node); }).detach()` 起分离线程。
  - **优点**：把 ROS2 的初始化、多节点 spin 全部隐藏在一个对象里，上层只调 `publish/subscribe_*`。
  - **缺点/隐患**：
    1. **`create_publisher` 用 `detach()` 的裸线程 spin 一个匿名节点**，该线程在程序退出时不会被 join，`rclcpp::shutdown()` 后分离线程仍在 spin 会导致析构竞态/崩溃风险，且每个 publisher 都新建独立节点，语义上浪费。
    2. `rclcpp::init(0, nullptr)` 没有传 domain id/选项，且若进程内其它模块也调用 `rclcpp::init` 会抛异常（不能重复 init）。
    3. 析构顺序 `shutdown()` 在 `join()` 之前，若 spin 尚未返回，join 会阻塞到 spin 退出，通常可行但不够严谨。

---

### 文件: io/ros2/publish2nav.cpp / io/ros2/publish2nav.hpp

总括：向导航发布"自瞄目标位置"的 ROS2 节点，消息类型为 `std_msgs::msg::String`。

- **`class Publish2Nav : public rclcpp::Node`**（`publish2nav.hpp:17-31`）
  - **构造函数**（`publish2nav.cpp:13-18`）逐行说明：`:13` 节点名 `auto_aim_target_pos_publisher`；`:15` 创建 `std_msgs::msg::String` 发布者，topic `auto_aim_target_pos`，队列 10。
  - **`send_data(const Eigen::Vector4d&)`**（`:25-40`）逐行说明：`:28` 新建 String 消息；`:31-32` 把 4 个 double 用逗号拼成字符串（`x,y,z,?`）；`:35` 发布。
    - **缺点/隐患**：**用 String 消息以逗号分隔字符串传输 4 个浮点数，类型不安全、无 schema、接收端要靠字符串解析**，是典型的设计缺陷（应使用自定义消息或 `Float64MultiArray`）；每次调用 `std::make_shared` 新建消息对象，高频下分配开销可优化。
  - **`start()`**（`:42-46`）：`rclcpp::spin(shared_from_this())` 阻塞 spin。
  - **优点**：结构简单，publisher 与 spin 分离（spin 在独立线程，`send_data` 可被主线程随时调用，ROS2 publisher 线程安全）。

---

### 文件: io/ros2/subscribe2nav.cpp / io/ros2/subscribe2nav.hpp

总括：订阅导航发来的敌方状态与自瞄目标消息的 ROS2 节点，用带超时清空的队列缓存最新消息。

- **`class Subscribe2Nav : public rclcpp::Node`**（`subscribe2nav.hpp:15-42`）
  - **构造函数**（`subscribe2nav.cpp:9-25`）逐行说明：`:10` 节点名 `nav_subscriber`；`:11-12` 两个队列容量 1；`:13-14` 计数器清零；`:16-18` 订阅 `enemy_status`（`EnemyStatusMsg`）；`:20-22` 订阅 `autoaim_target`（`AutoaimTargetMsg`）。
  - **`enemy_status_callback`**（`:32-50`）逐行说明：`:34-35` 清空队列并 push 最新；`:37` 计数器++；`:39-49` 计数器 ≥2 时（取消并）重建一个 1500ms 的 wall timer，到点清空队列、计数器归零。
  - **`autoaim_target_callback`**（`:52-70`）：与上完全对称。
  - **`start()`**（`:72-76`）：spin。
  - **`subscribe_enemy_status()`**（`:78-91`）逐行说明：`:80-82` 队列空返回空 `vector<int8_t>`；`:85` `back(msg)` 取最新消息；`:90` 返回 `invincible_enemy_ids`。
  - **`subscribe_autoaim_target()`**（`:93-106`）：同构，返回 `target_ids`。
  - **优点**：队列只保留最新一条、带 1.5s 超时清空，避免消费端拿到过期目标。
  - **缺点/隐患**：
    1. **watchdog 实现有误**：`counter_` 只在回调里自增、只在 timer 到期才归零，因此只要消息持续到达，`counter_` 会无界增长（`int` 终将溢出），且**每个回调都 cancel+create 一个 timer**（`:40-48`），消息频繁时反复创建/销毁 timer，存在 churn 与潜在泄漏；注释说的"after two messages"与实际"从第 2 条起每条都重建 timer"不符。
    2. 变量名 `enemy_statue_queue_`/`enemy_statue` 拼写错误（statue→status），蔓延到多个成员。
    3. `subscribe2nav.hpp:6` 直接 `#include <sp_msgs/msg/detail/autoaim_target_msg__struct.hpp>`，引用了 ROS 消息的 **detail 内部结构头**，属于脆弱实现，消息包升级后可能失效。
    4. `back(msg)` 依赖 `ThreadSafeQueue::back` 的拷贝语义（只读最新不弹出），若实现有误会阻塞或返回旧数据。
    5. 返回 `vector<int8_t>` 的 id 用 int8 存储，id 范围 0~255 会溢出（int8 最大 127）。

---

### 文件: io/serial/CMakeLists.txt

总括：第三方串口库 `serial`（wjwwood/serial，vendored 进 `io/serial/`）的构建脚本，被 `io/CMakeLists.txt:5` 通过 `add_subdirectory(serial)` 引入。

- **`cmake_minimum_required / project(serial)`**（`:1-2`）：声明为独立 `serial` 工程。
- **平台条件分支**（`:4-13`）：Apple 下查找 IOKit/Foundation；`UNIX AND NOT APPLE`（Linux）下设置 `rt`、`pthread` 链接库变量。
- **`serial_SRCS` 源列表**（`:16-20`）：`src/serial.cc`、`serial.h`、`v8stdint.h`。
- **平台实现源**（`:21-33`）：OSX 用 `unix.cc` + `list_ports_osx.cc`；其它 UNIX 用 `unix.cc` + `list_ports_linux.cc`；Windows 用 `win.cc` + `list_ports_win.cc`。
- **`add_library` 与链接库**（`:36-43`）：建 `serial` 库，按平台链接 IOKit/Foundation、rt pthread、或 setupapi。
- **`target_include_directories(… PUBLIC include)`**（`:46`）：对外暴露 `include`，这样上层 `#include <serial/serial.h>` 可用。
- **优点**：跨平台分支完整，作为 vendored 依赖与主工程解耦清晰。
- **缺点/隐患**：`project(serial)` 未设版本号/语言；Linux 分支用 `UNIX AND NOT APPLE` 判断，BSD/其它 Unix 也会被当作 Linux 处理（一般无害）；`add_library` 未指定 STATIC/SHARED，由上层默认决定。

---

## IO 层整体优缺点总结

**架构优点**

1. **生产者-消费者解耦**：所有设备驱动都遵循"后台线程取数 → 线程安全队列 → 上层按需消费"的模式（`ThreadSafeQueue`），相机/IMU/串口的阻塞 IO 不会卡住上层主循环。
2. **统一的设备抽象接口**：`CameraBase` + `Camera` 工厂把海康/迈德威视收口成一套 `read`；`Command` 结构体 + `send(Command)`/`q(t)`/`imu_at(t)` 让 CAN、UART、CDC 三套电控后端对上层暴露近似一致的接口。
3. **自愈能力**：海康/迈德威视/USB 相机的守护线程、`SocketCAN` 的重连线程、`Gimbal` 的无上限 `reconnect()`，都体现了"设备掉线后自动恢复、无需人工重启"的工程追求；海康相机的"NODATA 连续 50 次才判掉线"、Gimbal 的 termios 重枚举修复，都是真实的鲁棒性投入。
4. **`gimbal.cpp` 是质量标杆**：帧同步状态机、批量读、NaN/Inf 防护、插值 clamp、CRC 统计加锁、详尽踩坑注释，代表了本层应达到的水准。

**共性问题**

1. **线程安全不统一**：`HikRobot`、`Gimbal` 用了 `std::atomic`/`std::mutex`，而 `SocketCAN`、`USBCamera`、`MindVision`、`CBoard` 的 `quit_/ok_` 等仍是裸 `bool` 跨线程读写；`CBoard` 的 `mode/bullet_speed`、`CBoard1::crc_history_` 无锁。存在多处数据竞争（UB）。
2. **资源句柄不复位**：`SocketCAN::close()`、`MindVision::close()` 关闭后不把 fd/handle 置回无效值，重复 close 有双重释放/误关风险；`libusb_init` 无配对 `libusb_exit`。
3. **三套协议并存、CRC 分裂**：CAN（`CBoard`）、`0x5A/0xA5` UART（`CBoard1`）、`'SP'` CDC（`Gimbal`）三套后端并存，且存在 `protocol.cpp`（0x1021）、`vision_protocol.hpp`（0xA001）、`tools::get_crc16` 三个同名/近名 CRC 实现，`protocol.hpp` 注释甚至与实现矛盾，极易误用导致"CRC 永远对不上"。
4. **硬编码与魔法数字普遍**：`DM_IMU` 硬编码 `/dev/ttyACM0` + 921600 + `exit(0)`；`CBoard1::parseBuffer` 硬编码 `mode = Mode::auto_aim`；`HikRobot::SetFrameRate(150)`；`USBCamera` 用 `CAP_PROP_SHARPNESS==2/3` 识别左右相机；`Command::r1=r2=0.12` 物理参数写死在默认值里。
5. **返回值/异常检查缺失**：`DM_IMU` 的 `serial_.read` 不检查字节数、无重同步；`MindVision` 多数 SDK 调用不检查返回值；`CBoard1::send(8参数)` 让 `target_*` 全零发出。
6. **死代码与未收敛的历史版本**：`vision_protocol.hpp` 全仓库无人引用；`CBoard::imu_at`/`DM_IMU::imu_at` 仍保留 `Gimbal::q` 已修复的插值除零/外推 bug；`USBCamera` 未接入 `Camera` 工厂。
7. **兼容接口名不副实**：`CBoard1::imu_at` 实际不做插值、`CBoard1::state()` 大量字段恒为 0，却用与 `Gimbal` 相同的接口名，跨后端切换时存在静默语义差异。


---

# Auto Aim 任务层分析

## 模块概述

`tasks/auto_aim/` 是 sp_vision_25 视觉系统的自动瞄准（自瞄）任务层，负责把相机采集的彩色图像一路加工成发给下位机（电控/云台）的 yaw/pitch 控制指令与开火信号。整个管线可以概括为：**取流 → 检测 → 分类 → 跟踪 → 解算 → 发射 → 弹道补偿 → 云台控制**。

**数据流的起点是图像。** 相机线程通过 `MultiThreadDetector::push(cv::Mat img, steady_clock::time_point t)` 把帧和其采集时间戳压入一个基于 `ThreadSafeQueue` 的槽位池。`MultiThreadDetector` 是 OpenVINO 推理的生产者/消费者多线程封装：它预先建好 `num_streams` 个 `InferRequest` 与各自的输入缓冲，`push` 时挑一个空闲槽位做 letterbox 缩放后 `start_async()` 异步提交推理，消费线程 `pop()` 按 FIFO 顺序取已完成推理的帧做后处理。之所以绕开简单的同步 `detect()`，是因为注释里明确记录了实测结论：每帧 `create_infer_request()` 走的是 OpenVINO "单请求延迟"路径（44~57ms/帧），而预建多流复用走 "多流吞吐"路径（21~26ms/帧），这是帧率上不去的根本原因。

**检测环节由 YOLO 家族承担。** 抽象基类 `YOLOBase` 定义 `detect` 与 `postprocess` 两个纯虚接口，工厂类 `YOLO` 依据配置里的 `yolo_name` 动态实例化 `YOLOV5`、`YOLOV8` 或 `YOLO11`。三个实现共享同一条后处理主骨架（letterbox 缩放到 640/416、OpenVINO `PrePostProcessor` 做 BGR→RGB 与 1/255 归一化、NMS、关键点排序），区别在于输出头的列布局：YOLOv5 是 "4 关键点 xywh+颜色独热(4)+类别独热(9)" 共 22 列，YOLOv8 是 "xywh+2 类+4 关键点" 共 14 列，YOLO11 是 "xywh+38 类+4 关键点" 共 50 列。检测结果统一收敛为 `std::list<Armor>`。

**Armor 是贯穿全管线的核心结构体。** 它同时承载像素域的 `points`（四个装甲板角点）、`box`、`pattern`（数字图案 ROI）、`confidence`、`color/name/type/priority`，以及解算后的三维量 `xyz_in_gimbal/xyz_in_world`（米）、`ypr_in_gimbal/ypr_in_world`（弧度）、`ypd_in_world`（球坐标 yaw/pitch/distance）。传统检测路径（`Detector`）与神经网络路径（三个 YOLO 实现）都会产出 `Armor`，差别在于前者靠灯条配对 + 分类器数字识别，后者靠关键点回归 + 类别向量。

**传统 `Detector` 是神经网络检测的兜底/二次矫正工具。** 它基于全局灰度阈值 `threshold` 二值化 → `findContours` 找轮廓 → `minAreaRect` 拟合旋转矩形得灯条 → 灯条几何校验（角度误差、长宽比、长度）→ 按颜色配对成装甲板 → 几何校验 → `get_pattern` 裁数字图案 → `Classifier` 识别编号 → 类型/名称校验 → 输出。此外它还提供一个 `detect(Armor&, img)` 重载，供 YOLOv5 在神经网络角点基础上用传统灯条做角点二次矫正（`use_traditional_` 开关）。

**分类器 `Classifier` 负责数字（兵种编号）识别。** 它把装甲板图案灰度化后等比缩放到 32×32（保持宽高比、居中、不足补 0），输入一个 9 类的轻量卷积网络（convnet），输出经 softmax 归一化后取最大类作为 `armor.name`，同时回填 `armor.confidence`。类里同时保留了 `cv::dnn`（`classify`）和 OpenVINO（`ovclassify`）两套推理后端，当前主路径走 `cv::dnn`。识别出的 `name` 与 `type` 会被 `check_name`/`check_type` 校验，异常样本被 `save` 落盘到 `patterns/` 目录用于分类器迭代。

**跟踪由 `Tracker` + `Target` + `Solver` 三件套完成。** `Tracker` 维护一个 `lost → detecting → tracking → temp_lost/switching` 的状态机，先按颜色过滤敌方装甲板，再按"靠近图像中心 + 优先级"排序，在 `lost` 态用 `set_target` 新建目标（对哨兵/前哨站/基地/平衡步兵用不同初值），在跟踪态用 `update_target` 按名称/类型匹配后逐帧更新。`Solver` 负责把 `Armor` 的四个像素角点通过 `solvePnP(IPPE)` 反解出三维位姿，并在 `optimize_yaw` 里用重投影搜索修正 yaw。`Target` 则持有 11 维 EKF 状态（中心 xyz、速度、公转角度 a、角速度 w、半径 r、长板半径增量 l、高度增量 h），用 `predict/update` 维持对目标运动模型的估计，并输出每块装甲板的世界坐标 `armor_xyza_list()`。

**瞄准与发射由 `Aimer` + `Shooter` 完成。** `Aimer::aim` 先按目标角速度选高/低速延迟，弹速无效时用配置默认值兜底，`to_now` 模式下把帧龄 + 延迟（钳制在 `max_predict_time_` 内）计入预测时域；再通过 `choose_aim_point` 选板（普通/小陀螺/前哨站三套逻辑，带锁板滞回与 spin 滞回），随后用 `tools::Trajectory`（忽略空气阻力的抛物线模型）迭代求飞行时间，最终输出 yaw/pitch，并做近距离相机光轴横向偏置视差补偿。`Shooter` 依据云台当前位置与上一帧指令的偏差、以及 `aimer.debug_aim_point.valid` 做两级容差判定决定是否开火。

**弹道补偿与角度输出。** 弹道模型在 `tools::Trajectory`（见 tools/trajectory.cpp）：给定初速 v0、水平距离 d、高度 h，解一元二次方程 `a·tan²θ - d·tanθ + (a+h) = 0`（a = g·d²/(2v0²)），选飞行时间较小的解得到 pitch，飞行时间 `t = d/(v0·cosθ)`。`Aimer` 把弹道 pitch 与配置的 `pitch_offset_` 合并（世界系 pitch 向上为负），yaw 则由 `atan2(y,x)` 叠加视差补偿与 `yaw_offset_`。

**云台控制与指令下发的最后一段是 `multithread::CommandGener`。** 它起一个独立线程，`push` 只负责用互斥锁更新 `latest_` 输入并通知条件变量；消费线程以 ~500Hz 循环，取最新一帧（帧龄超过 0.35s 则视为过期），调用 `aimer_.aim(... to_now=true)` 生成指令、`shooter_.shoot` 决定开火，再 `board.send(command)` 下发；无新帧或帧龄超限时发 `{false,false,0,0}` 停止帧，避免电控执行过期指令导致云台抽搐。

**`planner/` 是一条并行演进的 MPC 控制路径。** 它与 `Aimer` 类似输出云台指令，但用 `TinyMPC` 求解器生成整条 yaw/pitch 参考轨迹：先 `aim` 求各时刻的期望角，`get_trajectory` 在 ±0.5s 时域上滚动预测得到 `Trajectory`（4×100 矩阵：yaw/yaw_vel/pitch/pitch_vel），再用两个 2 状态（角+角速度）线性二次 MPC（状态转移 `[[1,DT],[0,1]]`，输入为角加速度，带 `±max_*_acc` 输入约束）追踪这条轨迹，取时域中点处的状态/输入作为下发值。它把云台动力学约束（加速度上限）显式纳入优化，是相对 `Aimer` 的"开环+迭代弹道"更现代的控制方案。

**OpenVINO 在整条管线里的角色是推理加速器。** 三个 YOLO 模型、分类器、多线程检测器都通过 `ov::Core::read_model` + `PrePostProcessor` + `compile_model` 加载 IR/ONNX 模型，用 `InferRequest` 做异步或同步推理。预处理统一用 PPP 在推理图里完成 BGR→RGB、类型转换、1/255 归一化，推理后端可配置 CPU 或其它 device，性能模式可选 LATENCY 或 THROUGHPUT。它在 `MultiThreadDetector` 里的多流吞吐模式是整个系统实时性的关键。

**帧与结构体的流转总结。** 一条典型的链路是：相机帧 → `MultiThreadDetector::push`（存入 `Slot.img/t`）→ 异步推理 → `pop()` 得 `(std::list<Armor>, t)` → `Tracker::track`（过滤/排序/状态机）→ `std::list<Target>`（含 EKF）→ `CommandGener::push` → `Aimer::aim` → `io::Command{yaw,pitch,shoot,horizon_distance}` → `Shooter::shoot` 打 `shoot` 位 → `board.send`。其中 `Armor` 是"帧级观测"，`Target` 是"跨帧状态"，`io::Command` 是"对下位机的输出"，三者粒度由细到粗，层层抽象。

---

### 文件: tasks/auto_aim/armor.hpp / armor.cpp

**总括：** 定义装甲板领域模型——颜色/类型/编号/优先级四组枚举、`Lightbar`（灯条）与 `Armor`（装甲板）两个核心结构体，以及把不同检测源（传统灯条配对、YOLOv8/11 关键点、YOLOv5 关键点）统一转成 `Armor` 的多套构造函数。

**枚举 Color / ArmorType / ArmorName / ArmorPriority（armor.hpp:11-49）**
- **作用：** 定义装甲板的颜色（红/蓝/熄灭/紫）、类型（大/小）、编号（一~五号、哨兵、前哨站、基地、非装甲）、优先级（first~fifth）。
- **说明：** 每个枚举都配了一个对应的 `std::vector<std::string>`（`COLORS`、`ARMOR_TYPES`、`ARMOR_NAMES`）用于日志与调试输出。`Color::purple` 只用于基地大/小装甲的配色枚举，`extinguish`（熄灭）表示无颜色信息。
- **隐患：** 枚举值强依赖顺序（`COLORS[color]` 直接按下标索引），一旦有人在中途插入新颜色，所有序列化和 `Voter::index` 的三维下标计算都会错位。

**armor_properties 表（armor.hpp:52-64）**
- **作用：** 一个 `std::vector<std::tuple<Color, ArmorName, ArmorType>>`，把"类别 id → 颜色/编号/类型"的映射固化在代码里，共 36 项（含大/小三号、四号、五号，大/小基地，紫基地等）。
- **说明：** YOLOv8/11 的神经网络构造函数直接以 `class_id` 索引这张表得到 `color/name/type`。表内存在同一 (color,name) 对应两种 type 的情形（如基地大/小），因此单靠 class_id 无法区分大小基地——这是 `get_type` 里按 `name` 再判一次的原因之一。
- **隐患：** class_id 越界时构造函数回退到默认 `{blue, not_armor, small}`（armor.cpp:92-96），静默降级而非报错，可能掩盖模型输出维度与表不一致的问题。

**struct Lightbar（armor.hpp:67-78）**
- **作用：** 一条灯条的几何描述。成员含 `id`（灯条全局序号）、`color`、`center/top/bottom/top2bottom`（几何端点）、`points`、`angle/angle_error/length/width/ratio`、`rotated_rect`（原始旋转矩形）。
- **说明：** `top/bottom` 是灯条两端中点，`top2bottom = bottom - top` 是方向向量，`ratio = length/width` 用于长宽比几何校验。

**Lightbar::Lightbar(const RotatedRect&, size_t id)（armor.cpp:9-31）**
- **作用：** 传统灯条构造函数，从旋转矩形提取几何量。
- **逐段说明：** `rotated_rect.points(&corners[0])` 得四角点后按 y 坐标排序（:14-16），则 `corners[0],corners[1]` 是上端两角、`corners[2],corners[3]` 是下端两角（:19-20）；`top/bottom` 取两端中点（:19-20），`width` 用上端两角距离（:26），`angle = atan2(top2bottom.y, top2bottom.x)`（:27），`angle_error = |angle - π/2|`（:28）衡量灯条偏离竖直的程度。
- **优点：** 构造简洁，直接为几何校验提供量。
- **隐患：** 按 y 排序而非按"灯条主轴"排序，当灯条接近水平或画面翻转时 `top/bottom` 语义会错乱；`angle` 由 `atan2` 得到范围在 [-π,π]，`angle_error` 用绝对值处理，对接近 ±π 的跳变（-179° 与 179°）没有归一化，可能误判。

**Armor::Armor(const Lightbar&, const Lightbar&)（armor.cpp:34-56，传统灯条配对构造函数）**
- **作用：** 由左右两条灯条拼出装甲板，计算角点与几何校验量。
- **逐段说明：** `center` 取两灯条中心均值（:38，注意注释 "不是对角线交点，不能作为实际中心"）；四个角点按 `left.top, right.top, right.bottom, left.bottom` 顺序写入（:40-43），这是全项目统一的关键点顺序约定（左上→右上→右下→左下）；`ratio = 两灯条中心连线长 / 长灯条长度`（:49）、`side_ratio = 长灯条/短灯条`（:50）；`rectangular_error` 取左右灯条与中心连线夹角的偏差最大值（:52-55）。
- **隐患：** `center` 是灯条中点均值而非装甲板对角线交点，后续 `center_norm`（归一化）与排序都依赖它，对斜视角装甲板存在系统偏差。

**Armor 的四个神经网络构造函数（armor.cpp:59-219）**
- **作用：** 分别对应 YOLOv8/11（class_id 一体编码）、YOLOv8/11+ROI（加 offset）、YOLOv5（color_id+num_id 分离编码）、YOLOv5+ROI 四条输入路径。
- **逐段说明：** 公共逻辑是：`center = 四关键点均值`；`left_width = |p0-p3|`、`right_width = |p1-p2|`、`top_length = |p0-p1|`、`bottom_length = |p3-p2|`；`ratio = max_length/max_width`（:84，这里是长/宽而非传统构造的宽/长，注意两者语义不同）；`rectangular_error` 用左右灯条向量与中心连线的夹角偏差（:74-82）。区别在 color/name/type 的赋值：YOLOv8/11 查 `armor_properties` 表（:87-96），YOLOv5 用三元表达式映射 `color_id→color`、`num_id→name`（:175-179，且 `num_id==1` 判大装甲，`num_id>5` 直接强转枚举作编号）。
- **优点：** 多构造函数把不同模型头的差异收敛到同一结构体，下游无需关心来源。
- **隐患：** 四段几乎重复的几何计算（:63-82、:112-130、:153-172、:194-213）严重重复，未抽公共函数，改一处要改四处；YOLOv5 的 `num_id>5 ? ArmorName(num_id) : ArmorName(num_id-1)` 里 `num_id>5` 会越界访问 `ARMOR_NAMES`（向量只有 9 个元素），是潜在越界隐患；`type = num_id==1 ? big : small` 只对 num_id==1 判大装甲，与 `armor_properties` 表里"基地/三/四/五 都有大装甲"不一致。

---

### 文件: tasks/auto_aim/detector.hpp / detector.cpp

**总括：** 传统视觉检测器，独立于神经网络、基于灰度阈值 + 轮廓 + 灯条配对 + 分类器，输出 `std::list<Armor>`；同时提供针对单块装甲板的灯条二次矫正接口，被 YOLOv5 用作角点精修。

**构造函数 Detector(const string&, bool)（detector.cpp:13-31）**
- **作用：** 从 YAML 加载全部几何阈值，并初始化 `Classifier classifier_`。
- **逐段说明：** `max_angle_error_` 与 `max_rectangular_error_` 都做了 `度 / 57.3 → 弧度` 换算（:19、:27），其余阈值直接读。`save_path_ = "patterns"` 并 `create_directory`（:29-30）用于后续保存异常样本。
- **优点：** 阈值全部外置到配置，便于联调。
- **隐患：** `/57.3` 是硬编码的度转弧度近似常数（应为 π/180≈0.017453，1/57.3≈0.017452 误差可忽略但不够严谨）；`create_directory` 返回值未检查，目录创建失败时后续 `save` 静默失败。

**detect(const Mat&, int)（detector.cpp:33-120，主检测流程）**
- **逐段说明：** 灰度化 → 全局阈值二值化 → `findContours(RETR_EXTERNAL)`（:40-46）→ 逐轮廓 `minAreaRect` 拟合灯条（:52-53）→ `check_geometry(lightbar)` 过滤（:55）→ `get_color` 判色（:57）→ 灯条按 center.x 从左到右排序（:63）→ 双重循环配对同色灯条（:67-83）：构造 `Armor`、`check_geometry(armor)`、`get_pattern` 裁数字图案、`classifier_.classify` 识别编号、`check_name`、`get_type`、`check_type`、`get_center_norm`，全部通过才入列。
- **共用灯条去重（:87-115）：** 对任意两装甲板，若共享灯条则按两种情形处理——共享同一侧灯条（重叠）保留 `pattern` 面积小者（:96-103），首尾相连（`left.id==right.id`）保留置信度大者（:106-111），用 `duplicated` 标记后 `remove_if` 删除。
- **优点：** 完整覆盖了传统自瞄的经典流程，注释清楚；去重逻辑区分"重叠"与"相连"两种共用灯条场景，是实际联调中常见问题的显式处理。
- **隐患：** `cv::imshow("binary_img", ...)` 直接写在检测热路径里（:42），无 `debug_` 保护、无 `waitKey`，在多线程/无显示环境下会拖慢或报错；`lightbar_id` 递增只在通过几何校验后，但排序后 id 语义依赖生成顺序，配对去重逻辑对 id 的假设较脆弱；全局固定阈值对光照变化敏感（无自适应阈值）。

**detect(Armor&, const Mat&)（detector.cpp:122-231，角点二次矫正）**
- **作用：** 给定神经网络已定位的装甲板四角点，在局部 ROI 内重新做灯条检测，用最近的灯条端点替换原角点。
- **逐段说明：** 先沿灯条方向外推角点（:130-141，`tl1/bl1` 等是把灯条中点沿 top2bottom 方向延长），构造更大的 boundingBox（:143-145），越界或空 ROI 返回 false（:147-157）；在 ROI 内重复二值化 + 轮廓 + 灯条（:159-182，此处 `lightbar_points_corrector` 被注释关闭 :179）；若灯条不足 2 条返回 false（:184）；排序后分别找与"左灯条端点 (tl,bl)"和"右灯条端点 (br,tr)"距离最近的灯条（:190-209），当两最小距离和 <15px 时用灯条端点回写 `armor.points`（:219-227）。
- **优点：** 用神经网络角点作为先验、传统灯条作为精修，是两者互补的典型做法。
- **隐患：** 硬编码阈值 `15`（px）与 `0.75` 外推系数无配置化、无注释依据；`closest_left_lightbar` 与 `closest_right_lightbar` 可能指向同一条灯条，此时角点会退化。

**check_geometry(const Lightbar&)（detector.cpp:233-239）**
- **作用：** 灯条几何校验：角度误差、长宽比、长度三条同时满足。
- **说明：** 阈值来自配置，是灯条筛选的第一道闸。

**check_geometry(const Armor&)（detector.cpp:241-247）**
- **作用：** 装甲板几何校验：`ratio` 在区间内、`side_ratio` 小于上限、`rectangular_error` 小于上限。
- **说明：** 三道约束共同保证"两条近似平行等长、间距合理的灯条"才被当作装甲板。

**check_name(const Armor&)（detector.cpp:249-261）**
- **作用：** 校验识别编号有效且置信度达标。
- **说明：** 若名称有效但置信度不足，`save(armor)` 落盘用于分类器迭代（:255）；出现 5 号时打印 debug 但不过滤（:258）。返回 `name_ok && confidence_ok`。

**check_type(const Armor&)（detector.cpp:263-277）**
- **作用：** 校验"类型与编号"的一致性：小装甲只允许二~五号/哨兵/前哨站（一号和基地必须是大装甲），大装甲只允许一号/基地。
- **说明：** 不一致时打印 debug 并 `save` 保存异常样本（:270-274）。

**get_color（detector.cpp:279-289）**
- **作用：** 遍历轮廓像素累加 B、R 通道和，蓝大于红判蓝，否则判红。
- **隐患：** 逐像素 `at<Vec3b>` 访问效率低；只比 B/R 两通道，紫色/熄灭等其它颜色无法区分；对白平衡偏移敏感。

**get_pattern（detector.cpp:291-309）**
- **作用：** 从灯条中点沿 `top2bottom` 方向延长 1.125 倍得到装甲板四个外角，裁出数字图案 ROI。
- **说明：** 注释给出 1.125 的物理依据：`0.5 * armor_height / lightbar_length = 0.5 * 126mm / 56mm`（:294）。
- **隐患：** ROI 的左右/上下边界用 `std::max/std::min` 夹在图像内，但若灯条本身在图像边缘，外推点可能使 ROI 退化为空或极小，`classify` 里虽有 `h==0||w==0` 保护但 `get_pattern` 本身不校验。

**get_type（detector.cpp:311-338）**
- **作用：** 决定装甲板大小类型。
- **逐段说明：** 先按 `armor.ratio` 判断（>3.0 大、<2.5 小，:316-326）；中间地带按 `name` 判断：一号/基地必大（:331-333），其余（工程/哨兵/前哨站/步兵）判小（:337）。TODO 注明"基地顶装甲是小装甲板"（:336）未处理。
- **隐患：** 两个比例阈值 2.5/3.0 硬编码，与 `armor.hpp` 里神经网络路径的 `ratio`（长/宽）语义可能不一致（传统路径 ratio 是宽/长），需注意两者量纲相反。

**get_center_norm（detector.cpp:340-345）**
- **作用：** 把像素中心归一化到 [0,1]（x/w, y/h），供后续以分辨率无关的方式排序/记录。

**save（detector.cpp:347-352）**
- **作用：** 用时间戳命名把 `armor.pattern` 存为 jpg 到 `patterns/` 目录。
- **隐患：** 文件名只用"名称+时间戳"，同秒多帧可能覆盖；`armor.name` 直接进文件名但 `name` 是枚举，`fmt` 打印的是枚举整数值而非字符串（`ARMOR_NAMES` 未使用），可读性差。

**show_result（detector.cpp:354-384）**
- **作用：** 调试可视化：在检测图上画灯条端点与信息（:361-367）、装甲板角点与信息（:369-376），缩小 0.5 后 `imshow("detection")`。
- **隐患：** `imshow` 无 `waitKey`，非 debug 模式被 `debug_` 拦住，但 debug 模式下仍可能阻塞无头环境。

**lightbar_points_corrector（detector.cpp:386-489，PCA 角点回归）**
- **作用：** 用 PCA 求灯条对称轴，沿轴搜索亮度跳变点精修 `top/bottom`。参考自 FYT2024_vision（:41 注释）。
- **逐段说明：** 配置 MAX_BRIGHTNESS/ROI_SCALE/SEARCH_START/SEARCH_END（:389-392）→ 扩展 ROI 并做边界约束（:395-402）→ 归一化 ROI 亮度（:405-408）→ `moments` 求质心（:411-413）→ 生成稀疏点云（亮度>1e-3 的像素，:416-424）→ `cv::PCA` 求主轴（:427-430，`axis.y>0` 时取反统一方向）→ `find_corner` lambda 沿轴采样多条线、用双线性插值亮度差找跳变点（:432-484）→ 并行检测 top（direction=1）与 bottom（direction=-1）（:487-488）。
- **优点：** 思路完整，用 PCA 主轴 + 亮度跳变比纯 `minAreaRect` 更贴合灯条真实端点；有边界检查与候选点均值。
- **隐患：** 该函数在当前流程中被注释关闭（:179），是死代码路径；双线性插值注释声称用双线性（:462 注释），实际 `gray_img.at<uchar>(Point2i(...))` 只是最近邻取整（:463-464），注释与实现不符；`find_corner` 找不到时返回 `(-1,-1)`，调用方未检查。

---

### 文件: tasks/auto_aim/classifier.hpp / classifier.cpp

**总括：** 数字（兵种编号）分类器，把装甲板图案识别为 9 类编号（对应 `ArmorName` 前 9 项），回填 `armor.name` 与 `armor.confidence`。同时保留了 cv::dnn 与 OpenVINO 两套推理后端。

**成员（classifier.hpp:22-24）**
- **作用：** `cv::dnn::Net net_`（旧后端）、`ov::Core core_` + `ov::CompiledModel compiled_model_`（OpenVINO 后端）。

**构造函数（classifier.cpp:7-15）**
- **作用：** 从配置读 `classify_model` 路径，同时用 `cv::dnn::readNetFromONNX` 和 OpenVINO 各加载一份。
- **说明：** OpenVINO 侧 `compile_model(ovmodel, "AUTO", LATENCY)`（:13-14）。
- **隐患：** 一个类同时加载两份模型（cv::dnn + OpenVINO）占用双份内存，而实际主路径只用 cv::dnn，是资源浪费；`"AUTO"` 设备在无 GPU 时回退 CPU，行为依赖环境。

**classify(Armor&)（classifier.cpp:17-59，cv::dnn 后端，主路径）**
- **逐段说明：** 空 pattern 直接置 `not_armor` 返回（:19-22）；灰度化 → 等比缩放到 32×32（`scale = min(32/cols, 32/rows)`，保持宽高比，不足部分补 0，:27-39）；`blobFromImage(1/255)` 归一化到 [0,1]（:41）→ `net_.forward()`（:44）→ 手工 softmax（减去最大值防溢出 `cv::exp(outputs-max)` 再除以和，:47-50）→ `minMaxLoc` 取最大类与置信度（:52-55）→ `armor.name = static_cast<ArmorName>(label_id)`（:58）。
- **优点：** 手工 softmax 先减 max 数值稳定；保持宽高比的 letterbox 避免直接拉伸失真。
- **隐患：** `label_id` 直接强转为 `ArmorName`，若模型输出 9 类但 `ArmorName` 枚举另有 `not_armor`（第 9 项），下标可能错位（`label_id` 最大 8 恰好是 `not_armor` 的位置，语义上依赖"第 9 类=not_armor"的隐式约定）；`cv::dnn` 每次 forward 都重建输入 blob，性能不如复用。

**ovclassify(Armor&)（classifier.cpp:61-112，OpenVINO 后端）**
- **作用：** 与 `classify` 相同逻辑的 OpenVINO 版本。
- **逐段说明：** 预处理一致（:68-85），但用 `input.convertTo(CV_32F, 1/255)` 手动归一化（:87）；`ov::Tensor(ov::element::f32, {1,1,32,32}, input.data)`（:89）→ `create_infer_request` + `infer`（:91-93）→ `get_output_tensor` 包成 `cv::Mat(1,9,CV_32F, ...)`（:97）→ 同款 softmax + max 类（:100-111）。
- **隐患：** `cv::Mat(1, 9, CV_32F, output_tensor.data())` 直接借用 OpenVINO 张量内存，若模型输出 shape 不是 {1,9} 会越界（硬编码 9）；每帧 `create_infer_request` 走单请求延迟路径，性能不如复用；与 `classify` 完全重复的预处理逻辑未复用。

---

### 文件: tasks/auto_aim/yolo.hpp / yolo.cpp

**总括：** YOLO 抽象层与工厂。`YOLOBase` 定义统一接口，`YOLO` 根据配置动态实例化具体实现，是策略模式。

**class YOLOBase（yolo.hpp:10-17）**
- **作用：** 纯虚基类，声明 `detect` 与 `postprocess` 两个接口，返回值都是 `std::list<Armor>`。
- **说明：** `postprocess` 被 `MultiThreadDetector` 直接调用（它自己管理推理），`detect` 是"预处理+推理+后处理"的整包接口。

**class YOLO（yolo.hpp:19-31）**
- **作用：** 工厂类，持有 `std::unique_ptr<YOLOBase> yolo_` 并转发调用。

**构造函数 YOLO(const string&, bool)（yolo.cpp:11-31）**
- **作用：** 读 `yolo_name` 配置，分别构造 `YOLOV8`/`YOLO11`/`YOLOV5`；未知名称抛 `std::runtime_error`（:29）。
- **说明：** 支持三种检测器的热切换。

**detect / postprocess 转发（yolo.cpp:33-42）**
- **作用：** 纯转发，无额外逻辑。

---

### 文件: tasks/auto_aim/yolos/yolov5.hpp / yolov5.cpp

**总括：** YOLOv5 检测器实现，输出头为"4 关键点 + 颜色/类别双独热"（22 列），是三者中唯一把"颜色"与"编号"分两路预测、并支持传统角点二次矫正的实现。

**成员与常量（yolov5.hpp:27-44）**
- **作用：** `class_num_=13`（4 颜色 + 9 编号的合计，但实际输出列是 9+9=18 列，见后文）、`nms_threshold_=0.3`、`score_threshold_=0.7`；`min_confidence_`、`binary_threshold_`；`roi_/offset_` 用于 ROI 裁切；`detector_` 用于传统二次矫正；`tmp_img_` 存原图供 save。

**构造函数（yolov5.cpp:13-57）**
- **作用：** 读配置、加载模型并配置 PPP 预处理。
- **逐段说明：** `detector_(config_path, false)` 关闭传统检测的 debug 显示（:14）；读 ROI 四元组与 `use_roi_/use_traditional_` 开关（:22-30）；`save_path_="imgs"` 建目录（:32-33）；PPP 配置：输入 u8、NHWC、BGR、{1,640,640,3}，模型侧 NCHW，预处理转 f32、转 RGB、`scale(255.0)`（:35-49）；`compile_model(model, "CPU", LATENCY)`（:53-56）。
- **隐患：** 设备被硬编码为 `"CPU"`（:54），尽管 `device_` 成员从配置读了 `device`（:19），两者脱节——`device_` 字段实际是死配置；`input.model().set_layout("NCHW")` 后未显式 `resize`，依赖模型自身输入尺寸与 letterbox 一致。

**detect（yolov5.cpp:59-102）**
- **逐段说明：** 空图警告返回（:61-64）；按 `use_roi_` 裁 ROI（:67-77，`width==-1` 表示不裁该维度）；letterbox 缩放（`scale = min(640/rows, 640/cols)`，:79-88）；`ov::Tensor` 借 `input.data` 提交（:89）；`create_infer_request` + `infer`（:92-94）；输出张量包成 `cv::Mat(output_shape[1], output_shape[2], CV_32F, ...)`（:99）；调 `parse`。
- **隐患：** 每帧 `create_infer_request()`（:92）走单请求延迟路径，与 `MultiThreadDetector` 的优化结论相悖；`output_shape` 未校验维度，若模型输出 3 维以上会越界。

**parse（yolov5.cpp:104-195）**
- **逐段说明：** 逐行解析（每行一个候选）：`score = sigmoid(output(r,8))`（:113-114，第 8 列是目标置信度）低于阈值跳过；颜色独热取列 9~12、类别独热取列 13~21（:121-122），各自 `minMaxLoc` 取最大类（:126-127）；四个关键点按 (列0,1)/(6,7)/(4,5)/(2,3) 顺序取（:131-138，即 tl→tr→br→bl 顺序重排）；`/scale` 还原到原图坐标（:132-137）；`min/max` 求包围盒 rect（:140-152）；收集后 `cv::dnn::NMSBoxes`（:161-162）；按 `use_roi_` 决定是否加 offset 构造 `Armor`（:165-172）；随后 `check_name`、`check_type` 过滤（:176-185），`use_traditional_` 时调 `detector_.detect` 二次矫正角点（:186），回填 `center_norm`（:188）；debug 画图（:192）。
- **优点：** 颜色与编号解耦预测，能区分"熄灭"等无颜色装甲；支持传统二次矫正。
- **隐患：** `sigmoid` 只对第 8 列做，而颜色/类别列直接 `minMaxLoc` 取原始 logits（未过 softmax/sigmoid），虽然取最大类位置不变但语义上不是概率，与 `armor.confidence` 被赋值为 `score`（目标置信度）而非类别置信度，`check_name` 里的 `confidence > min_confidence_` 判断的其实是目标置信度；`score_threshold_` 与 `min_confidence_` 双重阈值逻辑重叠。

**check_name / check_type（yolov5.cpp:197-219）**
- **作用：** 同 Detector 版本，校验编号有效 + 置信度达标；类型一致性（小装甲排除一号/基地，大装甲排除二/哨兵/前哨站）。
- **说明：** 保存样本的 `save` 调用被注释掉（:203、:216），迭代收集功能在此路径关闭。

**draw_detections / save / sigmoid / postprocess（yolov5.cpp:228-269）**
- **作用：** 调试画图（画角点、置信度、ROI 框，`waitKey(1)` 泵窗口事件，:247）、`save` 存 `tmp_img_`、`sigmoid` 数值稳定实现（:257-263，分正负避免 `exp(-x)` 溢出）、`postprocess` 转 `parse`。
- **隐患：** `draw_detections` 的 `waitKey(1)` 会拖慢主循环，且 debug 分支在无头环境异常。

---

### 文件: tasks/auto_aim/yolos/yolov8.hpp / yolov8.cpp

**总括：** YOLOv8 检测器，输出头为"xywh + 2 类 + 4 关键点"（14 列），关键点需 `sort_keypoints` 排序，编号由 `Classifier` 二次识别（因为 2 类只区分"是否装甲"或颜色，不含编号）。

**成员与常量（yolov8.hpp:28-52）**
- **作用：** `class_num_=2`、`nms_threshold_=0.3`、`score_threshold_=0.7`；额外持有 `classifier_` 与 `detector_`；`use_roi_`、`roi_/offset_`。

**构造函数（yolov8.cpp:17-59）**
- **作用：** 同 YOLOv5，但输入尺寸 416×416（:44），设备用配置的 `device_`（:57-58，这点比 YOLOv5 正确）。
- **说明：** 2 类模型通常只做"颜色二分类"（红/蓝），编号依赖后续分类器。

**detect（yolov8.cpp:61-104）**
- **作用：** 同 YOLOv5 流程，416 letterbox + 推理 + parse。
- **说明：** 输出张量先 `cv::transpose` 成 [N, 14]（:110 在 parse 里）。

**parse（yolov8.cpp:106-185）**
- **逐段说明：** `cv::transpose(output, output)`（:110）；逐行取 xywh（列 0-3）、scores（列 4-5）、关键点（列 6-13）（:117-119）；`minMaxLoc(scores)` 取最大分（:125，注意这里 `score` 是原始值，YOLOv8 输出默认未过 sigmoid）；`(x-0.5w)/scale` 还原包围盒（:133-136）；关键点 `/scale` 还原（:138-143）；NMS（:151）；对保留候选 `sort_keypoints`（:155）后构造 `Armor`（:156-161）；随后 `get_pattern` + `classifier_.classify`（:164-165）识别编号，`check_name/check_type` 过滤（:167-177）。
- **优点：** 把"检测"与"编号识别"解耦，检测器轻量、编号分类器可独立迭代。
- **隐患：** 2 类 scores 未过 sigmoid 直接与 `score_threshold_=0.7` 比较，若模型输出是 logits（YOLOv8 默认不带 sigmoid 时）阈值语义错误，可能大量误过滤或误通过；`classifier_.classify` 是同步推理，叠加在检测之后增加单帧延迟。

**get_type（yolov8.cpp:211-227）**
- **作用：** 按编号决定类型：一号/基地大，二/哨兵/前哨站小，其余（步兵）默认小。
- **说明：** 与 Detector 的 `get_type` 相比少了"按 ratio 判大小"的分支，因为神经网络路径的 ratio 语义不同。

**sort_keypoints（yolov8.cpp:296-322）**
- **作用：** 把无序 4 关键点排序为 tl→tr→br→bl。
- **逐段说明：** 非 4 点打印告警返回（:298-301）；按 y 排序分上下两组（:303-305）；上下组各自按 x 排序（:310-316）；赋 `[0]=tl, [1]=tr, [2]=br(右下), [3]=bl(左下)`（:318-321）。
- **优点：** 用 y 后 x 的两级排序，对一般拍摄姿态正确。
- **隐患：** 当装甲板侧倾接近水平时，"上/下"分组失效，排序错乱；仅 `std::cout` 告警无异常。

**get_pattern（yolov8.cpp:236-267）**
- **作用：** 与 Detector::get_pattern 类似，用关键点外推 1.125 倍得 ROI，含 ROI 有效性校验（:253-264）。
- **说明：** 此版本有 `roi_right<=roi_left` 与越界校验（:254、:260），比 Detector 版本健壮。

**draw_detections / save / postprocess（yolov8.cpp:276-328）**
- **作用：** 同 YOLOv5，但 `draw_detections` 无 `waitKey`（:293 结尾无）。

---

### 文件: tasks/auto_aim/yolos/yolo11.hpp / yolo11.cpp

**总括：** YOLO11 检测器，输出头为"xywh + 38 类 + 4 关键点"（50 列），是当前主力。38 类一次性编码了颜色×编号×类型的组合，编号无需二次分类器，直接在构造函数里查 `armor_properties`。

**成员与常量（yolo11.hpp:27-44）**
- **作用：** `class_num_=38`（对照 `armor_properties` 表共 36 项 + 可能的背景类）、输入 640×640；持有 `detector_`（用于潜在的传统矫正）与 `tmp_img_`。

**构造函数（yolo11.cpp:13-54）**
- **作用：** 同 YOLOv8，输入 640，设备读配置 `device_`（:52-53）。

**detect（yolo11.cpp:56-100）**
- **作用：** 同 YOLOv8，640 letterbox；额外 `tmp_img_ = raw_img`（:64）供 save。

**parse（yolo11.cpp:102-176）**
- **逐段说明：** `cv::transpose`（:105）；逐行取 xywh（0-3）、38 类 scores（4-41）、关键点（42-49）（:112-114）；`minMaxLoc` 取最大类别（:120）；还原 box 与关键点（:124-138）；NMS（:146）；`sort_keypoints` + 构造 `Armor`（:150-156，class_id 直接就是 38 类之一，构造函数查表）；`check_name/check_type` 过滤（:159-169）；`center_norm`（:169）。
- **说明：** 与 YOLOv8 的唯一实质差异是 class_id 直接映射到完整 (color,name,type)，无需 `Classifier`。
- **优点：** 单模型端到端出"颜色+编号+类型"，消除了检测后二次分类的延迟与误差链，是当前架构里最简洁的检测路径。
- **隐患：** 38 类里若包含背景类，`class_id` 越界时 `Armor` 构造函数回退默认 `{blue,not_armor,small}` 而非显式剔除背景，依赖 `check_name`（`not_armor` 被过滤）兜底；`check_name` 里的 `save` 被注释（:183），异常样本迭代通道关闭。

**check_name / check_type / sort_keypoints / draw_detections / save / postprocess（yolo11.cpp:178-269）**
- **作用：** 与 YOLOv8 版本逻辑一致（sort_keypoints 完全相同，draw_detections 与 YOLOv5 相同且带 `waitKey`）。

---

### 文件: tasks/auto_aim/tracker.hpp / tracker.cpp

**总括：** 目标跟踪状态机。维护"丢失→检测中→跟踪中→临时丢失/切换"的状态转换，负责目标建立、切换与丢失判定，并协调 `Solver` 解算与 `Target` EKF 更新。文件注释大量记录了 2026 年联调实测的坑与修正。

**成员（tracker.hpp:32-48）**
- **作用：** `solver_`（解算器引用）、`enemy_color_`、`min_detect_count_`（进入 tracking 需连续命中帧数）、`max_temp_lost_count_`（临时丢失容忍帧数）、`outpost_max_temp_lost_count_`/`normal_temp_lost_count_`（前哨站特殊容忍）、`max_dt_=0.1`（帧间隔上限）、`target_`（当前目标）、`omni_target_priority_`（全向感知切换优先级）。

**构造函数（tracker.cpp:12-33）**
- **作用：** 读配置初始化敌方颜色、检测/丢失计数阈值、`max_dt_`（缺省 0.1s，注释说明该值按 ~100fps 设计，CPU 推理平台必须调大否则每帧超限永远建不起目标）。
- **说明：** `state_`、`pre_state_` 初始为 `"lost"`。

**track（tracker.cpp:37-108，主跟踪入口）**
- **逐段说明：** 计算 `dt` 并更新 `last_timestamp_`（:40-41）；`dt > max_dt_` 且非 lost 态时判丢失（:44-47）；`armors.remove_if` 过滤非敌方颜色（:49）；注释掉的前哨站顶部装甲过滤（:52-56，通过 `solver_.oupost_reprojection_error` 判断 pitch 正负）；先按"靠近图像中心 (1440/2,1080/2)"排序（:59-64，TODO 标记硬编码分辨率），再按 `priority` 排序（:67-68）；`lost` 态 `set_target`、否则 `update_target`（:70-77）；`state_machine(found)`（:79）；发散检测 `target_.diverged()` → lost（:82-86）；NIS 收敛检测（:89-96，`recent_nis_failures` 累计超过 `0.4*window_size` 判收敛失败）；`detecting` 期间不下发目标（:104，注释 2026-10-18：新目标单帧 PnP 初始化噪声 ±15~30°，立即下发会让云台猛晃，等 EKF 收敛到 tracking 再下发，代价 ~125ms）。
- **优点：** 状态机清晰，发散与 NIS 双重健康检测，detecting 态不发指令是防抖的关键设计。
- **隐患：** 图像中心 `1440/2, 1080/2` 硬编码（:60，TODO），与 `center_norm` 设计（本应分辨率无关）矛盾；排序用 `std::list::sort` 两次（先距离后优先级），第二次 `sort` 的稳定性依赖 `std::list::sort` 的稳定排序保证，逻辑上"优先级高者优先、同级靠近中心"依赖实现细节。

**track(detection_queue, ...)（tracker.cpp:110-190，全向感知重载）**
- **作用：** 融合全向感知（omniperception）检测队列的跟踪版本，支持从全向相机切换更高优先级目标。
- **逐段说明：** 取队列头为 `temp_target`（:116-118）；`dt` 超限判丢失（:124-127）；排序同主路径（:130-138）；状态分支：`lost` 建目标（:141-143）；`tracking` 且主相机出现更高优先级目标时切换（:146-149）；`tracking` 且全向感知出现更高优先级目标（且当前已收敛）时置 `switching` 态（:152-161）；`switching` 态等主相机出现对应优先级目标（:163-165）；`detecting` 且前一态是 `switching` 时重建（:167-169）；否则 `update_target`（:171-173）。返回 `{switch_target, targets}`。
- **说明：** `switch_target` 是让云台在切换前转向全向感知目标的预瞄结果。

**state_machine（tracker.cpp:192-242）**
- **逐段说明：** `lost`→命中后进 `detecting`（count=1）；`detecting` 命中则 count++，达 `min_detect_count_` 进 `tracking`，未命中归零回 `lost`；`tracking` 命中保持、未命中进 `temp_lost`（count=1）；`switching` 命中进 `detecting`、未命中 count++ 超 200 回 lost；`temp_lost` 命中回 `tracking`、未命中 count++，前哨站用更大的 `outpost_max_temp_lost_count_`（:233-237），超限回 `lost`。
- **优点：** 状态迁移完整，前哨站（转速快、易丢）单独放宽丢失容忍。

**set_target（tracker.cpp:244-277）**
- **逐段说明：** 取排序后最优先装甲板（:248）；`solver_.solve(armor)` 解算位姿（:249）；按兵种初始化 `Target`：平衡步兵（大三/四/五号）用 `radius=0.2, armor_num=2` 与特定 `P0_dig`（:252-259）；前哨站 `radius=0.2765, armor_num=3`（:261-264）；基地 `radius=0.3205, armor_num=3`（:266-269）；其余 `radius=0.2, armor_num=4`（:271-274）。`P0_dig` 是 EKF 初始协方差对角，前哨站/基地把 `r/l/h` 相关协方差设得极小（`1e-4, 0, 0`）表示这些自由度几乎不估计。
- **说明：** `radius`（公转半径）与 `armor_num`（装甲板数）是强先验，正确设置对 EKF 收敛至关重要。

**update_target（tracker.cpp:279-306）**
- **逐段说明：** `target_.predict(t)` 先预测（:281）；统计与当前目标 name/type 匹配的装甲板数（:283-289）；无匹配返回 false；对每个匹配装甲板 `solver_.solve` + `target_.update`（:293-303，注释里 `armor.center.x != min_x` 的最左侧过滤被注释掉）。
- **隐患：** `update_target` 里对多块匹配装甲板（同名同型）逐块 `update`，会多次更新 EKF；`min_x` 计算了却因注释而未使用，是残留死代码。

---

### 文件: tasks/auto_aim/target.hpp / target.cpp

**总括：** 单个目标的 11 维 EKF 状态估计。状态量为 `[center_x, vx, center_y, vy, center_z, vz, a(公转角), w(角速度), r(公转半径), l(长板半径增量), h(高度增量)]`，观测为 `[yaw, pitch, distance, angle]`。文件内注释极其详尽，记录了 2026-10 月至 11 月联调中近距抖动、镜像翻转、假自转等一系列问题的排查与修正，是理解整个系统鲁棒性设计的核心。

**成员（target.hpp:19-63）**
- **作用：** `name/armor_type/priority/jumped/last_id` 公开目标元信息；私有 `armor_num_`（装甲板数）、`switch_count_/update_count_/angle_hold_count_/pos_hold_count_`、`ekf_`、`t_` 等。
- **说明：** `jumped` 表示是否发生过装甲板序号跳变（id 翻转），`last_id` 仅调试。

**构造函数 Target(Armor, t, radius, armor_num, P0_dig)（target.cpp:19-59）**
- **逐段说明：** 由解算出的 `xyz_in_world` 与 `ypr_in_world` 反推旋转中心：`center_x = xyz[0] + r·cos(yaw)`、`center_y = xyz[1] + r·sin(yaw)`（:39-41，注意这里用 `r` 而非减号，与 `h_armor_xyz` 里的 `x - r·cos(angle)` 符号相反，构成"中心↔板心"互逆关系）；状态初值 `x0 = [cx,0,cy,0,cz,0,yaw,0,r,0,0]`（:48）；`P0 = P0_dig.asDiagonal()`（:49）；`x_add` lambda 把第 7 维（角度 a）用 `limit_rad` 归一化（:52-56）；构造 EKF（:58）。

**predict(double dt)（target.cpp:84-146）**
- **逐段说明：** 状态转移矩阵 F 是匀速运动模型：位置/速度成对 `[1,dt;0,1]`，角度/角速度 `[1,dt;0,1]`，r/l/h 恒等（:88-100）；过程噪声 Q 用 Piecewise White Noise Model（:118-131），`a=dt⁴/4, b=dt³/2, c=dt²` 按 dt 放缩，位置/速度通道用加速度方差 v1，角度/角速度通道用角加速度方差 v2；前哨站用 v1=10/v2=0.1，其它 v1=100/v2=400（:107-113）；`f` lambda 预测后把角度归一化（:135-139）；前哨站转速特判（:142-143）：已收敛且 |w|>2 时把 w 钳制到 ±2.51（前哨站真实转速）。
- **优点：** 过程噪声矩阵结构标准，前哨站特判体现对特殊目标运动学的理解。
- **隐患：** Q 矩阵里 r/l/h 三行的过程噪声恒为 0（:128-130），即这三个量是纯随机游走无噪声，依赖观测通过雅可比间接更新——这也是注释里反复提到的"噪声经 Jacobian 持续泵高 r"的根源之一（见 update_ypda）。

**update(const Armor&)（target.cpp:148-195）**
- **逐段说明：** 装甲板匹配：由当前状态生成各板世界坐标 `armor_xyza_list()`（:153），按距离排序取前 3（:160-166），对每块候选计算"角度误差 + yaw 误差"（:172-173），取误差最小的 id 作为本次观测对应的装甲板序号（:175-178）；`id != 0` 置 `jumped`（:181）；`id != last_id` 置 `is_switch_`（:183-187）并计数；记录 last_id、update_count++（:191-192）；`update_ypda(armor, id)`（:194）。
- **说明：** 匹配逻辑本质上在做"最近邻数据关联"，把观测分配到最可能的装甲板序号。

**update_ypda（target.cpp:197-353，EKF 观测更新 + 创新门控）**
- **逐段说明：** 这是全项目最核心、注释最密集的函数，分四段：
  1. **雅可比与测量噪声 R（:205-227）：** `H = h_jacobian(ekf_.x, id)`；位置通道测量噪声随距离自适应 `r_pos = 4e-3 · (1.5/max(d,0.4))²` 并 clamp 到 [4e-3, 0.05]（:219-221），注释记录了 2026-10-18 的教训：固定 4e-3 过度信任近距噪声导致 r 从 0.2→0.67→1.005 泵高触发误杀；(1.5/d)⁴ 上限 0.20 又压得太死导致滤波不响应。角度通道 R 用 `log(|delta_angle|+1)+1`、距离通道 `log(d+1)/200+9e-2`（:222-224）。
  2. **观测函数 h 与 z（:230-248）：** `h(x) = [yaw, pitch, distance, angle]`，其中 `angle = limit_rad(x[6] + id·2π/armor_num)`（:233）；观测 `z = [ypd[0], ypd[1], ypd[2], ypr[0]]`（:248）；`z_subtract` 对第 1/2/4 维做 `limit_rad`（:238-244）。
  3. **创新门控（:250-317）：** 计算 `innov = z - h(ekf_.x)`（:259）；角度通道门控：|innov[3]|>0.15 rad（≈11.5°）时，若连续拒帧达 6 帧上限则放行本帧，否则 `R(3,3) *= 1e4` 跳过该通道并衰减角速度 `ekf_.x[7] *= 0.6`（仅 |w|>0.8 且非前哨站）（:272-291）；位置通道门控：`gate_angle || |innov[0/1]|>0.20 || |innov[2]|>0.50` 时拒对应通道（:298-317）。注释详述了 2026-10-14 的近距抖动根因：旧实现用模型角顶替观测照常 update，创新≈0 被滤波器欣然接受，id 翻转注入 90° 伪测量跳变拟合出 w=±2~4rad/s 假自转。
  4. **噪声帧降权 + 几何钳制（:322-344）：** 浅谷/贴边帧（`yaw_drop<60 && yaw_slip>30° && d<1.8`，或 `edge_off>0.58`）把角度 R 乘 25 降权（:322-326）；`ekf_.update`（:328）；r/l 非负钳制（:334-335）；r 上界软钳 0.45、r+l≤0.45（:342-344）。
  5. **联调日志（:349-352）：** 逐帧打印 z_yaw/d/a/w/r/l/nis 与 gate 标志。
- **优点：** 创新门控 + 自适应 R + 假自转刹车 + 几何钳制构成了一套完整的、经过实测验证的防抖/防发散机制，注释详尽到可复现排查过程，是工程化 EKF 的范本。
- **隐患：** 大量魔数（0.15、0.20、0.50、1e4、25、0.6、0.45、0.8、6、8、30°、60、0.58 等）硬编码在函数体内而非配置，联调后未抽出，可维护性差；R 的自适应公式是经验拟合（log、幂次、clamp），缺乏理论推导，换相机/环境后需重新标定。

**armor_xyza_list（target.cpp:359-369）**
- **作用：** 把 EKF 状态展开为每块装甲板的 `[x, y, z, angle]`（xyza）列表，供 Aimer/Planner 选板瞄准。

**h_armor_xyz（target.cpp:408-419）**
- **作用：** 状态 → 装甲板世界坐标的观测模型：`armor_x = x[0] - r·cos(angle)`、`armor_y = x[2] - r·sin(angle)`，`angle = limit_rad(x[6]+id·2π/armor_num)`；`armor_num==4` 且 id 为 1/3 时用 `r+l`（长板半径）与 `z+h`（高度增量）（:411-416）。
- **说明：** `r+l`/`h` 只在四装甲板（普通步兵）的 id 1/3 生效，体现"长板相对短板的半径/高度增量"的几何模型。

**h_jacobian（target.cpp:421-458）**
- **作用：** 观测函数 h 的解析雅可比，分两段链式：`H_armor_xyza`（状态→装甲板 xyza，:438-443）与 `H_armor_ypd`（xyza→球坐标，调用 `tools::xyz2ypd_jacobian`，:447），相乘得 `H_armor_ypda`（:449-454）。
- **优点：** 解析雅可比避免数值差分，效率与精度俱佳。

**diverged（target.cpp:374-391）**
- **作用：** 发散判定：`r ∈ (-0.05, 1.0)` 且 `r+l ∈ (-0.05, 1.0)`。
- **说明：** 注释记录了阈值调参历史：下界曾设 0.05 导致 r 收敛到 ~0 时被误杀，上界曾设 0.5 导致甩板瞬时 r=0.53 被误杀，最终放宽到 1.0。

**convergened（target.cpp:393-405）**
- **作用：** 收敛判定：非前哨站 update_count>3 且未发散，前哨站 update_count>10 且未发散。
- **说明：** 该值被 Tracker 用于"全向切换需已收敛"与 Aimer 无关，是 `is_converged_` 的惰性置位。

---

### 文件: tasks/auto_aim/solver.hpp / solver.cpp

**总括：** 位姿解算器。用 `solvePnP(IPPE)` 把装甲板四角点反解出三维位姿，做坐标系级联变换（相机→云台→世界），并用重投影搜索修正 yaw（`optimize_yaw`）。文件内含大量 2026-10 月联调注释，详述 IPPE 双解镜像、roll 滑移等问题与修正。

**常量与目标点（solver.cpp:12-25）**
- **作用：** `LIGHTBAR_LENGTH=56mm`、`BIG_ARMOR_WIDTH=230mm`、`SMALL_ARMOR_WIDTH=135mm` 三个物理尺寸，以及大/小装甲板的 4 个三维角点（`BIG_ARMOR_POINTS`/`SMALL_ARMOR_POINTS`）。
- **说明：** 目标点定义在装甲板局部系（x 向前为 0，y 为宽度方向 ±W/2，z 为灯条方向 ±L/2），是 PnP 的 object_points。

**构造函数（solver.cpp:27-44）**
- **作用：** 从 YAML 读标定：`R_gimbal2imubody`（云台→IMU 体）、`R_camera2gimbal`、`t_camera2gimbal`（相机→云台外参）、`camera_matrix`（3×3）、`distort_coeffs`（1×5 畸变系数），经 `cv::eigen2cv` 转 OpenCV 格式。
- **说明：** `R_gimbal2world_` 初始为单位阵。

**set_R_gimbal2world（solver.cpp:48-52）**
- **作用：** 由 IMU 四元数更新云台→世界旋转：`R_gimbal2world = R_gimbal2imubody^T · R_imubody2imuabs · R_gimbal2imubody`。
- **说明：** 把 IMU 绝对姿态通过体→云台外参换算到云台姿态。

**solve(Armor&)（solver.cpp:55-92，PnP 解算）**
- **逐段说明：** 按类型选目标点（:57-58）；`solvePnP(..., SOLVEPNP_IPPE)`（:65-67，注释说明 IPPE 平面位姿有双解，逐帧翻转导致的 yaw 跳变不在求解器里选解，统一交给 Target 创新门控）；`xyz_in_gimbal = R_camera2gimbal·tvec + t_camera2gimbal`（:71）；`xyz_in_world = R_gimbal2world·xyz_in_gimbal`（:72）；由 rvec 得旋转矩阵，级联到 `R_armor2gimbal`/`R_armor2world`，用 `tools::eulers(...,2,1,0)`（z-y-x 欧拉）得 `ypr_in_gimbal/world`（:74-81）；`ypd_in_world = xyz2ypd(...)`（:83）；平衡步兵（大三/四/五号）跳过 yaw 优化（:86-89，因为 pitch 假设不成立）；否则 `optimize_yaw`（:91）。
- **说明：** 坐标级联链清晰：相机系 → 云台系 → 世界系。

**optimize_yaw（solver.cpp:200-328，yaw 重投影搜索）**
- **作用：** 用重投影误差在 ±70° 全范围搜索最佳 yaw，修正 PnP 的 yaw 估计，并对 roll 滑移做门控拉力。
- **逐段说明：** 搜索起点 `yaw0 = gimbal_ypr[0] - 70°`（:207-208）；`armor.yaw_raw = armor.ypr_in_world[0]`（:212）；**第一遍自由搜索**（:216-227）：逐度算 `armor_reprojection_error`，取最小误差的 yaw 为 `best_yaw`，顺带记录 raw 处误差 `e_raw`；**roll 滑移门控**（:251-290）：计算 `slip = |best_yaw - raw|`、`drop = e_raw - min(error(raw±15°))`、`edge_off`（板中心相对光轴偏移角正切），门控条件 `slip>8° && drop>60px && drop<300px && d<1.8m` 时 `pull_k = 2400 px/rad`（:287-290）；**第二遍带拉力搜索**（:295-306）：门控成立时重置 min_error，用 `armor_reprojection_error(..., pull_k)` 重搜，把候选拉向 raw；最后 `armor.ypr_in_world[0] = best_yaw`（:308）。注释详尽记录了 2026-10-12/13 的联调过程（无死区拉力、镜像盆地锁死、±10° 死区骑边界等问题）。
- **优点：** 全范围搜索避免时间先验锁死在镜像盆地；roll 滑移门控三条件 + 无死区拉力是经过实测验证的巧妙设计；`yaw_slip/yaw_drop/edge_off` 下传给 Target 做噪声帧降权，形成闭环。
- **隐患：** `SEARCH_RANGE=140`（度）逐度暴力搜索，每帧最多 140 次 `armor_reprojection_error`（内含 `cv::projectPoints`），计算开销可观；门控阈值（8°、60/300px、1.8m、2400）全是联调经验值，硬编码，环境变化需重新标定。

**armor_reprojection_error（solver.cpp:364-407）**
- **作用：** 计算给定 yaw 下重投影与检测角点的误差，并叠加透视不对称惩罚与 roll 拉力。
- **逐段说明：** `reproject_armor` 得投影点，累加四角点像素距离（:367-369）；透视不对称校验（:372-390）：检测左右灯条高度差 `det_asym` 与投影高度差 `proj_asym` 符号相反（镜像解）时，叠加 `max(50, 4·|det_asym|)` 惩罚，剔除镜像盆地；roll 拉力（:403-404）：`raw_pull_k>0` 时叠加 `raw_pull_k·|yaw - yaw_raw|`。
- **说明：** 注释里 `SJTU_cost`（:370 被注释）是另一套基于线段角度/长度差的代价函数，被弃用。

**SJTU_cost（solver.cpp:330-362）**
- **作用：** 一种基于"线段起点差 + 长度差 + 角度差"的代价函数（参考 SJTU 方案），当前被注释弃用（:370）。
- **隐患：** 死代码，保留在文件里增加维护负担。

**reproject_armor（solver.cpp:94-131）**
- **作用：** 由世界坐标 + yaw（+ 固定 pitch）反投影装甲板四角点到像素。
- **逐段说明：** 用 `yaw` 与固定 pitch（前哨站 -15°、其它 +15°，:100）构造 `R_armor2world`（:105-110）；反算 `R_armor2camera`/`t_armor2camera`（:114-117）；Rodrigues 转 rvec + `cv::projectPoints`（:120-129）。
- **说明：** 这里的 pitch 假设（装甲板相对世界俯仰 15°）正是 `solve` 里平衡步兵跳过 yaw 优化的原因（其 pitch 假设不成立）。

**oupost_reprojection_error（solver.cpp:133-198）**
- **作用：** 对给定 pitch 假设计算前哨站装甲板的重投影误差，用于判断前哨站顶部/底部装甲（当前在 tracker 里被注释掉）。
- **说明：** 与 `reproject_armor` 类似，但 pitch 作为参数传入。

**world2pixel（solver.cpp:410-436）**
- **作用：** 世界坐标点到像素坐标的投影，会剔除相机后方（z≤0）的点。
- **说明：** 供其它模块（如可视化/预瞄）使用。

---

### 文件: tasks/auto_aim/aimer.hpp / aimer.cpp

**总括：** 瞄准器。把 `Target`（EKF 状态）转成云台 yaw/pitch 指令：选板、弹道解算、飞行时间迭代、视差补偿。是小陀螺/普通/前哨站三种瞄准策略的载体。

**struct AimPoint（aimer.hpp:15-19）**
- **作用：** `{valid, xyza}` 描述一个瞄准点是否有效及其世界坐标+角度。

**成员（aimer.hpp:34-50）**
- **作用：** `yaw_offset_/pitch_offset_`（整体偏置）、`left/right_yaw_offset_`（左右枪位偏置）、`comming/leaving_angle_`（小陀螺进出射击区角度）、`spin_mode_`（小陀螺滞回）、`high/low_speed_delay_time_`、`max_predict_time_`（to_now 时域上限）、`camera_offset_y_`（相机横向偏置）、`decision_speed_`、`default_bullet_speed_`、`lock_id_`（锁板 id）。

**构造函数（aimer.cpp:15-51）**
- **作用：** 读配置并做单位换算（度→弧度）；`max_predict_time_` 缺省 0.15（:36，注释记录 2026-10-17/17:20 调参：预测误差 ∝ w，从 0.3 收到 0.15 再收 0.08）；`camera_offset_y_` 从 `t_camera2gimbal[1]` 读、缺省 0.104（:39-40，注释给出 0.65m 处视差 ≈ asin(0.104/0.65)≈9°）；`default_bullet_speed_` 缺省 23（:44）；左右枪位偏置可选加载（:45-50）。

**aim(targets, timestamp, bullet_speed, to_now)（aimer.cpp:53-164，主瞄准）**
- **逐段说明：** 无目标返回 `{false,false,0,0}`（:58）；取首个目标（:59）；按 `|w|` 与 `decision_speed_` 选高/低速延迟（:63-64，用绝对值避免漏掉反向旋转）；弹速 <14 时用默认值并节流告警（:67-75）；**预测时域**：`to_now` 时 `dt = 帧龄 + delay_time`，clamp 到 `[0, max_predict_time_]`（:82-84），否则用固定 `0.005 + delay_time`（:91）；`target.predict(future)`（:86/93）；`choose_aim_point` 选板（:97-99），无效返回空；**弹道迭代**（:104-151）：初算 `Trajectory(bullet_speed, d0, z0)`（:106），不可解返回（:107-112）；复制 10 份 Target 迭代预测（:118），每次用上一轮 `fly_time` 预测未来位置、重选板、重算弹道（:120-134），`|fly_time - prev|<0.001` 收敛（:146-149）；**输出**（:154-163）：`yaw = atan2(y,x) - asin(clamp(camera_offset_y_/d, ±1)) + yaw_offset_`（:160-161，视差补偿推导在注释 :155-158），`pitch = -(current_traj.pitch + pitch_offset_)`（:162，世界系向上为负）。
- **优点：** 飞行时间迭代是"移动目标超前量"的标准做法；视差补偿有明确推导；弹速无效兜底 + 告警节流健壮。
- **隐患：** `max_predict_time_` 钳制意味着帧龄超限时瞄准点滞后于真实时刻（注释自认代价）；迭代上限 10 次固定，收敛阈值 0.001s 固定。

**aim(targets, timestamp, bullet_speed, shoot_mode, to_now)（aimer.cpp:166-183）**
- **作用：** 带枪位（左/右 shoot mode）的瞄准重载。
- **说明：** 按 shoot_mode 选左/右/默认 yaw 偏置，调主 aim 后把 `command.yaw` 从默认偏置换成选定偏置（:179-180）。
- **隐患：** 主 `aim` 已经加了 `yaw_offset_`，这里 `command.yaw - yaw_offset_ + yaw_offset` 是为了抵消重加，逻辑绕且依赖主函数实现细节，若主函数偏移计算变化会出错。

**choose_aim_point（aimer.cpp:185-259，选板逻辑）**
- **逐段说明：** 未跳变（`!target.jumped`）直接返回板 0（:191）；计算整车旋转中心 yaw（:194）与各板 `delta_angle = limit_rad(板角 - center_yaw)`（:198-201）；**小陀螺滞回**（:209-211）：`|w|>2.2` 进入、`<1.6` 退出 spin_mode（注释 :203-208 说明曾误用 r 判断致死代码、滞回防逐帧横跳）；非 spin 且非前哨站分支（:213-240）：选 |delta_angle|<60° 的板，多板时用 `lock_id_` 锁板防两板间来回切换（:227-235），单板时退出锁板（:238-239）；前哨站/小陀螺分支（:242-258）：前哨站用固定 70°/30° 进出角（:244-245），否则用配置角（:247-249）；小陀螺时选"即将进入射击区"的板（:252-256，`w>0` 且 delta_angle<leaving 时选）。
- **优点：** 锁板滞回 + 小陀螺滞回双滞回设计消除阈值抖动；三种场景分离清晰。
- **隐患：** `comming/leaving` 拼写（应为 coming/leaving）虽不影响功能但反映历史遗留；锁板 `lock_id_` 无超时重置，目标切换后可能残留旧 id（`set_target` 重建 Target 但 Aimer 的 `lock_id_` 是成员变量跨目标保留）。

---

### 文件: tasks/auto_aim/shooter.hpp / shooter.cpp

**总括：** 开火决策器。根据云台当前位置与目标指令的偏差、以及瞄准点有效性，决定是否扣动扳机（`shoot` 位）。

**成员（shooter.hpp:21-25）**
- **作用：** `last_command_`（上一帧指令）、`judge_distance_`（距离阈值切换容差）、`first_tolerance_/second_tolerance_`（近/远两级容差）、`auto_fire_`（自动开火总开关）。

**构造函数（shooter.cpp:10-17）**
- **作用：** 读容差（度→弧度）、距离阈值、auto_fire 开关。

**shoot（shooter.cpp:19-41）**
- **逐段说明：** 无控制、无目标或未开 auto_fire 时返回 false（:23）；按目标水平距离与 `judge_distance_` 选近/远容差（:27-29）；三个条件同时满足才开火：`|last_command_.yaw - command.yaw| < tolerance*2`（指令未突变）、`|gimbal_pos[0] - last_command_.yaw| < tolerance`（云台已接近目标）、`aimer.debug_aim_point.valid`（瞄准点有效）（:31-34）；无论开火与否都更新 `last_command_`（:35/39）。
- **优点：** 三级门控（控制有效 + 云台就位 + 指令稳定 + 瞄准有效）避免抖动期误射。
- **隐患：** 只用 yaw 通道（`gimbal_pos[0]`）做就位判定，pitch 未参与；`tolerance*2` 的"2 倍"是经验魔数；`last_command_` 初值为 `{false,false,0,0}`，首帧 yaw=0 与真实指令差可能触发条件 2 误判。

---

### 文件: tasks/auto_aim/voter.hpp / voter.cpp

**总括：** 一个极简的投票计数器，把 `(color, name, type)` 三维离散状态映射到一个一维计数数组，用于跨帧投票去抖（统计某类装甲板出现次数）。

**成员（voter.hpp:19-20）**
- **作用：** `count_`（长度 = `COLORS.size() * ARMOR_NAMES.size() * ARMOR_TYPES.size()` 的计数数组）。

**index（voter.cpp:17-20）**
- **作用：** 三维下标展平：`color * (NAMES*TYPES) + name * TYPES + type`。
- **说明：** 与 `armor_properties` 的 class_id 编码方式不同（独立一套展平规则），两者无对应关系。

**vote / count（voter.cpp:7-15）**
- **作用：** `vote` 对某状态 +1，`count` 返回累计次数。
- **优点：** 实现极简、无锁、O(1)。
- **隐患：** `index` 无边界检查，非法枚举值会越界；`count_` 无清零接口，需外部重建 Voter 实例才能重置；当前代码库中 Voter 似乎未被主链路调用（疑似预留/历史遗留），需确认是否死代码。

---

### 文件: tasks/auto_aim/multithread/commandgener.hpp / commandgener.cpp

**总括：** 指令生成与下发线程。生产者（跟踪线程）`push` 最新目标/时间/弹速/云台位置，独立消费线程以 ~500Hz 循环生成 `io::Command` 并 `board.send`，是自瞄管线与下位机的最后一道接口。

**成员（commandgener.hpp:44-66）**
- **作用：** `Input{targets_, t, bullet_speed, gimbal_pos}` 结构体；`shooter_/aimer_/plotter_` 引用；`send_`（std::function 包装 board.send）；`latest_`（optional 最新输入）、`mtx_/cv_`（互斥+条件变量）、`thread_`、`stop_/debug_`。

**构造函数（模板，commandgener.hpp:24-36）**
- **作用：** 模板化接收任意 BoardT（io::CBoard 走 SocketCAN、io::Gimbal 走 USB CDC），用 lambda 捕获 `board.send` 存入 `send_`，并启动 `generate_command` 线程。
- **优点：** 模板 + `std::function` 解耦下位机类型，注释明确 board 生命周期需长于本对象。

**析构（commandgener.cpp:16-24）**
- **作用：** 置 `stop_`、`notify_all`、join 线程，优雅退出。

**push（commandgener.cpp:26-33）**
- **作用：** 加锁更新 `latest_` 并 `notify_one`，只保留最新一帧（丢弃中间帧）。
- **说明：** "只取最新"策略保证消费线程不被积压帧拖累。

**generate_command（commandgener.cpp:35-89）**
- **逐段说明：** 循环：取 `latest_`（帧龄 <0.35s 才接受，:45，注释记录 6 流管线实测滞后 ~159ms、原 0.2s 裕量不足导致停止帧与瞄准帧交替下发云台抽搐）；有输入则 `aimer_.aim(...to_now=true)`（:53）、`shooter_.shoot` 打 shoot 位（:54）、算 `horizon_distance`（:55-59）；debug 时用 `plotter_.plot` 出 JSON（:60-68）；无输入或超时发 `{false,false,0,0}` 停止帧（:71）；`send_(command)`（:73）；1Hz 节流打日志（:78-86，age>0.35 时区分"帧龄超限"与"无目标"两类 mode=0）；`sleep 2ms`（:87，注释约 500Hz）。
- **优点：** 帧龄上限 + 过期发停止帧，是防止"检测停摆后云台继续执行旧指令"的关键保护；日志设计能区分停止帧的两种成因。
- **隐患：** `sleep_for(2ms)` 的实际循环频率受系统调度影响，非精确 500Hz；`plotter_.plot` 在 debug 分支里是同步调用，可能拖慢下发节奏。

---

### 文件: tasks/auto_aim/multithread/mt_detector.hpp / mt_detector.cpp

**总括：** 多线程 OpenVINO 推理检测器。用槽位池 + 两个线程安全队列（free/pending）实现"异步提交、FIFO 取结果"的生产者/消费者流水线，是整条管线实时性的关键优化。

**成员（mt_detector.hpp:43-73）**
- **作用：** `struct Slot{request, input, img, t}`（:49-55，注释说明请求和输入缓冲只建一次反复用，每帧 create_infer_request 走单请求延迟路径 44~57ms，复用走多流吞吐 21~26ms，是帧率上不去的主因）；`cfg_`（必须先于 pool_ 声明，:57）；`core_/compiled_model_/device_/perf_mode_/streams_`；`yolo_`（工厂实例）；`pool_`（槽位数组）、`free_/pending_`（空闲/待取槽位队列）、`dropped_`（丢帧计数）。

**构造函数（mt_detector.cpp:12-72）**
- **逐段说明：** 成员初始化顺序与声明一致（:13-23）；`device_` 缺省 CPU、`perf_mode_` 缺省 throughput、`streams_` 缺省 4（:16-19）；按 `yolo_name + "_model_path"` 读模型路径（:25-26）；PPP 配置 u8/NHWC/BGR/640（:32-44）；`ov::AnyMap props` 按 perf_mode 选 LATENCY/THROUGHPUT，`streams_>1` 时设 `num_requests`（:53-56，注释记录吞吐+多流从 61ms→20ms 约 3 倍）；预建 `streams_` 个请求与输入缓冲并绑定（:62-66）；free_ 队列填充所有槽位（:67）。
- **优点：** 注释详实地记录了延迟/吞吐两种模式的实测对比与选择依据。

**析构（mt_detector.cpp:74-84）**
- **作用：** 对每个槽位的请求 `wait()` 并捕获异常，避免推理进行中销毁 InferRequest。

**push（mt_detector.cpp:86-125）**
- **逐段说明：** 从 free_ 取空闲槽位，失败则丢帧（:97-102，注释说明试过"丢最旧在飞帧"反而更差 20.5→10.6fps，且 FIFO 滞后 = 流数×单帧周期，帧率与滞后对立只能选工作点）；letterbox 缩放直接写入槽位自己的输入缓冲（:106-116）；`s.img = img.clone()`、存时间戳、`start_async()`（:118-120）；塞入 pending_，塞不进则还回 free_（:124）。
- **优点：** 输入缓冲与请求绑定，预处理无额外拷贝；丢新帧而非阻塞采集线程。

**finish（mt_detector.cpp:127-143）**
- **作用：** 等槽位推理结束（`wait`）、取输出张量包成 cv::Mat、算 scale、调 `yolo_.postprocess`（:138，注释"暂不支持 ROI"）、后处理完才把槽位还回 free_。
- **说明：** 后处理完成前不还槽位，防止生产者覆盖输出张量。

**pop / debug_pop（mt_detector.cpp:145-160）**
- **作用：** `pop` 取槽位 → `finish` → 返回 `(armors, t)`；`debug_pop` 先取 img 再 finish（因 finish 会还槽位，img 可能被改写，:156-157），返回 `(img, armors, t)`。
- **隐患：** `output_shape` 未校验维度；`pop` 是阻塞的（`pending_.pop()`），注释在头文件 `has_pending` 里说明采集线程若先挂，消费线程会永远等——调用方需用 `has_pending` 判退出。

---

### 文件: tasks/auto_aim/planner/planner.hpp / planner.cpp

**总括：** 基于 TinyMPC 的云台控制规划器，是相对 `Aimer` 的并行控制方案。显式把云台动力学（角加速度上限）纳入 MPC 优化，生成整条 yaw/pitch 参考轨迹并追踪。

**常量与类型（planner.hpp:14-32）**
- **作用：** `DT=0.01`（控制步长）、`HALF_HORIZON=50`、`HORIZON=100`（时域 1s，±0.5s）；`using Trajectory = Eigen::Matrix<double,4,HORIZON>`（yaw/yaw_vel/pitch/pitch_vel）；`struct Plan`（下发字段：control/fire/target_yaw/target_pitch 及各状态量）。

**成员（planner.hpp:36-59）**
- **作用：** `yaw/pitch_offset_`、`fire_thresh_`、延迟/速度阈值、`default_bullet_speed_`、`TinySolver* yaw_solver_/pitch_solver_`（两个 2 状态 MPC 求解器）。

**构造函数（planner.cpp:13-27）**
- **作用：** 读配置 + `setup_yaw_solver` + `setup_pitch_solver`。

**setup_yaw_solver / setup_pitch_solver（planner.cpp:118-162）**
- **逐段说明：** 读 max_*_acc 与 Q/R；状态转移 `A={{1,DT},{0,1}}`、`B={{0},{DT}}`（角+角速度积分模型，输入是角加速度）；`tiny_setup(&solver, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0)`（:130/:153，rho=1.0、nx=2、nu=1、N=HORIZON）；`tiny_set_bound_constraints` 设输入约束 `u ∈ [-max_acc, +max_acc]`（状态约束放宽到 ±1e17，:132-136）；`max_iter=10`（:138/:161）。
- **说明：** 两个求解器结构相同，仅权重与加速度上限不同，分别控制 yaw 与 pitch 通道。

**plan(Target, bullet_speed)（planner.cpp:29-102，主规划）**
- **逐段说明：** 弹速校验（<10 或 >25 用默认值，:32-40）；选最近装甲板 `xyz`（:43-51）；`Trajectory` 求飞行时间并 `target.predict(fly_time)`（:52-53）；`aim` 求 yaw0 与 `get_trajectory` 得整条轨迹，异常则返回 `{false}`（:57-64）；设 yaw 求解器 x0 与 Xref 为轨迹前两行、`tiny_solve`（:66-72）；同理 pitch 求解器（:74-79）；取时域中点（HALF_HORIZON）的状态/输入填 Plan（:81-93）；`fire` 判定（:95-100）：预测 `HALF_HORIZON+2` 处的轨迹与 MPC 输出差 < `fire_thresh_` 时开火。
- **优点：** MPC 把加速度约束显式纳入，输出的是平滑可行的轨迹而非瞬时跳变指令；fire 判定基于"未来时刻轨迹与指令的贴近度"。

**plan(optional<Target>, bullet_speed)（planner.cpp:104-116）**
- **作用：** 无目标返回 `{false}`；否则按 |w| 选延迟 → 预测未来 → 调主 plan。

**aim（planner.cpp:164-185）**
- **作用：** 选最近装甲板，返回 `[azim + yaw_offset, -pitch - pitch_offset]` 期望角；弹道不可解抛异常。
- **说明：** 与 Aimer 的弹道计算同源（`tools::Trajectory`），但只求角不求迭代飞行时间。

**get_trajectory（planner.cpp:187-211）**
- **逐段说明：** 从 `-DT*(HALF_HORIZON+1)` 时刻起，每 DT 步 `target.predict` 并 `aim` 求期望角，用中心差分 `(yaw_pitch_next - yaw_pitch_last)/(2DT)` 求角速度，填入 `Trajectory`（:191-208）。
- **说明：** 生成 100 步的参考轨迹（yaw/pitch 及其速度）。

**关于捆绑的 TinyMPC（planner/tinympc/*）：** TinyMPC 是一个小巧的线性二次 MPC（模型预测控制）求解器库，被整体捆绑进 `planner/tinympc/` 目录。它求解的是形如"最小化 Σ(x-xref)ᵀQ(x-xref) + uᵀRu，约束 x_{k+1}=A x_k + B u_k + f 及 x/u 上下界"的凸二次规划。核心算法基于 ADMM（交替方向乘子法）：`admm.cpp` 里实现 `update_primal`（原变量更新，含 Riccati backward pass 的梯度步骤 `backward_pass_grad`）、`forward_pass`、`update_slack`、`update_dual` 等标准 ADMM 迭代，`project_soc`/`project_hyperplane` 处理二阶锥与线性约束的投影；`types.hpp` 定义 `TinySolver`（内含 `TinySolution`、`TinySettings`、`TinyCache`、`TinyWorkspace`）等核心结构；`codegen.cpp/hpp` 提供把求解器 C 代码生成到独立目录的能力（`tiny_codegen`，用于嵌入式部署）。对外 API 集中在 `tiny_api.hpp` 的 C 接口：`tiny_setup`（建求解器）、`tiny_set_bound_constraints`（上下界约束）、`tiny_set_x0`（初始状态）、`tiny_set_x_ref`（参考轨迹）、`tiny_solve`（执行求解）等，求解结果通过 `solver->work->x`、`solver->work->u` 取出。

**为什么集成 TinyMPC：** `Planner` 需要一个在 1s 时域、100 个节点上实时求解带输入约束（云台角加速度上限）的最优控制问题的轻量求解器，而通用求解器（如 OSQP/CVXGEN）或手写 LQR 要么集成过重、要么无法处理约束。TinyMPC 恰好以"极简 C API + ADMM 迭代 + 代码生成"满足了嵌入式/实时场景下"毫秒级解 2 状态 100 节点问题"的需求——这正是云台控制这类低维、高频、带约束问题的典型求解目标。它通过 `TinySolver*` 指针在 `Planner` 里被持有，`tiny_solve` 每次只跑 `max_iter=10` 次 ADMM 迭代，用有界次数的近似解换取确定性的低延迟。

---

## Auto Aim 整体优缺点总结

### 算法层面优点
- **检测-跟踪-解算-控制分层清晰**，`Armor`（帧级观测）→ `Target`（跨帧状态）→ `io::Command`（下位机输出）逐层抽象，各模块职责单一、可独立替换（三种 YOLO 通过工厂切换、传统检测可作二次矫正）。
- **工程鲁棒性打磨充分**：大量注释记录了 2026 年 10-11 月联调实测发现并修复的问题（IPPE 双解镜像翻转、近距抖动、假自转、roll 滑移、r 泵高误杀、帧龄超限云台抽搐等），创新门控 + 自适应 R + 假自转刹车 + 几何钳制 + 无死区 roll 拉力构成一套经过实战验证的防抖/防发散体系。
- **多线程推理流水线设计合理**：槽位池 + 双队列 + 复用 InferRequest 的吞吐模式，把帧率从 ~20ms 级压到 ~50fps 级，是实时性的关键。
- **MPC 路径（Planner）** 把云台加速度约束显式纳入优化，输出平滑轨迹，是相对传统"开环弹道+迭代"更现代的方案。

### 工程实现优点
- **配置外置**：相机内参、外参、几何阈值、延迟、容差、MPC 权重等大量参数 YAML 化，便于联调。
- **日志与可观测性强**：CommandGener 1Hz 日志区分停止帧成因，Target/Solver 逐帧打印门控标志，Aimer 弹速告警节流，debug 绘图齐全。
- **健壮的线程同步**：条件变量 + 互斥 + 帧龄上限 + 过期发停止帧，析构优雅 join。

### 共性问题 / 可改进点
- **硬编码魔数泛滥**：创新门控阈值（0.15/0.20/0.50/1e4/25/0.6/0.45…）、roll 门控（8°/60/300px/1.8m/2400）、图像中心 1440×1080、YOLOv5 设备 "CPU"、PnP 求解 IPPE、选板 60°/70°/30°、弹道 g=9.7833 等大量经验值散落在源码，未抽配置，换环境/相机后需逐一重标定。
- **单位换算不统一**：度转弧度既有 `/57.3` 近似也有 `*CV_PI/180.0`，两种写法混用；`ratio` 在传统路径（宽/长）与神经网络路径（长/宽）语义相反，易踩坑。
- **重复代码**：三个 YOLO 实现的后处理/几何校验/绘制大量复制粘贴；`Armor` 四个神经网络构造函数几何计算重复四份；`classifier` 的 cv::dnn 与 OpenVINO 两条路径逻辑重复且同时占用双份模型内存。
- **死代码与残留**：`SJTU_cost`、`lightbar_points_corrector`（被注释关闭）、`Voter`（疑似未接入主链路）、`ovclassify`、tracker 里注释掉的前哨站过滤与最左侧过滤、Aimer 里 `comming/leaving` 拼写，都增加维护负担。
- **健壮性隐患**：`cv::imshow`/`waitKey` 散落于热路径（Detector::detect 的 imshow 无 debug 保护）；`output_shape` 多处未校验直接包 cv::Mat；YOLOv5 的 `num_id>5` 越界访问枚举向量；分类器 `label_id` 直接强转枚举依赖隐式约定。
- **性能隐患**：单线程路径（YOLOv5/V8/11 的 `detect`）每帧 `create_infer_request` 走单请求延迟路径，与 MultiThreadDetector 的优化结论相悖；`optimize_yaw` 逐度暴力搜索 140 次含 projectPoints，开销可观。
- **一致性风险**：`get_type` 在 Detector 与 YOLOV8/11 里实现不同、阈值不同（一处按 ratio 一处按 name），同一目标在不同检测路径下可能得到不同 type 判定。

整体而言，这套 auto_aim 是典型的 RoboMaster 强队视觉栈：算法骨架（YOLO 检测 + EKF 跟踪 + PnP 解算 + 弹道补偿）标准，真正的护城河在于大量联调驱动的鲁棒性工程与多线程推理优化——但这些经验大量以注释和魔数形式固化在代码里，而非以配置和测试形式沉淀，是后续维护与新机器迁移的主要成本点。


---

# Auto Buff(能量机关)任务层分析

## 模块概述

`tasks/auto_buff` 是 sp_vision_25 视觉系统中负责"能量机关"(俗称大风车/能量机关,RoboMaster 比赛中一个带有 5 片扇叶、绕固定轴旋转的标靶)的完整任务层。它的核心目标是:在底盘/云台运动、相机延迟、子弹飞行时间的多重约束下,精确预测旋转扇叶在"未来某时刻"的空间位置,解算出云台 yaw/pitch 角,并在扇叶转到可被击打的角度时给出射击指令,从而激活能量机关。

整个模块遵循一条清晰的流水线:`yolo11_buff`(深度学习关键点检测)→ `buff_detector`(扇叶检测、目标扇叶识别、旋转中心估计、丢失管理)→ `buff_type`(数据结构:扇叶 FanBlade 与整体 PowerRune)→ `buff_solver`(PnP + 坐标系链求解,把像素换算到世界系)→ `buff_target`(EKF 状态估计 + RANSAC 正弦拟合角速度 + 旋转模型)→ `buff_aimer`(弹道解算、提前量补偿、射击决策)。数据以 `std::optional<PowerRune>` 为基本载体,一帧一帧地从检测端流到预测端,最终输出 `io::Command`/`auto_aim::Plan` 给下游云台与发射控制。

`buff_type` 定义了三个层次的数据结构:最底层 `FanBlade` 描述单片扇叶(6 个关键点、中心、角度、类型),中间层 `PowerRune` 描述整个能量机关(旋转中心、按目标扇叶排序的 5 片扇叶、在世界系的多种表示),顶层枚举 `PowerRune_type`(SMALL/BIG)区分大小能量机关、`FanBlade_type`(_target/_unlight/_light)标记扇叶状态、`Track_status`(TRACK/TEM_LOSE/LOSE)描述跟踪状态。`PowerRune` 构造函数承担了"目标扇叶识别"这一关键职责:根据上一帧的目标扇叶位置与当前帧扇叶集合,判断哪片是当前应击打的扇叶,并对 5 个槽位做归一化排序。

`yolo11_buff` 是对 OpenVINO 运行时的封装,加载 YOLO11 关键点检测模型,输出每个扇叶的包围盒和 6 个关键点(其中关键点 4 是扇叶中心)。它提供两个推理接口:多候选框(`get_multicandidateboxes`,配合 NMS)和单候选框(`get_onecandidatebox`,取置信度最高者),分别对应 24 帧能量机关(需识别多片扇叶)与单扇叶跟踪模式。

`buff_detector` 把 YOLO 输出转成 `FanBlade` 列表,并用传统视觉方法估计能量机关旋转中心 `r_center`:先由关键点几何关系算粗略中心,再通过灰度二值化 + 膨胀 + 圆形 mask + 轮廓 `minAreaRect` 精修。它还维护 `status_`/`lose_`/`last_powerrune_` 三个状态,实现连续 20 帧丢失(全局常量 `LOSE_MAX = 20`)后判为 LOSE 的鲁棒跟踪状态机。

`buff_solver` 负责几何标定与坐标变换。它从 YAML 读取 `R_gimbal2imubody`、`R_camera2gimbal`、`t_camera2gimbal`、内参 `camera_matrix`、畸变 `distort_coeffs`,用 `cv::solvePnP`(IPPE 方法)从 4 个目标扇叶关键点解出扇叶平面相对相机的位姿,再通过"相机→云台→IMU body→世界"的旋转链,输出 `PowerRune` 的 `xyz_in_world`/`ypd_in_world`/`ypr_in_world` 以及扇叶中心 `blade_xyz_in_world`/`blade_ypd_in_world`。其中"扇叶中心在 buff 系内偏移 0.7 m(沿 z 轴)"这一常量被反复硬编码在 solver、target、aimer 三处。

`buff_target` 是预测的核心。它定义了一个 EKF 状态估计框架:`Target` 抽象基类下派生出 `SmallTarget`(小能量机关,恒定角速度 π/3 rad/s)和 `BigTarget`(大能量机关,角速度随时间正弦变化)。两者都用 7 维(小)/10 维(大)状态向量同时估计"能量机关在世界系中的方位(球坐标 yaw/pitch/distance)、扇叶旋转角(roll)、角速度等"。`BigTarget` 额外内嵌了一个 `RansacSineFitter`,用 RANSAC 对 EKF 输出的角速度序列拟合 `A·sin(ωt+φ)+C` 的正弦模型,以捕捉大能量机关角速度的周期脉动。`Voter` 是一个投票器,通过比较相邻两帧旋转角大小判断旋转方向(顺时针/逆时针)。

`buff_aimer` 是最后一环:它调用 `Target::predict` 预测未来角度,用 `tools::Trajectory` 做弹道解算(含子弹飞行时间),做一次"预测→弹道→再预测→再弹道"的迭代收敛,计算提前量补偿后的 yaw/pitch;并根据"当前角度与上次角度差是否超阈值"来判断是否切换扇叶、何时开火,输出射击指令。`mpc_aim` 额外输出 yaw/pitch 的速度与加速度估计,供云台前馈/MPC 使用。

需要特别说明的是 Ceres、RANSAC 与卡尔曼三者的真实分工:本层**实际没有调用 Ceres**。仓库中 vendor 了 `ceres-solver-2.1.0`(见 git status),但 `RansacSineFitter` 内部的最小二乘是用 **Eigen 的 SVD(`bdcSvd`)** 求解的,不是 Ceres。Ceres 在本层属于"已引入但未使用"的依赖。RANSAC 只用于大能量机关的角速度正弦拟合,卡尔曼(EKF)则贯穿小/大两个 Target 的位姿与角度估计。

数据流转可以概括为:图像 → `YOLO11_BUFF::Object`(6 关键点)→ `FanBlade` 列表 → `PowerRune`(含目标扇叶、旋转中心)→ `Solver::solve` 填充世界系表示 → `Target::get_target` 更新 EKF/正弦拟合 → `Aimer::get_send_angle` 用 `point_buff2world` 与 `Trajectory` 算出 yaw/pitch → `io::Command`/`Plan`。`buff_predict.hpp` 是早期的一套独立预测类(小/大 Predictor、XYZ_predictor),逻辑与 `buff_target` 重复,当前主流程未使用,属遗留代码。

---

### 文件: tasks/auto_buff/buff_type.hpp / buff_type.cpp

本文件定义 auto_buff 层的基础数据结构(扇叶、能量机关整体)以及三个枚举,是整条流水线的数据契约。

**常量 `INF = 1000000`(buff_type.hpp:16)**
- **作用**:作为"无穷大/极大初始值",用于 `get_r_center` 中轮廓筛选、`PowerRune` 构造中最小距离比较等。
- **缺点/隐患**:用整型 1000000 冒充浮点无穷大,若某处把它直接参与浮点运算(如加一个距离)可能溢出或精度不足;语义上应使用 `std::numeric_limits<float>::max()`。命名过于笼统,易误用。

**枚举 `PowerRune_type { SMALL, BIG }`(buff_type.hpp:17)**
- **作用**:区分小/大能量机关,决定采用何种旋转预测模型(恒速 vs 正弦变速)。
- **说明**:本层代码中未直接看到该枚举被消费的地方(预测模型选择是通过 `SmallTarget`/`BigTarget` 两个子类完成的),存在"类型信息与实际调度逻辑分离"的隐患。

**枚举 `FanBlade_type { _target, _unlight, _light }`(buff_type.hpp:18)**
- **作用**:标记扇叶状态——`_target` 当前待击打目标、`_unlight` 未点亮的占位扇叶、`_light` 已点亮的普通扇叶。
- **缺点/隐患**:下划线前缀命名与 C++ 保留标识符风格易冲突(双下划线或下划线+大写才是保留,单下划线+小写合法但仍不推荐);枚举值无显式赋值,顺序隐式依赖 `_target=0`,`fanblades[0]` 作为目标扇叶的约定依赖此顺序。

**枚举 `Track_status { TRACK, TEM_LOSE, LOSE }`(buff_type.hpp:19)**
- **作用**:跟踪状态机三态(稳定跟踪/临时丢失/彻底丢失)。

**`class FanBlade`(buff_type.hpp:21-37)**
- **作用**:单片扇叶的数据结构。
- **关键成员**:
  - `cv::Point2f center`(24):扇叶中心像素坐标(即关键点 4)。
  - `std::vector<cv::Point2f> points`(25):6 个关键点,"从左上角开始逆时针"(注释)。
  - `double angle, width, height`(26):`angle` 为相对 `PowerRune::r_center` 的极角(由 `PowerRune` 构造填充),`width/height` 声明但本层未见赋值使用。
  - `FanBlade_type type`(27):类型标记。
- **构造函数 `FanBlade(kpt, keypoints_center, t)`(buff_type.cpp:8-13)**
  - **作用**:用一组关键点 + 中心 + 类型构造扇叶,把 `kpt` 全部拷入 `points`。
  - **缺点/隐患**:未校验 `kpt` 长度,后续 `points[4]`/`points[5]`/`points[2]` 硬编码索引(见 detector)依赖"恰好 6 个点"的隐含契约,一旦模型输出点数变化即越界。
- **构造函数 `FanBlade(FanBlade_type t)`(buff_type.cpp:15-18)**
  - **作用**:仅允许构造 `_unlight` 占位扇叶(用于把不足 5 片的扇叶集合补齐成 5 槽位),其它类型直接 `exit(-1)`。
  - **缺点/隐患**:用 `exit(-1)` 终止整个进程处理"错误类型"过于暴力,应抛异常或断言;生产代码中任何输入错误即崩溃。

**`class PowerRune`(buff_type.hpp:39-68)**
- **作用**:整个能量机关的一帧描述。
- **关键成员**:
  - `cv::Point2f r_center`(42):能量机关旋转中心(像素)。
  - `std::vector<FanBlade> fanblades`(43):按目标扇叶起、顺时针排列的 5 片扇叶(不足则用 `_unlight` 补齐)。
  - `int light_num`(45):本帧点亮扇叶数。
  - `Eigen::Vector3d xyz_in_world / ypr_in_world / ypd_in_world`(47-49):能量机关中心在世界系的直角坐标、欧拉角(ypr)、球坐标(ypd:yaw/pitch/distance)。
  - `Eigen::Vector3d blade_xyz_in_world / blade_ypd_in_world`(51-52):扇叶中心在世界系的直角坐标与球坐标。
  - `target()`(59):返回 `fanblades[0]` 作为当前目标扇叶。
  - `is_unsolve()`(61):返回 `unsolvable_`。
  - 私有 `target_angle_`(64)声明但未见使用;`unsolvable_`(65)标记该帧不可解。
- **构造函数 `PowerRune(ts, center, last_powerrune)`(buff_type.cpp:20-94)**
  - **作用**:给定本帧扇叶集合、旋转中心与上一帧 `PowerRune`,识别目标扇叶、填充角度、归一化排序。
  - **目标扇叶识别(24-71)**:
    - 只有 1 片(27):直接 `ts[0].type = _target`。
    - 扇叶数相同(29-42):把与上一帧目标扇叶中心**最近**的那片设为 `_target`,并用 `std::iter_swap` 把它换到 `ts[0]`。
    - 新亮起一片(44-65):对每片计算它到上一帧所有点亮扇叶的**最小距离**,取这些最小距离中**最大**的那个(即"最远离已知扇叶的新扇叶")作为目标——这是识别"新点亮扇叶"的合理启发式。
    - 其它(67-71):`unsolvable_ = true` 并记录日志。
  - **填充角度(75-79)**:以 `ts[0]` 的极角为基准,每片 `t.angle = atan_angle(t.center) - angle`,小于 -1e-3 则加 `2π`,使各扇叶相对目标扇叶的角度落在 [0, 2π)。
  - **归一化排序(83-93)**:按 `angle` 升序排序;然后把每片扇叶对齐到标准槽位角 `{0, 2π/5, 4π/5, 6π/5, 8π/5}`,若某片与标准角偏差小于 `π/5` 则填入,否则填 `_unlight` 占位。最终 `fanblades` 恒为 5 片且 `fanblades[0]` 是目标。
  - **优点**:目标扇叶识别逻辑清晰,5 槽位归一化使下游可稳定按 `fanblades[0]` 取目标。
  - **缺点/隐患**:
    - `light_num == 1` 分支(27)只处理 `ts` 大小为 1,但 30 行判断 `ts.size() == last.light_num`,44 行 `light_num == last.light_num + 1`,当 `ts.size()` 与 `last.light_num` 既不相等也不是 +1 且非 1 时直接判 `unsolvable_`,容错窗口窄。
    - 88-92 的槽位对齐存在隐患:若扇叶角度因噪声跨过槽位边界,可能错填或漏填;且循环 `for (int i=0,j=0; i<5 && j<ts.size(); i++)` 中当 `ts` 有 6 片(理论上不会,但若上游出错)会越界访问 `ts[j++]`。
    - `std::fabs(ts[j].angle - target_angles[i]) < CV_PI / 5` 用 `<` 而非 `<=`,边界值可能漏配。
- **`atan_angle(cv::Point2f point)`(buff_type.cpp:96-101)**
  - **作用**:返回 `point - r_center` 的极角,归一化到 [0, 2π)。
  - **缺点/隐患**:若 `point == r_center`,`atan2(0,0)` 返回 0,不会出错但语义上角度无定义;未做除零/零向量保护。

---

### 文件: tasks/auto_buff/buff_detector.hpp / buff_detector.cpp

本文件实现扇叶检测、旋转中心估计与跟踪状态机,把 YOLO 输出组织成 `PowerRune`。

**常量 `LOSE_MAX = 20`(buff_detector.hpp:12)**
- **作用**:连续丢失 20 帧后判为 `LOSE`。
- **说明**:定义在命名空间外(全局),命名空间污染;与 `buff_type.hpp` 的 `INF` 一样是裸全局常量。

**`class Buff_Detector`(buff_detector.hpp:15-38)**
- **作用**:检测器的门面,持有模型、跟踪状态与上一帧结果。
- **成员**:
  - `YOLO11_BUFF MODE_`(33):推理引擎。
  - `Track_status status_`(34):跟踪状态。
  - `int lose_`(35):连续丢失计数。
  - `double lastlen_`(36):声明但未使用(死成员)。
  - `std::optional<PowerRune> last_powerrune_`(37):上一帧结果,用于目标扇叶延续。
- **构造函数 `Buff_Detector(config)`(buff_detector.cpp:7)**
  - **作用**:初始化 `status_=LOSE, lose_=0`,并构造 `MODE_(config)`。

**`handle_img(bgr_img, dilated_img)`(buff_detector.cpp:9-25)**
- **作用**:彩色图转灰度 → `threshold(100, 255, BINARY)` 二值化 → 5×5 矩形核 `dilate` 膨胀。
- **逐行**:`cvtColor`(13)、`threshold`(18)、`getStructuringElement(MORPH_RECT, 5×5)` + `dilate`(22-23)。
- **优点**:膨胀用于连接扇叶发光区域,便于轮廓提取。
- **缺点/隐患**:二值化阈值 100 为**硬编码**,对不同光照/相机曝光不鲁棒;灰度直接二值化在强光或反光下极易失效。

**`get_r_center(fanblades, bgr_img)`(buff_detector.cpp:27-77)**
- **作用**:估计能量机关旋转中心像素坐标。
- **逐行/逐段**:
  - 空集检查(31-34):返回 `{0,0}`。
  - 粗略中心(38-45):对每片扇叶用 `point5 = points[4]`(扇叶中心)、`point6 = points[5]`,`r_center_t += (point6 - point5) * 1.4 + point5`,再取平均。注释标了 `// TODO`,且注释掉的公式 `4.7 * point - (4.7 - 1) * center` 说明这里用的是经验外推:从扇叶中心沿 `point5→point6` 方向外推 1.4 倍距离逼近圆心。
  - mask 精修(49-54):`radius = norm(points[2] - center) * 0.8`(用第 3 个关键点到中心的距离估计半径),画实心圆 mask,`bitwise_and` 只保留圆心附近区域。
  - 轮廓精修(60-75):`findContours(RETR_EXTERNAL, CHAIN_APPROX_NONE)`,对每个轮廓 `minAreaRect`,计算 `ratio = max/min 边长比 + 轮廓中心到粗心的距离/(radius/3)`,取 `ratio` 最小的轮廓中心作为最终 `r_center`。`ratio_1` 初值 `INF`。
- **优点**:几何先验 + 二值化 + 轮廓的组合能在模型关键点有噪声时给出较稳的圆心估计。
- **缺点/隐患**:
  - 硬编码系数 `1.4`、`0.8`、`1/3` 均无注释依据,标了 `TODO`,换相机/分辨率/目标距离后可能失效。
  - `fanblades[0].points[2]`(51)假设 `points` 至少有 3 个点且 `points[2]` 是扇叶某个外角点,依赖模型关键点语义。
  - 若 `radius` 过小(远距离)mask 可能覆盖不到轮廓,`findContours` 返回空,`ratio_1` 仍为 `INF`,`r_center` 退回 `r_center_t`,行为依赖粗心估计。
  - `ratio += norm(center - r_center_t)/(radius/3)` 若 `radius≈0` 会除零(实际 `radius` 来自关键点距离,通常非零,但极端情况下仍是隐患)。

**`handle_lose()`(buff_detector.cpp:79-87)**
- **作用**:丢失计数 `lose_++`;达到 `LOSE_MAX` 时置 `status_=LOSE` 并清空 `last_powerrune_`;否则置 `TEM_LOSE`。
- **缺点/隐患**:`status_` 在被置为 `TEM_LOSE` 后又无条件覆盖为 `TEM_LOSE`(86 行在 if 外),逻辑上先置 LOSE 再被 TEM_LOSE 覆盖,顺序正确但可读性差;且 `last_powerrune_` 只在真正 LOSE 时才清空,临时丢失期间仍沿用旧目标,属有意为之但易引起"目标扇叶错位延续"。

**`detect_24(bgr_img)`(buff_detector.cpp:89-123)**
- **作用**:24 帧能量机关检测(多候选框模式),返回 `std::optional<PowerRune>`。
- **逐行**:`get_multicandidateboxes`(93)→ 空则 `handle_lose` 返回 nullopt(97-100)→ 每个 Object 转 `FanBlade(_light)`(104-105)→ `get_r_center`(108)→ 构造 `PowerRune`(109)→ 不可解则 `handle_lose` 返回 nullopt(112-115)→ 置 `TRACK, lose_=0`,缓存 `last_powerrune_` 并返回(117-122)。
- **缺点/隐患**:`FanBlade(result.kpt, result.kpt[4], _light)` 用 `result.kpt[4]` 作中心,假设 kpt 至少有 5 个点,与 `get_multicandidateboxes` 的越界读取问题(见 yolo 小节)叠加会放大崩溃风险。

**`detect(bgr_img)`(buff_detector.cpp:125-160)**
- **作用**:单扇叶跟踪模式(单候选框),流程与 `detect_24` 几乎一致,只取 `results[0]`。
- **缺点/隐患**:与 `detect_24` 高度重复(90% 相同),仅"单框 vs 多框"和 `get_*candidatebox` 不同,可抽象公共模板;`results[0]` 在 results 非空时安全,但重复代码增加了维护成本。

**`detect_debug(bgr_img, v)`(buff_detector.cpp:162-193)**
- **作用**:调试用,`get_multicandidateboxes` 后在候选扇叶中选 `norm((center - r_center) - v) < 10` 或唯一候选的那片,构造 `PowerRune(…, std::nullopt)`(不传上一帧)。
- **缺点/隐患**:`< 10`(像素)硬编码;`std::nullopt` 使 `PowerRune` 构造只能走"单扇叶"分支(否则 unsolvable),因此该调试路径只支持单扇叶识别。

---

### 文件: tasks/auto_buff/buff_solver.hpp / buff_solver.cpp

本文件负责坐标标定与几何解算:从像素关键点用 PnP 求位姿,并沿坐标系链换算到世界系。

**常量 `THETA = 2.0 * CV_PI / 5.0`(buff_solver.hpp:15)**
- **作用**:单个扇叶槽位的角度间隔 72°,用于把基准扇叶的 3D 点旋转出其余 4 片扇叶的 3D 点(见 `compute_rotated_points`)。

**`class Solver`(buff_solver.hpp:17-65)**
- **成员**:
  - `camera_matrix_` / `distort_coeffs_`(35-36):相机内参与畸变(cv::Mat)。
  - `R_gimbal2imubody_`(37)、`R_camera2gimbal_`(38)、`t_camera2gimbal_`(39):标定外参。
  - `R_gimbal2world_`(40):由 IMU 姿态动态更新的"云台→世界"旋转。
  - `rvec_, tvec_`(42):最近一次 PnP 解出的旋转/平移向量(用于 `point_buff2pixel` 调试)。
  - `OBJECT_POINTS`(54-57):7 个 3D 点(单位米):`{0,0,827e-3},{0,127e-3,700e-3},{0,0,573e-3},{0,-127e-3,700e-3},{0,0,700e-3},{0,0,220e-3},{0,0,0}`。注释掉的旧定义(44-51)说明这套 3D 点历经调整。前 4 个点构成一个"菱形"扇叶轮廓,第 5 点是扇叶中心(0,0,700e-3),第 6/7 点是圆心与另一个参考点。
- **`rotation_matrix(angle)`(buff_solver.cpp:4-8)**
  - **作用**:返回绕 x 轴旋转 `angle` 的 3×3 旋转矩阵。
  - **说明**:能量机关的旋转轴即 x 轴(扇叶平面在 y-z 平面内)。
- **`compute_rotated_points(object_points)`(buff_solver.cpp:10-24)**
  - **作用**:把第 0 组 3D 点绕 x 轴旋转 `i*THETA` 得到其余 4 组 3D 点。
  - **说明**:该函数在构造函数中被注释掉(44 行 `// compute_rotated_points(OBJECT_POINTS);`),当前 `OBJECT_POINTS` 已改为 7 个点、不再需要旋转生成,属遗留。
- **构造函数 `Solver(config_path)`(buff_solver.cpp:26-45)**
  - **作用**:从 YAML 读 `R_gimbal2imubody`、`R_camera2gimbal`、`t_camera2gimbal`、`camera_matrix`、`distort_coeffs`,并 `eigen2cv` 转成 OpenCV 矩阵。`R_gimbal2world_` 初始为单位阵。
  - **优点**:外参/内参全部外部化到 YAML,便于标定替换。
  - **缺点/隐患**:YAML 读取用 `.as<std::vector<double>>()` 直接索引,缺字段会抛异常导致启动崩溃;未校验旋转矩阵是否正交、向量维度是否正确(如 3×3 用 9 个数、1×5 用 5 个数)。
- **`R_gimbal2world()` / `set_R_gimbal2world(q)`(buff_solver.cpp:47-53)**
  - **作用**:获取/设置"云台→世界"旋转。`set` 用四元数 `q`(IMU body→绝对世界)算出 `R_imubody2imuabs`,再 `R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_`。
  - **说明**:这里假设 IMU 固定在 body,云台相对 body 的旋转由 `R_gimbal2imubody_` 描述。
- **`solve(ps)`(buff_solver.cpp:55-110)**
  - **作用**:对 `PowerRune` 填充世界系位姿。
  - **逐行/逐段**:
    - `image_points = p.target().points`(72),再 `emplace_back(p.r_center)`(74)——即 6 个扇叶关键点 + 圆心共 7 个 2D 点,与 `OBJECT_POINTS` 的 7 个 3D 点对应。
    - 但 PnP 只用前 4 对:`image_points_fourth`(76)与 `OBJECT_POINTS_FOURTH`(77),`solvePnP(..., SOLVEPNP_IPPE)`(78-80)。**注意**:用 IPPE 且传入的是 4 个共面点,IPPE 会返回两个解(平面位姿歧义),代码未处理歧义,直接用其一。
    - `cv2eigen(tvec_, t_buff2camera)`(82-83)、`Rodrigues(rvec_, rmat)`(84-87)得到 R/t。
    - `blade_xyz_in_buff = {0,0,700e-3}`(89):扇叶中心在 buff 系的偏移,硬编码 0.7 m。
    - camera 系坐标(92-93)、camera→gimbal(96-98)、gimbal→world(101)、`ypd_in_world = xyz2ypd`(104)、blade 系同理(106-107)、`ypr_in_world = eulers(R_buff2world, 2,1,0)`(109)。
  - **优点**:坐标系链完整、命名清晰,IPPE 对共面四点场景求解速度与稳定性较好。
  - **缺点/隐患**:
    - 只用了 4 个关键点做 PnP,却多收集了圆心/其它点但不参与求解(74 行 emplace 的 r_center 实际未被 PnP 使用),存在"看似 7 点、实则 4 点"的误导。
    - IPPE 的双解歧义未处理,可能偶发跳到错误姿态。
    - `solvePnP` 的 `rvec_`/`tvec_` 是成员变量,`solve` 声明为 `const` 却修改成员(49 行 `set_R_gimbal2world` 也非 const),`solve` 用 `const` 修饰但写 `rvec_/tvec_` 属于逻辑上不纯(靠 cv 接口绕开 const),并发下非线程安全。
    - 0.7 m 硬编码与 `buff_aimer`/`buff_target` 中的 `0.7` 需保持一致,分散多处易改漏。
- **`point_buff2pixel(x)`(buff_solver.cpp:113-121)**
  - **作用**:调试用,把 buff 系 3D 点投影回像素,依赖最近一次 `rvec_/tvec_`。
- **`reproject_buff(xyz_in_world, yaw, row)`(buff_solver.cpp:124-148)**
  - **作用**:调试/可视化用,给定世界坐标和 yaw/roll,反推 buff→camera 的 R/t 并投影 `OBJECT_POINTS` 到图像。
  - **缺点/隐患**:参数名 `row` 实为 roll 的拼写;`rotation_matrix(Eigen::Vector3d(yaw, 0.0, row))` 里把 roll 放在第三位(ypr 顺序 yaw/pitch/roll),与 `eulers(...,2,1,0)` 的约定需仔细核对,易混淆。

---

### 文件: tasks/auto_buff/buff_aimer.hpp / buff_aimer.cpp

本文件是射击决策与弹道解算的最后一环,输出云台指令。

**`class Aimer`(buff_aimer.hpp:19-57)**
- **成员**:
  - `double angle`(32)、`double t_gap = 0`(33):`angle` 在 `get_send_angle` 中赋值(aimer.cpp:158/175)用于外部观测/调试;`t_gap` 声明未使用。
  - 私有:`target_`(36,一个 `SmallTarget` 实例,疑似未使用/残留)、`yaw_offset_`/`pitch_offset_`(37-38)、`fire_gap_time_`/`predict_time_`(40-41)、`mistake_count_`(43)、`switch_fanblade_`(44)、`last_yaw_`/`last_pitch_`(46-47)、`first_in_aimer_`(50)、`last_fire_t_`(52)。
- **构造函数 `Aimer(config_path)`(buff_aimer.cpp:9-18)**
  - **作用**:从 YAML 读 `yaw_offset`/`pitch_offset`(度→弧度 `/57.3`)、`fire_gap_time`、`predict_time`,初始化 `last_fire_t_`。
  - **缺点/隐患**:`/57.3` 是 `180/π` 的粗略近似(应为 57.29578),精度略差;未读 `target_` 相关配置,`target_` 成员疑似死代码。

**`aim(target, timestamp, bullet_speed, to_now)`(buff_aimer.cpp:20-67)**
- **作用**:主射击决策,返回 `io::Command{yaw, pitch, shoot, control}`。
- **逐行/逐段**:
  - 不可解则返回空指令(25)。
  - `bullet_speed < 10` 时兜底为 24(28)。
  - `detect_now_gap = delta_time(now, timestamp)`(32),`future = to_now ? (detect_now_gap + predict_time_) : 0.1 + predict_time_`(33)。
  - 36-37 行计算 `angle_changed`,**但此时 `yaw/pitch` 尚未初始化**(34 行 `double yaw, pitch;` 声明后未赋值)——`angle_changed` 读取了未初始化的局部变量,且该变量**从未被使用**,是典型的"未初始化读取 + 死代码"缺陷。
  - `get_send_angle(...)`(38)成功后:`command.yaw = yaw; command.pitch = -pitch;`(39-40),`mistake_count_ > 3` 时切换扇叶并 `control=true`(41-44),否则若角度变化超 5°(`5/57.3`)则 `switch_fanblade_=true, control=false`(45-48),否则 `switch_fanblade_=false, control=true`(49-53);随后更新 `last_yaw_/last_pitch_`(54-55)。
  - 射击决策(58-64):`switch_fanblade_` 为真则 `shoot=false` 并刷新 `last_fire_t_`;否则若距上次开火超过 `fire_gap_time_` 则 `shoot=true`。
- **优点**:提前量由 `future` 统一考虑(检测延迟 + 预测时间),开火由"扇叶是否切换"和"冷却时间"双重门控,思路清晰。
- **缺点/隐患**:
  - 未初始化 `yaw/pitch` 即参与 `angle_changed` 计算(UB,未使用但仍是缺陷,编译器可能告警)。
  - `5 / 57.3` 多次内联硬编码,应提为常量。
  - `future` 中 `0.1`(非 to_now 分支)是魔法数字。
  - 若 `get_send_angle` 返回 false,`yaw/pitch` 保持旧值但 `command` 仍为初始空值(不影响射击),但 `last_yaw_/last_pitch_` 不更新,下次对比会基于陈旧值。
- **`mpc_aim(target, timestamp, gs, to_now)`(buff_aimer.cpp:69-148)**
  - **作用**:在 `aim` 基础上额外输出 yaw/pitch 的速度与加速度(`auto_aim::Plan`),供云台 MPC/前馈。
  - **逐行/逐段**:
    - 89-90 行同样存在未初始化 `yaw/pitch` 参与 `angle_changed` 的问题(死代码)。
    - 113-136 行在 `plan.control` 为真时计算前馈:首次进入(`first_in_aimer_`)置零;否则 `dt = predict_time_`,`get_send_angle(target, predict_time_*-1, ...)` 回算"过去一个 predict_time"的角度,用数值差分估 yaw 速度/加速度(125-127)、pitch 速度/加速度(131-133)。
    - 其中 `plan.pitch_acc = (-pitch - gs.pitch - (gs.pitch + last_pitch_mpc)) / dt²`(133)这一行符号与括号疑似混乱,与 yaw 侧的对称公式不一致。
  - **缺点/隐患**:
    - 用"未来预测点 − 过去预测点 / (2dt)"做中心差分,但两个预测都基于同一 EKF 状态的**内部推进**,且 `predict` 会**就地修改** `ekf_` 状态(见 target),连续调用 `get_send_angle` 时状态被反复前推/回退,数值差分基准不一致,速度/加速度估计可能不准确。
    - `predict_time_` 同时充当差分步长,若配置为 0 则除零。
    - 与 `aim` 高度重复,扇叶切换/射击逻辑两处拷贝,维护易漂移。
- **`get_send_angle(target, predict_time, bullet_speed, to_now, yaw, pitch)`(buff_aimer.cpp:150-199)**
  - **作用**:核心解算——预测目标位置 + 弹道解算 + 迭代收敛,输出 yaw/pitch。
  - **逐行/逐段**:
    - `target.predict(predict_time)`(156)推进状态;`angle = target.ekf_x()[5]`(158)取旋转角(仅用于调试观测)。
    - `aim_in_world = target.point_buff2world({0,0,0.7})`(161),`d = sqrt(x²+y²)`、`h = z`(162-163)。
    - 第一次弹道:`Trajectory trajectory0(bullet_speed, d, h)`(166),不可解则返回 false(167-171)。
    - 用 `trajectory0.fly_time` 再预测目标(174),重算 `aim_in_world/d/h`(178-180),第二次弹道 `trajectory1`(181),不可解返回 false(182-186)。
    - 时间误差检查:`time_error = fly_time1 - fly_time0`,若 `|time_error| > 0.01`(10ms)返回 false(189-193)。
    - `yaw = atan2(y, x) + yaw_offset_`(196),`pitch = trajectory1.pitch + pitch_offset_`(197)。
  - **优点**:采用"预测→弹道飞行时间→再预测→再弹道"的固定点迭代,收敛即停止,能补偿子弹飞行期间目标继续旋转带来的角度差,是本层少数有明确收敛判据的模块。
  - **缺点/隐患**:
    - `predict` 会**就地修改** `target` 内部 EKF 状态(156、174 两次),调用后状态已被前推到 `predict_time + fly_time` 之外,若上层后续还要用 `target` 的原始状态会读到"被污染"的状态。`to_now` 参数在 `get_send_angle` 内并未被使用(签名有但函数体未引用),属于死参数。
    - `point_buff2world({0,0,0.7})` 的 0.7 m 与 solver/target 的 0.7 硬编码重复。
    - `fsqrt` 应是 `std::sqrt`(179 行写的是 `fsqrt`),若项目没有别名定义 `fsqrt` 会编译报错(需确认,此处原文为 `fsqrt(aim_in_world[0]*...)`,可能笔误)。

---

### 文件: tasks/auto_buff/buff_target.hpp / buff_target.cpp

本文件是预测核心:状态估计(EKF)+ 旋转方向投票 + 大能量机关角速度正弦拟合。

**`class Voter`(buff_target.hpp:20-29; buff_target.cpp:5-18)**
- **作用**:通过相邻两帧旋转角的增减投票判断旋转方向。
- **`vote(angle_last, angle_now)`(9-16)**:`|clockwise_| > 50` 时不再投票(防止计数溢出/收敛后不再变化);`angle_last > angle_now` 则 `clockwise_--`(判为逆时针),否则 `clockwise_++`。
- **`clockwise()`(18)**:`clockwise_ > 0 ? 1 : -1`,返回 +1(顺时针)/-1(逆时针)。
- **优点**:投票机制对单帧噪声鲁棒。
- **缺点/隐患**:阈值 50 硬编码;初值 `clockwise_=0` 时 `clockwise()` 返回 -1(默认逆时针),首帧方向依赖初始猜测;`angle_last > angle_now` 判断"逆时针"的前提是角度已在 [0,2π) 归一化,若未归一化会误判。

**`class Target`(抽象基类)(buff_target.hpp:33-67)**
- **作用**:定义 `get_target`/`predict`/`init`/`update` 四个纯虚接口,以及公共的 `point_buff2world`、`is_unsolve`、`ekf_x`。
- **成员**:
  - `spd = 0`(49):调试用公开成员。
  - 保护成员:`x0_/P0_/A_/Q_/H_/R_`(56-61,`H_` 在子类中声明尺寸但实际更新时用的是局部 `H1/H2`,未走 `H_`)、`ekf_`(62,`tools::ExtendedKalmanFilter`)、`lasttime_`(63)、`voter`(64)、`first_in_`(65)、`unsolvable_`(66)。
- **构造函数 `Target()`(buff_target.cpp:22)**:`first_in_(true), unsolvable_(true)`。
- **`point_buff2world(point_in_buff)`(buff_target.cpp:24-39)**
  - **作用**:把 buff 系坐标(如扇叶中心 0.7 m)变换到世界系。
  - **逐行**:`R_buff2world = rotation_matrix({ekf_.x[4], 0, ekf_.x[5]})`(27-28,注意 pitch 恒 0,只用 yaw=x[4]、roll=x[5]);`R_yaw=x[0]`、`R_pitch=x[2]`、`R_dis=x[3]`(30-32);世界平移 `R_dis*(cos R_pitch cos R_yaw, cos R_pitch sin R_yaw, sin R_pitch)`(33-37)。
  - **说明**:这里把能量机关中心的球坐标(yaw=x[0], pitch=x[2], dis=x[3])反解为直角坐标,再叠加扇叶相对中心的旋转偏移。
  - **缺点/隐患**:`R_buff2world` 的 pitch 硬编码为 0,忽略了云台俯仰对扇叶朝向的影响;`unsolvable_` 时返回 `{0,0,0}`(26)会污染下游。
- **`is_unsolve()`(41)**、**`ekf_x()`(43)**:返回 `unsolvable_` 与 `ekf_.x`。

**`class SmallTarget`(buff_target.hpp:71-90)**
- **作用**:小能量机关(恒定角速度)的状态估计。
- **常量 `SMALL_W = CV_PI / 3`(88)**:小能量机关角速度约 1.047 rad/s(60°/s)。注释掉 `= 0` 说明曾考虑静止。
- **`get_target(p, timestamp)`(buff_target.cpp:49-90)**
  - **作用**:一帧入口,处理初始化、丢失、更新与发散检测。
  - **逐行**:无 `p` 则 `unsolvable_=true, lost_cn++`(53-58);`static start_timestamp`(60)作为时间基准,`time_gap = delta_time(timestamp, start_timestamp)`(61);首次 `first_in_` 时 `init` 并置 `unsolvable_=true`(64-68);`lost_cn > 6` 判丢失重置(71-77);正常则 `update`(79-81);发散检测:`|ekf_.x[6]| > SMALL_W ± π/18` 判发散并 `first_in_=true`(84-89)。
  - **缺点/隐患**:
    - `static int lost_cn` 与 `static start_timestamp` 用**静态局部变量**,意味着多个 `SmallTarget` 实例共享同一计数/起点(若同一进程创建多个实例会互相干扰),且 `start_timestamp` 只在首次调用时绑定。
    - `lost_cn > 6` 与 detector 的 `LOSE_MAX=20` 不一致,两处阈值分属不同层,需协调。
    - `time_gap` 基于 `start_timestamp` 而非上一帧,长时间运行后 `time_gap` 很大,`dt` 也随之变大,可能使 EKF 过程噪声矩阵发散。
- **`predict(dt)`(buff_target.cpp:92-126)**
  - **作用**:7 维状态的一步预测。
  - **状态向量**(注释 96-102):`[R_yaw, v_R_yaw, R_pitch, R_dis, yaw, roll, spd]`。
  - **A_**(96-102):近匀速模型,`roll += dt * spd`(第 5 行、第 6 行),`spd` 保持不变。
  - **Q_**(104-115):只给 `R_yaw`/`v_R_yaw` 注入了 `v1=0.001` 的连续加速度过程噪声(其余为 0),表示"中心方位允许缓慢变化,旋转角/速度被当作确定量"。
  - **f lambda**(117-124):`A_*x` 后对角度项 `x[0]/x[2]/x[4]/x[5]` 做 `limit_rad` 归一化。
  - **缺点/隐患**:过程噪声只加到 R_yaw 分量,`roll/spd` 噪声为 0,一旦初始方向或 `SMALL_W` 估错,滤波器无法通过过程噪声自适应,可能长期锁定在错误角速度;`dt` 为 0 时 A_ 退化为单位阵,行为尚可但物理意义弱。
- **`init(nowtime, p)`(buff_target.cpp:128-183)**
  - **作用**:初始化 7 维 EKF。
  - **逐行**:`x0_ << p.ypd_in_world[0], 0, p.ypd_in_world[1], p.ypd_in_world[2], p.ypr_in_world[0], p.ypr_in_world[2], SMALL_W * voter.clockwise()`(150-152);`P0_` 对角为 `{10,10,10,10,10,10,1e-2}`(154-160,角速度初值方差极小);`x_add` lambda 对角度项 `limit_rad`(173-180);构造 `ekf_`(182)。
  - **缺点/隐患**:角速度直接硬编码为 `SMALL_W * 方向`,未从测量估计初值,若方向投票错误则初期严重发散;`P0_` 对 spd 只给 1e-2(极自信),抑制了初期的修正能力。
- **`update(nowtime, p)`(buff_target.cpp:185-301)**
  - **作用**:两阶段量测更新:先更新"中心方位 + roll"的线性观测,再更新"扇叶中心"的非线性观测。
  - **逐行/逐段**:
    - 扇叶跳变处理(199-207):若 `|ypr[2] - x[5]| > π/12`,在 `x[5] + i*2π/5 (i=-5..5)` 中找与 `ypr[2]` 最接近的,给 `x[5]` 加上对应整数倍槽位角,消除"扇叶编号跳变"导致的 ±72° 突变。
    - 方向投票(210-211):`voter.vote(x[5], ypr[2])`,若方向与 `spd` 符号相反则 `spd *= -1`。
    - `predict(nowtime - lasttime_)`(214)。
    - 第一阶段:H1 为 4×7 观测矩阵(232-237,观测 `R_yaw/R_pitch/R_dis/roll`),R1 对角 `{0.01,0.01,0.5,0.1}`(239-244),`z_subtract1` 对角度项 `limit_rad`(248-254),`z1={R_ypd[0],R_ypd[1],R_ypd[2],ypr[2]}`(256),`ekf_.update`(258)。
    - 第二阶段:H2 = `h_jacobian()`(267,3×7),R2 对角 `{0.01,0.01,0.5}`(269-273),`h2` lambda 把状态映射到扇叶中心球坐标(277-284),`z_subtract2` 对角度 `limit_rad`(287-292),`z2={B_ypd[0],B_ypd[1],B_ypd[2]}`(294),`ekf_.update`(296)。
  - **优点**:分两阶段更新,先稳中心方位再约束扇叶中心,利用了 `blade_ypd_in_world` 提供的信息;扇叶跳变处理是能量机关旋转角估计的关键技巧(否则 72° 跳变会毁掉角度滤波)。
  - **缺点/隐患**:
    - `h2` lambda 内部 `R_xyz`、`R_xyz_and_yr`(278、280)被构造却未使用,`point_buff2world` 内部又读取 `ekf_.x` 而非传入的 `x`(见 `point_buff2world` 直接用成员 `ekf_.x`),**观测函数 h2 实际不依赖入参 `x`**,这使得 EKF 的雅可比/残差链不一致——`h2(x)` 应只依赖 `x`,但它隐式依赖 `ekf_.x` 当前值,会破坏 EKF 的数学假设。
    - `h_jacobian` 与 `h2` 手写解析雅可比,复杂且易与 `h2` 实现脱节(数值/解析不一致风险)。
- **`h_jacobian()`(buff_target.cpp:303-353)**
  - **作用**:计算"状态→扇叶中心球坐标"的解析雅可比 H3·H2·H1·H0。
  - **逐行**:H0(308-314,5×7 选择矩阵)、`H_ypd2xyz = ypd2xyz_jacobian`(317)、H1(318-324,5×5)、H2(333-337,由 `yaw=x[4]`、`roll=x[5]` 及 0.7 m 偏置构成的旋转对角度偏导)、`H3 = xyz2ypd_jacobian`(340)、返回 `H3*H2*H1*H0`(343)。
  - **缺点/隐患**:手写链式雅可比极易出错;`H2` 中 0.7 硬编码;与 `h2` 内部"再调 `point_buff2world`(又读取成员 `ekf_.x`)"形成双重不一致风险。

**`class BigTarget`(buff_target.hpp:94-114)**
- **作用**:大能量机关(角速度正弦变化)的状态估计。
- **成员 `spd_fitter_`(111,`RansacSineFitter`)**:对 EKF 角速度做正弦拟合;构造时 `(100, 0.5, 1.884, 2.000)`(buff_target.cpp:357)——最大迭代 100、内点阈值 0.5、角频率范围 [1.884, 2.000] rad/s。`fit_spd_`(113)存拟合角速度。
- **`get_target(p, timestamp)`(359-401)**:结构与 `SmallTarget::get_target` 相同,发散检测为 `x[7]`(振幅 a)与 `x[8]`(角频率 w)超出 `[0.78/1.5, 1.045*1.5]`、`[1.884/1.5, 2.0*1.5]`(394-396)。
- **`predict(dt)`(403-459)**
  - **作用**:10 维状态一步预测。
  - **状态向量**:`[R_yaw, v_R_yaw, R_pitch, R_dis, yaw, roll, spd, a, w, fi]`。
  - **A_**(413-422):`roll += voter.clockwise()*dt`(第 5 行→roll 行),`spd` 行填 `sin(wt+fi)-1, t·a·cos(wt+fi), a·cos(wt+fi)`(419)——把 spd 建模为 `a·sin(wt+fi)+C` 的导数项。
  - **f lambda**(447-456):显式做非线性状态推进:`roll += direction * (-a/w·cos(wt+fi) + a/w·cos(w·lasttime+fi) + (2.09 - a)·dt)`(452-453),`spd = a·sin(wt+fi) + 2.09 - a`(454)。这里 **2.09** 是大能量机关角速度的均值(rad/s),`a` 是正弦振幅、`w` 角频率、`fi` 初相。
  - **缺点/隐患**:
    - A_ 与 f lambda **两套模型并存**:A_ 是线性化近似,`ekf_.predict(A_, Q_, f)` 内部通常用 A_ 算协方差、用 f 算状态均值(取决于 EKF 实现),若 `tools::ExtendedKalmanFilter` 同时使用 A_ 和 f,二者必须一致;这里 A_ 的 spd 行(419)与 f 的 spd(454)形式不同,存在模型不一致隐患。
    - `2.09`、`1.884/2.000`、`0.78/1.045` 等硬编码散落,与规则参数耦合。
    - `w` 出现在 `-a/w` 等分母,若 `w≈0`(初始化或发散)会除零;`x[8]` 有下限保护但预测阶段未防护。
- **`init(nowtime, p)`(461-525)**
  - **作用**:初始化 10 维 EKF。
  - **逐行**:`x0_ << p.ypd_in_world[0], 0, p.ypd_in_world[1], p.ypd_in_world[2], p.ypr_in_world[0], p.ypr_in_world[2], 1.1775, 0.9125, 1.942, 0`(488-490);`P0_` 对角 `{10,10,10,10,10,10,100,10,10,400}`(492-501,spd 方差 100、fi 方差 400 表示初相高度不确定);`x_add` 对角度项 limit_rad(514-522);构造 `ekf_`(524)。
  - **缺点/隐患**:`1.1775/0.9125/1.942` 是经验初值(均值 2.09、振幅 0.9125 等),注释 `std::atan((spd-2.09)/0.9125 + 1` 表明初相本想反解但未实现,直接置 0。
- **`update(nowtime, p)`(527-660)**
  - **作用**:两阶段量测更新(同 SmallTarget,但状态 10 维),并在最后做 RANSAC 正弦拟合。
  - **逐行/逐段**:扇叶跳变(544-552)、方向投票(555)、`predict`(560)、第一阶段 H1/R1/z1/update(578-604)、第二阶段 H2/h2/z2/update(613-642);随后(644-654):`if (ekf_.x[6] < 2.1 && ekf_.x[6] >= 0) spd_fitter_.add_data(nowtime, ekf_.x[6])`(645),`spd_fitter_.fit()`(646),`fit_spd_ = spd_fitter_.sine_function(nowtime, A, omega, phi, C)`(648-650),`spd = voter.clockwise()*(x[5]-anglelast)/(nowtime-lasttime_)`(652,仅调试),`spd = fit_spd_`(653),`|spd|>4` 时置 0(654)。
  - **优点**:把 EKF 估计的角速度再喂给 RANSAC 正弦拟合,用拟合值覆盖下一帧 `predict` 的 `spd`,能在角速度周期脉动时给出更平滑的预测。
  - **缺点/隐患**:
    - `fit_spd_` 被写但 `predict` 内 `spd = fit_spd_`(406)用的是**上一帧**拟合结果,而拟合参数 A/ω/φ 本身未直接进 EKF 状态,信息闭环存在一拍滞后。
    - `nowtime - lasttime_`(652)若为 0 会除零(该行仅调试但仍是隐患)。
    - `spd_fitter_` 只在 `x[6] ∈ [0, 2.1)` 时喂数据,角速度异常/负值时数据断流,拟合可能停滞。
- **`h_jacobian()`(662-712)**:与 SmallTarget 结构相同但 H0 为 5×10,返回 `H3*H2*H1*H0`(3×10)。注释 702 行仍写 `3*7`,是复制粘贴遗留的笔误。

---

### 文件: tasks/auto_buff/buff_predict.hpp

本文件是一套**独立的、早期/备用的预测实现**,与 `buff_target` 逻辑重叠。它不在 `auto_buff` 命名空间内,且当前主流程(`buff_aimer` 只使用 `buff_target` 的 Target 子类)未见引用,判断为遗留/备选代码。

**常量 `SMALL_W = CV_PI / 3`(buff_predict.hpp:13)**
- **作用**:与 `SmallTarget` 一致的小能量机关角速度。
- **缺点/隐患**:与 `SmallTarget::SMALL_W`(buff_target.hpp:88)重复定义,两处若改漏会不一致。

**`class Predictor`(16-38)**
- **作用**:预测器抽象基类,接口 `update/predict/is_unsolve/getX_best`,成员 `x0/P0/A/Q/H/R/ekf/X_best/lastangle/lasttime/cw_ccw/first_in/unsolvable`。
- **说明**:`cw_ccw` 注释"逆时针-1 顺时针1",与 `Voter` 的 `clockwise()` 语义(+1 顺时针/-1 逆时针)方向约定需对照。

**`class Small_Predictor`(40-138)**
- **作用**:小能量机关恒定角速度预测(1 维 EKF 估角度,速度固定 `SMALL_W`)。
- **构造函数(43-67)**:`x0<<0`,P0=1,A=1,Q=0,H=1,R=0.05,构造 EKF。
- **`update(angle, nowtime)`(69-125)**:首帧初始化(75-81);扇叶跳变处理(84-92);方向投票(95-100);`deltatime = nowtime-lasttime`,`B = SMALL_W*(cw_ccw>0?1:-1)`,`predict` 用 `A*x + deltatime*B`(103-107);`X_best = ekf.update(z, H, R)`(112);PlotJuggler 绘图(118-122)。
- **`predict(delta_time)`(127-133)**:返回 `±SMALL_W*delta_time`,即角度增量(不含初始角)。
- **缺点/隐患**:只对角度做 1 维卡尔曼、角速度完全硬编码,无法估计/修正速度;与 `SmallTarget`(7 维)相比是大幅简化的旧实现。

**`class Big_Predictor`(140-277)**
- **作用**:大能量机关正弦变速预测(5 维:angle/spd/a/w/theta)。
- **构造函数(143-189)**:`x0 << 0, 1.1775, 0.9125, 1.942, 0`,P0/Q/H/R 初始化,Q 对角 `{0.03,0.05,0.01,0.01,0.05}`。
- **`update(angle, nowtime)`(191-262)**:首帧(194-201);扇叶跳变(204-213);方向投票(216-221);`A` 矩阵(228-234)与 `f` lambda(235-241)推进状态;`ekf.update`(246);PlotJuggler 绘图(253-261,注意 `#ifndef PLOTJUGGLER` 才绘图,与 Small_Predictor 的 `#ifdef` 相反,疑似笔误)。
- **`predict(delta_time)`(264-272)**:返回角度增量 `±(-a/w·cos(θ+w·dt) + a/w·cosθ + (2.09-a)·dt)`,即角速度 `a·sin(θ+w·t)+2.09-a` 的积分。
- **缺点/隐患**:`-a/w`、`a/(w²)` 等项当 `w≈0` 除零;`cw_ccw` 方向与 `2.09` 均值硬编码;A 矩阵第 2 行(228-230)与 f lambda(237-239)的积分式需严格一致,否则协方差与均值脱节。

**`class XYZ_predictor`(279-343)**
- **作用**:对 XYZ 直角坐标做 3 维线性卡尔曼(匀速模型)。
- **`kalman(XYZ)`(314-339)**:首帧初始化(316-322),否则 `predict(A,Q)`(324)、`update(z,H,R)`(329),PlotJuggler 绘图(331-337),最后 `XYZ = X_best`(338)。
- **缺点/隐患**:Q 全 0、A 单位阵(无速度项),本质是"低通滤波"而非预测;`first_in` 时直接改 `ekf.x` 而不更新协方差,与 EKF 语义不符。

---

### 文件: tasks/auto_buff/yolo11_buff.hpp / yolo11_buff.cpp

本文件封装 OpenVINO 推理,加载 YOLO11 关键点检测模型并解析输出。

**常量 `ConfidenceThreshold = 0.7f`、`IouThreshold = 0.4f`(yolo11_buff.cpp:3-4)**
- **作用**:置信度与 NMS 的 IoU 阈值。

**`class_names = {"buff", "r"}`(yolo11_buff.hpp:13)**
- **作用**:类别名(声明但本层解析未按类别区分,仅用 `det_output` 第 4 行 score)。

**`struct Object`(yolo11_buff.hpp:18-24)**
- **作用**:单个检测结果:`rect`(Rect_<float>)、`label`(int)、`prob`(float)、`kpt`(vector<Point2f>)。

**`NUM_POINTS = 6`(yolo11_buff.hpp:40)**
- **作用**:关键点数。**但这是两个模型共用的常量,而两个模型输出关键点数不同**(见下文),是重要缺陷来源。

**构造函数 `YOLO11_BUFF(config)`(yolo11_buff.cpp:7-20)**
- **作用**:读 YAML 的 `model` 路径,`read_model`、`compile_model(…,"CPU")`、`create_infer_request`、`get_input_tensor` 并 `set_shape({1,3,640,640})`(NCHW)。
- **缺点/隐患**:固定 CPU 推理;`set_shape` 在构造时绑定 NCHW {1,3,640,640},而 `get_multicandidateboxes` 却尝试用 NHWC {1,640,640,3} u8,两者不一致(见下)。

**`get_multicandidateboxes(image)`(yolo11_buff.cpp:22-145)**
- **作用**:多候选框检测(24 帧能量机关)。
- **逐行/逐段**:
  - 预处理(35-49):`x_scale=640/rows`、`y_scale=640/cols`、`scale=min`(37-39),`factor=scale`(43);新建 `cv::Mat(640,640,CV_8UC3)` 并把图像 resize 进左上角 roi(46-48);`ov::Tensor input_tensor(u8, {1,640,640,3}, input.data)`(49)——**局部变量遮蔽成员**。
  - `infer_request.infer()`(52)。
  - 输出解析(55-99):`output_shape[1]=out_rows`(注释 15 行)、`output_shape[2]=out_cols`(8400);遍历每列,`score=det_output(4,i)`,超阈值则取 cx/cy/ow/oh(72-75)、box 坐标乘 `factor`(77-80)、关键点 `rowRange(NUM_POINTS, 15)` 逐点乘 `factor`(88-97)。
  - NMS(102-103)、组装 Object(106-118)、绘图(121-134)、FPS(136-139)。
- **优点**:对多目标场景用 NMS,能同时识别多片扇叶。
- **缺点/隐患(多处真实缺陷)**:
  - **输入张量未生效**:局部 `ov::Tensor input_tensor(...)` 只被构造,**从未 `infer_request.set_input_tensor(...)`**,`infer()` 实际使用的是构造函数里 `get_input_tensor()` 得到的成员张量(其数据**从未被写入**)。因此该路径推理的是空/未初始化缓冲,结果不可信。
  - **布局/类型不一致**:成员张量在构造时被设为 NCHW {1,3,640,640}(默认 float),而局部张量是 NHWC u8 {1,640,640,3},即便 set 进去也与模型期望不符。
  - **`factor` 方向错误**:`factor = scale`(43),但关键点/框坐标应乘 `1/scale` 才能从 640 空间映射回原图(对比 `get_onecandidatebox` 用 `fill_tensor_data_image` 返回 `1/scale`)。此处会把坐标缩放到错误尺度。
  - **关键点数不匹配/越界**:该模型输出 `out_rows = 15`(4 bbox + 1 score + 5 关键点×2),但 `NUM_POINTS=6`,`rowRange(6,15)` 只有 9 个元素,而循环读 `j*2+1` 最大到 11,**越界读取**;且应从 `rowRange(5,15)` 读 5 个点,却读 6 个。
  - 注释与代码矛盾(66 行写"15 = 4+1+NUM_POINTS*2"却得到 17,"5*2关键点"与 NUM_POINTS=6 冲突)。
- **`get_onecandidatebox(image)`(yolo11_buff.cpp:147-231)**
  - **作用**:单候选框检测(取置信度最高),用于主跟踪 `detect`。
  - **逐行/逐段**:
    - `factor = fill_tensor_data_image(input_tensor, image)`(152,返回 `1/scale`)。
    - `infer()`(156);输出 17 行(4+1+6×2)、8400 列(163-164);遍历找最大置信度(170-178);超阈值则取框(183-190)、关键点 `rowRange(5, 5+NUM_POINTS*2)`(194,即 12 行 6 点);`if (max_confidence < 0.7) save(...)`(203);绘图(206-222);FPS(226-229)。
  - **优点**:输入路径正确(letterbox + `fill_tensor_data_image`),单框取最大置信度简单可靠。
  - **缺点/隐患**:
    - 203 行 `max_confidence < 0.7` 在 `max_confidence > 0.7`(180)分支内**恒为 false**,`save` 永远不触发,注释"0.3-0.7 save"表明意图是保存低置信度样本,但阈值写错,属死代码。
    - 无 NMS,若场景出现多个高置信框只取一个,可能取错扇叶。
- **`convert(input, output, normalize, BGR2RGB)`(233-239)**
  - **作用**:`convertTo(CV_32F)`、可选 `/255` 归一化、可选 `BGR2RGB`。
- **`fill_tensor_data_image(input_tensor, input_image)`(241-278)**
  - **作用**:letterbox 缩放(保持宽高比),按 scale 是否 <1 选择"先缩放后换通道"或"先换通道后缩放"(256-264),再逐像素按 CHW 填入 float 张量(267-276),返回 `1/scale`。
  - **优点**:正确处理了 HWC→CHW、归一化、RGB 顺序与 letterbox 反缩放因子。
  - **缺点/隐患**:`1/scale` 作为"反缩放因子"正确,但调用方(`get_multicandidateboxes`)却用 `scale`,语义易混;三通道 fill 三重循环慢(应转成 `cv::split`/直接 memcpy 布局),影响帧率。
- **`printInputAndOutputsInfo(network)`(280-311)**
  - **作用**:打印模型输入/输出名、类型、形状(调试)。
- **`save(programName, image)`(313-321)**
  - **作用**:把图像存到 `../result/`。
  - **缺点/隐患**:相对路径 `../result/` 依赖进程当前工作目录,部署时易写到意外位置;无并发文件名保护(programName 用 `start` tick 计数,基本唯一但非绝对)。

---

## Auto Buff 整体优缺点总结

**优点**
1. **模块分层清晰、职责单一**:检测(`detector`)、数据结构(`type`)、几何解算(`solver`)、预测(`target`)、决策(`aimer`)各司其职,数据以 `PowerRune` 为载体单向流转,易于理解与调试。
2. **标定/参数外部化**:内参、外参、偏移角、射速、时间等大量从 YAML 读取(`solver`/`aimer`),便于换机标定与调参。
3. **旋转角处理的工程经验到位**:扇叶 72° 跳变消除(`x[5] += i·2π/5`)、Voter 方向投票、5 槽位归一化排序,都是解决"旋转标靶角度估计"的关键技巧,体现了对问题的深刻理解。
4. **预测链完整**:EKF(位姿+角度)+ RANSAC 正弦拟合(大符角速度脉动)+ 弹道固定点迭代(子弹飞行时间收敛),覆盖了"旋转运动预测 + 弹道提前量"两个核心难点。
5. **几何正确性较好**:坐标系链(相机→云台→IMU body→世界)与球坐标/欧拉角转换借助 `tools` 库,`xyz2ypd`/`ypd2xyz` 及雅可比封装统一,降低手写出错概率。
6. **鲁棒状态机**:`TRACK/TEM_LOSE/LOSE` 三态 + 连续丢失计数,避免单帧噪声导致目标频繁丢失。

**共性问题**
1. **魔法数字/硬编码大量存在且分散**:0.7 m(扇叶中心偏移)在 solver/target/aimer 三处重复;2.09、1.884/2.000、0.78/1.045、π/3、`5/57.3`、`*1.4`、`*0.8` 等散落各处,改参数时极易漏改导致不一致。
2. **多套重复/遗留代码**:`detect_24` 与 `detect` 高度重复;`buff_predict.hpp` 与 `buff_target` 功能重叠且疑似未使用;`H_` 成员声明后未使用;`target_`/`lastlen_`/`t_gap`/`target_angle_` 等死成员。
3. **`get_multicandidateboxes` 路径存在严重缺陷**:输入张量未 set 到 infer request(推理空缓冲)、`factor` 缩放方向错误、关键点数 6 vs 5 不匹配导致越界读取。若实际使用 24 帧/调试多框模式,该路径基本不可用。
4. **EKF 观测函数与雅可比的一致性隐患**:`h2` lambda 内部 `point_buff2world` 读取成员 `ekf_.x` 而非入参 `x`,观测模型名义上依赖 `x` 实际依赖当前成员状态,破坏 EKF 数学假设;手写链式雅可比与实现易脱节。
5. **除零/数值稳定性隐患**:`-a/w`、`nowtime - lasttime_`、`radius/3`、`predict_time_`(作差分 dt)等分母未防护;`angle_changed` 读取未初始化 `yaw/pitch`(虽未使用,仍是 UB)。
6. **并发/状态污染**:`static` 局部变量(`lost_cn`/`start_timestamp`)跨实例共享;`predict` 就地修改 EKF 状态,`get_send_angle` 内多次调用导致状态被反复前推,`mpc_aim` 差分基准不一致;`Solver::solve` 标记 `const` 却写成员。
7. **Ceres 未真正使用**:仓库 vendor 了 ceres-solver,但正弦拟合实际用 Eigen SVD,依赖冗余;若本意用 Ceres 做非线性最小二乘,应替换或移除依赖。

**可改进点**
1. 修复 `get_multicandidateboxes`:统一走 `fill_tensor_data_image`(或正确 `set_input_tensor`),把 `factor` 改为 `1/scale`,为多框模型单独定义 `NUM_POINTS=5` 并正确 `rowRange(5,15)`,消除越界读取。
2. 将魔法数字(尤其 0.7 m、2.09、角速度范围、角度阈值)提为集中常量或 YAML 配置,消除多处重复。
3. 统一预测实现,删除 `buff_predict.hpp` 与其它死代码;抽象 `detect_24`/`detect` 公共逻辑。
4. 让 EKF 观测函数 `h2` 严格只依赖入参 `x`(把 `point_buff2world` 改为接受状态参数),并用数值差分或自动微分校验解析雅可比。
5. 对除零与奇异情形加保护(`w` 下限、时间差下限、`radius` 下限),用 `std::numeric_limits<float>::max()` 替代 `INF=1000000`。
6. 修复 `aim`/`mpc_aim` 的未初始化读取与 `save` 阈值写错的死代码;`predict` 改为返回预测值而非就地改状态,或提供"非破坏性预测"接口。
7. 若引入 Ceres,可用于大符正弦拟合的非线性最小二乘精修(替代当前 SVD 线性化 + 网格采样 ω 的 RANSAC),并明确依赖用途。


---

# Omniperception(哨兵感知)模块 与 src 主程序入口 分析

## 第一部分: Omniperception 模块概述

Omniperception 是哨兵(Sentry)机器人的"全向感知"模块。哨兵相比步兵(standard)的特殊之处在于:它安装在云台/顶部的固定平台上,不能像步兵那样通过整车转向来正对敌人,因此需要在除主摄像头之外额外布置多个固定朝向的 USB 摄像头(左、右、后等),把 360° 视野拼接成一个"哪里有人就往哪里转"的全向感知能力。本模块不负责传统的"盯住并解算目标"的自瞄闭环,而是解决一个更上层的问题:**当主视场里丢失目标时,根据侧视/后视摄像头看到的目标,把云台转到那个方向,让主摄像头重新捕获目标。**

模块内部由两个类分工:

- **Perceptron(感知层)**:负责"看"。它为每个 USB 摄像头开一条独立线程,各自持有一个 YOLO 检测器,持续读帧、跑推理,把检测到的装甲板连同该摄像头的角度偏移打包成 `DetectionResult`,塞进一个线程安全队列 `detection_queue_`,供主循环消费。它把"多路并行推理"从主循环里剥离出来,避免主循环被某一路慢速 USB 摄像头或一次推理卡住。

- **Decider(决策层)**:负责"算"。它对检测结果做过滤(敌方颜色、剔除无敌/前哨站/5 号装甲板)、按优先级排序,并根据"目标在哪个摄像头里、在画面的什么位置"换算成云台需要转动的 yaw/pitch 增量,最终产出 `io::Command`。它既提供"轮询切换摄像头"的串行 `decide(yolo, gimbal_pos, ...)` 接口(单线程哨兵用),也提供"消费检测队列"的 `decide(vector<DetectionResult>)` 接口(多线程哨兵用)。

**检测结果如何转化为决策(巡逻/开火/自瞄切换)**:实际切换逻辑不在 Decider 内部,而在各 `sentry*.cpp` 主循环里,基于 `Tracker` 的字符串状态机完成:`tracker.state() == "lost"` 时走全向感知(`Decider::decide`),`"switching"` 时直接按 `switch_target` 的角度转向,否则走正常自瞄(`Aimer::aim`)。所以本质是"**丢失→由全向感知补盲搜索→重新捕获→回到自瞄**"的循环,开火与否仍由 `Shooter::shoot` 决定。这里的"巡逻"体现为轮询多个摄像头(`Decider` 的 `count_` 计数器在 usbcam1/usbcam2/back_camera 之间 round-robin),而不是真正意义上的自主巡逻路径规划。

**与 ROS2 的关系**:Decider 和 Perceptron 本身不依赖 ROS2;ROS2 只在 `sentry*.cpp` 主程序里使用(`io::ROS2` 封装)。主循环通过 `ros2.subscribe_enemy_status()` 订阅裁判系统/导航下发的"无敌状态",通过 `ros2.subscribe_autoaim_target()` 订阅自瞄目标指令,并调用 `decider.get_invincible_armor()` / `get_auto_aim_target()` 把这些信息注入到过滤逻辑里;最后用 `decider.get_target_info()` 组装目标信息,由 `ros2.publish()` 发回导航。CMake 中只有检测到 ROS2 环境时才会编译 sentry 系列(sentry/sentry_bp/sentry_debug/sentry_multithread),见根 `CMakeLists.txt:125-151`。

整体数据流(多线程哨兵)是:4 路 USB 相机 + 主相机 → Perceptron 4 条推理线程 → `detection_queue_` → 主循环 `get_detection_queue()` → `Decider::sort`(过滤+排序)→ `Tracker::track` → 状态机决定走 `decide`/`aim` → `Shooter::shoot` → `CBoard::send`。

---

### 文件: tasks/omniperception/detection.hpp

这是一个只有 32 行的纯头文件,定义感知层与决策层之间的数据载体。

- **struct `DetectionResult`(:12-30)**
  - 作用:封装"一次全向感知检测"的完整结果。成员有:
    - `std::list<auto_aim::Armor> armors`(:14):该帧检测到的一组装甲板(可能是多个)。
    - `std::chrono::steady_clock::time_point timestamp`(:15):帧时间戳,供后续 IMU 外推对齐。
    - `double delta_yaw` / `double delta_pitch`(:16-17):单位 rad,表示"把云台朝该目标转过去"所需的相对角度增量。
  - 逐段说明:`armors` 用 `std::list` 而非 `vector`,原因是下游 `Tracker`/`Decider` 大量使用 `std::list::remove_if`、`sort` 等链式操作,list 的节点操作更契合这种"边过滤边排序"的场景。`delta_yaw/pitch` 直接内嵌在结果里,避免消费者再重复计算角度。
  - **优点**:结构简单、语义清晰,把"检测结果 + 角度偏移 + 时间戳"打包成单一单元,便于队列传输;手动定义的赋值运算符明确列出了所有成员,意图可读。
  - **缺点/隐患**:手写的 `operator=`(:20-29)完全是编译器默认浅拷贝行为,`std::list`、`std::chrono::time_point`、`double` 都是可平凡拷贝的类型,这里手写反而带来"以后新增成员忘记同步 operator="的维护风险,应直接用 `= default` 或干脆不写。另外 `delta_yaw/pitch` 只在 Perceptron 的推理线程里被填充,注释虽标了 rad,但没有任何字段级文档说明其参考系(相对哪个摄像头光轴、符号方向),后续维护容易搞反。

---

### 文件: tasks/omniperception/perceptron.hpp / perceptron.cpp

- **类 `Perceptron`(perceptron.hpp:18-44)**
  - 作用:多路 USB 相机并行推理的"感知器"。持有 4 个 YOLO 实例、4 条推理线程、1 个线程安全检测队列和 1 个 Decider。
  - 成员:`threads_`(线程容器)、`detection_queue_`(`ThreadSafeQueue<DetectionResult>`,容量 10)、`yolo_parallel1_~4_`(4 个 YOLO shared_ptr)、`decider_`(一个 Decider,用于算角度)、`stop_flag_`/`mutex_`/`condition_`(停止控制)。

- **构造函数 `Perceptron::Perceptron(...)`(perceptron.cpp:13-32)**
  - 作用:初始化 4 个 YOLO 检测器(都传 `false`,关闭可视化),睡 2 秒,再为每路相机各开一条 `parallel_infer` 线程。
  - 逐段说明:
    - `:16-17` 初始化成员:`detection_queue_(10)` 限制队列长度,`decider_(config_path)` 会**再构造一个 YOLO**(见 Decider 分析),`stop_flag_(false)`。
    - `:19-22` 用 `make_shared` 建 4 个 YOLO,每个都从同一 `config_path` 加载,4 份独立模型/推理资源。
    - `:24` `sleep_for(2s)` 是一个魔法延迟,推测是等相机/GPU/推理引擎就绪,但没有注释说明依据。
    - `:26-29` 以 lambda 捕获 `[&]` 创建 4 条线程,每条绑定一个相机指针和一个 YOLO。
  - **优点**:把 4 路独立推理隔离到独立线程,彼此不阻塞;用 shared_ptr 管理 YOLO 生命周期,析构安全。
  - **缺点/隐患**:4 份 YOLO 内存与推理资源开销大;`sleep_for(2s)` 属于拍脑袋式的固定延迟,在硬件差异下可能不够或多余;lambda 用 `[&]` 捕获,虽然对象生命周期覆盖线程(线程在析构里 join),但一旦未来改为延迟启动/异常路径,悬垂引用风险高。

- **析构函数 `Perceptron::~Perceptron()`(:34-49)**
  - 作用:置 `stop_flag_`、唤醒等待线程、join 所有推理线程。
  - 逐段说明:`:36-40` 在锁内设 `stop_flag_ = true`;`:40` `notify_all()` 唤醒等待者;`:43-47` join 每条线程。
  - **优点**:停止流程标准,先设标志再唤醒再 join。
  - **缺点/隐患**:`condition_` 其实是**摆设**——`parallel_infer` 从不调用 `wait(condition_)`,只在循环开头检查 `stop_flag_` 并靠 `cam->read()` 返回才退出,`notify_all()` 起不到唤醒作用。若某路相机 `read()` 长时间阻塞不返回,析构会卡在 join 上,无法及时退出。

- **`get_detection_queue()`(:51-63)**
  - 作用:一次性把队列里当前积累的所有 `DetectionResult` 全部取出,返回一个 `vector`。
  - 逐段说明:`:53-54` 定义结果容器和临时对象;`:57-60` 循环 `while (!detection_queue_.empty())` 并 `pop(temp)` 后 `move` 进结果。
  - **优点**:批量取走,减少主循环频繁加锁;`std::move` 避免大对象拷贝(其实 `DetectionResult` 含 list,移动有意义)。
  - **缺点/隐患**:代码注释(:56)说"pop 不阻塞(假设队列为空时会报错或忽略)",但**实际 `ThreadSafeQueue::pop(T&)` 是阻塞的**(见 `tools/thread_safe_queue.hpp:65-78`),依赖 `condition_variable::wait` 等非空。虽然当前只有主循环一个消费者、且用 `empty()` 做前置判断,功能上不会阻塞,但注释与实现相矛盾,是隐患:一旦将来出现第二个消费者,`empty()` 与 `pop()` 之间的竞态会导致一个消费者阻塞在 `pop` 上。

- **`parallel_infer(cam, yolo)`(:66-104)**
  - 作用:单路相机的推理循环——读帧→YOLO 检测→若有目标算角度→打包入队。
  - 逐段说明:
    - `:69-72` 空指针保护,直接返回并打 error。
    - `:73-101` 整个循环包在 `try/catch` 里,捕获 `std::exception` 并打 error(:101-103),保证线程不因单帧异常崩掉。
    - `:78-81` 每轮开始检查 `stop_flag_`(加锁)。
    - `:83-87` `cam->read(usb_img, ts)` 读帧,空帧则睡 30ms 继续。
    - `:89` `yolov8_parallel->detect(usb_img)` 推理。
    - `:90-99` 若检测到目标:`decider_.delta_angle(armors, cam->device_name)` 算角度,存进 `dr.delta_yaw/pitch`,除以 `57.3` 做"度→弧度"换算,`push` 进队列。
  - **优点**:异常隔离到位,单路相机故障不会拖垮整个感知器;空帧退避避免忙等。
  - **缺点/隐患**:
    1. **没有过滤与排序就取 `front()`**:`delta_angle` 内部使用 `armors.front()`,但这里没有先 `armor_filter`/`set_priority`/`sort`。YOLO 返回的 list 顺序并不代表优先级,可能把非敌方、低价值目标当成转过去的目标;敌方颜色过滤在感知层完全缺失。
    2. **`cam->device_name` 的识别机制脆弱**:`USBCamera` 靠相机硬件 `CAP_PROP_SHARPNESS` 的值(2→"left"、3→"right")来给 `device_name` 赋值(见 `io/usbcamera/usbcamera.cpp:109-119`),若某路相机 sharpness 既不是 2 也不是 3,`device_name` 为空串,`delta_angle` 会落入 `else` 分支按"后置相机 170° 偏移"计算,结果完全错误。多相机哨兵里 video4/video6 两路没有对应的 left/right 命名,几乎必然命中该分支。
    3. **`57.3` 魔法常量**:`delta_angle[0] / 57.3` 用 `57.3` 近似 `180/π`,精度略差且散落在多处(decider 里也是)。
    4. 队列满时 `ThreadSafeQueue` 默认 `PopWhenFull=false`,静默丢弃**新帧**,意味着最新目标可能被丢弃而保留旧目标,在快速切换目标时表现为感知滞后。

---

### 文件: tasks/omniperception/decider.hpp / decider.cpp

- **类 `Decider`(decider.hpp:19-86)**
  - 作用:全向感知的决策核心。提供 3 个 `decide` 重载、角度换算 `delta_angle`、过滤 `armor_filter`、优先级 `set_priority`、队列排序 `sort`、目标信息 `get_target_info`、无敌状态 `get_invincible_armor`、自瞄目标筛选 `get_auto_aim_target`。
  - 成员:`img_width_/img_height_`、`fov_h_/fov_v_`、`new_fov_h_/new_fov_v_`(新旧视场角)、`mode_`(优先级模式)、`count_`(轮询计数)、`enemy_color_`、`detector_`(一个 YOLO 成员)、`invincible_armor_`、两个 `PriorityMap`(mode1/mode2)。

- **构造函数 `Decider::Decider(config_path)`(decider.cpp:13-25)**
  - 作用:从 yaml 读取 `image_width/image_height/fov_h/fov_v/new_fov_h/new_fov_v/enemy_color/mode`,并**初始化成员 `detector_(config_path)`**。
  - 逐段说明:`:15-21` 读视场角等几何量;`:22-23` 把字符串颜色映射为 `auto_aim::Color`;`:24` 读 `mode_`。
  - **缺点/隐患**:`detector_` 是一个完整的 YOLO 成员,但**在当前所有代码路径里从未被使用**(三个 `decide` 都从参数接收 `yolo`)。这意味着每构造一个 Decider 就会多加载一份检测模型(占用显存/初始化时间),而 Perceptron 内部又持有一个 Decider,等于一份无用模型被白白加载。这是明显的资源浪费与设计瑕疵。

- **`decide(yolo, gimbal_pos, usbcam1, usbcam2, back_camera)`(:27-69)**
  - 作用:三相机轮询版决策。每次调用只读一路(usbcam1 / usbcam2 / back_camera 按 `count_` 轮流),检测到目标则返回"绝对 yaw = 当前云台 yaw + 相对增量"的指令。
  - 逐段说明:
    - `:31-32` 定义 `delta_angle` 与相机指针数组 `cams`。
    - `:36-38` 对 `count_` 做范围校验,越界抛异常——但由于 `count_` 只在 :59/:66 通过 `(count_+1)%3` 更新,永远落在 [0,2],这个检查是死代码(防御性)。
    - `:39-43` `count_==2` 时读后置相机,否则读 `cams[count_]`。
    - `:44-45` YOLO 检测 + `armor_filter` 过滤;`empty` 为 true 表示过滤后无目标。
    - `:47-64` 有目标时:按相机名算 `delta_angle`,打 debug 日志,`count_` 递增,返回 `io::Command{true, false, gimbal_pos[0] + delta_angle[0]/57.3, delta_angle[1]/57.3}`(注意 yaw 是绝对角、pitch 是相对角,且这里角度从"度"除以 57.3 转成"弧度")。
    - `:66-68` 无目标:递增 `count_`,返回 `io::Command{false,false,0,0}`。
  - **优点**:轮询机制简单直观,一次只处理一路相机,计算量可控;无目标时同样推进 `count_`,保证三路相机被公平轮询。
  - **缺点/隐患**:每次调用只"看一眼"一路相机,三路轮流意味着**每一路的刷新率只有主循环的 1/3**,目标出现在某路相机时可能被轮询机制延迟 2 个周期;`count_` 是实例级可变状态,多个调用点若并发会互相干扰(当前是单线程调用,暂无问题)。

- **`decide(yolo, gimbal_pos, back_camera)`(:71-92)**
  - 作用:仅后置相机的决策重载(对应 `sentry_bp.cpp` 这种"只有后置相机"的哨兵配置)。
  - 逐段说明:`:74-78` 读后置相机、检测、过滤;`:80-89` 有目标时算角度并返回绝对 yaw 指令;`:91` 无目标返回空指令。
  - **优点**:逻辑与三相机版一致,接口更精简。
  - **缺点/隐患**:与三相机版代码高度重复,`delta_angle` 换算、`Command` 组装逻辑两处抄写,后续改一处容易漏改另一处。

- **`decide(const vector<DetectionResult>&)`(:94-107)**
  - 作用:多线程哨兵用的队列版决策,直接消费 Perceptron 已经算好角度的结果。
  - 逐段说明:`:96-98` 队列空返回空指令;`:100-101` 取队首 `dr`,若 `armors` 空返回空指令;`:102-106` 打日志并返回 `io::Command{true,false,dr.delta_yaw,dr.delta_pitch}`。
  - **优点**:直接复用 Perceptron 线程算好的角度,主循环零计算量。
  - **缺点/隐患**:
    1. 只取 `front()`,**假设队列已经被 `sort()` 按优先级排好**;若调用方忘记先 `sort`,就会拿到优先级不定的目标(该假设是隐式的,没有断言或文档)。
    2. 返回的 `yaw` 是**相对增量**(rad),与另两个重载返回"绝对 yaw"的语义不一致,调用方必须记得再加 `gimbal_pos[0]`(`sentry_multithread.cpp:105` 就是这么做的),语义分裂容易用错。

- **`delta_angle(armors, camera)`(:109-130)**
  - 作用:把"目标在画面里的归一化坐标"换算成"云台需要转的角度增量",单位是**度**(调用方再除以 57.3 转弧度)。
  - 逐段说明:
    - `:113-117` `camera=="left"`:yaw 偏移基准 `+62°`,`(new_fov_h_/2) - center_norm.x * new_fov_h_` 表示"目标偏离画面中心的水平角",pitch 类似。
    - `:119-123` `camera=="right"`:yaw 基准 `-62°`。
    - `:125-129` else(后置相机):yaw 基准 `170°`,但视场角硬编码成 `54.2`/`44.5`(而非用成员 `fov_h_`/`fov_v_` 或 `new_fov_*`)。
  - **优点**:公式清晰,本质是"相机安装角 + (中心归一化坐标 − 0.5) × 视场角"。
  - **缺点/隐患**:
    1. **大量魔法数**:`62`、`-62`、`170`、`54.2`、`44.5` 都是相机安装角/视场角的硬编码,没有来自配置文件,换装相机或调整安装角度就要改代码重新编译。
    2. else 分支**没读配置里的 `fov`/`new_fov`**,与 left/right 分支数据来源不一致。
    3. 依赖 `armors.front()`,而本函数自身不做过滤/排序,`front()` 的正确性完全押在调用方是否先 `sort`/`filter`(而 `Perceptron::parallel_infer` 恰恰没有)。
    4. `camera` 是字符串比较,`"left"/"right"` 与 `USBCamera::device_name` 的赋值方式(靠 sharpness)耦合,改名/改识别逻辑就静默失效。

- **`armor_filter(armors)`(:132-153)**
  - 作用:过滤掉不该打的目标,返回"是否为空"。
  - 逐段说明:
    - `:134` 空则直接返回 true。
    - `:136` 剔除非敌方颜色(`enemy_color_`)。
    - `:139` 剔除 5 号装甲板(注释"25 赛季没有 5 号装甲板")。
    - `:141` 注释掉的"不打工程(2 号)"——当前被注释,意味着工程是可打目标。
    - `:143-144` 剔除前哨站(outpost)。
    - `:147-150` 剔除处于无敌状态的装甲板(与 `invincible_armor_` 比对)。
    - `:152` 返回过滤后是否为空。
  - **优点**:用 `list::remove_if` + 语义清晰的 lambda,规则集中、可读性好;注释记录了每一条规则的动机。
  - **缺点/隐患**:`invincible_armor_` 用 `std::find` 线性查找,目标种类少、性能无碍,但逻辑上是"名字在无敌名单里就剔除",与裁判系统无敌 ID 的映射(见 `get_invincible_armor`)需要保证一致;另外"无敌状态"更新依赖主循环每帧调用 `get_invincible_armor`,若某帧漏调,残留的旧无敌名单会继续生效。

- **`set_priority(armors)`(:155-166)**
  - 作用:根据 `mode_` 选择 mode1/mode2 优先级表,给每个 armor 赋 `priority`。
  - 逐段说明:`:159` 选表;`:161-165` 遍历赋值。
  - **优点**:优先级策略表驱动,两种模式切换方便。
  - **缺点/隐患**:`priority_map.at(armor.name)` 用 `at`,若名单里漏了某个 `ArmorName` 会抛异常(当前表覆盖了全部 9 个枚举值,尚安全,但新增枚举项时是隐患);mode1 里"工程(two)"是 `forth`、mode2 里"工程(two)"却是 `first`(最高),而 `armor_filter` 又没剔除工程,意味着 **mode2 下会优先打工程**,与"不打工程"的注释意图存在潜在矛盾,需确认是否有意为之。

- **`sort(detection_queue)`(:168-188)**
  - 作用:对每个 `DetectionResult` 先 `armor_filter` + `set_priority`,再按优先级排序 list;最后按各结果 `front().priority` 对整个 vector 排序。
  - 逐段说明:`:173-180` 逐个结果过滤、赋优先级、`list::sort`;`:183-187` 对整个 vector 按 `front().priority` 升序排序。
  - **优点**:一步到位完成"过滤 + 排序",配合 `decide(vector)` 使用语义正确。
  - **缺点/隐患**:`sort` 名字与 `std::sort` 易混;`std::sort` 的 lambda 直接访问 `a.armors.front()`,若某个 `DetectionResult` 在 `armor_filter` 后 `armors` 变为空,`front()` 是未定义行为(空 list 取 front 是 UB/段错误)。虽然 `armor_filter` 后为空的结果理论上仍会留在 vector 里,这是**真实存在的崩溃隐患**:应先把空结果从 vector 中剔除再排序。

- **`get_target_info(armors, targets)`(:190-206)**
  - 作用:从 targets 取第一个目标名,在 armors 中找同名 armor,返回 `{x, y, 1, name+1}` 四元组,用于 ROS2 发布。
  - 逐段说明:`:193` 空则返回零向量;`:195` 取 `targets.front()`;`:197-203` 匹配名字,命中则返回 `{xyz_in_gimbal[0], xyz_in_gimbal[1], 1, name+1}`;`:205` 未命中返回零向量。
  - **优点**:把内部 `Armor` 转成对外通信协议所需的精简四元组,`name+1` 的注释说明了协议约定(避免歧义)。
  - **缺点/隐患**:第 3 个元素硬编码为 `1`(z 坐标直接填 1),疑似占位/协议约定但无注释说明为何是 1;仅匹配 `targets.front()`,若 targets 有多个目标只取第一个。

- **`get_invincible_armor(invincible_enemy_ids)`(:208-218)**
  - 作用:把 ROS2 下发的"无敌敌人 ID"列表转成 `invincible_armor_`。
  - 逐段说明:`:210` 清空旧名单;`:212` 空则返回;`:214-217` 每个 id 打日志并 `push_back(ArmorName(id-1))`。
  - **优点**:每帧重建名单,不会残留旧状态。
  - **缺点/隐患**:`id - 1` 直接强转 `ArmorName`,没有做边界校验(若 id=0 或超出枚举范围会得到非法枚举值),上游数据异常时会污染过滤逻辑。

- **`get_auto_aim_target(armors, auto_aim_target)`(:220-242)**
  - 作用:按 ROS2 下发的自瞄目标编号列表,过滤 armors,只保留名单里的目标。
  - 逐段说明:`:223` 空则返回;`:227-234` 对每个 target 做范围校验(`<=0` 或超界则 warn + continue),合法则转成 `ArmorName(target-1)` 并打日志;`:236` 若全非法则返回;`:238-241` `remove_if` 剔除不在名单里的 armor。
  - **优点**:有边界校验和日志,比 `get_invincible_armor` 更健壮;非法值会被跳过而不是崩溃。
  - **缺点/隐患**:`static_cast<ArmorName>(target-1)` 与 `get_invincible_armor` 的 `ArmorName(id-1)` 是同一套"编号减 1"约定,但两处分别实现,约定散落、易不一致。

---

## 第二部分: src 主程序入口

`src/` 下每个 `.cpp` 都是一个可独立运行的 `main` 程序,对应不同机器人/不同硬件链路/不同调试需求。大体分为三组:

- **步兵(standard)系列**:`standard`、`standard1`、`mt_standard`、`standard_mpc`、`auto_aim_debug_mpc`、`mt_auto_aim_debug`。步兵是能整车转向的四轮机器人,主任务是自瞄(打装甲板)与打符(能量机关),核心硬件链路分两种:旧串口/CAN 的 `CBoard`(`standard`、`standard1`)和 USB CDC 虚拟串口的 `Gimbal`(`mt_standard`、`standard_mpc`、`auto_aim_debug_mpc`)。
- **哨兵(sentry)系列**:`sentry`、`sentry_bp`、`sentry_debug`、`sentry_multithread`。均依赖 ROS2,是全向感知(omniperception)的落地入口。
- **无人机(uav)系列**:`uav`、`uav_debug`。使用传统视觉检测器 `auto_aim::Detector`(而非 YOLO),但当前在 CMake 中**被注释掉不编译**(根 `CMakeLists.txt:53-54`),且 `uav.cpp` 引用了不存在的 `io::Mode::outpost`,属于已过时/未同步的代码。

所有 main 的运行流程高度一致:**解析命令行拿 yaml 路径 → 构造 Exiter/Plotter/Recorder → 构造相机与板卡(CBoard/CBoard1/Gimbal)→ 构造 YOLO/Solver/Tracker/Aimer/Shooter(及 buff 系列)→ 主 while 循环(读帧 → 读 IMU/云台状态 → 检测 → 跟踪 → 解算/规划 → 发送指令 → 录屏/可视化)→ 退出时 join 线程**。差异主要在:用哪个板卡通信用、用哪种解算器(Aimer 串行 vs Planner/MPC 并行)、是否多线程、是否开 debug 窗口。

---

### 文件: src/standard.cpp

- **作用**:最基础的单线程步兵自瞄程序,跑在步兵上,硬件为 `io::CBoard`(SocketCAN/CAN 板卡)+ `io::Camera` 工业相机,使用 `Aimer` 做瞄准解算。
- **逐行/逐段说明**:
  - `:24-26` 定义命令行参数 key,默认 `configs/standard3.yaml`。
  - `:28-35` 用 `cv::CommandLineParser` 解析配置路径;有 `help` 或路径为空则打印帮助并退出。
  - `:37-48` 依次构造 `Exiter`(退出标志)、`Plotter`(波形)、`Recorder`(录屏)、`CBoard`、`Camera`,以及 `YOLO(false)`(关可视化)、`Solver`、`Tracker`、`Aimer`、`Shooter`。
  - `:50-55` 声明 `img/q/t` 和 `mode/last_mode`(初始 idle)。
  - `:57-80` 主循环:
    - `:58-60` 读帧、按 `t-1ms` 取 IMU 四元数(减 1ms 是外推时间对齐的惯用补偿)、读当前 `mode`。
    - `:62-65` 模式变化时打日志。
    - `:67` `recorder.record` 被注释掉。
    - `:69-71` 设置"云台→世界"旋转,并解出 yaw/pitch/roll 欧拉角。
    - `:73-77` YOLO 检测 → Tracker 跟踪 → Aimer 解算 → 得到 `command`。
    - `:79` `cboard.send(command)` 发送。
  - **优点**:结构极其清晰,是标准自瞄流水线的"教科书"式最小实现;每一步职责单一,便于新手理解和在此基础上扩展。
  - **缺点/隐患**:`mode` 读了但**从未用于分支**(没有打符/自瞄切换逻辑,始终跑自瞄),说明此文件是早期/最简版本;没有可视化窗口(`YOLO(false)`、record 被注释),无法现场观察;`io::MODES[static_cast<int>(mode)]` 依赖 `Mode` 枚举值与 `MODES` 字符串数组下标严格一致,枚举改动时若忘改 `MODES` 会越界。

---

### 文件: src/standard1.cpp

- **作用**:步兵的"规划器(Planner)"版本。硬件为 `io::CBoard1`(硬编码 `/dev/ttyACM0` 串口)+ `io::Camera`,用 `auto_aim::Planner` 替代 `Aimer` 生成带速度/加速度前馈的 `Plan`。
- **逐行/逐段说明**:
  - `:28-35` 参数解析,默认 `configs/standard3.yaml`。
  - `:37-48` 构造 Exiter/Plotter/Recorder、`CBoard1("/dev/ttyACM0")`(注意板卡串口硬编码)、Camera、YOLO/Solver/Tracker/Planner/Shooter。
  - `:50-56` 声明变量,`last_crc_log` 用于 CRC 统计节流。
  - `:58-105` 主循环:
    - `:59-62` 读帧、`cboard.update()`(串口轮询)、取 IMU、读 mode。
    - `:64-76` 每秒打印一次串口 CRC 正确率(最近 5s 滑动窗口 + 累计),`rate<0` 表示窗口内无帧。
    - `:78-81` 模式变化日志。
    - `:83-87` 设置旋转、YOLO 检测、打日志输出检测数、Tracker 跟踪。
    - `:90-94` 取 `targets.front()` 包装成 `std::optional<Target>`,传给 `planner.plan(target, bullet_speed)`。
    - `:97-102` 把 `Plan` 的字段逐一复制到 `io::Command`(control/shoot/yaw/pitch/tracking)。
    - `:104` `cboard.send(command)`。
  - **优点**:引入了 CRC 正确率监控(串口链路质量),工程化程度比 `standard.cpp` 高;Planner 产出的 `Plan` 更完整,便于后续接速度/加速度前馈。
  - **缺点/隐患**:`CBoard1("/dev/ttyACM0")` 串口路径硬编码,换机器/多设备时易冲突;`Plan→Command` 的字段复制是手工逐字段搬运,`Plan` 或 `Command` 增字段时易漏;`planner.plan` 只吃单个 `target`(optional),多目标时只取第一个,丢掉了 Tracker 的多目标能力;仍没有 mode 分支。

---

### 文件: src/mt_standard.cpp

- **作用**:步兵的**多线程**自瞄 + 打符程序。硬件为 `io::Gimbal`(USB CDC 虚拟串口,SP 协议,默认 `/dev/gimbal`)+ `io::Camera`,自瞄用 `MultiThreadDetector` + `CommandGener`,打符用 buff 系列。
- **逐行/逐段说明**:
  - `:27-40` 参数解析(此处 `@config-path` 无默认值,必须显式传)。
  - `:42-66` 构造 Exiter/Plotter/Recorder、Camera、Gimbal、`MultiThreadDetector`(读 `debug_window` 控制是否可视化)、Solver/Tracker/Aimer/Shooter,以及 buff 的 Detector/Solver/SmallTarget/BigTarget/Aimer。
  - `:65` `CommandGener commandgener(shooter, aimer, gimbal, plotter, true)`——注意注释强调 **gimbal 必须在 CommandGener 之前构造**,因为后者持有其引用。
  - `:67-69` 声明原子 `mode`、`last_mode`、`last_crc_log`。
  - `:71-82` 检测线程:仅当 `mode==AUTO_AIM` 时才读帧并 `detector.push`,否则睡 5ms 避免白占 CPU。
  - `:84-167` 主循环:
    - `:86` 读 gimbal 模式(Gimbal 自带接收线程,无需像串口那样轮询 update)。
    - `:88-104` 每秒打印 CRC 正确率;`crc_check_enabled()` 为 false 时提示。
    - `:106-113` 模式变化时打日志,并在**切出自瞄时立即 `commandgener.push({})` 清空决策线程的最新帧**,避免残留自瞄指令与打符指令冲突导致云台抖动(注释解释得很清楚)。
    - `:116-132` AUTO_AIM 分支:`detector.debug_pop()` 取检测结果(注意 t 已滞后一个推理周期,Gimbal::q 会用最近两帧 IMU 反向外推),设旋转、Tracker 跟踪、`commandgener.push` 发给决策线程。
    - `:135-161` SMALL_BUFF/BIG_BUFF 分支:读帧、取 q、buff_solver 设旋转、buff_detector 检测、`solve`、按大小符选 target、`buff_aimer.aim` 得到指令、`gimbal.send`。
    - `:163-166` 其它模式睡 2ms 并 continue。
  - `:169` join 检测线程。
  - **优点**:多线程架构合理,检测与决策解耦;切模式时主动清空决策队列是**针对真实抖动问题**的细心修复;CRC 监控 + 注释详实,工程成熟度高;非自瞄模式用 sleep 退避避免抢推理资源。
  - **缺点/隐患**:`detector.debug_pop()` 若队列空会阻塞(依赖 `ThreadSafeQueue` 的阻塞语义),切出 AUTO_AIM 后检测线程不再 push,若主循环此时还卡在 `debug_pop` 可能短暂阻塞——不过有 sleep 兜底,风险中等;大小符两个分支的 target 逻辑几乎完全重复,可提炼。

---

### 文件: src/standard_mpc.cpp

- **作用**:步兵的 **MPC 规划器**版本。硬件为 `io::Gimbal` + `io::Camera`,自瞄用 `Planner`(MPC),打符用 `buff_aimer.mpc_aim`,自瞄与打符共用一条 200ms/10ms 周期的规划线程。
- **逐行/逐段说明**:
  - `:33-38` 参数解析。
  - `:40-59` 构造 Exiter/Plotter/Recorder、Gimbal、Camera、`YOLO(true)`(开可视化)、Solver/Tracker/Planner,以及 buff 系列;`:52-53` 建一个容量 1 的 `ThreadSafeQueue<optional<Target>, true>`(满时丢最旧),先 push `nullopt`。
  - `:65-88` 规划线程:循环里若非空且 AUTO_AIM,取 target、读 gimbal state、`planner.plan` 后 `gimbal.send`(8 参数带速度/加速度前馈),睡 10ms;否则睡 200ms。
  - `:90-193` 主循环:
    - `:91-96` 读模式、模式变化日志。
    - `:98-102` 读帧、`gimbal.q(t)`、`gimbal.state()`、`recorder.record`(这里没注释,实际录屏)、设旋转。
    - `:105-112` AUTO_AIM:YOLO 检测、Tracker 跟踪,有目标 push `front()`、无则 push `nullopt`。
    - `:115-190` SMALL_BUFF/BIG_BUFF:buff 检测、solve、按大小符选 target、`buff_aimer.mpc_aim`、`gimbal.send`(8 参数),随后组装一个**非常详细的 nlohmann::json 调试数据**(buff 原始观测、EKF 内部状态、云台响应、开火/指令),`plotter.plot(data)`。
    - `:191-192` 其它模式发送零指令。
  - `:195-197` 退出:置 `quit`、join 规划线程、发零指令。
  - **优点**:目标队列用 `optional<Target>` + 容量 1 的"丢最旧"队列,保证规划线程始终消费最新目标;规划与采集解耦,MPC 控制带完整速度/加速度前馈;打符的调试输出极其详尽(直接与 `auto_buff_debug_mpc` 对齐),便于离线分析波形。
  - **缺点/隐患**:规划线程内 `planner.plan` 在 10ms 周期内执行,若 MPC 解算耗时超过 10ms 会掉拍(没有自适应周期);`plan_thread` 用 `while(!quit)` 而 `quit` 是 `atomic<bool>`,退出时序正确,但主循环 `exiter.exit()` 与规划线程 `quit` 是两套退出标志,耦合较松;`buff_target` 指针在调试输出里被赋值(`buff_target = &buff_small_target` 等)但主要用途有限,属于调试代码混入主流程。

---

### 文件: src/auto_aim_debug_mpc.cpp

- **作用**:纯**自瞄 MPC 调试**程序(不涉及打符)。硬件 `io::Gimbal` + `io::Camera`,用 Planner 做 MPC,重点是把目标重投影画回图像、把规划量画到 Plotter,并支持 `q` 键退出。
- **逐行/逐段说明**:
  - `:24-38` 参数解析,默认 `configs/sentry.yaml`(注意默认配置名写的是 sentry,但这是自瞄调试)。
  - `:40-49` 构造 Gimbal、Camera、`YOLO(true)`、Solver、Tracker、Planner,建容量 1 目标队列并 push `nullopt`。
  - `:51-105` 规划线程:循环取 target、读 state、`planner.plan`、`gimbal.send`(8 参数),统计 `fired`(弹数变化),组装详细 json(云台量、目标量、规划量、fire/fired、target 的 z/vz/w),`plotter.plot`,睡 10ms。
  - `:110-143` 主循环:读帧、`gimbal.q(t)`、设旋转、YOLO 检测、Tracker 跟踪、push 目标;有目标时把 armor 观测点与 `planner.debug_xyza` 瞄准点重投影回图像并画点(绿色观测/红色瞄准);`:139-142` 缩放到 0.5 并 `imshow`,按 `q` 退出。
  - `:145-147` 退出:置 quit、join、发零指令。
  - **优点**:把 MPC 规划器的内部量(target/plan/云台/开火)完整可视化,是调 MPC 参数的利器;重投影可视化能直观看到"瞄准点 vs 实际装甲板"的偏差。
  - **缺点/隐患**:`data["w"]` 在 target 有值时读 `ekf_x()[7]`,无值时填 0,直接下标访问依赖 EKF 状态向量长度≥8,若状态向量结构变化会越界;`data["target_z"] = ekf_x()[4]`、`[5]` 同样依赖固定布局;默认 config 名写 `sentry.yaml` 易误导。

---

### 文件: src/auto_buff_debug.cpp

- **作用**:打符(能量机关)**串行调试**程序。硬件 `io::CBoard` + `io::Camera`,用传统 `Buff_Detector` + `Buff_Solver` + `BigTarget` + `Buff_Aimer::aim`(非 MPC),带图像与波形调试输出。
- **逐行/逐段说明**:
  - `:21-33` 参数解析。
  - `:35-49` 构造 Plotter/Recorder/Exiter、CBoard、Camera,以及 buff 的 Detector/Solver/BigTarget/Aimer(注释掉了 SmallTarget)。
  - `:55-155` 主循环:
    - `:56-58` 读帧、`cboard.imu_at(t)`(注意此处没有 `-1ms` 补偿,与其它文件不一致)、record 注释掉。
    - `:62-68` 设旋转、buff 检测、solve、`target.get_target`、`target_copy = target`(拷贝,供 `aim` 使用)。
    - `:70-74` `aimer.aim(target_copy, t, bullet_speed, true)` 得到 command,`cboard.send`。
    - `:76-148` 组装 json:原始观测、EKF 状态、云台响应、开火指令,并把 buff 靶标角点、中心、R 中心、EKF 预测位置重投影画到图像;`plotter.plot`、`imshow`、按 `q` 退出。
  - **优点**:调试信息非常完整(图像 + 波形双通道),是打符算法调试的标准入口;`target_copy = target` 保证传给 aimer 的是快照,避免 `get_target` 与 `aim` 之间状态被后续更新污染(虽然本循环单线程,意义有限)。
  - **缺点/隐患**:`imu_at(t)` 与其它文件统一用的 `t-1ms` 不一致,时间对齐口径不统一;`target.get_target` 若未能解算(`is_unsolve()`),`:91-135` 的 `auto & p = power_runes.value()` 会先访问 `power_runes.value()`,但外层 `if(!target.is_unsolve())` 并未保证 `power_runes` 有值,若 `power_runes` 为空且 target 非 unsolve,`value()` 会抛 `bad_optional_access`(实际中两者通常同步,但属隐患);`SmallTarget` 被注释,只支持大符。

---

### 文件: src/auto_buff_debug_mpc.cpp

- **作用**:打符 **MPC 调试**程序,是 `auto_buff_debug.cpp` 的 Gimbal + MPC 版本。硬件 `io::Gimbal` + `io::Camera`,用 `buff_aimer.mpc_aim` 生成带前馈的 `Plan`。
- **逐行/逐段说明**:
  - 结构与 `auto_buff_debug.cpp` 几乎一致,差异点:
    - `:41-49` 用 `io::Gimbal` 替代 `CBoard`,并构造 `BigTarget`(SmallTarget 注释)。
    - `:56-58` 读帧、`gimbal.q(t)`、`gimbal.state()`。
    - `:73-77` 用 `aimer.mpc_aim(target_copy, t, gs, true)` 得到 `Plan`,再 `gimbal.send`(8 参数,带速度/加速度前馈)。
    - `:140-153` 云台响应取自 `gs`(GimbalState)而非由 solver 欧拉角反推,`plan.control` 时输出 plan 各量。
  - **优点**:与 `standard_mpc.cpp` 的打符分支、`auto_buff_debug.cpp` 三处调试数据字段命名对齐,方便对比;MPC 版带完整前馈,能发挥 Gimbal 的速度/加速度控制能力。
  - **缺点/隐患**:与 `auto_buff_debug.cpp` 存在大量重复代码(整个 buff 检测/解算/调试输出块几乎逐字重复),维护成本高;其余隐患同 `auto_buff_debug.cpp`(`power_runes.value()` 的空值风险、仅支持大符)。

---

### 文件: src/mt_auto_aim_debug.cpp

- **作用**:步兵**多线程自瞄调试**程序。硬件 `io::CBoard` + `io::Camera`,用 `MultiThreadDetector` + `CommandGener`,带重投影与 Plotter 调试。注意:该文件在 CMake 中被**注释掉**(根 `CMakeLists.txt:50`),当前不参与编译。
- **逐行/逐段说明**:
  - `:23-36` 参数解析,默认 `configs/sentry.yaml`。
  - `:38-50` 构造 Exiter/Plotter/`Recorder(100)`(队列长度 100,注释"根据实际帧率调整")、CBoard、Camera、`MultiThreadDetector(config_path, true)`、Solver/Tracker/Aimer/Shooter、`CommandGener(shooter, aimer, cboard, plotter, true)`。
  - `:52-60` 检测线程:无限循环读帧 + `detector.push(img, t)`。
  - `:62-168` 主循环:
    - `:66-70` 记 t0、`detector.debug_pop()` 取帧/armors/t、按 `t-1ms` 取 IMU、读 mode。
    - `:72-75` 模式变化日志。
    - `:77-83` 设旋转、解欧拉角、Tracker 跟踪、`commandgener.push` 发给决策线程。
    - `:85-167` 调试:画 tracker 状态文字;组装 json(armor 原始观测、选最左装甲板 `solver.solve`、target 的 EKF 状态、卡方检验 residual/nis/nees、云台响应、bullet_speed);重投影画点;`plotter.plot`、`imshow`、`q` 退出。
  - `:170` join 检测线程。
  - **优点**:调试维度极全,尤其把 EKF 的**卡方检验(nis/nees/fail 计数)**都画出来,适合定位跟踪器发散问题;多线程检测 + 决策线程分离。
  - **缺点/隐患**:检测线程与主循环**同时访问 `exiter`**,退出时序依赖主循环 break 后 join 检测线程,但检测线程 `while(!exiter.exit())` 若阻塞在 `camera.read` 上则 join 可能卡住;`data["nis"]` 等用 `.at("...")` 访问 `target.ekf().data`,若 EKF 未填充这些 key 会抛 `out_of_range`(依赖 EKF 实现保证 key 存在);被注释出编译,存在与实际编译目标脱节的风险。

---

### 文件: src/sentry.cpp

- **作用**:哨兵全向感知的**基础单线程**入口(2 路 USB 相机 + 后置相机)。硬件 `io::ROS2` + `io::CBoard` + `io::Camera` + `io::Camera back_camera` + 2 路 `USBCamera`。仅当检测到 ROS2 环境时编译(根 `CMakeLists.txt:132-145`)。
- **逐行/逐段说明**:
  - `:28-43` 参数解析,默认 `configs/sentry.yaml`。
  - `:34-50` 构造 Exiter/Plotter/Recorder、`io::ROS2 ros2`(默认构造)、CBoard、主 Camera、`back_camera("configs/camera.yaml")`(后置相机路径**硬编码**,不走主 config)、`usbcam1("video0")`、`usbcam2("video2")`。
  - `:52-58` 构造 YOLO/Solver/Tracker/Aimer/Shooter,以及 `omniperception::Decider`。
  - `:60-63` 声明 img/timestamp/`last_command`(`last_command` 声明后未使用,死变量)。
  - `:65-104` 主循环:
    - `:66-68` 读帧、按 `t-1ms` 取 IMU、record 注释。
    - `:71-73` 设旋转、解出 `gimbal_pos`(欧拉角,注意这里叫 gimbal_pos 实为 ypr)。
    - `:75-83` YOLO 检测 → `get_invincible_armor(ros2.subscribe_enemy_status())` 更新无敌名单 → `armor_filter` → `set_priority`(`get_auto_aim_target` 被注释)。
    - `:85` Tracker 跟踪。
    - `:87-93` 状态机:`tracker.state()=="lost"` 走 `decider.decide(yolo, gimbal_pos, usbcam1, usbcam2, back_camera)`(轮询全向感知),否则 `aimer.aim(targets, t, bullet_speed, shoot_mode)` 正常自瞄。
    - `:96` `shooter.shoot(command, aimer, targets, gimbal_pos)` 决定开火。
    - `:98` `cboard.send`。
    - `:101-103` `decider.get_target_info(armors, targets)` + `ros2.publish` 回传导航。
  - **优点**:把"丢失→全向感知→重捕获→自瞄"的完整闭环串起来了;ROS2 订阅无敌状态/发布目标信息到位;`get_auto_aim_target` 被注释保留了可选的自瞄目标约束能力。
  - **缺点/隐患**:全向感知的 `decide` 是**串行轮询**,三路相机轮流看,每一路刷新率仅为 1/3;`tracker.state()` 用**字符串比较**("lost"),魔法字符串易写错且无编译期检查;`back_camera` 配置路径硬编码;`last_command` 未使用;`mode` 未参与分支(哨兵不像步兵那样读 `cboard.mode` 切模式,逻辑上始终处于自瞄+全向补盲状态)。

---

### 文件: src/sentry_bp.cpp

- **作用**:哨兵全向感知的"**仅后置相机**"简化版(无 USB 侧视相机)。硬件 `io::ROS2` + `io::CBoard` + 主 Camera + `back_camera`。
- **逐行/逐段说明**:与 `sentry.cpp` 几乎完全相同,差异仅在于:
  - `:48` 只有 `back_camera`,**没有** usbcam1/usbcam2。
  - `:89` lost 分支调用 `decider.decide(yolo, gimbal_pos, back_camera)`(后置相机专用重载),而非三相机版。
- **优点**:为"只装了后置相机"的哨兵配置提供轻量入口;复用 Decider 的后置相机重载。
- **缺点/隐患**:与 `sentry.cpp` 大面积重复,仅两处差异却维护两份文件;其余隐患同 `sentry.cpp`。

---

### 文件: src/sentry_debug.cpp

- **作用**:哨兵全向感知的**调试**版(2 路 USB + 后置相机),在 `sentry.cpp` 基础上增加重投影可视化、Plotter 波形,并**启用了 `get_auto_aim_target`**。
- **逐行/逐段说明**:与 `sentry.cpp` 结构一致,新增/差异点:
  - `:81` `decider.get_auto_aim_target(armors, ros2.subscribe_autoaim_target())` **启用**(sentry/sentry_bp 里被注释)。
  - `:91` lost 分支走三相机 `decide`。
  - `:105-194` 大段调试输出:画 tracker 状态;json 记录 armor 原始观测(选最左板 solve)、target EKF 状态、卡方检验量、云台响应(`gimbal_pos` 的 yaw 与 `-pitch`)、`shootmode`、cmd 量、bullet_speed;重投影画点;`plotter.plot`、`imshow`、`q` 退出。
- **优点**:调试信息最全的哨兵版本,能同时观察自瞄、全向感知、EKF、开火决策;启用了自瞄目标约束,可验证导航指定目标的效果。
- **缺点/隐患**:与 `sentry.cpp` 重复度高;调试代码(重投影、json 组装)与主逻辑耦合,上线前需手动剥离;`get_auto_aim_target` 会额外过滤 armors,若导航未下发目标则 list 为空,可能影响跟踪(取决于 Tracker 对空 list 的处理);其余隐患同 `sentry.cpp`。

---

### 文件: src/sentry_multithread.cpp

- **作用**:哨兵全向感知的**多线程**版(4 路 USB 相机 + 主相机),是 omniperception 模块(Perceptron + Decider)的完整落地。硬件 `io::ROS2` + `io::CBoard` + 主 Camera + 4 路 USBCamera(video0/2/4/6)。
- **逐行/逐段说明**:
  - `:37-46` 构造 Exiter/Plotter/Recorder、`io::ROS2 ros2`、CBoard、Camera,以及 `usbcam1~4`(video0/video2/video4/video6)。
  - `:56-63` 构造 YOLO/Solver/Tracker/Aimer/Shooter、`Decider decider`、`Perceptron perceptron(&usbcam1..4, config_path)`(启动 4 条推理线程)。
  - `:65-68` 声明 `switch_target`、img、timestamp、`last_command`(未使用)。
  - `:70-122` 主循环:
    - `:71-73` 读帧、`t-1ms` 取 IMU、`recorder.record`(这里启用录屏)。
    - `:75-77` 设旋转、解 `gimbal_pos`。
    - `:79-85` 主相机 YOLO 检测、`get_invincible_armor`、`armor_filter`、`set_priority`。
    - `:87-89` `perceptron.get_detection_queue()` 取全向检测队列、`decider.sort(detection_queue)`(过滤 + 排序)。
    - `:91` `tracker.track(detection_queue, armors, timestamp)`(多路重载),结构化绑定出 `switch_target` 与 `targets`。
    - `:96-101` `"switching"` 状态:`switch_target` 非空则 `control=true`,直接按 `switch_target.delta_pitch`/`delta_yaw + gimbal_pos[0]` 转向(不射击)。
    - `:103-106` `"lost"` 状态:`decider.decide(detection_queue)` 取已排序队首,`yaw = limit_rad(delta_yaw + gimbal_pos[0])`。
    - `:108-110` 否则正常 `aimer.aim(targets, t, bullet_speed)`。
    - `:113` `shooter.shoot` 决定开火。
    - `:116-121` `cboard.send` + ROS2 发布目标信息。
  - **优点**:这是模块设计的最终形态——4 路并行推理 + 队列 + `Decider::sort` + `Tracker` 多路跟踪 + 三态状态机(switching/lost/自瞄),把全向感知的"发现→转向→重捕获"过渡(switching)也显式建模了,比单线程版更完整。
  - **缺点/隐患**:
    1. 4 路 USB 相机里 video4/video6 的 `device_name` 大概率不是 "left"/"right"(见前文 `USBCamera` 靠 sharpness 赋值),`Perceptron::parallel_infer` 里对这两路算 `delta_angle` 会落进 `else` 分支按 170° 后置相机计算,角度可能错误。
    2. Perceptron 推理线程未做 `armor_filter`(敌方颜色过滤),`delta_angle` 直接取 `armors.front()`,切换目标可能是非敌方/低价值目标。
    3. `tracker.track(detection_queue, armors, timestamp)` 是未在本文档展示的重载,多路队列与主相机目标的融合逻辑需要额外确认;`switch_target` 角度直接相加 `gimbal_pos[0]`,符号/量纲依赖 Perceptron 里 `/57.3` 的正确性。
    4. `last_command` 死变量;`decide(detection_queue)` 假设已 `sort`,此处虽满足,但该假设是隐式契约。

---

### 文件: src/uav.cpp

- **作用**:无人机自瞄 + 打符程序。硬件 `io::Camera` + `io::CBoard`,检测用传统视觉 `auto_aim::Detector`(而非 YOLO,`:47` 处 YOLO 被注释)。**当前被注释出编译**(根 `CMakeLists.txt:53`),属于已过时代码。
- **逐行/逐段说明**:
  - `:25-38` 参数解析,默认 `configs/uav.yaml`。
  - `:40-58` 构造 Exiter/Plotter/Recorder、Camera、CBoard、`auto_aim::Detector`、Solver、Tracker、Aimer、Shooter,以及 buff 系列。
  - `:67-117` 主循环:
    - `:68-70` 读帧、`t-1ms` 取 IMU、读 mode。
    - `:72-75` 模式变化日志。
    - `:78-92` **自瞄或前哨站**分支(`mode==auto_aim || mode==outpost`):设旋转、`detector.detect`(传统 CV)、Tracker、`aimer.aim`、`shooter.shoot`、`cboard.send`。
    - `:95-113` 打符分支:buff 检测、solve、按大小符选 target、`buff_aimer.aim`、`cboard.send`。
    - `:115-116` 其它模式 continue。
  - **优点**:与步兵标准流程一致,且额外支持 `outpost`(前哨站)模式;`Detector` 传统视觉在算力受限的无人机平台上可能比 YOLO 更轻。
  - **缺点/隐患**:**`io::Mode::outpost` 在当前的 `io/command.hpp` 枚举中并不存在**(只有 idle/auto_aim/auto_buff/small_buff/big_buff),因此该文件按当前代码**无法编译通过**,属于枚举变更后未同步的历史遗留;文件整体被注释出构建,说明已弃用但未删除,长期留在仓库易误导。

---

### 文件: src/uav_debug.cpp

- **作用**:无人机自瞄**调试**版(带重投影 + Plotter),无打符逻辑。硬件 `io::Camera` + `io::CBoard`,检测用 `auto_aim::Detector`(YOLO 被注释但保留了 `yolo` 变量构造?实际 `:44` 构造了 `yolo(config_path)`,但主循环用的是 `detector.detect`,YOLO 未使用)。**同样被注释出编译**(根 `CMakeLists.txt:54`)。
- **逐行/逐段说明**:
  - `:26-40` 参数解析,默认 `configs/uav.yaml`。
  - `:35-47` 构造 Exiter/Plotter/Recorder、Camera、CBoard、`Detector`、Solver、**`YOLO yolo(config_path)`**(构造了但主循环未用)、Tracker、Aimer、Shooter。
  - `:56-172` 主循环:
    - `:58-66` 读帧、`t-1ms` 取 IMU、读 mode、模式日志。
    - `:69-81` 自瞄:`set_R_gimbal2world`、解欧拉角、`detector.detect`、Tracker、`aimer.aim`、`shooter.shoot`、`cboard.send`。
    - `:83-170` 调试:画 tracker 状态;json 记录 armor 原始观测(最左板 solve)、target EKF 状态、卡方检验量、云台响应、bullet_speed、cmd 量;重投影画点;`plotter.plot`、`imshow`、`q` 退出。
  - **优点**:调试维度与 `mt_auto_aim_debug` 类似,能完整观察自瞄 + EKF;无人机平台调试入口。
  - **缺点/隐患**:`:44` 构造的 `YOLO yolo` **从未使用**(白白加载模型),主循环用的是传统 `Detector`,这是明显资源浪费;文件被注释出编译;其余隐患同 `uav_debug` 同族(卡方量 `.at()` 依赖 EKF key 存在、调试代码与主逻辑耦合)。

---

## 本部分整体优缺点总结

**优点**

1. **层次清晰、职责分明**:omniperception 把"感知(Perceptron)"与"决策(Decider)"分离,src 各 main 只做编排;步兵/哨兵/无人机三套入口共享同一套 auto_aim / auto_buff 任务库,复用度高。
2. **全向感知闭环完整**:从多路并行推理 → 队列 → 过滤排序 → 三态状态机(switching/lost/自瞄)→ 云台转向 → 主相机重捕获,思路完整,尤其 `sentry_multithread.cpp` 显式建模了"切换"过渡态,工程考虑周到。
3. **工程化细节到位**:CRC 正确率监控(mt_standard/standard1)、切模式时清空决策队列防抖动(mt_standard)、异常 try/catch 隔离(perceptron)、详尽的调试波形与重投影可视化(debug 系列),都是真实调车场景中沉淀下来的经验。
4. **ROS2 解耦合理**:ROS2 只出现在 sentry 主程序入口,通过 `io::ROS2` 封装订阅/发布,任务层(Decider/Perceptron)不依赖 ROS2,方便单独测试。

**缺点/隐患**

1. **魔法常量与硬编码泛滥**:相机安装角(62/-62/170)、后置相机视场角(54.2/44.5)、角度换算系数 57.3、串口路径(/dev/ttyACM0)、后置相机配置路径(configs/camera.yaml)散落各处,换硬件/改安装角必须改代码。
2. **隐含契约过多、易用错**:`Decider::decide(vector)` 假设已 `sort`;`sort` 假设过滤后 list 非空(空 list `front()` 是 UB,可崩溃);`decide` 三/二相机版返回"绝对 yaw"而队列版返回"相对 yaw",语义不一致;`Perceptron` 推理线程不过滤就取 `front()`,敌方颜色过滤缺失。
3. **资源浪费**:`Decider::detector_` 是一个从未使用的 YOLO(每个 Decider 白加载一份模型);`uav_debug.cpp` 构造了未使用的 `YOLO yolo`;Perceptron 一次加载 4 份 YOLO。显存/内存与初始化时间开销大。
4. **注释与实现不一致**:`perceptron.cpp:56` 声称 `pop` 非阻塞,实际 `ThreadSafeQueue::pop` 是阻塞的;`Decider::armor_filter` 注释"不打工程"但对应 remove_if 被注释,且 mode2 优先级表又把工程列为最高优先。
5. **字符串状态机脆弱**:`tracker.state()=="lost"/"switching"` 用魔法字符串比较,无枚举/常量约束,拼写错误只能运行时暴露。
6. **代码重复**:sentry 四兄弟、auto_buff 两个 debug、standard 系列之间大量重复(尤其 buff 检测/解算/调试输出块),一处修 bug 需同步多处;历史遗留文件(uav/uav_debug/mt_auto_aim_debug)被注释出构建却仍留在仓库,`uav.cpp` 甚至引用了不存在的 `io::Mode::outpost`,与当前代码脱节。
7. **线程退出依赖阻塞读返回**:Perceptron 析构与各 debug 程序的 join 都假设 `camera.read()` 会及时返回,若相机阻塞可能卡住退出;Perceptron 的 `condition_variable` 是摆设,`notify_all()` 无法真正唤醒不 wait 的推理线程。


---

# 标定、测试、脚本与配置 分析

> 本文分析 sp_vision_25 项目的 calibration/、scripts/、configs/、tests/ 四个目录。
> 所有路径均为仓库内相对路径（项目根目录为 `/home/toor/Desktop/gongxiang/tongji/sp_vision_25`）。
> 涉及行号以撰写时的文件版本为准。

---

## 一、标定工具 calibration

`calibration/` 下有 5 个独立可执行程序，构成一条完整的标定流水线：

1. **capture.cpp** — 采集标定板图像 + 对应时刻的云台 IMU 四元数（数据采集）；
2. **calibrate_camera.cpp** — 单目相机内参标定（fx/fy/cx/cy + 畸变系数）；
3. **calibrate_handeye.cpp** — 手眼标定（eye-in-hand，求解相机到云台的位姿 `R_camera2gimbal / t_camera2gimbal`）；
4. **calibrate_robotworld_handeye.cpp** — 机器人-世界-手眼联合标定（同时求解相机-云台位姿与标定板-世界位姿）；
5. **split_video.cpp** — 离线视频切帧工具，从录制好的 avi + 时间戳文本中截取一段。

流水线关系：`capture` 采集 `jpg + txt(四元数)` → `calibrate_camera` 用这些 jpg 算内参 → 把内参和 `R_gimbal2imubody` 写进 calibration.yaml → `calibrate_handeye` 或 `calibrate_robotworld_handeye` 用同一批 jpg + txt + 内参解外参，输出 `R_camera2gimbal / t_camera2gimbal` 写回各机器人 yaml。`split_video` 与标定无直接关系，是测试数据预处理工具。

### 文件: calibration/capture.cpp

- **作用**：采集标定板图片，并同时记录图片曝光时刻云台 IMU（经 CBoard 从 CAN 总线读取）的姿态四元数。每一张图对应一个 txt 文件存四元数，供后续手眼标定使用。

- **逐段说明**：
  - L13-16：`cv::CommandLineParser` 定义命令行参数：`@config-path`（默认 `configs/calibration.yaml`）、`output-folder`（默认 `assets/img_with_q`）。
  - L18-25 `write_q`：把 Eigen `Quaterniond` 以 **wxyz** 顺序写入 txt（`q.coeffs()` 是 xyzw，故取 `xyzw[3],xyzw[0],xyzw[1],xyzw[2]`）。
  - L27-70 `capture_loop`：
    - L30-31：构造 `io::CBoard`（读云台 IMU/弹速的 CAN 板）与 `io::Camera`（工业相机）。
    - L37：`camera.read(img, timestamp)` 读一帧并取采集时间戳。
    - L38：`cboard.imu_at(timestamp)` 用时间戳插值取该帧对应的 IMU 姿态。
    - L42-45：把四元数转 ZYX 欧拉角画到图上，用于直观判断 imuabs 系 XYZ 方向与零漂。
    - L48-49：`cv::findCirclesGrid(img, Size(10,7), centers_2d)` 识别对称圆点标定板（10 列 7 行），`drawChessboardCorners` 可视化识别结果（名字虽叫 chessboard，实为圆点）。
    - L53-58：`waitKey(1)` 非阻塞，按 `s` 保存、`q` 退出。
    - L60-66：按序号保存 `1.jpg` 与 `1.txt`（四元数）。
  - L72-93 `main`：解析参数、`create_directory` 建输出目录、打印提示、进入主循环。

- **优点**：
  - 图像与 IMU 姿态按时间戳严格配对，为手眼标定提供时间一致的样本；
  - 交互式保存（按 s 才存）便于筛选质量好的帧；
  - 通过画面上的欧拉角可现场判断 IMU 姿态是否合理、有无零漂。

- **缺点/隐患**：
  - 标定板规格 `Size(10,7)` 与 `center_distance_mm` 硬编码在代码/注释里（L48、L86），若换标定板需改代码，而非完全从 yaml 读取（calibration.yaml 里其实有 pattern 字段，但 capture.cpp 没读）；
  - 保存的图片是**原始图**（`cv::imwrite(img_path, img)`），而显示的是缩小后的克隆图，不影响正确性但容易让使用者误以为存的是带标注的图；
  - 无成功保存的自动去重/防覆盖逻辑，若 output_folder 已存在同名文件会被覆盖。

### 文件: calibration/calibrate_camera.cpp

- **作用**：单目相机内参标定。读入 capture.cpp 采集的图片，识别圆点，调用 OpenCV `calibrateCamera` 输出相机内参矩阵与畸变系数，并计算重投影误差，以 YAML 格式打印。

- **逐段说明**：
  - L14-23 `centers_3d`：按 pattern_size 和点距生成世界系 3D 坐标（z=0，单位 mm）。
  - L25-65 `load`：
    - L31-35：从 yaml 读 `pattern_cols / pattern_rows / center_distance_mm`。
    - L37-41：从 1 开始递增读 `{i}.jpg`，读到空文件即结束（依赖文件名连续）。
    - L48：`findCirclesGrid(..., CALIB_CB_SYMMETRIC_GRID)` 识别对称圆点。
    - L54-55：逐张 `waitKey(0)` 等待人工确认。
    - L59：识别失败则 `continue`，跳过该帧但不中断。
    - L62-63：收集 `img_points`（2D）与 `obj_points`（3D）。
  - L67-85 `print_yaml`：用 `YAML::Emitter` 把内参、畸变、重投影误差以 Flow 风格打印。
  - L87-130 `main`：
    - L107-112：`cv::calibrateCamera`，`CALIB_FIX_K3` 固定 k3=0（注释解释视场角较小不需 k3）；TermCriteria 迭代次数改为 100（默认 30 有时发散）。
    - L114-126：逐张 `projectPoints` 反投影，累加 `cv::norm` 距离求平均重投影误差。

- **优点**：
  - 输出格式可直接粘贴进机器人 yaml（Flow 风格与 configs 里一致）；
  - 单张失败不中断，容错好；
  - 迭代次数、固定 k3 等细节有注释说明原因。

- **缺点/隐患**：
  - 读图靠"文件名从 1 连续递增 + 遇到空即停"，若中间缺号会提前终止；
  - 人工 `waitKey(0)` 逐张确认，图片多时繁琐；
  - `print_yaml` 只打印到 stdout，未写文件，需手动复制。

### 文件: calibration/calibrate_handeye.cpp

- **作用**：手眼标定（eye-in-hand）。利用 capture 得到的每帧四元数（云台在世界系中的姿态）+ 每帧 solvePnP 得到的相机-标定板位姿，调用 `cv::calibrateHandEye` 求解相机相对云台的固定位姿 `R_camera2gimbal / t_camera2gimbal`。

- **逐段说明**：
  - L17-26 `centers_3d`：与 calibrate_camera 相同的 3D 点生成（原点在标定板左上角，mm）。
  - L28-34 `read_q`：读 txt 的 wxyz 四元数。
  - L36-105 `load`：
    - L43-49：从 yaml 读 `pattern_cols/pattern_rows/center_distance_mm/R_gimbal2imubody/camera_matrix/distort_coeffs`。
    - L52：用 RowMajor 从 9 个 double 构造 `R_gimbal2imubody`。
    - L56-62：逐帧读图与四元数。
    - L65-67：`R_gimbal2world = R_gimbal2imubody^T · R_imubody2imuabs · R_gimbal2imubody` —— 把 IMU 绝对姿态经"云台系↔IMU body 系"旋转变换得到云台在世界系的姿态。
    - L68-74：转 yaw/pitch/roll 画到图上，用于现场校验 `R_gimbal2imubody` 是否正确。
    - L78：`findCirclesGrid` 识别圆点。
    - L91-97：`t_gimbal2world` 设为 0（eye-in-hand 手眼标定中相机与云台固连、云台在世界系的平移近似 0），`R_gimbal2world` 转 cv::Mat；`solvePnP(..., SOLVEPNP_IPPE)` 解相机-标定板位姿 rvec/tvec。
    - L100-103：收集 `R_gimbal2world / t_gimbal2world` 与 `rvec / tvec` 列表。
  - L107-132 `print_yaml`：输出 `R_gimbal2imubody`（原样带回，方便粘贴）、`R_camera2gimbal / t_camera2gimbal` 以及"相机同理想情况的偏角"注释。
  - L134-168 `main`：
    - L155-157：`cv::calibrateHandEye`；`t_camera2gimbal /= 1e3`（mm→m）。
    - L160-164：定义理想相机-云台旋转 `R_gimbal2ideal`，计算 `R_camera2ideal = R_gimbal2ideal · R_camera2gimbal`，转欧拉角作为"相机装偏程度"的直观指标。

- **优点**：
  - 复用 capture 的数据与 yaml 内参，流程清晰；
  - 在图上显示云台欧拉角，能及时发现 `R_gimbal2imubody` 标定错误；
  - 输出"相机同理想情况的偏角"，方便机械调装。

- **缺点/隐患**：
  - 把云台在世界系的平移 `t_gimbal2world` 恒置 0（L91），这一假设仅在云台旋转中心与标定板距离远大于旋转半径、且旋转时近似不动时成立，会引入平移项误差；
  - `read_q` 无格式/失败检查，txt 缺行会读到未初始化值；
  - `SOLVEPNP_IPPE` 对平面点集假设点共面，标定板需严格平整。

### 文件: calibration/calibrate_robotworld_handeye.cpp

- **作用**：机器人-世界-手眼联合标定。与 handeye 不同，它不把世界系放在云台，而是调用 `cv::calibrateRobotWorldHandEye` 同时求解"世界↔标定板"和"云台↔相机"两组变换，从而还能得到标定板相对世界原点的水平距离与摆放偏角。

- **逐段说明**：
  - L17-31 `centers_3d`：3D 点生成方式与前面不同——x 恒为 0，y/z 以标定板中心为原点（`-j + 0.5*width`、`-i + 0.5*height` 乘点距），即把 3D 点原点放在板中心、且 x 轴垂直板面朝外。这是为了配合 robot-world 方法中"世界系原点放在某固定点"的约定。
  - L33-39 `read_q`：同 handeye。
  - L41-111 `load`：
    - 基本同 handeye，区别在 L96：`R_world2gimbal = R_gimbal2world^T`（取转置，得到世界系到云台的旋转），平移到 0。
    - L102-103：同样 `solvePnP(..., SOLVEPNP_IPPE)`。
    - L106-109：收集 `R_world2gimbal / t_world2gimbal` 与 `rvec / tvec`。
  - L113-146 `print_yaml`：额外输出两条注释——"标定板到世界坐标系原点的水平距离"、"标定板同竖直摆放时的偏角"。
  - L148-204 `main`：
    - L170-172：`cv::calibrateRobotWorldHandEye(rvec, tvec, R_world2gimbal, t_world2gimbal, R_world2board, t_world2board, R_gimbal2camera, t_gimbal2camera)`。
    - L173-174：两个平移量 mm→m。
    - L177-182：转置求逆得到 `R_camera2gimbal`、`R_board2world` 与对应平移。
    - L185-189：计算相机相对理想情况的偏角。
    - L192-194：由 `t_board2world` 的 x/y 分量求标定板到世界原点水平距离。
    - L197-199：`R_board2world` 转欧拉角求标定板竖直摆放偏角。

- **优点**：
  - 无需把世界原点强行放到云台，能同时得到相机-云台外参和标定板-世界外参，信息更完整；
  - 额外给出"标定板水平距离 / 偏角"，便于校验世界系定义是否合理。

- **缺点/隐患**：
  - `calibrateRobotWorldHandEye` 需要足够的位姿多样性（不同云台角度、不同板位置），否则解不稳定，代码没有样本数/姿态分布检查；
  - 同样存在 `t_world2gimbal` 恒 0 的近似；
  - 3D 点坐标系约定（原点在板中心、x 垂直板面）与 `centers_3d` 在 calibrate_camera/handeye 中的约定（原点在角点、z=0）不一致，混用样本或标定板会出错。

### 文件: calibration/split_video.cpp

- **作用**：把录制的 `xxx.avi` 视频 + `xxx.txt`（每帧的时间戳 + wxyz 四元数）按帧下标区间切出一段，输出新的 avi + txt，用于离线测试数据的裁剪。

- **逐段说明**：
  - L15-20：参数 `start-index / end-index / output-path / @input-path`。注意 `output-path` 和 `@input-path` 的 help 文案重复了（都写"avi和txt文件的路径"）。
  - L30-34：读取起止下标。
  - L40-43：构造 `{input}.avi` 与 `{input}.txt`。
  - L46-50：`video.set(CAP_PROP_POS_FRAMES, start_index)` 设置视频起点；同时把 txt 前 start_index 行读掉（同步跳过）。
  - L52-61：读原视频的宽高/fps/fourcc，创建 `VideoWriter`（用原 fourcc、fps、尺寸）与输出 txt。
  - L67-83：从 start_index 起逐帧：超过 end_index 或读空即停；写视频帧；`getline` 读对应时间戳行写入 outtext；缩放显示，`q` 退出。
  - L84-88：销毁窗口、关闭文件。

- **优点**：
  - 视频与时间戳文本同步裁剪，保证切出的离线数据帧帧对齐；
  - 保留原始编码参数（fourcc/fps/尺寸），不重新压缩。

- **缺点/隐患**：
  - 依赖 avi 的帧随机访问（`CAP_PROP_POS_FRAMES`），部分编码（如某些 MJPG/H264）定位不准；
  - txt 行数与视频帧数不一致时会错位，代码没有校验；
  - `end_index` 用 `> end_index` 判断，`end_index=0` 表示不限制（无上限保护）。

---

## 二、脚本 scripts

### 文件: scripts/setup_deps.sh

- **作用**：安装 sp_vision_25 的编译/运行系统依赖，需 root。可选的第二个参数是 OpenVINO 2025.4.0 的 tgz 包路径，给出才安装。

- **逐段说明**：
  - L9：`set -euo pipefail`，严格错误处理。
  - L11-14：非 root 直接退出并提示用 sudo。
  - L16：`TARGET_USER="${SUDO_USER:-$USER}"`，把用户加组时用真正的登录用户而非 root。
  - L21-24：`apt-get install` 安装 `build-essential cmake ccache screen` 与 `libopencv-dev libeigen3-dev libspdlog-dev libyaml-cpp-dev libceres-dev libusb-1.0-0-dev`。注释说明相机 SDK（MvCameraControl/MVSDK）、串口库 serial 随仓库自带。
  - L27-32：把目标用户加入 `dialout` 组（串口权限），幂等判断。
  - L35-64：OpenVINO 安装分支：
    - L35-38：`readlink -f` 取 tgz 绝对路径并检查存在。
    - L39-52：若 `/opt/intel/openvino_2025.4.0` 已存在则跳过；否则解压到 `mktemp -d` 临时目录，`trap` 清理，`mv` 到目标路径。
    - L54-61：若存在 `install_openvino_dependencies.sh` 则非交互式安装 libtbb 等运行时依赖。
  - L62-64：未提供 tgz 时跳过，继续用旧的 `/opt/intel/openvino_2024.6.0`。
  - L66-70：结尾提示 dialout 需重新登录、电控串口软链命令、需 `mkdir -p logs`。

- **优点**：
  - 幂等、严格模式、临时目录 + trap 清理，健壮；
  - 对"要不要装 OpenVINO 2025.4.0"做了清晰的可选开关，与 CMake 优先路径一致。

- **缺点/隐患**：
  - 无 OS 版本检查（依赖项名按 Ubuntu 22.04 假设），其他发行版会失败；
  - OpenVINO 依赖脚本的路径假设较强，失败只提示手动安装。

### 文件: scripts/setup_gimbal_udev.sh

- **作用**：为下位机 USB CDC 虚拟串口（`/dev/ttyACM*`）建立固定的 `/dev/gimbal` 符号链接，写 udev 规则。

- **逐段说明**：
  - L10-14：默认 `LINKNAME=gimbal`，规则路径 `/etc/udev/rules.d/99-usb-serial.rules`；非 root 退出。
  - L22-36：未显式给设备时，用 `nullglob` 展开 `/dev/ttyACM*`；0 个报错、多个则要求显式指定（避免绑错板子）。
  - L38-41：检查设备是否存在。
  - L44-47：`udevadm info -a -n` 抓 `idVendor / idProduct / serial` 属性。
  - L49-52：VID/PID 缺失即报错退出。
  - L57-63：serial 存在则加进匹配条件，否则仅按 VID/PID 匹配并警告"插第二块同型号板会冲突"。
  - L66：拼 udev 规则 `SUBSYSTEM=="tty", ... , SYMLINK+=..., GROUP="dialout", MODE="0660"`。
  - L70-74：防护——规则为空或缺关键字段时拒绝落盘，避免把已有规则文件截断成 0 字节。
  - L76-84：写临时文件 `tmp.$$` 再 `mv` 原子替换，避免空文件窗口。
  - L90-93：`udevadm control --reload-rules` + `trigger`，sleep 1 等 udevd 处理。
  - L96-110：验证 `/dev/gimbal` 是否建好，指向是否与预期设备一致，不一致给出"拔插 USB"提示。

- **优点**：
  - 幂等、原子写、对空 serial/多设备/空规则等边界情况都有防护，工程质量高；
  - 写前校验规则合法性，避免误清空 udev 规则文件。

- **缺点/隐患**：
  - 用 `awk` 取 `udevadm info` 属性依赖其输出格式（`ATTRS{...}=="..."`），udev 版本变化可能解析失败（但已有 VID/PID 为空检查兜底）；
  - 规则文件统一叫 `99-usb-serial.rules`，若系统已有同名文件且包含其它规则，会被整体覆盖（脚本只检测是否已有同名 SYMLINK，未合并）。

### 文件: scripts/gimbal_monitor.py

- **作用**：USB CDC（SP 协议）串口联调工具。纯标准库（不用 pyserial），接收解析电控发来的 `GimbalToVision`（43 字节）帧、校验 CRC16 并打印各字段；`--send-idle` 时以 10Hz 发送 `VisionToGimbal`（29 字节）mode=0 帧，用于验证"视觉能发但云台不动"这条最基础的通路。

> 说明：本文件实际为 **217 行**（非任务描述中的 396 行），下面按真实行号分段说明。

- **逐段说明**：
  - L1-15：模块 docstring，说明帧格式与 CRC 算法对齐 `io/gimbal/gimbal.hpp` 与 `tools/crc.cpp`：
    - `GimbalToVision` 43B：head(2) mode(1) q[4]·16 yaw(4) yaw_vel(4) pitch(4) pitch_vel(4) bullet_speed(4) bullet_count(2) crc16(2)；
    - `VisionToGimbal` 29B：head(2) mode(1) yaw(4) yaw_vel(4) yaw_acc(4) pitch(4) pitch_vel(4) pitch_acc(4) crc16(2)；
    - CRC16：poly=0x1021 init=0xFFFF，小端，覆盖除末 2 字节外全部内容。
  - L24-25：定义两帧长度常量。
  - L28-35 `crc16_ccitt_false`：CRC-16/CCITT-FALSE 逐字节实现，对齐 C++ 端 `get_crc16`。
  - L38-41 `check_crc16`：取末 2 字节（`frame[-1]<<8 | frame[-2]`，小端）与算出的 CRC 比较。
  - L44-63 `open_port`：
    - L46：`os.open(..., O_RDWR|O_NOCTTY|O_NONBLOCK)` 非阻塞打开。
    - L49-56：`termios` 置 raw 模式——关 IXON/IXOFF/IXANY 软件流控、INLCR/ICRNL/IGNCR 换行改写、OPOST 输出处理、ICANON/ECHO/ISIG 行规程，置 CLOCAL|CREAD、CS8、无奇偶/停止位/硬件流控。
    - L57-61：`B{baud}` 设置波特率（CDC ACM 下无实义但需设），VMIN=VTIME=0 非阻塞读，`tcsetattr` 一次性应用，`tcflush` 清缓冲。
  - L66-74 `build_idle_frame`：构造 29B VisionToGimbal，head="SP"、mode=0，6 个 float 全 0，算 CRC 填末 2 字节。
  - L77-92 `main` 参数解析：
    - `port` 默认 `/dev/ttyACM0`；`--baud` 默认 115200；`--send-idle`；`--seconds`（0=一直跑）；`--raw`（额外打印十六进制）；`--no-crc`（不校验 CRC，改用单位四元数 `|q|²≈1` 做合理性校验，注释说明实测当前 C 板固件 crc16 字段疑似未赋值）。
  - L94-103：打开串口的异常处理——`PermissionError` 提示加 dialout 组，`FileNotFoundError` 提示确认下位机枚举。
  - L105-110：打印表头（时间/mode/yaw/yaw_vel/pitch/pitch_vel/弹速/弹量/q）。
  - L112-118：初始化缓冲、计数统计、时间戳、idle 帧。
  - L120-193 主循环：
    - L122-123：`--seconds` 到时退出。
    - L125-130：`--send-idle` 时每 0.1s（10Hz）写一次 idle 帧。
    - L132-141：`os.read(fd, 4096)` 非阻塞读（`BlockingIOError` 置空 chunk），失败退出；有数据则 append 到 buf。
    - L144-188 逐帧切分解析：
      - L145-154：在 buf 里 `find(b"SP")` 找帧头；找不到就 `bad_head += len(buf)` 并清空；找到但 idx>0 说明前面是垃圾字节，丢弃并计数；buf 不足 43 字节则等待下一轮。
      - L156-157：取满 43 字节，`total++`。
      - L159-170：`--no-crc` 时用 `struct.unpack_from("<4f", frame, 3)` 取四元数，`norm2` 与 1 偏差 >1e-2 判失败并 `del buf[:1]`（只前移 1 字节避免误吞下一帧）；否则 `check_crc16` 失败同样 `del buf[:1]`。
      - L172-176：校验通过后解析 mode、q（4f）、yaw/yaw_vel/pitch/pitch_vel/bullet_speed（5f，从 offset 19）、bullet_count（H，offset 39）。
      - L178-183：1Hz 节流打印（避免刷屏）。
      - L185-186：`--raw` 打印十六进制。
      - L188：成功帧 `del buf[:43]` 前移。
    - L190-193：每 2s 打印一次统计（累计帧/CRC 通过/CRC 失败/丢弃字节）。
  - L195-209：结束收尾——按校验方式打印统计；`total==0` 返回 2；`ok==0` 返回 3 并给出排查建议（`--no-crc` 绕过 / 让电控确认固件是否真的算了 crc16）；全通过返回 0。
  - L212-216：`__main__` 捕获 `KeyboardInterrupt` 友好退出。

- **优点**：
  - 纯标准库零依赖，便于在电控现场快速部署；
  - 帧头搜索 + 失败只前移 1 字节的切分策略鲁棒，能容忍乱码/半帧；
  - CRC 与 C++ 端严格对齐，且有 `--no-crc` 四元数校验降级方案，兼顾严谨与现场灵活性；
  - 统计信息（总帧/通过/失败/丢弃字节）与退出码让脚本可直接用于自动化联调判定。

- **缺点/隐患**：
  - 帧头 `b"SP"`（0x53 0x50）若恰好出现在 payload 数据里（尤其 float 的字节流）会被误判为帧头，造成丢帧/错位（虽然概率低，但无长度前向/转义校验）；
  - 四元数合理性校验阈值 `1e-2` 较宽松，`--no-crc` 下弱校验；
  - 主循环无 `sleep`，`--send-idle` 关闭且无数据时是忙等（非阻塞读立即返回空），CPU 占用偏高。

---

## 三、配置 configs

### 3.1 总述

所有 yaml 用 `yaml-cpp` 读取，字段按功能分组并用 `#####-----xxx参数-----#####` 注释隔开。配置在结构上可归纳为几大块（并非每个文件都含全部块）：

- **敌我颜色**：`enemy_color`；
- **神经网络参数**：模型路径、设备、推理性能参数、置信度等；
- **ROI**：感兴趣区域；
- **传统方法参数**：阈值、灯条/装甲板几何约束；
- **相机参数**（工业相机 / USB 相机）；
- **tracker（跟踪器）参数**、**aimer（瞄准器）参数**、**shooter（射击器）参数**、**decider（决策）参数**；
- **标定参数**：`R_gimbal2imubody`、`camera_matrix`、`distort_coeffs`、`R_camera2gimbal`、`t_camera2gimbal`；
- **cboard（CAN 板）参数**、**gimbal（云台串口）参数**、**planner（规划器）参数**；
- **buff_detector / buff_aimer（能量机关）参数**。

各机器人配置（standard3/sentry/uav/standard4 等）是完整运行配置；`calibration.yaml` 只含标定工具所需的最小字段；`camera.yaml` 只含相机字段，被标定/相机测试程序单独引用。

### 3.2 configs/standard3.yaml（步兵 3 号，最详尽，重点逐项说明）

- `enemy_color: "red"`（L1，另有注释的 blue 备选）：敌方装甲板颜色，决定颜色识别与灯条颜色。
- 神经网络参数（L4-38）：
  - `yolo_name: yolov5`：选用哪个 YOLO 系列；
  - `classify_model: assets/tiny_resnet.onnx`：装甲板数字分类模型；
  - `yolo11_model_path / yolov8_model_path / yolov5_model_path`：三个版本的 YOLO 模型文件（xml，OpenVINO IR 格式）；
  - `device: CPU`：推理设备；
  - `performance_mode: throughput`：推理性能模式，throughput=吞吐优先 / latency=单帧延迟优先；注释给出实测数据（吞吐+多流 ~20ms/帧）；
  - `num_streams: 6`：OpenVINO 并发推理流数，是帧率主旋钮；注释详列 3~12 流的帧率/滞后实测表，并解释 6 流是更稳默认（帧龄上限 200ms 的硬规则，8 流 177ms 滞后只剩 23ms 裕量）；
  - `debug_window: true`：检测可视化窗口开关；
  - `min_confidence: 0.8`：检测置信度阈值；
  - `use_traditional: true`：是否启用传统视觉方法（与 YOLO 并行/兜底）。
- ROI（L40-47）：`roi{x,y,width,height}` 与 `use_roi: false` 是否启用 ROI 裁剪。
- 传统方法参数（L49-58）：`threshold:150` 二值化阈值；`max_angle_error:45` 灯条最大倾角误差(度)；`min/max_lightbar_ratio` 灯条长宽比范围；`min_lightbar_length:8` 灯条最小长度；`min/max_armor_ratio` 装甲板长宽比范围；`max_side_ratio:1.5` 两侧灯条比例上限；`max_rectangular_error:25` 矩形误差(度)。
- tracker 参数（L60-67）：`min_detect_count:5` 连续检测到多少帧才确认目标；`max_temp_lost_count:15` 暂时丢失多少帧内仍保留；`outpost_max_temp_lost_count:75` 前哨站特例的丢失容忍帧数；`max_dt:0.3` 帧间隔上限(秒)，超时判丢，注释解释因 CPU 推理 ~11fps 故从 0.1 放宽到 0.3。
- aimer 参数（L69-101）：核心瞄准偏置与预测参数。
  - `yaw_offset:0.5`、`pitch_offset:5.0`：常数瞄准偏置(度)，长注释记录了大量实测标定过程与"偏右→加大 yaw、偏下→减小 pitch"的调节规则；
  - `comming_angle:55`、`leaving_angle:20`：进入/离开(度)，用于切换瞄准策略的俯仰/来袭角度判定；
  - `decision_speed:3`：高/低速档判定阈值(rad/s)，按角速度绝对值判断；
  - `high_speed_delay_time:0.03`、`low_speed_delay_time:0.03`：发弹+云台响应延迟补偿(秒)，注释记录近距抖动排查将其从 0.1 降到 0.03；
  - `max_predict_time:0.15`：`to_now` 预测时域上限(秒)，钳制帧龄+延迟，注释记录 0.08→0.15 的调参过程；
  - `default_bullet_speed:23.0`：电控帧内弹速无效(<14m/s)时的兜底弹速。
- shooter 参数（L103-109）：`first_tolerance:3` 近距离射击容差(度)、`second_tolerance:2` 远距离容差(度)、`judge_distance:2` 近/远距离阈值(m)、`auto_fire:false` 是否由自瞄控制开火（首次联调建议 false）。
- 相机（L111-114）：`camera_name:"hikrobot"`、`exposure_ms:8.0`、`gain:14.0`、`vid_pid:"2bdf:0001"`（海康工业相机的 USB VID:PID）。
- 标定参数（L116-129）：
  - `R_gimbal2imubody`（L119）：云台系到 IMU body 系的旋转（此处单位阵）；
  - `camera_matrix`（L122）：3×3 内参（fx≈1785, cx≈672, fy≈1785, cy≈560）；上方注释"重投影误差: 0.1820px"；
  - `distort_coeffs`（L123）：5 个畸变系数（k1,k2,p1,p2,k3=0）；
  - `R_camera2gimbal / t_camera2gimbal`（L128-129）：手眼标定结果（相机相对云台的旋转矩阵与平移，单位 m），注释记录偏角 yaw1.44 pitch-7.28 roll0.96 与标定板距离 1.13m。
- cboard 参数（L131-135）：`quaternion_canid:0x100` 云台四元数 CAN ID、`bullet_speed_canid:0x101` 弹速 CAN ID、`send_canid:0xff` 发送 CAN ID、`can_interface:"can0"`。
- gimbal 参数（L137-145）：`com_port:"/dev/gimbal"` 云台串口软链；`crc_check:true` 是否校验云台帧 CRC（注释说明当前固件 crc16 疑似未实现，联调期可关）；`yaw_kp/yaw_kd/pitch_kp/pitch_kd` 云台 PD 参数（此处全 0，由电控侧决定）。
- planner 参数（L147-156）：`fire_thresh:0.0035` 开火阈值；`max_yaw_acc:50`、`max_pitch_acc:100` 云台最大加速度；`Q_yaw:[9e6,0]`、`R_yaw:[1]`、`Q_pitch:[9e6,0]`、`R_pitch:[1]` 卡尔曼滤波/规划器的过程噪声与观测噪声协方差。
- buff_detector 参数（L158-159）：`model:"assets/yolo11_buff_int8.xml"` 能量机关检测模型。
- buff_aimer 参数（L161-163）：`fire_gap_time:0.700` 发弹间隔(秒)、`predict_time:0.120` 预测时间(秒)。

### 3.3 configs/sentry.yaml（哨兵，含 USB 相机，逐项）

- `enemy_color:"blue"`（L2）。
- 神经网络参数（L5-13）：同 standard3，但无 `performance_mode/num_streams/debug_window`（用默认值）。
- ROI（L15-22）：同 standard3。
- **USB 相机参数**（L24-34）：
  - `image_width:1280 / image_height:720` 采集分辨率；`fov_h:57.7 / fov_v:56.7` 原始视场角(度)；
  - `new_fov_h:27 / new_fov_v:40.9` 裁剪/缩放后的等效视场角；
  - `usb_frame_rate:120` 帧率；`usb_exposure:500`（范围注释 1-80000）；`usb_gamma:160`；`usb_gain:10`（范围 0-96）。
- **工业相机参数**（L36-40）：`camera_name:"hikrobot"`、`exposure_ms:5.0`、`gain:15.0`、`vid_pid:"2bdf:0001"`。
- 传统方法参数（L42-52）：与 standard3 相同（末尾又重复了 `min_confidence:0.8`）。
- cboard 参数（L54-58）：`quaternion_canid:0x01`、`bullet_speed_canid:0x110`（与 standard3 不同，说明哨兵板 CAN ID 分配不同）、`send_canid:0xff`、`can_interface:"can0"`。
- tracker 参数（L60-63）：`max_temp_lost_count:25`（比步兵更容忍丢失）。
- aimer 参数（L65-74）：`yaw_offset:-0.8`、`pitch_offset:-1`；`comming_angle:60 / leaving_angle:20`；`left_yaw_offset:-1 / right_yaw_offset:-0.6`（左右不同偏置）；`decision_speed:10`；`high_speed_delay_time:0.026 / low_speed_delay_time:0.010`。
- shooter 参数（L76-80）：`first_tolerance:5`、`second_tolerance:2`、`judge_distance:3`、`auto_fire:true`。
- decider 参数（L82-83）：`mode:1`（决策器模式）。
- 标定参数（L85-96）：单位阵 `R_gimbal2imubody`；内参（fx≈2415, cx≈717, fy≈2418, cy≈583，重投影误差 0.1833px）；畸变；`R_camera2gimbal / t_camera2gimbal`（偏角 yaw-1.11 pitch0.01 roll-0.06，板距离 1.20m，板偏角 yaw123.89）。

### 3.4 configs/uav.yaml（无人机，含 USB 相机 + 能量机关，逐项）

- `enemy_color:"red"`（L1）。
- 神经网络参数（L5-13）：同标准，`use_traditional:true`。
- ROI（L15-22）：同标准。
- 工业相机参数（L24-28）：`camera_name:"mindvision"`（迈德威视）、`exposure_ms:8`、`gamma:0.6`、`vid_pid:"f622:d13a"`。
- USB 相机参数（L30-42）：`image_width:640 / image_height:360`，`new_image_width:1280 / new_image_height:720`（放大目标分辨率）；`fov_h:87.7 / fov_v:56.7`，`new_fov_h:67 / new_fov_v:40.9`；`usb_frame_rate:100`；`usb_exposure:315`；`usb_gamma:160`；`usb_gain:10`。
- 传统方法参数（L45-55）：同标准。
- cboard 参数（L57-61）：`quaternion_canid:0x001`、`bullet_speed_canid:0x110`、`send_canid:0xff`、`can_interface:"can0"`。
- tracker 参数（L63-66）：`min_detect_count:5`、`max_temp_lost_count:15`、`outpost_max_temp_lost_count:150`（前哨丢失容忍更大）。
- aimer 参数（L68-76）：`yaw_offset:0`、`pitch_offset:1.5`；`comming_angle:70 / leaving_angle:30`；`min_spin_speed:2`（最小自转速度）；`decision_speed:12`；`high/low_speed_delay_time:0.005`。
- decider 参数（L78-79）：`mode:1`。
- shooter 参数（L81-85）：`first_tolerance:2`、`second_tolerance:0.8`、`judge_distance:0.5`、`auto_fire:true`。
- 标定参数（L87-97）：单位阵 `R_gimbal2imubody`；内参（fx≈2925, cx≈647, fy≈2927, cy≈388，重投影误差 0.1032px）；畸变（k1=-0.589 较大，说明广角/短焦镜头畸变明显）；`R_camera2gimbal / t_camera2gimbal`（偏角 yaw0.02 pitch1.94 roll1.40，板距离 4.12m）。
- buff 参数（L100-105）：`model:"assets/yolo11_buff_int8.xml"`；`fire_gap_time:0.600`、`predict_time:0.090`。

### 3.5 configs/calibration.yaml（标定工具最小配置，逐项）

- `pattern_cols:10`（L1）：标定板列数（圆点列数）。
- `pattern_rows:7`（L2）：标定板行数。
- `center_distance_mm:40`（L3）：相邻圆点圆心距(mm)。
- `R_gimbal2imubody`（L5）：单位阵（云台↔IMU body 旋转，标定时先假设一致）。
- 相机字段（L7-10）：`camera_name:"hikrobot"`、`exposure_ms:3`、`gain:10.0`、`vid_pid:"2bdf:0001"`。
- cboard 参数（L12-16）：`quaternion_canid:0x01`、`bullet_speed_canid:0x110`、`send_canid:0xff`、`can_interface:"can0"`。

注意：该文件**不含** `camera_matrix / distort_coeffs`——它们是 calibrate_camera 的**输出**；而 calibrate_handeye/robotworld 又**需要**读这两个字段，因此实际使用顺序是：先 calibrate_camera 输出内参 → 手工把内参填进 calibration.yaml → 再跑 handeye。

### 3.6 configs/camera.yaml（纯相机配置，逐项）

- L1-4：注释掉的 mindvision 备选（`camera_name/exposure_ms/gamma/vid_pid`）。
- L6-9：当前生效配置：`camera_name:"hikrobot"`、`exposure_ms:8.0`、`gain:14.0`、`vid_pid:"2bdf:0001"`。

该文件只含相机字段，被 `tests/camera_test.cpp`、`tests/multi_usbcamera_test.cpp` 等单独引用。

### 3.7 其它 configs（一句话概括）

- **ascento.yaml**：哨兵/步兵"小能量机关"配置，`use_traditional:false`，`com_port:"/dev/gimbal"`，`crc_check:false`，`first/second_tolerance` 设为 100（宽容差测试用）。
- **demo.yaml**：离线 demo 配置，`R_gimbal2imubody` 为 -x/-y 翻转（`[-1,0,0,0,-1,0,0,0,1]`），含传统 buff 检测的 contrast/brightness/morphology 等完整 detect 子块与 buff_aimer 的 aim_time/wait_time。
- **example.yaml**：早期示例配置，mindvision 相机、USB 1920×1080，字段较旧、缺 use_roi/gimbal 等新字段。
- **mvs.yaml**：迈德威视(MVS) USB 相机配置，含 `new_image_width/height` 放大与 `new_usb_exposure` 双曝光。
- **standard4.yaml**：步兵 4 号，`crc_check:false`，`R_camera2gimbal` 为手工设定的理想位姿 `[0,0,1,-1,0,0,0,-1,0]`（非实测标定），含完整传统 buff detect 子块。

---

## 四、测试程序 tests

### 4.1 总述

`tests/` 是围绕各模块的**离线/在线验证小程序**，每个编译成一个独立可执行文件，用于在整机联调前单独验证某条链路。按验证对象可分组：

- **相机链路**：camera_test、usbcamera_test、multi_usbcamera_test、usbcamera_detect_test、camera_detect_test、cam_preview、camera_thread_test；
- **云台/CAN/IMU 链路**：cboard_test、dm_test、gimbal_test、fire_test、gimbal_response_test；
- **自瞄算法离线回放**：detector_video_test、auto_aim_test、auto_buff_test；
- **手眼标定验证**：handeye_test；
- **规划器**：planner_test、planner_test_offline；
- **ROS2 通信**：publish_test、subscribe_test、topic_loop_test；
- **整机最小闭环**：minimum_vision_system。

通用模式：`cv::CommandLineParser` 读参数 → `tools::Exiter`（全局退出标志，Ctrl-C 安全退出）→ 循环采集/处理 → `tools::Plotter` 把 JSON 数据画成实时曲线 → `q` 退出。除 ROS2 三个测试外，大多用 `Exiter` 而非裸 `while(true)`，退出干净。

### 文件: tests/camera_test.cpp

- **作用**：测工业相机（`io::Camera`）采集与帧率。
- **main 流程**：构造 Camera（默认 `configs/camera.yaml`）→ 循环 `camera.read(img, timestamp)` → 用相邻时间戳 `delta_time` 算 FPS 并 log → 可选 `--display` 显示。
- **优点**：极简、直观验证相机能出图且帧率正常。
- **缺点**：`display` 参数用 `cli.has("display")` 但 keys 里写的是 `{d display}`（`display` 为 flag 无值），用法正确但若传 `-d` 也能触发；无取流失败的显式提示（读空帧不退出也不告警）。

### 文件: tests/usbcamera_test.cpp

- **作用**：测单个 USB 相机（`io::USBCamera`）采集与帧率。
- **main 流程**：参数 `--name`（默认 `video0`）选设备 → 循环 `usbcam.read` → 算 FPS → `sleep 10ms` → 可选显示。
- **优点**：可直接指定 V4L2 设备名。
- **缺点**：FPS 统计含 10ms 人工 sleep，非纯相机帧率。

### 文件: tests/multi_usbcamera_test.cpp

- **作用**：同时测两个 USB 相机（video0、video2）+ 一个工业相机三路采集。
- **main 流程**：构造 `usbcam1("video0")`、`usbcam2("video2")`、`io::Camera("configs/camera.yaml")` → 循环三路 `read` → 统一按 timestamp 算 FPS → 三个窗口显示。
- **优点**：验证多路相机并存、USB 带宽/枚举是否正常。
- **缺点**：设备名硬编码 `video0/video2`，换机需改代码；三路共用一个 timestamp，FPS 以最后一次 read 的时间为准，只能反映整体循环频率。

### 文件: tests/usbcamera_detect_test.cpp

- **作用**：USB 相机 + YOLO 检测的端到端小测。
- **main 流程**：`USBCamera` + `YOLO(config, true)` → 循环读图 → `yolo.detect(img)` → FPS。
- **优点**：验证 USB 相机出图能被检测模型消费。
- **缺点**：检测结果未可视化、未校验，只看 FPS。

### 文件: tests/camera_detect_test.cpp

- **作用**：工业相机 + 传统检测器 / YOLO 二选一测帧率。
- **main 流程**：`io::Camera` + `Detector(config, true)` + `YOLO(config, true)` → 循环读图 → 按 `--tradition` 走 `detector.detect` 或 `yolo.detect` → 统计检测 FPS。
- **优点**：可对比传统方法与 YOLO 的帧率。
- **缺点**：`Detector`/`YOLO` 构造都传 `true`（可能是 debug/显示标志），与其它地方用法略不一致。

### 文件: tests/cam_preview.cpp

- **作用**：相机实时预览 + 检测框 + 帧率面板。用于联调时边看画面边摆装甲板。文件头注释非常详尽。

- **main 流程**（三线程架构，注释说明拆线程的原因）：
  - L38-53：构造 `Camera`、`MultiThreadDetector`，建窗口。
  - L56-68：原子量 `grabbing/running/cam_fps/proc_fps` + mutex 保护的 `latest_img/latest_armors/latest_t/has_latest`（渲染慢时最新帧被覆盖，不排队）。
  - L70-94 采集线程 `grabber`：`camera.read` → 指数平滑统计相机帧率 → `detector.push(raw, t_raw)`；退出时置 `grabbing=false`。
  - L96-123 推理线程 `inferer`：`detector.debug_pop()` 取结果 → 滑窗统计处理帧率 → 上锁把最新帧/装甲板发布给渲染线程；若采集线程已停且无 pending 则退出（避免死等）。
  - L125-178 主线程（渲染）：锁内拷贝最新帧 → 计算本帧滞后 age_ms → `vis.clone()` 后画装甲板多边形 + 颜色/类型文本 → 画帧率面板（处理帧率/相机帧率/检测数/滞后）→ imshow，q/ESC 退出。
  - L180-186：先 `running=false` 停采集，再 join 两个线程，最后销毁窗口。
- **优点**：采集/推理/渲染解耦，渲染慢不会拖累自瞄帧率（注释给实测 19.5 vs 31.6 fps、滞后 214→401ms 的对比）；用原子量 + 最新帧覆盖避免锁竞争与排队；退出顺序清晰。
- **缺点**：无装甲板时 `latest_armors` 为空但画面照常显示（合理）；`MultiThreadDetector` 的线程管理对调用方是黑盒，需配合其实现理解。

### 文件: tests/camera_thread_test.cpp

- **作用**：验证"多 YOLO 实例 + 线程池"并行检测流水线（`tools::create_yolov8s` 建 8 个实例 + `ThreadPool` 并发推理）。

- **main 流程**：
  - L29：全局 `OrderedQueue frame_queue`（有序队列）。
  - L32-36 `detect_frame`：对一帧做 yolo.detect 后 enqueue。
  - L52-67 process 线程：`frame_queue.dequeue()` 取检测结果，plot 装甲板数量。
  - L69-74：`create_yolov8s(config, 8, true)` 建 8 个 YOLO；`ThreadPool(8)`。
  - L83-119 主循环：`camera.read` → FPS → 提交 lambda 到线程池，其中用一个 `yolo_used` 数组找空闲 YOLO 实例，`img.clone()` 后 `detect_frame` 处理，用毕释放。
- **优点**：验证了"多实例 + 线程池 + 有序队列"的并行推理原型，是 mt_detector 的前身。
- **缺点/隐患**：
  - `yolo_used` 是 `std::vector<bool>`，但读写它时**没有用互斥锁保护**（L96 的 `std::mutex yolo_mutex` 声明后从未使用），多个线程池 worker 并发访问会数据竞争，虽不一定崩但可能重复分配同一实例；
  - `img.clone()` 每帧深拷贝、无背压，帧处理不过来时内存与队列会膨胀；
  - `process_thread` 里 `process_frame` 从队列取出但基本只数装甲板数量，未真正消费图像。

### 文件: tests/detector_video_test.cpp

- **作用**：离线视频 + 传统/YOLO 检测回放，画第一块装甲板的四个角点坐标曲线。
- **main 流程**：读 avi → `Detector`/`YOLO` → 循环 `video.read` → 按 `--tradition` 检测 → 若有装甲板，把 4 个角点像素坐标写 JSON 给 Plotter → `waitKey(33)`。
- **优点**：快速离线验证检测器在录播数据上的输出。
- **缺点**：只取 `armors.front()`，且无 imshow 可视化（注释掉的 GaussianBlur 未启用），主要靠曲线看结果。

### 文件: tests/auto_aim_test.cpp

- **作用**：自瞄算法**离线全链路回放**（YOLO→Solver→Tracker→Aimer），验证跟踪、解算、瞄准与 EKF 卡方检验，是自瞄核心的调试主程序。

- **main 流程**：
  - L18-23 参数：config 默认 `configs/demo.yaml`，`start-index/end-index`，`@input-path` 默认 `assets/demo/demo`。
  - L41-49：打开 avi + txt；构造 YOLO/Solver/Tracker/Aimer。
  - L58-62：`video.set` 起帧，并把 txt 前 start_index 行跳掉。
  - L64-195 主循环：
    - L67-73：读帧 + 读时间戳行（`t,w,x,y,z`），用 `t0 + t*1e6µs` 重建时间戳；
    - L76：`solver.set_R_gimbal2world({w,x,y,z})` 设置云台姿态；
    - L79：`yolo.detect(img, frame_count)`；
    - L82：`tracker.track(armors, timestamp)`；
    - L85：`aimer.aim(targets, timestamp, 27, false)`（弹速 27 硬编码）；
    - L87-92：若目标有效且命令相对上帧 yaw 变化 <2°，置 `command.shoot=true`（模拟开火）；
    - L96-100：打点 yolo/tracker/aimer 各段耗时；
    - L102-114：把 command 与云台 yaw 画到图上；
    - L116-134：收集装甲板观测数据到 JSON；
    - L136-187：若有目标，取 `target.armor_xyza_list()` 反投影画点（绿=观测、红=aim 点），并把 EKF 状态 `x/vx/y/vy/z/vz/a/w/r/l/h`、卡方检验 `nis/nees/nis_fail/nees_fail/recent_nis_failures` 等写入 JSON；
    - L189-194：Plotter 画曲线、imshow。
- **优点**：一条命令离线复现完整自瞄链，输出极其丰富的诊断数据（EKF 状态、卡方检验、各阶段耗时），非常适合定位跟踪/预测问题。
- **缺点**：弹速 27、`shoot` 判定阈值 2° 等硬编码；无传统检测分支；`last_target/last_t` 声明了但仅首次赋值后 `continue`，后续基本未用（残留代码）。

### 文件: tests/auto_buff_test.cpp

- **作用**：能量机关（大符）算法离线回放（Buff_Detector→Solver→BigTarget→Aimer）。

- **main 流程**：
  - L19-24：config 默认 `configs/sentry.yaml`。
  - L47-51：构造 `Buff_Detector/Solver/BigTarget/Aimer`（注释掉 SmallTarget）。
  - L65-174 主循环：读帧 + 时间戳 → `solver.set_R_gimbal2world` → `detector.detect(img)` 得到 `power_runes` → `solver.solve(power_runes)` → `target.get_target(power_runes, timestamp)` → `target_copy = target` → `aimer.aim(target_copy, timestamp, 22, false)`（弹速 22 硬编码）。
  - L98-152：收集能量机关 R 的 yaw/pitch/dis 观测、预测位姿反投影画点（绿=当前、红=预测）、EKF 状态（R_yaw/R_V_yaw/R_pitch/R_dis/yaw/angle/spd/a/w/fi/spd0）。
  - L154-162：云台 yaw/pitch 与命令输出。
  - L168-173：`cv::waitKey(1)`，按空格暂停、q 退出（`while(key==' ')` 内再 waitKey 实现单步暂停）。
- **优点**：离线验证大符全链路，含观测/预测反投影对比，便于调 buff 模型与 EKF。
- **缺点**：`auto_buff::Solver`/`BigTarget` 等命名（大小写）与 auto_aim 不一致；弹速硬编码；空格暂停逻辑有嵌套 waitKey，体验一般。

### 文件: tests/cboard_test.cpp

- **作用**：测 CBoard（CAN 板）读云台 IMU 四元数与弹速。

- **main 流程**：构造 `CBoard(config)` → 循环 `cboard.imu_at(timestamp)` 取姿态 → 转 ZYX 欧拉角(度)打印 z/y/x → 打印 `cboard.bullet_speed`。
- **优点**：验证 CAN 链路的 IMU 与弹速读取。
- **缺点**：默认 config 是 `configs/standard.yaml`，但 configs 目录下**没有 standard.yaml**（只有 standard3/standard4），默认跑会失败（残留路径）。

### 文件: tests/dm_test.cpp

- **作用**：测 `io::DM_IMU`（大疆/达妙 DM 电机内置 IMU）读姿态。
- **main 流程**：构造 `DM_IMU` → 循环 `imu.imu_at(timestamp)` → 转欧拉角打印。
- **优点**：验证 DM IMU 直读链路（区别于 CBoard 的 CAN 链路）。
- **缺点**：与 cboard_test 高度重复，仅数据源不同；无单位/坐标系说明。

### 文件: tests/gimbal_test.cpp

- **作用**：测云台 `io::Gimbal`（串口）的状态读取与开火控制。
- **main 流程**：
  - L32-41：构造 Gimbal，记录初始 mode、bullet_count、开火时间戳。
  - L43-94 循环：读 mode（变化则 log）→ 读 state/q → 由 `bullet_count` 增加判断是否已开火，首次开火打印耗时 → 每 0.2s 开/1s 关周期切换 fire 标志 → `gimbal.send(true, test_fire && fire, ...)` 发送（yaw/pitch 等全 0）→ 把 q_yaw/q_pitch/yaw/vyaw/pitch/vpitch/bullet_speed/bullet_count/fired/fire 画曲线 → sleep 9ms。
  - L96：退出前发送停火。
- **优点**：完整验证云台收发与开火闭环，用弹量变化客观判断是否真的开火。
- **缺点**：`--f` 参数实际控制是否允许开火，但 keys 里 `f` 定义为 `bool` 且无默认值，`cli.get<bool>("f")` 在未传时行为依赖 OpenCV 版本（可能抛异常）。

### 文件: tests/fire_test.cpp

- **作用**：用 `io::Gimbal` + `VisionToGimbal` 结构体直接发开火命令，验证发弹链路。
- **main 流程**：构造 Gimbal → 每 1.6s 置 `plan.mode=2`（开火）并 log "fire!"，其余时间 mode=1 → `gimbal.send(plan)` → plot shoot 标志 → q 退出。
- **优点**：验证 mode 位控制开火的最简路径。
- **缺点**：`plan` 各字段（yaw/pitch 等）全 0，仅测 mode；1.6s 周期硬编码。

### 文件: tests/gimbal_response_test.cpp

- **作用**：云台**响应/频响测试**，发三角波/阶跃/正弦信号，对比命令角与实际 IMU 角。

- **main 流程**：
  - L23-37：`yaw_cal/pitch_cal` 生成幅度 7°、周期 4s 的正弦（pitch 有 π/2 相位与 18° 初相）。
  - L39-58：参数 `--delta-angle`(8°)、`--circle`(0.2)、`--signal-mode`(triangle_wave)、`--axis`(yaw)。`slice=circle*100`（切片数），`dangle=delta_angle/slice`。
  - L67-69：发一个 `init_command{1,0,0,0}`（control=true，角度 0），sleep 5s 等云台归零。
  - L80-163 主循环：
    - L86：`cboard.imu_at` 取实际姿态；
    - **triangle_wave**（L90-123）：每 slice 个切片让角度从 0 增到 delta_angle 再复位；按 axis 设置 `command.yaw` 或 `command.pitch`；`cboard.send(command)`；plot cmd/last_cmd/实际角。
    - **step**（L125-140）：每 300 拍 `cmd_angle += delta_angle`，`command={1,0,limit_rad(cmd_angle/57.3),0}`（注意按 `io::Command` 字段序 control,shoot,yaw,pitch 此初始化实为 yaw=cmd_angle，因此**step 模式始终测 yaw，忽略 --axis**），plot cmd_yaw 等。
    - **circle**（L142-162）：`command.yaw=yaw_cal(t)/57.3; command.pitch=pitch_cal(t)/57.3` 画圆/正弦轨迹，每 2s 时间跳跃 2.4s。
- **优点**：多种激励信号（三角波扫幅、阶跃、正弦），能把命令与实际响应画成曲线对比，用于测云台带宽、滞后、超调。
- **缺点/隐患**：
  - `io::Command` 字段顺序为 `control, shoot, yaw, pitch, horizon_distance...`，triangle_wave 分支里**从未显式设 `command.control=true`**（只在 `count==slice` 复位时 `command={1,0,0,0}` 设了一次），导致**第一个三角波斜坡阶段 control 仍为初始的 false**，云台不会被控制；
  - step 模式忽略 `--axis`，且 yaw/pitch 混用 `tools::limit_rad`；
  - circle 模式里 `if (t - last_t > 2) { t += 2.4; ... }` 的时间跳变意图不明，可能为跳过某相位，属"魔法数"。

### 文件: tests/handeye_test.cpp

- **作用**：验证手眼标定结果——把一组世界系 3D 点经 `Solver` 投影回像素，观察是否与预期位置吻合。
- **main 流程**：
  - L37-41：从 yaml 读 `height/grid_num/grid_size/delay`（默认 config 为 `configs/handeye.yaml`）。
  - L50-55：生成 2×grid_num² 个 3D 点（在高度 -height 的平面上铺网格）。
  - L56-74 循环：`camera.read` → `q = cboard.imu_at(t - 1ms*delay)` → `solver.set_R_gimbal2world(q)` → `solver.world2pixel(points)` → 把投影点画白点 + 画 yaw/pitch/roll → 可选显示。
- **优点**：可视化校验标定外参 + 时间对齐（带 delay 补偿）。
- **缺点/隐患**：默认配置路径 `configs/handeye.yaml` 在仓库中**不存在**（configs 目录无此文件），默认运行会直接失败。

### 文件: tests/planner_test.cpp

- **作用**：测云台规划器 `Planner`（真机在线），用仿真目标驱动规划并下发云台。
- **main 流程**：`Planner` + `Gimbal` + 仿真 `Target(d, w, 0.2, 0.1)` → 循环 `target.predict(0.01)` 推进目标 → `planner.plan(target, gs.bullet_speed)` → `gimbal.send(...)` 下发 → plot 云台实际角/目标角/规划角、速度、加速度。
- **优点**：真机验证规划器输出与云台执行。
- **缺点**：目标为纯仿真匀速/旋转模型，不反映真实检测噪声；退出前有停火保护。

### 文件: tests/planner_test_offline.cpp

- **作用**：同上，但**不接云台**，`planner.plan(target, 22)` 用固定弹速 22 离线验证规划器输出。
- **main 流程**：同 planner_test，去掉 Gimbal，直接 plot 规划结果。
- **优点**：无需硬件即可验证规划器数值行为。
- **缺点**：弹速硬编码 22。

### 文件: tests/publish_test.cpp

- **作用**：测 ROS2 发布（`io::ROS2::publish`）。
- **main 流程**：构造 `io::ROS2` → 循环发布 `Eigen::Vector4d`（x=i, y=i+1, z=1, w=ArmorName::sentry+1 当作浮点），i 递增，1s 间隔，>1000 退出。
- **优点**：验证 ROS2 发布通道。
- **缺点**：把枚举名强转 double 塞进 Vector4d 是 hack 用法，仅作链路测试。

### 文件: tests/subscribe_test.cpp

- **作用**：测 ROS2 订阅（`subscribe_enemy_status`）。
- **main 流程**：构造 `io::ROS2` → 循环取无敌敌方 id 列表并打印，0.5ms 间隔。
- **优点**：验证 ROS2 订阅通道。
- **缺点**：注释掉的计数逻辑残留；0.5ms 轮询间隔过短，实际是忙等。

### 文件: tests/topic_loop_test.cpp

- **作用**：测 ROS2 同 topic 的发布+订阅回环。
- **main 流程**：构造 `io::ROS2` + `create_publisher<EnemyStatusMsg>` → 循环发布 `{1,2,3}` 无敌 id 与时间戳，每 3 拍 `subscribe_enemy_status()` 收一次，1s 间隔。
- **优点**：验证自定义消息类型（EnemyStatusMsg）收发闭环与时间戳。
- **缺点**：`RCLCPP_INFO` 的 logger 名写成描述性句子（"msg send timestamp is"），非标准命名。

### 文件: tests/minimum_vision_system.cpp

- **作用**：**最小整机视觉闭环**——把相机、DM IMU、多线程检测、Solver、Tracker、Aimer、Shooter 串起来跑，是离 mt_standard 最近的自测程序。

- **main 流程**：
  - L34-41：构造 Camera、DM_IMU、MultiThreadDetector、Solver、Tracker、Aimer、Shooter。
  - L43-51 detect 线程：`camera.read` → `detector.push(img, t)`。
  - L56-148 主循环：
    - L57：`detector.debug_pop()` 取检测结果；
    - L59-61：`dm_imu.imu_at(t)` 取姿态 → `solver.set_R_gimbal2world(q)`；
    - L63：求云台欧拉角；
    - L65：`tracker.track(armors, t)`；
    - L67：`aimer.aim(targets, t, 22)`（弹速 22 硬编码）；
    - L69：`shooter.shoot(command, aimer, targets, gimbal_pos)`（决策开火，但此处无 Gimbal/CBoard，实际不真正下发）；
    - L71-76：dt/fps 画曲线；
    - L78-93：从 armors 里挑**最靠左**（center.x 最小）的装甲板 `solver.solve`，收集观测数据；
    - L95-143：若有目标，反投影画观测点（绿）与瞄准点（红/蓝），并 dump EKF 状态与卡方检验数据；
    - L144-147：imshow + q 退出。
  - L150：join detect 线程。
- **优点**：单文件串联几乎全部视觉模块，能快速验证"相机→检测→跟踪→瞄准→射击决策"整条链路；检测线程与主循环分离。
- **缺点**：
  - 没有真正的云台/CAN 输出（`shooter.shoot` 只算不算发），不是真正闭环；
  - 弹速 22 硬编码；挑选目标用"最靠左"启发式，与真实 decider 策略可能不一致；
  - `detect_thread` 与主循环共用 `exiter`，但主循环退出后 detect 线程可能仍在阻塞 `detector.push`，join 前无 push 停止信号。

---

## 本部分整体优缺点总结

**优点**：
1. **标定工具链完整且分工清晰**：capture → calibrate_camera → handeye/robotworld_handeye 一环扣一环，数据格式（jpg + wxyz txt、YAML Flow 输出）前后一致，可直接粘贴进运行配置，减少了手工拷贝出错。
2. **脚本工程质量高**：setup_gimbal_udev.sh 对空 serial、多设备、空规则、原子写等边界情况都有防护；gimbal_monitor.py 用纯标准库实现，帧解析鲁棒、有 CRC 降级方案与统计/退出码，适合现场联调。
3. **配置体系清晰、可读性强**：字段按功能分组、大量中文注释记录了实测数据与调参依据（如 standard3.yaml 中 num_streams 的帧率/滞后实测表、aimer 各偏置的推导过程），是宝贵的工程知识沉淀。
4. **测试程序覆盖面广**：从单模块（相机/CAN/IMU/串口/ROS2/规划器）到离线全链路回放（auto_aim_test、auto_buff_test）再到最小整机闭环（minimum_vision_system），层次分明；`tools::Exiter/Plotter` 的统一封装让测试程序安全退出并支持实时曲线。
5. **离线回放测试极有价值**：auto_aim_test/auto_buff_test 输出 EKF 状态、NIS/NEES 卡方检验、各阶段耗时等诊断数据，可在无硬件时定位算法问题。

**缺点/隐患（共性问题）**：
1. **硬编码与魔法数较多**：多处弹速（22/27）、开火阈值（2°）、发弹周期（1.6s）、相机设备名（video0/video2）等直接写死，换配置/换机需改源码。
2. **残留/过期默认路径**：cboard_test 默认 `configs/standard.yaml`、handeye_test 默认 `configs/handeye.yaml` 在 configs 目录中均不存在，默认运行即失败。
3. **并发正确性问题**：camera_thread_test.cpp 中 `yolo_used` 数组在无锁下被多线程读写（`std::mutex yolo_mutex` 声明了却未用），存在数据竞争。
4. **gimbal_response_test.cpp 存在疑似 bug**：triangle_wave 模式下 `command.control` 未在循环开始时置 true，导致首个斜坡阶段云台不受控；step 模式忽略 `--axis` 参数。
5. **手眼标定中的平移近似**：handeye/robotworld 标定把云台相对世界的平移恒置 0，引入了系统误差，可能影响近距离外参精度。
6. **错误处理参差**：部分测试（如 camera_test、usbcamera 系列）读帧失败不告警不退出，可能静默输出错误的 0/异常 FPS。
7. **命名与风格不统一**：`auto_buff` 与 `auto_aim` 的类命名大小写风格不一致；部分文件（fire_test、camera_test 等）注释与 keys 文案有重复或误导（如 split_video 的 output-path 与 input-path help 文案相同）。
