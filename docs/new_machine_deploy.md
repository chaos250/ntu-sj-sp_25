# 新机部署与适配清单(sp_vision_25)

> 生成时间:2026-10-21,基于当前小电脑(root1-NUC8i5BEH)与仓库 `main` 分支实测。
> 定位:本文与 [deploy_guide.md](deploy_guide.md) 配套 —— deploy_guide 是"按部就班的 7 步操作",本文是**框架与环境的全景图 + 换机时必须逐项检查/调整的清单**,并补上 deploy_guide 未覆盖的环境项(尤其海康 MVS SDK)。

---

## 1. 框架总览

### 1.1 运行链(开机自启)

```
~/.config/autostart/sp_vision.desktop          # XDG 自启(需要账号自动登录进桌面)
  └─ autostart.sh                              # sleep 5 等设备枚举,起独立 screen 会话
       └─ screen -L -Logfile logs/autostart-*.screenlog
            └─ watchdog.sh                     # 守护进程,崩溃指数退避重启(2s→60s 封顶)
                 └─ ./build/mt_standard configs/standard3.yaml   # 当前实车主程序
```

- watchdog.sh 支持环境变量覆盖:`VISION_BIN`(默认 `./build/mt_standard`)、`VISION_CONFIG`(默认 `configs/standard3.yaml`)、`RESTART_DELAY`(默认 2s)。
- watchdog 与程序都 `cd` 到仓库根运行——**模型、配置、日志全是相对路径,程序必须在仓库根目录启动**;拷贝到新机任意路径都可以,但 `.desktop` 里的 `Exec=` 必须指到新路径。

### 1.2 模块结构

| 目录 | 内容 |
|---|---|
| `src/` | 各车种主程序(见 1.3) |
| `io/` | 设备接入:海康相机 `hikrobot/`、迈德威视 `mindvision/`、USB 相机 `usbcamera/`、云台串口 `gimbal/`、CAN `cboard/`、串口 `cboard1/`、IMU `dm_imu/`、ROS2 `ros2/`、vendored 串口库 `serial/` |
| `tasks/` | 算法:自瞄 `auto_aim/`(YOLO 检测+跟踪+solver+aimer+shooter+planner/TinyMPC)、打符 `auto_buff/`、全向感知 `omniperception/` |
| `tools/` | 静态库:logger、plotter(UDP→PlotJuggler)、EKF、PID、recorder、CRC |
| `configs/` | 全部 yaml 配置(见 1.4) |
| `calibration/` | 标定工具链:capture → calibrate_camera → calibrate_handeye / calibrate_robotworld_handeye |
| `assets/` | 模型(yolov5/v8/11 xml+bin、tiny_resnet.onnx 等)与离线测试数据,按 `assets/...` 相对路径加载 |
| `scripts/` | `setup_deps.sh`(装依赖+解压 OpenVINO)、`setup_gimbal_udev.sh`(建 /dev/gimbal)、`gimbal_monitor.py`(串口联调) |
| `docs/` | 协议/坐标/部署文档 |

### 1.3 可执行文件与用途

| 可执行 | main | 默认配置 | 用途 |
|---|---|---|---|
| **mt_standard** | src/mt_standard.cpp | 无默认(watchdog 传 standard3.yaml) | **当前实车主程序**:/dev/gimbal 串口 + 多流推理 + 决策线程,自瞄/大小符按电控 mode 切换 |
| standard | src/standard.cpp | standard3.yaml | 老版步兵(CAN + 单线程) |
| standard1 | src/standard1.cpp | standard3.yaml | CBoard1 串口版(硬编码 /dev/ttyACM0) |
| standard_mpc | src/standard_mpc.cpp | 无默认 | Gimbal 串口 + Planner(TinyMPC)版 |
| auto_aim_debug_mpc | src/auto_aim_debug_mpc.cpp | sentry.yaml | 自瞄规划调试(重投影可视化) |
| auto_buff_debug(_mpc) | src/auto_buff_debug(_mpc).cpp | 无默认 | 打符调试(CAN / 串口) |
| sentry* | src/sentry*.cpp | sentry.yaml + camera.yaml | 哨兵(**仅检出 ROS2 环境时编译**,可选) |
| capture / calibrate_camera / calibrate_handeye / calibrate_robotworld_handeye | calibration/*.cpp | -c configs/calibration.yaml | 标定工具链 |
| tests/ 约 20 个 | tests/*.cpp | 各异 | 模块测试/联调 |

### 1.4 配置与标定体系

- 加载方式:各 main 用 `cv::CommandLineParser` 收位置参数 `@config-path`(`./build/mt_standard configs/standard3.yaml`);无统一 config 中心,各模块自行 `YAML::LoadFile`。
- `configs/` 现状(各有独立标定值,换车要用对应车的那份或重标):

| 配置 | 车种/场景 |
|---|---|
| **standard3.yaml** | 当前实车主配置(watchdog 默认加载,2026-10-18 最新) |
| standard4.yaml | 3/4 号步兵国赛配置 |
| ascento.yaml | 英雄 |
| mvs.yaml | 迈德威视/海康 USB 相机场景 |
| sentry.yaml | 哨兵(仅 ROS 环境) |
| uav.yaml | 无人机(未编译) |
| demo.yaml / example.yaml | 演示/模板 |
| camera.yaml | 单独相机参数(哨兵后置相机) |
| calibration.yaml | 标定专用(手眼标定前须先把内参结果补进去) |

- **标定结果没有独立产物文件**:相机内参、手眼矩阵全部以键值+注释内嵌在各 yaml 的 `camera_matrix / distort_coeffs / R_camera2gimbal / t_camera2gimbal / R_gimbal2imubody` 段,由 `tasks/auto_aim/solver.cpp`、`tasks/auto_buff/buff_solver.cpp` 读取。换车 = 整段复制原车标定(硬件不变才有效)或重标后人工粘贴。

---

## 2. 当前小电脑环境快照(2026-10-21 实测)

| 项 | 当前值 |
|---|---|
| 系统 | Ubuntu 22.04.5 LTS,kernel 6.8.0-101-generic,x86_64 |
| 硬件 | Intel NUC(i5-1135G7,**8 线程**,15GiB 内存,NVMe 208G)。注:readme.md 描述的目标环境是 NUC12WSKI7(i7-1260P),若新机是后者,CPU 代际不同但同为 x86_64 8+ 线程,依赖兼容 |
| 用户/组 | 账号在 `dialout`(串口)、`plugdev` 组 |
| OpenVINO | `/opt/intel/openvino_2024.6.0` + `/opt/intel/openvino_2025.4.0` 并存;CMake 优先 2025.4.0,回退 2024.6.0 |
| 海康 MVS SDK | `/opt/MVS`(库版本 4.5.0.3) |
| apt 依赖 | OpenCV 4.5.4、fmt 8.1.1、Eigen 3.4.0、spdlog、yaml-cpp、nlohmann-json、Ceres(本机源码装的 /usr/local,新机用 apt libceres-dev)、libusb |
| 相机 | **海康 MV-CA013-21UC**,USB3 Vision,VID/PID `2bdf:0001`;hikrobot 封装只枚举 USB 相机、打开第 0 台,不设分辨率(以相机内部保存的参数为准) |
| 云台串口 | STM32 "Vision Comm port" `0483:00ca`,serial `206538474D4B` → udev 规则建软链 **`/dev/gimbal`** → 主程序只认这个名字;波特率 115200 |
| udev 规则 | ① `80-drivers-SDK-2bdf.rules`(海康相机 USB 0666,MVS 安装器生成)② `99-usb-serial.rules`(gimbal 软链)③ `99-ttyUSB-permissions.rules`(ttyUSB0 0777,旧遗留,无 ttyUSB 设备可不建) |
| 自启 | `~/.config/autostart/sp_vision.desktop`,`Exec=/home/root1/共享文件/tongji/sp_vision_25/autostart.sh`(**硬编码绝对路径,换机必须重新生成**);另有 x11vnc.desktop 供远程看画面 |
| 网络 | NetworkManager 管理,WiFi 192.168.1.135/24;enp89s0 未用(相机走 USB3,无 GigE/静态 IP 配置) |
| 调试 | Plotter UDP → 127.0.0.1:9870(PlotJuggler 本机跑),布局文件 buff_layout.xml / mpc_layout.xml |

### 2.1 ⚠️ MVS SDK 环境变量(新机最容易漏的一环)

本机在 **`/etc/profile`、`~/.profile`、`~/.bashrc` 三处**都导出了以下 5 行,缺了相机枚举不到(运行时实测 `mt_standard` 加载的是 `/opt/MVS/lib/64/libMvCameraControl.so`,而非仓库自带副本;U3V 传输层 `libMvUsb3vTL.so`、GenICam CLProtocol 靠 `MVCAM_*` 定位):

```bash
export MVCAM_SDK_PATH=/opt/MVS
export MVCAM_COMMON_RUNENV=/opt/MVS/lib
export MVCAM_GENICAM_CLPROTOCOL=/opt/MVS/lib/CLProtocol
export ALLUSERSPROFILE=/opt/MVS/MVFG
export LD_LIBRARY_PATH=/opt/MVS/lib/64:/opt/MVS/lib/32:$LD_LIBRARY_PATH
```

写入 `/etc/profile`(全局)+ `~/.profile`(桌面会话,自启链路靠它)即可;deploy_guide.md 没有这一步,是新机部署必须补上的缺口。

---

## 3. 新机部署:环境侧清单

按 [deploy_guide.md](deploy_guide.md) 第 0~7 步执行(打包 → 解包+setup_deps.sh → 注销重登 → 编译 → 冒烟 → 注册自启 → 建 /dev/gimbal → 验收),**另外补以下 4 项**:

| # | 步骤 | 说明 |
|---|---|---|
| 0a | **安装海康 MVS SDK 到 /opt/MVS**(版本对齐 4.5.0.3) | 安装包不在仓库里(仓库只带 libMvCameraControl.so,传输层/CLProtocol 不在),需另备 MVS Linux 安装包;装完确认生成 `/etc/udev/rules.d/80-drivers-SDK-2bdf.rules`(相机 USB 0666 权限) |
| 0b | **写入 §2.1 的 5 行 MVS 环境变量**到 `/etc/profile` 和 `~/.profile`,注销重登验证 `echo $MVCAM_SDK_PATH` | 不写这步,程序能编译、能启动,但打开相机失败 |
| 0c | (可选)x11vnc + 自动登录 | 远程看画面/无键鼠调试用;自动登录是自启链路的前提(deploy_guide 第 5 步已提) |
| 0d | (可选)PlotJuggler(snap/apt)+ 布局文件 | 在线看 plotter 数据用;不影响主程序 |

打包传输注意(deploy_guide 第 0 步):
- 必须整目录打包(构建需要若干未跟踪文件);排除 build/、三方源码(abseil/ceres)、logs/、MvSdkLog/、imgs/、records*、assets/img_with_q/、plot.jsonl。
- 必须保留:`io/hikrobot`、`io/mindvision`(相机 SDK)、`assets/`(模型)、根目录 `openvino_toolkit_*.tgz`(setup_deps.sh 要解压到 /opt/intel)。
- `install/`、`log/` 是 colcon 旧产物(含 `/home/root1/...` 绝对前缀),**不要拷贝**;当前构建是纯 CMake,不需要它们。

---

## 4. 与具体机器/车辆绑定的配置项(换机必查)

先判别场景:

- **场景 A:换机不换车**——同一套相机+云台+电控整体搬到新小电脑:configs 可整份复制,只需复核第 4、1 行(弹速、offset),其余(内参/手眼/vid_pid)理论上不动。
- **场景 B:新车(自己的相机+云台+电控)**:下表"标定类"全部要重标,offset 要重调。

下表为 `configs/standard3.yaml` 当前值(其他车种配置各有各的值,思路相同):

| # | 参数 | 位置 | 当前值 | 何时要改 |
|---|---|---|---|---|
| 1 | `default_bullet_speed` | aimer 段 | 23.0 | **必查**:改成新电控实际弹速(15/18/23/30 等以实测为准) |
| 2 | `yaw_offset` / `pitch_offset` | aimer 段 | 8.0 / 5.0(本车实测调优,勿动) | 场景 A 不换;场景 B 重装后重新校:静态目标实测符号与量,只调这两项;注意 pitch_offset ↑ = 瞄准点抬高 |
| 3 | `camera_matrix` / `distort_coeffs` | 标定段 | 内参(重投影误差 0.1820px,主点 ≈672.5, 559.9) | 换相机/换分辨率必重标;同型号新相机也**先确认分辨率设置一致**(hikrobot 封装不设宽高,以相机内部参数为准,可用 MVS 客户端核对) |
| 4 | `R_camera2gimbal` / `t_camera2gimbal` / `R_gimbal2imubody` | 标定段 | 手眼矩阵(注释 yaw1.44 / pitch-7.28 / roll0.96;t≈[0.132, 0.104, 0.025]) | 相机与云台相对安装变过必重标(§4.1) |
| 5 | `vid_pid` | camera 段 | `2bdf:0001` | 同型号相机一致;换了相机型号用 `lsusb` 查新 VID/PID 改 |
| 6 | `com_port` | gimbal 段 | `/dev/gimbal` | **不改**(代码只认这个名字);但新电控板 serial 不同 → 新机跑一次 `sudo bash scripts/setup_gimbal_udev.sh` 重建 udev 软链 |
| 7 | `crc_check` | gimbal 段 | true(联调期曾关) | 电控 CRC 对表通过后改回 true;先用测试向量 `"SP" → 0x5BA2` 对表 |
| 8 | `exposure_ms` / `gain` | camera 段 | 8.0 / 14.0 | 同型号相机可沿用,场地光照变化时联调再调 |
| 9 | `num_streams` | mt_detector 段 | 6(8 核实测最优,12 流回落) | 同型号 CPU(8 线程)沿用;新机核数不同按 yaml 内实测表重调 |
| 10 | `device`(推理设备) | yolo 段 | CPU(classifier 用 AUTO) | 新机同为 x86_64 CPU 推理,无需改;无 GPU/VPU 依赖 |
| 11 | `auto_fire` | shooter 段 | false(联调期) | 联调通过后按需打开 |
| 12 | buff 参数(model/fire_gap_time/predict_time) | buff 段 | yolo11_buff_int8,0.700/0.120 | 打符车才需要核对 |

### 4.1 新车标定流程(场景 B)

1. 跑 `capture`(读电控 IMU 四元数+存图到 assets/img_with_q);
2. `calibrate_camera` 打印内参 YAML → 粘贴进目标配置;
3. 把内参先补进 `configs/calibration.yaml`,再跑 `calibrate_robotworld_handeye`(当前实车用的方案)→ 打印手眼矩阵与偏角注释 → 粘贴;
4. 重校 `yaw_offset`/`pitch_offset`(静态目标、yaw/pitch 各两个位置实测符号与量)。

---

## 5. 代码中与机器绑定的硬编码(换硬件/换场景才需要动代码)

| 位置 | 内容 | 影响 / 何时改 |
|---|---|---|
| io/hikrobot/hikrobot.cpp:73 | `EnumDevices(MV_USB_DEVICE)` 后打开 `device_list[0]` | 只认 USB 相机第 0 台;新机**只插一台**海康 USB 相机即可;插多台会开错 |
| io/hikrobot/hikrobot.cpp:90 | 帧率上限硬编码 150fps | 同型号相机无影响 |
| io/hikrobot/hikrobot.cpp:241-277 | 按 `vid_pid` 用 libusb 重置 USB | 换了相机型号要同步改配置里的 vid_pid |
| tasks/auto_aim/tracker.cpp:60,131 | 图像中心硬编码 1440/2、1080/2(带 TODO) | 与当前内参主点(≈672, 560)不一致,属已知待修项;若换相机/分辨率,此处与内参都要核对 |
| src/standard1.cpp:41 | CBoard1 硬编码 `/dev/ttyACM0` | 仅 standard1(CBoard1 串口版)受影响;当前主程序 mt_standard 走 /dev/gimbal,无碍 |
| io/dm_imu/dm_imu.cpp:40 | 达妙 IMU 硬编码 `/dev/ttyACM0`@921600 | 仅用达妙 IMU 的分支受影响 |
| scripts/gimbal_monitor.py | 默认串口 /dev/ttyACM0 | 联调工具,新机按 `ls /dev/ttyACM*` 传参即可 |
| CMakeLists.txt:4,25-30 | OpenVINO 路径 `/opt/intel/openvino_2025.4.0` → 回退 2024.6.0 | setup_deps.sh 会装到该路径;不按默认装才要改 |
| io/CMakeLists.txt:23-39 | 相机 SDK 按 `CMAKE_SYSTEM_PROCESSOR` 选 amd64/arm64 | 新机必须 x86_64(OpenVINO tgz 也是 x86_64 版);arm64 机型需另备 OpenVINO |
| tools/plotter.hpp:15 | UDP 默认 `127.0.0.1:9870` | 只能本机看;想远程看要改 host 或 ssh 转发 |
| tasks/auto_buff/yolo11_buff.cpp:14、tasks/auto_aim/classifier.cpp:14 | 推理设备硬编码 "CPU" / "AUTO" | 新机同为 CPU 推理,无影响 |
| src/sentry*.cpp、tests/*usbcamera* | `/dev/video0/2/4/6` | 仅哨兵/ROS 分支(用 UVC 相机)受影响;当前主程序不用 |
| .vscode/c_cpp_properties.json | includePath 含过时 `/opt/intel/openvino_2023.3.0/...`、`/home/rm/sp_msg_25/...` | 只影响 IDE 提示,不影响编译;新机如开 VSCode 顺手更新 |
| readme.md:96-105 | 旧自启示例 `Exec=/home/rm/Desktop/...` | 历史示例,勿照抄;按 deploy_guide 用 `$PWD` 生成 |
| tools/logger.cpp、watchdog.sh、autostart.sh | 日志相对 CWD(`logs/`),MvSdkLog 也生成在 CWD | 必须从仓库根运行(自启链已保证);换路径无需改代码 |

---

## 6. 新机验收清单

- [ ] `lsusb` 能看到 `2bdf:0001` 相机与 STM32 电控;`ls -l /dev/gimbal` 软链正常
- [ ] `echo $MVCAM_SDK_PATH` = /opt/MVS;`ldd build/mt_standard | grep MvCamera` 能解析(本机实测解析到 /opt/MVS/lib/64,属正常)
- [ ] `./build/mt_standard` 不带参数打印 Usage 并退出(不碰硬件)
- [ ] 接相机跑 `./build/mt_standard configs/standard3.yaml`,稳定出检测输出;`debug_window: true` 时能弹窗口(远程可用 x11vnc 看)
- [ ] `screen -ls` 有会话,`tail -f logs/watchdog.log` 无持续快速崩溃刷屏
- [ ] 重启小电脑验证自启链路(需要自动登录);改配置后用 `screen -r` 附加、Ctrl+C 杀掉,watchdog 2s 自动拉起
- [ ] 联调当天:先电控后小电脑上电;按 deploy_guide 第 7 步清单与电控对 CRC、弹速、mode 语义、yaw/pitch 符号

---

## 7. 已知风险与注意事项

1. **MVS 环境变量漏配**是新机部署最常见的坑:编译、启动都正常,唯独相机打开失败(MV_CC_EnumDevices 返回空)。§2.1 五行走 /etc/profile + ~/.profile 双保险。
2. **LD_LIBRARY_PATH 覆盖 RUNPATH**:本机 `mt_standard` 实际加载 `/opt/MVS/lib/64/libMvCameraControl.so`;仓库自带 SDK(io/hikrobot)与 /opt/MVS 版本都是 4.5.0.x,保持两者版本一致,避免混用不同版本的 lib 与 .cti 传输层。
3. **相机分辨率由相机内部保存的参数决定**(hikrobot.cpp 不设宽高):新相机必须用 MVS 客户端把分辨率等参数设成与原车一致,否则原内参直接失效。
4. tracker 图像中心 1440×1080 是硬编码(带 TODO),与当前标定主点不一致——换相机/改分辨率时这是隐蔽 bug 点。
5. OpenVINO tgz 为 x86_64 版,新机必须同架构;推理全走 CPU,对核数敏感(num_streams 按 8 核调过)。
6. 打包时排除 `plot.jsonl`(60MB+)与三方源码,否则部署包膨胀;`assets/demo`、records* 等数据目录按需带。
