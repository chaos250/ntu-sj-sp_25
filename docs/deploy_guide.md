# 小电脑部署指南(sp_vision_25)

从开发机打包 → 小电脑解包装依赖 → 编译 → 开机自启 → 接电控联调,共 7 步。
按顺序执行即可;第 0 步在**开发机**上做,第 1~7 步在**小电脑**上做。

---

## 第 0 步:开发机上打包(拿到 U 盘 / 局域网传输)

打包整个仓库目录(必须整目录打包,不能只打包 git 跟踪的文件——构建需要几个未跟踪文件)。

```bash
cd ~/共享文件/tongji/sp_vision_25
tar \
  --exclude=build \
  --exclude=abseil-cpp --exclude=ceres-solver --exclude=ceres-solver-2.1.0 --exclude=ceres-solver-2.1.0.tar.gz \
  --exclude=logs --exclude=MvSdkLog --exclude=imgs --exclude=testvideos \
  --exclude=records --exclude=records_24 --exclude=records_25 --exclude=records_new --exclude=records_auto_buff_big \
  --exclude=assets/img_with_q \
  -czf ~/sp_vision_25_小电脑部署包.tar.gz .
ls -lh ~/sp_vision_25_小电脑部署包.tar.gz   # 期望约 100~200MB
```

排除的都是构建/运行不需要的:三方源码(ceres/abseil 用系统的)、编译产物、录屏数据。
**必须保留**:`io/hikrobot`、`io/mindvision`(相机 SDK 库在仓库内)、`assets`(模型)、根目录的 `openvino_toolkit_ubuntu22_2025.4.0.*.tgz`(第 1 步要用)。

传到小电脑(二选一):
- U 盘拷贝过去;
- 或同一局域网 scp:`scp ~/sp_vision_25_小电脑部署包.tar.gz 小电脑用户名@小电脑IP:~`

## 第 1 步:小电脑上解包 + 装依赖(需联网)

```bash
mkdir -p ~/sp_vision_25
tar -xzf ~/sp_vision_25_小电脑部署包.tar.gz -C ~/sp_vision_25
cd ~/sp_vision_25
sudo bash scripts/setup_deps.sh openvino_toolkit_ubuntu22_2025.4.0.20398.8fdad55727d_x86_64.tgz
```

setup_deps.sh 干三件事:
1. apt 安装编译工具链 + 依赖库(g++/cmake/opencv/eigen/spdlog/yaml-cpp/ceres/libusb/ccache/screen)——**需要联网**,约几百 MB;
2. 把你的账号加入 `dialout` 组(串口权限);
3. 解压 OpenVINO 到 `/opt/intel/openvino_2025.4.0` 并装它的运行时依赖。

看到 `完成。后续提示:` 即成功。若 apt 报网络错误,先 `ping baidu.com` 确认小电脑已联网(或插网线/连 WiFi)。

## 第 2 步:注销重新登录(dialout 组生效)

- 图形界面:右上角电源菜单 → **注销**,重新登录;
- 或终端执行 `gnome-session-quit --no-prompt`。

验证:`groups | grep dialout` 有输出即生效。不做这步,后面打不开 `/dev/gimbal`。

## 第 3 步:编译(约 10~20 分钟)

```bash
cd ~/sp_vision_25
cmake -B build -S .
cmake --build build -j$(nproc) --target mt_standard
```

结束无报错即成功,产物 `build/mt_standard`。ccache 首次无缓存,时间取决于核数;期间别开重活。
若报缺库(如 `No package 'xxx' found`),按提示 `sudo apt-get install` 补装即可(第 1 步已装齐,正常不会遇到)。

## 第 4 步:冒烟验证(没接相机/电控也能做)

```bash
./build/mt_standard
```

不带参数只打印用法并退出,不碰硬件。能看到 Usage 说明 = 程序本身没问题。
接上相机后可以再跑一次完整程序(`./build/mt_standard configs/standard3.yaml`),能稳定出检测输出即 OK。

## 第 5 步:注册开机自启

```bash
cd ~/sp_vision_25
mkdir -p ~/.config/autostart
printf '[Desktop Entry]\nType=Application\nName=sp_vision\nExec=%s/autostart.sh\nX-GNOME-Autostart-enabled=true\n' "$PWD" > ~/.config/autostart/sp_vision.desktop
cat ~/.config/autostart/sp_vision.desktop   # 确认 Exec= 指向 ~/sp_vision_25/autostart.sh
```

用 `$PWD` 自动填路径,不用手改。**前提**:小电脑账号要能登录进桌面(建议设置 → 用户 → 打开"自动登录",这样开机后自动触发)。登录后自启链路:`autostart.sh`(延时 5s)→ screen 会话里跑 `watchdog.sh` → 视觉程序;崩溃自动拉起,日志在 `logs/`。

> 注意:此时电控/相机没接的话,程序会启动失败退出,watchdog 会退避重试(2s→4s→…→60s 封顶,不会刷屏);设备接好后下一次重试自动恢复正常。所以自启可以先注册,不影响。

## 第 6 步:插电控 → 建 /dev/gimbal 串口软链

电控板用 USB 线接到小电脑后:

```bash
ls /dev/ttyACM*                          # 确认电控串口已枚举(如 ttyACM0)
sudo bash ~/sp_vision_25/scripts/setup_gimbal_udev.sh
ls -l /dev/gimbal                        # 期望:gimbal -> ttyACM0
```

脚本自动找 `ttyACM*` 并写 udev 规则(按 VID/PID 匹配,以后每次插拔都自动建软链,只需跑一次)。
若 `ls -l /dev/gimbal` 没有:USB 拔插一次再看。程序只认 `/dev/gimbal` 这个名字。

## 第 7 步:验收 + 联调当天流程

```bash
cd ~/sp_vision_25
screen -ls                                # 自启后应有一个 screen 会话
tail -f logs/watchdog.log                 # 看启动/重启记录
screen -r                                 # 附加进去看程序实时输出;Ctrl+A 再按 D 退出(不杀程序)
```

**联调当天上电顺序:先电控,后小电脑**(视觉启动 3 秒内等电控首帧,之后进入正常流程;顺序反了 watchdog 也会自动重试接上)。

改配置后重启:`screen -r` 附加 → 按 `Ctrl+C` 杀掉程序,watchdog 2 秒后自动用新配置拉起。

## 联调前确认清单(和电控当面核对)

1. `configs/standard3.yaml` 里 `default_bullet_speed: 23.0` 改成**实际弹速**;
2. 电控按 [docs/sp_protocol.md](sp_protocol.md) 实现 43B/29B 帧,先对 CRC 测试向量:`"SP" → 0x5BA2`;
3. 静态目标实测 yaw/pitch 符号(偏左偏上目标,看云台转向),只调 `yaw_offset`/`pitch_offset`;
4. 电控回传 CRC 且双方对表通过后,把 `crc_check: false` 改回 `true`;
5. 电控 mode 语义(0 空闲/1 自瞄/2 小符/3 大符)和视觉发 0(不控制)时的行为提前说定。
