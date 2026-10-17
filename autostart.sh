#!/usr/bin/env bash
# 开机自启入口:由 ~/.config/autostart/sp_vision.desktop 在登录后调用
#
# 作用:等系统/设备就绪后,在独立的 screen 会话里启动 watchdog.sh。
# 视觉程序崩溃后 watchdog 会自动拉起,运行日志与崩溃记录都落盘到 logs/。
# 调试时可随时用 `screen -r` 附加到该会话查看输出。
#
# 脚本按自身所在位置定位仓库根目录,拷到小电脑任意路径都能用,
# 只需保证 ~/.config/autostart/sp_vision.desktop 里的 Exec 指向它。
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

sleep 5  # 等待系统与设备就绪(相机、串口枚举)

cd "$SCRIPT_DIR" || exit 1
mkdir -p logs

screen \
    -L \
    -Logfile "$SCRIPT_DIR/logs/autostart-$(date '+%Y-%m-%d_%H-%M-%S').screenlog" \
    -d \
    -m \
    bash "$SCRIPT_DIR/watchdog.sh"
