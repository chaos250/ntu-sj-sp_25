#!/usr/bin/env bash
# 视觉程序守护进程:程序退出/崩溃后自动重启,日志写入 logs/watchdog.log
#
# 可用环境变量覆盖(在 autostart.sh 或 .desktop 里设置):
#   VISION_BIN     要运行的二进制(默认 ./build/standard_mpc)
#   VISION_CONFIG  配置文件(默认 configs/standard3.yaml)
#   RESTART_DELAY  崩溃后重启间隔秒数(默认 2)
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR" || exit 1  # assets 模型、logs/ 等都是相对路径,必须在仓库根目录运行

VISION_BIN="${VISION_BIN:-./build/standard_mpc}"
VISION_CONFIG="${VISION_CONFIG:-configs/standard3.yaml}"
RESTART_DELAY="${RESTART_DELAY:-2}"

mkdir -p logs

echo "[watchdog] $(date '+%F %T') 启动,程序: $VISION_BIN,配置: $VISION_CONFIG" | tee -a logs/watchdog.log

fail_streak=0  # 连续快速崩溃计数,用于退避(设备未就绪时不刷屏)

while true; do
  if [ ! -x "$VISION_BIN" ]; then
    echo "[watchdog] $(date '+%F %T') 找不到可执行文件 $VISION_BIN,10s 后重试(请先编译)" | tee -a logs/watchdog.log
    sleep 10
    continue
  fi

  start=$(date +%s)
  echo "[watchdog] $(date '+%F %T') 启动视觉程序 ..." | tee -a logs/watchdog.log
  "$VISION_BIN" "$VISION_CONFIG"
  code=$?
  run_secs=$(( $(date +%s) - start ))

  if [ "$run_secs" -gt 60 ]; then
    fail_streak=0  # 正常运行超过 1 分钟,视为健康,清零退避
  else
    fail_streak=$((fail_streak + 1))
  fi

  # 退避间隔:RESTART_DELAY * 2^n,上限 60s(连续快速崩溃时避免刷日志/占 CPU)
  pow=1
  for ((i = 1; i < fail_streak && i < 6; i++)); do pow=$((pow * 2)); done
  delay=$((RESTART_DELAY * pow))
  [ "$delay" -gt 60 ] && delay=60

  echo "[watchdog] $(date '+%F %T') 程序退出(运行 ${run_secs}s,退出码 $code,连续失败 $fail_streak 次),${delay}s 后重启" | tee -a logs/watchdog.log
  sleep "$delay"
done
