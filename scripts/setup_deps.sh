#!/usr/bin/env bash
# 安装 sp_vision_25 所需系统依赖（需 root 权限运行）
#
# 用法:
#   sudo bash scripts/setup_deps.sh [openvino_toolkit_ubuntu22_*.tgz 的路径]
#
# 不带参数时跳过 OpenVINO 2025.4.0 安装（继续使用已有的 /opt/intel/openvino_2024.6.0）。
# 提供 tgz 路径时会安装到 /opt/intel/openvino_2025.4.0（与 CMake 优先路径一致）。
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
  echo "请用 sudo 运行: sudo bash $0 [tgz路径]" >&2
  exit 1
fi

TARGET_USER="${SUDO_USER:-$USER}"

# 1. apt 依赖:编译工具链 + 程序链接的库(相机库 MvCameraControl/MVSDK、串口库 serial 已随仓库自带)
#    ccache: 编译缓存;screen: 开机自启链路(autostart.sh + watchdog.sh)依赖
echo "[*] 安装编译工具链与依赖库(首次运行需联网,约几百 MB) ..."
apt-get update
apt-get install -y \
  build-essential cmake ccache screen \
  libopencv-dev libeigen3-dev libspdlog-dev libyaml-cpp-dev libceres-dev libusb-1.0-0-dev

# 2. 串口权限（dialout 组，注销重新登录后生效）
if id -nG "$TARGET_USER" | grep -qw dialout; then
  echo "[ok] $TARGET_USER 已在 dialout 组"
else
  usermod -aG dialout "$TARGET_USER"
  echo "[ok] 已将 $TARGET_USER 加入 dialout 组（重新登录后生效）"
fi

# 3. OpenVINO 2025.4.0（可选：给出 tgz 路径才安装）
if [ $# -ge 1 ] && [ -n "$1" ]; then
  TGZ="$(readlink -f "$1")"
  [ -f "$TGZ" ] || { echo "找不到文件: $TGZ" >&2; exit 1; }

  if [ -e /opt/intel/openvino_2025.4.0 ]; then
    echo "[!] /opt/intel/openvino_2025.4.0 已存在，跳过安装"
  else
    TMPD="$(mktemp -d)"
    trap 'rm -rf "$TMPD"' EXIT
    echo "[*] 解压 OpenVINO 到临时目录 ..."
    tar -xzf "$TGZ" -C "$TMPD"
    TOP="$(ls -d "$TMPD"/*/ | head -1)"
    [ -n "$TOP" ] || { echo "tgz 内容为空" >&2; exit 1; }

    mkdir -p /opt/intel
    mv "$TOP" /opt/intel/openvino_2025.4.0
    echo "[ok] 已安装到 /opt/intel/openvino_2025.4.0"
  fi

  DEPS_SH=/opt/intel/openvino_2025.4.0/install_dependencies/install_openvino_dependencies.sh
  if [ -f "$DEPS_SH" ]; then
    echo "[*] 安装 OpenVINO 运行时依赖（libtbb 等）..."
    DEBIAN_FRONTEND=noninteractive bash "$DEPS_SH" -y
    echo "[ok] OpenVINO 依赖安装完成"
  else
    echo "[!] 未找到 $DEPS_SH，请手动安装 libtbb 等运行时依赖"
  fi
else
  echo "[i] 未提供 tgz 路径，跳过 OpenVINO 2025.4.0（继续使用 /opt/intel/openvino_2024.6.0）"
fi

echo ""
echo "完成。后续提示："
echo " - dialout 组权限需注销重新登录后生效"
echo " - 电控串口软链: sudo bash scripts/setup_gimbal_udev.sh /dev/ttyACM0 gimbal"
echo " - 运行目录需要 logs/: mkdir -p logs"
