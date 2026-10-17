#!/usr/bin/env bash
# 为下位机 USB CDC 虚拟串口建立固定的 /dev/gimbal 符号链接。
#
# 用法: sudo bash scripts/setup_gimbal_udev.sh [串口设备] [符号链接名]
#   sudo bash scripts/setup_gimbal_udev.sh                    # 自动找 /dev/ttyACM*，链接名 gimbal
#   sudo bash scripts/setup_gimbal_udev.sh /dev/ttyACM0 gimbal
#
# 幂等: 重复执行只会覆盖同一份规则文件。改动后拔插一次 USB 最保险。

set -euo pipefail

DEV="${1:-}"
LINKNAME="${2:-gimbal}"
RULES=/etc/udev/rules.d/99-usb-serial.rules

if [[ $EUID -ne 0 ]]; then
  echo "[x] 需要 root 权限: sudo bash $0" >&2
  exit 1
fi

# ---- 找设备 ----
if [[ -z "$DEV" ]]; then
  shopt -s nullglob
  candidates=(/dev/ttyACM*)
  shopt -u nullglob
  if [[ ${#candidates[@]} -eq 0 ]]; then
    echo "[x] 没找到 /dev/ttyACM* —— 确认下位机已上电并插好 USB" >&2
    exit 1
  elif [[ ${#candidates[@]} -gt 1 ]]; then
    echo "[!] 发现多个候选设备，请显式指定:" >&2
    printf '     %s\n' "${candidates[@]}" >&2
    echo "     sudo bash $0 ${candidates[0]} $LINKNAME" >&2
    exit 1
  fi
  DEV="${candidates[0]}"
fi

if [[ ! -e "$DEV" ]]; then
  echo "[x] 设备不存在: $DEV" >&2
  exit 1
fi

# ---- 取 USB 属性 ----
info="$(udevadm info -a -n "$DEV")"
vid="$(awk -F'"' '/ATTRS\{idVendor\}/{print $2; exit}' <<<"$info")"
pid="$(awk -F'"' '/ATTRS\{idProduct\}/{print $2; exit}' <<<"$info")"
serial="$(awk -F'"' '/ATTRS\{serial\}/{print $2; exit}' <<<"$info")"

if [[ -z "$vid" || -z "$pid" ]]; then
  echo "[x] 无法从 $DEV 读取 idVendor/idProduct" >&2
  exit 1
fi

echo "[*] 设备: $DEV"
echo "    idVendor=$vid  idProduct=$pid  serial=${serial:-<无>}"

# serial 缺失时不要写空属性，否则 ATTRS{serial}=="" 永远匹配不上
if [[ -n "$serial" ]]; then
  match="ATTRS{idVendor}==\"$vid\", ATTRS{idProduct}==\"$pid\", ATTRS{serial}==\"$serial\""
else
  match="ATTRS{idVendor}==\"$vid\", ATTRS{idProduct}==\"$pid\""
  echo "[!] 该设备无 serial，规则仅靠 VID/PID 匹配 —— 插第二块同型号板子会冲突"
fi

# ---- 写入规则 ----
RULE="SUBSYSTEM==\"tty\", $match, SYMLINK+=\"$LINKNAME\", GROUP=\"dialout\", MODE=\"0660\""

# 防护: 规则为空或不含关键字段时绝不落盘。
# 否则一旦上游变量取空，就会把已有规则文件静默截断成 0 字节。
if [[ -z "$RULE" || "$RULE" != *'SUBSYSTEM=="tty"'* || "$RULE" != *"SYMLINK+=\"$LINKNAME\""* ]]; then
  echo "[x] 生成的规则不合法，拒绝写入以免清空 $RULES:" >&2
  echo "    [$RULE]" >&2
  exit 1
fi

if [[ -f "$RULES" ]] && grep -qF "SYMLINK+=\"$LINKNAME\"" "$RULES"; then
  echo "[*] $RULES 中已有 $LINKNAME 规则，将覆盖"
fi

# 先写临时文件再原子替换，避免出现"文件存在但内容为空"的窗口
tmp="${RULES}.tmp.$$"
printf '%s\n' "$RULE" > "$tmp"
chmod 644 "$tmp"
mv -f "$tmp" "$RULES"

echo "[*] 已写入 $RULES:"
echo "    $RULE"

# ---- 重载并触发 ----
udevadm control --reload-rules
udevadm trigger --subsystem-match=tty
# 给 udevd 一点处理 add 事件的时间
sleep 1

# ---- 验证 ----
echo
if [[ -L "/dev/$LINKNAME" ]]; then
  ls -l "/dev/$LINKNAME"
  target="$(readlink -f "/dev/$LINKNAME")"
  if [[ "$target" == "$DEV" ]]; then
    echo "[+] 完成: /dev/$LINKNAME -> $target"
  else
    echo "[!] /dev/$LINKNAME 指向 $target，但预期是 $DEV"
    echo "    可能有多块同型号设备，或需要拔插一次 USB"
  fi
else
  echo "[!] /dev/$LINKNAME 尚未创建。"
  echo "    对已经枚举完成的设备，trigger 有时不会重建节点 —— 拔插一次 USB 即可:"
  echo "      ls -l /dev/$LINKNAME"
fi
