#!/usr/bin/env python3
"""USB CDC (SP 协议) 串口联调工具 —— 纯标准库实现，不依赖 pyserial。

收方向: 解析电控发来的 GimbalToVision (43 字节)，校验 CRC16 并打印各字段。
发方向: --send-idle 时以 10Hz 发送 VisionToGimbal (29 字节) mode=0 (不控制)，
        用于验证"视觉能发、但云台不动"这条最基本的通路。

帧格式与 CRC 算法严格对齐 io/gimbal/gimbal.hpp 与 tools/crc.cpp:
  GimbalToVision (43B): head(2) mode(1) q[4](16) yaw(4) yaw_vel(4)
                        pitch(4) pitch_vel(4) bullet_speed(4)
                        bullet_count(2) crc16(2)
  VisionToGimbal (29B): head(2) mode(1) yaw(4) yaw_vel(4) yaw_acc(4)
                        pitch(4) pitch_vel(4) pitch_acc(4) crc16(2)
  CRC16: poly=0x1021 init=0xFFFF，小端存放，覆盖除末 2 字节外的全部内容。
"""

import argparse
import os
import struct
import sys
import termios
import time

GIMBAL_TO_VISION_LEN = 43
VISION_TO_GIMBAL_LEN = 29


def crc16_ccitt_false(data: bytes) -> int:
    """对齐 tools/crc.cpp 的 get_crc16 (poly=0x1021, init=0xFFFF)。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def check_crc16(frame: bytes) -> bool:
    """对齐 tools/crc.cpp 的 check_crc16：CRC 在末 2 字节，小端。"""
    expect = frame[-1] << 8 | frame[-2]
    return crc16_ccitt_false(frame[:-2]) == expect


def open_port(path: str, baud: int):
    """以 raw 模式打开串口。CDC ACM 下波特率对线路无意义，但需设 raw 避免行规程改写。"""
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    iflag, oflag, cflag, lflag, ispeed, ospeed, cc = attrs
    iflag &= ~(termios.IXON | termios.IXOFF | termios.IXANY
               | termios.INLCR | termios.ICRNL | termios.IGNCR)
    oflag &= ~termios.OPOST
    lflag &= ~(termios.ICANON | termios.ECHO | termios.ECHOE | termios.ISIG)
    cflag |= termios.CLOCAL | termios.CREAD
    cflag &= ~termios.CSIZE
    cflag |= termios.CS8
    cflag &= ~(termios.PARENB | termios.CSTOPB | termios.CRTSCTS)
    baud_const = getattr(termios, f"B{baud}")
    cc[termios.VMIN] = 0
    cc[termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, [iflag, oflag, cflag, lflag,
                                            baud_const, baud_const, cc])
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def build_idle_frame() -> bytes:
    """VisionToGimbal, mode=0 (不控制)，其余全 0 —— 最安全的发送测试。"""
    frame = bytearray(VISION_TO_GIMBAL_LEN)
    frame[0:2] = b"SP"
    frame[2] = 0  # mode 0 = 不控制
    struct.pack_into("<6f", frame, 3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    crc = crc16_ccitt_false(bytes(frame[:-2]))
    struct.pack_into("<H", frame, VISION_TO_GIMBAL_LEN - 2, crc)
    return bytes(frame)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", nargs="?", default="/dev/ttyACM0",
                    help="串口设备，默认 /dev/ttyACM0（/dev/gimbal 建好软链后可直接用它）")
    ap.add_argument("--baud", type=int, default=115200, help="CDC 下无实义，默认 115200")
    ap.add_argument("--send-idle", action="store_true",
                    help="以 10Hz 发送 mode=0 帧（云台不受控，仅验证视觉→电控通路）")
    ap.add_argument("--seconds", type=float, default=0,
                    help="运行时长（秒），0 表示一直运行")
    ap.add_argument("--raw", action="store_true", help="额外打印原始十六进制")
    ap.add_argument("--no-crc", action="store_true",
                    help="不校验 CRC16，改用单位四元数(|q|^2≈1)做合理性校验。"
                         "实测当前 C 板固件的 crc16 字段匹配不上任何已知算法（疑似未赋值），"
                         "联调阶段用这个开关绕过。")
    args = ap.parse_args()

    try:
        fd = open_port(args.port, args.baud)
    except PermissionError:
        print(f"[x] 无权打开 {args.port}，检查是否在 dialout 组: sudo usermod -a -G dialout $USER",
              file=sys.stderr)
        return 1
    except FileNotFoundError:
        print(f"[x] {args.port} 不存在。先确认下位机已上电枚举: ls -l /dev/ttyACM*",
              file=sys.stderr)
        return 1

    print(f"[*] 已打开 {args.port}，开始监听 {GIMBAL_TO_VISION_LEN} 字节 GimbalToVision 帧 "
          f"(Ctrl-C 退出)")
    if args.send_idle:
        print("[*] --send-idle: 以 10Hz 发送 mode=0 帧（不控制云台）")
    print(f"{'时间':>7} {'mode':>5} {'yaw':>8} {'yaw_vel':>9} {'pitch':>8} "
          f"{'pitch_vel':>9} {'弹速':>6} {'弹量':>6}  q(wxyz)")

    buf = bytearray()
    total, ok, bad_crc, bad_head = 0, 0, 0, 0
    t0 = time.time()
    last_stat = t0
    last_send = 0.0
    idle_frame = build_idle_frame() if args.send_idle else b""
    last_logged = None

    while True:
        now = time.time()
        if args.seconds and now - t0 >= args.seconds:
            break

        if args.send_idle and now - last_send >= 0.1:
            try:
                os.write(fd, idle_frame)
            except OSError as e:
                print(f"[!] 写失败: {e}", file=sys.stderr)
            last_send = now

        try:
            chunk = os.read(fd, 4096)
        except BlockingIOError:
            chunk = b""
        except OSError as e:
            print(f"[x] 读失败: {e}", file=sys.stderr)
            break

        if chunk:
            buf.extend(chunk)

        # 逐帧切分：找 'SP' 头，取满 43 字节后校验 CRC
        while True:
            idx = buf.find(b"SP")
            if idx < 0:
                bad_head += len(buf)
                buf.clear()
                break
            if idx > 0:
                bad_head += idx
                del buf[:idx]
            if len(buf) < GIMBAL_TO_VISION_LEN:
                break

            frame = bytes(buf[:GIMBAL_TO_VISION_LEN])
            total += 1

            if args.no_crc:
                # 退而求其次：用单位四元数约束做合理性校验
                qv = struct.unpack_from("<4f", frame, 3)
                norm2 = sum(v * v for v in qv)
                if abs(norm2 - 1.0) > 1e-2:
                    bad_crc += 1
                    del buf[:1]  # 只前移 1 字节，避免误吞真正的下一帧
                    continue
            elif not check_crc16(frame):
                bad_crc += 1
                del buf[:1]
                continue

            ok += 1
            mode = frame[2]
            q = struct.unpack_from("<4f", frame, 3)
            yaw, yaw_vel, pitch, pitch_vel, bullet_speed = struct.unpack_from("<5f", frame, 19)
            bullet_count = struct.unpack_from("<H", frame, 39)[0]

            # 1Hz 打印，避免刷屏
            if last_logged is None or now - last_logged >= 1.0:
                print(f"{now - t0:7.2f} {mode:5d} {yaw:8.3f} {yaw_vel:9.3f} {pitch:8.3f} "
                      f"{pitch_vel:9.3f} {bullet_speed:6.2f} {bullet_count:6d}  "
                      f"({q[0]:+.3f},{q[1]:+.3f},{q[2]:+.3f},{q[3]:+.3f})")
                last_logged = now

            if args.raw:
                print("    " + frame.hex(" "))

            del buf[:GIMBAL_TO_VISION_LEN]

        if chunk and now - last_stat >= 2.0:
            print(f"[stat] 累计帧 {total}  CRC通过 {ok}  CRC失败 {bad_crc}  "
                  f"丢弃字节 {bad_head}")
            last_stat = now

    os.close(fd)
    check_name = "四元数校验" if args.no_crc else "CRC16"
    print(f"\n[*] 结束: 共收到 {total} 帧，{check_name}通过 {ok}，失败 {bad_crc}，丢弃字节 {bad_head}")
    if total == 0:
        print("[!] 一帧都没收到 —— 检查下位机是否在往这个串口发数据、是否选对了端口")
        return 2
    if ok == 0:
        if args.no_crc:
            print("[!] 四元数校验全失败 —— 帧偏移或 float 字节序可能不对")
        else:
            print("[!] 收到数据但 CRC 全失败 —— 可加 --no-crc 用四元数校验绕过，"
                  "同时让电控确认固件是否真的计算了 crc16")
        return 3
    print(f"[+] {check_name}全部通过，SP 协议解析正常")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n[*] 已中断")
