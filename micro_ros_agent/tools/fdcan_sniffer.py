#!/usr/bin/env python3
"""FDCAN1 真机台架只读嗅探：配置 CANable2（C/S8/Y5/O）后打印总线帧。

与 fdcan_motor_sim.py 的本质区别：本工具**只收不发**——除 SLCAN 通道配置
与收尾 `C` 关通道外，不向总线写任何 CAN 帧，真机台架安全。复用模拟器的
SLCAN 配置序列与 RX 解析（同一 CANable2 方言）。

用法：
  python3 fdcan_sniffer.py                       # 持续打印，Ctrl-C 结束
  python3 fdcan_sniffer.py --duration 30         # 跑 30 秒自动结束并出统计
  python3 fdcan_sniffer.py --log /tmp/sniff.jsonl

Ctrl-C / 时限到：发 `C` 关 CANable 通道，输出按 CAN ID 的收帧统计。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import termios
import time
import tty

from fdcan_motor_sim import parse_slcan, slcan_config_commands


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tty", default="/dev/ttyACM1", help="CANable2 串口")
    parser.add_argument("--data-bitrate-index", type=int, default=5,
                        choices=range(10), help="SLCAN Y 索引（上游 b158aa7 的 5Mbps=5）")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="嗅探秒数；0 = 直到 Ctrl-C")
    parser.add_argument("--log", default="", help="可选 JSONL 帧日志路径")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    fd = os.open(args.tty, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    tty.setraw(fd, when=termios.TCSANOW)
    attrs = termios.tcgetattr(fd)
    attrs[2] |= termios.CLOCAL | termios.CREAD
    termios.tcsetattr(fd, termios.TCSANOW, attrs)

    # CANable 必须先成为 ACK 节点，控制板才可复位（台架指南铁律）。
    print(f"配置 {args.tty}：C/S8/Y{args.data_bitrate_index}/O（150ms 间隔）…")
    for cmd in slcan_config_commands(args.data_bitrate_index):
        os.write(fd, cmd)
        time.sleep(0.15)
    termios.tcflush(fd, termios.TCIOFLUSH)
    print("通道已开，开始嗅探（只收不发）…")

    log_f = open(args.log, "a") if args.log else None
    rx_buf = b""
    counts: dict[int, int] = {}
    last_report = time.monotonic()
    deadline = time.monotonic() + args.duration if args.duration > 0 else None
    try:
        while True:
            try:
                rx_buf += os.read(fd, 4096)
            except BlockingIOError:
                pass
            while b"\r" in rx_buf:
                line, rx_buf = rx_buf.split(b"\r", 1)
                text = line.decode(errors="replace")
                parsed = parse_slcan(text)
                if parsed is None:
                    continue
                can_id, data, is_fd = parsed
                counts[can_id] = counts.get(can_id, 0) + 1
                stamp = time.strftime("%H:%M:%S")
                flag = "FD" if is_fd else "  "
                print(f"{stamp} {flag} 0x{can_id:03X} [{len(data):2d}] "
                      f"{data.hex().upper()}")
                if log_f:
                    log_f.write(json.dumps({
                        "t": round(time.monotonic(), 4), "id": can_id,
                        "fd": is_fd, "data": data.hex()}) + "\n")
            now = time.monotonic()
            if now - last_report >= 1.0:
                last_report = now
                summary = " ".join(f"0x{k:03X}:{v}" for k, v in sorted(counts.items()))
                print(f"-- {summary or '（无帧）'}")
            if deadline is not None and now >= deadline:
                break
            time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        os.write(fd, b"C\r")  # 关闭 CANable 通道
        os.close(fd)
        if log_f:
            log_f.close()
    print("\n按 CAN ID 统计：")
    for can_id, n in sorted(counts.items()):
        print(f"  0x{can_id:03X}: {n}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
