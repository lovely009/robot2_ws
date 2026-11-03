#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CERB SDO 探针（真机台架专用）：**默认只读**（只发「加速上传」请求）；
带 `--write IDX:SUB=VAL` 时才发快速写（0x23），那会**改设备对象**。

用途：核对某个对象/子索引在真机上是否可读。首要用例是 TBD-CONFIG-02——
`0x2650:01` 的读子索引（TBD-CONFIG-02）——**已结案**：读子索引是 `0x00`，
`:01`/`:02` 一律 ABORT `0x06090011`（子索引不存在）。对象语义：读
`40 50 26 00 00 …` → 反馈 `43 50 26 00 0x …`；`23 50 26 00 **00** …` = 开启、
`23 50 26 00 **01** …` = 关闭；**掉电不保存**（实测）。
厂商协议手册位于根 `docs/巨蟹电机/`，资料入口见 `docs/README.md`。

默认只发一种字节形态（8 字节，恰好一个 SDO 上传发起）：

    40 <idx_lo> <idx_hi> <sub> 00 00 00 00

`--write` 时才会发快速写（0x23 + 4 字节小端值）并**读回核验**。两种形态都不含
使能帧、不含 MIT 帧，电不会动——但写会改设备里的对象，只在明确知道后果时用：
`0x2650:00` = 看门狗/限位保护开关（0 = 开、1 = 关），`0x2541:00` = 方向
（0 = CW、1 = CCW）。两者都**掉电不保存**，要复原就再写回去或断一次电。

台架前提：
  - CANable2 先成为 ACK 节点（本脚本自动 C→S8→Y5→M0→A1→O，间隔 ≥150ms）；
  - **同总线不得有第二个 SDO 主站**：控制板的配置事务用同一 COB-ID
    （0x600+id），并发发问会撞帧。不要并发触发 `system.refresh`；本脚本
    开场先静默监听一小段，看到别人的 SDO 请求就拒绝开机（--force 可越过）。
  - **控制板不是前置条件**：2026-10-15 用 gs_usb/SocketCAN 在**无控制板**的总线上
    读 node 1 成功（0x1008 回 R58LITE_V2）。2026-09-19 记录的"断开固件后 CERB
    对外部 SDO 主站一概不应答"作废——当时用的是数据段采样点配错的 SLCAN
    CANable，收不到设备的 FD+BRS 应答帧，看着像不应答。
  - **设备只认标准（11 位）帧**：用扩展帧发同 ID 的请求照样被 ACK（错误计数不涨），
    但设备按 ID 过滤后静默忽略，现象是"发得出去、一条不回"。SocketCAN 路径已按
    标准帧发送（见 cerb_sdo_probe.py 的 pack_socketcan_fd）。

用法：
  # gs_usb / SocketCAN（现场当前配置）
  python3 tools/cerb_sdo_probe.py --interface can0 --node 1

  # SLCAN 串口（旧适配器）
  python3 tools/cerb_sdo_probe.py --dev /dev/serial/by-id/<CANable> --node 2
  python3 tools/cerb_sdo_probe.py --dev /dev/ttyACM1 --node 2 \
      --read 0x2650:00,0x2650:01,0x2650:02,0x2541:00

  # 写方向为 CCW 并读回核验（会改设备对象）：
  python3 tools/cerb_sdo_probe.py --dev /dev/ttyACM1 --node 2 \
      --read 0x2541:00 --write 0x2541:00=1 --read 0x2541:00
"""

from __future__ import annotations

import argparse
import os
import select
import socket
import struct
import sys
import termios
import time
import tty

from fdcan_motor_sim import parse_slcan, slcan_config_commands, slcan_line

# SocketCAN 常量：部分 Python 版本不导出 SOL_CAN_RAW / CAN_RAW_FD_FRAMES，兜住。
SOL_CAN_RAW = getattr(socket, "SOL_CAN_RAW", 101)
CAN_RAW_FD_FRAMES = getattr(socket, "CAN_RAW_FD_FRAMES", 5)
CANFD_MTU = 72          # struct canfd_frame：8B 头 + 64B 数据
CANFD_BRS = 0x01        # flags 位：数据段切换位率

SDO_REQUEST_ID_BASE = 0x600
SDO_RESPONSE_ID_BASE = 0x580

# 内置对象清单：--default 时按序问这几个。`六对象`指固件 REFRESH 的回读集合。
DEFAULT_OBJECTS = (
    (0x1008, 0x00, "型号身份串（分段上传）"),
    (0x607D, 0x01, "负载位置下限 cnt"),
    (0x607D, 0x02, "负载位置上限 cnt"),
    (0x6080, 0x00, "电机最大转速 RPM"),
    (0x2538, 0x00, "电机峰值电流 mA"),
    (0x2650, 0x00, "看门狗/限位保护开关（手册写命令的子索引）"),
    (0x2650, 0x01, "看门狗/限位保护开关（固件当前读的子索引）"),
    (0x2650, 0x02, "看门狗/限位保护开关（存在性核对）"),
    (0x2541, 0x00, "电机方向定义 0=CW 1=CCW"),
)

# CANopen 标准中止码（CiA 301 表）。认得几个常见的就够定位，其余按十六进制打。
ABORT_CODES = {
    0x05030000: "toggle 位未变化",
    0x05040000: "SDO 协议超时",
    0x05040001: "客户端命令字非法",
    0x06010000: "对象不支持访问",
    0x06010001: "只读对象",
    0x06010002: "只写对象",
    0x06020000: "对象字典里没有这个对象",
    0x06090011: "**子索引不存在**",
    0x06090030: "参数值超范围",
    0x06090031: "参数值过大",
    0x06090032: "参数值过小",
    0x08000000: "通用错误",
}


def parse_write(text: str) -> tuple[int, int, int]:
    """`0x2541:00=1` → (0x2541, 0x00, 1)。值可写十进制或 0x 十六进制。"""
    if "=" not in text or ":" not in text:
        raise argparse.ArgumentTypeError(f"写入写法应为 index:sub=value，得到 {text!r}")
    obj, val = text.split("=", 1)
    idx, sub = parse_obj(obj)
    try:
        return idx, sub, int(val, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"值不是整数：{val!r}") from exc


def parse_obj(text: str) -> tuple[int, int]:
    """`0x2650:01` / `2650:1` → (0x2650, 0x01)。"""
    if ":" not in text:
        raise argparse.ArgumentTypeError(f"对象写法应为 index:sub，得到 {text!r}")
    idx_s, sub_s = text.split(":", 1)
    # 一律按十六进制解析：子索引常写成 `01`，而 int(x, 0) 不接受前导零。
    return int(idx_s, 16), int(sub_s, 16)


def pack_socketcan_fd(can_id: int, data: bytes, brs: bool = True) -> bytes:
    """打包一帧 CAN FD 标准帧（struct canfd_frame，72 字节定长）。"""
    if len(data) > 64:
        raise ValueError(f"CAN FD 负载超长：{len(data)}")
    flags = CANFD_BRS if brs else 0
    return struct.pack("=IBBBB", can_id, len(data), flags, 0, 0) \
        + data.ljust(64, b"\x00")


def parse_socketcan_fd(frame: bytes):
    """解析 struct canfd_frame → (can_id, data)；扩展帧返回 None。"""
    if len(frame) < 8:
        return None
    can_id, length, _flags, _r0, _r1 = struct.unpack("=IBBBB", frame[:8])
    if can_id & 0x80000000:      # CAN_EFF_FLAG：本工具只认标准帧（设备也只认标准帧）
        return None
    return can_id, frame[8:8 + min(length, 64)]


class SlcanSdoProbe:
    """SLCAN 串口传输（上游 CANable2 等）。"""

    def __init__(self, dev: str, node: int, data_bitrate_index: int):
        self.dev = dev
        self.node = node
        self.data_bitrate_index = data_bitrate_index
        self.fd = -1
        self.rx_buf = b""

    def open(self):
        self.fd = os.open(self.dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        tty.setraw(self.fd, when=termios.TCSANOW)
        attrs = termios.tcgetattr(self.fd)
        attrs[2] |= termios.CLOCAL | termios.CREAD
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        for cmd in slcan_config_commands(self.data_bitrate_index):
            os.write(self.fd, cmd)
            time.sleep(0.15)
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def close(self):
        if self.fd >= 0:
            try:
                os.write(self.fd, b"C\r")
            except OSError:
                pass
            os.close(self.fd)
            self.fd = -1

    def drain(self, timeout: float) -> list[tuple[int, bytes]]:
        frames: list[tuple[int, bytes]] = []
        deadline = time.monotonic() + timeout
        while True:
            left = deadline - time.monotonic()
            if left <= 0:
                return frames
            r, _, _ = select.select([self.fd], [], [], left)
            if not r:
                return frames
            try:
                chunk = os.read(self.fd, 4096)
            except OSError:
                continue
            self.rx_buf += chunk
            while b"\r" in self.rx_buf:
                line, self.rx_buf = self.rx_buf.split(b"\r", 1)
                parsed = parse_slcan(line.decode(errors="replace"))
                if parsed:
                    frames.append((parsed[0], parsed[1]))

    def foreign_master_seen(self, listen_s: float) -> bool:
        """静默监听：同总线是否已有别人（控制板）在发 SDO 请求。"""
        return any(can_id == SDO_REQUEST_ID_BASE + self.node
                   for can_id, _ in self.drain(listen_s))

    def read_once(self, index: int, sub: int, timeout: float = 0.3):
        """发一条加速上传请求，返回 (状态, 说明, 原始值)。

        状态 ∈ {"ok", "segmented", "abort", "timeout"}。
        """
        want = SDO_RESPONSE_ID_BASE + self.node
        req = bytes([0x40, index & 0xFF, (index >> 8) & 0xFF, sub, 0, 0, 0, 0])
        deadline = time.monotonic() + timeout
        sent = False
        while True:
            if not sent:
                self.send_request(req)
                sent = True
            for can_id, data in self.drain(max(0.0, deadline - time.monotonic())):
                if can_id != want or len(data) < 8:
                    continue
                ccs = data[0] & 0xE0
                got_idx = (data[1] | (data[2] << 8))
                if ccs == 0x80:
                    # 中止帧也带对象标识：不匹配就是别的对象的过期应答
                    # （总线上还跑着 50Hz 反馈与板子的配置事务），继续等本次的。
                    if got_idx != index or data[3] != sub:
                        continue
                    return "abort", int.from_bytes(data[4:8], "little"), None
                if ccs != 0x40:
                    continue  # 迟到的写应答之类：继续等本阶段
                if got_idx != index or data[3] != sub:
                    continue
                n = 4 - ((data[0] >> 2) & 0x03)  # 0x43→4B，0x4B→2B，0x4F→1B
                if (data[0] & 0x02) == 0:  # 非加速：分段上传
                    total = int.from_bytes(data[4:8], "little")
                    return "segmented", total, None
                raw = int.from_bytes(data[4:4 + n], "little", signed=True)
                return "ok", n, raw
            if time.monotonic() >= deadline:
                return "timeout", None, None

    def write_once(self, index: int, sub: int, value: int,
                   timeout: float = 0.3):
        """发一条快速 SDO 写（0x23 + 4 字节小端）并读回核验。

        返回 (状态, 说明)。状态 ∈ {"ok", "abort", "mismatch", "timeout"}。
        **会改设备对象**：只对明确知道后果的对象用（0x2650:00 保护 / 0x2541:00 方向）。
        """
        want = SDO_RESPONSE_ID_BASE + self.node
        payload = int(value).to_bytes(4, "little", signed=True)
        req = bytes([0x23, index & 0xFF, (index >> 8) & 0xFF, sub]) + payload
        self.send_request(req)
        for can_id, data in self.drain(timeout):
            if can_id != want or len(data) < 8:
                continue
            ccs = data[0] & 0xE0
            got_idx = data[1] | (data[2] << 8)
            if got_idx != index or data[3] != sub:
                continue
            if ccs == 0x80:
                return "abort", int.from_bytes(data[4:8], "little")
            if ccs == 0x60:
                status, _, raw, _ = self.read(index, sub)
                if status != "ok":
                    return status, raw
                return ("ok" if raw == value else "mismatch"), raw
        return "timeout", None

    def read(self, index: int, sub: int, attempts: int = 3):
        """带重试的读。**只对超时重试**：CANable2 在 200 帧/秒反馈下会丢帧，
        丢一帧这条应答就没了（设备不会重发，SDO 是一问一答）；而设备一旦应答
        就是 OK 或 ABORT，两者都是定论，直接返回。重试次数一并报出——
        "第 3 次才收到"与"一次就答"是不同强度的证据。
        """
        for attempt in range(1, attempts + 1):
            status, detail, raw = self.read_once(index, sub)
            if status == "segmented":
                body = self.upload_segments(detail)
                if body is None:
                    continue          # 取段丢帧：重试整条事务
                return "upload", detail, body, attempt
            if status != "timeout":
                return status, detail, raw, attempt
        return "timeout", None, None, attempts

    def upload_segments(self, total: int, timeout: float = 0.3):
        """分段上传取段并拼接（命令字 bit4 = 切换位，0x60/0x70 交替）。

        段应答 ccs 位为 0，末段带 n（未用字节数）与 c=1。丢一段就返回 None——调用方
        重试整条事务：SDO 是一问一答，中间断一次就无法续上。
        """
        want = SDO_RESPONSE_ID_BASE + self.node
        body, toggle = b"", 0
        while len(body) < total:
            self.send_request(bytes([0x60 | (toggle << 4), 0, 0, 0, 0, 0, 0, 0]))
            deadline = time.monotonic() + timeout
            got = None
            while got is None:
                left = deadline - time.monotonic()
                if left <= 0:
                    return None
                for can_id, data in self.drain(left):
                    if can_id == want and len(data) >= 8 and (data[0] & 0xE0) == 0:
                        got = data
                        break
            if got[0] & 0x80:            # 段应答里不该出现的位：当作丢帧重来
                return None
            body += got[1:8]
            toggle ^= 1
            if got[0] & 0x01:
                break
        return body[:total]

    def send_request(self, req: bytes) -> None:
        os.write(self.fd, slcan_line(self.node + SDO_REQUEST_ID_BASE, req,
                                     True, True))


class SocketCanSdoProbe(SlcanSdoProbe):
    """SocketCAN 传输（gs_usb 等）。现场适配器换成 gs_usb 后，SLCAN 路径用不上。"""

    def __init__(self, interface: str, node: int):
        self.interface = interface
        self.node = node
        self.sock = None
        self.rx_buf = b""

    def open(self):
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW,
                                  socket.CAN_RAW)
        self.sock.setsockopt(SOL_CAN_RAW, CAN_RAW_FD_FRAMES, 1)
        self.sock.bind((self.interface,))
        self.sock.setblocking(False)

    def close(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def send_request(self, req: bytes) -> None:
        # 设备只认**标准帧**：同 ID 的扩展帧照样被 ACK，但按 ID 过滤后静默忽略。
        self.sock.send(pack_socketcan_fd(self.node + SDO_REQUEST_ID_BASE, req))

    def drain(self, timeout: float) -> list[tuple[int, bytes]]:
        frames: list[tuple[int, bytes]] = []
        deadline = time.monotonic() + timeout
        while True:
            left = deadline - time.monotonic()
            if left <= 0:
                return frames
            readable, _, _ = select.select([self.sock], [], [], left)
            if not readable:
                return frames
            try:
                raw = self.sock.recv(CANFD_MTU)
            except (BlockingIOError, OSError):
                continue
            parsed = parse_socketcan_fd(raw)
            if parsed:
                frames.append(parsed)


def format_result(index: int, sub: int, note: str, status, detail, raw,
                  tries: int = 1) -> str:
    head = f"  0x{index:04X}:{sub:02X}  "
    if status == "ok":
        body = f"OK   {detail} 字节  raw={raw} (0x{raw & 0xFFFFFFFF:08X})"
    elif status == "upload":
        text = raw.decode("ascii", "replace")
        body = (f"分段上传 {detail} 字节 = {text!r}  hex={' '.join(f'{b:02X}' for b in raw)}")
    elif status == "abort":
        name = ABORT_CODES.get(detail, "（未收录的中止码）")
        body = f"**ABORT 0x{detail:08X}  {name}**"
    else:
        body = f"超时（{tries} 次重试均无应答）"
    return f"{head}{body}    # {note}"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    target = ap.add_mutually_exclusive_group(required=True)
    target.add_argument("--dev", help="SLCAN 串口（旧适配器）")
    target.add_argument("--interface", help="SocketCAN 接口（gs_usb，如 can0）")
    ap.add_argument("--node", type=int, required=True, help="CERB 节点 id")
    ap.add_argument("--read", type=parse_obj, action="append", default=None,
                    help="index:sub，可重复；缺省或 --default 时用内置对象清单")
    ap.add_argument("--default", action="store_true", help="用内置对象清单")
    ap.add_argument("--write", type=parse_write, action="append", default=None,
                    metavar="IDX:SUB=VAL",
                    help="发一条快速 SDO 写（0x23）并读回核验。**会改设备对象**，"
                         "只给 0x2650:00（保护开关）/ 0x2541:00（方向）这类"
                         "明确知道后果的对象用；两者都掉电不保存")
    ap.add_argument("--listen-s", type=float, default=0.5,
                    help="开场静默监听时长（查第二个 SDO 主站）")
    ap.add_argument("--force", action="store_true", help="发现别的 SDO 主站也照发")
    ap.add_argument("--repeat", type=int, default=1,
                    help="每条对象重复读几遍（区分稳定读不动 vs 偶发丢帧）")
    ap.add_argument("--data-bitrate-index", type=int, choices=range(10), default=5,
                    help="SLCAN Y 命令索引（上游 CANable2 的 5Mbps 为 5）；"
                         "--interface 时忽略")
    args = ap.parse_args()

    targets = ([(i, s, "命令行指定") for i, s in args.read] if args.read
               else list(DEFAULT_OBJECTS))
    if not args.read and not args.default:
        print("[probe] 未指定 --read，改用内置对象清单")

    probe = (SocketCanSdoProbe(args.interface, args.node) if args.interface
             else SlcanSdoProbe(args.dev, args.node, args.data_bitrate_index))
    probe.open()
    try:
        if probe.foreign_master_seen(args.listen_s) and not args.force:
            print(f"[probe] 拒绝执行：总线上已有 node {args.node} 的 SDO 请求"
                  f"（控制板配置事务在飞）。等它结束再试，或用 --force。")
            return 2
        for index, sub, value in (args.write or []):
            status, detail = probe.write_once(index, sub, value)
            print(f"  0x{index:04X}:{sub:02X}  WRITE {value} → {status}"
                  f"{'' if detail is None else f' (读回 {detail})'}")
        label = "只读上传请求" if not args.write else "只读上传请求（写已完成）"
        where = args.interface or args.dev
        print(f"[probe] node {args.node} @ {where}，{label}：")
        for index, sub, note in targets:
            for attempt in range(max(1, args.repeat)):
                status, detail, raw, tries = probe.read(index, sub)
                tag = note if args.repeat == 1 else f"{note} [第{attempt + 1}遍]"
                if status == "ok" and tries > 1:
                    tag += f"（第{tries}次才收到）"
                print(format_result(index, sub, tag, status, detail, raw, tries))
    finally:
        probe.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
