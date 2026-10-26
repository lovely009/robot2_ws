#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CERB MIT 满量程反推探针（真机台架专用；只发 MIT 帧 + SDO 读，不烧板子、不改设备）。

⚠️ **只对台架上的真电机运行**。本仓库自带的 `fdcan_motor_sim.py` 用 `PI/PI/300.0`
（按型号库 `mit_torque_limit_nm`）解码 MIT 帧——那是照固件假设写死的，用它回答
"MIT 满量程是多少"是**自证循环**（循环论证），**不能**作为证据。真电机到场前这个
问题无从回答。对应厂商问题清单第 1 条（见
`micro_ros_firmware/docs/architecture/motor-runtime-decision-log.md` §12）。

本脚本只做两件事，**不 flash、不写任何设备对象**：
  1. 发满量程 MIT 单轴帧（`0x110|id`，12B，控制字 0xcc）：
       目标力矩 = 4095（12bit 满量程）→ 读 `0x6077`（力矩传感器，0.01 N·m）
       目标速度 = 4095              → 读 `0x606C`（速度实际值）
       目标位置 = 65535 / 0（16bit 两极端）→ 读 `0x6064`（位置实际值）
  2. 顺带读 `0x6080`（电机端最大转速 RPM）与 `0x2538`（峰值电流 mA）作对照。

把反算出的物理满量程与当时的 `0x6080` / `0x2538` 一比，MIT 的
`Pos Max` / `Vel Max` / `T_Max` 取自哪个对象即见分晓。

用法（SLCAN 链接与组帧复用 `fdcan_motor_sim.py`，不另造一套）：
  python3 tools/cerb_fullscale_probe.py --dev /dev/serial/by-id/<CANable> --node 1
  python3 tools/cerb_fullscale_probe.py --dev /dev/ttyACM1 --node 3 --hold-ms 300

台架前提（见 `micro_ros_firmware/docs/guides/fdcan1-bench-guide.md`）：CANable 先
成为 ACK 节点（本脚本自动 C→S8→Y5→M0→A1→O）。**本脚本会发使能 + 满量程出力帧，电机会
真的出力**——务必保证机构已支撑/空载且急停可用。
"""

from __future__ import annotations

import argparse
import os
import select
import termios
import time
import tty

from fdcan_motor_sim import slcan_config_commands, slcan_line, parse_slcan

CERB_MIT_ID_BASE = 0x110
SDO_REQUEST_ID_BASE = 0x600
SDO_RESPONSE_ID_BASE = 0x580

# 只读对照对象。子索引均为 0。
OBJ_TORQUE_SENSOR = 0x6077    # 力矩传感器（RO，0.01 N·m）
OBJ_VELOCITY_ACTUAL = 0x606C  # 速度实际值
OBJ_POSITION_ACTUAL = 0x6064  # 位置实际值
OBJ_MAX_SPEED = 0x6080        # 电机端最大转速（RPM）
OBJ_MAX_CURRENT = 0x2538      # 峰值电流（mA）


def cerb_mit_frame(enabled: bool, pos_raw: int = 0x8000, vel_raw: int = 0x800,
                   kp_raw: int = 0, kd_raw: int = 0,
                   tau_raw: int = 0x800) -> bytes:
    """按固件 cerb 打包布局组 12B MIT 帧，各物理量用**原始码值**直填。

    与固件 `motor_protocol_pack_cerb_mit_frame` 逐位对齐：
      [0] bit7=使能 bit6=抱闸释放 bit[4:1]=0x06(MIT)（使能=0xcc，否则 0x0c）
      [1:2] 位置 16bit | [3]+[4]高4 速度 12bit | [4]低4+[5] kp 12bit
      [6]+[7]高4 kd 12bit | [7]低4+[8] 力矩 12bit | [9..11] 0
    默认码值取各字段量程中点（物理 0）。
    """
    data = bytearray(12)
    data[0] = (0xC0 if enabled else 0x00) | 0x0C
    data[1] = (pos_raw >> 8) & 0xFF
    data[2] = pos_raw & 0xFF
    data[3] = (vel_raw >> 4) & 0xFF
    data[4] = ((vel_raw & 0x0F) << 4) | ((kp_raw >> 8) & 0x0F)
    data[5] = kp_raw & 0xFF
    data[6] = (kd_raw >> 4) & 0xFF
    data[7] = ((kd_raw & 0x0F) << 4) | ((tau_raw >> 8) & 0x0F)
    data[8] = tau_raw & 0xFF
    return bytes(data)


class CerbProbe:
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
        # 与 fdcan_motor_sim 同一序列（C→S8→Y?→M0→A1→O，150ms 间隔）。
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

    def send(self, can_id: int, payload: bytes):
        os.write(self.fd, slcan_line(can_id, payload, True, True))

    def _drain(self, timeout: float):
        """读一段 SLCAN 行，返回 [(id, data)]，最多等 timeout 秒。"""
        frames = []
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            r, _, _ = select.select([self.fd], [], [],
                                    max(0.0, deadline - time.monotonic()))
            if not r:
                break
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
        return frames

    def sdo_read(self, index: int, sub: int = 0, timeout: float = 0.3):
        """发 SDO 快速上传读（0x40），等 0x580+node 应答，返回 (raw int, ok)。"""
        req = bytes([0x40, index & 0xFF, (index >> 8) & 0xFF, sub, 0, 0, 0, 0])
        self.send(SDO_REQUEST_ID_BASE + self.node, req)
        for can_id, data in self._drain(timeout):
            if can_id == SDO_RESPONSE_ID_BASE + self.node and len(data) >= 8:
                if data[0] == 0x43:  # 快速上传 4 字节
                    return int.from_bytes(data[4:8], "little", signed=True), True
                if data[0] == 0x80:  # abort
                    return None, False
        return None, False

    def hold_mit(self, frame: bytes, hold_ms: int, period_ms: int = 10):
        """在 hold_ms 内周期重发 MIT 帧（设备需持续命令才保持出力）。"""
        end = time.monotonic() + hold_ms / 1000.0
        while time.monotonic() < end:
            self.send(CERB_MIT_ID_BASE + self.node, frame)
            time.sleep(period_ms / 1000.0)


def _line(label: str, raw, ok: bool, scale: float = 1.0, unit: str = ""):
    if not ok:
        print(f"  {label:22s}: <读失败 / abort>")
    else:
        print(f"  {label:22s}: raw={raw:>12d}  ≈ {raw * scale:.4f} {unit}")


def _self_check():
    """钉住 MIT 组帧与固件 `motor_protocol_pack_cerb_mit_frame` 的手工对齐。

    这份 bit 布局是照抄固件写的，固件一改、这里静默漂移，探针就会发错帧还看不出
    来。用一组非平凡码值当已知向量：任一位字段错位都会让断言炸。
    """
    got = cerb_mit_frame(True, pos_raw=0x1234, vel_raw=0xABC, kp_raw=0x567,
                         kd_raw=0x89A, tau_raw=0xDEF)
    want = bytes([0xCC, 0x12, 0x34, 0xAB, 0xC5, 0x67, 0x89, 0xAD, 0xEF, 0, 0, 0])
    assert got == want, (got.hex(), want.hex())
    # 失能只清 bit7：控制字 0x0C，其余字节与使能态一致。
    assert cerb_mit_frame(False, pos_raw=0x1234)[0] == 0x0C


def main():
    ap = argparse.ArgumentParser(
        description="CERB MIT 满量程反推探针（只发 MIT 帧 + SDO 读，不烧板子/不改设备）")
    ap.add_argument("--dev", required=True,
                    help="CANable SLCAN 串口，如 /dev/serial/by-id/<CANable>")
    ap.add_argument("--node", type=int, default=1, help="CERB 节点 id（1..127）")
    ap.add_argument("--hold-ms", type=int, default=200,
                    help="每段满量程命令的保持时长（默认 200ms）")
    ap.add_argument("--kp", type=int, default=0, help="kp 原始码（12bit，默认 0）")
    ap.add_argument("--kd", type=int, default=0, help="kd 原始码（12bit，默认 0）")
    ap.add_argument("--data-bitrate-index", type=int, choices=range(10), default=5,
                    help="SLCAN Y 命令索引（上游 CANable2 的 5Mbps 为 5）")
    args = ap.parse_args()

    _self_check()  # 组帧布局先自检，别拿错帧去对真机

    print(f"[probe] 目标：CERB node {args.node} @ {args.dev}")
    print("[probe] ⚠️ 将发使能 + 满量程 MIT 帧，电机会真的出力——"
          "确认机构已支撑/空载、急停可用")

    probe = CerbProbe(args.dev, args.node, args.data_bitrate_index)
    probe.open()
    try:
        print("[probe] 对照对象（只读）：")
        max_speed, ok_s = probe.sdo_read(OBJ_MAX_SPEED)
        _line("0x6080 最大转速", max_speed, ok_s, 1.0, "RPM 电机端")
        max_cur, ok_c = probe.sdo_read(OBJ_MAX_CURRENT)
        _line("0x2538 峰值电流", max_cur, ok_c, 0.001, "A 电机端")

        print("[probe] 满量程力矩 4095 → 读 0x6077（0.01 N·m）")
        probe.hold_mit(cerb_mit_frame(True, tau_raw=0xFFF,
                                      kp_raw=args.kp, kd_raw=args.kd),
                       args.hold_ms)
        tau, ok = probe.sdo_read(OBJ_TORQUE_SENSOR)
        _line("0x6077 力矩传感器", tau, ok, 0.01, "N·m")

        print("[probe] 满量程速度 4095 → 读 0x606C")
        probe.hold_mit(cerb_mit_frame(True, vel_raw=0xFFF,
                                      kp_raw=args.kp, kd_raw=args.kd),
                       args.hold_ms)
        vel, ok = probe.sdo_read(OBJ_VELOCITY_ACTUAL)
        _line("0x606C 速度实际值", vel, ok)

        for raw, tag in ((0xFFFF, "16bit 顶"), (0x0000, "16bit 底")):
            print(f"[probe] 位置 {tag} = {raw} → 读 0x6064")
            probe.hold_mit(cerb_mit_frame(True, pos_raw=raw,
                                          kp_raw=args.kp, kd_raw=args.kd),
                           args.hold_ms)
            pos, ok = probe.sdo_read(OBJ_POSITION_ACTUAL)
            _line("0x6064 位置实际值", pos, ok)

        print("[probe] 满量程序列结束。把反算的物理满量程与 0x6080 / 0x2538 "
              "对照即见分晓。")
    finally:
        # 无论中途异常都先发失能帧：设备侧保护若被外部工具关掉就没有 >500ms
        # 自锁兜底，异常退出时电机会停在满量程出力。
        try:
            probe.hold_mit(cerb_mit_frame(False), 60)
        except OSError:
            pass
        probe.close()


if __name__ == "__main__":
    main()
