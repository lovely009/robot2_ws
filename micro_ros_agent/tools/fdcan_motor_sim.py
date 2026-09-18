#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""FDCAN1 台架电机模拟器（9 节点：CERB 1-6 + VAXOR 7-9）。

主机经 CANable2（SLCAN 串口，默认 /dev/ttyACM1）在控制板 FDCAN1 上模拟电机，
用于验证「固件 motor runtime ⇄ micro-ROS 话题」全通路。总线上无真机。
仲裁段 1M（S8）、数据段 5M（上游 CANable2 固件 b158aa7 用 Y5）。
自定义固件若使用其他 Y 索引，可通过 --data-bitrate-index 显式覆盖。

线协议字节布局严格对齐固件：
  micro_ros_firmware/lib/motor_protocol/src/motor_protocol.c
  micro_ros_firmware/lib/motor_protocol/src/{cerb,vaxor}_adapter.c

上行（本脚本 → 固件）：
  CERB  feedback  0x300+id        FD/BRS 12B 大端：deg(182count/°) / rpm / mA，
                                    data[11] bit5=0（故障位）；由 0x080 SYNC 触发
  VAXOR status    0x702+4*id      FD/BRS 20B 小端（见 motor_protocol_pack_vaxor_status）
  VAXOR reply     0x701+4*id      FD/BRS 12B，收到 0x501+4*id MIT 命令后逐帧应答

上行（本脚本 → 固件，身份识别应答）：
  CERB  SDO 应答 0x580+id        FD/BRS 8B；0x1008 分段上传（41 → 000 t nnn c）
        ——固件发现建档后读 0x1008 自报身份串并按串绑定型号，本脚本按
        --cerb-r58 分配 R58LITE_V2 / R48LITE_V2

下行（固件 → 本脚本，仅记录/驱动趋近）：
  CERB  SYNC 0x080（经典 0B，10ms）；MIT 0x110+id（FD 12B，控制字 0xcc 使能/0x0c 失能）
  CERB  SDO 请求 0x600+id（FD 8B；40 08 10 00 上传发起 → 60/70 取分段）
  VAXOR 控制 0x502+4*id（经典 8B）；MIT 0x501+4*id（FD 12B）

用法：
  python3 fdcan_motor_sim.py                 # 默认 /dev/ttyACM1，自动 S8/Y5/O 开通道
  python3 fdcan_motor_sim.py --tty /dev/ttyACM1 --log /tmp/sim.jsonl
  python3 fdcan_motor_sim.py --data-bitrate-index 7  # 仅用于对应的自定义固件
节点清单与节奏可调：--cerb/--vaxor（默认 1-6 / 7-9）、--period-ms（板端 SYNC
周期）、--fb-period-ms/--stagger-ms（反馈周期/错峰）、--no-brs（数据段不切 5M）。
开场时序要求：先开本脚本（成为 ACK 节点），再复位控制板。Ctrl-C 收尾并发 C 关通道。
"""

from __future__ import annotations

import argparse
import json
import os
import select
import signal
import struct
import termios
import time
import tty

PI = 3.141592653589793
DEFAULT_DATA_BITRATE_INDEX = 5

# ---- SLCAN -------------------------------------------------------------

FD_LEN_CODE = {0: "0", 1: "1", 2: "2", 3: "3", 4: "4", 5: "5", 6: "6",
               7: "7", 8: "8", 12: "9", 16: "A", 20: "B", 24: "C",
               32: "D", 48: "E", 64: "F"}
_CODE_LEN = {v: k for k, v in FD_LEN_CODE.items()}


def slcan_config_commands(
        data_bitrate_index: int = DEFAULT_DATA_BITRATE_INDEX) -> tuple[bytes, ...]:
    """构造关闭、1M 仲裁位率、数据位率和开通道命令。"""
    if not 0 <= data_bitrate_index <= 9:
        raise ValueError("SLCAN data bitrate index must be one decimal digit")
    return (b"C\r", b"S8\r", f"Y{data_bitrate_index}\r".encode(), b"O\r")


def schedule_feedback_window(pending: dict[int, list[float]], motor_ids,
                             window_start: float, stagger: float) -> None:
    """登记本轮反馈 deadline，保留尚未消费的上一轮 deadline。"""
    for slot, motor_id in enumerate(motor_ids):
        pending.setdefault(motor_id, []).append(
            window_start + slot * stagger)


def slcan_line(can_id: int, payload: bytes, is_fd: bool, brs: bool) -> bytes:
    """标准 11bit ID 的 SLCAN 行（CANable2 方言），以 \\r 结尾。"""
    if can_id > 0x7FF:
        raise ValueError("standard ID out of range")
    cmd = "t" if not is_fd else ("b" if brs else "d")
    return f"{cmd}{can_id:03X}{FD_LEN_CODE[len(payload)]}".encode() \
        + payload.hex().upper().encode() + b"\r"


def parse_slcan(line: str):
    """解析固件上报帧 -> (id, data, is_fd)；非数据帧返回 None。"""
    line = line.strip()
    if len(line) < 5:
        return None
    cmd, rest = line[0], line[1:]
    if cmd in "tT":
        is_fd, id_len = False, 3 if cmd == "t" else 8
    elif cmd in "dDbB":
        is_fd, id_len = True, 3 if cmd in "db" else 8
    else:
        return None  # a/w/f/错误状态等忽略
    try:
        can_id = int(rest[:id_len], 16)
        n = _CODE_LEN[rest[id_len]]
        data = bytes.fromhex(rest[id_len + 1:id_len + 1 + 2 * n])
    except (ValueError, KeyError):
        return None
    if len(data) != n:
        return None
    return can_id, data, is_fd


# ---- 线协议码值（与固件 codec 同常量） -----------------------------------

VAXOR_TICKS_PER_REV = 2_097_152
VAXOR_POS_LIMIT = 0x7CFFFFFF
VAXOR_VEL_LIMIT = 30_000
VAXOR_CUR_SCALE = 11.0 / 4096.0


def vaxor_pos_code(motor_rad: float) -> bytes:
    ticks = round(motor_rad / (2 * PI) * VAXOR_TICKS_PER_REV)
    ticks = max(-VAXOR_POS_LIMIT, min(VAXOR_POS_LIMIT, ticks))
    return int(ticks + 0x80000000).to_bytes(4, "little", signed=False)


def vaxor_vel_code(motor_rad_s: float) -> bytes:
    rpm10 = round(motor_rad_s * 60.0 / (2 * PI) * 10.0)
    rpm10 = max(-VAXOR_VEL_LIMIT, min(VAXOR_VEL_LIMIT, rpm10))
    return int(rpm10 + 0x8000).to_bytes(2, "little")


def vaxor_cur_code(current_a: float) -> bytes:
    raw = max(-32768, min(32767, round(current_a / VAXOR_CUR_SCALE)))
    return int(raw + 0x8000).to_bytes(2, "little")


def vaxor_decode_pos(code4: bytes) -> float:
    return (int.from_bytes(code4, "little", signed=False) - 0x80000000) \
        / VAXOR_TICKS_PER_REV * 2 * PI


def cerb_symmetric(code: int, limit: float, bits: int) -> float:
    return (code / float((1 << bits) - 1)) * 2.0 * limit - limit


def cerb_decode_command(data: bytes):
    """固件下行 CERB MIT（0x110+id，12B）-> dict（关节侧 rad/rad/s/Nm）。"""
    enabled = (data[0] & 0x80) != 0
    pos = cerb_symmetric((data[1] << 8) | data[2], PI, 16)
    vel = cerb_symmetric(((data[3] & 0xF0) << 4) | data[4], PI, 12)
    kp_raw = ((data[4] & 0x0F) << 8) | data[5]
    kd_raw = (data[6] << 4) | ((data[7] & 0xF0) >> 4)
    tau = cerb_symmetric(((data[7] & 0x0F) << 8) | data[8], 45.0, 12)
    return {"enabled": enabled, "pos_rad": pos, "vel_rad_s": vel,
            "kp": kp_raw * 500.0 / 4095.0, "kd": kd_raw * 5.0 / 4095.0,
            "tau_nm": tau}


# ---- 节点模型 -----------------------------------------------------------

class MotorModel:
    def __init__(self, motor_id: int, kind: str):
        self.id = motor_id
        self.kind = kind  # "cerb" | "vaxor"
        self.tx_count = 0
        self.rx_count = 0
        self.enabled = False
        self.last_cmd = None
        if kind == "cerb":
            # 关节侧：初值每台 10° 递增、101rpm(=0.1047rad/s 关节)、1A
            self.pos_deg = float(10 * motor_id)
            self.tgt_deg = self.pos_deg
            self.rpm = 101.0
            self.iq_ma = 1000.0
            # 设备自报身份（SDO 0x1008 回串），由 --cerb-r58 分配；固件据此
            # 自动绑定型号，模拟器必须实现同一对象否则台架全是不绑定电机。
            self.identity = b"R48LITE_V2"
            self.sdo_offset = 0
            # 厂家管理对象（CANopen 字典）：位置限幅 cnt（±180°）、
            # 最大转速 RPM、峰值电流 mA。固件写后读回/REFRESH 上传的对端。
            # 0x2650:01 保护开关：0 = 看门狗 + 限位保护开（出厂默认），1 = 关。
            # 0x2541 方向：0 = CW，1 = CCW。
            # 0x6064 负载端位置当前值：设零后应为 0（设零生效核验用；固件暂未读）。
            self.sdo_params = {
                (0x607D, 1): -32768,
                (0x607D, 2): 32768,
                (0x6080, 0): 600,
                (0x2538, 0): 1000,
                (0x2650, 0x01): 0,
                (0x2541, 0x00): 0,
                (0x6064, 0x00): 0,
            }
        else:
            # VAXOR 反馈为电机轴量；关节初值 0.1*(id-6) rad，gear=50
            self.pos_motor_rad = 0.1 * (motor_id - 6) * 50.0
            self.tgt_motor_rad = self.pos_motor_rad
            self.vel_motor_rad_s = 2.5
            self.current_a = 0.5

    def approach(self, dt_s: float):
        # 极简一阶趋近，仅用于演示「命令影响反馈」，不模拟真实动力学。
        rate = 2.0 * dt_s
        if self.kind == "cerb":
            self.pos_deg += (self.tgt_deg - self.pos_deg) * min(1.0, rate)
        else:
            self.pos_motor_rad += (self.tgt_motor_rad - self.pos_motor_rad) \
                * min(1.0, rate)

    def cerb_frame(self) -> bytes:
        data = struct.pack(">hhh", round(self.pos_deg * 182.0),
                           round(self.rpm), round(self.iq_ma))
        return data + b"\x00" * 6  # data[11] bit5=0（无故障）

    def vaxor_status_frame(self) -> bytes:
        cmd_id = 0x501 + 4 * self.id
        out = cmd_id.to_bytes(2, "little")
        out += vaxor_pos_code(self.pos_motor_rad)
        out += vaxor_vel_code(self.vel_motor_rad_s)
        out += vaxor_cur_code(self.current_a)
        out += struct.pack("<h", 150)          # 15.0 V
        out += (0).to_bytes(4, "little")       # fault=0
        out += bytes([4, 0, 0, 0x02])          # motor_state/calib/mode(MIT)/25mm
        return out

    def vaxor_reply_frame(self) -> bytes:
        cmd_id = 0x501 + 4 * self.id
        return cmd_id.to_bytes(2, "little") + \
            vaxor_pos_code(self.pos_motor_rad) + \
            vaxor_vel_code(self.vel_motor_rad_s) + \
            vaxor_cur_code(self.current_a)


# ---- 主循环 -------------------------------------------------------------

class SlcanSim:
    def __init__(self, tty: str, cerb_ids, vaxor_ids, period_ms: int,
                 log_path: str | None, use_brs: bool = True,
                 data_bitrate_index: int = DEFAULT_DATA_BITRATE_INDEX,
                 fb_period_ms: int = 20, stagger_ms: float = 2.2,
                 r58_ids=(), safety_off_ids=(), dir_ccw_ids=()):
        self.tty_path = tty
        self.use_brs = use_brs
        self.data_bitrate_index = data_bitrate_index
        if period_ms <= 0 or fb_period_ms <= 0:
            raise ValueError("SYNC and feedback periods must be positive")
        self.feedback_sync_interval = max(1, round(fb_period_ms / period_ms))
        self.stagger = stagger_ms / 1000.0
        self.models = {i: MotorModel(i, "cerb") for i in cerb_ids}
        self.models.update({i: MotorModel(i, "vaxor") for i in vaxor_ids})
        for mid in r58_ids:
            if mid in self.models and self.models[mid].kind == "cerb":
                self.models[mid].identity = b"R58LITE_V2"
        # 故意制造与固件记录不一致的场景：保护关 / 方向 CCW。
        for mid in safety_off_ids:
            if mid in self.models and self.models[mid].kind == "cerb":
                self.models[mid].sdo_params[(0x2650, 0x01)] = 1
        for mid in dir_ccw_ids:
            if mid in self.models and self.models[mid].kind == "cerb":
                self.models[mid].sdo_params[(0x2541, 0x00)] = 1
        self.rx_buf = b""
        self.sync_seen = False
        self.sync_count = 0
        self.next_send = {}
        self.last_cerb_emit = 0.0
        self.stop = False
        self.log_f = open(log_path, "w") if log_path else None
        self.fd = os.open(tty, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)

    def configure(self):
        tty.setraw(self.fd, when=termios.TCSANOW)
        attrs = termios.tcgetattr(self.fd)
        attrs[2] |= termios.CLOCAL | termios.CREAD
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        # 默认与 motor_drive SerialSlcanTransport 同一序列（150ms 间隔）；
        # 非上游 CANable2 固件可用 CLI 覆盖其 Y 索引。
        for cmd in slcan_config_commands(self.data_bitrate_index):
            os.write(self.fd, cmd)
            time.sleep(0.15)
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def send(self, can_id: int, payload: bytes, is_fd: bool = True,
             brs: bool = True, model: "MotorModel | None" = None,
             tag: str = ""):
        os.write(self.fd, slcan_line(can_id, payload, is_fd, brs))
        if model:
            model.tx_count += 1
        if self.log_f:
            self.log_f.write(json.dumps({
                "t": round(time.monotonic(), 4), "dir": "tx", "tag": tag,
                "id": can_id, "data": payload.hex()}) + "\n")

    def log_rx(self, can_id: int, data: bytes, tag: str):
        m = self._model_for_downlink(can_id)
        if m:
            m.rx_count += 1
        if self.log_f:
            self.log_f.write(json.dumps({
                "t": round(time.monotonic(), 4), "dir": "rx", "tag": tag,
                "id": can_id, "data": data.hex()}) + "\n")

    def _model_for_downlink(self, can_id: int):
        # CERB MIT 0x110+id(1..127)；VAXOR 命令 0x501+4n / 控制 0x502+4n
        node = can_id - 0x110
        if 1 <= node <= 127 and node in self.models \
                and self.models[node].kind == "cerb":
            return self.models[node]
        for base, kind in ((0x501, "vaxor"), (0x502, "vaxor")):
            delta = can_id - base
            if delta >= 4 and delta % 4 == 0 and (delta // 4) in self.models \
                    and self.models[delta // 4].kind == kind:
                return self.models[delta // 4]
        return None

    def handle_sdo(self, node: int, data: bytes) -> None:
        """CERB 身份读取应答（SDO 0x1008 分段上传），与固件 cerb_adapter 对向。

        厂家属具书 §4.3：上传发起 40 08 10 00 → 应答 41 08 10 00 + 总字节数；
        随后 60/70 交替取段 → 应答 000 t nnn c + 7 字节数据。
        """
        m = self.models.get(node)
        if m is None or m.kind != "cerb":
            return
        m.rx_count += 1
        command = data[0]
        index = data[1] | (data[2] << 8)
        sub = data[3]
        key = (index, sub)
        if command == 0x23:
            # 4 字节快速写：存储并回写成功 ack（0x60），无有对象则 abort。
            if key in m.sdo_params:
                m.sdo_params[key] = int.from_bytes(data[4:8], "little",
                                                   signed=True)
                self.send(0x580 + node,
                          bytes([0x60, data[1], data[2], data[3], 0, 0, 0, 0]),
                          True, self.use_brs, m, "sdo_write_ack")
            else:
                self.send(0x580 + node,
                          bytes([0x80, data[1], data[2], data[3], 0, 0, 0, 0]),
                          True, self.use_brs, m, "sdo_abort")
            return
        if command == 0x40:
            if key in m.sdo_params:
                # 快速上传（0x43）+ 当前值（小端有符号 32 位）。
                raw = int(m.sdo_params[key]).to_bytes(4, "little", signed=True)
                self.send(0x580 + node,
                          bytes([0x43, data[1], data[2], data[3]]) + raw,
                          True, self.use_brs, m, "sdo_read")
                return
            if index != 0x1008:
                self.send(0x580 + node,
                          bytes([0x80, data[1], data[2], data[3], 0, 0, 0, 0]),
                          True, self.use_brs, m, "sdo_abort")
                return
            m.sdo_offset = 0
            size = len(m.identity)
            self.send(0x580 + node,
                      bytes([0x41, 0x08, 0x10, 0x00, size & 0xFF,
                             (size >> 8) & 0xFF, 0, 0]),
                      True, self.use_brs, m, "sdo_initiate")
            return
        if (command & 0xE0) == 0x60:  # 分段上传请求（bit4 = 切换位）
            chunk = m.identity[m.sdo_offset:m.sdo_offset + 7]
            m.sdo_offset += len(chunk)
            last = m.sdo_offset >= len(m.identity)
            unused = (7 - len(chunk)) if last else 0
            head = ((command >> 4) & 1) << 4 | (unused << 1) | (1 if last else 0)
            self.send(0x580 + node, bytes([head]) + chunk.ljust(7, b"\x00"),
                      True, self.use_brs, m, "sdo_segment")

    def handle_downlink(self, can_id: int, data: bytes, is_fd: bool):
        if can_id == 0x080:  # CERB SYNC（经典 0B），链路存活探针与调度基准
            now = time.monotonic()
            if not self.sync_seen:
                self.sync_seen = True
                print("[sim] 收到首个 SYNC，链路已通，开始 SYNC 对齐错峰上报")
            self.sync_count += 1
            # 真实设备在 SYNC 后窗口上报：按反馈周期折算 SYNC 间隔，9 帧从
            # 窗口起点按 stagger 错峰，跨窗口的末槽 deadline 进入队列而不覆盖。
            if (self.sync_count - 1) % self.feedback_sync_interval == 0:
                schedule_feedback_window(
                    self.next_send, sorted(self.models), now, self.stagger)
            return
        if 0x601 <= can_id <= 0x67F and len(data) >= 8:  # SDO 请求 0x600+node
            self.handle_sdo(can_id - 0x600, data)
            return
        m = self._model_for_downlink(can_id)
        if m is None:
            return
        if m.kind == "cerb" and can_id == 0x110 + m.id and len(data) >= 9:
            cmd = cerb_decode_command(data)
            m.enabled = cmd["enabled"]
            m.last_cmd = cmd
            if cmd["enabled"]:
                m.tgt_deg = cmd["pos_rad"] * 180.0 / PI
            self.log_rx(can_id, data, "cerb_mit")
        elif m.kind == "vaxor" and can_id == 0x502 + 4 * m.id:
            m.enabled = bool(data[0])
            m.last_cmd = {"run": bool(data[0]), "set_zero": bool(data[1]),
                          "mode": data[2]}
            self.log_rx(can_id, data, "vaxor_ctrl")
        elif m.kind == "vaxor" and can_id == 0x501 + 4 * m.id:
            tgt = vaxor_decode_pos(data[2:6])
            m.tgt_motor_rad = tgt
            m.last_cmd = {"tgt_motor_rad": tgt}
            self.log_rx(can_id, data, "vaxor_mit")
            # 手册：MIT 命令逐帧应答。
            self.send(0x701 + 4 * m.id, m.vaxor_reply_frame(), True,
                      self.use_brs, m, "vaxor_reply")

    def emit_one(self, mid: int):
        m = self.models[mid]
        # --no-brs：FD 帧数据段也走仲裁 1M，用于隔离 BRS 数据相位问题。
        if m.kind == "cerb":
            self.send(0x300 + mid, m.cerb_frame(), True, self.use_brs, m,
                      "cerb_fb")
        else:
            self.send(0x702 + 4 * mid, m.vaxor_status_frame(), True,
                      self.use_brs, m, "vaxor_status")

    def emit_cerb(self):
        for mid, m in self.models.items():
            if m.kind == "cerb":
                self.send(0x300 + mid, m.cerb_frame(), True, True, m,
                          "cerb_fb")
        self.last_cerb_emit = time.monotonic()

    def emit_vaxor(self):
        for mid, m in self.models.items():
            if m.kind == "vaxor":
                self.send(0x702 + 4 * mid, m.vaxor_status_frame(), True, True,
                          m, "vaxor_status")

    def read_available(self):
        try:
            chunk = os.read(self.fd, 4096)
        except OSError:
            return
        if not chunk:
            return
        self.rx_buf += chunk
        while b"\r" in self.rx_buf:
            line, self.rx_buf = self.rx_buf.split(b"\r", 1)
            parsed = parse_slcan(line.decode(errors="replace"))
            if parsed:
                self.handle_downlink(*parsed)

    def report(self):
        print("\n=== 模拟器节点状态 ===")
        for mid in sorted(self.models):
            m = self.models[mid]
            if m.kind == "cerb":
                extra = f"pos={m.pos_deg:.1f}deg tgt={m.tgt_deg:.1f}"
            else:
                extra = f"pos={m.pos_motor_rad:.2f}rad_m tgt={m.tgt_motor_rad:.2f}"
            print(f"  {m.kind:5s} id={mid:d} tx={m.tx_count:5d} "
                  f"rx={m.rx_count:4d} en={int(m.enabled)} {extra} "
                  f"last={m.last_cmd}")
        print(f"  SYNC(0x080) seen: {self.sync_seen}")
        if self.log_f:
            self.log_f.flush()

    def run(self):
        cerb_ids = sorted(i for i, m in self.models.items() if m.kind == "cerb")
        vaxor_ids = sorted(i for i, m in self.models.items() if m.kind == "vaxor")
        print(f"[sim] {self.tty_path} S8/Y{self.data_bitrate_index}/O opened; "
              f"CERB {cerb_ids} "
              f"VAXOR {vaxor_ids}")
        print("[sim] 通道已开、静默监听中（仅做 ACK 节点）。请复位控制板；"
              "收到首个 0x080 SYNC 后开始错峰上报。Ctrl-C 结束。")
        gate_start = time.monotonic()
        last_step = gate_start
        warned = False
        while not self.stop:
            now = time.monotonic()
            r, _, _ = select.select([self.fd], [], [], 0.002)
            if r:
                self.read_available()
            if not self.sync_seen:
                # 门控：板子上线前绝不发帧（否则无人 ACK 会把 CANable 自己打进
                # bus-off，板子启动后也无人 ACK 其 SYNC，双双死锁）。
                if not warned and now - gate_start > 3.0:
                    print("[sim] 警告：3s 未收到 SYNC，请确认板子已复位且接线正确")
                    warned = True
                continue
            dt = now - last_step
            for m in self.models.values():
                m.approach(dt)
            last_step = now
            # 错峰：频率由 --fb-period-ms 决定，默认每节点 50Hz。
            for mid in sorted(self.models):
                deadlines = self.next_send.get(mid)
                if deadlines and now >= deadlines[0]:
                    self.emit_one(mid)
                    deadlines.pop(0)
                    if not deadlines:
                        del self.next_send[mid]
        self.report()
        try:
            os.write(self.fd, b"C\r")  # 收尾关通道
        except OSError:
            pass
        if self.log_f:
            self.log_f.close()
        os.close(self.fd)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tty", default="/dev/ttyACM1")
    ap.add_argument("--cerb", default="1,2,3,4,5,6")
    ap.add_argument("--vaxor", default="7,8,9")
    ap.add_argument("--cerb-r58", default="1",
                    help="自报 R58LITE_V2 身份的 CERB 节点（其余 CERB 节点"
                         "自报 R48LITE_V2；固件按身份串自动绑定型号）")
    ap.add_argument("--safety-off", type=int, nargs="*", default=[],
                    help="把这些 CERB 节点的 0x2650:01 置 1（关闭看门狗+限位保护），"
                         "用于制造与固件记录不一致的场景")
    ap.add_argument("--dir-ccw", type=int, nargs="*", default=[],
                    help="把这些 CERB 节点的 0x2541 置 1（CCW），用于制造方向不一致")
    ap.add_argument("--period-ms", type=int, default=20,
                    help="板端 SYNC 周期，用于换算反馈窗口（默认 20ms）")
    ap.add_argument("--data-bitrate-index", type=int, choices=range(10),
                    default=DEFAULT_DATA_BITRATE_INDEX,
                    help="SLCAN Y 命令索引（上游 CANable2 的 5Mbps 为 5）")
    ap.add_argument("--no-brs", action="store_true",
                    help="FD 帧不切 5M 数据段（全程 1M），用于 BRS 数据相位对照")
    ap.add_argument("--fb-period-ms", type=int, default=20,
                    help="每电机反馈周期（默认 20ms=50Hz；降低反馈负载时可调到 50）")
    ap.add_argument("--stagger-ms", type=float, default=2.2,
                    help="多节点反馈相位错开间隔")
    ap.add_argument("--log", default=None)
    args = ap.parse_args()
    cerb_ids = [int(x) for x in args.cerb.split(",") if x]
    vaxor_ids = [int(x) for x in args.vaxor.split(",") if x]
    r58_ids = [int(x) for x in args.cerb_r58.split(",") if x]
    sim = SlcanSim(args.tty, cerb_ids, vaxor_ids, args.period_ms, args.log,
                   use_brs=not args.no_brs,
                   data_bitrate_index=args.data_bitrate_index,
                   fb_period_ms=args.fb_period_ms,
                   stagger_ms=args.stagger_ms,
                   r58_ids=r58_ids,
                   safety_off_ids=args.safety_off,
                   dir_ccw_ids=args.dir_ccw)

    def _sig(_a, _b):
        sim.stop = True
    signal.signal(signal.SIGTERM, _sig)
    signal.signal(signal.SIGINT, _sig)

    sim.configure()
    sim.run()


if __name__ == "__main__":
    main()
