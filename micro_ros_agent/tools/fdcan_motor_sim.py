#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""FDCAN 台架电机模拟器（默认 4 节点：CERB 1=R58、2-4=R48）。

主机经 CANable2（SLCAN 串口，默认 /dev/ttyACM1）在控制板物理电机总线上模拟电机，
用于验证「固件 motor runtime ⇄ micro-ROS 话题」全通路。总线上无真机。
仲裁段 1M（S8）、数据段 5M。CANable2 的数据位率索引按 V 查询自动选择；
当前实机返回的自定义版本 16e7497-dirty 使用 Y7，上游 b158aa7 使用 Y5。
也可通过 --data-bitrate-index 显式覆盖。

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
  CERB  SYNC 0x080（FD+BRS 0B，默认 20ms）；MIT 0x110+id（FD 12B，控制字 0xcc 使能/0x0c 失能）
  CERB  SDO 请求 0x600+id（FD 8B；40 08 10 00 上传发起 → 60/70 取分段）
  VAXOR 控制 0x502+4*id（经典 8B）；MIT 0x501+4*id（FD 12B）

用法：
  python3 fdcan_motor_sim.py                 # 默认 /dev/ttyACM1，自动 V/S8/Y*/M0/A1/O 开通道
  python3 fdcan_motor_sim.py --tty /dev/ttyACM1 --log /tmp/sim.jsonl
  python3 fdcan_motor_sim.py --data-bitrate-index 7  # 仅用于对应的自定义固件
  # 与 Gazebo effort 控制器做真实 MIT 等效力矩端到端验证：
  python3 fdcan_motor_sim.py --gazebo --gazebo-control-mode effort \
    --joint-map 7:joint_link1,8:joint_link2,9:joint_link3,10:joint_link4
  # 需要真实 effort 动力学时，Gazebo 也必须用 control_mode:=effort：
  python3 fdcan_motor_sim.py --gazebo --gazebo-control-mode effort \
    --effort-topic /arm_effort_controller/commands
节点清单与节奏可调：--cerb/--vaxor（默认 CERB 1-4 / 无 VAXOR）、--period-ms（板端 SYNC
周期）、--fb-period-ms/--stagger-ms（反馈周期/错峰，默认 60ms/台）、--no-brs（数据段不切 5M）。
开场时序要求：先开本脚本（成为 ACK 节点），再复位控制板。Ctrl-C 收尾并发 C 关通道。
"""

from __future__ import annotations

import argparse
from collections import deque
import fcntl
import glob
import json
import os
import queue
import select
import signal
import struct
import sys
import termios
import threading
import time
import tty

PI = 3.141592653589793
DEFAULT_DATA_BITRATE_INDEX = 5
AUTO_DATA_BITRATE_INDEX = None
SLCAN_VERSION_DATA_BITRATE = {
    # 当前现场 V 响应是自定义 16e7497-dirty；历史台架验证该版本的
    # 5Mbps 索引为 Y7。上游 b158aa7 的官方语义则是 Y5=5Mbps。
    "16e7497": 7,
    "b158aa7": 5,
}

# ---- SLCAN -------------------------------------------------------------

FD_LEN_CODE = {0: "0", 1: "1", 2: "2", 3: "3", 4: "4", 5: "5", 6: "6",
               7: "7", 8: "8", 12: "9", 16: "A", 20: "B", 24: "C",
               32: "D", 48: "E", 64: "F"}
_CODE_LEN = {v: k for k, v in FD_LEN_CODE.items()}


def slcan_config_commands(
        data_bitrate_index: int = DEFAULT_DATA_BITRATE_INDEX) -> tuple[bytes, ...]:
    """构造关闭、位率、正常模式、自动重发和开通道命令。"""
    if not 0 <= data_bitrate_index <= 9:
        raise ValueError("SLCAN data bitrate index must be one decimal digit")
    return (b"C\r", b"S8\r", f"Y{data_bitrate_index}\r".encode(),
            b"M0\r", b"A1\r", b"O\r")


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
    # 查表失败要报得清楚：隐式 KeyError 只留下一个数字，定位不到是哪条帧
    # （曾因此把整个模拟器打死而看不出原因）。合法长度见 FD_LEN_CODE。
    code = FD_LEN_CODE.get(len(payload))
    if code is None:
        raise ValueError(
            f"CAN FD 非法负载长度 {len(payload)}（id=0x{can_id:03X}，"
            f"合法：{sorted(FD_LEN_CODE)}）")
    return f"{cmd}{can_id:03X}{code}".encode() \
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
VAXOR_KP_SCALE = 0.00001
VAXOR_KD_SCALE = 0.01
VAXOR_GEAR_RATIO = 50.0
VAXOR_TORQUE_PER_AMP = 0.042 * VAXOR_GEAR_RATIO * 0.65

# 启动期开通道命令的串口写重试预算（秒）。运行期数据帧不在收包路径里阻塞，
# 统一进入发送队列，由主循环泵出；否则 SDO 应答遇到 USB CDC 背压会卡住反馈，
# CERB 100ms 心跳门会把所有节点判成 OFFLINE。
TX_RETRY_S = 0.25
TX_QUEUE_MAX = 512
TX_PUMP_BUDGET = 8192


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


def vaxor_decode_vel(code2: bytes) -> float:
    rpm10 = int.from_bytes(code2, "little", signed=False) - 0x8000
    return rpm10 / 10.0 * 2 * PI / 60.0


def vaxor_decode_current(code2: bytes) -> float:
    raw = int.from_bytes(code2, "little", signed=False) - 0x8000
    return raw * VAXOR_CUR_SCALE


def vaxor_decode_command(data: bytes):
    """固件下行 VAXOR MIT（12B）-> 关节侧控制量。

    固件先把位置/速度从关节侧换算到电机轴，再按协议写入：
    position[0:4]、velocity[4:6]、kp[6:8]、kd[8:10]、Iq[10:12]。
    旧代码从 data[2:6] 取位置且没有增益，导致目标角和力矩都失真。
    """
    if len(data) != 12:
        return None
    position_motor = vaxor_decode_pos(data[0:4])
    velocity_motor = vaxor_decode_vel(data[4:6])
    kp = int.from_bytes(data[6:8], "little", signed=False) * VAXOR_KP_SCALE
    kd = int.from_bytes(data[8:10], "little", signed=False) * VAXOR_KD_SCALE
    feedforward_a = vaxor_decode_current(data[10:12])
    return {
        "position_motor_rad": position_motor,
        "velocity_motor_rad_s": velocity_motor,
        "pos_rad": position_motor / VAXOR_GEAR_RATIO,
        "vel_rad_s": velocity_motor / VAXOR_GEAR_RATIO,
        "kp": kp,
        "kd": kd,
        "tau_nm": feedforward_a * VAXOR_TORQUE_PER_AMP,
    }


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
            # 电机线圈温度（°C），进反馈帧 data[8..9]（0.1 °C 定点）。
            # 每台给不同初值，便于在页面上区分"读到了哪一臺"而不是一片同值。
            self.temp_c = 30.0 + 2.0 * motor_id
            # 设备自报身份（SDO 0x1008 回串），由 --cerb-r58 分配；固件据此
            # 自动绑定型号，模拟器必须实现同一对象否则台架全是不绑定电机。
            self.identity = b"R48LITE_V2"
            self.sdo_offset = 0
            # 厂家管理对象（CANopen 字典）：位置限幅 cnt（±180°）、
            # 最大转速 RPM、峰值电流 mA。固件写后读回/REFRESH 上传的对端。
            # 0x2650:00 保护开关：0 = 看门狗 + 限位保护开（出厂默认），1 = 关。
            # 子索引 0x00 是 2026-09-19 真机实测确认的**唯一**存在的那一个
            # （:01/:02 回 ABORT 0x06090011）；建错子索引会让台架永远复现不出固件
            # 读不动的现象——模拟器要照对端建模，不是照固件假设建模。
            # 0x2541 方向：0 = CW，1 = CCW。
            # 0x6064 负载端位置当前值：设零后应为 0（设零生效核验用；固件暂未读）。
            # ⚠️ 初值 = **真机台架实测的上电初值**（进程重启即等价于掉电复现）：
            # 0x2650:00 = 1（保护**关**）、0x2541:00 = 0（CW）。这两个对象都易失，
            # 靠固件上电归一在回读比对之前写成"保护生效 + 记录里的方向"。初值给
            # 0（保护开）会让台架永远复现不出"上电后保护是关的"这个场景——模拟器
            # 照对端建模，不照固件假设建模（2026-09-19 的教训）。
            self.sdo_params = {
                (0x607D, 1): -32768,
                (0x607D, 2): 32768,
                (0x6080, 0): 600,
                (0x2538, 0): 1000,
                (0x2650, 0x00): 1,
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
        """12B 周期状态帧（0x300+n）：位置 / 转速 / Iq / 错误码 / 线圈温度 / 模式 / 状态位。

        温度在 data[8..9]（大端 int16，0.1 °C），固件 cerb_adapter 会读它填
        MotorStatusArray.temperature——曾经这里是 6 字节零填充，页面上恒显示
        "--"，看着像 UI 缺字段，实为台架不发。
        """
        data = struct.pack(">hhh", round(self.pos_deg * 182.0),
                           round(self.rpm), round(self.iq_ma))
        data += struct.pack(">h", 0)                      # data[6..7] 错误代码
        data += struct.pack(">h", round(self.temp_c * 10))  # data[8..9] 线圈温度
        # data[10] 控制模式反馈、data[11] 状态位（使能/抱闸/报错/到位）本模拟器
        # 不建模，保持 0；固件目前也只读 data[11] bit5（故障）。bit5=0 → 无故障。
        return data + bytes([0x00, 0x00])

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
        """12B 回显帧（0x701+4n）——必须补满 12，不能只发内容那 10 字节。

        固件侧 motor_protocol_pack_vaxor_reply 写的是 uint8_t[12]，解码
        （motor_protocol.c 的 unpack_vaxor_feedback）认 size == 12 或 20，
        11/13 直接拒（test_motor_protocol.cpp 有断言），并且只读字节 2..9
        （位置 4B / 速度 2B / 电流 2B）——尾 2 字节固件不读。

        而且 10 在 CAN FD 里根本不是合法长度（只有 0-8/12/16/20/24/32/48/64），
        真硬件也发不出来：漏补会让 slcan_line 的 FD_LEN_CODE 查表 KeyError，
        把整个模拟器打死在半路上（2026-09-20 踩过：武装 m7/m8 后第一条 VAXOR
        回显就崩，之前所有台架轮次跑的都是 CERB，这条路从没被走到）。
        """
        cmd_id = 0x501 + 4 * self.id
        return cmd_id.to_bytes(2, "little") + \
            vaxor_pos_code(self.pos_motor_rad) + \
            vaxor_vel_code(self.vel_motor_rad_s) + \
            vaxor_cur_code(self.current_a) + b"\x00\x00"


# ---- 主循环 -------------------------------------------------------------

class GazeboBridge:
    """--gazebo：把仿真电机命令接到 Gazebo 的关节上。

    `position` 是端到端功能验证的默认模式：把固件已经解码出的关节目标
    交给 Gazebo 的 position controller，反馈仍然来自真实 Gazebo 关节状态，
    因而可稳定验证 ID/单位/关节顺序/轨迹时序，不把未建模的重力补偿误判为
    通信故障。`effort` 是显式动力学模式，按 MIT 律计算力矩；使用它时必须
    自己提供足够的增益/前馈来对抗模型重力。

    未映射的电机照旧走原来的假 plant（--joint-map 决定谁接 Gazebo）。
    """

    def __init__(self, joint_map, control_mode, position_topic, effort_topic,
                 tau_limit):
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray
        if control_mode not in ("position", "effort"):
            raise ValueError("Gazebo control mode must be position or effort")
        self._rclpy = rclpy
        self._msg_type = Float64MultiArray
        rclpy.init()
        self.node = Node("fdcan_sim_gazebo_bridge")
        self.joint_map = dict(joint_map)          # {motor_id: joint_name}
        self.joints = sorted(set(joint_map.values()))
        self.pos = {j: 0.0 for j in self.joints}
        self.vel = {j: 0.0 for j in self.joints}
        # 当前模型的 ros2_control 初始位置契约是 0 rad。position 模式在
        # 控制器启动稍晚于模型生成时仍要把关节拉回这个已知安全姿态，不能把
        # 启动窗口里因尚未收到命令而坠落的反馈当成新的 hold 目标。
        self.hold_pos = {joint: 0.0 for joint in self.joints}
        self.control_mode = control_mode
        self.tau_limit = tau_limit
        self.last_pub = 0.0
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_state, 10)
        topic = position_topic if control_mode == "position" else effort_topic
        self.pub = self.node.create_publisher(Float64MultiArray, topic, 10)

    def _on_state(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.pos and i < len(msg.position):
                self.pos[name] = msg.position[i]
                self.vel[name] = msg.velocity[i] if i < len(msg.velocity) else 0.0

    def mapped(self, motor_id) -> bool:
        return motor_id in self.joint_map

    def joint_deg(self, motor_id) -> float:
        return self.pos[self.joint_map[motor_id]] * 180.0 / PI

    def joint_position(self, motor_id) -> float:
        return self.pos[self.joint_map[motor_id]]

    def joint_velocity(self, motor_id) -> float:
        return self.vel[self.joint_map[motor_id]]

    def spin(self):
        self._rclpy.spin_once(self.node, timeout_sec=0.0)

    def publish_commands(self, models):
        now = time.monotonic()
        if now - self.last_pub < 0.015:   # ~66Hz 上限，别把串口/CPU 打满
            return
        self.last_pub = now
        out = self._msg_type()
        # 没有有效下行命令时保持仿真器启动时的姿态；不能每拍把当前反馈
        # 重新当成目标，否则模型会在重力下自由下落后锁存新姿态。
        out.data = [self.hold_pos.get(joint, self.pos[joint])
                    for joint in self.joints]
        for mid, joint in self.joint_map.items():
            m = models.get(mid)
            cmd = m.last_cmd if m is not None else None
            if m is None or not m.enabled or not cmd:
                continue
            index = self.joints.index(joint)
            if self.control_mode == "position":
                if "pos_rad" in cmd:
                    out.data[index] = cmd["pos_rad"]
                continue
            if "kp" not in cmd:
                continue
            tau = (cmd["kp"] * (cmd["pos_rad"] - self.pos[joint])
                   + cmd["kd"] * (cmd["vel_rad_s"] - self.vel[joint])
                   + cmd["tau_nm"])
            out.data[index] = max(-self.tau_limit, min(self.tau_limit, tau))
        self.pub.publish(out)


class SlcanSim:
    bridge = None   # 由 main() 在 --gazebo 时注入 GazeboBridge
    def __init__(self, tty: str, cerb_ids, vaxor_ids, period_ms: int,
                 log_path: str | None, use_brs: bool = True,
                 data_bitrate_index: int | None = AUTO_DATA_BITRATE_INDEX,
                 fb_period_ms: int = 60, stagger_ms: float = 2.2,
                 r58_ids=(), safety_off_ids=(), dir_ccw_ids=()):
        self.tty_path = tty
        self.use_brs = use_brs
        self.data_bitrate_index = data_bitrate_index
        if period_ms <= 0 or fb_period_ms <= 0:
            raise ValueError("SYNC and feedback periods must be positive")
        self.feedback_sync_interval = max(1, round(fb_period_ms / period_ms))
        self.stagger = stagger_ms / 1000.0
        self.tx_dropped = 0   # 发送队列满/串口错误丢帧
        self._tx_control = deque()   # SDO/应答/管理帧：优先于周期反馈
        self._tx_feedback = deque()  # 周期反馈：队列满时允许丢弃
        self._tx_active = None
        self.models = {i: MotorModel(i, "cerb") for i in cerb_ids}
        self.models.update({i: MotorModel(i, "vaxor") for i in vaxor_ids})
        for mid in r58_ids:
            if mid in self.models and self.models[mid].kind == "cerb":
                self.models[mid].identity = b"R58LITE_V2"
        # 故意制造与固件记录不一致的场景：保护关 / 方向 CCW。
        for mid in safety_off_ids:
            if mid in self.models and self.models[mid].kind == "cerb":
                self.models[mid].sdo_params[(0x2650, 0x00)] = 1
        for mid in dir_ccw_ids:
            if mid in self.models and self.models[mid].kind == "cerb":
                self.models[mid].sdo_params[(0x2541, 0x00)] = 1
        # 请求的 id 若没匹配到任何 CERB 节点，两个开关都静默无效——操作员会拿到
        # 一次"匹配"的运行并误判固件的不一致门没触发（安全属性假阴性）。显式报出。
        for flag, ids in (("--safety-off", safety_off_ids),
                          ("--dir-ccw", dir_ccw_ids)):
            missed = [i for i in ids if i not in self.models
                      or self.models[i].kind != "cerb"]
            if missed:
                print(f"[sim] 警告：{flag} 的 id {missed} 未匹配任何 CERB 节点，"
                      f"不会产生不一致场景", file=sys.stderr)
        self.rx_buf = b""
        self._rxq = queue.Queue()   # 读线程 -> 主循环的已解析帧
        self._reader = None
        self.sync_seen = False
        self.sync_count = 0
        self.last_sync_at = None
        self.max_sync_gap = 0.0
        self.next_send = {}
        self.last_cerb_emit = 0.0
        self.stop = False
        self.log_f = open(log_path, "w") if log_path else None
        self.fd = os.open(tty, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        # 独占锁：两个实例同时开同一个 CANable，会各自下发 SLCAN 配置并互相
        # 打断 → 表现为随机总线错乱、SYNC 间隔异常、某些电机不跟（2026-09-22
        # 踩过，排查了很久才发现是残留实例）。宁可第二个实例立刻失败。
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            raise SystemExit(
                f"{tty} 已被另一个进程占用（很可能是残留的模拟器实例）；"
                "先停掉它再启动，否则两条 SLCAN 流会互相打断。")

    def configure(self):
        tty.setraw(self.fd, when=termios.TCSANOW)
        attrs = termios.tcgetattr(self.fd)
        # tty.setraw 不会修正继承的字符宽度。手工查询 CANable 时若把 tty
        # 留在 CS5/CS7，后续 SLCAN 命令会被截断，表现为“设备已打开但无帧”。
        attrs[2] = ((attrs[2] & ~termios.CSIZE) | termios.CS8
                    | termios.CLOCAL | termios.CREAD)
        attrs[2] &= ~(termios.PARENB | termios.CSTOPB)
        attrs[4] = termios.B115200
        attrs[5] = termios.B115200
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        if self.data_bitrate_index is None:
            version = self._probe_version()
            self.data_bitrate_index = self._bitrate_for_version(version)
            if version:
                print(f"[sim] CANable V={version}; 自动选择 Y{self.data_bitrate_index}")
            else:
                print(f"[sim] 未读到 CANable V 响应；回退 Y{self.data_bitrate_index}，"
                      "如仍 ACK error 请显式指定 --data-bitrate-index")
        # 默认与 motor_drive SerialSlcanTransport 同一序列（150ms 间隔）；
        # 非上游 CANable2 固件可用 CLI 覆盖其 Y 索引。
        for cmd in slcan_config_commands(self.data_bitrate_index):
            self._write(cmd, drop_ok=False)
            time.sleep(0.15)
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        # 配置期（含 V 探测）不用读线程；配置完再起，避免与探测抢 fd。
        self._reader = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader.start()

    def _probe_version(self) -> str | None:
        """读取 CANable 的 V 响应；只用于选择数据位率，不消费 CAN 帧。"""
        self._write(b"V\r", drop_ok=False)
        deadline = time.monotonic() + 0.6
        buf = b""
        while time.monotonic() < deadline:
            wait = max(0.0, min(0.05, deadline - time.monotonic()))
            readable, _, _ = select.select([self.fd], [], [], wait)
            if not readable:
                continue
            try:
                buf += os.read(self.fd, 256)
            except BlockingIOError:
                continue
            for raw in buf.split(b"\r"):
                line = raw.decode("ascii", errors="ignore").strip()
                if line and not line.startswith(("t", "T", "d", "D", "b", "B")):
                    return line
        return None

    @staticmethod
    def _bitrate_for_version(version: str | None) -> int:
        if version:
            for prefix, index in SLCAN_VERSION_DATA_BITRATE.items():
                if version.startswith(prefix):
                    return index
        return DEFAULT_DATA_BITRATE_INDEX

    def _write(self, data: bytes, drop_ok: bool) -> bool:
        """带 EAGAIN 重试的串口写。

        drop_ok=False（开通道的配置命令）——重试超时后照旧抛出：启动期写不进去
        是配置错误，要立刻暴露。
        drop_ok=True（周期反馈/回显）——超时则丢这一帧并计数，绝不退出。
        """
        deadline = time.monotonic() + TX_RETRY_S
        while True:
            try:
                os.write(self.fd, data)
                return True
            except BlockingIOError:
                if not drop_ok:
                    raise
                if time.monotonic() >= deadline:
                    self.tx_dropped += 1
                    return False
                time.sleep(0.001)

    def _pump_tx(self):
        """非阻塞泵出运行期 SLCAN 帧，绝不在收包路径等待 USB CDC。"""
        budget = TX_PUMP_BUDGET
        while budget > 0:
            if self._tx_active is None:
                if self._tx_control:
                    self._tx_active = self._tx_control.popleft()
                elif self._tx_feedback:
                    self._tx_active = self._tx_feedback.popleft()
                else:
                    return
            item = self._tx_active
            try:
                written = os.write(self.fd, item["line"][item["offset"]:])
            except BlockingIOError:
                return
            except OSError:
                self.tx_dropped += 1
                self._tx_active = None
                return
            if written <= 0:
                return
            item["offset"] += written
            budget -= written
            if item["offset"] != len(item["line"]):
                return
            model = item["model"]
            if model:
                model.tx_count += 1
            if self.log_f:
                self.log_f.write(json.dumps({
                    "t": round(time.monotonic(), 4), "dir": "tx",
                    "tag": item["tag"], "id": item["can_id"],
                    "data": item["payload"].hex()}) + "\n")
            self._tx_active = None

    def send(self, can_id: int, payload: bytes, is_fd: bool = True,
             brs: bool = True, model: "MotorModel | None" = None,
             tag: str = ""):
        line = slcan_line(can_id, payload, is_fd, brs)
        # 周期反馈可丢；SDO/控制回显不能被反馈队列长期挤压。
        queue = (self._tx_feedback if tag in ("cerb_fb", "vaxor_status")
                 else self._tx_control)
        if len(queue) >= TX_QUEUE_MAX:
            self.tx_dropped += 1
            return
        queue.append({"line": line, "offset": 0, "model": model,
                      "tag": tag, "can_id": can_id, "payload": payload})

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
                          bytes([0x80, data[1], data[2], data[3],
                                 0x11, 0x00, 0x09, 0x06]),  # 0x06090011 子索引不存在
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
                          bytes([0x80, data[1], data[2], data[3],
                                 0x11, 0x00, 0x09, 0x06]),  # 0x06090011 子索引不存在
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
            if self.last_sync_at is not None:
                gap = now - self.last_sync_at
                self.max_sync_gap = max(self.max_sync_gap, gap)
                if gap > 0.1:
                    print(f"[sim] SYNC 间隔异常：{gap:.3f}s（可能是控制帧负载/总线错误）")
            self.last_sync_at = now
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
            cmd = vaxor_decode_command(data)
            if cmd is None:
                return
            m.tgt_motor_rad = cmd["position_motor_rad"]
            m.last_cmd = cmd
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

    def _reader_loop(self):
        """独立线程：只负责尽快把 CANable 的 SLCAN 数据读干净并入队。

        为什么必须单开线程：串口读原先与 rclpy/Gazebo 桥共用一个循环，主机
        负载一上来（Gazebo + 测试脚本），循环周期涨到 0.1–0.6s，CANable 的
        USB 缓冲被写满 → 它写阻塞、控制器出错、**停发 ACK** → 固件侧表现为
        `TEC=128/LEC=ACK error/TXBRP=0xff`、整条总线看着死掉。串口服务必须
        与其它工作解耦：读线程只管读，主循环只管消费。
        本线程独占 self.fd 的读与 self.rx_buf（主循环不再读 fd）。
        """
        while not self.stop:
            try:
                r, _, _ = select.select([self.fd], [], [], 0.05)
            except (OSError, ValueError):
                return
            if not r:
                continue
            try:
                chunk = os.read(self.fd, 4096)
            except BlockingIOError:
                continue
            except OSError:
                return
            if not chunk:
                continue
            self.rx_buf += chunk
            while b"\r" in self.rx_buf:
                line, self.rx_buf = self.rx_buf.split(b"\r", 1)
                parsed = parse_slcan(line.decode(errors="replace"))
                if parsed:
                    self._rxq.put(parsed)

    def _drain_rx(self):
        """主循环侧：把读线程已入队的帧全部处理掉。"""
        while True:
            try:
                can_id, data, is_fd = self._rxq.get_nowait()
            except queue.Empty:
                return
            self.handle_downlink(can_id, data, is_fd)

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
        print(f"  SYNC max gap: {self.max_sync_gap:.4f}s")
        if self.tx_dropped:
            print(f"  串口背压丢帧: {self.tx_dropped}（反馈是周期性的，丢帧可容忍）")
        print(f"  TX queue: control={len(self._tx_control)} "
              f"feedback={len(self._tx_feedback)}")
        if self.log_f:
            self.log_f.flush()

    def run(self):
        cerb_ids = sorted(i for i, m in self.models.items() if m.kind == "cerb")
        vaxor_ids = sorted(i for i, m in self.models.items() if m.kind == "vaxor")
        print(f"[sim] {self.tty_path} S8/Y{self.data_bitrate_index}/M0/A1/O opened; "
              f"CERB {cerb_ids} "
              f"VAXOR {vaxor_ids}")
        print("[sim] 通道已开、静默监听中（仅做 ACK 节点）。请复位控制板；"
              "收到首个 0x080 SYNC 后开始错峰上报。Ctrl-C 结束。")
        gate_start = time.monotonic()
        last_step = gate_start
        last_bridge = 0.0
        warned = False
        while not self.stop:
            now = time.monotonic()
            self._pump_tx()
            # 串口由 _reader_loop 独占读取；这里只消费队列，不再碰 fd 读。
            self._drain_rx()
            self._pump_tx()
            time.sleep(0.002)          # 保持原有节拍，别忙等
            if not self.sync_seen:
                # 门控：板子上线前绝不发帧（否则无人 ACK 会把 CANable 自己打进
                # bus-off，板子启动后也无人 ACK 其 SYNC，双双死锁）。
                if not warned and now - gate_start > 3.0:
                    print("[sim] 警告：3s 未收到 SYNC，请确认板子已复位且接线正确")
                    warned = True
                continue
            dt = now - last_step
            # 桥的 spin 限到 100Hz（Gazebo 控制器 update_rate 就是 100）：
            # 原先每圈都 spin，而循环周期由 2ms 的 select 决定，等于每秒几百次
            # rclpy 调用，把串口服务挤在后面。
            if self.bridge is not None and now - last_bridge >= 0.01:
                self.bridge.spin()
                last_bridge = now
                self.bridge.publish_commands(self.models)
            for m in self.models.values():
                if self.bridge is not None and self.bridge.mapped(m.id):
                    # Gazebo 模式：该台的反馈取自真实关节角，假 plant 不参与
                    if m.kind == "cerb":
                        m.pos_deg = self.bridge.joint_deg(m.id)
                    else:
                        m.pos_motor_rad = (
                            self.bridge.joint_position(m.id) * VAXOR_GEAR_RATIO
                        )
                        m.vel_motor_rad_s = (
                            self.bridge.joint_velocity(m.id) * VAXOR_GEAR_RATIO
                        )
                else:
                    m.approach(dt)
            last_step = now
            # 错峰：频率由 --fb-period-ms 决定，默认每节点约 16.7Hz。
            for mid in sorted(self.models):
                deadlines = self.next_send.get(mid)
                if deadlines and now >= deadlines[0]:
                    self.emit_one(mid)
                    deadlines.pop(0)
                    if not deadlines:
                        del self.next_send[mid]
            self._pump_tx()
        self.report()
        self.stop = True
        if self._reader is not None:
            self._reader.join(timeout=0.5)
        try:
            os.write(self.fd, b"C\r")  # 收尾关通道
        except OSError:
            pass
        if self.log_f:
            self.log_f.close()
        os.close(self.fd)


def default_canable_tty():
    """按 by-id 找 CANable 的 tty。

    **不要硬编码 /dev/ttyACM1**：USB 重新枚举（拔插、HUB 复位、板子重上电）后
    ACM 编号会交换——2026-09-22 就因为模拟器硬编码 ttyACM1 而开到了别的节点，
    总线上没有 ACK 节点 → 固件 FDCAN1 bus-off → 现象是"总线全死"，查了很久。
    """
    hits = sorted(glob.glob("/dev/serial/by-id/*CANable*"))
    if hits:
        return hits[0]
    print("[sim] 警告：未在 /dev/serial/by-id/ 找到 CANable，回退 /dev/ttyACM1；"
          "若总线无 ACK 节点，先核对设备节点（ACM 编号会交换）。", file=sys.stderr)
    return "/dev/ttyACM1"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tty", default=None,
                    help="默认按 /dev/serial/by-id/ 里含 CANable 的节点解析；"
                         "显式给路径可覆盖")
    ap.add_argument("--cerb", default="1,2,3,4")
    ap.add_argument("--vaxor", default="")
    ap.add_argument("--cerb-r58", default="1",
                    help="自报 R58LITE_V2 身份的 CERB 节点（其余 CERB 节点"
                         "自报 R48LITE_V2；固件按身份串自动绑定型号）")
    ap.add_argument("--safety-off", type=int, nargs="*", default=[],
                    help="把这些 CERB 节点的 0x2650:00 置 1（关闭看门狗+限位保护），"
                         "用于制造与固件记录不一致的场景")
    ap.add_argument("--dir-ccw", type=int, nargs="*", default=[],
                    help="把这些 CERB 节点的 0x2541 置 1（CCW），用于制造方向不一致")
    ap.add_argument("--period-ms", type=int, default=20,
                    help="板端 SYNC 周期，用于换算反馈窗口（默认 20ms）")
    ap.add_argument("--data-bitrate-index", type=int, choices=range(10),
                    default=AUTO_DATA_BITRATE_INDEX,
                    help="SLCAN Y 命令索引；省略时先发 V 自动识别（16e7497→7，b158aa7→5）")
    ap.add_argument("--no-brs", action="store_true",
                    help="FD 帧不切 5M 数据段（全程 1M），用于 BRS 数据相位对照")
    ap.add_argument("--fb-period-ms", type=int, default=60,
                    help="每电机反馈周期（默认 60ms≈16.7Hz；必须小于固件离线阈值）")
    ap.add_argument("--stagger-ms", type=float, default=2.2,
                    help="多节点反馈相位错开间隔")
    ap.add_argument("--log", default=None)
    ap.add_argument("--gazebo", action="store_true",
                    help="把下行电机命令接到 Gazebo；反馈取自 /joint_states")
    ap.add_argument("--gazebo-control-mode", choices=("position", "effort"),
                    default="effort",
                    help="Gazebo 接口：effort（默认，MIT 动力学）或 position（仅通信对照）")
    ap.add_argument("--joint-map",
                    default="1:joint_link1,2:joint_link2,3:joint_link3,"
                            "4:joint_link4",
                    help="电机 id → Gazebo 关节名，逗号分隔")
    ap.add_argument("--position-topic", default="/arm_position_controller/commands")
    ap.add_argument("--effort-topic", default="/arm_effort_controller/commands")
    ap.add_argument("--tau-limit", type=float, default=10.0,
                    help="力矩饱和 N·m；URDF 里关节 effort 限幅是 10.0")
    args = ap.parse_args()
    if args.tty is None:
        args.tty = default_canable_tty()
        print(f"[sim] 设备: {args.tty}")
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

    if args.gazebo:
        jm = {}
        for item in args.joint_map.split(","):
            if item.strip():
                mid, jname = item.split(":")
                jm[int(mid)] = jname.strip()
        sim.bridge = GazeboBridge(jm, args.gazebo_control_mode,
                                  args.position_topic, args.effort_topic,
                                  args.tau_limit)
        topic = (args.position_topic if args.gazebo_control_mode == "position"
                 else args.effort_topic)
        print("[sim] Gazebo 桥已接：%s → %s (%s)" %
              (jm, topic, args.gazebo_control_mode))

    def _sig(_a, _b):
        sim.stop = True
    signal.signal(signal.SIGTERM, _sig)
    signal.signal(signal.SIGINT, _sig)

    sim.configure()
    sim.run()


if __name__ == "__main__":
    main()
