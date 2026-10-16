#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""FDCAN 台架电机模拟器（默认 4 节点：CERB 1=R58、2-4=R48）。

主机可经 CANable2（SLCAN 串口）或 Linux CAN-FD SocketCAN 接口，在控制板物理
电机总线上模拟电机，用于验证「固件 motor runtime ⇄ micro-ROS 话题」全通路。
总线上无真机。SLCAN 仲裁段 1M（S8）、数据段 5M；SocketCAN 位率由系统预先配置。
CANable2 的数据位率索引按 V 查询自动选择；
**两只已知版本（自定义 16e7497-dirty 与上游 b158aa7）的 5Mbps 索引都是 Y5**
（2026-10-12 实测修正了 16e7497-dirty=Y7 的旧说法，判据见 SLCAN_VERSION_DATA_BITRATE）。
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
  CERB  SYNC 0x080（FD+BRS 0B，默认 20ms）；MIT 0x110+id（FD 12B）/多轴 0x210（FD 64B）
  CERB  SDO 请求 0x600+id（FD 8B；40 08 10 00 上传发起 → 60/70 取分段）
  VAXOR 控制 0x502+4*id（经典 8B）；MIT 0x501+4*id（FD 12B）

用法：
  python3 fdcan_motor_sim.py                 # 默认 /dev/ttyACM1，自动 V/S8/Y*/M0/A1/O 开通道
  python3 fdcan_motor_sim.py --tty /dev/ttyACM1 --log /tmp/sim.jsonl
  python3 fdcan_motor_sim.py --interface can0 --gazebo --joint-map 1:joint_link1
  # 默认自动 V 探测选索引；已知两只 CANable 版本都落到 Y5。--data-bitrate-index
  # 只在自动探测失败时手工覆盖（2026-10-12 起没有任何已知版本需要 Y7）。
  # 与 Gazebo effort 控制器做真实 MIT 等效力矩端到端验证：
  python3 fdcan_motor_sim.py --gazebo --gazebo-control-mode effort \
    --joint-map 7:joint_link1,8:joint_link2,9:joint_link3,10:joint_link4
  # 需要真实 effort 动力学时，Gazebo 也必须用 control_mode:=effort：
  python3 fdcan_motor_sim.py --gazebo --gazebo-control-mode effort \
    --effort-topic /arm_effort_controller/commands
节点清单与节奏可调：--cerb/--vaxor（默认 CERB 1-4 / 无 VAXOR）、--period-ms（板端 SYNC
周期）、--fb-period-ms/--stagger-ms（台架降载用的反馈周期/错峰，默认 0 = 照真机每个 SYNC
都回）、--no-brs（数据段不切 5M）。
运行期控制（可选）：--control-socket <path> 开一条 Unix socket JSON 行协议，
支持 status/add/remove/link（加电机、移除、断开/恢复），供 web 上位机仿真页使用；
协议与语义见 micro_ros_agent/README.md。
开场时序要求：先开本脚本（成为 ACK 节点），再复位控制板。Ctrl-C 收尾；SLCAN 关闭
通道，SocketCAN 关闭 raw socket。
"""

from __future__ import annotations

import argparse
from collections import deque
import fcntl
import glob
import json
import os
import queue
import re
import select
import signal
import socket
import struct
import subprocess
import sys
import termios
import threading
import time
import tty

PI = 3.141592653589793
DEFAULT_DATA_BITRATE_INDEX = 5
AUTO_DATA_BITRATE_INDEX = None
# CANable 非标准 E 查询的应答形如 "CANable Error Register: 4"，但**没有 \r 结尾**，
# 会与紧随其后的帧行粘在一起，故按前缀+数字取，不依赖行边界。
_CANABLE_ERR_RE = re.compile(r"CANable Error Register:\s*([0-9]+)")
# 缓冲里未成行时也要认得出 E 应答，故另备字节版（见 _consume_control_reply）。
_CANABLE_ERR_RE_B = re.compile(rb"CANable Error Register:\s*([0-9]+)")
# SLCAN 帧行首字符；不以这些开头的文本是命令应答（V 的版本串、E 的错误码行）。
# r/R 是远程帧，漏了会把帧行当应答文本吃掉（见 _consume_control_reply）。
_SLCAN_FRAME_HEADS = b"tTdDrRbB"
# CERB 的 MIT 力矩量化上限是**逐型号**的（R48 11 N·m / R58 45 N·m），由调用方按
# 电机传入。这个常量只在拿不到型号时兜底——用错会让 τ_ff 被整体缩放。
CERB_MIT_TORQUE_FALLBACK = 45.0
# SDO 中止码：设备是**分层**的——索引不在字典里回"对象不存在"，索引在而子索引没有
# 回"子索引不存在"（2026-10-15 真机 node 1 实测：0x9999:00 → 0x06020000、
# 0x6080:01 → 0x06090011）。一律回后者就复现不出对端的这一层语义。
CERB_SDO_ABORT_NO_OBJECT = 0x06020000
CERB_SDO_ABORT_NO_SUBINDEX = 0x06090011
# 设备对象字典里**存在**的主索引（厂家 PR0002/TN0002 + 2026-10-15 实测）。只用来
# 选上面两个中止码，不表示本模拟器实现了它们——没实现的对象仍按"只有这些子索引"答。
CERB_OBJECT_INDICES = frozenset({
    0x1000,          # 设备类型
    0x1008,          # 设备名（身份串，分段上传）
    0x1009, 0x100A,  # 硬件 / 软件版本
    0x1011,          # 恢复默认参数
    0x1017,          # 心跳时间
    0x1018,          # 厂商识别对象
    0x1800,          # TPDO1 通信参数
    0x2530,          # 设备 ID
    0x2531,          # 设置零位
    0x2538,          # 电机最大电流
    0x2539,          # Flash 保存码
    0x2540,          # CAN 通信速率
    0x2541,          # 方向
    0x2650,          # 看门狗 / 限位保护开关
    0x2654,          # 故障项控制
    0x3100,          # Boot / 主线版本
    0x4500,          # 最近故障诊断 ID
    0x603F,          # 当前错误码
    0x6041,          # 状态字
    0x6060, 0x6061,  # 模式命令 / 模式显示
    0x6064,          # 负载端实际位置
    0x607D,          # 位置限位
    0x6080,          # 电机最大转速
})
# 设备出厂值（对端真值，**不是**固件假设）：0x6080 电机端最大转速 RPM、
# 0x2538 电机端最大电流 mA。2026-10-15 真机 node 1 实测 R58 = 3325/13300、
# R48 = 3325/5000。**0x6080 两台相同**——它是模块侧共用值，不是逐型号额定；
# 真正逐型号的是 0x2538。这两个量在固件侧会换算成关节侧 rad/s 与 N·m，用错会把
# 台架的速度/力矩天花板整体缩放（见 docs/research/cerb-device-reference.md）。
CERB_DEVICE_DEFAULTS = {
    "r48": {"max_speed_rpm": 3325, "max_current_ma": 5000},
    "r58": {"max_speed_rpm": 3325, "max_current_ma": 13300},
}
# 固件生命周期九态里的"运行"（与 can_bridge_msgs/MotorStatusTarget.msg 对齐）。
LIFECYCLE_ACTIVE = 4
SLCAN_VERSION_DATA_BITRATE = {
    # ⚠ 2026-10-12 实测修正：现场这只 CANable（USB 描述符 b158aa7、V 响应
    # 16e7497-dirty）的 5Mbps 索引是 **Y5**，不是 Y7。
    #
    # 判据（同一硬件、同一固件，只换索引做 A/B）：
    #   Y7 → 9 台全部 OFFLINE、fault=5；固件 tx/rx 计数几十秒内冻结；
    #        读 FDCAN1 寄存器得 PSR.DLEC=3（**数据相位** ACK error）——
    #        典型的"仲裁相位对、数据相位位率/采样点不匹配"指纹。
    #   Y5 → 9 台全部 READY_DISABLED、fault=0、online=true，长期稳定。
    #
    # 注意：0x080 SYNC 是 DLC=0 帧、没有数据相位，所以"能收到 SYNC"或
    # "CAN 分析仪 0 错误"**都不能**证明数据位率正确——必须看带数据载荷的
    # FD 帧（DLC 8/12/20）是否畅通。
    #
    # 历史注释曾记该版本 Y7=5Mbps，现证伪（V 串带 -dirty，是改动过的固件，
    # 索引语义已与上游/历史不同）。
    "16e7497": 5,
    "b158aa7": 5,
}

# ---- SLCAN -------------------------------------------------------------

FD_LEN_CODE = {0: "0", 1: "1", 2: "2", 3: "3", 4: "4", 5: "5", 6: "6",
               7: "7", 8: "8", 12: "9", 16: "A", 20: "B", 24: "C",
               32: "D", 48: "E", 64: "F"}
_CODE_LEN = {v: k for k, v in FD_LEN_CODE.items()}

# ---- SocketCAN ---------------------------------------------------------

CAN_MTU = 16
CANFD_MTU = 72
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x7FF
CANFD_BRS = 0x01
CAN_RAW_FD_FRAMES = 5


def pack_socketcan_frame(can_id: int, payload: bytes, is_fd: bool,
                         brs: bool) -> bytes:
    """编码 Linux CAN_RAW 帧；本模拟器只发送标准 11 bit CAN ID。"""
    if not 0 <= can_id <= CAN_SFF_MASK:
        raise ValueError(f"standard CAN ID out of range: 0x{can_id:X}")
    if is_fd:
        if len(payload) not in FD_LEN_CODE:
            raise ValueError(
                f"invalid CAN FD payload length {len(payload)}; "
                f"valid lengths are {sorted(FD_LEN_CODE)}"
            )
        flags = CANFD_BRS if brs else 0
        return struct.pack("=IBBBB64s", can_id, len(payload), flags, 0, 0,
                           payload.ljust(64, bytes(1)))
    if len(payload) > 8:
        raise ValueError("classical CAN payload exceeds 8 bytes")
    return struct.pack("=IBBBB8s", can_id, len(payload), 0, 0, 0,
                       payload.ljust(8, bytes(1)))


def parse_socketcan_frame(frame: bytes):
    """解析 CAN_RAW recv() 数据 -> (id, payload, is_fd)，忽略错误帧。"""
    if len(frame) not in (CAN_MTU, CANFD_MTU):
        return None
    can_id, length, _flags, _res0, _res1 = struct.unpack_from("=IBBBB", frame)
    if can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG | CAN_EFF_FLAG):
        return None
    normalized_id = can_id & CAN_SFF_MASK
    if len(frame) == CAN_MTU:
        if length > 8:
            return None
        return normalized_id, frame[8:8 + length], False
    if length > 64:
        return None
    return normalized_id, frame[8:8 + length], True


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


def parse_joint_map(spec: str) -> dict[int, str]:
    """解析 --joint-map（"id:关节名,id:关节名"）→ {motor_id: 关节名}。

    ⚠ **重复 motor_id 静默取后者**：dict 赋值天然覆盖，不给任何提示。这与
    固件侧"重号进 CONFIG_CONFLICT"完全不同——模拟器这一侧不报，接线错位只会
    表现为"某台电机不跟"，不会报错。测试固化此行为（见
    tools 测试 test_fdcan_motor_sim.py 的 test_joint_map_duplicate_*），
    要改成报警或硬失败请先想清楚会不会打断既有台架命令。
    """
    out: dict[int, str] = {}
    for item in spec.split(","):
        if item.strip():
            mid, jname = item.split(":")
            out[int(mid)] = jname.strip()
    return out


def parse_id_floats(spec: str) -> dict[int, float]:
    """解析 "id:值,id:值" → {motor_id: 值}。空串 → 空字典。

    重复 motor_id 取后者、不报错（同 parse_joint_map 的约定，见那里的理由）。
    """
    out: dict[int, float] = {}
    for item in spec.split(","):
        if item.strip():
            mid, value = item.split(":")
            out[int(mid)] = float(value)
    return out


def parse_tau_limits(spec: str) -> dict[int, float]:
    """解析 --tau-limits（"id:Nm,id:Nm"）→ {motor_id: 关节侧饱和力矩}。

    与 --tau-limit（全局兜底）并存：型号库里写了 peak_torque_nm 的电机走逐台值，
    其余（如 vaxor 未给峰值力矩）退回全局值。
    """
    return parse_id_floats(spec)


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

# CERB 反馈里的线圈温度上报值（℃）。本模拟器不建模热学，报名义值——与 C++
# motor_sim 的 kCerbNominalTemperatureDeciDegC = 250（25.0 ℃）同值，两个后端
# 在页面上必须给出同一个数。
CERB_NOMINAL_TEMPERATURE_C = 25.0

# 启动期开通道命令的串口写重试预算（秒）。运行期数据帧不在收包路径里阻塞，
# 统一进入发送队列，由主循环泵出；否则 SDO 应答遇到 USB CDC 背压会卡住反馈，
# CERB 100ms 心跳门会把所有节点判成 OFFLINE。
TX_RETRY_S = 0.25
TX_QUEUE_MAX = 512
TX_PUMP_BUDGET = 8192
# 控制 socket 单条请求的读取上限（超出的部分截断后按非法 JSON 处理）
CONTROL_MAX_REQUEST_BYTES = 4096


_CLAMP_WARNED: set[str] = set()


def clamp_i16(value: float, what: str) -> int:
    """把值夹进 int16 再交给 struct.pack。

    CERB 反馈帧把位置/转速/Iq/温度都塞进 int16，越界时 struct.pack 抛
    struct.error，**打死整个模拟器进程**——2026-10-13 台架实测：武装后关节
    瞬时摆过 ±180°，pos_deg*182 溢出，模拟器崩溃，launch 连同 MuJoCo 一起
    关掉，台架全下线（固件侧 18 台全 OFFLINE、fault=5，mujoco 桥 exit -11）。
    vaxor 那三个编码函数一直有 clamp，CERB 这条路漏了。

    夹取而非抛错：真机上报的是有符号 16 位量，越界在物理上就是"到量程端点"，
    崩溃不是它的语义。但**首次**越界必须响一声——静默夹取会把"关节真的跑出
    量程"这个建模/限位问题藏起来。
    """
    rounded = int(round(value))
    if -32768 <= rounded <= 32767:
        return rounded
    if what not in _CLAMP_WARNED:
        _CLAMP_WARNED.add(what)
        print(f"[sim] ⚠ {what} 越界 {value:.1f}，已夹取到 int16 端点"
              f"（后续同类不再重复打印）——真实电机到量程端也会这样，"
              f"但若常态出现说明关节确实跑出了 ±180° 量程", file=sys.stderr)
    return -32768 if rounded < 0 else 32767


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


def cerb_decode_command(data: bytes, torque_limit: float = CERB_MIT_TORQUE_FALLBACK):
    """固件下行 CERB MIT（0x110+id，12B）-> dict（关节侧 rad/rad/s/Nm）。

    `torque_limit` 是这台电机的 **MIT 力矩量化上限**（规格书：P/V/T Max 必须与
    电机参数页一致）——**逐型号**：R48 11 N·m、R58 45 N·m。固件编码端用的就是
    Profile 里的 `mit_torque_limit_nm`，这里必须跟着对应，否则 τ_ff 会被缩放
    （历史 bug：写死 45.0 ⇒ R48 的 τ_ff 被缩成 11/45）。
    """
    enabled = (data[0] & 0x80) != 0
    pos = cerb_symmetric((data[1] << 8) | data[2], PI, 16)
    vel = cerb_symmetric(((data[3] & 0xF0) << 4) | data[4], PI, 12)
    kp_raw = ((data[4] & 0x0F) << 8) | data[5]
    kd_raw = (data[6] << 4) | ((data[7] & 0xF0) >> 4)
    tau = cerb_symmetric(((data[7] & 0x0F) << 8) | data[8], torque_limit, 12)
    return {"enabled": enabled, "pos_rad": pos, "vel_rad_s": vel,
            "kp": kp_raw * 500.0 / 4095.0, "kd": kd_raw * 5.0 / 4095.0,
            "tau_nm": tau}


# ---- 节点模型 -----------------------------------------------------------

class MotorModel:
    def __init__(self, motor_id: int, kind: str, gear_ratio: float | None = None,
                 joint_nm_per_amp: float | None = None):
        self.id = motor_id
        self.kind = kind  # "cerb" | "vaxor"
        self.tx_count = 0
        self.rx_count = 0
        self.enabled = False
        self.last_cmd = None
        # 传动与 Kt（关节侧 N·m/A）。反馈里的电流与转速都由它们换算，值来自型号库
        # （launch 逐台传入），模拟器不自带第二个真相源——重复定义正是两边漂移的成因。
        self.gear_ratio = gear_ratio or (VAXOR_GEAR_RATIO if kind == "vaxor" else None)
        self.joint_nm_per_amp = joint_nm_per_amp
        # 本拍**实际施加**给被控对象的关节力矩（N·m）。反馈的 Iq 由它反算——这是
        # 「台架告诉固件的电流」与「假电机真出了多少力」之间唯一的联系。纯台架
        # （无 --gazebo）没有被控对象，恒为 0。
        self.applied_joint_nm = 0.0
        # 关节侧速度（rad/s）。两种模式各有来源：桥模式取真实关节速度，纯台架取
        # 玩具模型一阶趋近的导数。反馈帧按各自协议换算成电机侧量。
        self.vel_joint_rad_s = 0.0
        # 链路开关（运行期控制通道）：False = 完全静默（不发反馈、不回 SDO、
        # 忽略下行帧），对固件等价于设备掉电；恢复时按"重新上电"清命令状态。
        self.linked = True
        if kind == "cerb":
            # 位置保留"每台上电初值递增"，用于在页面上区分读到了哪一台。转速、
            # 电流、温度都**不再**是上电常量：前两个由被控对象状态每拍导出，
            # 温度是名义值（见下）。曾经这里是 rpm=101 / iq_ma=1000 的写死值，
            # 于是不发任何指令时页面也恒有力矩与转速，而真机未使能就是 0。
            self.pos_deg = float(10 * motor_id)
            self.tgt_deg = self.pos_deg
            # 线圈温度：名义值，与 C++ motor_sim 的 kCerbNominalTemperatureDeciDegC
            # 同值。模拟器不建模热学，报一个常量比报一条按 id 递增的假升温曲线诚实。
            self.temp_c = CERB_NOMINAL_TEMPERATURE_C
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
                (0x6080, 0): CERB_DEVICE_DEFAULTS["r48"]["max_speed_rpm"],
                (0x2538, 0): CERB_DEVICE_DEFAULTS["r48"]["max_current_ma"],
                (0x2650, 0x00): 1,
                (0x2541, 0x00): 0,
                (0x6064, 0x00): 0,
            }
        else:
            # VAXOR 反馈为电机轴量；关节初值 0.1*(id-6) rad。速度与电流同 CERB：
            # 状态导出，不设写死初值。
            self.pos_motor_rad = 0.1 * (motor_id - 6) * self.gear_ratio
            self.tgt_motor_rad = self.pos_motor_rad

    def approach(self, dt_s: float):
        # 极简一阶趋近，仅用于演示「命令影响反馈」，不模拟真实动力学。
        rate = 2.0 * dt_s
        if self.kind == "cerb":
            self.pos_deg += (self.tgt_deg - self.pos_deg) * min(1.0, rate)
            # 上报速度取本模型位置律的导数（连续形式），而不是另编一个常量：
            # 否则纯台架会出现「位置在动、转速恒 0」的自相矛盾。
            self.vel_joint_rad_s = (self.tgt_deg - self.pos_deg) * 2.0 * PI / 180.0
        else:
            self.pos_motor_rad += (self.tgt_motor_rad - self.pos_motor_rad) \
                * min(1.0, rate)
            gear = self.gear_ratio or VAXOR_GEAR_RATIO
            self.vel_joint_rad_s = \
                (self.tgt_motor_rad - self.pos_motor_rad) * 2.0 / gear

    def report_current_a(self) -> float:
        """反馈电流（电机侧 A）＝ 实际施加的关节力矩经传动与 Kt 反算。

        与 C++ `motor_sim` 的 `frame_router.cpp` 同式：
        `joint_nm / (Kt × 速比 × 效率)`。未施加的场合（未使能 / 无命令 / 固件未
        授权 / 纯台架无被控对象）`applied_joint_nm` 为 0，于是报 0——真机未使能
        就是没有电流，不发指令不该看到力矩。

        不另设电流上限钳位：力矩已在上游按 `peak_torque_nm` 饱和，电流天然有界；
        编码层另有 `clamp_i16` 兜底。缺 `joint_nm_per_amp`（型号未知，如控制通道
        动态 add 进来的电机）时报 0 而不是猜一个系数。
        """
        if not self.joint_nm_per_amp:
            return 0.0
        return self.applied_joint_nm / self.joint_nm_per_amp

    def report_motor_rpm(self) -> float:
        """反馈转速（电机侧 RPM）：CERB 帧按此编码，固件解码后除速比回关节侧。"""
        if not self.gear_ratio:
            return 0.0
        return self.vel_joint_rad_s * self.gear_ratio * 60.0 / (2.0 * PI)

    def report_motor_rad_s(self) -> float:
        """反馈速度（电机侧 rad/s）：VAXOR 帧按此编码。"""
        if not self.gear_ratio:
            return 0.0
        return self.vel_joint_rad_s * self.gear_ratio

    def cerb_frame(self) -> bytes:
        """12B 周期状态帧（0x300+n）：位置 / 转速 / Iq / 错误码 / 线圈温度 / 模式 / 状态位。

        温度在 data[8..9]（大端 int16，0.1 °C），固件 cerb_adapter 会读它填
        MotorStatusArray.temperature——曾经这里是 6 字节零填充，页面上恒显示
        "--"，看着像 UI 缺字段，实为台架不发。
        """
        data = struct.pack(">hhh", clamp_i16(self.pos_deg * 182.0, "CERB 位置"),
                           clamp_i16(self.report_motor_rpm(), "CERB 转速"),
                           clamp_i16(self.report_current_a() * 1000.0, "CERB Iq"))
        data += struct.pack(">h", 0)                      # data[6..7] 错误代码
        data += struct.pack(">h",
                            clamp_i16(self.temp_c * 10, "CERB 温度"))  # data[8..9]
        # data[10] 控制模式反馈、data[11] 状态位（使能/抱闸/报错/到位）本模拟器
        # 不建模，保持 0；固件目前也只读 data[11] bit5（故障）。bit5=0 → 无故障。
        return data + bytes([0x00, 0x00])

    def vaxor_status_frame(self) -> bytes:
        cmd_id = 0x501 + 4 * self.id
        out = cmd_id.to_bytes(2, "little")
        out += vaxor_pos_code(self.pos_motor_rad)
        out += vaxor_vel_code(self.report_motor_rad_s())
        out += vaxor_cur_code(self.report_current_a())
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
            vaxor_vel_code(self.report_motor_rad_s()) + \
            vaxor_cur_code(self.report_current_a()) + b"\x00\x00"


# ---- 运行期控制（--control-socket） --------------------------------------

# 各族 id 上限：CERB 状态帧 0x300+id 与 SDO 0x580+node 都要求 ≤127；
# VAXOR 命令帧 0x501+4n 到 0x600 之前，n ≤ 63。
SIM_CONTROL_ID_LIMITS = {"cerb": 127, "vaxor": 63}


def feedback_cadence(period_ms: int, fb_period_ms: int, stagger_ms: float):
    """反馈节拍 → (device_sync_feedback, sync_interval, stagger_s, fb_period_s)。

    `fb_period_ms = 0`（默认）是**对端保真**：真机 TPDO 传输类型 = 1，每个 SYNC 立即
    回一帧，不错峰、无墙钟兜底。2026-10-15 单机 R58 实测 50Hz 下 48 发 48 收、延迟约
    124us，无 SYNC 时零帧。`> 0` 是台架降载模式（历史默认 60ms）：按周期出窗口 +
    stagger 错峰，墙钟兜底把该周期变成**上界**（SYNC 慢于假定值时仍按时出窗口）。
    """
    if period_ms <= 0 or fb_period_ms < 0:
        raise ValueError(
            "SYNC period must be positive and feedback period non-negative")
    if fb_period_ms == 0:
        return True, 1, 0.0, float("inf")
    return (False, max(1, round(fb_period_ms / period_ms)),
            stagger_ms / 1000.0, fb_period_ms / 1000.0)


def apply_cerb_device_model(model: MotorModel, name: str) -> None:
    """把型号落到一台 CERB 上：身份串 + 该型号的出厂值。

    身份串是固件绑定型号的依据，0x6080/0x2538 是它回读比对的依据——**必须同源**，
    否则会出现"自报 R58、参数却是 R48"这种真机上不存在的组合。
    """
    defaults = CERB_DEVICE_DEFAULTS[name]
    model.identity = b"R58LITE_V2" if name == "r58" else b"R48LITE_V2"
    model.sdo_params[(0x6080, 0)] = defaults["max_speed_rpm"]
    model.sdo_params[(0x2538, 0)] = defaults["max_current_ma"]


def _control_motor_info(motor_id: int, model: MotorModel,
                        joint_map: dict[int, str] | None) -> dict:
    return {
        "id": motor_id,
        "kind": model.kind,
        "model": ("r58" if model.identity == b"R58LITE_V2" else "r48")
        if model.kind == "cerb" else None,
        "joint": (joint_map or {}).get(motor_id),
        "linked": model.linked,
        "enabled": bool(model.enabled),
        "rx": model.rx_count,
        "tx": model.tx_count,
    }


def apply_sim_control(models: dict[int, MotorModel], command,
                      joint_map: dict[int, str] | None = None,
                      joint_nm_per_amp=None, gear_ratios=None) -> dict:
    """应用一条控制命令，返回响应 dict（纯逻辑，不碰 IO 与调度）。

    命令：{"cmd": "status"}；{"cmd": "add", "id", "kind", "model"?}（model 只对
    cerb 有意义：r48/r58）；{"cmd": "remove", "id"}；{"cmd": "link", "id", "up"}。
    add 的 id 若在 joint_map 里（--gazebo 的拓扑电机）自动挂对应关节；否则是
    只走线协议的裸假节点（反馈取内部假 plant）。

    joint_nm_per_amp / gear_ratios 是型号库的逐台投影（见 CanMotorSim.__init__），
    add 出来的电机在这里挂上，否则它会上报 0 电流、0 转速——不是"没使能"，
    而是压根不知道自己的 Kt 与速比。
    """
    joint_nm_per_amp = joint_nm_per_amp or {}
    gear_ratios = gear_ratios or {}
    if not isinstance(command, dict):
        return {"ok": False, "error": "命令必须是 JSON 对象"}
    action = command.get("cmd")
    if action == "status":
        return {
            "ok": True,
            "motors": [_control_motor_info(mid, m, joint_map)
                       for mid, m in sorted(models.items())],
        }
    if action not in ("add", "remove", "link"):
        return {"ok": False, "error": f"未知命令 {action!r}"}
    motor_id = command.get("id")
    if not isinstance(motor_id, int) or isinstance(motor_id, bool) or motor_id <= 0:
        return {"ok": False, "error": "id 必须是正整数"}
    if action == "add":
        kind = command.get("kind")
        if kind not in SIM_CONTROL_ID_LIMITS:
            return {"ok": False, "error": "kind 必须是 cerb 或 vaxor"}
        if motor_id in models:
            return {"ok": False, "error": f"id {motor_id} 已在模拟集合中"}
        if motor_id > SIM_CONTROL_ID_LIMITS[kind]:
            return {"ok": False, "error":
                    f"{kind} 的 id 上限是 {SIM_CONTROL_ID_LIMITS[kind]}"}
        model_name = command.get("model")
        if kind == "cerb" and model_name not in (None, "r48", "r58"):
            return {"ok": False, "error": "model 必须是 r48 或 r58"}
        model = MotorModel(motor_id, kind,
                           gear_ratio=gear_ratios.get(motor_id),
                           joint_nm_per_amp=joint_nm_per_amp.get(motor_id))
        if kind == "cerb":
            apply_cerb_device_model(model, model_name or "r48")
        models[motor_id] = model
        return {"ok": True, "id": motor_id,
                "motor": _control_motor_info(motor_id, model, joint_map)}
    if motor_id not in models:
        return {"ok": False, "error": f"id {motor_id} 不在模拟集合中"}
    if action == "remove":
        del models[motor_id]
        return {"ok": True, "id": motor_id}
    up = command.get("up")
    if not isinstance(up, bool):
        return {"ok": False, "error": "up 必须是布尔值"}
    model = models[motor_id]
    model.linked = up
    if up:
        # 重新上线 = 重新上电：丢弃旧命令状态，等固件重识别后重新使能
        model.enabled = False
        model.last_cmd = None
    return {"ok": True, "id": motor_id,
            "motor": _control_motor_info(motor_id, model, joint_map)}


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
                 tau_limit, tau_limits=None, publish_interval_s=0.015):
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray
        try:
            from can_bridge_msgs.msg import MotorStatusArray
        except ImportError:
            # 只在 HIL 用得上；纯台架可能没构建电机栈，缺了就跳过固件镜像。
            MotorStatusArray = None
            print("[sim] 未找到 can_bridge_msgs：不镜像固件生命周期——HIL 场合"
                  "应当没装电机栈，请先 source 两个工作区", file=sys.stderr)
        if control_mode not in ("position", "effort"):
            raise ValueError("Gazebo control mode must be position or effort")
        self._rclpy = rclpy
        self._msg_type = Float64MultiArray
        rclpy.init()
        self.node = Node("fdcan_sim_gazebo_bridge")
        self.joint_map = dict(joint_map)          # {motor_id: joint_name}
        # 顺序必须与**订阅端解释命令数组的顺序**一致：上游（simulation.launch.py 的
        # firmware 分支）按 simulation_topology 的电机顺序拼 --joint-map，而被控对象
        # （Gazebo 控制器 / MuJoCo 桥）也按拓扑顺序解释这一串数。
        # 这里曾经是 sorted(set(...))：对 chest_joint1..9 那种名字恰好等于声明顺序，
        # 但 half_body_robot 第二链的名字带点号（joint_link1.1..9.9）时字典序会把 link2 排到
        # link1.1 前面（link1, link1.1, link2, link2.2 …）→ 除首尾关节外力矩全部错位，
        # 而且两边的长度都一样，不会报错。改为保留声明顺序并去重。
        self.joints = list(dict.fromkeys(joint_map.values()))
        self.pos = {j: 0.0 for j in self.joints}
        self.vel = {j: 0.0 for j in self.joints}
        # 当前模型的 ros2_control 初始位置契约是 0 rad。position 模式在
        # 控制器启动稍晚于模型生成时仍要把关节拉回这个已知安全姿态，不能把
        # 启动窗口里因尚未收到命令而坠落的反馈当成新的 hold 目标。
        self.hold_pos = {joint: 0.0 for joint in self.joints}
        self.control_mode = control_mode
        self.tau_limit = tau_limit
        # 逐电机饱和力矩（关节侧 N·m）。**必须逐台**：真机每个型号的峰值差很多
        # （r58 45 Nm / r48 11 Nm），用一个全局小值（历史默认 10 Nm，照抄 URDF
        # 的 effort 限幅）会让大关节出不上力——2026-10-14 台架实测：被推歪之后
        # PD 一路饱和、机构收不回来（表现为"乱动且停不下来"）。真值来自型号库的
        # peak_torque_nm，由 simulation.launch.py 逐台传入。
        self.tau_limits = dict(tau_limits or {})
        # 发力矩的最小间隔。**这是回路的死区**：假电机的"内部环路"本该像真机
        # 一样跑在 kHz 级，而这里只能在 ROS/IPC 之间来回——间隔越大，MIT 律的
        # 相位滞后越大。2026-10-14 实测：跟随 CAN 主循环（50 Hz，20ms）+ 被控
        # 对象 100 Hz 的 /joint_states ≈ 30ms 死区，配置里的 (kp=30, kd=1) 恰好
        # 越过稳定边界（判据 atan(kd·ω/kp) > ω·T，ω≈√(kp/J)），表现为
        # 静止目标下 ±0.3 rad 的极限环。由 --effort-rate-hz 提到 200 Hz 后死区
        # 降到 ~15ms，留出裕度。
        self.publish_interval = publish_interval_s
        self.last_pub = 0.0
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_state, 10)
        # HIL 下**固件是权威**：它撤权（OFFLINE/FAULT/冻结；ARD §36 下不会自动
        # 复权）时，主机侧若还按自己的 enabled 发力矩，被控对象就被一个幽灵控制器
        # 驱动——2026-10-14 实测 MuJoCo 关节点冲到 24 rad/s、4 路顶死在 r48 峰值
        # 钳位，而板子侧 18 台全是"就绪未使能"。只在本话题出现过之后才启用这条
        # 镜像：纯台架没有固件，也就没有该话题，行为不变。
        self.fw_lifecycle = {}
        if MotorStatusArray is not None:
            self.node.create_subscription(MotorStatusArray,
                                          "/micro_ros_up/motor_status_array",
                                          self._on_fw_status, 10)
        topic = position_topic if control_mode == "position" else effort_topic
        self.pub = self.node.create_publisher(Float64MultiArray, topic, 10)

    def _on_fw_status(self, msg):
        self.fw_lifecycle = {s.motor_id: s.lifecycle for s in msg.statuses}

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
        if now - self.last_pub < self.publish_interval:
            return
        self.last_pub = now
        # 本拍**实际施加**的力矩先全部清零，再只给真正下发的那几台赋值——反馈电流
        # 从这里反算（见 MotorModel.report_current_a）。不能只清"这一拍被跳过的"，
        # 未使能 / 无命令 / 固件未授权的电机也在此一并归零：真机在这些状态下就是
        # 没电流。限频提前 return 的那几拍不动它，因为被控对象仍在吃上一次的力矩，
        # 反馈跟着保持才与台架现场一致。
        for m in models.values():
            m.applied_joint_nm = 0.0
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
            # 固件没把这台放在运行态 ⇒ 真机上它就是没力矩（无抱闸，关节会塌）。
            # 主机侧照做：不发这一路，让它保持 0，别用幽灵力矩把机构顶住。
            if self.fw_lifecycle and self.fw_lifecycle.get(mid) != LIFECYCLE_ACTIVE:
                continue
            index = self.joints.index(joint)
            if self.control_mode == "position":
                # 位置模式下发的是目标点，实际力矩由被控对象的位置环决定，本处
                # 读不到 ⇒ 电流按 0 上报。台架固定用 effort 模式，这条只在把
                # sim 当独立位置环用时走到。
                if "pos_rad" in cmd:
                    out.data[index] = cmd["pos_rad"]
                continue
            if "kp" not in cmd:
                continue
            driver = cmd["kp"] * (cmd["pos_rad"] - self.pos[joint])
            damp = cmd["kd"] * (cmd["vel_rad_s"] - self.vel[joint])
            torque = driver + damp + cmd["tau_nm"]
            limit = self.tau_limits.get(mid, self.tau_limit)
            out.data[index] = max(-limit, min(limit, torque))
            m.applied_joint_nm = out.data[index]
        self.pub.publish(out)


class CanMotorSim:
    bridge = None   # 由 main() 在 --gazebo 时注入 GazeboBridge
    # 型号库的逐台投影，由 __init__ 覆盖。类级缺省是给 `__new__` 造出来的测试替身
    # 用的——host 测试用 __new__ 跳过 __init__ 来单测某个方法，apply_control 会把
    # 这两个转交给 apply_sim_control（它自己就把 None 当"没配"）。
    joint_nm_per_amp = None
    gear_ratios = None
    def __init__(self, tty: str | None, cerb_ids, vaxor_ids, period_ms: int,
                 log_path: str | None, use_brs: bool = True,
                 data_bitrate_index: int | None = AUTO_DATA_BITRATE_INDEX,
                 fb_period_ms: int = 0, stagger_ms: float = 2.2,
                 r58_ids=(), safety_off_ids=(), dir_ccw_ids=(),
                 can_interface: str | None = None,
                 setup_can: bool = True,
                 control_socket_path: str | None = None,
                 probe_adapter: bool = False,
                 mit_torque_limits=None,
                 joint_nm_per_amp=None, gear_ratios=None):
        # {motor_id: N·m/A}——**关节侧**「力矩 ÷ 电流」= Kt × 速比 × 效率。
        # 反馈电流由它反算（见 MotorModel.report_current_a），因而必须与真机
        # 型号库同源；launch 逐台传入。查不到的电机报 0 电流而不是猜系数。
        self.joint_nm_per_amp = dict(joint_nm_per_amp or {})
        # {motor_id: 速比}——反馈转速按它换算到电机侧。缺省时 vaxor 退回
        # VAXOR_GEAR_RATIO、cerb 无从可退（报 0 转速）。
        self.gear_ratios = dict(gear_ratios or {})
        # {motor_id: N·m}——**MIT 力矩量化上限**，逐型号（R48 11 / R58 45）。
        # 与桥的 tau_limits 同值同源（型号库 peak_torque_nm），但角色不同：
        # 这个是解码量程，那个是饱和钳位。固件按 Profile 的 mit_torque_limit_nm
        # 编码，解码端必须跟着对应，否则 τ_ff 被整体缩放（曾是写死 45.0）。
        self.mit_torque_limits = dict(mit_torque_limits or {})
        # 静默期是否向适配器问 V/E 取证。**默认关**：诊断不得改变被测对象——
        # 2026-10-12 记录过周期问 E 会让锁存从"偶发"变"90 秒 5 次"，而 2026-10-14
        # 加基线 V 后首次静默由 169s 提前到 0.6s。要取判据时显式开。
        self.probe_adapter = probe_adapter
        self.tty_path = tty
        self.can_interface = can_interface
        self.setup_can = setup_can
        self.bus_socket = None
        self.use_brs = use_brs
        self.data_bitrate_index = data_bitrate_index
        (self.device_sync_feedback, self.feedback_sync_interval,
         self.stagger, self.fb_period) = feedback_cadence(
            period_ms, fb_period_ms, stagger_ms)
        # 下面这段兜底**只在 --fb-period-ms > 0 的降载模式生效**（保真模式
        # fb_period = inf，兜底恒不触发）。
        # 反馈窗口的墙钟周期与兜底（2026-10-13 加）。窗口原本**只**由 SYNC 计数
        # 驱动：每 feedback_sync_interval 个 SYNC 登记一轮 deadline。但 SYNC 是
        # 固件侧事件，其实际节拍可能慢于 --period-ms 假定的 20ms——本台架实测
        # 反馈窗口约 110~120ms（固件侧 rx 160~170 帧/s ÷ 18 台 ≈ 106ms/台，
        # 模拟器侧写入 4931B/s ÷ 604B/窗口 ≈ 8.2 窗口/s ≈ 122ms），即 SYNC 实际
        # ≈40ms 而非 20ms。窗口 = interval×SYNC 被拉长后**越过固件 100ms 离线门**
        # （vaxor_adapter.c / cerb_adapter.c 的 offline_timeout_ms），全台架陷入
        # 3~5Hz 的 OFFLINE→RECONNECTED 循环，每次重连清空 profile → 速度/力矩
        # 大面积 NaN、state_age 抖动到秒级。
        # 兜底：窗口至少每 fb_period_ms 登记一次，与 SYNC 快慢解耦，
        # 使 --fb-period-ms 成为**上界**而不是"SYNC 快才成立"的愿望值。
        self.last_window_at = 0.0
        self.window_count = 0          # 累计反馈窗口数（含 SYNC 驱动与兜底）
        self.sync_window_count = 0     # 其中由 SYNC 驱动的部分
        self.last_stats_at = 0.0
        self._stats_sync_mark = 0      # 体检差分的上一次采样基准
        self._stats_window_mark = 0
        self.tx_dropped = 0   # 发送队列满/串口错误丢帧
        self._tx_error_reported = False
        self._tx_control = deque()   # SDO/应答/管理帧：优先于周期反馈
        self._tx_feedback = deque()  # 周期反馈：队列满时允许丢弃
        self._tx_active = None
        self.models = {i: MotorModel(i, "cerb",
                                     gear_ratio=self.gear_ratios.get(i),
                                     joint_nm_per_amp=self.joint_nm_per_amp.get(i))
                       for i in cerb_ids}
        self.models.update({i: MotorModel(i, "vaxor",
                                          gear_ratio=self.gear_ratios.get(i),
                                          joint_nm_per_amp=self.joint_nm_per_amp.get(i))
                            for i in vaxor_ids})
        for mid in r58_ids:
            if mid in self.models and self.models[mid].kind == "cerb":
                apply_cerb_device_model(self.models[mid], "r58")
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
        # ── 链路健康监测 ─────────────────────────────────────────────
        # 两侧一起静默时根因完全隐形：固件 TX FIFO（16 深）被自动重传占满后不再发
        # SYNC，而模拟器反馈是 SYNC 驱动的、也没活干。这里补两件事：长时间无帧自动
        # 重开通道（自愈；重开前先取一次 E 留判据），读线程猝死自动重启。
        self.canable_error = ""      # 最近一次 E 查询的应答值（仅静默期查）
        # 读线程留下的非帧文本行（V 的版本串等）——静默期没有帧行，不靠队列就只能
        # 靠 fd 直读，那会和读线程抢；故由读线程留痕、主线程消费。
        self._reply_lines = deque(maxlen=4)
        # V 探针的阳性对照是否已取得（configure() 里打基线）。没有它，静默期的
        # "V 无应答"分不清是适配器死了还是探针路径本身不工作。
        self._adapter_baseline = False
        self.last_rx_at = None       # 最近一次收到**有效 CAN 帧**的时刻
        self.stall_recoveries = 0    # 自动重开通道次数
        self._reader_restarts = 0
        self.last_cerb_emit = 0.0
        self.control_socket_path = control_socket_path
        self._control_srv = None
        self.stop = False
        self.log_f = open(log_path, "w") if log_path else None
        self.fd = None
        if can_interface is None:
            if tty is None:
                raise ValueError("SLCAN transport requires a tty")
            self.fd = os.open(tty, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            # 独占锁：两个实例同时开同一个 CANable，会各自下发 SLCAN 配置并互相
            # 打断 → 表现为随机总线错乱、SYNC 间隔异常、某些电机不跟。宁可第二个
            # 实例立刻失败。
            try:
                fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError:
                raise SystemExit(
                    f"{tty} 已被另一个进程占用（很可能是残留的模拟器实例）；"
                    "先停掉它再启动，否则两条 SLCAN 流会互相打断。")

    def _ensure_socketcan_config(self):
        """SocketCAN 的前置条件归本模拟器负责——它是这条口唯一的消费者。

        SLCAN 那条路位时序在**适配器**里，本脚本能无权限自配（`S8/Y5/M0/A1/O`）；
        SocketCAN 的位时序在**内核**里、只有 root 能设，这就是两者的不对称。

        先 `--check` 与标定比对：**符合就一个字节都不碰**（与 udev/networkd 配好的
        情况共存，也不打断已经跑着的控制板）。不符才调 can_up.sh 施加；拿不到权限时
        拒绝启动并打印那一行命令——绝不"打开了就当能用"：2026-10-16 就是 bind 成功、
        打印 opened，然后 40 分钟发不出帧，页面表现为"一台电机都没有"。
        """
        script = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "can_up.sh")
        if not os.path.isfile(script):
            print(f"[sim] 警告：找不到 {script}，跳过 CAN 配置校验", file=sys.stderr)
            return
        check = subprocess.run([script, "--check", self.can_interface],
                               capture_output=True, text=True)
        if check.returncode == 0:
            print(f"[sim] {self.can_interface} 配置符合台架标定（未改动接口）")
            return
        why = (check.stdout + check.stderr).strip() or "与标定不符"
        print(f"[sim] {self.can_interface} 配置不符台架标定（{why}），"
              f"调 can_up.sh 施加…")
        fix = subprocess.run([script, self.can_interface],
                             capture_output=True, text=True)
        if fix.returncode != 0:
            raise RuntimeError(
                f"{self.can_interface} 的位时序/txqueuelen 不符台架标定，自动施加也"
                f"失败：{(fix.stderr or fix.stdout).strip()}；请手动执行 "
                f"{script} {self.can_interface}")
        print(f"[sim] {self.can_interface} 已按标定重配——若控制板此前已在运行，"
              f"这次 down/up 会打断链路，需要复位控制板")

    def configure(self):
        if self.can_interface is not None:
            if self.setup_can:
                self._ensure_socketcan_config()
            protocol = getattr(socket, "CAN_RAW", 1)
            level = getattr(socket, "SOL_CAN_RAW", 101)
            try:
                self.bus_socket = socket.socket(
                    socket.PF_CAN, socket.SOCK_RAW, protocol
                )
                self.bus_socket.setsockopt(
                    level, CAN_RAW_FD_FRAMES, struct.pack("=I", 1)
                )
                self.bus_socket.bind((self.can_interface,))
                self.bus_socket.setblocking(False)
            except OSError as error:
                if self.bus_socket is not None:
                    self.bus_socket.close()
                self.bus_socket = None
                raise RuntimeError(
                    f"cannot open CAN FD SocketCAN interface "
                    f"'{self.can_interface}': {error}; check that it exists, is UP, "
                    "and has CAN-FD bitrates configured"
                ) from error
            self._reader = threading.Thread(target=self._reader_loop, daemon=True)
            self._reader.start()
            return

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
        # 基线：此刻总线健康，走**读线程那条路**再问一次 V。它同时验证探针路径
        # 本身可用——静默期的 "V 无应答" 只有在这条基线存在时才是判据。
        if self.probe_adapter:
            baseline = self._probe_adapter_alive()
            self._adapter_baseline = baseline is not None
            print(f"[sim] CANable V 基线={baseline!r}" if baseline is not None else
                  "[sim] ⚠ CANable V 基线未取到（探针路径不工作，静默期的无应答"
                  "不可作判据）")

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
        """非阻塞发送 CAN 帧；控制响应优先于可丢弃的周期反馈。"""
        if self.bus_socket is not None:
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
                    written = self.bus_socket.send(item["line"])
                except BlockingIOError:
                    return
                except OSError:
                    self.tx_dropped += 1
                    if not self._tx_error_reported:
                        print(
                            f"[sim] SocketCAN send on {self.can_interface} failed; "
                            "check interface state and CAN-FD configuration",
                            file=sys.stderr,
                        )
                        self._tx_error_reported = True
                    self._tx_active = None
                    return
                if written != len(item["line"]):
                    self.tx_dropped += 1
                    self._tx_active = None
                    continue
                self._record_tx(item)
                self._tx_active = None
                budget -= written
            return

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
            self._record_tx(item)
            self._tx_active = None

    def _record_tx(self, item) -> None:
        model = item["model"]
        if model:
            model.tx_count += 1
        if self.log_f:
            self.log_f.write(json.dumps({
                "t": round(time.monotonic(), 4), "dir": "tx",
                "tag": item["tag"], "id": item["can_id"],
                "data": item["payload"].hex()}) + "\n")

    def send(self, can_id: int, payload: bytes, is_fd: bool = True,
             brs: bool = True, model: "MotorModel | None" = None,
             tag: str = ""):
        frame = (
            pack_socketcan_frame(can_id, payload, is_fd, brs)
            if self.bus_socket is not None
            else slcan_line(can_id, payload, is_fd, brs)
        )
        # 周期反馈可丢；SDO/控制回显不能被反馈队列长期挤压。
        queue = (self._tx_feedback if tag in ("cerb_fb", "vaxor_status")
                 else self._tx_control)
        if len(queue) >= TX_QUEUE_MAX:
            self.tx_dropped += 1
            return
        queue.append({"line": frame, "offset": 0, "model": model,
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
                and self.models[node].kind == "cerb" \
                and self.models[node].linked:
            return self.models[node]
        for base, kind in ((0x501, "vaxor"), (0x502, "vaxor")):
            delta = can_id - base
            if delta >= 4 and delta % 4 == 0 and (delta // 4) in self.models \
                    and self.models[delta // 4].kind == kind \
                    and self.models[delta // 4].linked:
                return self.models[delta // 4]
        return None

    def send_sdo_abort(self, node: int, data: bytes, index: int,
                       model) -> None:
        """回一条 SDO 中止，**码分层**：索引不在字典 → 对象不存在；索引在而子索引
        没有 → 子索引不存在（2026-10-15 真机实测）。"""
        code = (CERB_SDO_ABORT_NO_SUBINDEX if index in CERB_OBJECT_INDICES
                else CERB_SDO_ABORT_NO_OBJECT)
        self.send(0x580 + node,
                  bytes([0x80, data[1], data[2], data[3],
                         code & 0xFF, (code >> 8) & 0xFF,
                         (code >> 16) & 0xFF, (code >> 24) & 0xFF]),
                  True, self.use_brs, model, "sdo_abort")

    def handle_sdo(self, node: int, data: bytes) -> None:
        """SDO 从站：身份串（0x1008:00 分段上传）+ 易失管理对象（快速读/写）。

        厂家属具书 §4.3：上传发起 40 08 10 00 → 应答 41 08 10 00 + 总字节数；
        随后 60/70 交替取段 → 应答 000 t nnn c + 7 字节数据。
        """
        m = self.models.get(node)
        if m is None or m.kind != "cerb" or not m.linked:
            return
        m.rx_count += 1
        command = data[0]
        index = data[1] | (data[2] << 8)
        sub = data[3]
        key = (index, sub)
        if command == 0x23:
            # 4 字节快速写：存储并回写成功 ack（0x60），无此对象则 abort。
            if key in m.sdo_params:
                m.sdo_params[key] = int.from_bytes(data[4:8], "little",
                                                   signed=True)
                self.send(0x580 + node,
                          bytes([0x60, data[1], data[2], data[3], 0, 0, 0, 0]),
                          True, self.use_brs, m, "sdo_write_ack")
            else:
                self.send_sdo_abort(node, data, index, m)
            return
        if command == 0x40:
            if key in m.sdo_params:
                # 快速上传（0x43）+ 当前值（小端有符号 32 位）。
                raw = int(m.sdo_params[key]).to_bytes(4, "little", signed=True)
                self.send(0x580 + node,
                          bytes([0x43, data[1], data[2], data[3]]) + raw,
                          True, self.use_brs, m, "sdo_read")
                return
            # 身份串只挂在 :00（真机 :01/:02 回"子索引不存在"）。
            if index != 0x1008 or sub != 0x00:
                self.send_sdo_abort(node, data, index, m)
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
                self.register_feedback_window(now)
                self.sync_window_count += 1
            return
        if can_id == 0x210:
            if not is_fd or len(data) != 64:
                return
            for slot in range(6):
                node = data[56 + slot]
                m = self.models.get(node)
                if m is None or m.kind != "cerb" or not m.linked:
                    continue
                subframe = data[slot * 9:slot * 9 + 9]
                cmd = cerb_decode_command(
                    subframe, self.mit_torque_limits.get(
                        m.id, CERB_MIT_TORQUE_FALLBACK))
                m.enabled = cmd["enabled"]
                m.last_cmd = cmd
                if cmd["enabled"]:
                    m.tgt_deg = cmd["pos_rad"] * 180.0 / PI
                m.rx_count += 1
            self.log_rx(can_id, data, "cerb_multi_mit")
            return
        if 0x601 <= can_id <= 0x67F and len(data) >= 8:  # SDO 请求 0x600+node
            self.handle_sdo(can_id - 0x600, data)
            return
        m = self._model_for_downlink(can_id)
        if m is None:
            return
        if m.kind == "cerb" and can_id == 0x110 + m.id and len(data) >= 9:
            cmd = cerb_decode_command(
                data, self.mit_torque_limits.get(m.id, CERB_MIT_TORQUE_FALLBACK))
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

    def register_feedback_window(self, now: float) -> None:
        """登记一轮反馈窗口（按 stagger 错峰），并记账。"""
        schedule_feedback_window(
            self.next_send,
            sorted(mid for mid, m in self.models.items() if m.linked),
            now, self.stagger)
        self.last_window_at = now
        self.window_count += 1

    def maybe_fallback_window(self, now: float) -> bool:
        """墙钟兜底：距上次窗口已达 fb_period 就补一轮，返回是否补了。

        窗口原本**只**由 SYNC 计数驱动，周期 = interval × **实际** SYNC 周期；
        而 SYNC 是固件侧事件，实际节拍可能远慢于 --period-ms 假定值（本台架
        2026-10-13 实测仅 12~24Hz），窗口被拉长到 3×SYNC ≈ 120ms，越过固件
        100ms 离线门 → 全台架 OFFLINE↔重连抖动 → 重连清空 profile → NaN。
        兜底让 --fb-period-ms 成为反馈周期的**上界**，与 SYNC 快慢解耦。
        """
        if now - self.last_window_at < self.fb_period:
            return False
        self.register_feedback_window(now)
        return True

    def emit_one(self, mid: int):
        m = self.models[mid]
        if not m.linked:
            return
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
        """独立读取线程：尽快读取物理总线，并把帧交给主循环处理。"""
        if self.bus_socket is not None:
            while not self.stop:
                try:
                    readable, _, _ = select.select([self.bus_socket], [], [], 0.05)
                except (OSError, ValueError):
                    return
                if not readable:
                    continue
                try:
                    frame = self.bus_socket.recv(CANFD_MTU)
                except BlockingIOError:
                    continue
                except OSError:
                    return
                parsed = parse_socketcan_frame(frame)
                if parsed:
                    self._rxq.put(parsed)
            return

        # SLCAN 串口接收与 rclpy/Gazebo 桥隔离，避免串口压力令 CANable 停发 ACK。
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
                self._classify_line(line)
            self._consume_control_reply()

    def _classify_line(self, line: bytes):
        """一条以 \\r 收尾的输入：CAN 帧入队，命令应答（V/E）留痕。

        命令应答不是 CAN 帧，parse_slcan 会返回 None——此前被静默丢掉，那正是
        本工具长期看不见适配器状态的原因。
        """
        text = line.decode(errors="replace").strip()
        match = _CANABLE_ERR_RE.search(text)
        if match:
            self.canable_error = match.group(1)
            return
        if "CANable Error Register" in text:
            return
        parsed = parse_slcan(text)
        if parsed:
            self._rxq.put(parsed)
        elif text:
            self._reply_lines.append(text)

    def _consume_control_reply(self):
        """取出缓冲里**未成行**的命令应答。

        ⚠ V/E 的应答都不以 \\r 结尾（2026-10-12 实测：E 应答会与下一条帧粘成
        一行）。正常有帧流时下一条帧的 \\r 会把它们顶出来；但**静默期正是要问
        V/E 的时候**，没有帧流 ⇒ 应答会一直留在 rx_buf 里读不到。判据：帧行必
        以 t/T/d/D/b/B 开头，否则就是应答文本。
        """
        if not self.rx_buf:
            return
        match = _CANABLE_ERR_RE_B.search(self.rx_buf)
        if match:
            self.canable_error = match.group(1).decode()
            self.rx_buf = self.rx_buf[match.end():]
            return
        if self.rx_buf[:1] and self.rx_buf[:1] not in _SLCAN_FRAME_HEADS:
            text = self.rx_buf.decode(errors="replace").strip()
            if text:
                self._reply_lines.append(text)
            self.rx_buf = b""

    def _drain_rx(self):
        """主循环侧：把读线程已入队的帧全部处理掉。"""
        while True:
            try:
                can_id, data, is_fd = self._rxq.get_nowait()
                self.last_rx_at = time.monotonic()
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
            print(f"  发送背压丢帧: {self.tx_dropped}（周期反馈可丢，控制响应应检查总线负载）")
        print(f"  TX queue: control={len(self._tx_control)} "
              f"feedback={len(self._tx_feedback)}")
        if self.log_f:
            self.log_f.flush()

    def _probe_adapter_alive(self, timeout: float = 0.6) -> str | None:
        """问一次 V（版本）判断适配器**串口侧**是否还活着；无应答返回 None。

        V 有阳性对照：开通道时 _probe_version 用同一条命令读到过版本串，且
        configure() 会把基线打进日志。所以"静默期问 V 无应答"只可能是适配器真的
        哑了，不是这条命令不被支持——这正是 E 做不到的（E 是非标准命令，且曾有
        周期轮询加剧锁存的观察，见 __init__ 的健康监测注释）。
        """
        self._reply_lines.clear()
        try:
            self._write(b"V\r", drop_ok=True)
        except OSError:
            return None
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._reply_lines:
                return self._reply_lines[-1]
            time.sleep(0.01)
        return None

    def _query_canable_error(self, timeout: float = 0.5) -> str | None:
        """问一次 CANable 的 E（错误寄存器）；无应答返回 None。

        E 读的是适配器**CAN 控制器**的错误态，与 V 的串口侧存活是两件事——只有
        V 答得出来时这个读数才有意义。非标准命令，只在静默期问。
        """
        self.canable_error = ""
        try:
            self._write(b"E\r", drop_ok=True)
        except OSError:
            return None
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.canable_error:
                return self.canable_error
            time.sleep(0.01)
        return None

    def _recover_latched_bus(self):
        """长时间收不到帧：先取适配器判据，再重下发整条 SLCAN 配置序列。

        实测（2026-10-12）：单独 C（关通道）清不掉；必须重走
        C→S8→Y<n>→M0→A1→O。重开后固件**不用复位**就能把被重传占住的 TX FIFO
        排空，SYNC 自动回到 50Hz。

        适配器是否真的"锁存"是**未决**问题（2026-10-14 的数据相位结论与之冲突）。
        加了 --probe-adapter 时，重开前先问 V/E 留判据：V 无应答 ⇒ 适配器串口侧已
        死（锁存说成立）；V 有应答而 E≠0 ⇒ 适配器活着但它的 CAN 控制器在错误态。
        默认**不问**——诊断不应改变被测对象，理由见 __init__ 的 probe_adapter。
        """
        self.stall_recoveries += 1
        if not self.probe_adapter:
            print(f"[sim] ⚠ 总线静默 >3s（第 {self.stall_recoveries} 次）：重开通道…"
                  "（加 --probe-adapter 可在重开前取适配器 V/E 判据）")
            self._reopen_channel()
            return
        alive = self._probe_adapter_alive()
        if alive is None:
            # 基线由 configure() 打印；无基线时（老进程/异常启动）不下结论。
            verdict = ("无应答 ⇒ 适配器串口侧已死" if self._adapter_baseline
                       else "无应答（无基线，不能作判据）")
            error_text = ""
        else:
            error = self._query_canable_error()
            verdict = f"应答={alive!r} ⇒ 适配器串口侧存活"
            error_text = f"；E={error if error is not None else '无应答'}"
        print(f"[sim] ⚠ 总线静默 >3s（第 {self.stall_recoveries} 次）：V={verdict}"
              f"{error_text}；重开通道…")
        self._reopen_channel()

    def _reopen_channel(self):
        """关通道再整条重配；成功后重新走 SYNC 门控。"""
        try:
            for cmd in slcan_config_commands(self.data_bitrate_index):
                self._write(cmd, drop_ok=True)
                time.sleep(0.15)
        except OSError as error:
            print(f"[sim] ✗ 重开通道失败：{error}")
            return
        self.rx_buf = b""
        self.last_rx_at = None
        self.sync_seen = False      # 重新走门控，等板子的下一个 SYNC
        print("[sim] 通道已重开，等待 SYNC 恢复")

    # ---- 运行期控制 socket ---------------------------------------------

    def _setup_control_socket(self):
        """开 Unix socket 控制通道（0600）。路径被真正占用时报错退出。

        残留的**死** socket 文件（异常退出留下的）直接清掉复用；但如果有人
        在监听，说明另一个模拟器实例活着——两个实例会互相打断总线，宁可拒绝。
        """
        path = self.control_socket_path
        if not path:
            return
        if os.path.exists(path):
            probe = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            try:
                probe.settimeout(0.3)
                probe.connect(path)
            except OSError:
                os.unlink(path)
            else:
                raise SystemExit(
                    f"控制 socket {path} 已在监听（另一个模拟器实例？）；"
                    "先停掉它，否则两个实例会互相打断总线。")
            finally:
                probe.close()
        server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        server.bind(path)
        os.chmod(path, 0o600)
        server.listen(4)
        server.setblocking(False)
        self._control_srv = server
        print(f"[sim] 控制通道就绪: {path}")

    def _teardown_control_socket(self):
        if self._control_srv is not None:
            self._control_srv.close()
            self._control_srv = None
        if self.control_socket_path:
            try:
                os.unlink(self.control_socket_path)
            except OSError:
                pass

    def apply_control(self, command: dict) -> dict:
        """控制命令入口：应用后清掉被撤销电机的待发调度。"""
        joint_map = self.bridge.joint_map if self.bridge is not None else None
        response = apply_sim_control(self.models, command, joint_map,
                                     self.joint_nm_per_amp, self.gear_ratios)
        if response.get("ok") and command.get("cmd") in ("remove", "link"):
            motor_id = response.get("id")
            if isinstance(motor_id, int):
                # 未链接/已移除电机的残留 deadline 一并清掉，避免恢复后
                # 补发一串旧窗口的反馈。
                self.next_send.pop(motor_id, None)
        return response

    def _service_control_socket(self):
        """每轮主循环至多处理一条命令（非阻塞 accept + 短超时读写）。"""
        if self._control_srv is None:
            return
        try:
            conn, _ = self._control_srv.accept()
        except (BlockingIOError, OSError):
            return
        with conn:
            conn.settimeout(0.2)
            data = b""
            try:
                while b"\n" not in data and len(data) < CONTROL_MAX_REQUEST_BYTES:
                    chunk = conn.recv(4096)
                    if not chunk:
                        break
                    data += chunk
            except OSError:
                return
            line = data.split(b"\n", 1)[0]
            try:
                response = self.apply_control(json.loads(line.decode("utf-8")))
            except (ValueError, UnicodeDecodeError):
                response = {"ok": False, "error": "请求不是合法 JSON"}
            try:
                conn.sendall(json.dumps(response, ensure_ascii=False)
                             .encode("utf-8") + b"\n")
            except OSError:
                pass

    def run(self):
        self._setup_control_socket()
        cerb_ids = sorted(i for i, m in self.models.items() if m.kind == "cerb")
        vaxor_ids = sorted(i for i, m in self.models.items() if m.kind == "vaxor")
        if self.bus_socket is not None:
            print(f"[sim] {self.can_interface} SocketCAN CAN-FD opened; "
                  f"CERB {cerb_ids} VAXOR {vaxor_ids}")
        else:
            print(f"[sim] {self.tty_path} S8/Y{self.data_bitrate_index}/M0/A1/O opened; "
                  f"CERB {cerb_ids} VAXOR {vaxor_ids}")
        print("[sim] 总线已开、静默监听中（电机响应暂不发送）。请复位控制板；"
              "收到首个 0x080 SYNC 后开始错峰上报。Ctrl-C 结束。")
        gate_start = time.monotonic()
        last_step = gate_start
        last_bridge = 0.0
        warned = False
        self.last_stats_at = gate_start   # 体检从门控起点开始计
        while not self.stop:
            now = time.monotonic()
            self._pump_tx()
            # 总线由 _reader_loop 独占读取；这里只消费队列。
            self._drain_rx()
            self._service_control_socket()
            self._pump_tx()
            # ── 健康巡检：只跑 SLCAN、且没有半条 TX 行在飞时（避免把帧行截断）──
            if self.bus_socket is None and self._tx_active is None:
                # ⚠ 2026-10-12 起**停用**周期性 E 轮询：E 是非标准命令，现场
                # 这只 -dirty CANable 疑似处理它有副作用（加轮询后锁存频率从
                # "偶发"变成"90 秒 5 次"）。E 只在需要时手工查，不自动发。
                if (self.last_rx_at is not None
                        and now - self.last_rx_at > 3.0):
                    self._recover_latched_bus()
            # 读线程遇 OSError 会直接 return（旧代码无人发现，模拟器"看着活、
            # 其实聋了"）——这里发现即重启。
            if (self.bus_socket is None and self._reader is not None
                    and not self._reader.is_alive() and not self.stop):
                self._reader_restarts += 1
                print(f"[sim] ⚠ 串口读线程已退出，重启（第 {self._reader_restarts} 次）")
                self._reader = threading.Thread(
                    target=self._reader_loop, daemon=True)
                self._reader.start()
            time.sleep(0.002)          # 保持原有节拍，别忙等
            if not self.sync_seen:
                # 门控：板子上线前不发电机反馈帧，避免启动次序错误时总线没有
                # ACK 节点而进入 bus-off；收到 SYNC 代表控制板已进入总线。
                if not warned and now - gate_start > 3.0:
                    print("[sim] 警告：3s 未收到 SYNC，请确认板子已复位且接线正确")
                    warned = True
                continue
            dt = now - last_step
            # 桥的 spin 限到 100Hz（Gazebo 控制器 update_rate 就是 100），让 CAN
            # 收发在每轮主循环里优先完成。
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
                        # VAXOR 反馈为电机轴量，按本台自己的速比换算
                        m.pos_motor_rad = (self.bridge.joint_position(m.id)
                                           * (m.gear_ratio or VAXOR_GEAR_RATIO))
                    # 速度同样取自被控对象。两个 kind 都要设：CERB 反馈里的转速
                    # 是电机侧量（report_motor_rpm 里再乘速比），缺了它这一路恒 0。
                    m.vel_joint_rad_s = self.bridge.joint_velocity(m.id)
                else:
                    m.approach(dt)
            last_step = now
            # 墙钟兜底（理由见 maybe_fallback_window）：SYNC 驱动的窗口若迟迟
            # 不来，这里按 fb_period 补一轮，保证反馈周期不超过 --fb-period-ms。
            self.maybe_fallback_window(now)
            # 错峰：周期由 --fb-period-ms 决定（默认每节点约 16.7Hz）。
            for mid in sorted(self.models):
                deadlines = self.next_send.get(mid)
                if deadlines and now >= deadlines[0]:
                    self.emit_one(mid)
                    deadlines.pop(0)
                    if not deadlines:
                        del self.next_send[mid]
            # 周期体检（每 5s 一行）：把链路健康从"退出时才打印"变成在线可见。
            # SYNC 实际节拍 = sync_count 差分，窗口里 SYNC 驱动占比 = sync_window，
            # 两者一起就能分辨「窗口被 SYNC 拉长」与「发送队列丢帧」。
            if now - self.last_stats_at >= 5.0:
                sync_hz = (self.sync_count - self._stats_sync_mark) / (now - self.last_stats_at)
                win_hz = (self.window_count - self._stats_window_mark) / (now - self.last_stats_at)
                print(f"[sim] 体检 SYNC={sync_hz:.1f}Hz 窗口={win_hz:.2f}/s "
                      f"(SYNC驱动 {self.sync_window_count}/{self.window_count}) "
                      f"错峰跨度={(len(self.models) - 1) * self.stagger * 1000:.1f}ms "
                      f"tx_dropped={self.tx_dropped} "
                      f"queue=ctrl{len(self._tx_control)}/fb{len(self._tx_feedback)} "
                      f"SYNC最大间隔={self.max_sync_gap * 1000:.0f}ms")
                self.last_stats_at = now
                self._stats_sync_mark = self.sync_count
                self._stats_window_mark = self.window_count
            self._pump_tx()
        self._teardown_control_socket()
        self.report()
        self.stop = True
        if self._reader is not None:
            self._reader.join(timeout=0.5)
        if self.bus_socket is not None:
            self.bus_socket.close()
            self.bus_socket = None
        else:
            try:
                os.write(self.fd, b"C\r")  # 收尾关通道
            except OSError:
                pass
            os.close(self.fd)
        if self.log_f:
            self.log_f.close()


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
    endpoint = ap.add_mutually_exclusive_group()
    endpoint.add_argument("--tty", default=None,
                          help="SLCAN 设备；默认按 /dev/serial/by-id/ 解析 CANable")
    endpoint.add_argument("--interface", default=None,
                          help="CAN-FD SocketCAN 接口，例如 can0")
    ap.add_argument("--no-can-setup", action="store_true",
                    help="不校验/施加 SocketCAN 位时序——给接口由 udev/networkd 等"
                         "外部机制管理的情况（正常路径下 --check 通过就不会动手）")
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
    ap.add_argument("--fb-period-ms", type=int, default=0,
                    help="台架降载用的每电机反馈周期；0（默认）= 照真机每个 SYNC 都回"
                         "（真机 TPDO 传输类型=1）。>0 时按该周期出窗口并带墙钟兜底，"
                         "必须小于固件离线阈值（100ms）")
    ap.add_argument("--stagger-ms", type=float, default=2.2,
                    help="多节点反馈相位错开间隔；仅 --fb-period-ms > 0 时生效"
                         "（真机不错峰，回帧在 SYNC 后约 124us）")
    ap.add_argument("--log", default=None)
    ap.add_argument("--ready-file", default=None,
                    help="CAN 接口打开且读线程就绪后创建该文件，供总启动器排序")
    ap.add_argument("--probe-adapter", action="store_true",
                    help="总线静默时向适配器问 V/E 取证（默认关：问 E 在本台架有"
                         "加剧静默的记录，诊断不应改变被测对象）")
    ap.add_argument("--control-socket", default=None,
                    help="运行期控制通道：Unix socket 路径（JSON 行协议，"
                         "status/add/remove/link），供 web 上位机的仿真页使用；"
                         "省略 = 不开控制通道")
    ap.add_argument("--effort-rate-hz", type=float, default=200.0,
                    help="假电机发力矩的频率（Hz）；它是被控对象回路的执行器更新率，"
                         "不是 CAN 节拍。0 = 关（退回随 CAN 主循环，约 50 Hz）。"
                         "200 Hz 是为了把 MIT 律的死区压到 ~15ms：50 Hz 时配置的 "
                         "(kp=30, kd=1) 会越过稳定边界、静止目标下也振荡")
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
                    help="未被 --tau-limits 覆盖的电机用的力矩饱和兜底 N·m；"
                         "10.0 是照抄 URDF 关节 effort 限幅的历史值，只对 Gazebo "
                         "ros2_control 那条路成立")
    ap.add_argument("--tau-limits", default="",
                    help="逐电机力矩饱和 N·m（关节侧），形如 1:11.0,3:45.0；"
                         "取值来自型号库 peak_torque_nm（真机峰值），"
                         "未列出的电机退回 --tau-limit")
    ap.add_argument("--joint-nm-per-amp", default="",
                    help="逐电机关节侧 Kt×速比×效率（N·m/A），形如 1:8.686,7:1.365；"
                         "反馈电流由「实际施加力矩 ÷ 它」反算。取值来自型号库，"
                         "未列出的电机上报 0 电流")
    ap.add_argument("--gear", default="",
                    help="逐电机速比，形如 1:101,7:50；反馈转速按它换算到电机侧。"
                         "未列出的 vaxor 退回内置 50，cerb 上报 0 转速")
    args = ap.parse_args()
    if args.interface is None and args.tty is None:
        args.tty = default_canable_tty()
        print(f"[sim] 设备: {args.tty}")
    cerb_ids = [int(x) for x in args.cerb.split(",") if x]
    vaxor_ids = [int(x) for x in args.vaxor.split(",") if x]
    r58_ids = [int(x) for x in args.cerb_r58.split(",") if x]
    # 必须在构造之前解析：CanMotorSim 的 mit_torque_limits 要用它。放进 --gazebo
    # 分支里赋值会让这个名字在分支外成为未绑定局部名（构造处 UnboundLocalError，
    # 任何参数组合都起不来）。
    tau_limits = parse_tau_limits(args.tau_limits)
    joint_nm_per_amp = parse_id_floats(args.joint_nm_per_amp)
    gear_ratios = parse_id_floats(args.gear)
    sim = CanMotorSim(args.tty, cerb_ids, vaxor_ids, args.period_ms, args.log,
                   use_brs=not args.no_brs,
                   data_bitrate_index=args.data_bitrate_index,
                   fb_period_ms=args.fb_period_ms,
                   stagger_ms=args.stagger_ms,
                   r58_ids=r58_ids,
                   safety_off_ids=args.safety_off,
                   dir_ccw_ids=args.dir_ccw,
                   can_interface=args.interface,
                   setup_can=not args.no_can_setup,
                   control_socket_path=args.control_socket,
                   probe_adapter=args.probe_adapter,
                   mit_torque_limits=tau_limits,
                   joint_nm_per_amp=joint_nm_per_amp,
                   gear_ratios=gear_ratios)

    if args.gazebo:
        jm = parse_joint_map(args.joint_map)
        effort_rate = args.effort_rate_hz if args.effort_rate_hz > 0 else 0.0
        sim.bridge = GazeboBridge(jm, args.gazebo_control_mode,
                                  args.position_topic, args.effort_topic,
                                  args.tau_limit, tau_limits,
                                  publish_interval_s=(1.0 / effort_rate
                                                      if effort_rate else 0.015))
        topic = (args.position_topic if args.gazebo_control_mode == "position"
                 else args.effort_topic)
        print("[sim] Gazebo 桥已接：%s → %s (%s)；力矩饱和：%s（兜底 %.1f N·m）；"
              "发力矩 %s" %
              (jm, topic, args.gazebo_control_mode,
               ", ".join(f"{k}:{v:g}" for k, v in sorted(tau_limits.items())) or "无",
               args.tau_limit,
               f"{effort_rate:g} Hz（独立线程）" if effort_rate else "随 CAN 主循环"))
        print("[sim] 反馈换算：Kt×速比×效率 %s；速比 %s" %
              (", ".join(f"{k}:{v:g}" for k, v in sorted(joint_nm_per_amp.items()))
               or "无（反馈电流恒 0）",
               ", ".join(f"{k}:{v:g}" for k, v in sorted(gear_ratios.items()))
               or "无（vaxor 用内置 50，cerb 转速恒 0）"))
        # 缺 --joint-nm-per-amp 的后果是"页面显示 0 A，胳膊照样动"——比不模拟更误导，
        # 且与本次修复的初衷（不发指令才该是 0）恰好反向。宁可吵一句。
        missing = sorted(set(jm) - set(joint_nm_per_amp))
        if missing:
            print("[sim] 警告：电机 %s 没给 --joint-nm-per-amp，其反馈电流将恒报 0，"
                  "即使正在出力。取值 = 型号库的 Kt×速比×效率（motor.yaml）；"
                  "simulation.launch.py 已按拓扑自动生成。"
                  % ",".join(str(i) for i in missing))
        # MIT 律跑在独立线程里，与 CAN 主循环（SYNC 20ms）解耦：主循环里那份
        # 调用留着做兜底，但会被 publish_interval 节流掉，实际生效的是本线程。
        if effort_rate:
            def _effort_loop():
                while not sim.stop:
                    sim.bridge.publish_commands(sim.models)
                    time.sleep(1.0 / effort_rate * 0.5)
            threading.Thread(target=_effort_loop, name="effort-publish",
                             daemon=True).start()

    def _sig(_a, _b):
        sim.stop = True
    signal.signal(signal.SIGTERM, _sig)
    signal.signal(signal.SIGINT, _sig)

    sim.configure()
    if args.ready_file:
        ready_parent = os.path.dirname(os.path.abspath(args.ready_file))
        os.makedirs(ready_parent, exist_ok=True)
        with open(args.ready_file, "w", encoding="utf-8") as ready_handle:
            ready_handle.write(f"{os.getpid()}\n")
            ready_handle.flush()
            os.fsync(ready_handle.fileno())
        print(f"[sim] CAN 接口已就绪: {args.ready_file}")
    sim.run()


if __name__ == "__main__":
    main()
