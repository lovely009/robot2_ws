"""电机调试共享模块：slcan 帧收发与 RP / vaxor（25/30mm）编解码。

纯函数，无 curses、无硬件依赖，可直接单测。vaxor 部分从
verify_2530_serial.py 迁入并保持数值一致；RP 部分照 motor_drive/protocol.cpp
复制（接受与 C++ 的漂移）。
"""

from __future__ import annotations

# ---------------------------------------------------------------------------
# slcan 帧收发
# ---------------------------------------------------------------------------

_FD_LEN_CODE = {
    0: "0", 1: "1", 2: "2", 3: "3", 4: "4", 5: "5", 6: "6", 7: "7", 8: "8",
    12: "9", 16: "A", 20: "B", 24: "C", 32: "D", 48: "E", 64: "F",
}
_DECODE_LEN_CODE = {v: k for k, v in _FD_LEN_CODE.items()}


def fd_len_code(nbytes: int) -> str:
    if nbytes not in _FD_LEN_CODE:
        raise ValueError(f"unsupported slcan FD length {nbytes}")
    return _FD_LEN_CODE[nbytes]


def encode_frame(identifier: int, data: bytes, is_fd: bool, brs: bool) -> str:
    """按 CANable2 slcan 方言组帧（标准 ID 11 bit）。"""
    if identifier > 0x7FF:
        raise ValueError(f"standard ID 0x{identifier:X} out of range")
    if is_fd:
        cmd = "b" if brs else "d"
    else:
        cmd = "t"
    return f"{cmd}{identifier:03X}{fd_len_code(len(data))}{data.hex().upper()}\r"


def decode_frame(line: str):
    """解码 CANable2 上报帧，返回 (id, data, is_fd) 或 None。"""
    line = line.strip()
    if len(line) < 5:
        return None
    cmd, rest = line[0], line[1:]
    if cmd in "tT":
        is_fd = False
        id_len = 3 if cmd == "t" else 8
    elif cmd in "dDbB":
        is_fd = True
        id_len = 3 if cmd in "db" else 8
    else:
        return None
    if len(rest) < id_len + 1:
        return None
    try:
        identifier = int(rest[:id_len], 16)
        len_code = rest[id_len]
        payload = rest[id_len + 1:]
    except ValueError:
        return None
    nbytes = _DECODE_LEN_CODE.get(len_code)
    if nbytes is None or len(payload) != nbytes * 2:
        return None
    try:
        data = bytes.fromhex(payload)
    except ValueError:
        return None
    return identifier, data, is_fd


# ---------------------------------------------------------------------------
# vaxor（25/30 mm）编解码
# ---------------------------------------------------------------------------

WR_TICKS_PER_REV = 2_097_152
WR_CURRENT_SCALE_A = 11.0 / 4096.0
WR_KP_SCALE = 0.00001
WR_KD_SCALE = 0.01
_WR_PI = 3.141592653589793


def wrist_command_id(node: int) -> int:
    return 0x501 + 4 * node


def wrist_control_id(node: int) -> int:
    return 0x502 + 4 * node


def wrist_reply_id(node: int) -> int:
    return 0x701 + 4 * node


def wrist_status_id(node: int) -> int:
    return 0x702 + 4 * node


def wrist_calib_id(node: int) -> int:
    return 0x503 + 4 * node


def wrist_calib_result_id(node: int) -> int:
    return 0x703 + 4 * node


def pos_to_code(rad: float) -> int:
    return round(rad / (2.0 * _WR_PI) * WR_TICKS_PER_REV) + 0x80000000


def code_to_pos(code: int) -> float:
    return (code - 0x80000000) / WR_TICKS_PER_REV * 2.0 * _WR_PI


def vel_to_code(rad_s: float) -> int:
    rpm10 = round(rad_s * 60.0 / (2.0 * _WR_PI) * 10.0)
    return max(-30000, min(30000, rpm10)) + 0x8000


def code_to_vel(code: int) -> float:
    return (code - 0x8000) / 10.0 * 2.0 * _WR_PI / 60.0


def current_to_code(amp: float) -> int:
    return max(-32768, min(32767, round(amp / WR_CURRENT_SCALE_A))) + 0x8000


def code_to_current(code: int) -> float:
    return (code - 0x8000) * WR_CURRENT_SCALE_A


def le16(data: bytes, off: int) -> int:
    return int.from_bytes(data[off:off + 2], "little")


def le32(data: bytes, off: int) -> int:
    return int.from_bytes(data[off:off + 4], "little")


def pack_wrist_command(position_rad: float, velocity_rad_s: float,
                       kp: float, kd: float, current_a: float) -> bytes:
    """与 protocol.cpp::pack_wrist_command 相同的 12 字节 MIT 下行。"""
    return (
        pos_to_code(position_rad).to_bytes(4, "little")
        + vel_to_code(velocity_rad_s).to_bytes(2, "little")
        + round(kp / WR_KP_SCALE).to_bytes(2, "little")
        + round(kd / WR_KD_SCALE).to_bytes(2, "little")
        + current_to_code(current_a).to_bytes(2, "little")
    )


def pack_wrist_control(run: bool, set_zero: bool = False, mode: int = 0) -> bytes:
    return bytes([int(run), int(set_zero), mode, 0, 0, 0, 0, 0])


def decode_vaxor_feedback(frame_id: int, data: bytes, node: int):
    """解码 vaxor 回复/状态帧，返回统一状态 dict 或 None。"""
    if frame_id == wrist_reply_id(node) and len(data) >= 12:
        return {
            "position": code_to_pos(le32(data, 2)),
            "velocity": code_to_vel(le16(data, 6)),
            "current_a": code_to_current(le16(data, 8)),
        }
    if frame_id == wrist_status_id(node) and len(data) >= 20:
        return {
            "position": code_to_pos(le32(data, 2)),
            "velocity": code_to_vel(le16(data, 6)),
            "current_a": code_to_current(le16(data, 8)),
            "voltage_v": le16(data, 10) / 10.0,
            "fault": le32(data, 12),
            "motor_state": data[16],
            "calibration_state": data[17],
            "mode": data[18],
            "motor_type": data[19],
        }
    return None


# ---------------------------------------------------------------------------
# RP 编解码（照 motor_drive/protocol.cpp）
# ---------------------------------------------------------------------------

RP_ENABLE_ID = 0x00
RP_COMMAND_ID = 0x80
RP_BROADCAST_SLOTS = 8

_RP_POS_RANGE = (-10.0, 10.0, 16)
_RP_VEL_RANGE = (-60.0, 60.0, 12)
_RP_KP_RANGE = (0.0, 500.0, 12)
_RP_KD_RANGE = (0.0, 10.0, 12)
_RP_TORQUE_RANGE = (-60.0, 60.0, 12)

_RP_ZERO_COMMAND = {"position": 0.0, "velocity": 0.0, "kp": 0.0, "kd": 0.0,
                    "feedforward": 0.0}


def _float_to_uint(value: float, low: float, high: float, bits: int) -> int:
    # 与 protocol.cpp::float_to_uint 一致：先限幅，再线性映射并截断（非四舍五入）。
    value = max(low, min(high, value))
    maximum = (1 << bits) - 1
    return int((value - low) * maximum / (high - low))


def _uint_to_float(value: int, low: float, high: float, bits: int) -> float:
    maximum = (1 << bits) - 1
    return value * (high - low) / maximum + low


def pack_rp_slot(command: dict) -> bytes:
    """RP MIT 8 Byte 槽：P(16)+V(12)+Kp(12)+Kd(12)+Tau(12)。"""
    position = _float_to_uint(command.get("position", 0.0), *_RP_POS_RANGE)
    velocity = _float_to_uint(command.get("velocity", 0.0), *_RP_VEL_RANGE)
    kp = _float_to_uint(command.get("kp", 0.0), *_RP_KP_RANGE)
    kd = _float_to_uint(command.get("kd", 0.0), *_RP_KD_RANGE)
    torque = _float_to_uint(command.get("feedforward", 0.0), *_RP_TORQUE_RANGE)
    return bytes([
        position >> 8, position & 0xFF,
        velocity >> 4, ((velocity & 0x0F) << 4) | (kp >> 8), kp & 0xFF,
        kd >> 4, ((kd & 0x0F) << 4) | (torque >> 8), torque & 0xFF,
    ])


def pack_rp_command(slot_commands: dict) -> bytes:
    """RP 8 槽 64 字节广播：slot_commands 键为电机 id（1-8），未配置槽填中性槽。"""
    payload = bytearray(RP_BROADCAST_SLOTS * 8)
    for slot in range(RP_BROADCAST_SLOTS):
        command = slot_commands.get(slot + 1, _RP_ZERO_COMMAND)
        payload[slot * 8:(slot + 1) * 8] = pack_rp_slot(command)
    return bytes(payload)


def pack_rp_enable(enabled: bool) -> bytes:
    return bytes([int(enabled), 0, 0, 0, 0, 0, 0, 0])


def unpack_rp_state(data: bytes) -> dict:
    """RP 12 字节状态帧 -> 位姿/速度/扭矩 + MOS 温度/status/电压/线圈温度。"""
    if len(data) < 12:
        raise ValueError("RP state must be 12 bytes")
    return {
        "position": _uint_to_float((data[0] << 8) | data[1], *_RP_POS_RANGE),
        "velocity": _uint_to_float((data[2] << 4) | (data[3] >> 4), *_RP_VEL_RANGE),
        "effort": _uint_to_float(((data[3] & 0x0F) << 8) | data[4], *_RP_TORQUE_RANGE),
        "mos_temperature_c": data[5] - 40.0,
        "status": (data[6] << 8) | data[7],
        "voltage_v": data[8],
        "coil_temperature_c": data[9] - 40.0,
    }
