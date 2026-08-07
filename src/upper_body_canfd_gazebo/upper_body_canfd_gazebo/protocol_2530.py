"""Codec for the supplied 25/30 mm integrated drive CAN-FD protocol.

Source: ``25-30mm驱控一体使用说明手册_20260204.pdf``.

Important project correction from the user (2026-08-07): the manual says the
CAN-FD data phase is 4 Mbps, but the corrected hardware configuration for this
robot is 5 Mbps. Packet encoding is unchanged by that correction.
"""

from dataclasses import dataclass
import math
from typing import Literal


TICKS_PER_REVOLUTION = 2_097_152
POSITION_TICK_LIMIT = 0x7CFFFFFF
VELOCITY_CODE_LIMIT = 30_000
CURRENT_SCALE_A = 11.0 / 4096.0
KP_SCALE = 0.00001
KD_SCALE = 0.01

MODE_MIT = 0x00
MODE_VELOCITY = 0x01
MODE_POSITION = 0x02


@dataclass(frozen=True)
class WristMitCommand:
    position: float = 0.0       # rad, multi-turn
    velocity: float = 0.0       # rad/s
    kp: float = 0.0             # protocol coefficient
    kd: float = 0.0             # protocol coefficient
    current_ff: float = 0.0     # A


@dataclass(frozen=True)
class WristReply:
    command_can_id: int
    position: float
    velocity: float
    current: float


def _validate_node(node: int) -> None:
    if not 1 <= node <= 60:
        raise ValueError("25/30 mm controller N must be in [1, 60]")


def mit_command_id(node: int) -> int:
    _validate_node(node)
    return 0x501 + 4 * node


def motor_control_id(node: int) -> int:
    _validate_node(node)
    return 0x502 + 4 * node


def mit_reply_id(node: int) -> int:
    _validate_node(node)
    return 0x701 + 4 * node


def status_id(node: int) -> int:
    _validate_node(node)
    return 0x702 + 4 * node


def _signed_to_biased(value: int, bits: int) -> int:
    limit = 1 << (bits - 1)
    value = min(max(value, -limit), limit - 1)
    return value + limit


def _biased_to_signed(value: int, bits: int) -> int:
    return value - (1 << (bits - 1))


def _position_to_code(position_rad: float) -> int:
    ticks = int(round(position_rad / (2.0 * math.pi) * TICKS_PER_REVOLUTION))
    ticks = min(max(ticks, -POSITION_TICK_LIMIT), POSITION_TICK_LIMIT)
    return ticks + 0x80000000


def _code_to_position(code: int) -> float:
    ticks = code - 0x80000000
    return ticks / TICKS_PER_REVOLUTION * (2.0 * math.pi)


def _velocity_to_code(velocity_rad_s: float) -> int:
    rpm = velocity_rad_s * 60.0 / (2.0 * math.pi)
    raw = int(round(rpm * 10.0))
    raw = min(max(raw, -VELOCITY_CODE_LIMIT), VELOCITY_CODE_LIMIT)
    return raw + 0x8000


def _code_to_velocity(code: int) -> float:
    rpm = _biased_to_signed(code, 16) / 10.0
    return rpm * (2.0 * math.pi) / 60.0


def _current_to_code(current_a: float) -> int:
    raw = int(round(current_a / CURRENT_SCALE_A))
    return _signed_to_biased(raw, 16)


def _code_to_current(code: int) -> float:
    return _biased_to_signed(code, 16) * CURRENT_SCALE_A


def pack_mit_command(command: WristMitCommand) -> bytes:
    """Pack the 12-byte host -> motor MIT command (little-endian fields)."""
    p = _position_to_code(command.position)
    v = _velocity_to_code(command.velocity)
    kp = min(max(int(round(command.kp / KP_SCALE)), 0), 0xFFFF)
    kd = min(max(int(round(command.kd / KD_SCALE)), 0), 0xFFFF)
    current = _current_to_code(command.current_ff)
    return (
        p.to_bytes(4, "little", signed=False)
        + v.to_bytes(2, "little", signed=False)
        + kp.to_bytes(2, "little", signed=False)
        + kd.to_bytes(2, "little", signed=False)
        + current.to_bytes(2, "little", signed=False)
    )


def unpack_mit_command(data: bytes) -> WristMitCommand:
    if len(data) != 12:
        raise ValueError("25/30 mm MIT command must be exactly 12 bytes")
    p = int.from_bytes(data[0:4], "little")
    v = int.from_bytes(data[4:6], "little")
    kp = int.from_bytes(data[6:8], "little")
    kd = int.from_bytes(data[8:10], "little")
    current = int.from_bytes(data[10:12], "little")
    return WristMitCommand(
        position=_code_to_position(p),
        velocity=_code_to_velocity(v),
        kp=kp * KP_SCALE,
        kd=kd * KD_SCALE,
        current_ff=_code_to_current(current),
    )


def pack_motor_control(
    run: bool,
    *,
    set_zero: bool = False,
    mode: Literal[0, 1, 2] = MODE_MIT,
) -> bytes:
    if mode not in (MODE_MIT, MODE_VELOCITY, MODE_POSITION):
        raise ValueError("mode must be MIT(0), velocity(1), or position(2)")
    return bytes([int(run), int(set_zero), mode, 0, 0, 0, 0, 0])


def pack_mit_reply(node: int, position: float, velocity: float, current: float) -> bytes:
    """Pack the manual's 12-byte motor -> host MIT reply."""
    command_id_le = mit_command_id(node).to_bytes(2, "little")
    p = _position_to_code(position).to_bytes(4, "little")
    v = _velocity_to_code(velocity).to_bytes(2, "little")
    i = _current_to_code(current).to_bytes(2, "little")
    return command_id_le + p + v + i + bytes(2)


def unpack_mit_reply(data: bytes) -> WristReply:
    if len(data) != 12:
        raise ValueError("25/30 mm MIT reply must be exactly 12 bytes")
    return WristReply(
        command_can_id=int.from_bytes(data[0:2], "little"),
        position=_code_to_position(int.from_bytes(data[2:6], "little")),
        velocity=_code_to_velocity(int.from_bytes(data[6:8], "little")),
        current=_code_to_current(int.from_bytes(data[8:10], "little")),
    )


def pack_status(
    node: int,
    position: float,
    velocity: float,
    current: float,
    *,
    voltage_v: float = 12.0,
    fault: int = 0,
    motor_state: int = 4,
    calibration_state: int = 0,
    mode: int = MODE_MIT,
    motor_type: int = 0x01,
) -> bytes:
    """Pack the 20-byte periodic status frame (manual section 3.3)."""
    voltage_code = _signed_to_biased(int(round(voltage_v * 10.0)), 16)
    return (
        mit_command_id(node).to_bytes(2, "little")
        + _position_to_code(position).to_bytes(4, "little")
        + _velocity_to_code(velocity).to_bytes(2, "little")
        + _current_to_code(current).to_bytes(2, "little")
        + voltage_code.to_bytes(2, "little")
        + int(fault & 0xFFFFFFFF).to_bytes(4, "little")
        + bytes([motor_state & 0xFF, calibration_state & 0xFF, mode & 0xFF, motor_type & 0xFF])
    )


def kp_to_current_gain(kp_protocol: float) -> float:
    """Convert protocol Kp to the manual's physical A/rad gain."""
    return kp_protocol / 0.001115


def kd_to_current_gain(kd_protocol: float) -> float:
    """Convert protocol Kd to the manual's physical A/(rad/s) gain."""
    return kd_protocol / 85.636
