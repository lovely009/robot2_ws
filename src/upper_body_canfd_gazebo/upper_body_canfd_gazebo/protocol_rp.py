"""Codec for the supplied YiYou RP-series CAN-FD MIT protocol.

Primary source: ``意优RP系列行星关节CAN-FD通信协议.pdf`` V1.0,
revised 2026-03-30. The MIT broadcast uses CAN ID 0x80 and one 8-byte
slot per joint, up to eight joints / 64 bytes.
"""

from dataclasses import dataclass
from typing import Iterable, List


POSITION_RANGE = (-10.0, 10.0)       # rad, 16 bit
VELOCITY_RANGE = (-60.0, 60.0)       # rad/s, 12 bit
KP_RANGE = (0.0, 500.0)              # 12 bit
KD_RANGE = (0.0, 10.0)               # 12 bit
TORQUE_RANGE = (-60.0, 60.0)         # Nm, 12 bit
MIT_BROADCAST_ID = 0x80
ENABLE_ID = 0x00
MOTOR_COUNT_PER_BROADCAST = 8


@dataclass(frozen=True)
class RpMitCommand:
    position: float = 0.0
    velocity: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    torque_ff: float = 0.0


@dataclass(frozen=True)
class RpState:
    position: float
    velocity: float
    torque: float
    mos_temperature_c: float
    status: int
    voltage_v: float
    coil_temperature_c: float


def clamp(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)


def float_to_uint(value: float, low: float, high: float, bits: int) -> int:
    value = clamp(value, low, high)
    return int((value - low) * ((1 << bits) - 1) / (high - low))


def uint_to_float(value: int, low: float, high: float, bits: int) -> float:
    return value * (high - low) / ((1 << bits) - 1) + low


def pack_mit_slot(command: RpMitCommand) -> bytes:
    """Pack one 8-byte MIT command slot exactly as the RP manual specifies."""
    p = float_to_uint(command.position, *POSITION_RANGE, 16)
    v = float_to_uint(command.velocity, *VELOCITY_RANGE, 12)
    kp = float_to_uint(command.kp, *KP_RANGE, 12)
    kd = float_to_uint(command.kd, *KD_RANGE, 12)
    tq = float_to_uint(command.torque_ff, *TORQUE_RANGE, 12)
    return bytes(
        [
            (p >> 8) & 0xFF,
            p & 0xFF,
            (v >> 4) & 0xFF,
            ((v & 0x0F) << 4) | ((kp >> 8) & 0x0F),
            kp & 0xFF,
            (kd >> 4) & 0xFF,
            ((kd & 0x0F) << 4) | ((tq >> 8) & 0x0F),
            tq & 0xFF,
        ]
    )


def unpack_mit_slot(data: bytes) -> RpMitCommand:
    if len(data) != 8:
        raise ValueError("RP MIT slot must be exactly 8 bytes")
    p = (data[0] << 8) | data[1]
    v = (data[2] << 4) | (data[3] >> 4)
    kp = ((data[3] & 0x0F) << 8) | data[4]
    kd = (data[5] << 4) | (data[6] >> 4)
    tq = ((data[6] & 0x0F) << 8) | data[7]
    return RpMitCommand(
        position=uint_to_float(p, *POSITION_RANGE, 16),
        velocity=uint_to_float(v, *VELOCITY_RANGE, 12),
        kp=uint_to_float(kp, *KP_RANGE, 12),
        kd=uint_to_float(kd, *KD_RANGE, 12),
        torque_ff=uint_to_float(tq, *TORQUE_RANGE, 12),
    )


def pack_broadcast(commands: Iterable[RpMitCommand]) -> bytes:
    commands = list(commands)
    if len(commands) > MOTOR_COUNT_PER_BROADCAST:
        raise ValueError("RP broadcast supports at most 8 joints")
    # Never pad with zero bytes: all-zero bytes decode to -60 Nm feed-forward.
    neutral = RpMitCommand()
    commands.extend([neutral] * (MOTOR_COUNT_PER_BROADCAST - len(commands)))
    return b"".join(pack_mit_slot(command) for command in commands)


def unpack_broadcast(data: bytes) -> List[RpMitCommand]:
    if len(data) != 64:
        raise ValueError("RP MIT broadcast must be exactly 64 bytes")
    return [unpack_mit_slot(data[i : i + 8]) for i in range(0, 64, 8)]


def pack_enable(enabled: bool) -> bytes:
    """Pack the RP standard CAN ID 0x00 enable/disable frame."""
    return bytes([0x01 if enabled else 0x00]) + bytes(7)


def pack_state(
    position: float,
    velocity: float,
    torque: float,
    *,
    mos_temperature_c: float = 35.0,
    status: int = 0x0001,
    voltage_v: float = 24.0,
    coil_temperature_c: float = 33.0,
) -> bytes:
    """Pack the 12-byte RP MIT state frame returned on CAN ID = motor ID."""
    p = float_to_uint(position, *POSITION_RANGE, 16)
    v = float_to_uint(velocity, *VELOCITY_RANGE, 12)
    tq = float_to_uint(torque, *TORQUE_RANGE, 12)
    mos = int(clamp(mos_temperature_c + 40.0, 0.0, 255.0))
    coil = int(clamp(coil_temperature_c + 40.0, 0.0, 255.0))
    voltage = int(clamp(voltage_v, 0.0, 255.0))
    return bytes(
        [
            (p >> 8) & 0xFF,
            p & 0xFF,
            (v >> 4) & 0xFF,
            ((v & 0x0F) << 4) | ((tq >> 8) & 0x0F),
            tq & 0xFF,
            mos,
            status & 0xFF,
            (status >> 8) & 0xFF,
            voltage,
            coil,
            0x00,
            0x00,
        ]
    )


def unpack_state(data: bytes) -> RpState:
    if len(data) != 12:
        raise ValueError("RP state frame must be exactly 12 bytes")
    p = (data[0] << 8) | data[1]
    v = (data[2] << 4) | (data[3] >> 4)
    tq = ((data[3] & 0x0F) << 8) | data[4]
    return RpState(
        position=uint_to_float(p, *POSITION_RANGE, 16),
        velocity=uint_to_float(v, *VELOCITY_RANGE, 12),
        torque=uint_to_float(tq, *TORQUE_RANGE, 12),
        mos_temperature_c=float(data[5]) - 40.0,
        status=data[6] | (data[7] << 8),
        voltage_v=float(data[8]),
        coil_temperature_c=float(data[9]) - 40.0,
    )
