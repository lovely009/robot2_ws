"""Small dependency-free SocketCAN / CAN-FD wrapper.

The module deliberately uses Linux SocketCAN directly instead of python-can.
That keeps the simulation protocol path very close to a future real CAN-FD
interface: change ``vcan0`` / ``vcan1`` to the physical interface names and
the packet API stays the same.
"""

from dataclasses import dataclass
import socket
import struct
from typing import List


SOL_CAN_RAW = getattr(socket, "SOL_CAN_RAW", 101)
CAN_RAW_FD_FRAMES = 5
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x000007FF
CAN_EFF_MASK = 0x1FFFFFFF
CANFD_BRS = 0x01

# Linux ABI layouts from <linux/can.h>. Both structures are fixed-size.
_CAN_FRAME = struct.Struct("=IBBBB8s")       # sizeof(struct can_frame) = 16
_CANFD_FRAME = struct.Struct("=IBBBB64s")   # sizeof(struct canfd_frame) = 72


@dataclass(frozen=True)
class CanFrame:
    """A decoded standard/extended SocketCAN frame."""

    arbitration_id: int
    data: bytes
    is_fd: bool
    bitrate_switch: bool = False
    is_extended_id: bool = False


class SocketCanBus:
    """Non-blocking RAW SocketCAN endpoint with CAN-FD enabled."""

    def __init__(self, interface: str) -> None:
        self.interface = interface
        self._socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self._socket.setsockopt(SOL_CAN_RAW, CAN_RAW_FD_FRAMES, 1)
        self._socket.bind((interface,))
        self._socket.setblocking(False)

    def close(self) -> None:
        self._socket.close()

    def send(
        self,
        arbitration_id: int,
        data: bytes,
        *,
        force_fd: bool | None = None,
        bitrate_switch: bool = True,
        is_extended_id: bool = False,
    ) -> None:
        payload = bytes(data)
        if len(payload) > 64:
            raise ValueError("CAN-FD payload cannot exceed 64 bytes")

        can_id = arbitration_id
        if is_extended_id:
            if not 0 <= can_id <= CAN_EFF_MASK:
                raise ValueError("29-bit CAN ID out of range")
            can_id |= CAN_EFF_FLAG
        else:
            if not 0 <= can_id <= CAN_SFF_MASK:
                raise ValueError("11-bit CAN ID out of range")

        use_fd = (len(payload) > 8) if force_fd is None else force_fd
        if not use_fd:
            if len(payload) > 8:
                raise ValueError("Classic CAN payload cannot exceed 8 bytes")
            raw = _CAN_FRAME.pack(can_id, len(payload), 0, 0, 0, payload.ljust(8, b"\x00"))
        else:
            flags = CANFD_BRS if bitrate_switch else 0
            raw = _CANFD_FRAME.pack(
                can_id,
                len(payload),
                flags,
                0,
                0,
                payload.ljust(64, b"\x00"),
            )
        self._socket.send(raw)

    def recv_available(self, max_frames: int = 128) -> List[CanFrame]:
        """Drain currently queued frames without blocking the ROS executor."""
        frames: List[CanFrame] = []
        for _ in range(max_frames):
            try:
                raw = self._socket.recv(_CANFD_FRAME.size)
            except BlockingIOError:
                break

            if len(raw) == _CAN_FRAME.size:
                can_id, length, _pad, _res0, _len8_dlc, payload = _CAN_FRAME.unpack(raw)
                is_fd = False
                brs = False
            elif len(raw) == _CANFD_FRAME.size:
                can_id, length, flags, _res0, _res1, payload = _CANFD_FRAME.unpack(raw)
                is_fd = True
                brs = bool(flags & CANFD_BRS)
            else:
                # A malformed kernel frame should not terminate a long simulation.
                continue

            is_extended = bool(can_id & CAN_EFF_FLAG)
            mask = CAN_EFF_MASK if is_extended else CAN_SFF_MASK
            frames.append(
                CanFrame(
                    arbitration_id=can_id & mask,
                    data=payload[:length],
                    is_fd=is_fd,
                    bitrate_switch=brs,
                    is_extended_id=is_extended,
                )
            )
        return frames
