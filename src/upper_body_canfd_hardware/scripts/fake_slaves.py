#!/usr/bin/env python3
"""Standalone vcan motor emulator for the C++ hardware plugin.

Opens two SocketCAN interfaces (default vcan0/vcan1) and emulates:
  * RP-series MIT broadcast joints (CAN ID 0x80, reply per motor ID 1..4)
  * 25/30 mm wrist drives (MIT command 0x501+4N, reply 0x701+4N, periodic
    status 0x702+4N)

This script needs no ROS installation and is used by
upper_body_canfd_gazebo/launch/hardware_sim.launch.py to exercise the
upper_body_canfd_hardware plugin end-to-end on virtual buses.

Run:  ros2 run upper_body_canfd_hardware fake_slaves.py
"""

import argparse
import math
import socket
import struct
import sys
import threading
import time


SOL_CAN_RAW = getattr(socket, "SOL_CAN_RAW", 101)
CAN_RAW_FD_FRAMES = 5
CAN_RAW_RECV_OWN_MSGS = 4
CANFD_BRS = 0x01

CAN_FRAME = struct.Struct("=IBBBB8s")
CANFD_FRAME = struct.Struct("=IBBBB64s")

TICKS_PER_REVOLUTION = 2_097_152
POSITION_TICK_LIMIT = 0x7CFFFFFF
VELOCITY_CODE_LIMIT = 30_000
CURRENT_SCALE_A = 11.0 / 4096.0

MIT_BROADCAST_ID = 0x80
ENABLE_ID = 0x00


def _clamp(value, low, high):
    return min(max(value, low), high)


def float_to_uint(value, low, high, bits):
    value = _clamp(value, low, high)
    return int((value - low) * ((1 << bits) - 1) / (high - low))


def rp_pack_state(position, velocity, torque, mos=35.0, status=0x0001, voltage=24.0, coil=33.0):
    p = float_to_uint(position, -10.0, 10.0, 16)
    v = float_to_uint(velocity, -60.0, 60.0, 12)
    tq = float_to_uint(torque, -60.0, 60.0, 12)
    return bytes([
        (p >> 8) & 0xFF, p & 0xFF,
        (v >> 4) & 0xFF, ((v & 0x0F) << 4) | ((tq >> 8) & 0x0F), tq & 0xFF,
        int(_clamp(mos + 40.0, 0.0, 255.0)),
        status & 0xFF, (status >> 8) & 0xFF,
        int(_clamp(voltage, 0.0, 255.0)),
        int(_clamp(coil + 40.0, 0.0, 255.0)),
        0x00, 0x00,
    ])


def _position_to_code(position_rad):
    ticks = int(round(position_rad / (2.0 * math.pi) * TICKS_PER_REVOLUTION))
    ticks = _clamp(ticks, -POSITION_TICK_LIMIT, POSITION_TICK_LIMIT)
    return ticks + 0x80000000


def _velocity_to_code(velocity_rad_s):
    rpm = velocity_rad_s * 60.0 / (2.0 * math.pi)
    raw = int(round(rpm * 10.0))
    raw = _clamp(raw, -VELOCITY_CODE_LIMIT, VELOCITY_CODE_LIMIT)
    return raw + 0x8000


def _current_to_code(current_a):
    raw = int(round(current_a / CURRENT_SCALE_A))
    raw = _clamp(raw, -(1 << 15), (1 << 15) - 1)
    return raw + (1 << 15)


def wrist_pack_reply(node, position, velocity, current):
    command_id = (0x501 + 4 * node).to_bytes(2, "little")
    return (
        command_id
        + _position_to_code(position).to_bytes(4, "little")
        + _velocity_to_code(velocity).to_bytes(2, "little")
        + _current_to_code(current).to_bytes(2, "little")
        + bytes(2)
    )


def wrist_pack_status(node, position, velocity, current, enabled):
    command_id = (0x501 + 4 * node).to_bytes(2, "little")
    voltage = int(round(12.0 * 10.0)) + (1 << 15)
    return (
        command_id
        + _position_to_code(position).to_bytes(4, "little")
        + _velocity_to_code(velocity).to_bytes(2, "little")
        + _current_to_code(current).to_bytes(2, "little")
        + (voltage & 0xFFFF).to_bytes(2, "little")
        + (0).to_bytes(4, "little")
        + bytes([4 if enabled else 0, 0, 0, 0x01])
    )


class EmulatedBus:
    def __init__(self, interface, rp_count, wrist_nodes, sock):
        self.interface = interface
        self.rp_count = rp_count
        self.wrist_nodes = wrist_nodes
        self.sock = sock
        self.rp_enabled = False
        self.wrist_enabled = {node: False for node in wrist_nodes}
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()
        return self

    def _send(self, sock, arbitration_id, payload, force_fd=False, bitrate_switch=True):
        if len(payload) > 64:
            raise ValueError("CAN-FD payload cannot exceed 64 bytes")
        use_fd = force_fd or len(payload) > 8
        if use_fd:
            raw = CANFD_FRAME.pack(
                arbitration_id, len(payload), CANFD_BRS if bitrate_switch else 0,
                0, 0, payload.ljust(64, b"\x00"))
        else:
            raw = CAN_FRAME.pack(arbitration_id, len(payload), 0, 0, 0, payload.ljust(8, b"\x00"))
        sock.send(raw)

    def _recv(self, sock):
        raw = sock.recv(CANFD_FRAME.size)
        if len(raw) == CAN_FRAME.size:
            can_id, length, _pad, _res0, _res1, payload = CAN_FRAME.unpack(raw)
        elif len(raw) == CANFD_FRAME.size:
            can_id, length, _flags, _res0, _res1, payload = CANFD_FRAME.unpack(raw)
        else:
            return None
        return can_id, payload[:length]

    def _run(self):
        # The socket is created in the main thread and only *used* here: on
        # this platform, raw CAN sockets created inside a worker thread are
        # not delivered to other sockets on the same vcan interface.
        sock = self.sock
        last_status = 0.0
        while not self.stop.is_set():
            while True:
                try:
                    frame = self._recv(sock)
                except BlockingIOError:
                    break
                if frame is None:
                    continue
                self._handle(sock, frame)

            now = time.monotonic()
            if now - last_status >= 0.05:
                last_status = now
                for node in self.wrist_nodes:
                    if self.wrist_enabled[node]:
                        self._send(
                            sock, 0x702 + 4 * node,
                            wrist_pack_status(node, 0.25, 0.1, 0.0, True))
            time.sleep(0.001)

    def _handle(self, sock, frame):
        can_id, payload = frame
        if can_id == ENABLE_ID and len(payload) == 8:
            self.rp_enabled = payload[0] == 1
            return
        if can_id == MIT_BROADCAST_ID and len(payload) == 64:
            for motor in range(1, self.rp_count + 1):
                slot = payload[(motor - 1) * 8: motor * 8]
                # Decode torque feed-forward from the 12-bit slot tail.
                tq_code = ((slot[6] & 0x0F) << 8) | slot[7]
                tq = tq_code * 120.0 / 4095.0 - 60.0
                self._send(
                    sock, motor,
                    rp_pack_state(0.6 + 0.1 * (motor - 1), 0.05, tq))
            return
        for node in self.wrist_nodes:
            if can_id == 0x501 + 4 * node and len(payload) == 12:
                current_code = int.from_bytes(payload[10:12], "little") - (1 << 15)
                current = current_code * CURRENT_SCALE_A
                self._send(
                    sock, 0x701 + 4 * node,
                    wrist_pack_reply(node, 0.25, 0.1, current))
                return
            if can_id == 0x502 + 4 * node and len(payload) == 8:
                self.wrist_enabled[node] = payload[0] == 1
                return


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left-interface", default="vcan0")
    parser.add_argument("--right-interface", default="vcan1")
    parser.add_argument("--rp-count", type=int, default=4)
    # launch_ros appends --ros-args ... to every node command; drop that part.
    argv = sys.argv[1:]
    if "--ros-args" in argv:
        argv = argv[: argv.index("--ros-args")]
    args = parser.parse_args(argv)

    buses = []
    for interface in (args.left_interface, args.right_interface):
        try:
            sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            sock.setsockopt(SOL_CAN_RAW, CAN_RAW_FD_FRAMES, 1)
            sock.setsockopt(SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, 0)
            sock.bind((interface,))
            sock.setblocking(False)
        except OSError as error:
            print(f"[fake_slaves:{interface}] open failed: {error}", flush=True)
            raise
        buses.append(
            EmulatedBus(interface, args.rp_count, [1, 2, 3], sock).start())
    print(
        f"[fake_slaves] emulating RP x{args.rp_count} + wrist x3 on "
        f"{args.left_interface} / {args.right_interface}",
        flush=True)
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        for bus in buses:
            bus.stop.set()
            bus.thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
