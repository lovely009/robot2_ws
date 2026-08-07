"""Safe vcan-only demo commander for the 14-DOF upper body.

It sends exactly the same motor packets that an upper controller would send to
real devices. By default it refuses non-vcan interface names.
"""

import argparse
import math
import time
from typing import Dict, List

from . import protocol_2530 as wrist_protocol
from . import protocol_rp as rp_protocol
from .socketcan import SocketCanBus


def _rp_targets(t: float, side_sign: float) -> List[rp_protocol.RpMitCommand]:
    w = 0.55
    s = math.sin(w * t)
    c = math.cos(w * t)
    return [
        rp_protocol.RpMitCommand(0.22 * side_sign * s, 0.22 * side_sign * w * c, 36.0, 2.0, 0.0),
        rp_protocol.RpMitCommand(0.18 * side_sign * s, 0.18 * side_sign * w * c, 32.0, 1.8, 0.0),
        rp_protocol.RpMitCommand(0.28 * s, 0.28 * w * c, 38.0, 2.2, 0.0),
        rp_protocol.RpMitCommand(0.65 + 0.22 * s, 0.22 * w * c, 26.0, 1.6, 0.0),
    ]


def _wrist_targets(t: float, side_sign: float) -> List[wrist_protocol.WristMitCommand]:
    w = 0.75
    phases = (0.0, 0.8, 1.6)
    amplitudes = (0.30, 0.22, 0.38)
    commands: List[wrist_protocol.WristMitCommand] = []
    for index, (phase, amplitude) in enumerate(zip(phases, amplitudes)):
        q = amplitude * math.sin(w * t + phase)
        v = amplitude * w * math.cos(w * t + phase)
        if index == 0:
            q *= side_sign
            v *= side_sign
        commands.append(
            wrist_protocol.WristMitCommand(
                position=q,
                velocity=v,
                kp=0.008,
                kd=1.0,
                current_ff=0.0,
            )
        )
    return commands


def _enable(bus: SocketCanBus) -> None:
    bus.send(rp_protocol.ENABLE_ID, rp_protocol.pack_enable(True), force_fd=False)
    for node in (1, 2, 3):
        # Manual-required sequence: stop while selecting MIT, then run.
        bus.send(
            wrist_protocol.motor_control_id(node),
            wrist_protocol.pack_motor_control(False, mode=wrist_protocol.MODE_MIT),
            force_fd=False,
        )
        bus.send(
            wrist_protocol.motor_control_id(node),
            wrist_protocol.pack_motor_control(True, mode=wrist_protocol.MODE_MIT),
            force_fd=False,
        )


def _safe_stop(bus: SocketCanBus) -> None:
    bus.send(
        rp_protocol.MIT_BROADCAST_ID,
        rp_protocol.pack_broadcast([rp_protocol.RpMitCommand()] * 4),
        force_fd=True,
    )
    bus.send(rp_protocol.ENABLE_ID, rp_protocol.pack_enable(False), force_fd=False)
    for node in (1, 2, 3):
        bus.send(
            wrist_protocol.mit_command_id(node),
            wrist_protocol.pack_mit_command(wrist_protocol.WristMitCommand()),
            force_fd=True,
        )
        bus.send(
            wrist_protocol.motor_control_id(node),
            wrist_protocol.pack_motor_control(False, mode=wrist_protocol.MODE_MIT),
            force_fd=False,
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="Send a gentle dual-arm MIT demo over SocketCAN")
    parser.add_argument("--left-interface", default="vcan0")
    parser.add_argument("--right-interface", default="vcan1")
    parser.add_argument("--rate", type=float, default=100.0, help="command rate in Hz")
    parser.add_argument("--duration", type=float, default=0.0, help="seconds; 0 means until Ctrl+C")
    parser.add_argument(
        "--allow-hardware",
        action="store_true",
        help="permit non-vcan interface names (DANGEROUS on a real robot)",
    )
    args = parser.parse_args()

    if args.rate <= 0.0:
        raise SystemExit("--rate must be positive")
    if args.duration < 0.0:
        raise SystemExit("--duration cannot be negative")
    if not args.allow_hardware:
        for interface in (args.left_interface, args.right_interface):
            if not interface.startswith("vcan"):
                raise SystemExit(
                    f"Refusing to command '{interface}'. Use --allow-hardware only after hardware validation."
                )

    buses: Dict[str, SocketCanBus] = {
        "left": SocketCanBus(args.left_interface),
        "right": SocketCanBus(args.right_interface),
    }
    period = 1.0 / args.rate
    start = time.monotonic()
    next_tick = start

    print(
        f"MIT demo started: left={args.left_interface}, right={args.right_interface}. "
        "Ctrl+C performs a stop/disable sequence."
    )
    try:
        for bus in buses.values():
            _enable(bus)

        while True:
            now = time.monotonic()
            elapsed = now - start
            if args.duration > 0.0 and elapsed >= args.duration:
                break

            for arm, sign in (("left", 1.0), ("right", -1.0)):
                bus = buses[arm]
                bus.send(
                    rp_protocol.MIT_BROADCAST_ID,
                    rp_protocol.pack_broadcast(_rp_targets(elapsed, sign)),
                    force_fd=True,
                )
                for node, command in enumerate(_wrist_targets(elapsed, sign), start=1):
                    bus.send(
                        wrist_protocol.mit_command_id(node),
                        wrist_protocol.pack_mit_command(command),
                        force_fd=True,
                    )

            next_tick += period
            sleep_time = next_tick - time.monotonic()
            if sleep_time > 0.0:
                time.sleep(sleep_time)
            else:
                next_tick = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        for bus in buses.values():
            try:
                _safe_stop(bus)
            finally:
                bus.close()
        print("MIT demo stopped; RP and wrist motors were disabled.")


if __name__ == "__main__":
    main()
