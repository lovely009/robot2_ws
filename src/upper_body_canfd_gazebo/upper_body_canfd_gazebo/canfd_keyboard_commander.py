"""Interactive keyboard commander for the 14-DOF upper body.

Sends the same CAN-FD motor packets as ``canfd_demo``, but accepts manual
commands from stdin: enable/disable arms, move individual joints with MIT
setpoints, and hold them. A background thread keeps streaming the commanded
state so the bridge's command timeout never expires.

By default it refuses non-vcan interface names.
"""

import argparse
import math
import shlex
import threading
import time
from typing import Dict, List, Optional, Tuple

from . import protocol_2530 as wrist_protocol
from . import protocol_rp as rp_protocol
from .socketcan import SocketCanBus


JOINT_INDEX: Dict[str, int] = {
    "shoulder_yaw": 0,
    "shoulder_roll": 1,
    "shoulder_pitch": 2,
    "elbow_pitch": 3,
    "wrist_yaw": 4,
    "wrist_pitch": 5,
    "wrist_roll": 6,
}

# Default gains used by the pos/vel shortcuts (same magnitude as canfd_demo).
RP_KP = 36.0
RP_KD = 2.0
WRIST_KP = 0.008
WRIST_KD = 1.0

HELP_TEXT = """\
Commands:
  enable [left|right|all]          enable motors (default: all)
  disable [left|right|all]         safe stop + disable (default: all)
  neutral [left|right|all]         zero all targets (arms relax), keep enabled
  pos <side> <joint> <rad> [kp] [kd]   position setpoint, vel=0, ff=0
  vel <side> <joint> <rad/s>       velocity setpoint (kp=0, keeps kd)
  mit <side> <joint> <pos> <vel> <kp> <kd> <ff>   full MIT setpoint
  status                           print commanded state for both arms
  act [<n>]                        play preset action 1..6; bare 'act' lists them
  stop                             stop the playing action and hold the pose
  quit                             safe stop + exit (same as Ctrl+C)

<side> is left|right. <joint> is 1..7 or a name:
  1=shoulder_yaw 2=shoulder_roll 3=shoulder_pitch 4=elbow_pitch (RP)
  5=wrist_yaw    6=wrist_pitch   7=wrist_roll              (25/30 mm)
RP ff is torque (Nm); wrist ff is current (A).
播放动作期间请先执行 stop，再发送手动指令。
"""


class Action:
    """A timed preset trajectory; ``fn(t)`` returns per-arm target lists."""

    def __init__(self, name: str, duration: float, fn) -> None:
        self.name = name
        self.duration = duration
        self.fn = fn
        self.start = time.monotonic()


def _ramp(t: float, t0: float, t1: float) -> float:
    """Smoothstep 0 -> 1 between t0 and t1, clamped outside."""
    if t <= t0:
        return 0.0
    if t >= t1:
        return 1.0
    x = (t - t0) / (t1 - t0)
    return x * x * (3.0 - 2.0 * x)


def _rp_command(
    position: float = 0.0,
    velocity: float = 0.0,
    kp: float = RP_KP,
    kd: float = RP_KD,
    torque_ff: float = 0.0,
) -> rp_protocol.RpMitCommand:
    return rp_protocol.RpMitCommand(position, velocity, kp, kd, torque_ff)


def _wrist_command(
    position: float = 0.0,
    velocity: float = 0.0,
    kp: float = WRIST_KP,
    kd: float = WRIST_KD,
    current_ff: float = 0.0,
) -> wrist_protocol.WristMitCommand:
    return wrist_protocol.WristMitCommand(position, velocity, kp, kd, current_ff)


def _neutral_targets() -> Dict[str, Tuple[List[rp_protocol.RpMitCommand], List[wrist_protocol.WristMitCommand]]]:
    """Held neutral targets: all joints zeroed with holding gains, so
    un-involved joints do not flail under dynamic coupling."""
    return {
        arm: (
            [_rp_command() for _ in range(4)],
            [_wrist_command() for _ in range(3)],
        )
        for arm in ("left", "right")
    }


def _wave_targets(t: float) -> Dict[str, Tuple[List[rp_protocol.RpMitCommand], List[wrist_protocol.WristMitCommand]]]:
    """Action 1: raise the right arm and wave it side to side."""
    targets = _neutral_targets()
    raise_env = _ramp(t, 0.0, 1.5) * (1.0 - _ramp(t, 7.0, 8.5))
    rp = targets["right"][0]
    rp[1] = _rp_command(1.1 * raise_env)    # shoulder_roll: raise arm out
    rp[2] = _rp_command(0.35 * raise_env)   # shoulder_pitch
    rp[3] = _rp_command(1.2 * raise_env)    # elbow_pitch: bend the arm
    if 1.5 <= t < 7.0:
        w = 2.0 * math.pi * 1.2
        rp[0] = _rp_command(
            0.45 * math.sin(w * (t - 1.5)),
            0.45 * w * math.cos(w * (t - 1.5)),
        )
    return targets


def _hug_targets(t: float) -> Dict[str, Tuple[List[rp_protocol.RpMitCommand], List[wrist_protocol.WristMitCommand]]]:
    """Action 2: spread both arms, then fold them inward (hug)."""
    targets = _neutral_targets()
    open_env = _ramp(t, 0.0, 2.0) * (1.0 - _ramp(t, 5.0, 8.0))
    close_env = _ramp(t, 5.0, 8.0)
    for arm, sign in (("left", 1.0), ("right", -1.0)):
        rp = targets[arm][0]
        rp[1] = _rp_command(sign * 1.1 * open_env)               # spread sideways
        rp[2] = _rp_command(0.25 * open_env + 0.55 * close_env)
        rp[3] = _rp_command(0.4 * open_env + 1.4 * close_env)    # fold for the hug
    return targets


def _walk_targets(t: float) -> Dict[str, Tuple[List[rp_protocol.RpMitCommand], List[wrist_protocol.WristMitCommand]]]:
    """Action 3: alternate the arms like a walking gait."""
    targets = _neutral_targets()
    env = _ramp(t, 0.0, 1.0) * (1.0 - _ramp(t, 9.0, 10.0))
    w = 2.0 * math.pi * 0.8
    phase = w * t
    for arm, sign in (("left", 1.0), ("right", -1.0)):
        rp = targets[arm][0]
        rp[2] = _rp_command(
            sign * 0.5 * math.sin(phase) * env,
            sign * 0.5 * w * math.cos(phase) * env,
        )
        rp[3] = _rp_command(0.9 * env)  # slightly bent arms while swinging
    return targets


def _cheer_targets(t: float) -> Dict[str, Tuple[List[rp_protocol.RpMitCommand], List[wrist_protocol.WristMitCommand]]]:
    """Action 4: raise both arms up (cheer), hold, then lower them."""
    targets = _neutral_targets()
    raise_env = _ramp(t, 0.0, 2.0) * (1.0 - _ramp(t, 6.0, 8.0))
    for arm, sign in (("left", 1.0), ("right", -1.0)):
        rp = targets[arm][0]
        rp[1] = _rp_command(sign * 1.0 * raise_env)   # spread sideways
        rp[2] = _rp_command(0.9 * raise_env)          # pitch the arms up
        rp[3] = _rp_command(0.5 * raise_env)          # slightly bent elbows
    return targets


def _sway_targets(t: float) -> Dict[str, Tuple[List[rp_protocol.RpMitCommand], List[wrist_protocol.WristMitCommand]]]:
    """Action 5: swing both arms side to side together (like waving 'no')."""
    targets = _neutral_targets()
    env = _ramp(t, 0.0, 1.0) * (1.0 - _ramp(t, 8.0, 9.5))
    w = 2.0 * math.pi * 0.9
    for arm, sign in (("left", 1.0), ("right", -1.0)):
        rp = targets[arm][0]
        rp[0] = _rp_command(
            0.5 * math.sin(w * t) * env,
            0.5 * w * math.cos(w * t) * env,
        )
        rp[1] = _rp_command(sign * 0.3 * env)  # slightly raise arms out
        rp[3] = _rp_command(0.9 * env)         # bent elbows
    return targets


def _wrist_spin_targets(t: float) -> Dict[str, Tuple[List[rp_protocol.RpMitCommand], List[wrist_protocol.WristMitCommand]]]:
    """Action 6: spin both wrists (roll) back and forth."""
    targets = _neutral_targets()
    env = _ramp(t, 0.0, 1.0) * (1.0 - _ramp(t, 8.0, 10.0))
    w = 2.0 * math.pi * 0.7
    for arm, sign in (("left", 1.0), ("right", -1.0)):
        wrist = targets[arm][1]
        wrist[2] = _wrist_command(
            sign * 1.2 * math.sin(w * t) * env,
            sign * 1.2 * w * math.cos(w * t) * env,
        )
    return targets


ACTIONS: Dict[str, Action] = {
    "1": Action("右手挥手", 8.5, _wave_targets),
    "2": Action("双臂开合（拥抱）", 10.0, _hug_targets),
    "3": Action("交替摆臂（走路）", 10.0, _walk_targets),
    "4": Action("双手上举（欢呼）", 8.0, _cheer_targets),
    "5": Action("左右齐摆（打招呼）", 9.5, _sway_targets),
    "6": Action("转动手腕", 10.0, _wrist_spin_targets),
}


class ArmState:
    """Commanded MIT state for one arm, shared with the stream thread."""

    def __init__(self) -> None:
        self.enabled = False
        self.rp: List[rp_protocol.RpMitCommand] = [
            rp_protocol.RpMitCommand() for _ in range(4)
        ]
        self.wrist: List[wrist_protocol.WristMitCommand] = [
            wrist_protocol.WristMitCommand() for _ in range(3)
        ]


class KeyboardCommander:
    def __init__(self, buses: Dict[str, SocketCanBus], rate: float) -> None:
        self._buses = buses
        self._period = 1.0 / rate
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._arms: Dict[str, ArmState] = {arm: ArmState() for arm in buses}
        self._action: Optional[Action] = None

    # -- low-level frame helpers (mirror canfd_demo_commander) ---------------

    def _send_arm(self, arm: str) -> None:
        bus = self._buses[arm]
        state = self._arms[arm]
        bus.send(
            rp_protocol.MIT_BROADCAST_ID,
            rp_protocol.pack_broadcast(state.rp),
            force_fd=True,
        )
        for node, command in enumerate(state.wrist, start=1):
            bus.send(
                wrist_protocol.mit_command_id(node),
                wrist_protocol.pack_mit_command(command),
                force_fd=True,
            )

    def _enable_arm(self, arm: str) -> None:
        bus = self._buses[arm]
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

    def _safe_stop_arm(self, arm: str) -> None:
        bus = self._buses[arm]
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

    # -- stream thread --------------------------------------------------------

    def _stream_loop(self) -> None:
        while not self._stop.is_set():
            start = time.monotonic()
            with self._lock:
                action = self._action
                if action is not None:
                    t = time.monotonic() - action.start
                    if t >= action.duration:
                        self._action = None
                    else:
                        for arm, (rp_cmds, wrist_cmds) in action.fn(t).items():
                            state = self._arms[arm]
                            state.rp = rp_cmds
                            state.wrist = wrist_cmds
                for arm, state in self._arms.items():
                    if state.enabled:
                        self._send_arm(arm)
            remaining = self._period - (time.monotonic() - start)
            if remaining > 0:
                self._stop.wait(remaining)

    # -- command parsing ------------------------------------------------------

    def _resolve_joint(self, spec: str) -> Optional[int]:
        spec = spec.strip().lower()
        if spec.isdigit():
            index = int(spec) - 1
            return index if 0 <= index <= 6 else None
        return JOINT_INDEX.get(spec)

    def _arms_for(self, spec: str) -> Optional[List[str]]:
        spec = spec.strip().lower()
        if spec in ("all", ""):
            return list(self._arms)
        if spec in ("left", "l"):
            return ["left"]
        if spec in ("right", "r"):
            return ["right"]
        return None

    def _joint_info(self, arm: str, index: int) -> Tuple[str, str]:
        if index < 4:
            names = ("shoulder_yaw", "shoulder_roll", "shoulder_pitch", "elbow_pitch")
            return f"{arm}_{names[index]}_joint", "rp"
        names = ("wrist_yaw", "wrist_pitch", "wrist_roll")
        return f"{arm}_{names[index - 4]}_joint", "wrist"

    def _update_mit(self, arm: str, index: int, command) -> None:
        with self._lock:
            state = self._arms[arm]
            if index < 4:
                state.rp[index] = command
            else:
                state.wrist[index - 4] = command

    # -- interactive commands --------------------------------------------------

    def cmd_enable(self, args: List[str]) -> None:
        arms = self._arms_for(args[0] if args else "all")
        if arms is None:
            print("Unknown side. Use left, right or all.")
            return
        with self._lock:
            for arm in arms:
                if self._arms[arm].enabled:
                    print(f"{arm}: already enabled")
                    continue
                self._enable_arm(arm)
                self._arms[arm].enabled = True
                print(f"{arm}: enabled (vcan streaming started)")

    def cmd_disable(self, args: List[str]) -> None:
        arms = self._arms_for(args[0] if args else "all")
        if arms is None:
            print("Unknown side. Use left, right or all.")
            return
        with self._lock:
            for arm in arms:
                self._safe_stop_arm(arm)
                print(f"{arm}: neutral + disabled")

    def cmd_neutral(self, args: List[str]) -> None:
        arms = self._arms_for(args[0] if args else "all")
        if arms is None:
            print("Unknown side. Use left, right or all.")
            return
        with self._lock:
            for arm in arms:
                state = self._arms[arm]
                state.rp = [rp_protocol.RpMitCommand() for _ in range(4)]
                state.wrist = [wrist_protocol.WristMitCommand() for _ in range(3)]
                print(f"{arm}: neutral targets (motors stay enabled, arms relax)")

    def cmd_pos(self, args: List[str]) -> None:
        # pos <side> <joint> <rad> [kp] [kd]
        if len(args) < 3 or len(args) > 5:
            print("Usage: pos <side> <joint> <rad> [kp] [kd]")
            return
        arm, index = self._parse_side_joint(args)
        if arm is None or index is None:
            return
        _, kind = self._joint_info(arm, index)
        try:
            position = float(args[2])
            kp = float(args[3]) if len(args) > 3 else (RP_KP if kind == "rp" else WRIST_KP)
            kd = float(args[4]) if len(args) > 4 else (RP_KD if kind == "rp" else WRIST_KD)
        except ValueError:
            print("Numeric arguments expected.")
            return
        if kind == "rp":
            command = rp_protocol.RpMitCommand(position, 0.0, kp, kd, 0.0)
        else:
            command = wrist_protocol.WristMitCommand(position, 0.0, kp, kd, 0.0)
        self._update_mit(arm, index, command)
        print(f"{self._joint_info(arm, index)[0]}: pos={position:.3f} kp={kp:g} kd={kd:g}")

    def cmd_vel(self, args: List[str]) -> None:
        # vel <side> <joint> <rad/s>
        if len(args) != 3:
            print("Usage: vel <side> <joint> <rad/s>")
            return
        arm, index = self._parse_side_joint(args)
        if arm is None or index is None:
            return
        _, kind = self._joint_info(arm, index)
        try:
            velocity = float(args[2])
        except ValueError:
            print("Numeric arguments expected.")
            return
        kd = RP_KD if kind == "rp" else WRIST_KD
        if kind == "rp":
            command = rp_protocol.RpMitCommand(0.0, velocity, 0.0, kd, 0.0)
        else:
            command = wrist_protocol.WristMitCommand(0.0, velocity, 0.0, kd, 0.0)
        self._update_mit(arm, index, command)
        print(f"{self._joint_info(arm, index)[0]}: vel={velocity:.3f} kd={kd:g} (kp=0)")

    def cmd_mit(self, args: List[str]) -> None:
        # mit <side> <joint> <pos> <vel> <kp> <kd> <ff>
        if len(args) != 7:
            print("Usage: mit <side> <joint> <pos> <vel> <kp> <kd> <ff>")
            return
        arm, index = self._parse_side_joint(args)
        if arm is None or index is None:
            return
        _, kind = self._joint_info(arm, index)
        try:
            values = [float(value) for value in args[2:]]
        except ValueError:
            print("Numeric arguments expected.")
            return
        if kind == "rp":
            command = rp_protocol.RpMitCommand(*values)
        else:
            command = wrist_protocol.WristMitCommand(*values)
        self._update_mit(arm, index, command)
        print(f"{self._joint_info(arm, index)[0]}: {values}")

    def cmd_status(self, args: List[str]) -> None:
        with self._lock:
            for arm, state in self._arms.items():
                print(f"{arm}: {'enabled' if state.enabled else 'disabled'}")
                for index in range(7):
                    name, kind = self._joint_info(arm, index)
                    if kind == "rp":
                        c = state.rp[index]
                        fields = (c.position, c.velocity, c.kp, c.kd, c.torque_ff)
                    else:
                        c = state.wrist[index - 4]
                        fields = (c.position, c.velocity, c.kp, c.kd, c.current_ff)
                    print(
                        f"  {index + 1} {name:<28} pos={fields[0]:+.3f} "
                        f"vel={fields[1]:+.3f} kp={fields[2]:.4g} "
                        f"kd={fields[3]:.4g} ff={fields[4]:+.4g}"
                    )

    def cmd_quit(self, args: List[str]) -> None:
        print("Stopping all motors...")
        with self._lock:
            for arm in self._arms:
                self._safe_stop_arm(arm)

    def cmd_action(self, args: List[str]) -> None:
        if not args:
            print("可用动作（输入 act <编号> 播放）：")
            for number, action in ACTIONS.items():
                print(f"  {number}: {action.name}（{action.duration:.0f}s）")
            return
        action = ACTIONS.get(args[0].strip())
        if action is None:
            print(f"未知动作编号 '{args[0]}'。输入 act 查看列表。")
            return
        with self._lock:
            if not any(state.enabled for state in self._arms.values()):
                print("电机未使能：先执行 enable 再播放动作。")
            self._action = Action(action.name, action.duration, action.fn)
        print(f"动作 {args[0]} [{action.name}] 开始播放（{action.duration:.0f}s），输入 stop 可提前结束")

    def cmd_stop(self, args: List[str]) -> None:
        with self._lock:
            if self._action is None:
                print("当前没有正在播放的动作。")
                return
            self._action = None
        print("动作已停止，保持在当前姿态（可继续手动指令）。")

    # -- helpers ---------------------------------------------------------------

    def _parse_side_joint(self, args: List[str]) -> Tuple[Optional[str], Optional[int]]:
        arm = args[0].strip().lower()
        if arm not in self._arms:
            print(f"Unknown side '{arm}'. Use left or right.")
            return None, None
        index = self._resolve_joint(args[1])
        if index is None:
            print(f"Unknown joint '{args[1]}'. Use 1..7 or a joint name.")
            return None, None
        return arm, index

    def run(self) -> None:
        commands = {
            "enable": self.cmd_enable,
            "disable": self.cmd_disable,
            "neutral": self.cmd_neutral,
            "pos": self.cmd_pos,
            "vel": self.cmd_vel,
            "mit": self.cmd_mit,
            "status": self.cmd_status,
            "act": self.cmd_action,
            "action": self.cmd_action,
            "stop": self.cmd_stop,
            "quit": self.cmd_quit,
        }
        stream = threading.Thread(target=self._stream_loop, daemon=True)
        stream.start()
        print("Keyboard commander ready. Type 'help' for commands.")
        try:
            while True:
                try:
                    line = input("> ")
                except EOFError:
                    print()
                    break
                tokens = shlex.split(line.strip())
                if not tokens:
                    continue
                verb = tokens[0].lower()
                if verb in ("help", "h", "?"):
                    print(HELP_TEXT)
                elif verb == "quit":
                    self.cmd_quit([])
                    break
                elif verb in commands:
                    commands[verb](tokens[1:])
                else:
                    print(f"Unknown command '{verb}'. Type 'help'.")
        except KeyboardInterrupt:
            print()
            self.cmd_quit([])
        finally:
            self._stop.set()
            stream.join(timeout=2.0)
            for bus in self._buses.values():
                bus.close()
            print("Keyboard commander stopped; motors were disabled.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Manual keyboard commander for the upper body over SocketCAN")
    parser.add_argument("--left-interface", default="vcan0")
    parser.add_argument("--right-interface", default="vcan1")
    parser.add_argument("--rate", type=float, default=50.0, help="command stream rate in Hz")
    parser.add_argument(
        "--allow-hardware",
        action="store_true",
        help="permit non-vcan interface names (DANGEROUS on a real robot)",
    )
    args = parser.parse_args()

    if args.rate <= 0.0:
        raise SystemExit("--rate must be positive")
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
    print(f"CAN interfaces: left={args.left_interface}, right={args.right_interface}, stream rate={args.rate} Hz")
    KeyboardCommander(buses, args.rate).run()


if __name__ == "__main__":
    main()
