"""通过终端输入编号播放双臂参考动作，并始终走真实的私有 CAN-FD 编解码链路。"""

import argparse
import select
import sys
import time
from typing import Dict, List, Optional, Tuple

from . import protocol_2530 as wrist_protocol
from . import protocol_rp as rp_protocol
from .motion_library import ACTIONS, JOINT_NAMES, READY, Action, Pose, menu_text, smoothstep_interpolate
from .socketcan import SocketCanBus


ZERO_ARM = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
ZERO_VELOCITY = Pose(ZERO_ARM, ZERO_ARM)


class CanFdActionPlayer:
    """维持 100 Hz 左右的 MIT 指令流，并在关键帧之间做平滑轨迹。"""

    def __init__(
        self,
        left_interface: str,
        right_interface: str,
        *,
        rate_hz: float,
        speed_scale: float,
        rp_kp: float,
        rp_kd: float,
        wrist_kp: float,
        wrist_kd: float,
    ) -> None:
        self.buses: Dict[str, SocketCanBus] = {
            "left": SocketCanBus(left_interface),
            "right": SocketCanBus(right_interface),
        }
        self.period = 1.0 / rate_hz
        self.speed_scale = speed_scale
        self.rp_kp = rp_kp
        self.rp_kd = rp_kd
        self.wrist_kp = wrist_kp
        self.wrist_kd = wrist_kd
        self.current_pose = READY
        self._enabled = False

    def close(self) -> None:
        """无论正常退出还是 Ctrl+C，都先发中性目标，再使能关闭。"""

        for bus in self.buses.values():
            try:
                self._safe_stop_bus(bus)
            finally:
                bus.close()
        self._enabled = False

    @staticmethod
    def _safe_stop_bus(bus: SocketCanBus) -> None:
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

    def emergency_stop(self) -> None:
        for bus in self.buses.values():
            self._safe_stop_bus(bus)
        self._enabled = False

    def _enable(self) -> None:
        for bus in self.buses.values():
            bus.send(rp_protocol.ENABLE_ID, rp_protocol.pack_enable(True), force_fd=False)
            for node in (1, 2, 3):
                # 手册要求切换 MIT 模式时先停止，再进入运行状态。
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
        self._enabled = True

    def _send_pose(self, position: Pose, velocity: Pose = ZERO_VELOCITY) -> None:
        for side in ("left", "right"):
            bus = self.buses[side]
            q = position.left if side == "left" else position.right
            dq = velocity.left if side == "left" else velocity.right

            rp_commands = [
                rp_protocol.RpMitCommand(
                    position=q[index],
                    velocity=dq[index],
                    kp=self.rp_kp,
                    kd=self.rp_kd,
                    torque_ff=0.0,
                )
                for index in range(4)
            ]
            bus.send(
                rp_protocol.MIT_BROADCAST_ID,
                rp_protocol.pack_broadcast(rp_commands),
                force_fd=True,
            )

            for node in (1, 2, 3):
                index = 3 + node
                command = wrist_protocol.WristMitCommand(
                    position=q[index],
                    velocity=dq[index],
                    kp=self.wrist_kp,
                    kd=self.wrist_kd,
                    current_ff=0.0,
                )
                bus.send(
                    wrist_protocol.mit_command_id(node),
                    wrist_protocol.pack_mit_command(command),
                    force_fd=True,
                )

            # 仿真电机的状态回复数量较多。持续清空接收队列，避免长时间播放时缓存堆积。
            bus.recv_available(max_frames=256)

    def probe_pose(self, timeout_s: float = 0.8) -> Tuple[Optional[Pose], List[str]]:
        """在使能之前用零增益请求 14 个电机反馈，取得当前实际起点。"""

        values: Dict[str, List[Optional[float]]] = {
            "left": [None] * 7,
            "right": [None] * 7,
        }

        # 零 Kp/Kd/前馈意味着请求反馈时不会主动施加力矩。
        for bus in self.buses.values():
            bus.send(
                rp_protocol.MIT_BROADCAST_ID,
                rp_protocol.pack_broadcast([rp_protocol.RpMitCommand()] * 4),
                force_fd=True,
            )
            for node in (1, 2, 3):
                bus.send(
                    wrist_protocol.mit_command_id(node),
                    wrist_protocol.pack_mit_command(wrist_protocol.WristMitCommand()),
                    force_fd=True,
                )

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            for side, bus in self.buses.items():
                for frame in bus.recv_available(max_frames=256):
                    if frame.is_extended_id:
                        continue
                    if 1 <= frame.arbitration_id <= 4 and len(frame.data) == 12:
                        state = rp_protocol.unpack_state(frame.data)
                        values[side][frame.arbitration_id - 1] = state.position
                        continue
                    for node in (1, 2, 3):
                        if (
                            frame.arbitration_id == wrist_protocol.mit_reply_id(node)
                            and len(frame.data) == 12
                        ):
                            reply = wrist_protocol.unpack_mit_reply(frame.data)
                            values[side][3 + node] = reply.position
                            break

            if all(value is not None for side in values.values() for value in side):
                left = tuple(float(value) for value in values["left"])
                right = tuple(float(value) for value in values["right"])
                return Pose(left, right), []  # type: ignore[arg-type]
            time.sleep(0.002)

        missing: List[str] = []
        for side in ("left", "right"):
            for index, value in enumerate(values[side]):
                if value is None:
                    missing.append(f"{side}_{JOINT_NAMES[index]}")
        return None, missing

    def start(self, initial_pose: Pose) -> None:
        self.current_pose = initial_pose
        self._enable()
        print("已取得当前关节位置；先平滑进入统一准备姿态。")
        self._transition_to(READY, 1.8, "进入准备姿态")
        self._hold(0.30)

    def _transition_to(self, target: Pose, duration_s: float, label: str = "") -> None:
        duration = duration_s / self.speed_scale
        start_pose = self.current_pose
        start_time = time.monotonic()
        next_tick = start_time

        while True:
            now = time.monotonic()
            u = (now - start_time) / duration
            if u >= 1.0:
                self._send_pose(target, ZERO_VELOCITY)
                self.current_pose = target
                return

            position, velocity = smoothstep_interpolate(start_pose, target, u, duration)
            self._send_pose(position, velocity)
            self.current_pose = position

            next_tick += self.period
            delay = next_tick - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()

    def _hold(self, duration_s: float) -> None:
        duration = duration_s / self.speed_scale
        end = time.monotonic() + duration
        next_tick = time.monotonic()
        while time.monotonic() < end:
            self._send_pose(self.current_pose, ZERO_VELOCITY)
            next_tick += self.period
            delay = next_tick - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()

    def play(self, action: Action) -> None:
        print(f"\n开始 [{action.number}] {action.name}")
        print(f"参考意义：{action.purpose}")
        for index, frame in enumerate(action.frames, start=1):
            print(f"  {index:02d}/{len(action.frames):02d}  {frame.label}")
            self._transition_to(frame.pose, frame.duration, frame.label)
            if frame.hold > 0.0:
                self._hold(frame.hold)
        print(f"完成 [{action.number}] {action.name}；当前姿态继续保持。")

    def idle_tick(self) -> None:
        # 等待用户考虑下一条编号时仍持续发 MIT 目标，不能触发桥接器的 0.15 s 超时。
        self._send_pose(self.current_pose, ZERO_VELOCITY)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="按终端编号播放 14-DoF 双臂 CAN-FD 参考动作")
    parser.add_argument("--left-interface", default="vcan0")
    parser.add_argument("--right-interface", default="vcan1")
    parser.add_argument("--rate", type=float, default=100.0, help="CAN 指令刷新率，默认 100 Hz")
    parser.add_argument(
        "--speed-scale",
        type=float,
        default=1.0,
        help="动作速度倍率；0.5 更慢，2.0 更快，默认 1.0",
    )
    parser.add_argument("--rp-kp", type=float, default=32.0)
    parser.add_argument("--rp-kd", type=float, default=2.2)
    parser.add_argument("--wrist-kp", type=float, default=0.008)
    parser.add_argument("--wrist-kd", type=float, default=1.0)
    parser.add_argument("--feedback-timeout", type=float, default=0.8)
    parser.add_argument("--skip-feedback-check", action="store_true")
    parser.add_argument("--action", type=int, choices=sorted(ACTIONS), help="非交互模式：只播放一个编号")
    parser.add_argument("--list", action="store_true", help="只打印动作列表，不打开 CAN")
    parser.add_argument(
        "--allow-hardware",
        action="store_true",
        help="允许非 vcan 接口；真机必须先完成零位、方向、限位和急停验证",
    )
    return parser.parse_args()


def _validate_args(args: argparse.Namespace) -> None:
    if args.rate <= 0.0:
        raise SystemExit("--rate 必须 > 0")
    if args.speed_scale <= 0.0:
        raise SystemExit("--speed-scale 必须 > 0")
    if args.feedback_timeout <= 0.0:
        raise SystemExit("--feedback-timeout 必须 > 0")
    if not args.allow_hardware:
        for interface in (args.left_interface, args.right_interface):
            if not interface.startswith("vcan"):
                raise SystemExit(
                    f"拒绝向真实接口 {interface!r} 发送参考动作。"
                    "仿真请使用 vcan；真机完成零位/方向/限位验证后才可显式加 --allow-hardware。"
                )


def _interactive(player: CanFdActionPlayer) -> None:
    print("\n" + menu_text())
    print("\n直接输入数字后按 Enter。等待输入期间机器人仍保持当前姿态。")
    prompt_needed = True

    while True:
        if prompt_needed:
            print("动作编号> ", end="", flush=True)
            prompt_needed = False

        # 用 select 做带超时的终端读取，而不是阻塞 input()；超时时继续刷新 MIT 指令。
        readable, _, _ = select.select([sys.stdin], [], [], player.period)
        if not readable:
            player.idle_tick()
            continue

        line = sys.stdin.readline()
        if line == "":
            print("\n终端输入已关闭，退出。")
            return
        command = line.strip().lower()

        if command in ("m", "menu", "?"):
            print("\n" + menu_text())
            prompt_needed = True
            continue
        if command in ("q", "quit", "exit"):
            print("平滑返回准备姿态后退出……")
            player.play(ACTIONS[0])
            return
        if command in ("x", "stop", "estop"):
            print("立即失能。")
            player.emergency_stop()
            return

        try:
            number = int(command)
        except ValueError:
            print("请输入 0~13；m 显示菜单；q 正常退出；x 立即失能。")
            prompt_needed = True
            continue

        action = ACTIONS.get(number)
        if action is None:
            print("没有这个动作编号，请输入 0~13。")
            prompt_needed = True
            continue

        player.play(action)
        prompt_needed = True


def main() -> None:
    args = _parse_args()
    if args.list:
        print(menu_text())
        return
    _validate_args(args)

    if args.allow_hardware:
        print(
            "警告：已允许非 vcan 接口。这里的动作角度以 Gazebo URDF 零位/正方向为准，"
            "不能未经逐关节标定直接当作真机动作库。"
        )

    player = CanFdActionPlayer(
        args.left_interface,
        args.right_interface,
        rate_hz=args.rate,
        speed_scale=args.speed_scale,
        rp_kp=args.rp_kp,
        rp_kd=args.rp_kd,
        wrist_kp=args.wrist_kp,
        wrist_kd=args.wrist_kd,
    )

    try:
        initial_pose, missing = player.probe_pose(args.feedback_timeout)
        if initial_pose is None:
            if not args.skip_feedback_check:
                preview = ", ".join(missing[:5])
                if len(missing) > 5:
                    preview += f" ... 共缺 {len(missing)} 个关节"
                raise RuntimeError(
                    "没有收到完整的仿真电机反馈（"
                    + preview
                    + "）。请先启动 sim.launch.py，并确认 vcan0/vcan1；"
                    "仅在明确知道仿真初始姿态时才使用 --skip-feedback-check。"
                )
            print("警告：跳过启动反馈检查，按 Gazebo 默认初始零位进入准备姿态。")
            initial_pose = Pose(ZERO_ARM, ZERO_ARM)

        player.start(initial_pose)
        if args.action is not None:
            player.play(ACTIONS[args.action])
            player._hold(0.8)
        else:
            _interactive(player)
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，执行失能退出。")
    finally:
        player.close()


if __name__ == "__main__":
    main()
