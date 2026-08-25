"""通过有界关节空间 PD 直接播放 Gazebo 30 轴参考动作。"""

import argparse
import select
import sys
import time
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from .motion_library import (
    ACTIONS,
    JOINT_EFFORT_LIMITS,
    JOINT_ORDER,
    NEUTRAL,
    Action,
    JointVector,
    menu_text,
    smoothstep_interpolate,
)


def bounded_pd_effort(
    target: Sequence[float],
    position: Sequence[float],
    velocity: Sequence[float],
    *,
    kp: float,
    kd: float,
) -> List[float]:
    """按 30 轴 ABI 计算 PD effort，并逐轴限制到仿名义力矩。"""

    expected = len(JOINT_ORDER)
    if not all(len(values) == expected for values in (target, position, velocity)):
        raise ValueError("target、position、velocity 都必须包含 30 个值")
    if kp < 0.0 or kd < 0.0:
        raise ValueError("kp 和 kd 必须 >= 0")
    efforts: List[float] = []
    for joint, desired, actual, speed in zip(
        JOINT_ORDER, target, position, velocity
    ):
        effort = kp * (desired - actual) - kd * speed
        limit = JOINT_EFFORT_LIMITS[joint]
        efforts.append(min(max(effort, -limit), limit))
    return efforts


class RosActionPlayer(Node):
    """读取 Gazebo joint states，向 effort controller 输出 30 轴 PD 力矩。"""

    def __init__(
        self,
        *,
        command_topic: str,
        joint_states_topic: str,
        rate_hz: float,
        speed_scale: float,
        kp: float,
        kd: float,
    ) -> None:
        super().__init__("ros_action_player")
        self.period = 1.0 / rate_hz
        self.speed_scale = speed_scale
        self.kp = kp
        self.kd = kd
        self.current_target: JointVector = NEUTRAL
        self._joint_positions: Dict[str, float] = {}
        self._joint_velocities: Dict[str, float] = {}
        self._publisher = self.create_publisher(Float64MultiArray, command_topic, 10)
        self._subscription = self.create_subscription(
            JointState, joint_states_topic, self._on_joint_states, 50
        )

    def _on_joint_states(self, message: JointState) -> None:
        for index, name in enumerate(message.name):
            if index < len(message.position):
                self._joint_positions[name] = float(message.position[index])
            if index < len(message.velocity):
                self._joint_velocities[name] = float(message.velocity[index])

    def _ordered_state(self) -> Tuple[JointVector, JointVector]:
        positions = tuple(self._joint_positions[name] for name in JOINT_ORDER)
        velocities = tuple(self._joint_velocities.get(name, 0.0) for name in JOINT_ORDER)
        return positions, velocities

    def _publish_target(self, target: Sequence[float]) -> None:
        positions, velocities = self._ordered_state()
        message = Float64MultiArray()
        message.data = bounded_pd_effort(
            target,
            positions,
            velocities,
            kp=self.kp,
            kd=self.kd,
        )
        self._publisher.publish(message)

    def publish_zero_effort(self) -> None:
        message = Float64MultiArray()
        message.data = [0.0] * len(JOINT_ORDER)
        self._publisher.publish(message)

    def wait_for_initial_state(
        self, timeout_s: float
    ) -> Tuple[Optional[JointVector], List[str]]:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(
                self, timeout_sec=max(0.0, min(0.05, deadline - time.monotonic()))
            )
            missing = [name for name in JOINT_ORDER if name not in self._joint_positions]
            if not missing:
                return self._ordered_state()[0], []
        return None, [name for name in JOINT_ORDER if name not in self._joint_positions]

    def wait_for_controller(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._publisher.get_subscription_count() > 0:
                return True
            rclpy.spin_once(
                self, timeout_sec=max(0.0, min(0.05, deadline - time.monotonic()))
            )
        return False

    def start(self, initial_pose: JointVector) -> None:
        self.current_target = initial_pose
        print("已取得 Gazebo 的完整 30 轴状态；平滑进入零位。")
        self._transition_to(NEUTRAL, 1.8)
        self._hold(0.3)

    def _transition_to(self, target: JointVector, duration_s: float) -> None:
        duration = duration_s / self.speed_scale
        start_target = self.current_target
        start_time = time.monotonic()
        next_tick = start_time
        while rclpy.ok():
            now = time.monotonic()
            progress = (now - start_time) / duration
            if progress >= 1.0:
                self._publish_target(target)
                self.current_target = target
                return
            desired, _ = smoothstep_interpolate(
                start_target, target, progress, duration
            )
            self._publish_target(desired)
            self.current_target = desired
            rclpy.spin_once(self, timeout_sec=0.0)
            next_tick += self.period
            delay = next_tick - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()

    def _hold(self, duration_s: float) -> None:
        end = time.monotonic() + duration_s / self.speed_scale
        while rclpy.ok() and time.monotonic() < end:
            self._publish_target(self.current_target)
            rclpy.spin_once(
                self, timeout_sec=max(0.0, min(self.period, end - time.monotonic()))
            )

    def play(self, action: Action) -> None:
        print(f"\n开始 {action.name}")
        print(f"参考意义：{action.purpose}")
        for index, frame in enumerate(action.keyframes, start=1):
            print(f"  {index:02d}/{len(action.keyframes):02d}  {frame.label}")
            self._transition_to(frame.values, frame.duration)
            if frame.hold > 0.0:
                self._hold(frame.hold)
        print(f"完成 {action.name}；PD 将保持最后一个关键帧。")

    def idle_tick(self) -> None:
        self._publish_target(self.current_target)
        rclpy.spin_once(self, timeout_sec=0.0)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="直接驱动 Gazebo 的 30 轴动作验收工具")
    parser.add_argument("--command-topic", default="/effort_controller/commands")
    parser.add_argument("--joint-states-topic", default="/joint_states")
    parser.add_argument("--rate", type=float, default=100.0)
    parser.add_argument("--speed-scale", type=float, default=1.0)
    parser.add_argument("--kp", type=float, default=30.0)
    parser.add_argument("--kd", type=float, default=1.0)
    parser.add_argument("--feedback-timeout", type=float, default=5.0)
    parser.add_argument("--action", choices=tuple(ACTIONS))
    parser.add_argument("--list", action="store_true")
    return parser.parse_args()


def _interactive(player: RosActionPlayer) -> None:
    print("\n" + menu_text())
    print("\n输入动作名；m 显示菜单；q 回零并退出。")
    while rclpy.ok():
        print("动作> ", end="", flush=True)
        while True:
            readable, _, _ = select.select([sys.stdin], [], [], player.period)
            if readable:
                break
            player.idle_tick()
        line = sys.stdin.readline()
        if line == "":
            return
        command = line.strip()
        if command in ("m", "menu", "?"):
            print("\n" + menu_text())
            continue
        if command in ("q", "quit", "exit"):
            player._transition_to(NEUTRAL, 1.8)
            return
        action = ACTIONS.get(command)
        if action is None:
            print("未知动作；输入 m 查看动作名。")
            continue
        player.play(action)


def main() -> None:
    args = _parse_args()
    if args.list:
        print(menu_text())
        return
    if (
        args.rate <= 0.0
        or args.speed_scale <= 0.0
        or args.feedback_timeout <= 0.0
        or args.kp < 0.0
        or args.kd < 0.0
    ):
        raise SystemExit("rate、speed-scale、feedback-timeout 必须 > 0，kp/kd 必须 >= 0")

    rclpy.init()
    player = RosActionPlayer(
        command_topic=args.command_topic,
        joint_states_topic=args.joint_states_topic,
        rate_hz=args.rate,
        speed_scale=args.speed_scale,
        kp=args.kp,
        kd=args.kd,
    )
    try:
        if not player.wait_for_controller(args.feedback_timeout):
            raise RuntimeError("effort controller 尚未订阅；请先启动 gazebo.launch.py")
        initial_pose, missing = player.wait_for_initial_state(args.feedback_timeout)
        if initial_pose is None:
            raise RuntimeError("没有收到完整的 30 轴状态：" + ", ".join(missing))
        player.start(initial_pose)
        if args.action is None:
            _interactive(player)
        else:
            player.play(ACTIONS[args.action])
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，输出零力矩后退出。")
    finally:
        player.publish_zero_effort()
        player.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
