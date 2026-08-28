"""通过有界 PD effort 播放 UR5e Gazebo 六轴验收动作。"""

import argparse
import math
import time
from typing import List, Sequence

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from .motion_library import (
    ACTIONS,
    HOME,
    JOINT_EFFORT_LIMITS,
    JOINT_ORDER,
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
    """计算六轴 joint-space PD，并按仿名义 effort 上限逐轴裁剪。"""

    if not all(len(values) == 6 for values in (target, position, velocity)):
        raise ValueError("target、position、velocity 都必须包含六个值")
    if kp < 0.0 or kd < 0.0:
        raise ValueError("kp 和 kd 必须 >= 0")
    efforts = []
    for joint, desired, actual, speed in zip(
        JOINT_ORDER, target, position, velocity
    ):
        effort = kp * (desired - actual) - kd * speed
        limit = JOINT_EFFORT_LIMITS[joint]
        efforts.append(min(max(effort, -limit), limit))
    return efforts


def validate_joint_state(
    names: Sequence[str],
    positions: Sequence[float],
    velocities: Sequence[float],
    *,
    age_s: float,
    stale_after_s: float,
) -> None:
    """拒绝 ABI 顺序不符、非有限或过期的反馈，避免错轴输出力矩。"""

    if tuple(names) != JOINT_ORDER or len(positions) != 6 or len(velocities) != 6:
        raise RuntimeError("joint-state name/order mismatch")
    if not all(math.isfinite(value) for value in (*positions, *velocities)):
        raise RuntimeError("joint-state contains nonfinite values")
    if age_s > stale_after_s:
        raise RuntimeError("joint-state feedback is stale")


def validate_command_graph(
    controller_subscriptions: int, effort_publishers: int
) -> None:
    """确保 controller 已连接，且本播放器是唯一 effort 发布者。"""

    if controller_subscriptions < 1:
        raise RuntimeError("effort controller subscription is missing")
    if effort_publishers > 1:
        raise RuntimeError("second effort publisher detected")


class RosActionPlayer(Node):
    def __init__(
        self,
        *,
        command_topic: str,
        joint_states_topic: str,
        rate_hz: float,
        kp: float,
        kd: float,
        stale_after_s: float,
    ) -> None:
        super().__init__("robot_arm_actions")
        self.command_topic = command_topic
        self.period = 1.0 / rate_hz
        self.kp = kp
        self.kd = kd
        self.stale_after_s = stale_after_s
        self.current_target: JointVector = HOME
        self._positions: JointVector | None = None
        self._velocities: JointVector | None = None
        self._feedback_time = 0.0
        self._publisher = self.create_publisher(Float64MultiArray, command_topic, 10)
        self._subscription = self.create_subscription(
            JointState, joint_states_topic, self._on_joint_states, 50
        )

    def _on_joint_states(self, message: JointState) -> None:
        velocities = message.velocity if message.velocity else [0.0] * len(message.name)
        validate_joint_state(
            message.name,
            message.position,
            velocities,
            age_s=0.0,
            stale_after_s=self.stale_after_s,
        )
        self._positions = tuple(float(value) for value in message.position)
        self._velocities = tuple(float(value) for value in velocities)
        self._feedback_time = time.monotonic()

    def _validate_ready(self) -> None:
        validate_command_graph(
            self._publisher.get_subscription_count(),
            self.count_publishers(self.command_topic),
        )
        if self._positions is None or self._velocities is None:
            raise RuntimeError("joint-state feedback is missing")
        validate_joint_state(
            JOINT_ORDER,
            self._positions,
            self._velocities,
            age_s=time.monotonic() - self._feedback_time,
            stale_after_s=self.stale_after_s,
        )

    def _publish_target(self, target: Sequence[float]) -> None:
        self._validate_ready()
        assert self._positions is not None and self._velocities is not None
        message = Float64MultiArray()
        message.data = bounded_pd_effort(
            target, self._positions, self._velocities, kp=self.kp, kd=self.kd
        )
        self._publisher.publish(message)

    def publish_zero_effort(self) -> None:
        message = Float64MultiArray()
        message.data = [0.0] * 6
        self._publisher.publish(message)

    def wait_until_ready(self, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        last_error = "尚未收到完整反馈"
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.05, deadline - time.monotonic()))
            try:
                self._validate_ready()
                self.current_target = self._positions or HOME
                return
            except RuntimeError as error:
                last_error = str(error)
        raise RuntimeError(last_error)

    def _transition_to(self, target: JointVector, duration_s: float) -> None:
        start = self.current_target
        started = time.monotonic()
        while rclpy.ok():
            progress = (time.monotonic() - started) / duration_s
            desired = smoothstep_interpolate(start, target, progress)
            self._publish_target(desired)
            self.current_target = desired
            if progress >= 1.0:
                return
            rclpy.spin_once(self, timeout_sec=self.period)

    def play(self, action: Action) -> None:
        for target, duration in zip(action.targets, action.durations):
            self._transition_to(target, duration)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="UR5e Gazebo 六轴动作验收工具")
    parser.add_argument(
        "--command-topic", default="/robot_arm/effort_controller/commands"
    )
    parser.add_argument("--joint-states-topic", default="/robot_arm/joint_states")
    parser.add_argument("--rate", type=float, default=100.0)
    parser.add_argument("--kp", type=float, default=30.0)
    parser.add_argument("--kd", type=float, default=1.0)
    parser.add_argument("--feedback-timeout", type=float, default=5.0)
    parser.add_argument("--stale-after", type=float, default=0.15)
    parser.add_argument("--action", choices=tuple(ACTIONS), default="home")
    parser.add_argument("--list", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    if args.list:
        print(menu_text())
        return
    if min(args.rate, args.feedback_timeout, args.stale_after) <= 0.0 or min(
        args.kp, args.kd
    ) < 0.0:
        raise SystemExit("rate/timeout 必须 > 0，kp/kd 必须 >= 0")
    rclpy.init()
    player = RosActionPlayer(
        command_topic=args.command_topic,
        joint_states_topic=args.joint_states_topic,
        rate_hz=args.rate,
        kp=args.kp,
        kd=args.kd,
        stale_after_s=args.stale_after,
    )
    try:
        player.wait_until_ready(args.feedback_timeout)
        player.play(ACTIONS[args.action])
    finally:
        # 正常结束和异常路径都解除 effort，避免遗留最后一帧命令。
        player.publish_zero_effort()
        player.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
