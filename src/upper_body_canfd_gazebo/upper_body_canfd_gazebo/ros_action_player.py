"""通过 ROS 2 position controller 播放双臂参考动作。"""

import argparse
import select
import sys
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from .motion_library import (
    ACTIONS,
    JOINT_NAMES,
    READY,
    Action,
    Pose,
    menu_text,
    pose_to_command,
    smoothstep_interpolate,
)


ZERO_ARM = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


class RosActionPlayer(Node):
    """读取 ros2_control 状态，并向 position controller 发布平滑位置目标。"""

    def __init__(
        self,
        *,
        command_topic: str,
        joint_states_topic: str,
        rate_hz: float,
        speed_scale: float,
    ) -> None:
        super().__init__("ros_action_player")
        self.period = 1.0 / rate_hz
        self.speed_scale = speed_scale
        self.current_pose = READY
        self._joint_positions: Dict[str, float] = {}
        self._publisher = self.create_publisher(Float64MultiArray, command_topic, 10)
        self._subscription = self.create_subscription(
            JointState, joint_states_topic, self._on_joint_states, 50
        )

    def _on_joint_states(self, message: JointState) -> None:
        for index, name in enumerate(message.name):
            if index < len(message.position):
                self._joint_positions[name] = float(message.position[index])

    def _publish_pose(self, pose: Pose) -> None:
        message = Float64MultiArray()
        message.data = pose_to_command(pose)
        self._publisher.publish(message)

    def wait_for_initial_pose(self, timeout_s: float) -> Tuple[Optional[Pose], List[str]]:
        """等待驱动侧 joint_state_broadcaster 给出完整的 14 关节状态。"""

        expected = [
            *(f"left_{name}_joint" for name in JOINT_NAMES),
            *(f"right_{name}_joint" for name in JOINT_NAMES),
        ]
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.05, deadline - time.monotonic()))
            if all(name in self._joint_positions for name in expected):
                values = [self._joint_positions[name] for name in expected]
                return Pose(tuple(values[:7]), tuple(values[7:])), []
        return None, [name for name in expected if name not in self._joint_positions]

    def wait_for_controller(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._publisher.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self, timeout_sec=min(0.05, deadline - time.monotonic()))
        return False

    def start(self, initial_pose: Pose) -> None:
        self.current_pose = initial_pose
        print("已取得 ros2_control 关节状态；先平滑进入统一准备姿态。")
        self._transition_to(READY, 1.8)
        self._hold(0.30)

    def _transition_to(self, target: Pose, duration_s: float) -> None:
        duration = duration_s / self.speed_scale
        start_pose = self.current_pose
        start_time = time.monotonic()
        next_tick = start_time
        while True:
            now = time.monotonic()
            u = (now - start_time) / duration
            if u >= 1.0:
                self._publish_pose(target)
                self.current_pose = target
                return
            position, _velocity = smoothstep_interpolate(start_pose, target, u, duration)
            self._publish_pose(position)
            self.current_pose = position
            rclpy.spin_once(self, timeout_sec=0.0)
            next_tick += self.period
            delay = next_tick - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()

    def _hold(self, duration_s: float) -> None:
        end = time.monotonic() + duration_s / self.speed_scale
        while time.monotonic() < end:
            self._publish_pose(self.current_pose)
            rclpy.spin_once(self, timeout_sec=min(self.period, end - time.monotonic()))

    def play(self, action: Action) -> None:
        print(f"\n开始 [{action.number}] {action.name}")
        print(f"参考意义：{action.purpose}")
        for index, frame in enumerate(action.frames, start=1):
            print(f"  {index:02d}/{len(action.frames):02d}  {frame.label}")
            self._transition_to(frame.pose, frame.duration)
            if frame.hold > 0.0:
                self._hold(frame.hold)
        print(f"完成 [{action.number}] {action.name}；position controller 将继续保持当前目标。")

    def idle_tick(self) -> None:
        rclpy.spin_once(self, timeout_sec=0.0)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="通过 ROS 2 控制器播放 14-DoF 双臂参考动作")
    parser.add_argument("--command-topic", default="/driver/position_controller/commands")
    parser.add_argument("--joint-states-topic", default="/driver/joint_states")
    parser.add_argument("--rate", type=float, default=100.0, help="ROS 目标刷新率，默认 100 Hz")
    parser.add_argument("--speed-scale", type=float, default=1.0)
    parser.add_argument("--feedback-timeout", type=float, default=3.0)
    parser.add_argument("--action", type=int, choices=sorted(ACTIONS))
    parser.add_argument("--list", action="store_true", help="只打印动作列表，不连接 ROS graph")
    return parser.parse_args()


def _interactive(player: RosActionPlayer) -> None:
    print("\n" + menu_text())
    print("\n直接输入数字后按 Enter；q 返回准备姿态并退出。")
    while True:
        print("动作编号> ", end="", flush=True)
        while True:
            readable, _, _ = select.select([sys.stdin], [], [], player.period)
            if readable:
                break
            player.idle_tick()
        line = sys.stdin.readline()
        if line == "":
            return
        command = line.strip().lower()
        if command in ("m", "menu", "?"):
            print("\n" + menu_text())
            continue
        if command in ("q", "quit", "exit"):
            player.play(ACTIONS[0])
            return
        try:
            action = ACTIONS.get(int(command))
        except ValueError:
            action = None
        if action is None:
            print("请输入 0~13；m 显示菜单；q 正常退出。")
            continue
        player.play(action)


def main() -> None:
    args = _parse_args()
    if args.list:
        print(menu_text())
        return
    if args.rate <= 0.0 or args.speed_scale <= 0.0 or args.feedback_timeout <= 0.0:
        raise SystemExit("--rate、--speed-scale 和 --feedback-timeout 必须 > 0")

    rclpy.init()
    player = RosActionPlayer(
        command_topic=args.command_topic,
        joint_states_topic=args.joint_states_topic,
        rate_hz=args.rate,
        speed_scale=args.speed_scale,
    )
    try:
        if not player.wait_for_controller(args.feedback_timeout):
            raise RuntimeError("position controller 尚未订阅命令 Topic；请先启动完整仿真回环")
        initial_pose, missing = player.wait_for_initial_pose(args.feedback_timeout)
        if initial_pose is None:
            raise RuntimeError("没有收到完整的 ros2_control 关节状态：" + ", ".join(missing))
        player.start(initial_pose)
        if args.action is not None:
            player.play(ACTIONS[args.action])
        else:
            _interactive(player)
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C；保留 position controller 当前保持目标后退出。")
    finally:
        player.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
