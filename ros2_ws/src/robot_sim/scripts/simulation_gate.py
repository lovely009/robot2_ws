#!/usr/bin/env python3
"""按 motor_sim 生命周期开关 Gazebo effort 输出门。"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class SimulationGate(Node):
    def __init__(self) -> None:
        super().__init__("motor_sim_gate")
        self._open = False
        self._publisher = self.create_publisher(
            Bool, "/motor_sim/control_gate", 10
        )
        self._subscription = self.create_subscription(
            String,
            "/motor_sim/drive_state",
            self._on_drive_state,
            10,
        )

    def _on_drive_state(self, message: String) -> None:
        should_open = message.data in {
            "ready_for_control",
            "motor_control_active",
        }
        if should_open == self._open:
            return
        self._open = should_open
        command = Bool()
        command.data = should_open
        self._publisher.publish(command)
        self.get_logger().info(
            "motor_sim effort gate " + ("opened" if should_open else "closed")
        )


def main() -> None:
    rclpy.init()
    node = SimulationGate()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
