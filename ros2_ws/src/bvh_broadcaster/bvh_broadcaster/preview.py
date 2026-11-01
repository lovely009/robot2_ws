"""Play validated joint-side motion on a dedicated preview topic."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .bvh import playback_frame
from .retarget import load_motion


class RobotPreview(Node):
    def __init__(self):
        super().__init__('mocap_robot_preview')
        self.declare_parameter('motion_file', '')
        self.declare_parameter('urdf', '')
        names, self.positions, self.dt = load_motion(
            self.get_parameter('motion_file').value, self.get_parameter('urdf').value)
        self.message = JointState()
        self.message.name = names
        self.publisher = self.create_publisher(JointState, '/mocap_robot/joint_states', 10)
        self.started = self.get_clock().now()
        self.finished = False
        self.timer = self.create_timer(self.dt, self.publish_frame)
        self.get_logger().info(f'Preview only: {len(self.positions)} frames, {len(names)} joints')
        self.publish_frame()

    def publish_frame(self):
        now = self.get_clock().now()
        elapsed = (now - self.started).nanoseconds / 1e9
        if elapsed < 0:
            self.started, elapsed, self.finished = now, 0., False
        index = playback_frame(elapsed, self.dt, len(self.positions), 1., False)
        self.message.header.stamp = now.to_msg()
        self.message.position = self.positions[index].tolist()
        self.publisher.publish(self.message)
        if not self.finished and elapsed >= len(self.positions) * self.dt:
            self.get_logger().info('Preview finished; holding last frame.')
            self.finished = True


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RobotPreview()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
