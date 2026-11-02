#!/usr/bin/env python3
"""只读反馈桥：将解码后的 /motor/* 状态转换为 /joint_states 与 TF 输入。

运动请求由 workbench_control 提交给 ros2_control；本节点没有下行发布者。
"""

from motor_hardware_interface.topic_bridge import (motion_states_to_joint_state,
                                                  motor_accepts_joint_target,
                                                  parse_joint_map)
from motor_hardware_interface.transmission_bridge import TransmissionBridge
from motor_hardware_interface.topic_bridge import BridgeError
from motor_hardware_interface.control_state import ControlStateTracker
import json
import math
import signal
import time


# ---------------------------------------------------------------------------
# 节点
# ---------------------------------------------------------------------------

def main():  # pragma: no cover - 需要 ROS 环境
    import rclpy
    from rclpy.node import Node
    from rclpy.duration import Duration
    from rclpy.parameter import Parameter
    from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
    from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,
                           QoSDurabilityPolicy)
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from can_bridge_msgs.msg import (MotorMotionStateArray, MotorStatusArray, MotorControlStateArray)

    def stream_qos():
        """best-effort depth 1：与固件运动状态流 QoS 对齐。"""
        return QoSProfile(depth=1,
                          reliability=QoSReliabilityPolicy.BEST_EFFORT,
                          durability=QoSDurabilityPolicy.VOLATILE,
                          history=QoSHistoryPolicy.KEEP_LAST)

    class FirmwareTopicBridge(Node):
        def __init__(self):
            super().__init__('firmware_topic_bridge')
            self.declare_parameter('motor_ids', Parameter.Type.INTEGER_ARRAY)
            self.declare_parameter('joint_names', Parameter.Type.STRING_ARRAY)
            immutable = ParameterDescriptor(read_only=True)
            if self.declare_parameter('enable_trajectory', False).value:
                raise ValueError('只读反馈桥不接受运动请求，请使用 host_trajectory.launch.py')
            self.declare_parameter('joint_limits_json', '')
            self.declare_parameter('publish_joint_states', True, immutable)
            self.declare_parameter('driver_file', '', immutable)
            self.declare_parameter('transmissions_file', '', immutable)
            self.declare_parameter('urdf_file', '', immutable)
            self.declare_parameter('model_name', '', immutable)
            self.declare_parameter('feedback_timeout_s', .5, immutable)
            self.declare_parameter('feedback_max_skew_s', .05, immutable)
            self.declare_parameter('joint_state_topic', '/joint_states', immutable)
            driver_file = self.get_parameter('driver_file').value
            self.transmission = None
            self.motor_health = {}
            self.control_states = ControlStateTracker()
            self.last_health = 0.
            if driver_file:
                self.transmission = TransmissionBridge(
                    driver_file, self.get_parameter('transmissions_file').value,
                    self.get_parameter('urdf_file').value,
                    self.get_parameter('feedback_timeout_s').value,
                    self.get_parameter('feedback_max_skew_s').value)
                self.joint_of_motor = {}
                self.motor_of_joint = {}
            else:
                ids = list(self.get_parameter('motor_ids').value)
                names = list(self.get_parameter('joint_names').value)
                self.joint_of_motor = parse_joint_map(ids, names)
                self.motor_of_joint = {j: i for i, j in self.joint_of_motor.items()}

            self.joint_limits_from_parameter(self.get_parameter('joint_limits_json').value)
            self.add_on_set_parameters_callback(self.validate_parameters)

            status_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                    reliability=QoSReliabilityPolicy.RELIABLE)
            self.status_pub = self.create_publisher(String, '/robot_transmission/status', status_qos)
            self.create_timer(1., self.publish_status)
            self.create_subscription(MotorControlStateArray, '/motor/control_states',
                                     self.on_control_states, 10)
            self.create_subscription(MotorStatusArray, '/motor/status_array',
                                     self.on_health, 10)

            self.js_pub = self.create_publisher(
                JointState, self.get_parameter('joint_state_topic').value, 10)
            self.create_subscription(
                MotorMotionStateArray,
                '/motor/motion_states',
                self.on_motion_states, stream_qos())
            self.get_logger().info('只读关节反馈桥已就绪')

        def joint_limits_from_parameter(self, value):
            if self.transmission:
                return self.transmission.joint_limits_from_json(
                    value, self.get_parameter('model_name').value)
            if value not in ('', '{}'):
                raise BridgeError('当前未加载传动模型，不能设置关节限位')
            return {}

        def validate_parameters(self, parameters):
            try:
                for parameter in parameters:
                    if parameter.name == 'enable_trajectory' and parameter.value:
                        raise BridgeError('只读反馈桥不接受运动请求')
                    if parameter.name == 'joint_limits_json':
                        self.joint_limits_from_parameter(parameter.value)
            except BridgeError as exc:
                return SetParametersResult(successful=False, reason=str(exc))
            return SetParametersResult(successful=True)

        def publish_status(self):
            limits = self.joint_limits_from_parameter(self.get_parameter('joint_limits_json').value)
            msg = String()
            msg.data = json.dumps({
                'model': self.get_parameter('model_name').value,
                'driver_file': self.get_parameter('driver_file').value,
                'trajectory_enabled': False,
                'control_state_source': 'mcu' if self.control_states.seen else 'legacy',
                'control_service': self.get_fully_qualified_name() + '/set_parameters_atomically',
                'last_error': '',
                'joint_names': (list(self.transmission.model.joint_names) if self.transmission
                                else list(self.motor_of_joint)),
                'joint_limits': {name: {'lower': lower if math.isfinite(lower) else None,
                                        'upper': upper if math.isfinite(upper) else None}
                                 for name, (lower, upper) in limits.items()},
            })
            self.status_pub.publish(msg)

        def on_control_states(self, msg):
            previous_session = self.control_states.session
            self.control_states.update(msg.schema_version, msg.session_id,
                                       msg.sequence, msg.states, time.monotonic())
            if self.transmission:
                if previous_session != self.control_states.session:
                    self.transmission.held.clear()
                for group in self.transmission.model.groups:
                    if not all(self.control_states.ready(self.transmission.ids[a],
                               time.monotonic(), self.get_parameter('feedback_timeout_s').value)
                               and self.control_states.states[self.transmission.ids[a]].execution_state not in (5, 7)
                               for a in group.actuators):
                        for joint in group.joints:
                            self.transmission.held.pop(joint, None)

        def on_health(self, msg):
            self.last_health = time.monotonic()
            self.motor_health = {s.motor_id: motor_accepts_joint_target(
                s.online, s.lifecycle, s.unified_fault_code) for s in msg.statuses}
            active = {s.motor_id for s in msg.statuses if s.online
                      and s.lifecycle == s.LIFECYCLE_ACTIVE and s.unified_fault_code == 0}
            if self.transmission and not self.control_states.seen:
                for group in self.transmission.model.groups:
                    if not all(self.transmission.ids[a] in active for a in group.actuators):
                        for joint in group.joints:
                            self.transmission.held.pop(joint, None)

        # ---- 上行 ----
        def on_motion_states(self, msg):
            monotonic_now = time.monotonic()
            states = [{'motor_id': s.motor_id, 'position': s.position,
                       'velocity': s.velocity, 'effort': s.effort,
                       'state_age_ms': s.state_age_ms}
                      for s in msg.states]
            if self.transmission:
                joints = self.transmission.update(states, monotonic_now)
                names = list(joints)
                pos = [joints[j].position for j in names]
                vel = [joints[j].velocity for j in names]
                eff = [joints[j].effort for j in names]
            else:
                names, pos, vel, eff = motion_states_to_joint_state(states, self.joint_of_motor)
            if not names or not self.get_parameter('publish_joint_states').value:
                return   # 一个可用的都没有：不发空消息（发空会让 rviz 把关节清零）
            out = JointState()
            stamp = self.get_clock().now()
            if self.transmission and self.transmission.state_stamp is not None:
                stamp -= Duration(seconds=max(0., monotonic_now-self.transmission.state_stamp))
            out.header.stamp = stamp.to_msg()
            out.name, out.position, out.velocity, out.effort = names, pos, vel, eff
            self.js_pub.publish(out)

    rclpy.init()
    node = FirmwareTopicBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # launch 与进程组可能重复转发 SIGINT，清理阶段只执行一次。
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
