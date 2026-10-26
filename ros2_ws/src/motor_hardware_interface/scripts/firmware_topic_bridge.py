#!/usr/bin/env python3
"""标准 ROS 话题桥：消费 micro_ros_bridge 解码后的 /motor/* 状态并发布 /joint_states。

轨迹下行仍从 /arm_controller/joint_trajectory 转发到 /motor_command/trajectory；
MCU 私有上行协议由 micro_ros_bridge 负责解码，关节映射留在本节点。

QoS：运动状态是 best-effort depth 1；轨迹是一条可分片的大消息，本节点发布到
/motor_command/trajectory，由 command_streamer 按当前接线转发到直连或重力补偿入口。
轨迹发布者必须 reliable，不能复用状态流 QoS。
单位：固件发模组输出轴 SI（rad / rad·s⁻¹ / N·m）；传动模型换算为 URDF 关节坐标。
"""

from motor_hardware_interface.topic_bridge import (BridgeError,
                                                  motion_states_to_joint_state,
                                                  motor_accepts_joint_target,
                                                  parse_joint_map,
                                                  split_trajectory)
from motor_hardware_interface.transmission_bridge import TransmissionBridge
from motor_hardware_interface.control_state import ControlStateTracker
import json
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
    from rcl_interfaces.msg import ParameterDescriptor
    from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,
                           QoSDurabilityPolicy)
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from trajectory_msgs.msg import JointTrajectory
    from can_bridge_msgs.msg import (MotorMotionStateArray, MotorStatusArray, MotorControlStateArray, MotorTrajectory,
                                     MotorTrajectoryPoint)

    def stream_qos():
        """best-effort depth 1：与固件运动状态流 QoS 对齐。"""
        return QoSProfile(depth=1,
                          reliability=QoSReliabilityPolicy.BEST_EFFORT,
                          durability=QoSDurabilityPolicy.VOLATILE,
                          history=QoSHistoryPolicy.KEEP_LAST)

    def trajectory_qos():
        """reliable depth 1：MotorTrajectory 允许 XRCE 分片传输。"""
        return QoSProfile(depth=1,
                          reliability=QoSReliabilityPolicy.RELIABLE,
                          durability=QoSDurabilityPolicy.VOLATILE,
                          history=QoSHistoryPolicy.KEEP_LAST)

    class FirmwareTopicBridge(Node):
        def __init__(self):
            super().__init__('firmware_topic_bridge')
            self.declare_parameter('motor_ids', Parameter.Type.INTEGER_ARRAY)
            self.declare_parameter('joint_names', Parameter.Type.STRING_ARRAY)
            immutable = ParameterDescriptor(read_only=True)
            self.declare_parameter('enable_trajectory', False)
            self.declare_parameter('publish_joint_states', True, immutable)
            self.declare_parameter('driver_file', '', immutable)
            self.declare_parameter('transmissions_file', '', immutable)
            self.declare_parameter('urdf_file', '', immutable)
            self.declare_parameter('model_name', '', immutable)
            self.declare_parameter('feedback_timeout_s', .5, immutable)
            self.declare_parameter('feedback_max_skew_s', .05, immutable)
            self.declare_parameter('joint_state_topic', '/joint_states', immutable)
            self.declare_parameter('trajectory_in_topic',
                                   '/arm_controller/joint_trajectory', immutable)
            self.declare_parameter('trajectory_out_topic',
                                   '/motor_command/trajectory', immutable)

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
            self.traj_pub = self.create_publisher(
                MotorTrajectory,
                self.get_parameter('trajectory_out_topic').value, trajectory_qos())
            self.create_subscription(
                JointTrajectory,
                self.get_parameter('trajectory_in_topic').value,
                self.on_trajectory, 10)
            self.create_subscription(MotorTrajectory,
                self.get_parameter('trajectory_out_topic').value,
                self.on_motor_trajectory, trajectory_qos())

            self.sequence = 0
            self.last_trajectory_error = ''
            self.get_logger().info(
                '桥已就绪：%s → %s（映射 %s，轨迹转发 %s）'
                % ('/motor/motion_states',
                   self.get_parameter('joint_state_topic').value,
                   self.joint_of_motor,
                   self.get_parameter('enable_trajectory').value))

        def publish_status(self):
            msg = String()
            msg.data = json.dumps({
                'model': self.get_parameter('model_name').value,
                'driver_file': self.get_parameter('driver_file').value,
                'trajectory_enabled': self.get_parameter('enable_trajectory').value,
                'control_state_source': 'mcu' if self.control_states.seen else 'legacy',
                'control_service': self.get_fully_qualified_name() + '/set_parameters_atomically',
                'last_error': self.last_trajectory_error,
                'trajectory_topic': self.get_parameter('trajectory_out_topic').value,
                'joint_names': (list(self.transmission.model.joint_names) if self.transmission
                                else list(self.motor_of_joint)),
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

        def on_motor_trajectory(self, msg):
            if self.transmission and msg.count == 0:
                self.transmission.held.clear()

        # ---- 下行 ----
        def on_trajectory(self, msg):
            if not self.get_parameter('enable_trajectory').value:
                if self.transmission:
                    self.transmission.held.clear()
                self.last_trajectory_error = '主机传动桥为只读模式'
                self.publish_status()
                return
            self.last_trajectory_error = ''
            if not msg.joint_names and not msg.points:
                if self.transmission:
                    self.transmission.held.clear()
                self.sequence += 1
                out = MotorTrajectory()
                out.sequence = self.sequence
                out.trajectory_id = self.sequence
                out.segment_count = 1
                self.traj_pub.publish(out)
                self.publish_status()
                return
            samples = []
            for p in msg.points:
                samples.append({
                    't_ms': p.time_from_start.sec * 1000
                            + p.time_from_start.nanosec // 1000000,
                    'positions': list(p.positions),
                    'velocities': list(p.velocities) or None,
                    'effort': list(p.effort) or None,
                })
            try:
                # 本节点另有一个取消观察订阅，不能把它当作主机路由已连接。
                if self.traj_pub.get_subscription_count() <= 1:
                    raise BridgeError('主机轨迹路由未连接，拒绝轨迹')
                if self.transmission:
                    required = {self.transmission.ids[a] for j in msg.joint_names
                                for a in self.transmission.model.actuators_for_joint(j)}
                    now = time.monotonic()
                    timeout = self.get_parameter('feedback_timeout_s').value
                    if self.control_states.seen:
                        if not all(self.control_states.ready(i, now, timeout) for i in required):
                            raise BridgeError('MCU 控制状态未激活、无效或反馈过期')
                    elif (now-self.last_health > timeout
                            or not all(self.motor_health.get(i, False) for i in required)):
                        raise BridgeError('传动组未全部就绪/健康，或健康反馈过期')
                    target_ids, flat, warnings = self.transmission.trajectory(
                        list(msg.joint_names), samples, time.monotonic())
                    segments = [flat]
                else:
                    target_ids, segments, warnings = split_trajectory(
                        list(msg.joint_names), samples, self.motor_of_joint)
            except (BridgeError, ValueError, RuntimeError) as exc:
                self.last_trajectory_error = str(exc)
                self.publish_status()
                # 关键：把固件会**静默丢弃**的失败在这里讲清楚。
                self.get_logger().error('轨迹被本地校验拒绝：%s' % exc)
                return
            for w in warnings:
                self.get_logger().warn(w)

            self.sequence += 1
            trajectory_id = self.sequence
            sync_time_ns = self.get_clock().now().nanoseconds
            segment_count = len(segments)
            for segment_index, flat in enumerate(segments):
                out = MotorTrajectory()
                out.sync_time_ns = sync_time_ns
                out.sequence = self.sequence
                out.trajectory_id = trajectory_id
                out.segment_index = segment_index
                out.segment_count = segment_count
                out.count = len(flat)
                out.target_count = len(target_ids)
                out.target_ids = target_ids
                out.points = []
                for (t_ms, motor_id, p, v, e) in flat:
                    pt = MotorTrajectoryPoint()
                    pt.time_from_start_ms = t_ms
                    pt.motor_id = motor_id
                    pt.position, pt.velocity, pt.effort = p, v, e
                    out.points.append(pt)
                self.traj_pub.publish(out)
            self.publish_status()
            flat = segments[-1]
            self.get_logger().info('轨迹已转发：%d 台 / %d 点 / %d 段 / 共 %d ms'
                                   % (len(target_ids),
                                      sum(len(segment) for segment in segments),
                                      segment_count,
                                      flat[-1][0] if flat else 0))

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
