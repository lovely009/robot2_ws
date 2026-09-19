#!/usr/bin/env python3
"""固件 ↔ ROS 标准话题适配桥（节点入口；转换与校验见 topic_bridge 模块）。

    上行  micro_ros_up/motion_states         → /joint_states
    下行  /arm_controller/joint_trajectory → micro_ros_do/motor_trajectory

固件接口是 ID 化、无关节名的私有话题（micro_ros_up/...）；ROS 生态用的是按关节名
索引的标准话题。ARD-0007 禁止 MCU 维护 joint_name，所以映射放在这一侧。

QoS：固件订阅是 best-effort depth 1，本节点的轨迹发布者必须同 QoS。
单位：固件发 SI（rad / rad·s⁻¹ / N·m），与标准消息约定一致，零换算。
"""

from motor_hardware_interface.topic_bridge import (BridgeError,
                                                  motion_states_to_joint_state,
                                                  parse_joint_map,
                                                  transpose_trajectory)


# ---------------------------------------------------------------------------
# 节点
# ---------------------------------------------------------------------------

def main():  # pragma: no cover - 需要 ROS 环境
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,
                           QoSDurabilityPolicy)
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory
    from can_bridge_msgs.msg import (MotorMotionStateArray, MotorTrajectory,
                                     MotorTrajectoryPoint)

    def stream_qos():
        """best-effort depth 1：与固件的订阅 QoS 对齐（不同就收不到）。"""
        return QoSProfile(depth=1,
                          reliability=QoSReliabilityPolicy.BEST_EFFORT,
                          durability=QoSDurabilityPolicy.VOLATILE,
                          history=QoSHistoryPolicy.KEEP_LAST)

    class FirmwareTopicBridge(Node):
        def __init__(self):
            super().__init__('firmware_topic_bridge')
            # 默认映射 = 现阶段实机：4 台，m1..m4 对应四个关节。
            self.declare_parameter('motor_ids', [1, 2, 3, 4])
            self.declare_parameter('joint_names',
                                   ['joint_link1', 'joint_link2',
                                    'joint_link3', 'joint_link4'])
            self.declare_parameter('motion_states_topic', 'micro_ros_up/motion_states')
            self.declare_parameter('joint_state_topic', '/joint_states')
            self.declare_parameter('trajectory_in_topic',
                                   '/arm_controller/joint_trajectory')
            self.declare_parameter('trajectory_out_topic',
                                   'micro_ros_do/motor_trajectory')

            ids = list(self.get_parameter('motor_ids').value)
            names = list(self.get_parameter('joint_names').value)
            self.joint_of_motor = parse_joint_map(ids, names)   # {id: joint}
            self.motor_of_joint = {j: i for i, j in self.joint_of_motor.items()}

            self.js_pub = self.create_publisher(
                JointState, self.get_parameter('joint_state_topic').value, 10)
            self.traj_pub = self.create_publisher(
                MotorTrajectory,
                self.get_parameter('trajectory_out_topic').value, stream_qos())

            self.create_subscription(
                MotorMotionStateArray,
                self.get_parameter('motion_states_topic').value,
                self.on_motion_states, stream_qos())
            self.create_subscription(
                JointTrajectory,
                self.get_parameter('trajectory_in_topic').value,
                self.on_trajectory, 10)

            self.sequence = 0
            self.get_logger().info(
                '桥已就绪：%s → %s ； %s → %s（映射 %s）'
                % (self.get_parameter('motion_states_topic').value,
                   self.get_parameter('joint_state_topic').value,
                   self.get_parameter('trajectory_in_topic').value,
                   self.get_parameter('trajectory_out_topic').value,
                   self.joint_of_motor))

        # ---- 上行 ----
        def on_motion_states(self, msg):
            states = [{'motor_id': s.motor_id, 'position': s.position,
                       'velocity': s.velocity, 'effort': s.effort}
                      for s in msg.states]
            names, pos, vel, eff = motion_states_to_joint_state(
                states, self.joint_of_motor)
            if not names:
                return   # 一个可用的都没有：不发空消息（发空会让 rviz 把关节清零）
            out = JointState()
            out.header.stamp = self.get_clock().now().to_msg()
            out.name, out.position, out.velocity, out.effort = names, pos, vel, eff
            self.js_pub.publish(out)

        # ---- 下行 ----
        def on_trajectory(self, msg):
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
                target_ids, flat, warnings = transpose_trajectory(
                    list(msg.joint_names), samples, self.motor_of_joint)
            except BridgeError as exc:
                # 关键：把固件会**静默丢弃**的失败在这里讲清楚。
                self.get_logger().error('轨迹被本地校验拒绝：%s' % exc)
                return
            for w in warnings:
                self.get_logger().warn(w)

            out = MotorTrajectory()
            out.sync_time_ns = self.get_clock().now().nanoseconds
            self.sequence += 1
            out.sequence = self.sequence
            out.trajectory_id = self.sequence
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
            self.get_logger().info('轨迹已转发：%d 台 / %d 点 / 共 %d ms'
                                   % (len(target_ids), len(flat),
                                      flat[-1][0] if flat else 0))

    rclpy.init()
    node = FirmwareTopicBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
