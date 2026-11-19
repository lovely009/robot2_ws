"""线上契约：真实类型生成、有界接收及序列化往返约束。"""
import unittest

import can_bridge_msgs.msg as messages
from rclpy.serialization import deserialize_message, serialize_message


class AgentWireContractTest(unittest.TestCase):
    def require_type(self, name):
        self.assertTrue(hasattr(messages, name), f"缺少 V3 生成类型 {name}")
        return getattr(messages, name)

    def test_topic_set_exact(self):
        contract = self.require_type('AgentWireContract')
        topics = {getattr(contract, name) for name in dir(contract)
                  if name.startswith('TOPIC_')}
        self.assertEqual(topics, {
            '/micro_ros_do/command', '/micro_ros_do/trajectory',
            '/micro_ros_do/config', '/micro_ros_up/motion',
            '/micro_ros_up/status', '/micro_ros_up/info',
            '/micro_ros_up/imu/data', '/micro_ros_up/imu/temperature'})

    def test_bounds(self):
        command = self.require_type('AgentCommand')()
        command.motor_ids = list(range(48))
        command.position_q = [32767] * 48
        command.velocity_q = [-32767] * 48
        command.effort_q = [0] * 48
        recovered = deserialize_message(serialize_message(command), type(command))
        self.assertEqual(list(recovered.motor_ids), list(range(48)))
        with self.assertRaises(AssertionError):
            command.motor_ids = list(range(49))
        motion = self.require_type('AgentMotion')()
        motion.motor_ids = list(range(32))
        with self.assertRaises(AssertionError):
            motion.motor_ids = list(range(33))
        info = self.require_type('AgentInfo')()
        entry = self.require_type('AgentDeviceInfo')
        info.devices = [entry() for _ in range(16)]
        with self.assertRaises(AssertionError):
            info.devices = [entry() for _ in range(17)]
        status = self.require_type('AgentStatus')()
        result = self.require_type('AgentConfigResult')
        status.results = [result() for _ in range(8)]
        with self.assertRaises(AssertionError):
            status.results = [result() for _ in range(9)]
        trajectory = self.require_type('AgentTrajectoryRequest')()
        point = messages.MotorTrajectoryPoint
        trajectory.points = [point() for _ in range(32)]
        with self.assertRaises(AssertionError):
            trajectory.points = [point() for _ in range(33)]
        self.assertEqual(self.require_type('AgentWireContract').TRAJECTORY_POINTS_MAX, 256)



if __name__ == '__main__':
    unittest.main()
