"""V3 线上契约：真实类型生成、满容量序列化和有界接收约束。"""
import unittest

import can_bridge_msgs.msg as messages
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Imu, Temperature


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

    def test_max_cdr(self):
        self.require_type('AgentCommand')
        from measure_agent_wire import maximum_message, wire_budget
        best_effort, reliable = wire_budget()
        for name in ('AgentCommand', 'AgentMotion', 'AgentTrajectoryRequest',
                     'AgentConfigRequest', 'AgentStatus', 'AgentInfo'):
            cls = self.require_type(name)
            value = maximum_message(cls)
            data = serialize_message(value)
            limit = best_effort if name in ('AgentCommand', 'AgentMotion') else reliable
            with self.subTest(message=name, size=len(data), limit=limit):
                self.assertLessEqual(len(data), limit)
                recovered = deserialize_message(data, cls)
                # CDR 对齐填充字节不保证初始化；校验字段与长度，不比较填充内容。
                self.assertEqual(recovered, value)
                self.assertEqual(len(serialize_message(recovered)), len(data))
        for cls in (Imu, Temperature):
            self.assertLessEqual(len(serialize_message(maximum_message(cls))), best_effort)


if __name__ == '__main__':
    unittest.main()
