"""Schema 6 原生线契约：真实类型生成、37 轴边界和单帧/可靠 CDR 预算。"""
import unittest

import can_bridge_msgs.msg as messages
import can_bridge_msgs.srv as services
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Imu, Temperature


class AgentWireContractTest(unittest.TestCase):
    def require_msg(self, name):
        self.assertTrue(hasattr(messages, name), f"缺少 Schema 6 消息类型 {name}")
        return getattr(messages, name)

    def require_srv(self, name):
        self.assertTrue(hasattr(services, name), f"缺少 Schema 6 服务类型 {name}")
        return getattr(services, name)

    def test_schema_and_native_services_exist(self):
        self.assertEqual(self.require_msg('MotorSystemState').SCHEMA_VERSION, 6)
        self.assertEqual(self.require_msg('MotorDeviceInfo').SCHEMA_VERSION, 6)
        self.assertEqual(self.require_msg('SystemCommand').COMMAND_SYNC_W25, 6)
        for srv_name in ('SubmitPolicy', 'ExecuteConfig', 'GetDeviceInfo'):
            self.require_srv(srv_name)

    def test_bounds(self):
        cmd_cls = self.require_msg('MotorCommandArray')
        cmd_target = self.require_msg('MotorCommandTarget')
        cmd = cmd_cls()
        cmd.targets = [cmd_target() for _ in range(37)]
        recovered = deserialize_message(serialize_message(cmd), cmd_cls)
        self.assertEqual(len(recovered.targets), 37)
        with self.assertRaises(AssertionError):
            cmd.targets = [cmd_target() for _ in range(38)]

        motion_cls = self.require_msg('MotorMotionStateArray')
        motion_target = self.require_msg('MotorMotionStateTarget')
        motion = motion_cls()
        motion.states = [motion_target() for _ in range(37)]
        with self.assertRaises(AssertionError):
            motion.states = [motion_target() for _ in range(38)]

        state_cls = self.require_msg('MotorSystemState')
        ctrl_state = self.require_msg('MotorControlState')
        state = state_cls()
        state.control_states = [ctrl_state() for _ in range(37)]
        with self.assertRaises(AssertionError):
            state.control_states = [ctrl_state() for _ in range(38)]

        cfg_cls = self.require_msg('ConfigRequest')
        param_target = self.require_msg('ParamSetTarget')
        cfg = cfg_cls()
        cfg.targets = [param_target() for _ in range(16)]
        with self.assertRaises(AssertionError):
            cfg.targets = [param_target() for _ in range(17)]

    def test_max_cdr(self):
        from measure_agent_wire import maximum_message, wire_budget
        best_effort, reliable = wire_budget()
        for name in ('MotorCommandArray', 'MotorMotionStateArray',
                     'MotorSystemState', 'MotorDeviceInfo', 'MotorEvent', 'ConfigRequest'):
            cls = self.require_msg(name)
            value = maximum_message(cls)
            data = serialize_message(value)
            limit = best_effort if name in ('MotorCommandArray', 'MotorMotionStateArray') else reliable
            with self.subTest(message=name, size=len(data), limit=limit):
                self.assertLessEqual(len(data), limit)
                recovered = deserialize_message(data, cls)
                self.assertEqual(recovered, value)
                self.assertEqual(len(serialize_message(recovered)), len(data))
        for cls in (Imu, Temperature):
            self.assertLessEqual(len(serialize_message(maximum_message(cls))), best_effort)


if __name__ == '__main__':
    unittest.main()
