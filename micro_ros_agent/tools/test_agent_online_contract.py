#!/usr/bin/env python3
"""V3 在线桥接静态契约；无需 ROS 环境，不替代 MCU 编译或台架验证。"""
import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
FW = ROOT / "micro_ros_firmware"
MSG = ROOT / "micro_ros_agent/src/can_bridge_msgs/msg"
EXPECTED = {
    "/micro_ros_do/command": ("subscription", "can_bridge_msgs", "AgentCommand"),
    "/micro_ros_do/trajectory": ("subscription", "can_bridge_msgs", "AgentTrajectoryRequest"),
    "/micro_ros_do/config": ("subscription", "can_bridge_msgs", "AgentConfigRequest"),
    "/micro_ros_up/motion": ("publisher", "can_bridge_msgs", "AgentMotion"),
    "/micro_ros_up/status": ("publisher", "can_bridge_msgs", "AgentStatus"),
    "/micro_ros_up/info": ("publisher", "can_bridge_msgs", "AgentInfo"),
    "/micro_ros_up/imu/data": ("publisher", "sensor_msgs", "Imu"),
    "/micro_ros_up/imu/temperature": ("publisher", "sensor_msgs", "Temperature"),
}


def source(name):
    return (FW / "src" / name).read_text(encoding="utf-8")


class AgentOnlineContract(unittest.TestCase):
    def test_exact_eight_paths_types_and_entity_counts(self):
        contract = (MSG / "AgentWireContract.msg").read_text(encoding="utf-8")
        constants = dict(re.findall(r"string (TOPIC_\w+)=(/\S+)", contract))
        actual = {}
        count = 0
        for name in ("motor_bridge.cpp", "param_bridge.cpp", "imu_bridge.cpp"):
            text = source(name)
            literals = dict(re.findall(r'(?:char\s*\*|char\s+)\s*(\w+)(?:\[\])?\s*=\s*"([^"]+)"', text))
            pattern = r"rcl_(publisher|subscription)_init\(\s*&\w+,\s*\w+,\s*ROSIDL_GET_MSG_TYPE_SUPPORT\(\s*(\w+),\s*msg,\s*(\w+)\s*\),\s*([\w:]+|\"[^\"]+\")"
            for kind, package, message, topic in re.findall(pattern, text):
                count += 1
                if "__TOPIC_" in topic:
                    path = constants.get(topic.split("__")[-1])
                else:
                    path = literals.get(topic, topic.strip('"'))
                self.assertNotIn(path, actual, f"重复实体 {path}")
                actual[path] = (kind, package, message)
        self.assertEqual(actual, EXPECTED)
        self.assertEqual(count, 8)
        self.assertEqual(sum(value[0] == "subscription" for value in actual.values()), 3)
        self.assertEqual(sum(value[0] == "publisher" for value in actual.values()), 5)

    def test_no_service_client_action_or_legacy_entities(self):
        text = "\n".join(source(name) for name in (
            "motor_bridge.cpp", "param_bridge.cpp", "imu_bridge.cpp", "ros_bridge.cpp"))
        self.assertNotRegex(text, r"\b(?:rcl|rclc)_(?:service|client|action)\w*\s*\(")
        self.assertNotRegex(text, r"ROSIDL_GET_(?:SRV|ACTION)_TYPE_SUPPORT")
        self.assertNotRegex(text, r"ROSIDL_GET_MSG_TYPE_SUPPORT\([^)]*\b(?:LowCommand|LowState|MotorStatus|MotorPolicyArray|MotorTrajectory|MotorEvent|DeviceParamReport|ConfigResult|Float32)\b")

    def test_four_executor_handles(self):
        motor = source("motor_bridge.cpp")
        self.assertRegex(motor, r"motor_bridge_executor_handle_count\(void\)\s*\{\s*return 2U;")
        self.assertEqual(motor.count("rclc_executor_add_subscription("), 2)
        self.assertIn("#define PARAM_BRIDGE_EXECUTOR_HANDLES 1U", (FW / "include/param_bridge.h").read_text())
        self.assertIn("1U + motor_bridge_executor_handle_count()", source("ros_bridge.cpp"))

    def test_stop_and_deadlines_are_explicit(self):
        text = source("motor_bridge.cpp")
        self.assertIn("motor_rt_request_stop_all()", text)
        self.assertIn("valid_until_tick_ms", text)
        self.assertIn("agent_deadline_valid(", text)
        self.assertIn("agent_request_deadline(", text)
        self.assertIn("motor_rt_submit_agent_trajectory(", text)
        self.assertNotIn("wire_command_guard", text)

    def test_reliable_budget_and_info_retry_contract(self):
        text = source("motor_bridge.cpp")
        self.assertIn("serial_transport_send_begin()", text)
        self.assertIn("serial_transport_send_end()", text)
        self.assertIn("param_bridge_fill_status(", text)
        self.assertIn("param_bridge_info_sent(", text)
        self.assertIn("kReliableIntervalMs = 10U", text)
        self.assertIn("kStatusMinIntervalMs = 20U", text)
        self.assertIn("kStatusIntervalMs = 100U", text)
        self.assertIn("kMotionIntervalMs = 20U", text)
        self.assertRegex(text, r"g_consecutive_status\s*<\s*2U")

    def test_temperature_uses_actual_sample_stamp(self):
        text = source("imu_bridge.cpp")
        self.assertIn("sensor_msgs/msg/temperature.h", text)
        self.assertIn("fill_epoch_stamp(&g_temp_msg.header.stamp, metadata.epoch_ns)", text)
        self.assertIn("g_temp_msg.variance = 0", text)
        self.assertIn("sensor_task_get_temperature(", text)
        self.assertIn("metadata.sequence != g_temp_sequence", text)
        self.assertIn("1000U / FIRMWARE_TEMPERATURE_REPORT_HZ", text)
        self.assertIn("g_temp_tick = now", text)


if __name__ == "__main__":
    unittest.main()
