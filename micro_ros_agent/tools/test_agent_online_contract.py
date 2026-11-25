#!/usr/bin/env python3
"""Schema 6 原生桥接静态契约；无需 ROS 环境，不替代 MCU 编译或台架验证。"""
import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
FW = ROOT / "micro_ros_firmware"
EXPECTED_TOPICS = {
    "/motor/command": ("subscription", "can_bridge_msgs", "MotorCommandArray"),
    "/control/stop": ("subscription", "std_msgs", "Empty"),
    "/client_count": ("subscription", "std_msgs", "Int32"),
    "/motor/motion": ("publisher", "can_bridge_msgs", "MotorMotionStateArray"),
    "/motor/state": ("publisher", "can_bridge_msgs", "MotorSystemState"),
    "/motor/events": ("publisher", "can_bridge_msgs", "MotorEvent"),
    "/imu/data": ("publisher", "sensor_msgs", "Imu"),
    "/imu/temp_raw": ("publisher", "sensor_msgs", "Temperature"),
}
EXPECTED_SERVICES = {
    "/motor/submit_policy": ("can_bridge_msgs", "SubmitPolicy"),
    "/motor/execute_config": ("can_bridge_msgs", "ExecuteConfig"),
    "/motor/get_device_info": ("can_bridge_msgs", "GetDeviceInfo"),
}

FW_SOURCES = {
    "motor_bridge.cpp": "src/motor/motor_bridge.cpp",
    "param_bridge.cpp": "src/motor/param_bridge.cpp",
    "imu_bridge.cpp": "src/sensor/imu_bridge.cpp",
    "ros_bridge.cpp": "src/micro_ros/ros_bridge.cpp",
}


def source(name):
    return (FW / FW_SOURCES[name]).read_text(encoding="utf-8")


class AgentOnlineContract(unittest.TestCase):
    def test_exact_native_topics_and_services(self):
        actual_topics = {}
        for name in ("motor_bridge.cpp", "imu_bridge.cpp"):
            text = source(name)
            pattern = r'rcl_(publisher|subscription)_init\(\s*&\w+,\s*\w+,\s*ROSIDL_GET_MSG_TYPE_SUPPORT\(\s*(\w+),\s*msg,\s*(\w+)\s*\),\s*"([^"]+)"'
            for kind, package, message, topic in re.findall(pattern, text):
                self.assertNotIn(topic, actual_topics, f"重复话题 {topic}")
                actual_topics[topic] = (kind, package, message)
        self.assertEqual(actual_topics, EXPECTED_TOPICS)

        param_text = source("param_bridge.cpp")
        srv_pattern = r'rcl_service_init\(\s*&\w+,\s*\w+,\s*ROSIDL_GET_SRV_TYPE_SUPPORT\(\s*(\w+),\s*srv,\s*(\w+)\s*\),\s*"([^"]+)"'
        actual_services = {
            srv: (pkg, name)
            for pkg, name, srv in re.findall(srv_pattern, param_text)
        }
        self.assertEqual(actual_services, EXPECTED_SERVICES)

    def test_rclc_lifecycle_enabled(self):
        ros_text = source("ros_bridge.cpp")
        self.assertIn("rclc_make_node_a_lifecycle_node(", ros_text)
        self.assertIn("rclc_lifecycle_init_get_state_server(", ros_text)
        self.assertIn("rclc_lifecycle_change_state(", ros_text)

    def test_stop_and_deadlines_are_explicit(self):
        text = source("motor_bridge.cpp")
        self.assertIn("motor_rt_request_stop_all()", text)
        self.assertIn("kCommandTimeoutMs", text)
        self.assertNotIn("wire_command_guard", text)

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
