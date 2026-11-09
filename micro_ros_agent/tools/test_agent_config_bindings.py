#!/usr/bin/env python3
"""固件配置线上接线门禁；行为测试另由真实 Coordinator/mailbox 宿主测试覆盖。"""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]


class ConfigBindings(unittest.TestCase):
    def test_bounded_v3_subscription(self):
        text = (ROOT / 'micro_ros_firmware/src/motor/param_bridge.cpp').read_text()
        self.assertIn('msg, AgentConfigRequest)', text)
        self.assertIn('qos_reliable(4)', text)
        self.assertNotIn('msg, ConfigRequest)', text)

    def test_explicit_ack_and_paginated_info(self):
        text = (ROOT / 'micro_ros_firmware/src/motor/param_bridge.cpp').read_text()
        self.assertIn('g_transactions.ack_result(', text)
        self.assertIn('g_transactions.copy_results(', text)
        self.assertIn('g_pager.request(', text)
        self.assertNotIn('g_transactions.ack();', text)

    def test_original_deadline_checked_before_kernel_submit(self):
        text = (ROOT / 'micro_ros_firmware/src/motor/motor_rt_task.cpp').read_text()
        start = text.index('got_config = control_mailbox_take_config')
        end = text.index('motor_control_submit_config(g_motor_control', start)
        self.assertIn('request_in.valid_until_tick_ms - now_ms', text[start:end])
        self.assertIn('motor_control_report_config_result', text[start:end])


if __name__ == '__main__':
    unittest.main()
