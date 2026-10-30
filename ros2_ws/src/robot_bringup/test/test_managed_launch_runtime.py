"""Real launch_testing smoke test: no Agent, CAN interface or serial device.

The production launch owns a child launch service. This lets its own Shutdown
event end the managed graph while launch_testing remains alive to inspect it;
otherwise the test runner would terminate that graph itself at test completion.
"""
from __future__ import annotations

import os
from pathlib import Path
import re
import signal
import tempfile
import time
import unittest

import launch
from launch.actions import ExecuteProcess
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
from launch_testing_ros.tools import basic_output_filter
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from robot_system_msgs.msg import RobotState
from robot_system_msgs.srv import AcquireControl


_DOMAIN_ID = 193
os.environ.setdefault('ROS_LOG_DIR', str(Path(tempfile.gettempdir()) / 'robot-bringup-launch-tests'))
_PROCESS_STARTED = re.compile(
    r'\[INFO\] \[([^\]]+)\]: process started with pid \[(\d+)\]')


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    package = Path(__file__).resolve().parents[1]
    model_dir = package.parent / 'robot_description'
    artifacts = tempfile.TemporaryDirectory(prefix='robot-managed-launch-')
    managed_launch = ExecuteProcess(
        cmd=[
            'ros2', 'launch', str(package / 'launch' / 'robot.launch.py'),
            'model:=half_body_robot', f'model_dir:={model_dir}',
            'robot_id:=launch-test-uncommissioned',
            'start_agent:=false', 'allow_trajectories:=false', 'start_web:=false',
        ],
        additional_env={
            'ROS_DOMAIN_ID': str(_DOMAIN_ID), 'ROS_LOCALHOST_ONLY': '1',
            'ROS_LOG_DIR': str(Path(artifacts.name) / 'ros-log'),
        },
        output='screen', sigterm_timeout='1', sigkill_timeout='1',
    )
    return launch.LaunchDescription([
        managed_launch, launch_testing.actions.ReadyToTest(),
    ]), {'managed_launch': managed_launch, 'artifacts': artifacts}


def _output(proc_output, process):
    return ''.join(event.text.decode('utf-8', errors='replace')
                   for event in proc_output[process])


class TestManagedRuntime(unittest.TestCase):
    def test_uncommissioned_graph_and_critical_exit(
            self, managed_launch, proc_info, proc_output):
        proc_info.assertWaitForStartup(managed_launch, timeout=5)
        context = Context()
        rclpy.init(args=[], context=context, domain_id=_DOMAIN_ID)
        probe = Node('managed_launch_test_probe', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(probe)
        states = []
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        subscription = probe.create_subscription(
            RobotState, '/robot/state', states.append, qos)
        client = probe.create_client(AcquireControl, '/robot/acquire_control')

        def wait_until(condition, timeout=7.):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.02)
                if condition():
                    return
            self.fail('managed launch did not reach its expected ROS graph/state')

        try:
            expected_nodes = {
                'calibration_validator', 'robot_supervisor', 'micro_ros_bridge',
                'firmware_topic_bridge', 'firmware_robot_state_publisher',
            }
            wait_until(lambda: expected_nodes <= set(probe.get_node_names())
                       and states and client.service_is_ready())
            self.assertNotIn('managed_micro_ros_agent', probe.get_node_names())
            self.assertFalse(states[-1].ready)
            self.assertFalse(states[-1].armed)
            self.assertEqual(states[-1].owner, '')
            self.assertTrue(any('calibration' in blocker for blocker in states[-1].blockers))

            future = client.call_async(AcquireControl.Request(
                owner='launch-test', mode=AcquireControl.Request.MANUAL, lease_ms=1000))
            wait_until(future.done, timeout=2.)
            self.assertFalse(future.result().success)
            self.assertEqual(future.result().lease_id, '')

            # Observe actual DDS endpoints, not just the launch description.
            old_topics = (
                '/motor/command_array', '/motor/policy_array', '/motor/trajectory',
                '/motor/control_lease', '/motor/param_set', '/motor/system_command',
            )
            managed_topics = (
                '/robot/managed/motor/command_array',
                '/robot/managed/motor/policy_array',
                '/robot/managed/motor/trajectory',
                '/robot/managed/motor/control_lease',
                '/robot/managed/maintenance/param_set',
                '/robot/managed/maintenance/system_command',
            )
            wait_until(lambda: all(any(info.node_name == 'micro_ros_bridge' for info in
                probe.get_subscriptions_info_by_topic(topic)) for topic in managed_topics))
            for topic in old_topics:
                self.assertNotIn('micro_ros_bridge', {
                    info.node_name for info in probe.get_subscriptions_info_by_topic(topic)})
            services = dict(probe.get_service_names_and_types())
            self.assertIn('/robot/managed/maintenance/execute_config', services)
            self.assertNotIn('/motor/execute_config', services)

            # The PID comes only from this child launch's own startup records.
            # Signal one managed process, never the outer launch: its production
            # OnProcessExit handler must end the remaining managed processes.
            proc_output.assertWaitFor('robot_supervisor-2', process=managed_launch,
                                      stream='stdout', timeout=2.,
                                      output_filter=basic_output_filter())
            started = {name: int(pid) for name, pid in
                       _PROCESS_STARTED.findall(_output(proc_output, managed_launch))}
            self.assertEqual(len(started), 5)
            target = next(pid for name, pid in started.items()
                          if name.startswith('robot_supervisor-'))
            command = Path(f'/proc/{target}/cmdline').read_bytes()
            self.assertIn(b'robot_supervisor', command)
            os.kill(target, signal.SIGINT)
            proc_info.assertWaitForShutdown(managed_launch, timeout=5.)
            output = _output(proc_output, managed_launch)
            for name in started:
                self.assertIn(f'[{name}]: process has finished cleanly', output)
            for pid in started.values():
                with self.assertRaises(ProcessLookupError):
                    os.kill(pid, 0)
        finally:
            probe.destroy_subscription(subscription)
            probe.destroy_client(client)
            executor.remove_node(probe)
            executor.shutdown()
            probe.destroy_node()
            if context.ok():
                context.shutdown()


@launch_testing.post_shutdown_test()
class TestManagedExit(unittest.TestCase):
    def test_child_launch_exits_cleanly(self, proc_info, managed_launch, artifacts):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=managed_launch, allowable_exit_codes=[0])
        artifacts.cleanup()
