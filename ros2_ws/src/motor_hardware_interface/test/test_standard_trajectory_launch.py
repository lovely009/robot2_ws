"""Official ros2_control/JTC Action acceptance against GenericSystem only.

This launches the mock-only launch with an immutable mock-only xacro.
It never opens CAN, serial or a micro-ROS Agent, and never implements an Action
server in the test. Goal acceptance, interpolation, feedback, result and cancel
are supplied by the installed joint_trajectory_controller.
"""
from __future__ import annotations

import os
from pathlib import Path
import time
import unittest

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('launch_testing')
pytest.importorskip('control_msgs.action')
pytest.importorskip('controller_manager_msgs.srv')

from ament_index_python.packages import get_package_prefix, PackageNotFoundError
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents, ListHardwareInterfaces
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
from launch_testing.actions import ReadyToTest
import rclpy
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from trajectory_msgs.msg import JointTrajectoryPoint


for _package in ('controller_manager', 'joint_trajectory_controller',
                 'joint_state_broadcaster', 'robot_state_publisher', 'xacro'):
    try:
        get_package_prefix(_package)
    except PackageNotFoundError:
        pytest.skip(f'ROS integration dependency is not installed: {_package}', allow_module_level=True)

HERE = Path(__file__).parent
NAMESPACE = '/ecosystem_trajectory'
MANAGER = NAMESPACE + '/robot_manager'
ACTION = NAMESPACE + '/joint_trajectory_controller/follow_joint_trajectory'
JOINTS = ['ecosystem_joint1', 'ecosystem_joint2']


@pytest.mark.launch_test
def generate_test_description():
    log_directory = os.environ.get('ROS_LOG_DIR', str(Path.cwd() / 'work' / 'ros2-ecosystem-tests'))
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(HERE.parent / 'launch' / 'mock_control.launch.py')),
        launch_arguments={
            'model_xacro': str(HERE / 'fixtures' / 'standard_trajectory_mock.urdf.xacro'),
            'motor_config_file': str(HERE / 'fixtures' / 'standard_trajectory_mock.yaml'),
            'control_mode': 'trajectory',
            'controller_manager_name': MANAGER,
            'shutdown_on_manager_exit': 'true',
            'trajectory_path_tolerance': '0.2',
            'trajectory_goal_tolerance': '0.02',
            'trajectory_goal_time': '1.0',
            'trajectory_stopped_velocity_tolerance': '0.01',
        }.items(),
    )
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '194'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        SetEnvironmentVariable('ROS_LOG_DIR', log_directory),
        stack,
        ReadyToTest(),
    ])


class TestStandardTrajectory(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.context = Context()
        rclpy.init(args=[], context=cls.context, domain_id=194)
        cls.node = rclpy.create_node('standard_trajectory_acceptance', context=cls.context)
        cls.executor = SingleThreadedExecutor(context=cls.context)
        cls.executor.add_node(cls.node)
        cls.action = ActionClient(cls.node, FollowJointTrajectory, ACTION)

    @classmethod
    def tearDownClass(cls):
        cls.action.destroy()
        cls.executor.remove_node(cls.node)
        cls.node.destroy_node()
        cls.executor.shutdown()
        cls.context.shutdown()

    def wait(self, condition, timeout=15.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=.01)
            if condition():
                return
        self.fail('timed out waiting for standard ros2_control graph or Action response')

    def future(self, future, timeout=15.):
        self.wait(future.done, timeout)
        return future.result()

    def call(self, service_type, name):
        client = self.node.create_client(service_type, name)
        try:
            self.wait(client.service_is_ready)
            return self.future(client.call_async(service_type.Request()))
        finally:
            self.node.destroy_client(client)

    def controllers_ready(self):
        deadline = time.monotonic() + 20.
        while time.monotonic() < deadline:
            response = self.call(ListControllers, MANAGER + '/list_controllers')
            if any(c.name == 'joint_trajectory_controller' and c.state == 'active'
                   for c in response.controller):
                return response
            self.executor.spin_once(timeout_sec=.05)
        self.fail('official JointTrajectoryController did not become active')

    def test_01_only_standard_jtc_claims_mock_position_interfaces(self):
        controllers = self.controllers_ready().controller
        active_writers = [c for c in controllers if c.state == 'active' and c.claimed_interfaces]
        self.assertEqual([c.name for c in active_writers], ['joint_trajectory_controller'])
        self.assertEqual(active_writers[0].type, 'joint_trajectory_controller/JointTrajectoryController')
        expected = {joint + '/position' for joint in JOINTS}
        self.assertEqual(set(active_writers[0].claimed_interfaces), expected)
        self.assertFalse(any(c.name == 'position_controller' and c.state == 'active'
                             for c in controllers))
        hardware = self.call(ListHardwareComponents, MANAGER + '/list_hardware_components').component
        self.assertEqual(len(hardware), 1)
        self.assertEqual(hardware[0].class_type, 'mock_components/GenericSystem')
        self.assertEqual(hardware[0].state.label, 'active')
        interfaces = self.call(ListHardwareInterfaces, MANAGER + '/list_hardware_interfaces')
        self.assertEqual({i.name for i in interfaces.command_interfaces if i.is_claimed}, expected)

    def test_02_standard_action_reports_feedback_and_success(self):
        self.controllers_ready()
        self.wait(self.action.server_is_ready)
        feedback = []
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        goal.trajectory.points = [JointTrajectoryPoint(
            positions=[.15, -.1], velocities=[0., 0.], time_from_start=Duration(sec=1))]
        started = time.monotonic()
        handle = self.future(self.action.send_goal_async(
            goal, feedback_callback=lambda message: feedback.append(message.feedback)))
        self.assertTrue(handle.accepted)
        result = self.future(handle.get_result_async(), timeout=5.)
        self.assertEqual(result.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(result.result.error_code, FollowJointTrajectory.Result.SUCCESSFUL)
        self.assertGreater(time.monotonic() - started, .5)
        self.assertTrue(feedback)
        sample = feedback[-1]
        self.assertEqual(list(sample.joint_names), JOINTS)
        self.assertEqual(len(sample.actual.positions), len(JOINTS))
        self.assertEqual(len(sample.error.positions), len(JOINTS))
        for actual, target in zip(sample.actual.positions, (.15, -.1)):
            self.assertAlmostEqual(actual, target, delta=.025)

    def test_03_standard_action_cancellation_is_acknowledged(self):
        self.controllers_ready()
        self.wait(self.action.server_is_ready)
        feedback = []
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        goal.trajectory.points = [JointTrajectoryPoint(
            positions=[-.4, .3], velocities=[0., 0.], time_from_start=Duration(sec=4))]
        handle = self.future(self.action.send_goal_async(
            goal, feedback_callback=lambda message: feedback.append(message.feedback)))
        self.assertTrue(handle.accepted)
        self.wait(lambda: len(feedback) >= 2, timeout=2.)
        cancel = self.future(handle.cancel_goal_async(), timeout=2.)
        self.assertEqual(len(cancel.goals_canceling), 1)
        self.assertEqual(list(cancel.goals_canceling[0].goal_id.uuid), list(handle.goal_id.uuid))
        result = self.future(handle.get_result_async(), timeout=3.)
        self.assertEqual(result.status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(result.result.error_code, FollowJointTrajectory.Result.SUCCESSFUL)


@launch_testing.post_shutdown_test()
class TestStandardTrajectoryShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
