"""Actual DDS publication, latching and file-change invalidation; no hardware."""
from __future__ import annotations

import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from robot_calibration.node import CalibrationValidator
from robot_system_msgs.msg import CalibrationState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from test_validation import accepted, check, files


@pytest.fixture
def context():
    rclpy.init(domain_id=192)
    yield
    rclpy.shutdown()


def _wait(executor, predicate):
    deadline = time.monotonic() + 4.
    while not predicate() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=.02)
    assert predicate(), 'calibration publication did not arrive'


def test_latched_accepted_state_and_periodic_file_invalidation(files, context):
    assert check(files, accepted(files)).valid
    overrides = [Parameter(name, value=value) for name, value in {
        'driver_file': str(files[0]), 'urdf_file': str(files[1]),
        'manifest_file': str(files[2]), 'model_name': 'two_axes',
        'robot_id': 'assembly-001',
    }.items()]
    validator = CalibrationValidator(parameter_overrides=overrides)
    validator._timer.cancel()  # The next subscriber can only receive the latched first report.
    observer = rclpy.create_node('calibration_report_observer')
    executor = SingleThreadedExecutor()
    received = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    observer.create_subscription(CalibrationState, '/robot/calibration_state', received.append, qos)
    executor.add_node(validator)
    executor.add_node(observer)
    try:
        _wait(executor, lambda: bool(received))
        assert received[-1].valid and received[-1].commissioned
        assert list(received[-1].motor_ids) == [1, 2]
        assert list(received[-1].bus_indices) == [1, 1]
        files[2].unlink()
        validator._timer.reset()
        _wait(executor, lambda: any(not state.valid for state in received))
        assert received[-1].blockers
        assert received[-1].header.stamp.sec > 0
    finally:
        executor.shutdown()
        validator.destroy_node()
        observer.destroy_node()


def test_empty_parameters_publish_blocked_state_instead_of_starting_motion(context):
    node = CalibrationValidator()
    try:
        assert node._last_blockers
        assert any('model_name' in reason for reason in node._last_blockers)
    finally:
        node.destroy_node()


def test_standard_diagnostics_follow_commissioning_and_file_changes(files, context):
    assert check(files, accepted(files)).valid
    overrides = [Parameter(name, value=value) for name, value in {
        'driver_file': str(files[0]), 'urdf_file': str(files[1]),
        'manifest_file': str(files[2]), 'model_name': 'two_axes',
        'robot_id': 'assembly-001',
    }.items()]
    node = CalibrationValidator(parameter_overrides=overrides)
    observer = rclpy.create_node('calibration_diagnostic_observer')
    executor = SingleThreadedExecutor()
    received = []
    observer.create_subscription(DiagnosticArray, '/diagnostics', received.append, 10)
    executor.add_node(node)
    executor.add_node(observer)
    try:
        _wait(executor, lambda: bool(received))
        status = received[-1].status[0]
        assert status.name == 'calibration_validator/commissioning'
        assert status.level == DiagnosticStatus.OK
        assert status.hardware_id == 'assembly-001'
        files[2].unlink()
        _wait(executor, lambda: any(array.status[0].level == DiagnosticStatus.WARN
                                  for array in received))
        status = received[-1].status[0]
        assert 'manifest_file' in status.message or 'cannot read' in status.message
        assert {value.key: value.value for value in status.values}['valid'] == 'false'
        assert received[-1].header.stamp.sec > 0
    finally:
        executor.shutdown()
        node.destroy_node()
        observer.destroy_node()
