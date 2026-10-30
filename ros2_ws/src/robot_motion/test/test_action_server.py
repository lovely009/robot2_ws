"""Actual ROS Action transport against a fake MCU in an isolated DDS domain."""
from __future__ import annotations

import copy
import time
import uuid

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from can_bridge_msgs.msg import MotorMotionStateArray, MotorMotionStateTarget, MotorTrajectoryState
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint
from unique_identifier_msgs.msg import UUID

from robot_motion.action_server import MotionActionAdapter


SESSION = 1_800_000_000_000


class Harness:
    def __init__(self, model):
        self.context = Context()
        rclpy.init(args=[], context=self.context, domain_id=196)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.node = Node('motion_supervisor_fixture', context=self.context)
        self.io = Node('fake_mcu_motion_fixture', context=self.context)
        self.executor.add_node(self.node)
        self.executor.add_node(self.io)
        self.owner = dict(lease_id='private-test-lease', epoch=42, session_id=SESSION, sequence=3)
        self.authorized = True
        self.session = SESSION
        self.sent = []
        self.stops = []
        self.command = None
        self.started = None
        self.sequence = 0
        self.health_ticks = 0
        self.publish_ack = True
        self.publish_motion = True
        self.ack_stop = True
        self.stopped = False
        self.force_position = None
        self.force_completed = False
        self.status_replay = False
        self.sample_ages = {}
        self._previous_status = None
        self.adapter = MotionActionAdapter(
            self.node, model, lambda _trajectory: None, self.submit,
            lambda owner: self.authorized and owner == self.owner,
            self.stop, lambda _owner: self.stopped,
            feedback_timeout_s=.18, ack_timeout_s=.15, stop_timeout_s=.18,
            goal_tolerance=.005, stopped_velocity_tolerance=.01)
        self.motion_pub = self.io.create_publisher(
            MotorMotionStateArray, '/motor/motion_states',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.status_pub = self.io.create_publisher(MotorTrajectoryState, '/motor/trajectory_state', 10)
        self.client = ActionClient(self.io, FollowJointTrajectory, '/robot/follow_joint_trajectory')
        self.feedback = []
        self.timer = self.io.create_timer(.01, self.health)
        self.node.create_subscription(MotorTrajectoryState, '/motor/trajectory_state', self.stop_ack, 10)
        self.service_ticks = 0
        def ping(_request, response):
            self.service_ticks += 1
            response.success = True
            return response
        self.node.create_service(Trigger, '/motion_fixture_ping', ping)
        self.ping_client = self.io.create_client(Trigger, '/motion_fixture_ping')
        assert self.client.wait_for_server(timeout_sec=2.)
        self.wait(lambda: self.health_ticks >= 2)

    def submit(self, trajectory, owner):
        assert owner == self.owner
        self.sent.append(copy.deepcopy(trajectory))
        self.command, self.started = copy.deepcopy(trajectory), time.monotonic()
        self.stopped = False

    def stop(self, _owner, reason):
        self.stops.append((time.monotonic(), reason))

    def stop_ack(self, message):
        if (self.stops and self.ack_stop and message.session_id == self.session
                and not message.active and time.monotonic() - self.stops[-1][0] > .02):
            self.stopped = True

    def health(self):
        self.health_ticks += 1
        self.sequence += 1
        now = time.monotonic()
        tick = int(now * 1000) & 0xFFFFFFFF
        active = self.command is not None and not (self.stops and self.ack_stop)
        elapsed = max(0, tick - (int(self.started * 1000) & 0xFFFFFFFF)) if active else 0
        total = self.command.points[-1].time_from_start_ms if active else 0
        report = MotorTrajectoryState(
            sync_time_ns=self.io.get_clock().now().nanoseconds,
            session_id=self.session, sequence=self.sequence, tick_ms=tick,
            trajectory_id=self.command.trajectory_id if self.command is not None else 0,
            active=active, completed=(elapsed >= total or self.force_completed) if active else False,
            elapsed_ms=elapsed, total_ms=total,
            target_count=2 if active else 0, target_ids=[8, 9] if active else [])
        if self.publish_ack:
            if self.status_replay and self._previous_status is not None:
                self.status_pub.publish(self._previous_status)
            else:
                self.status_pub.publish(report)
                self._previous_status = copy.deepcopy(report)
        if not self.publish_motion:
            return
        states = []
        for motor_id in (8, 9):
            position, velocity = 0., 0.
            sample_age = self.sample_ages.get(motor_id, 0)
            if active:
                endpoints = [p.position for p in self.command.points if p.motor_id == motor_id]
                measured_elapsed = max(0, elapsed - sample_age)
                u = min(1., measured_elapsed / total)
                position = endpoints[0] + (endpoints[-1] - endpoints[0]) * (3 * u*u - 2 * u*u*u)
                velocity = ((endpoints[-1] - endpoints[0]) * (6 * u - 6 * u*u) / (total / 1000.)
                            if measured_elapsed < total else 0.)
            if self.force_position is not None:
                position = self.force_position
            states.append(MotorMotionStateTarget(
                motor_id=motor_id, position=position, velocity=velocity, effort=0., state_age_ms=sample_age,
                target_source=MotorMotionStateTarget.TARGET_SOURCE_TRAJECTORY if active else 0))
        self.motion_pub.publish(MotorMotionStateArray(
            sync_time_ns=self.io.get_clock().now().nanoseconds,
            session_id=self.session, tick_ms=tick, seq=self.sequence, states=states))

    def goal(self, duration_ms=200):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['yaw', 'pitch']
        goal.trajectory.points = [
            JointTrajectoryPoint(positions=[0., 0.]),
            JointTrajectoryPoint(positions=[.04, .02], time_from_start=Duration(
                sec=duration_ms // 1000, nanosec=(duration_ms % 1000) * 1_000_000))]
        goal.goal_time_tolerance = Duration(nanosec=150_000_000)
        return goal

    def send(self, goal=None, *, authorize=True, wrong_uuid=False):
        goal_id = uuid.uuid4().bytes
        if authorize:
            self.adapter.authorize_goal(goal_id, self.owner)
        future = self.client.send_goal_async(
            self.goal() if goal is None else goal,
            goal_uuid=UUID(uuid=list(uuid.uuid4().bytes if wrong_uuid else goal_id)),
            feedback_callback=lambda feedback: self.feedback.append(feedback.feedback))
        self.wait(future.done)
        return future.result()

    def result(self, handle, timeout=2.):
        future = handle.get_result_async()
        self.wait(future.done, timeout)
        return future.result()

    def wait(self, condition, timeout=2.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=.005)
            if condition():
                return
        raise AssertionError(f'timeout; ticks={self.health_ticks}, stops={self.stops}, sent={len(self.sent)}')

    def close(self):
        self.adapter.shutdown()
        self.client.destroy()
        self.executor.remove_node(self.node)
        self.executor.remove_node(self.io)
        self.executor.shutdown()
        self.io.destroy_node()
        self.node.destroy_node()
        self.context.shutdown()


@pytest.fixture
def harness(model):
    value = Harness(model)
    try:
        yield value
    finally:
        value.close()


def test_standard_action_requires_one_use_uuid_authorization(harness):
    assert not harness.send(authorize=False).accepted
    assert not harness.sent
    handle = harness.send(wrong_uuid=True)
    assert handle.accepted  # rclpy exposes UUID only after goal callback.
    result = harness.result(handle)
    assert result.status == GoalStatus.STATUS_ABORTED
    assert result.result.error_code == FollowJointTrajectory.Result.INVALID_GOAL
    assert not harness.sent


def test_action_feedback_and_success_require_real_mcu_confirmation(harness):
    handle = harness.send()
    assert handle.accepted
    result = harness.result(handle)
    assert result.status == GoalStatus.STATUS_SUCCEEDED
    assert result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
    assert harness.feedback
    assert harness.feedback[-1].joint_names == ['yaw', 'pitch']
    assert len(harness.sent) == 1
    assert not harness.stops  # Completion keeps the explicitly authorized final hold.
    assert not harness.send(authorize=False).accepted  # Grant was consumed.


def test_differential_axes_track_against_their_own_measurement_times(harness):
    harness.sample_ages = {8: 50, 9: 0}
    goal = harness.goal()
    goal.path_tolerance = [JointTolerance(name=name, position=.0001)
                           for name in goal.trajectory.joint_names]
    handle = harness.send(goal)
    result = harness.result(handle)
    assert result.status == GoalStatus.STATUS_SUCCEEDED
    assert not harness.stops
    assert harness.feedback
    for feedback in harness.feedback:
        assert max(abs(value) for value in feedback.error.positions) < .0001


def test_pending_action_does_not_starve_timers_services_or_cancellation(harness):
    handle = harness.send(harness.goal(1000))
    harness.wait(lambda: len(harness.feedback) >= 2)
    initial_ticks = harness.health_ticks
    ping = harness.ping_client.call_async(Trigger.Request())
    harness.wait(ping.done)
    assert ping.result().success and harness.service_ticks == 1
    harness.ack_stop = False
    cancel = handle.cancel_goal_async()
    harness.wait(cancel.done)
    assert cancel.result().goals_canceling
    result = handle.get_result_async()
    harness.wait(lambda: harness.health_ticks > initial_ticks + 4)
    assert not result.done()  # Cancel request is not a stop acknowledgement.
    harness.ack_stop = True
    harness.wait(result.done)
    assert result.result().status == GoalStatus.STATUS_CANCELED
    assert harness.stopped


def test_cancel_without_stop_acknowledgement_aborts_instead_of_claiming_canceled(harness):
    handle = harness.send(harness.goal(1000))
    harness.wait(lambda: bool(harness.feedback))
    harness.ack_stop = False
    cancel = handle.cancel_goal_async()
    harness.wait(cancel.done)
    result = harness.result(handle)
    assert result.status == GoalStatus.STATUS_ABORTED
    assert 'stop acknowledgement timed out' in result.result.error_string


@pytest.mark.parametrize('failure', ['authority', 'motion_missing', 'ack_missing', 'old_session', 'replayed_ack', 'path_error'])
def test_active_goal_fails_closed_on_authority_or_measurement_failure(harness, failure):
    handle = harness.send(harness.goal(1000))
    harness.wait(lambda: bool(harness.feedback))
    if failure == 'authority':
        harness.authorized = False
    elif failure == 'motion_missing':
        harness.publish_motion = False
    elif failure == 'ack_missing':
        harness.publish_ack = False
    elif failure == 'old_session':
        harness.session = SESSION - 1
    elif failure == 'replayed_ack':
        harness.status_replay = True
    else:
        harness.force_position = 2.
    result = harness.result(handle)
    assert result.status == GoalStatus.STATUS_ABORTED
    assert harness.stops
    assert len(harness.sent) == 1


def test_no_mcu_ack_never_succeeds_merely_because_goal_time_passed(harness):
    harness.publish_ack = False
    handle = harness.send()
    result = harness.result(handle)
    assert result.status == GoalStatus.STATUS_ABORTED
    assert harness.stops


def test_completion_ack_without_measured_arrival_is_not_success(harness):
    harness.force_position = 0.
    handle = harness.send()
    result = harness.result(handle)
    assert result.status == GoalStatus.STATUS_ABORTED
    assert result.result.error_code == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED


def test_first_measured_arrival_after_goal_deadline_is_rejected(harness):
    harness.publish_ack = harness.publish_motion = False
    handle = harness.send()
    command = harness.command
    sequence = harness.sequence + 1
    # Final position is correct, but the MCU reports it 200 ms after the end;
    # this goal only permits 150 ms. Send both reports via the real DDS path.
    harness.status_pub.publish(MotorTrajectoryState(
        session_id=SESSION, sequence=sequence, tick_ms=1400,
        trajectory_id=command.trajectory_id, active=True, completed=True,
        elapsed_ms=400, total_ms=200, target_count=2, target_ids=[8, 9]))
    states = [MotorMotionStateTarget(
        motor_id=mid, position=[p.position for p in command.points if p.motor_id == mid][-1],
        velocity=0., state_age_ms=0,
        target_source=MotorMotionStateTarget.TARGET_SOURCE_TRAJECTORY)
        for mid in (8, 9)]
    harness.motion_pub.publish(MotorMotionStateArray(
        session_id=SESSION, seq=sequence, tick_ms=1400, states=states))
    harness.wait(lambda: bool(harness.stops))
    harness.stopped = True
    result = harness.result(handle)
    assert result.status == GoalStatus.STATUS_ABORTED
    assert result.result.error_code == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED


def test_early_completion_ack_is_rejected(harness):
    harness.force_completed = True
    handle = harness.send(harness.goal(1000))
    result = harness.result(handle)
    assert result.status == GoalStatus.STATUS_ABORTED
    assert 'early MCU completion' in result.result.error_string


def test_second_goal_cannot_replace_an_active_goal(harness):
    handle = harness.send(harness.goal(1000))
    with pytest.raises(ValueError, match='executing'):
        harness.adapter.authorize_goal(uuid.uuid4().bytes, harness.owner)
    assert not harness.send(authorize=False).accepted
    assert len(harness.sent) == 1
    cancel = handle.cancel_goal_async()
    harness.wait(cancel.done)
    assert harness.result(handle).status == GoalStatus.STATUS_CANCELED
