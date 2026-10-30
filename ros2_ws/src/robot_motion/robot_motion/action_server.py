"""Standard Action server hosted by the single-threaded robot supervisor.

Authorization and physical execution are injected; this module opens no device,
never enables a motor and never renews control ownership. A one-use authorization
binds the standard action's transport UUID to an existing explicit control lease.
"""
from __future__ import annotations

import copy
import math
import secrets
import time

from can_bridge_msgs.msg import MotorMotionStateArray, MotorTrajectoryState
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from robot_transmission import Axis

from .trajectory import TrajectoryCompiler, signed_tick_difference


class MotionActionAdapter:
    def __init__(self, node, model, validate_trajectory, submit_trajectory,
                 authorization_valid, request_stop, stop_confirmed, *,
                 action_name='/robot/follow_joint_trajectory', max_trajectory_ms=2000,
                 feedback_timeout_s=.5, ack_timeout_s=.5, stop_timeout_s=1.,
                 path_tolerance=.2, goal_tolerance=.02, stopped_velocity_tolerance=.01):
        for value in (feedback_timeout_s, ack_timeout_s, stop_timeout_s):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('motion timeouts must be finite and positive')
        self.node, self.model = node, model
        self._submit, self._valid = submit_trajectory, authorization_valid
        self._request_stop, self._stop_confirmed = request_stop, stop_confirmed
        self._feedback_timeout, self._ack_timeout = feedback_timeout_s, ack_timeout_s
        self._stop_timeout = stop_timeout_s
        self.compiler = TrajectoryCompiler(
            model, validate_trajectory, max_trajectory_ms=max_trajectory_ms,
            path_tolerance=path_tolerance, goal_tolerance=goal_tolerance,
            stopped_velocity_tolerance=stopped_velocity_tolerance)
        self._grant = None
        self._reservation = False
        self._active = None
        self._motion = None
        self._trajectory = None
        self._motion_order = None
        self._trajectory_order = None
        self._closed = False
        self._action_callbacks = ReentrantCallbackGroup()
        self._server = ActionServer(
            node, FollowJointTrajectory, action_name,
            execute_callback=self._execute, goal_callback=self._goal,
            handle_accepted_callback=self._accepted, cancel_callback=self._cancel,
            callback_group=self._action_callbacks)
        self._motion_sub = node.create_subscription(
            MotorMotionStateArray, '/motor/motion_states', self._on_motion,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self._trajectory_sub = node.create_subscription(
            MotorTrajectoryState, '/motor/trajectory_state', self._on_trajectory,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self._timer = node.create_timer(.02, self._tick)

    @property
    def busy(self):
        return self._active is not None or self._reservation

    def authorize_goal(self, goal_id, context, expires_in_s=1.):
        goal_id = bytes(goal_id)
        if len(goal_id) != 16 or not any(goal_id):
            raise ValueError('goal_id must be a nonzero 16-byte UUID')
        if self._closed or self.busy:
            raise ValueError('motion executor is unavailable or executing a goal')
        if not math.isfinite(expires_in_s) or not 0 < expires_in_s <= 1.:
            raise ValueError('one-use motion authorization must expire within one second')
        if not self._valid(context):
            raise ValueError('motion requires a current, armed trajectory lease')
        self._grant = (goal_id, copy.deepcopy(context), time.monotonic() + expires_in_s)

    def _grant_valid(self):
        return bool(self._grant and time.monotonic() < self._grant[2]
                    and self._valid(self._grant[1]))

    def _goal(self, request):
        if self._closed or self.busy or not self._grant_valid():
            return GoalResponse.REJECT
        try:
            self.compiler.compile(request, 1)
        except (ValueError, TypeError, OverflowError, RuntimeError) as error:
            self.node.get_logger().warning('trajectory goal rejected: ' + str(error))
            return GoalResponse.REJECT
        # UUID is only exposed by rclpy in handle_accepted_callback. Reserve the
        # single execution slot now, then verify identity before any publication.
        self._reservation = True
        return GoalResponse.ACCEPT

    def _accepted(self, handle):
        self._reservation = False
        if not self._grant_valid() or bytes(handle.goal_id.uuid) != self._grant[0]:
            # No hardware command and no ownership change for an unbound UUID.
            handle.execute()
            return
        _, context, _ = self._grant
        self._grant = None
        trajectory_id = secrets.randbelow(0xFFFFFFFF) + 1
        try:
            plan = self.compiler.compile(handle.request, trajectory_id)
            self._active = {
                'handle': handle, 'context': context, 'plan': plan,
                'future': Future(), 'started': time.monotonic(),
                'accepted': False, 'canceling': False, 'stopping': False,
                'failure': None, 'last_feedback': -math.inf,
                'start_tick': None, 'completion_tick': None,
            }
            plan.message.session_id = context['session_id']
            self._submit(plan.message, context)
        except (ValueError, TypeError, OverflowError, RuntimeError) as error:
            if self._active is not None:
                self._active['failure'] = (FollowJointTrajectory.Result.INVALID_GOAL, str(error))
                self._finish('abort', *self._active['failure'])
        handle.execute()

    async def _execute(self, handle):
        active = self._active
        if active is None or active['handle'] is not handle:
            handle.abort()
            return FollowJointTrajectory.Result(
                error_code=FollowJointTrajectory.Result.INVALID_GOAL,
                error_string='goal UUID has no current one-use authorization')
        outcome, result = await active['future']
        if outcome == 'success':
            handle.succeed()
        elif outcome == 'cancel':
            handle.canceled()
        else:
            handle.abort()
        if self._active is active:
            self._active = None
        return result

    def _cancel(self, handle):
        active = self._active
        if active is None or active['handle'] is not handle or active['future'].done():
            return CancelResponse.REJECT
        active['canceling'] = True
        self._begin_stop('action_cancel_requested')
        return CancelResponse.ACCEPT

    def _begin_stop(self, reason, *, code=None):
        active = self._active
        if active is None or active['stopping']:
            return
        active['stopping'], active['stop_at'] = True, time.monotonic()
        active['failure'] = (code if code is not None else FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,
                             reason)
        self._request_stop(active['context'], reason)

    def _finish(self, outcome, code, reason=''):
        active = self._active
        if active is None or active['future'].done():
            return
        active['future'].set_result((outcome, FollowJointTrajectory.Result(
            error_code=code, error_string=reason)))

    @staticmethod
    def _new_report(order, session, sequence):
        if not session:
            return False
        if order is None or session > order[0]:
            return True
        return session == order[0] and 0 < ((sequence - order[1]) & 0xFFFFFFFF) < 0x80000000

    def _on_motion(self, message):
        # The facade carries MCU session and tick metadata; old firmware cannot
        # provide the contract and is intentionally not accepted by this path.
        if not self._new_report(self._motion_order, message.session_id, message.seq):
            return
        self._motion_order = (message.session_id, message.seq)
        self._motion = (copy.deepcopy(message), time.monotonic())

    def _on_trajectory(self, message):
        if not self._new_report(self._trajectory_order, message.session_id, message.sequence):
            return
        self._trajectory_order = (message.session_id, message.sequence)
        self._trajectory = (copy.deepcopy(message), time.monotonic())

    def _actual(self, active, now):
        if self._motion is None:
            return None
        message, received = self._motion
        if (message.session_id != active['context']['session_id']
                or received < active['started'] or now - received > self._feedback_timeout):
            return None
        by_id = {sample.motor_id: sample for sample in message.states}
        if len(by_id) != len(message.states):
            return None
        plan = active['plan']
        elapsed_samples, axes, references = [], {}, {}
        for name, mid in self.model.actuator_ids.items():
            sample = by_id.get(mid)
            if sample is None:
                return None
            values = (sample.position, sample.velocity, sample.effort)
            age = sample.state_age_ms / 1000.
            if (not all(math.isfinite(value) for value in values)
                    or age + now - received > self._feedback_timeout
                    or sample.target_source != sample.TARGET_SOURCE_TRAJECTORY):
                return None
            sample_tick = (message.tick_ms - sample.state_age_ms) & 0xFFFFFFFF
            elapsed = signed_tick_difference(sample_tick, active['start_tick'])
            if elapsed < 0:
                return None
            axes[name] = Axis(*values)
            references[name] = plan.desired_actuator(mid, elapsed)
            elapsed_samples.append(elapsed)
        # Each actuator is compared with its reference at that actuator's own
        # measurement tick. Map both complete vectors so differential gearing
        # and joint offsets cancel correctly in the tracking error. Assigning
        # all measurements the oldest tick would invent errors on newer axes.
        oldest = (active['start_tick'] + min(elapsed_samples)) & 0xFFFFFFFF
        latest = (active['start_tick'] + max(elapsed_samples)) & 0xFFFFFFFF
        return (self.model.actuator_to_joint(axes),
                self.model.actuator_to_joint(references), oldest, latest)

    def _tick(self):
        active = self._active
        if active is None or active['future'].done():
            return
        now = time.monotonic()
        if active['stopping']:
            if self._stop_confirmed(active['context']):
                if active['canceling']:
                    self._finish('cancel', FollowJointTrajectory.Result.SUCCESSFUL,
                                 'canceled after fresh MCU disabled acknowledgement')
                else:
                    self._finish('abort', *active['failure'])
            elif now - active['stop_at'] > self._stop_timeout:
                self._finish('abort', FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,
                             'stop acknowledgement timed out; motion stop is unconfirmed')
            return
        if not self._valid(active['context']):
            self._begin_stop('motion authorization expired or changed')
            return
        plan = active['plan']
        report = None
        if self._trajectory is not None:
            candidate, received = self._trajectory
            if (candidate.session_id == active['context']['session_id']
                    and received >= active['started'] and now - received <= self._feedback_timeout):
                report = candidate
        if report is not None and report.trajectory_id == plan.message.trajectory_id:
            if (not report.active or report.total_ms != plan.total_ms
                    or report.target_count != len(plan.motor_ids)
                    or set(report.target_ids) != set(plan.motor_ids)):
                self._begin_stop('MCU trajectory acknowledgement does not match the submitted goal')
                return
            active['accepted'] = True
            start_tick = (report.tick_ms - report.elapsed_ms) & 0xFFFFFFFF
            if active['start_tick'] is None:
                active['start_tick'] = start_tick
            elif abs(signed_tick_difference(start_tick, active['start_tick'])) > 2:
                self._begin_stop('MCU trajectory timeline changed during execution')
                return
            if report.completed:
                if report.elapsed_ms < plan.total_ms:
                    self._begin_stop('invalid early MCU completion acknowledgement')
                    return
                if active['completion_tick'] is None:
                    active['completion_tick'] = report.tick_ms
        elif active['accepted']:
            self._begin_stop('MCU trajectory acknowledgement expired or was replaced')
            return
        elif now - active['started'] > self._ack_timeout:
            self._begin_stop('MCU did not acknowledge the trajectory within the acceptance budget')
            return
        if not active['accepted']:
            return
        actual = self._actual(active, now)
        if actual is None:
            if now - active['started'] > self._feedback_timeout:
                self._begin_stop('fresh measured joint feedback is unavailable')
            return
        measured, desired, tick, latest_tick = actual
        elapsed = signed_tick_difference(tick, active['start_tick'])
        latest_elapsed = signed_tick_difference(latest_tick, active['start_tick'])
        tolerances = plan.tolerances
        if elapsed < plan.total_ms:
            if any(abs(desired[j].position - measured[j].position) > tolerances.path_position[j]
                   or (tolerances.path_velocity[j] > 0 and
                       abs(desired[j].velocity - measured[j].velocity) > tolerances.path_velocity[j])
                   for j in plan.joint_names):
                self._begin_stop('measured trajectory path tolerance violated')
                return
        if now - active['last_feedback'] >= .05:
            feedback = FollowJointTrajectory.Feedback()
            feedback.header.stamp = self.node.get_clock().now().to_msg()
            feedback.joint_names = list(plan.joint_names)
            for field, samples in (('desired', desired), ('actual', measured)):
                value = getattr(feedback, field)
                value.positions = [samples[j].position for j in plan.joint_names]
                value.velocities = [samples[j].velocity for j in plan.joint_names]
                value.time_from_start.sec, ms = divmod(max(0, elapsed), 1000)
                value.time_from_start.nanosec = ms * 1_000_000
            feedback.error.positions = [desired[j].position - measured[j].position for j in plan.joint_names]
            feedback.error.velocities = [desired[j].velocity - measured[j].velocity for j in plan.joint_names]
            feedback.error.time_from_start = copy.deepcopy(feedback.actual.time_from_start)
            active['handle'].publish_feedback(feedback)
            active['last_feedback'] = now
        # A late first in-tolerance sample must not turn an expired goal into a
        # success, even if completion and measured arrival arrive together.
        if (latest_elapsed > plan.total_ms + tolerances.goal_time_s * 1000.
                or now - active['started'] > self._ack_timeout + plan.total_ms / 1000. + tolerances.goal_time_s):
            self._begin_stop('trajectory goal tolerance or completion deadline violated',
                             code=FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED)
            return
        completion = active['completion_tick']
        if completion is not None and signed_tick_difference(tick, completion) >= 0:
            if all(abs(desired[j].position - measured[j].position) <= tolerances.goal_position[j]
                   and abs(measured[j].velocity) <= tolerances.goal_velocity[j]
                   for j in plan.joint_names):
                # The MCU keeps its final hold while the explicit lease remains
                # valid. Success is not an implicit disable or lease renewal.
                self._finish('success', FollowJointTrajectory.Result.SUCCESSFUL)
                return

    def shutdown(self):
        if self._closed:
            return
        self._closed = True
        self._grant = None
        if self._active is not None and not self._active['future'].done():
            self._begin_stop('motion action server shutdown')
            self._finish('abort', FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,
                         'motion action server shutdown; stop confirmation unavailable')
        self.node.destroy_timer(self._timer)
        self.node.destroy_subscription(self._motion_sub)
        self.node.destroy_subscription(self._trajectory_sub)
        self._server.destroy()
