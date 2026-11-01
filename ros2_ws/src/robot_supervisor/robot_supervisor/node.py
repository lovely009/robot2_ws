"""Managed host ingress. MCU state remains the authority for drive state.

All callbacks run on one executor thread. Lease expiry uses a monotonic clock;
ROS stamps additionally reject messages delayed before this process received them.
The only maintenance command emitted here is the read-only parameter REPORT.
"""
from __future__ import annotations

import copy
import math
import secrets
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor
from rosidl_runtime_py.convert import message_to_ordereddict
from diagnostic_msgs.msg import DiagnosticArray
from std_srvs.srv import Trigger

from can_bridge_msgs.msg import (
    AgentWireContract,
    ConfigRequest, ControlLease, DeviceParamReport, MotorCommandArray, MotorControlStateArray,
    MotorMotionStateArray, MotorPolicyArray, MotorPolicyTarget, MotorTopology,
    MotorTrajectory, SystemCommand, WireContract,
)
from robot_system_msgs.msg import (
    CalibrationState, LeasedCommand, LeasedPolicy, LeasedTrajectory, RobotState,
)
from robot_system_msgs.srv import AcquireControl, AuthorizeMotion, ExecuteMaintenance, ReleaseControl, RenewControl, SetArmed
from robot_motion.host_action import HostMotionActionAdapter
from std_msgs.msg import Empty, String
import json
from motor_hardware_interface.control_state import ControlStateTracker
from robot_calibration.validation import compute_fingerprint

from .core import SupervisorCore
from .diagnostics import supervisor_diagnostics
from .validation import CommandValidator
from .mcu_lease import McuLease


def _reliable(depth=1, *, latched=False):
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE,
                      durability=(DurabilityPolicy.TRANSIENT_LOCAL if latched
                                  else DurabilityPolicy.VOLATILE))


class SupervisorNode(Node):
    def __init__(self, **kwargs):
        super().__init__('robot_supervisor', **kwargs)
        immutable = ParameterDescriptor(read_only=True)

        def parameter(name, default):
            return self.declare_parameter(name, default, immutable).value

        self._robot_id = parameter('robot_id', '')
        self._model_name = parameter('model_name', '')
        self._driver_file = parameter('driver_file', '')
        self._urdf_file = parameter('urdf_file', '')
        if not all((self._robot_id, self._model_name, self._driver_file, self._urdf_file)):
            raise ValueError('robot_id, model_name, driver_file and urdf_file are required')
        self._allow_trajectories = parameter('allow_trajectories', False)
        self._exclusive_graph = parameter('require_exclusive_bridge', True)
        self._command_timeout = float(parameter('command_timeout_s', .2))
        self._input_max_age = float(parameter('input_max_age_s', .25))
        self._start_tolerance = float(parameter('start_tolerance_rad', .02))
        self._control_timeout = float(parameter('control_timeout_s', .5))
        self._calibration_timeout = float(parameter('calibration_timeout_s', 1.5))
        stream_hz = float(parameter('stream_hz', 50.))
        max_trajectory_ms = parameter('max_trajectory_ms', 2000)
        for label, value in (
                ('command_timeout_s', self._command_timeout),
                ('input_max_age_s', self._input_max_age),
                ('control_timeout_s', self._control_timeout),
                ('calibration_timeout_s', self._calibration_timeout),
                ('stream_hz', stream_hz)):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{label} must be finite and positive')
        if (not math.isfinite(self._start_tolerance) or self._start_tolerance < 0
                or not 50 <= stream_hz <= 500 or self._command_timeout > 1.):
            raise ValueError('invalid start tolerance, stream rate or command timeout')
        # The checker is a fixed snapshot. A file change requires a new process,
        # including after an operator resets a fault; old limits cannot survive.
        self._config_hash = compute_fingerprint(self._driver_file, self._urdf_file)
        self._validator = CommandValidator(
            self._driver_file, self._urdf_file, max_trajectory_ms=max_trajectory_ms)
        if compute_fingerprint(self._driver_file, self._urdf_file) != self._config_hash:
            raise ValueError('configuration changed while loading; restart required')
        self._restart_required = False
        self.core = SupervisorCore(
            robot_id=self._robot_id, model_name=self._model_name,
            control_timeout_s=self._control_timeout,
            calibration_timeout_s=self._calibration_timeout)
        self._authority = ControlStateTracker()
        self._command = None
        self._manual_until = -math.inf
        self._facade_context = None
        self._command_at = 0.
        self._motion = {}
        self._motion_sequence = None
        self._motion_at = 0.
        self._wire_sequence = 0
        self._request_id = secrets.randbelow(0x3FFFFFFE) + 1
        self._last_report_request = -math.inf
        self._last_state_publish = -math.inf
        self._last_diagnostics_publish = -math.inf
        self._last_input_result = None
        self._last_rejection = None
        self._last_graph_check = -math.inf
        self._graph_blockers = []
        self._calibration_hash_matches = False
        self._stop_retry_until = 0.
        self._next_stop_retry = 0.
        self._mcu_lease = McuLease()
        self._last_lease_publish = -math.inf

        self._pub_command = self.create_publisher(
            MotorCommandArray, '/robot/managed/motor/command_array', _reliable())
        self._pub_manual = self.create_publisher(MotorCommandArray, '/robot/managed/control/setpoint', _reliable())
        self._pub_host_trajectory = self.create_publisher(MotorTrajectory, '/robot/managed/control/trajectory', _reliable(10))
        self._pub_host_stop = self.create_publisher(Empty, '/robot/managed/control/stop', _reliable())
        self.create_subscription(MotorCommandArray, '/motor/host_command_array', self._on_host_command, _reliable())
        self.create_subscription(String, '/control/status', self._on_host_status, _reliable())
        self._pub_policy = self.create_publisher(
            MotorPolicyArray, '/robot/managed/motor/policy_array', _reliable(10))
        self._pub_trajectory = self.create_publisher(
            MotorTrajectory, '/robot/managed/motor/trajectory', _reliable(10))
        self._pub_lease = self.create_publisher(
            ControlLease, '/robot/managed/motor/control_lease', _reliable())
        self._pub_report = self.create_publisher(
            SystemCommand, '/robot/managed/maintenance/system_command', _reliable(10))
        self._pub_maintenance = self.create_publisher(
            ConfigRequest, '/robot/managed/maintenance/config_request', _reliable(10))
        self._pub_state = self.create_publisher(RobotState, '/robot/state', _reliable(latched=True))
        self._pub_diagnostics = self.create_publisher(
            DiagnosticArray, '/diagnostics', _reliable(10))
        self.create_subscription(CalibrationState, '/robot/calibration_state',
                                 self._on_calibration, _reliable(latched=True))
        self.create_subscription(MotorControlStateArray, '/motor/control_states',
                                 self._on_control, _reliable(10))
        self.create_subscription(MotorTopology, '/motor/topology',
                                 self._on_topology, _reliable(10))
        self.create_subscription(DeviceParamReport, '/motor/device_param_report',
                                 self._on_device_params, _reliable(10))
        self.create_subscription(
            MotorMotionStateArray, '/motor/motion_states', self._on_motion,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        for message_type, channel in ((LeasedCommand, 'command'),
                                      (LeasedPolicy, 'policy'),
                                      (LeasedTrajectory, 'trajectory')):
            self.create_subscription(
                message_type, '/robot/control/' + channel,
                lambda message, kind=channel: self._on_input(kind, message),
                _reliable(1 if channel == 'command' else 10))
        for service_type, name, method in (
                (AcquireControl, 'acquire_control', self._acquire),
                (RenewControl, 'renew_control', self._renew),
                (ReleaseControl, 'release_control', self._release),
                (SetArmed, 'set_armed', self._set_armed),
                (AuthorizeMotion, 'authorize_motion', self._authorize_motion),
                (ExecuteMaintenance, 'execute_maintenance', self._maintenance),
                (Trigger, 'stop', self._stop),
                (Trigger, 'reset_fault', self._reset)):
            self.create_service(service_type, '/robot/' + name, method)
        self._motion_adapter = HostMotionActionAdapter(self, self._driver_file, self._urdf_file,
            self._motion_authorized, self._motion_stop)
        self.create_timer(1. / stream_hz, self._tick)

    def _on_host_status(self, message):
        try:
            status = json.loads(message.data)
            if status.get('mode') == 'error' and self.core.armed:
                self.core.stop('host_controller_error')
                self._flush_stop()
        except (ValueError, TypeError): pass

    def _on_host_command(self, message):
        # Only newly generated ros2_control output, with its ORIGINAL stamp,
        # reaches the bridge. Neither lease renewal nor this node retimes it.
        now_ns = self.get_clock().now().nanoseconds
        if (not message.sync_time_ns or not 0 <= now_ns - message.sync_time_ns <= 30_000_000
                or self.count_publishers('/motor/host_command_array') != 1): return
        if message.count:
            context = getattr(self._motion_adapter, 'context', None) or self._facade_context
            if not self.core.armed or not self._mcu_lease.confirmed: return
            moving = time.monotonic() < self._manual_until or (context and self._motion_authorized(context))
            if moving:
                try:
                    self._validator.command(message)
                    self._active_targets([t.motor_id for t in message.targets])
                except ValueError: return
        else: moving = False
        outgoing = copy.deepcopy(message)
        if not moving:
            outgoing.count = 0
            outgoing.targets = []
        outgoing.session_id = self.core.session_id
        outgoing.lease_id = self._mcu_lease.lease_id
        self._pub_command.publish(outgoing)

    def _fresh_stamp(self, stamp, maximum_age):
        if stamp.sec < 0 or not 0 <= stamp.nanosec < 1000000000:
            return False
        sent_ns = stamp.sec * 1000000000 + stamp.nanosec
        age = (self.get_clock().now().nanoseconds - sent_ns) / 1e9
        return sent_ns > 0 and 0 <= age <= maximum_age

    def _maintenance(self, envelope, response):
        request = envelope.request
        report = request.kind == ConfigRequest.CONFIG_KIND_SYSTEM_COMMAND and request.command == SystemCommand.COMMAND_REPORT
        try:
            age = (self.get_clock().now().nanoseconds - request.sync_time_ns) / 1e9
            if not request.sync_time_ns or not 0 <= age <= .25:
                raise ValueError('maintenance source timestamp expired')
            if request.kind not in (0, 1, 2) or not 0 < request.request_id < 0x40000000:
                raise ValueError('invalid maintenance kind or request ID')
            if not report:
                if not self.core.lease_matches(envelope.lease_id, envelope.epoch, envelope.session_id):
                    raise ValueError('current control ownership required')
                now = time.monotonic()
                tracker = self._authority
                disabled = (tracker.valid and 0 <= now-tracker.at <= self._control_timeout
                    and sorted(tracker.states) == self._validator.motor_ids
                    and all(s.online and not s.policy_enabled and not s.motion_enabled
                        and s.control_lifecycle == 2 and not s.unified_fault_code
                        and not s.vendor_fault_code and s.state_age_ms/1000+now-tracker.at <= self._control_timeout
                        for s in tracker.states.values()))
                if self.core.armed or self._motion_adapter.busy or not disabled:
                    raise ValueError('maintenance requires unarmed, fresh disabled MCU feedback')
            outgoing = copy.deepcopy(request)
            outgoing.session_id = self.core.session_id
            self._pub_maintenance.publish(outgoing)
            response.success, response.reason = True, 'accepted; wait for device config_result'
        except (ValueError, TypeError) as error:
            response.success, response.reason = False, str(error)
        return response

    def _on_calibration(self, message):
        valid_stamp = self._fresh_stamp(message.header.stamp, self._calibration_timeout)
        report = dict(message_to_ordereddict(message)) if valid_stamp else None
        self._calibration_hash_matches = bool(
            report and report.get('config_hash') == self._config_hash
            and sorted(report.get('motor_ids', [])) == self._validator.motor_ids
            and sorted(report.get('joint_names', []))
            == sorted(self._validator.model.joint_names))
        if (report and report.get('config_hash')
                and report['config_hash'] != self._config_hash):
            self._restart_required = True
        self.core.update_calibration(report)
        self._update_external_blockers()
        self._flush_stop()

    def _on_control(self, message):
        previous = self.core.session_id
        report = dict(message_to_ordereddict(message))
        accepted = self.core.update_control(report)
        self._after_session_update(previous)
        if accepted:
            try:
                self._mcu_lease.observe(message)
            except ValueError:
                self.core.update_control(None)
                self._flush_stop()
                return
            self._authority.update(message.schema_version, message.session_id,
                                   message.sequence, list(message.states), time.monotonic())
        self._flush_stop()

    def _on_topology(self, message):
        previous = self.core.session_id
        self.core.update_topology(dict(message_to_ordereddict(message)))
        self._after_session_update(previous)
        self._flush_stop()

    def _on_device_params(self, message):
        previous = self.core.session_id
        self.core.update_device_params(dict(message_to_ordereddict(message)))
        self._after_session_update(previous)
        self._flush_stop()

    def _after_session_update(self, previous):
        # Any authoritative channel may be the first report of a new session.
        # Reset before processing its first control frame, regardless of order.
        if self.core.session_id != previous:
            self._mcu_lease = McuLease()
            self._authority = ControlStateTracker()
            self._motion.clear()
            self._motion_sequence = None
            self._motion_at = 0.
            self._command = None
            self._last_report_request = -math.inf

    def _on_motion(self, message):
        if message.session_id != self.core.session_id:
            return
        sequence = int(message.seq)
        if self._motion_sequence is not None:
            advance = (sequence - self._motion_sequence) & 0xFFFFFFFF
            if not 0 < advance < 0x80000000:
                return
        samples = {}
        received = time.monotonic()
        for sample in message.states:
            if sample.motor_id in samples:
                return
            if (sample.state_age_ms / 1000. > self._control_timeout
                    or not all(math.isfinite(value) for value in
                               (sample.position, sample.velocity, sample.effort))):
                continue
            samples[int(sample.motor_id)] = (
                float(sample.position), received - sample.state_age_ms / 1000.)
        self._motion, self._motion_sequence = samples, sequence
        self._motion_at = received

    @staticmethod
    def _respond(response, result):
        for key, value in result.items():
            if hasattr(response, key):
                setattr(response, key, value)
        return response

    def _acquire(self, request, response):
        self._check_graph()
        result = self.core.acquire(request.owner, request.mode, request.lease_ms)
        if result['success']:
            self._command = None
            self._stop_retry_until = 0.
        self._flush_stop()
        return self._respond(response, result)

    def _renew(self, request, response):
        result = self.core.renew(request.lease_id, request.epoch, request.lease_ms)
        self._flush_stop()
        return self._respond(response, result)

    def _release(self, request, response):
        result = self.core.release(request.lease_id, request.epoch)
        self._flush_stop()
        return self._respond(response, result)

    def _set_armed(self, request, response):
        was_armed = self.core.armed
        result = self.core.set_armed(request.lease_id, request.epoch, request.armed)
        if result['success'] and request.armed:
            self._stop_retry_until = 0.
            if not was_armed:
                try:
                    self._mcu_lease.start(self.core.session_id)
                    self._publish_mcu_lease()
                except ValueError as error:
                    self.core.set_armed(request.lease_id, request.epoch, False)
                    result = dict(success=False, reason=str(error))
        self._flush_stop()
        return self._respond(response, result)

    def _require_mcu_lease(self):
        if not self._mcu_lease.confirmed:
            raise ValueError('waiting for fresh MCU authorization acknowledgement')

    def _publish_mcu_lease(self, *, revoke=False):
        remaining = self.core.snapshot()['lease_remaining_ms']
        packet = self._mcu_lease.packet(remaining, revoke=revoke)
        if packet is not None:
            self._pub_lease.publish(ControlLease(**packet))
            self._last_lease_publish = time.monotonic()

    def _authorize_motion(self, request, response):
        try:
            if not self._allow_trajectories:
                raise ValueError('MCU trajectories are disabled in this launch')
            if not self._fresh_stamp(request.header.stamp, self._input_max_age):
                raise ValueError('motion authorization timestamp is not fresh')
            if self._motion_adapter.busy:
                raise ValueError('another motion goal is running')
            self._require_mcu_lease()
            result = self.core.authorize(request.lease_id, request.epoch,
                                         request.session_id, request.sequence, 'trajectory')
            if not result['success']:
                raise ValueError(result['reason'])
            context = dict(lease_id=request.lease_id, epoch=request.epoch,
                           session_id=request.session_id, sequence=request.sequence,
                           mcu_lease_id=self._mcu_lease.lease_id)
            self._motion_adapter.authorize_goal(bytes(request.goal_id), context,
                                                 expires_in_s=.5)
            response.success, response.reason = True, ''
        except (ValueError, TypeError, OverflowError) as error:
            response.success, response.reason = False, str(error)
        self._flush_stop()
        return response

    def _motion_authorized(self, context):
        return (self.core.lease_matches(context['lease_id'], context['epoch'], context['session_id'])
                and context.get('mcu_lease_id') == self._mcu_lease.lease_id
                and self.core.armed and self._mcu_lease.confirmed)

    def _submit_motion(self, message, context):
        if not self._allow_trajectories or not self._motion_authorized(context):
            raise ValueError('motion authorization expired')
        self._validator.trajectory(message)
        self._active_targets(list(message.target_ids))
        self._trajectory_start(message)
        message.session_id = self.core.session_id
        message.lease_id = self._mcu_lease.lease_id
        message.sync_time_ns = self.get_clock().now().nanoseconds
        self._command = None
        self._pub_trajectory.publish(message)

    def _motion_stop(self, context, reason):
        # Never stop a newer owner's motion on behalf of a stale Action.
        if self.core.lease_matches(context['lease_id'], context['epoch'], context['session_id']):
            self.core.stop('trajectory:' + reason)
        self._flush_stop()

    def _motion_stopped(self, context):
        state = self.core.snapshot()
        return (context['session_id'] == state['session_id'] and not state['armed']
                and 'awaiting_disabled' not in state['blockers']
                and self._mcu_lease.report is not None
                and time.monotonic() - self._mcu_lease.report_at <= .25
                and not self._mcu_lease.report.lease_active)

    def _stop(self, _request, response):
        self.core.stop('operator_stop')
        self._flush_stop()
        response.success = True
        response.message = 'stop requested; physical stop requires MCU acknowledgement'
        return response

    def _reset(self, _request, response):
        result = self.core.reset_fault()
        response.success, response.message = result['success'], result['reason']
        if result['success']:
            self._stop_retry_until = 0.
        self._flush_stop()
        return response

    def _active_targets(self, motor_ids):
        now = time.monotonic()
        if any(not self._authority.policy_ready(mid, now, self._control_timeout) for mid in motor_ids):
            raise ValueError('MCU has not confirmed enabled policy and fresh feedback for every target')

    def _trajectory_start(self, trajectory):
        first = {}
        for point in trajectory.points:
            first.setdefault(point.motor_id, float(point.position))
        now = time.monotonic()
        for mid in trajectory.target_ids:
            sample = self._motion.get(mid)
            if mid not in first:
                raise ValueError(f'm{mid}: trajectory start sample missing')
            if not sample or not 0 <= now - sample[1] <= self._control_timeout:
                raise ValueError(f'm{mid}: fresh measured trajectory start is required')
            if abs(first[mid] - sample[0]) > self._start_tolerance:
                raise ValueError(f'm{mid}: trajectory start differs from measured position')

    def _on_input(self, channel, envelope):
        try:
            if not self._fresh_stamp(envelope.header.stamp, self._input_max_age):
                raise ValueError('input timestamp missing, expired or in the future')
            message = getattr(envelope, channel)
            disabling = (channel == 'policy' and message.targets
                         and not any(target.enabled for target in message.targets))
            canceling = channel == 'trajectory' and not message.count and not message.points
            if self._motion_adapter.busy and not disabling and not canceling:
                raise ValueError('standard Action owns motion; cancel it before another input')
            if channel != 'trajectory':
                getattr(self._validator, channel)(message)
            # Host JTC facade assembles and validates complete plans in WorkbenchModel;
            # the MCU's 256-point upload bound is not a host plan length bound.
            if (channel == 'policy' and any(target.enabled for target in message.targets)
                    and any(not target.enabled for target in message.targets)):
                raise ValueError('managed policies cannot mix enable and disable intents')
            if channel == 'trajectory' and message.count and not self._allow_trajectories:
                raise ValueError('trajectories require the managed launch to own the Agent')
            result = self.core.authorize(
                envelope.lease_id, envelope.epoch, envelope.session_id,
                envelope.sequence, channel)
            if not result['success']:
                raise ValueError(result['reason'])
            if not disabling:
                self._require_mcu_lease()
            outgoing = copy.deepcopy(message)
            outgoing.sync_time_ns = message.sync_time_ns or (envelope.header.stamp.sec * 10**9 + envelope.header.stamp.nanosec)
            if hasattr(outgoing, 'session_id'):
                outgoing.session_id = self.core.session_id
            outgoing.lease_id = self._mcu_lease.lease_id
            if channel == 'policy':
                if any(target.enabled for target in outgoing.targets):
                    outgoing.policy_generation = self.core.next_policy_generation()
                    outgoing.request_id = self._next_request_id()
                    self._pub_policy.publish(outgoing)
                elif outgoing.targets:
                    # First version owns the whole robot. An explicit disable
                    # is an authorization handoff, never a pause of cached
                    # motion. Require a new off acknowledgement and arm before
                    # any later enable; an old command/trajectory cannot revive.
                    self.core.set_armed(envelope.lease_id, envelope.epoch, False)
                    self._flush_stop()
                else:
                    raise ValueError('managed policy requires at least one target')
            elif channel == 'command':
                self._active_targets([target.motor_id for target in outgoing.targets])
                age = (self.get_clock().now().nanoseconds - outgoing.sync_time_ns) / 1e9
                if not 0 <= age < self._command_timeout:
                    raise ValueError('original manual source expired')
                self._manual_until = time.monotonic() + self._command_timeout - age
                self._motion_adapter.context = None
                self._facade_context = None
                self._pub_manual.publish(outgoing)
            else:
                self._active_targets(list(outgoing.target_ids))
                if not canceling and outgoing.segment_count == 1:
                    self._trajectory_start(outgoing)
                self._manual_until = -math.inf
                if canceling:
                    self._motion_adapter.cancel_current()
                else:
                    self._facade_context = dict(lease_id=envelope.lease_id,
                        epoch=envelope.epoch, session_id=envelope.session_id, sequence=envelope.sequence,
                        mcu_lease_id=self._mcu_lease.lease_id)
                self._pub_host_trajectory.publish(outgoing)
            self._record_input_result(channel, envelope.sequence, True, '')
        except (ValueError, TypeError, OverflowError) as error:
            self._record_input_result(channel, envelope.sequence, False, str(error))
        self._flush_stop()

    def _next_request_id(self):
        self._request_id = (self._request_id + 1) & 0x3FFFFFFF
        if not self._request_id:
            self._request_id = 1
        return self._request_id

    def _record_input_result(self, channel, sequence, accepted, reason):
        # This is observational state, not an acknowledgement protocol. Keep
        # ingress diagnostics bounded even while commands arrive at 100 Hz.
        result = {'channel': channel, 'sequence': int(sequence),
                  'accepted': accepted, 'reason': reason,
                  'time_ns': self.get_clock().now().nanoseconds}
        self._last_input_result = result
        if not accepted:
            self._last_rejection = result

    def _publish_diagnostics(self, state):
        output = DiagnosticArray()
        output.header.stamp = self.get_clock().now().to_msg()
        output.status = supervisor_diagnostics(
            state, self._robot_id, self._model_name,
            node_name=self.get_fully_qualified_name().lstrip('/'),
            last_input=self._last_input_result, last_rejection=self._last_rejection)
        self._pub_diagnostics.publish(output)

    def _send_stop(self):
        self._facade_context = None
        self._manual_until = -math.inf
        self._pub_host_stop.publish(Empty())
        self._command = None
        try:
            self._publish_mcu_lease(revoke=True)
        except ValueError as error:
            # Do not let a sequence exhaustion prevent the independent cancel
            # and disable paths; the MCU's outstanding deadline also expires.
            self.get_logger().error(str(error))
        stamp = self.get_clock().now().nanoseconds
        cancel = MotorTrajectory()
        cancel.sync_time_ns, cancel.session_id = stamp, self.core.session_id
        cancel.segment_count = 1
        self._pub_trajectory.publish(cancel)
        self._pub_command.publish(MotorCommandArray(sync_time_ns=stamp))
        # History may include more than one 64-motor assembly after a session or
        # configuration change. Keep each bounded wire message within capacity.
        motor_ids = self.core.known_motor_ids
        for offset in range(0, len(motor_ids), WireContract.MOTOR_COUNT_MAX):
            disable = MotorPolicyArray()
            disable.sync_time_ns, disable.session_id = stamp, self.core.session_id
            disable.request_id = self._next_request_id()
            disable.policy_generation = self.core.policy_generation
            disable.targets = [MotorPolicyTarget(motor_id=mid, enabled=False, kp=0., kd=0.)
                               for mid in motor_ids[offset:offset + WireContract.MOTOR_COUNT_MAX]]
            disable.count = len(disable.targets)
            self._pub_policy.publish(disable)

    def _flush_stop(self):
        reason = self.core.take_stop_request()
        if reason:
            self._send_stop()
            self._stop_retry_until = time.monotonic() + 1.
            self._next_stop_retry = time.monotonic() + .1
            self.get_logger().warning('managed stop requested: ' + reason)

    def _update_external_blockers(self):
        blockers = list(self._graph_blockers)
        if not self._calibration_hash_matches:
            blockers.append('validator_configuration_mismatch')
        if self._restart_required:
            blockers.append('configuration_changed_restart_required')
        self.core.set_external_blockers(blockers)

    def _check_graph(self):
        blockers = []
        if self._exclusive_graph:
            topics = (
                '/robot/managed/motor/command_array', '/robot/managed/motor/policy_array',
                '/robot/managed/motor/trajectory', '/robot/calibration_state',
                '/robot/managed/motor/control_lease', AgentWireContract.TOPIC_COMMAND,
                '/motor/control_states', '/motor/topology',
                AgentWireContract.TOPIC_TRAJECTORY, AgentWireContract.TOPIC_CONFIG,
                '/motor/host_command_array',
                '/robot/managed/maintenance/config_request',
            )
            for topic in topics:
                if self.count_publishers(topic) != 1:
                    blockers.append('publisher_count:' + topic)
        self._graph_blockers = blockers
        self._update_external_blockers()

    def _tick(self):
        now = time.monotonic()
        if now - self._last_graph_check >= .25:
            self._check_graph()
            self._last_graph_check = now
        self.core.tick()
        self._flush_stop()
        if self.core.armed:
            if self._mcu_lease.failed:
                self.core.stop('mcu_authorization_lost')
                self._flush_stop()
            elif now - self._last_lease_publish >= .05:
                try:
                    self._publish_mcu_lease()
                except ValueError as error:
                    self.core.stop(str(error))
                    self._flush_stop()
        if self._manual_until != -math.inf and now >= self._manual_until:
            self.core.stop('manual_source_expired')
            self._flush_stop()
        if now < self._stop_retry_until and now >= self._next_stop_retry:
            self._send_stop()
            self._next_stop_retry = now + .1
        # REPORT reads only the MCU's existing parameter view; it does not write
        # parameters, clear faults, issue SDO or change motor authorization.
        state = self.core.snapshot()
        if (state['session_id'] and not state['armed']
                and now - self._last_report_request >= 1.):
            report = SystemCommand(
                sync_time_ns=self.get_clock().now().nanoseconds,
                request_id=0, command=SystemCommand.COMMAND_REPORT,
                motor_id=SystemCommand.MOTOR_ID_NONE)
            self._pub_report.publish(report)
            self._last_report_request = now
        if now - self._last_state_publish >= .1:
            output = RobotState()
            output.header.stamp = self.get_clock().now().to_msg()
            for key, value in state.items():
                if hasattr(output, key):
                    setattr(output, key, value)
            output.mcu_authorized = bool(state['armed'] and self._mcu_lease.confirmed)
            self._pub_state.publish(output)
            self._last_state_publish = now
        if now - self._last_diagnostics_publish >= 1.:
            self._publish_diagnostics(state)
            self._last_diagnostics_publish = now

    def destroy_node(self):
        if hasattr(self, '_motion_adapter'):
            self._motion_adapter.shutdown()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SupervisorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # A terminal may signal the whole process group before launch forwards
        # SIGINT again. Do not interrupt destruction with that second signal.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            if rclpy.ok():
                node.core.stop('supervisor_shutdown')
                node._flush_stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
