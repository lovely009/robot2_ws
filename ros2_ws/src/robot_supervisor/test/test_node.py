"""Managed ROS ingress regressions with a fake MCU, never hardware.

The reports here deliberately stand in for a commissioned test fixture. They do
not create or certify a calibration record for a real machine.
"""
from __future__ import annotations

import copy
import time
from pathlib import Path

import pytest
import rclpy
import yaml
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

from can_bridge_msgs.msg import (
    AgentCommand, AgentConfigRequest, AgentTrajectoryRequest, AgentWireContract,
    ConfigRequest, ControlLease, DeviceParamEntry, DeviceParamReport, LowCommand,
    MotorCommandArray, MotorCommandTarget,
    MotorControlState, MotorControlStateArray, MotorMotionStateArray,
    MotorMotionStateTarget, MotorPolicyArray, MotorPolicyTarget, MotorTopology,
    MotorTopologyEntry, MotorTrajectory, MotorTrajectoryPoint, WireContract,
)
from robot_calibration.validation import compute_fingerprint
from robot_system_msgs.msg import (
    CalibrationState, LeasedCommand, LeasedPolicy, LeasedTrajectory, RobotState,
)
from robot_system_msgs.srv import AcquireControl, ExecuteMaintenance, SetArmed
from robot_supervisor.node import SupervisorNode


RELIABLE = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
STREAM = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
SESSION = 1_800_000_000_000


def fixture_files(folder: Path):
    (folder / 'specs.yaml').write_text(yaml.safe_dump({'motor_types': {
        'sample': {'protocol': 'cerb', 'parameters': {
            'mit_position_limit_rad': 3., 'mit_velocity_limit_rad_s': 3.,
            'mit_torque_limit_nm': 10., 'peak_torque_nm': 4.,
            'mit_kp_limit': 5., 'mit_kd_limit': 1.,
        }}}}), encoding='utf-8')
    driver = folder / 'driver.yaml'
    driver.write_text(yaml.safe_dump({'motor_hardware': {
        'motor_specs_file': 'specs.yaml',
        'buses': {'arm': {'transport': 'serial_slcan', 'tty': '/unused/test-only'}},
        'motors': {'a1': {'id': 1, 'joint': 'j1', 'type': 'sample', 'bus': 'arm'}},
    }}), encoding='utf-8')
    urdf = folder / 'fixture.urdf'
    urdf.write_text('<robot name="fixture"><link name="base"/><link name="tip"/>'
        '<joint name="j1" type="revolute"><parent link="base"/>'
        '<child link="tip"/><axis xyz="0 0 1"/>'
        '<limit lower="-1" upper="1" velocity="1" effort="10"/>'
        '</joint></robot>', encoding='utf-8')
    return driver, urdf


class Harness:
    def __init__(self, folder, *, allow_trajectories=True, exclusive_graph=False,
                 domain_id=187):
        driver, urdf = fixture_files(folder)
        self.context = Context()
        rclpy.init(args=[], context=self.context, domain_id=domain_id)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.supervisor = SupervisorNode(context=self.context, parameter_overrides=[
            Parameter('driver_file', value=str(driver)),
            Parameter('urdf_file', value=str(urdf)),
            Parameter('robot_id', value='fixture-01'),
            Parameter('model_name', value='fixture'),
            Parameter('require_exclusive_bridge', value=exclusive_graph),
            Parameter('allow_trajectories', value=allow_trajectories),
            Parameter('control_timeout_s', value=.5),
            Parameter('calibration_timeout_s', value=1.5),
            Parameter('command_timeout_s', value=.2),
            Parameter('input_max_age_s', value=.25),
        ])
        self.io = Node('fake_commissioned_mcu', context=self.context)
        self.executor.add_node(self.supervisor)
        self.executor.add_node(self.io)
        self.hash = compute_fingerprint(str(driver), str(urdf))
        self.session = SESSION
        self.sequence = 0
        self.generation = 41
        self.active = False
        self.auto_ack = True
        self.ack_disable = True
        self.commissioned = True
        self.health_enabled = True
        self.motion_reports_enabled = True
        self.position = .1
        self.wire_lease = None
        self.ack_lease = True
        self.latest = None
        self.commands, self.policies, self.trajectories = [], [], []
        self.host_intent = None
        self.host_intent_at = 0.
        self.host_pub = self.io.create_publisher(MotorCommandArray, '/motor/host_command_array', RELIABLE)
        self.cal_pub = self.io.create_publisher(CalibrationState, '/robot/calibration_state', LATCHED)
        self.control_pub = self.io.create_publisher(MotorControlStateArray, '/motor/control_states', RELIABLE)
        self.topology_pub = self.io.create_publisher(MotorTopology, '/motor/topology', RELIABLE)
        self.params_pub = self.io.create_publisher(DeviceParamReport, '/motor/device_param_report', RELIABLE)
        self.motion_pub = self.io.create_publisher(MotorMotionStateArray, '/motor/motion_states', STREAM)
        self.command_pub = self.io.create_publisher(LeasedCommand, '/robot/control/command', RELIABLE)
        self.policy_pub = self.io.create_publisher(LeasedPolicy, '/robot/control/policy', RELIABLE)
        self.trajectory_pub = self.io.create_publisher(LeasedTrajectory, '/robot/control/trajectory', RELIABLE)
        self.raw_pub = self.io.create_publisher(MotorCommandArray, '/motor/command_array', RELIABLE)
        self.subscriptions = [
            self.io.create_subscription(MotorCommandArray, '/robot/managed/control/setpoint',
                                        self._manual, RELIABLE),
            self.io.create_subscription(MotorTrajectory, '/robot/managed/control/trajectory',
                                        lambda m: self.trajectories.append(copy.deepcopy(m)), RELIABLE),
            self.io.create_subscription(ControlLease, '/robot/managed/motor/control_lease',
                                        self._lease, RELIABLE),
            self.io.create_subscription(RobotState, '/robot/state', self._state, LATCHED),
            self.io.create_subscription(MotorCommandArray, '/robot/managed/motor/command_array',
                                        lambda m: self.commands.append(copy.deepcopy(m)), RELIABLE),
            self.io.create_subscription(MotorPolicyArray, '/robot/managed/motor/policy_array',
                                        self._policy, RELIABLE),
            self.io.create_subscription(MotorTrajectory, '/robot/managed/motor/trajectory',
                                        lambda m: self.trajectories.append(copy.deepcopy(m)), RELIABLE),
        ]
        self.timer = self.io.create_timer(.025, self._health)

    def _state(self, message):
        self.latest = message

    def _manual(self, message):
        # A controller-output fixture, separate from the operator request.
        # The supervisor must preserve this newly generated sample's stamp.
        self.host_intent, self.host_intent_at = copy.deepcopy(message), time.monotonic()

    def _lease(self, message):
        if self.ack_lease:
            self.wire_lease = copy.deepcopy(message)
            if not message.active:
                self.active = False

    def _policy(self, message):
        self.policies.append(copy.deepcopy(message))
        for target in message.targets:
            acknowledge = self.auto_ack if target.enabled else self.ack_disable
            if target.motor_id == 1 and acknowledge:
                self.active = target.enabled
                self.generation = message.policy_generation

    def _health(self):
        if not self.health_enabled:
            return
        self.sequence += 1
        host = (copy.deepcopy(self.host_intent) if self.host_intent is not None
                and time.monotonic()-self.host_intent_at < .2 else MotorCommandArray())
        host.sync_time_ns = self.io.get_clock().now().nanoseconds
        host.sequence = self.sequence
        self.host_pub.publish(host)
        tick_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF
        wire = self.wire_lease
        lease_id = wire.lease_id if wire and wire.session_id == self.session else 0
        lease_active = bool(lease_id and wire.active and
                            0 < (wire.valid_until_tick_ms - tick_ms) & 0xFFFFFFFF <= 300)
        if wire and not lease_active:
            self.active = False
        report = CalibrationState()
        report.header.stamp = self.io.get_clock().now().to_msg()
        report.robot_id, report.model_name = 'fixture-01', 'fixture'
        report.calibration_id, report.config_hash = 'fixture-acceptance-only', self.hash
        report.valid, report.commissioned = True, self.commissioned
        report.motor_ids, report.bus_indices, report.protocols = [1], [0], [1]
        report.model_names, report.joint_names = ['sample'], ['j1']
        self.cal_pub.publish(report)
        self.topology_pub.publish(MotorTopology(
            session_id=self.session, schema_version=3, entries=[MotorTopologyEntry(
                motor_id=1, bus_index=0, protocol=1, model_name='sample', online=True, conflict=False)]))
        self.params_pub.publish(DeviceParamReport(session_id=self.session, count=1,
            entries=[DeviceParamEntry(motor_id=1, dev_init_state=2,
                record_dirty=False, mismatch_mask=0, dev_safety=0.)]))
        self.control_pub.publish(MotorControlStateArray(
            schema_version=2, session_id=self.session, sequence=self.sequence, tick_ms=tick_ms,
            lease_id=lease_id, lease_sequence=wire.sequence if lease_id else 0,
            lease_valid_until_tick_ms=wire.valid_until_tick_ms if lease_id else 0,
            lease_active=lease_active,
            states=[MotorControlState(motor_id=1, online=True,
                control_lifecycle=3 if self.active else 2,
                execution_state=3 if self.active else 1,
                policy_enabled=self.active, motion_enabled=self.active,
                state_age_ms=0, policy_generation=self.generation)]))
        if self.motion_reports_enabled:
            self.motion_pub.publish(MotorMotionStateArray(seq=self.sequence,
                session_id=self.session, tick_ms=tick_ms,
                states=[MotorMotionStateTarget(motor_id=1, position=self.position,
                    velocity=0., effort=0., state_age_ms=0)]))

    def wait(self, condition, timeout=2.):
        until = time.monotonic() + timeout
        while time.monotonic() < until:
            self.executor.spin_once(timeout_sec=.01)
            if condition():
                return
        raise AssertionError(f'condition timed out; latest state={self.latest}')

    def spin(self, duration):
        until = time.monotonic() + duration
        while time.monotonic() < until:
            self.executor.spin_once(timeout_sec=min(.01, max(0., until-time.monotonic())))

    def call(self, srv_type, name, request):
        client = self.io.create_client(srv_type, name)
        try:
            self.wait(client.service_is_ready)
            future = client.call_async(request)
            self.wait(future.done)
            return future.result()
        finally:
            self.io.destroy_client(client)

    def acquire(self, *, mode=1, lease_ms=2000):
        return self.call(AcquireControl, '/robot/acquire_control',
                         AcquireControl.Request(owner='integration-test', mode=mode, lease_ms=lease_ms))

    def arm(self, lease):
        response = self.call(SetArmed, '/robot/set_armed', SetArmed.Request(
            lease_id=lease.lease_id, epoch=lease.epoch, armed=True))
        assert response.success, response.reason
        self.wait(lambda: self.supervisor._mcu_lease.confirmed)

    def envelope(self, cls, lease, sequence, payload, *, token=None, stamp=None):
        message = cls(lease_id=lease.lease_id if token is None else token,
                      epoch=lease.epoch, session_id=lease.session_id, sequence=sequence)
        message.header.stamp = stamp or self.io.get_clock().now().to_msg()
        setattr(message, {LeasedCommand: 'command', LeasedPolicy: 'policy',
                          LeasedTrajectory: 'trajectory'}[cls], payload)
        return message

    def enable(self, lease, sequence=1):
        self.arm(lease)
        self.policy_pub.publish(self.envelope(LeasedPolicy, lease, sequence,
            MotorPolicyArray(count=1, policy_generation=0xFFFFFFFF, targets=[
                MotorPolicyTarget(motor_id=1, enabled=True, kp=1., kd=.1)])))
        self.wait(lambda: any(p.targets and p.targets[0].enabled for p in self.policies))
        self.wait(lambda: self.latest is not None and self.latest.armed and self.latest.ready)
        self.spin(.06)  # deliver the separate authoritative control acknowledgement

    def command(self, lease, sequence=2, position=.2, **envelope_kwargs):
        payload = MotorCommandArray(count=1, targets=[MotorCommandTarget(
            motor_id=1, position=position, velocity=0., effort=0.)])
        self.command_pub.publish(self.envelope(LeasedCommand, lease, sequence,
                                               payload, **envelope_kwargs))

    def close(self):
        self.executor.remove_node(self.io)
        self.executor.remove_node(self.supervisor)
        self.io.destroy_node()
        self.supervisor.destroy_node()
        self.executor.shutdown()
        self.context.shutdown()


@pytest.fixture
def harness(tmp_path):
    instance = Harness(tmp_path)
    try:
        instance.wait(lambda: instance.latest is not None and instance.latest.ready)
        yield instance
    finally:
        instance.close()


def test_uncommissioned_report_blocks_control(harness):
    h = harness
    h.commissioned = False
    h.wait(lambda: h.latest is not None and not h.latest.ready)
    reply = h.acquire()
    assert not reply.success
    assert 'not_ready' in reply.reason
    assert not any(p.targets and p.targets[0].enabled for p in h.policies)


def test_maintenance_requires_owner_and_disabled_feedback_and_keeps_stamp(harness):
    h=harness
    writes=[]
    h.io.create_subscription(ConfigRequest,'/robot/managed/maintenance/config_request',writes.append,RELIABLE)
    lease=h.acquire()
    assert lease.success
    stamp=h.io.get_clock().now().nanoseconds
    request=ExecuteMaintenance.Request(lease_id='wrong',epoch=lease.epoch,session_id=lease.session_id,
        request=ConfigRequest(sync_time_ns=stamp,request_id=7,kind=1,command=0,motor_id=65535))
    refused=h.call(ExecuteMaintenance,'/robot/execute_maintenance',request)
    assert not refused.success and not writes
    request.lease_id=lease.lease_id
    accepted=h.call(ExecuteMaintenance,'/robot/execute_maintenance',request)
    assert accepted.success,accepted.reason
    h.wait(lambda:len(writes)==1)
    assert writes[0].sync_time_ns==stamp and writes[0].request_id==7
    h.arm(lease)
    request.request.sync_time_ns=h.io.get_clock().now().nanoseconds
    refused=h.call(ExecuteMaintenance,'/robot/execute_maintenance',request)
    assert not refused.success
    h.spin(.03)
    assert len(writes)==1


def test_explicit_arm_policy_generation_and_authoritative_ack(harness):
    h = harness
    lease = h.acquire()
    assert lease.success, lease.reason
    assert lease.session_id == SESSION  # the epoch-ms session must never truncate
    assert not h.latest.armed
    h.enable(lease)
    accepted_policy = next(p for p in h.policies if p.targets and p.targets[0].enabled)
    assert accepted_policy.policy_generation == 42
    h.command(lease)
    h.wait(lambda: any(m.count == 1 for m in h.commands))
    assert next(m for m in h.commands if m.count).targets[0].position == pytest.approx(.2)
    assert not hasattr(h.latest, 'lease_id')


def test_wrong_token_and_replayed_global_sequence_do_not_replace_command(harness):
    h = harness
    lease = h.acquire()
    assert lease.success
    h.enable(lease)
    h.command(lease, sequence=2, position=.2)
    h.wait(lambda: any(m.count for m in h.commands))
    h.command(lease, sequence=3, position=.4, token='retired-or-wrong-token')
    h.command(lease, sequence=2, position=.5)
    h.spin(.08)
    assert all(m.targets[0].position == pytest.approx(.2) for m in h.commands if m.count)


def test_host_arm_and_policy_send_do_not_substitute_for_mcu_acknowledgement(harness):
    h = harness
    h.auto_ack = False
    lease = h.acquire()
    assert lease.success
    h.arm(lease)
    h.policy_pub.publish(h.envelope(LeasedPolicy, lease, 1,
        MotorPolicyArray(count=1, targets=[
            MotorPolicyTarget(motor_id=1, enabled=True, kp=1., kd=.1)])))
    h.wait(lambda: any(p.targets and p.targets[0].enabled for p in h.policies))
    h.command(lease, sequence=2)
    h.spin(.06)
    assert not any(m.count for m in h.commands)
    # Only a later authoritative control sample opens the command path.
    h.active = True
    h.generation = next(p.policy_generation for p in h.policies
                        if p.targets and p.targets[0].enabled)
    h.spin(.06)
    h.command(lease, sequence=3)
    h.wait(lambda: any(m.count == 1 for m in h.commands))


@pytest.mark.parametrize('timestamp', ['missing', 'expired'])
def test_missing_or_expired_envelope_timestamp_cannot_start_command(harness, timestamp):
    h = harness
    lease = h.acquire()
    assert lease.success
    h.enable(lease)
    stamp = h.io.get_clock().now().to_msg()
    if timestamp == 'missing':
        stamp.sec, stamp.nanosec = 0, 0
    else:
        stamp.sec -= 1
    h.command(lease, sequence=2, stamp=stamp)
    h.spin(.06)
    assert not any(m.count for m in h.commands)


def test_disable_then_fast_enable_cannot_revive_cached_position_without_new_intent(harness):
    h = harness
    lease = h.acquire()
    assert lease.success
    h.enable(lease)
    h.command(lease, sequence=2, position=.2)
    h.wait(lambda: any(m.count for m in h.commands))
    initially_enabled = sum(any(t.enabled for t in p.targets) for p in h.policies)
    # Deliberately withhold the MCU's disabled acknowledgement. Confirm only
    # that the supervisor processed disable before immediately trying enable;
    # no fake MCU state transition is interposed between the two intents.
    h.auto_ack, h.ack_disable = False, False
    h.policy_pub.publish(h.envelope(LeasedPolicy, lease, 3,
        MotorPolicyArray(count=1, targets=[
            MotorPolicyTarget(motor_id=1, enabled=False, kp=0., kd=0.)])))
    h.wait(lambda: any(p.targets and not p.targets[0].enabled for p in h.policies)
           and any(m.count == 0 for m in h.trajectories))
    h.policy_pub.publish(h.envelope(LeasedPolicy, lease, 4,
        MotorPolicyArray(count=1, targets=[
            MotorPolicyTarget(motor_id=1, enabled=True, kp=1., kd=.1)])))
    h.wait(lambda: h.latest is not None and not h.latest.armed
           and 'awaiting_disabled' in h.latest.blockers)
    h.spin(.04)
    assert h.latest.owner == 'integration-test'
    assert sum(any(t.enabled for t in p.targets) for p in h.policies) == initially_enabled
    assert any(m.count == 0 for m in h.trajectories)
    assert h.supervisor._command is None

    # The separate fresh off sample is required before explicit arm is possible.
    h.active = False
    h.wait(lambda: h.latest is not None and not h.latest.armed
           and 'awaiting_disabled' not in h.latest.blockers)
    old_target_count = sum(bool(m.count) for m in h.commands)
    h.auto_ack, h.ack_disable = True, True
    h.enable(lease, sequence=5)
    h.spin(.06)
    assert sum(bool(m.count) for m in h.commands) == old_target_count
    h.command(lease, sequence=6, position=.3)
    h.wait(lambda: any(m.count and m.targets[0].position == pytest.approx(.3)
                       for m in h.commands))


def test_command_producer_timeout_cancels_disables_and_does_not_resume(harness):
    h = harness
    lease = h.acquire()
    assert lease.success
    h.enable(lease)
    h.command(lease)
    h.wait(lambda: any(m.count for m in h.commands))
    h.wait(lambda: h.latest is not None and h.latest.fault_latched)
    h.wait(lambda: any(not p.targets[0].enabled for p in h.policies if p.targets))
    assert any(m.count == 0 and m.target_count == 0 for m in h.trajectories)
    assert any(m.count == 0 for m in h.commands)
    assert not h.latest.owner and not h.latest.armed
    before = sum(bool(m.count) for m in h.commands)
    h.command(lease, sequence=100)
    h.spin(.08)
    assert sum(bool(m.count) for m in h.commands) == before


def test_lease_expiry_without_commands_still_requests_disable(harness):
    h = harness
    lease = h.acquire(lease_ms=150)
    assert lease.success
    h.arm(lease)
    h.wait(lambda: h.latest is not None and h.latest.fault_latched)
    h.wait(lambda: any(p.targets and not p.targets[0].enabled for p in h.policies))
    assert not h.latest.owner and not h.latest.armed


def test_old_raw_ingress_never_reaches_managed_outputs(harness):
    h = harness
    h.raw_pub.publish(MotorCommandArray(count=1, targets=[MotorCommandTarget(
        motor_id=1, position=.6, velocity=0., effort=0.)]))
    h.spin(.12)
    assert not any(m.count for m in h.commands)


def test_session_change_retires_control_without_automatic_rearm(harness):
    h = harness
    lease = h.acquire()
    assert lease.success
    h.enable(lease)
    h.session += 1
    h.sequence = 0
    h.wait(lambda: h.latest is not None and h.latest.session_id == h.session
           and h.latest.fault_latched)
    assert not h.latest.armed and not h.latest.owner
    h.command(lease)
    h.spin(.06)
    assert not any(m.count for m in h.commands)


@pytest.mark.parametrize('first_report', ['topology', 'device_parameters'])
def test_new_session_from_other_channels_retires_old_motion_and_accepts_sequence_zero(
        harness, first_report):
    h = harness
    h.wait(lambda: bool(h.supervisor._motion)
           and h.supervisor._motion_sequence is not None
           and h.supervisor._motion_sequence > 0)
    # Drain old-session reports, then let the event-style authority channel be
    # the first observable report of the new session. No motion is published
    # until after the external trajectory rejection below.
    h.health_enabled = False
    h.spin(.06)
    h.session += 1
    h.sequence = 0
    h.motion_reports_enabled = False
    if first_report == 'topology':
        h.topology_pub.publish(MotorTopology(
            session_id=h.session, schema_version=3, entries=[MotorTopologyEntry(
                motor_id=1, bus_index=0, protocol=1, model_name='sample',
                online=True, conflict=False)]))
    else:
        h.params_pub.publish(DeviceParamReport(session_id=h.session, count=1,
            entries=[DeviceParamEntry(motor_id=1, dev_init_state=2,
                record_dirty=False, mismatch_mask=0, dev_safety=0.)]))
    h.wait(lambda: h.supervisor.core.session_id == h.session)
    assert h.supervisor._motion_sequence is None
    assert not h.supervisor._motion

    # Fresh control/identity/calibration reports permit explicit recovery, but
    # they must not make the old-session position valid as a trajectory start.
    h.health_enabled = True
    h.wait(lambda: h.latest is not None and h.latest.session_id == h.session
           and h.latest.fault_latched and 'awaiting_disabled' not in h.latest.blockers)
    reset = h.call(Trigger, '/robot/reset_fault', Trigger.Request())
    assert reset.success, reset.message
    lease = h.acquire(mode=2)
    assert lease.success, lease.reason
    h.enable(lease)
    trajectory = MotorTrajectory(trajectory_id=17, segment_index=0, segment_count=1,
        count=2, target_count=1, target_ids=[1], points=[
            MotorTrajectoryPoint(motor_id=1, time_from_start_ms=0,
                                 position=.1, velocity=0., effort=0.),
            MotorTrajectoryPoint(motor_id=1, time_from_start_ms=1000,
                                 position=.2, velocity=0., effort=0.),
        ])
    h.trajectory_pub.publish(h.envelope(LeasedTrajectory, lease, 2, trajectory))
    h.spin(.06)
    assert not any(m.count for m in h.trajectories)

    # The new source starts its uint32 motion sequence at zero. A retained old
    # sequence watermark would silently drop this fresh pose. Verify through
    # accepted managed output as well as the cache observation.
    h.motion_pub.publish(MotorMotionStateArray(seq=0, session_id=h.session, states=[
        MotorMotionStateTarget(motor_id=1, position=.3,
                               velocity=0., effort=0., state_age_ms=0)]))
    h.wait(lambda: h.supervisor._motion_sequence == 0)
    trajectory.trajectory_id = 18
    trajectory.points[0].position, trajectory.points[1].position = .3, .4
    h.trajectory_pub.publish(h.envelope(LeasedTrajectory, lease, 3, trajectory))
    h.wait(lambda: any(m.count == 2 and m.trajectory_id == 18 for m in h.trajectories))
    output = next(m for m in h.trajectories if m.count == 2 and m.trajectory_id == 18)
    assert output.points[0].position == pytest.approx(.3)


def test_trajectory_start_must_match_fresh_measured_position(harness):
    h = harness
    lease = h.acquire(mode=2)
    assert lease.success
    h.enable(lease)
    trajectory = MotorTrajectory(trajectory_id=7, segment_index=0, segment_count=1,
        count=2, target_count=1, target_ids=[1], points=[
            MotorTrajectoryPoint(motor_id=1, time_from_start_ms=0,
                                 position=.5, velocity=0., effort=0.),
            MotorTrajectoryPoint(motor_id=1, time_from_start_ms=1000,
                                 position=.6, velocity=0., effort=0.),
        ])
    h.trajectory_pub.publish(h.envelope(LeasedTrajectory, lease, 2, trajectory))
    h.spin(.08)
    assert not any(m.count for m in h.trajectories)
    # The rejected intent must not prevent a fresh, measured-start intent.
    trajectory.points[0].position = h.position
    h.trajectory_pub.publish(h.envelope(LeasedTrajectory, lease, 3, trajectory))
    h.wait(lambda: any(m.count == 2 for m in h.trajectories))


def test_stop_is_available_without_a_lease_or_readiness(harness):
    h = harness
    h.commissioned = False
    h.wait(lambda: h.latest is not None and not h.latest.ready)
    response = h.call(Trigger, '/robot/stop', Trigger.Request())
    assert response.success
    h.wait(lambda: any(m.count == 0 for m in h.trajectories))
    assert not h.latest.armed


def test_an_additional_wire_writer_revokes_existing_control(tmp_path):
    # These publishers only provide a discovery graph. They never send wire
    # messages, and the ROS container exposes no serial or CAN hardware.
    h = Harness(tmp_path, exclusive_graph=True)
    try:
        wire_publishers = [h.io.create_publisher(kind, topic, RELIABLE)
                           for kind, topic in (
                               (AgentCommand, AgentWireContract.TOPIC_COMMAND),
                               (AgentTrajectoryRequest, AgentWireContract.TOPIC_TRAJECTORY),
                               (AgentConfigRequest, AgentWireContract.TOPIC_CONFIG),
                           )]
        h.wait(lambda: h.latest is not None and h.latest.ready)
        lease = h.acquire()
        assert lease.success, lease.reason
        h.arm(lease)
        duplicate = h.io.create_publisher(AgentCommand, AgentWireContract.TOPIC_COMMAND, RELIABLE)
        h.wait(lambda: h.latest is not None and h.latest.fault_latched)
        assert not h.latest.owner and not h.latest.armed
        assert any('publisher_count:' in blocker for blocker in h.latest.blockers)
        h.wait(lambda: any(p.targets and not p.targets[0].enabled for p in h.policies))
        h.io.destroy_publisher(duplicate)
        h.spin(.3)
        assert h.latest.fault_latched and not h.latest.armed
        assert all(wire_publishers)  # retain ownership until the harness closes
    finally:
        h.close()


def test_standard_diagnostics_report_readiness_rejection_and_fault_without_token(tmp_path):
    h = Harness(tmp_path, domain_id=191)
    received = []
    h.subscriptions.append(h.io.create_subscription(
        DiagnosticArray, '/diagnostics', received.append, RELIABLE))

    def latest(name):
        for array in reversed(received):
            for status in array.status:
                if status.name == 'robot_supervisor/' + name:
                    return status
        return None

    try:
        h.commissioned = False
        h.wait(lambda: latest('readiness') is not None)
        assert latest('readiness').level == DiagnosticStatus.WARN
        assert latest('readiness').hardware_id == 'fixture-01'
        h.commissioned = True
        h.wait(lambda: latest('readiness').level == DiagnosticStatus.OK)
        lease = h.acquire()
        assert lease.success, lease.reason
        h.enable(lease)
        h.command(lease, token='wrong-' + lease.lease_id)
        h.wait(lambda: latest('control_input') is not None
               and latest('control_input').level == DiagnosticStatus.WARN)
        values = {entry.key: entry.value for entry in latest('control_input').values}
        assert 'control_lease' in values['last_rejection_reason']
        assert lease.lease_id not in repr(received)
        before = len(received)
        for number in range(20):
            h.command(lease, sequence=3 + number, token='wrong-' + lease.lease_id)
            h.spin(.005)
        h.spin(.2)
        assert len(received) <= before + 1  # No diagnostic message per command.
        stopped = h.call(Trigger, '/robot/stop', Trigger.Request())
        assert stopped.success
        h.wait(lambda: latest('readiness').level == DiagnosticStatus.ERROR)
        assert 'operator_stop' in latest('readiness').message
        assert received[-1].header.stamp.sec > 0
        assert lease.lease_id not in repr(received)
    finally:
        h.close()
