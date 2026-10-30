"""Production supervisor and C++ bridge over DDS, with a device-free MCU peer.

Only test fixture identity/measurements are simulated. Every host command and
lease crosses the production facade remaps and wire codecs. MCU deadline
enforcement itself is tested separately against the firmware's C library.
"""
from __future__ import annotations

import copy
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

import pytest
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_prefix
from builtin_interfaces.msg import Duration
from can_bridge_msgs.msg import (
    ConfigRequest, ControlLease, DeviceParamEntry, DeviceParamReport, LowCommand, LowState,
    MotorCommandArray, MotorCommandTarget, MotorControlState, MotorControlStateArray,
    MotorPolicyArray, MotorPolicyTarget, MotorStatus, MotorTopology, MotorTopologyEntry,
    MotorTrajectory, WireContract,
    SystemCommand,
)
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robot_calibration.validation import compute_fingerprint
from robot_supervisor.node import SupervisorNode
from robot_system_msgs.msg import CalibrationState, LeasedCommand, LeasedPolicy, RobotState
from robot_system_msgs.srv import AcquireControl, AuthorizeMotion, SetArmed
from trajectory_msgs.msg import JointTrajectoryPoint
from unique_identifier_msgs.msg import UUID


SESSION = 1_800_000_000_000
DOMAIN = 198
RELIABLE = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
STREAM = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def fixture_files(folder):
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


class WireHarness:
    def __init__(self, folder):
        driver, urdf = fixture_files(folder)
        self.context = Context()
        rclpy.init(args=[], context=self.context, domain_id=DOMAIN)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.io = Node('fake_wire_mcu', context=self.context)
        self.supervisor = SupervisorNode(context=self.context, parameter_overrides=[
            Parameter('driver_file', value=str(driver)), Parameter('urdf_file', value=str(urdf)),
            Parameter('robot_id', value='fixture-01'), Parameter('model_name', value='fixture'),
            Parameter('require_exclusive_bridge', value=True),
            Parameter('allow_trajectories', value=True),
        ])
        self.executor.add_node(self.io)
        self.executor.add_node(self.supervisor)
        self.hash = compute_fingerprint(str(driver), str(urdf))
        self.sequence = 0
        self.generation = 41
        self.active = False
        self.enabled = False
        self.ack_lease = True
        self.grant = None
        self.received_grants, self.received_policies = [], []
        self.received_commands, self.received_trajectories = [], []
        self.trajectory, self.trajectory_tick = None, 0
        self.position = .1
        self.latest = None
        self.statuses = self.io.create_publisher(MotorStatus, WireContract.TOPIC_MOTOR_STATUS, RELIABLE)
        self.topologies = self.io.create_publisher(MotorTopology, WireContract.TOPIC_TOPOLOGY, RELIABLE)
        self.controls = self.io.create_publisher(MotorControlStateArray, WireContract.TOPIC_CONTROL_STATES, RELIABLE)
        self.params = self.io.create_publisher(DeviceParamReport, WireContract.TOPIC_DEVICE_PARAM, RELIABLE)
        self.motion = self.io.create_publisher(LowState, WireContract.TOPIC_LOW_STATE, STREAM)
        self.calibration = self.io.create_publisher(CalibrationState, '/robot/calibration_state', LATCHED)
        self.policy = self.io.create_publisher(LeasedPolicy, '/robot/control/policy', RELIABLE)
        self.command = self.io.create_publisher(LeasedCommand, '/robot/control/command', RELIABLE)
        self.subscriptions = [
            self.io.create_subscription(ControlLease, WireContract.TOPIC_CONTROL_LEASE, self.on_grant, RELIABLE),
            self.io.create_subscription(MotorPolicyArray, WireContract.TOPIC_POLICY, self.on_policy, RELIABLE),
            self.io.create_subscription(LowCommand, WireContract.TOPIC_LOW_CMD,
                                        self.on_command, STREAM),
            self.io.create_subscription(MotorTrajectory, WireContract.TOPIC_TRAJECTORY,
                                        self.on_trajectory, RELIABLE),
            self.io.create_subscription(RobotState, '/robot/state', self.on_state, LATCHED),
        ]
        executable = Path(get_package_prefix('micro_ros_bridge')) / 'lib/micro_ros_bridge/micro_ros_bridge_node'
        remaps = {
            '/motor/command_array': '/robot/managed/motor/command_array',
            '/motor/policy_array': '/robot/managed/motor/policy_array',
            '/motor/trajectory': '/robot/managed/motor/trajectory',
            '/motor/control_lease': '/robot/managed/motor/control_lease',
            '/motor/param_set': '/robot/managed/maintenance/param_set',
            '/motor/system_command': '/robot/managed/maintenance/system_command',
            '/motor/execute_config': '/robot/managed/maintenance/execute_config',
        }
        arguments = [str(executable), '--ros-args']
        for source, target in remaps.items():
            arguments += ['-r', source + ':=' + target]
        self.log = (folder / 'bridge.log').open('w+')
        self.bridge = subprocess.Popen(arguments, env={**os.environ, 'ROS_DOMAIN_ID': str(DOMAIN),
                                                     'ROS_LOCALHOST_ONLY': '1'},
                                       stdout=self.log, stderr=subprocess.STDOUT)
        self.timer = self.io.create_timer(.02, self.health)
        self.action = ActionClient(self.io, FollowJointTrajectory, '/robot/follow_joint_trajectory')

    @staticmethod
    def tick():
        return int(time.monotonic() * 1000) & 0xFFFFFFFF

    def on_state(self, message):
        self.latest = message

    def on_grant(self, message):
        self.received_grants.append(copy.deepcopy(message))
        if self.ack_lease:
            self.grant = copy.deepcopy(message)
            if not message.active:
                self.active = False
                self.enabled = False
                self.trajectory = None

    def on_policy(self, message):
        self.received_policies.append(copy.deepcopy(message))
        for target in message.targets:
            self.enabled = target.enabled
            if not target.enabled:
                self.active = False
            self.generation = message.policy_generation

    def on_command(self, message):
        self.received_commands.append(copy.deepcopy(message))
        if message.count and self.enabled:
            self.active = True

    def on_trajectory(self, message):
        self.received_trajectories.append(copy.deepcopy(message))
        self.trajectory = copy.deepcopy(message) if message.count else None
        if self.trajectory and self.enabled:
            self.active = True
        self.trajectory_tick = self.tick()

    def health(self):
        self.sequence += 1
        stamp = self.io.get_clock().now().nanoseconds
        tick = self.tick()
        grant = self.grant
        lease_active = bool(grant and grant.active and
                            0 < ((grant.valid_until_tick_ms - tick) & 0xFFFFFFFF) <= 300)
        if not lease_active:
            self.active = False
            self.enabled = False
            self.trajectory = None
        trajectory = self.trajectory
        elapsed, total, velocity = 0, 0, 0.
        if trajectory:
            elapsed = (tick - self.trajectory_tick) & 0xFFFFFFFF
            total = trajectory.points[-1].time_from_start_ms
            start, end = trajectory.points[0].position, trajectory.points[-1].position
            u = min(1., elapsed / total)
            self.position = start + (end - start) * (3*u*u - 2*u*u*u)
            velocity = (end-start) * (6*u - 6*u*u) / (total/1000.) if elapsed < total else 0.
        self.statuses.publish(MotorStatus(sync_time_ns=stamp, session_id=SESSION,
            sequence=self.sequence, tick_ms=tick, schema_version=WireContract.SCHEMA_VERSION,
            trajectory_id=trajectory.trajectory_id if trajectory else 0,
            traj_active=bool(trajectory), traj_completed=bool(trajectory and elapsed >= total),
            traj_elapsed_ms=elapsed, traj_total_ms=total,
            traj_target_count=1 if trajectory else 0, traj_target_ids=[1] if trajectory else []))
        self.topologies.publish(MotorTopology(sync_time_ns=stamp, session_id=SESSION,
            schema_version=WireContract.SCHEMA_VERSION, entries=[MotorTopologyEntry(
                motor_id=1, bus_index=0, protocol=1, model_name='sample', online=True)]))
        self.params.publish(DeviceParamReport(session_id=SESSION, count=1,
            entries=[DeviceParamEntry(motor_id=1, dev_init_state=2, dev_safety=0.)]))
        self.controls.publish(MotorControlStateArray(sync_time_ns=stamp,
            schema_version=MotorControlStateArray.SCHEMA_VERSION, session_id=SESSION,
            sequence=self.sequence, tick_ms=tick,
            lease_id=grant.lease_id if grant else 0, lease_sequence=grant.sequence if grant else 0,
            lease_active=lease_active,
            lease_valid_until_tick_ms=grant.valid_until_tick_ms if grant else 0,
            states=[MotorControlState(motor_id=1, online=True,
                control_lifecycle=3 if self.active else 2,
                execution_state=4 if trajectory else 3 if self.active else 1,
                policy_enabled=self.enabled, motion_enabled=self.active,
                state_age_ms=0, policy_generation=self.generation)]))
        self.motion.publish(LowState(sync_time_ns=stamp, session_id=SESSION,
            seq=self.sequence, tick_ms=tick, schema_version=WireContract.SCHEMA_VERSION,
            chunk_count=1, total_count=1, count=1, motor_ids=[1],
            position_q=[round(self.position * 32767 / WireContract.SCALE_POSITION_RAD)],
            velocity_q=[round(velocity * 32767 / WireContract.SCALE_VELOCITY_RAD_S)],
            effort_q=[0], state_age_ms=[0], target_source=[2 if trajectory else 1 if self.active else 0]))
        report = CalibrationState(robot_id='fixture-01', model_name='fixture',
            calibration_id='test-fixture-only', config_hash=self.hash, valid=True,
            commissioned=True, motor_ids=[1], bus_indices=[0], protocols=[1],
            model_names=['sample'], joint_names=['j1'])
        report.header.stamp = self.io.get_clock().now().to_msg()
        self.calibration.publish(report)

    def spin(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=.005)

    def wait(self, predicate, timeout=3.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            assert self.bridge.poll() is None, 'production bridge exited unexpectedly'
            self.executor.spin_once(timeout_sec=.005)
            if predicate():
                return
        self.log.flush()
        self.log.seek(0)
        raise AssertionError(f'timed out: state={self.latest}\n{self.log.read()}')

    def call(self, service, name, request):
        client = self.io.create_client(service, '/robot/' + name)
        try:
            self.wait(client.service_is_ready)
            future = client.call_async(request)
            self.wait(future.done)
            return future.result()
        finally:
            self.io.destroy_client(client)

    def policy_message(self, lease, sequence):
        message = LeasedPolicy(lease_id=lease.lease_id, epoch=lease.epoch,
            session_id=lease.session_id, sequence=sequence,
            policy=MotorPolicyArray(count=1, targets=[
                MotorPolicyTarget(motor_id=1, enabled=True, kp=1., kd=.1)]))
        message.header.stamp = self.io.get_clock().now().to_msg()
        return message

    def close(self):
        self.action.destroy()
        self.executor.remove_node(self.supervisor)
        self.supervisor.destroy_node()
        self.executor.remove_node(self.io)
        self.io.destroy_node()
        self.executor.shutdown()
        self.context.shutdown()
        if self.bridge.poll() is None:
            self.bridge.send_signal(signal.SIGINT)
            try:
                self.bridge.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.bridge.kill()
                self.bridge.wait(timeout=3)
        self.log.close()


def test_wire_handshake_standard_action_and_no_bridge_renewal(tmp_path):
    h = WireHarness(tmp_path)
    try:
        h.wait(lambda: h.latest is not None and h.latest.ready)
        lease = h.call(AcquireControl, 'acquire_control', AcquireControl.Request(
            owner='wire-integration', mode=2, lease_ms=2000))
        assert lease.success, lease.reason
        h.ack_lease = False
        armed = h.call(SetArmed, 'set_armed', SetArmed.Request(
            lease_id=lease.lease_id, epoch=lease.epoch, armed=True))
        assert armed.success, armed.reason
        h.wait(lambda: any(m.active for m in h.received_grants))
        h.policy.publish(h.policy_message(lease, 1))
        h.spin(.03)
        assert not any(any(t.enabled for t in m.targets) for m in h.received_policies)
        h.ack_lease = True
        h.wait(lambda: h.supervisor._mcu_lease.confirmed)
        h.policy.publish(h.policy_message(lease, 2))
        h.wait(lambda: h.enabled and h.supervisor._authority.policy_ready(1, time.monotonic(), .5))
        assert not h.active  # A policy alone does not make the MCU lifecycle ACTIVE.
        grant_id = h.grant.lease_id
        assert all(m.lease_id == grant_id for m in h.received_policies if any(t.enabled for t in m.targets))

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['j1']
        goal.trajectory.points = [
            JointTrajectoryPoint(positions=[h.position], time_from_start=Duration()),
            JointTrajectoryPoint(positions=[.12], time_from_start=Duration(nanosec=200_000_000)),
        ]
        goal_id = uuid.uuid4().bytes
        request = AuthorizeMotion.Request(lease_id=lease.lease_id, epoch=lease.epoch,
            session_id=lease.session_id, sequence=3, goal_id=list(goal_id))
        request.header.stamp = h.io.get_clock().now().to_msg()
        permission = h.call(AuthorizeMotion, 'authorize_motion', request)
        assert permission.success, permission.reason
        h.wait(h.action.server_is_ready)
        feedback = []
        sent = h.action.send_goal_async(goal, goal_uuid=UUID(uuid=list(goal_id)),
                                       feedback_callback=feedback.append)
        h.wait(sent.done)
        handle = sent.result()
        assert handle.accepted
        result = handle.get_result_async()
        h.wait(result.done)
        assert result.result().status == GoalStatus.STATUS_SUCCEEDED
        assert result.result().result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        assert feedback and any(m.count and m.lease_id == grant_id for m in h.received_trajectories)
        assert h.active  # Successful Action preserves final hold under explicit lease.

        command = LeasedCommand(lease_id=lease.lease_id, epoch=lease.epoch,
            session_id=lease.session_id, sequence=4,
            command=MotorCommandArray(count=1, targets=[
                MotorCommandTarget(motor_id=1, position=.12)]))
        command.header.stamp = h.io.get_clock().now().to_msg()
        h.command.publish(command)
        h.wait(lambda: any(m.count for m in h.received_commands))
        assert all(m.lease_id == grant_id for m in h.received_commands if m.count)

        # Simulate a stuck supervisor: its executor stops running while the
        # bridge and MCU peer continue. The bridge must not synthesize renewals.
        h.executor.remove_node(h.supervisor)
        h.spin(.04)  # Drain any already queued lease publication.
        count = len(h.received_grants)
        h.spin(.32)
        assert len(h.received_grants) == count
        assert h.bridge.poll() is None
        assert not h.active

        # Even a fresh facade publication cannot revive the expired generation.
        facade = h.io.create_publisher(MotorTrajectory, '/robot/managed/motor/trajectory', RELIABLE)
        h.wait(lambda: facade.get_subscription_count() == 1)
        stale = copy.deepcopy(next(m for m in h.received_trajectories if m.count))
        stale.sync_time_ns = h.io.get_clock().now().nanoseconds
        before = len([m for m in h.received_trajectories if m.count])
        facade.publish(stale)
        h.spin(.08)
        assert len([m for m in h.received_trajectories if m.count]) == before
    finally:
        h.close()


def test_wire_action_cancel_waits_for_mcu_disabled_acknowledgement(tmp_path):
    h = WireHarness(tmp_path)
    try:
        h.wait(lambda: h.latest is not None and h.latest.ready)
        lease = h.call(AcquireControl, 'acquire_control', AcquireControl.Request(
            owner='wire-cancel-integration', mode=2, lease_ms=2000))
        assert lease.success, lease.reason
        armed = h.call(SetArmed, 'set_armed', SetArmed.Request(
            lease_id=lease.lease_id, epoch=lease.epoch, armed=True))
        assert armed.success, armed.reason
        h.wait(lambda: h.supervisor._mcu_lease.confirmed)
        h.policy.publish(h.policy_message(lease, 1))
        h.wait(lambda: h.enabled and h.supervisor._authority.policy_ready(1, time.monotonic(), .5))
        assert not h.active
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['j1']
        goal.trajectory.points = [
            JointTrajectoryPoint(positions=[h.position], time_from_start=Duration()),
            JointTrajectoryPoint(positions=[.15], time_from_start=Duration(sec=1)),
        ]
        goal_id = uuid.uuid4().bytes
        request = AuthorizeMotion.Request(lease_id=lease.lease_id, epoch=lease.epoch,
            session_id=lease.session_id, sequence=2, goal_id=list(goal_id))
        request.header.stamp = h.io.get_clock().now().to_msg()
        permission = h.call(AuthorizeMotion, 'authorize_motion', request)
        assert permission.success, permission.reason
        h.wait(h.action.server_is_ready)
        sent = h.action.send_goal_async(goal, goal_uuid=UUID(uuid=list(goal_id)))
        h.wait(sent.done)
        handle = sent.result()
        assert handle.accepted
        h.wait(lambda: any(m.count for m in h.received_trajectories))
        result = handle.get_result_async()
        cancel = handle.cancel_goal_async()
        h.wait(cancel.done)
        assert any(bytes(item.goal_id.uuid) == goal_id for item in cancel.result().goals_canceling)
        h.wait(result.done)
        assert result.result().status == GoalStatus.STATUS_CANCELED
        assert result.result().result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        assert any(not message.active for message in h.received_grants)
        assert any(not message.count for message in h.received_trajectories)
        assert not h.active and not h.enabled
        assert not h.supervisor.core.armed
        assert 'awaiting_disabled' not in h.supervisor.core.snapshot()['blockers']
    finally:
        h.close()


def test_parameter_report_uses_bridge_request_id_namespace(tmp_path):
    h = WireHarness(tmp_path)
    reports = []
    h.subscriptions.append(h.io.create_subscription(
        ConfigRequest, WireContract.TOPIC_CONFIG, reports.append, RELIABLE))
    # Policy correlation IDs and the MCU's configuration request namespace are
    # distinct. A supervisor restart must not guess the latter's high water.
    h.supervisor._request_id = 0x7FFFFFF0
    try:
        h.wait(lambda: h.latest is not None and h.latest.ready)
        h.wait(lambda: any(message.command == SystemCommand.COMMAND_REPORT for message in reports))
        report = next(message for message in reports if message.command == SystemCommand.COMMAND_REPORT)
        assert 0 < report.request_id < 0x40000000
        assert report.session_id == SESSION
    finally:
        h.close()
