"""公开转换接口及隔离 ROS 图；不启动 Agent、设备或运行服务。"""
import copy
import json
import math
from pathlib import Path
import sys
import threading
import time
from types import MethodType, SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from can_bridge_msgs.msg import (MotorCommandArray, MotorCommandTarget, MotorTrajectory,
                                 MotorPolicyArray, MotorTrajectoryPoint, MotorTrajectoryState)
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers, SwitchController
from rcl_interfaces.msg import ParameterType, ParameterValue, SetParametersResult
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.serialization import deserialize_message, serialize_message
from rclpy.task import Future
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float64MultiArray, Int32, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from motor_hardware_interface import workbench_control as wc

NOW = 10_000_000_000


@pytest.fixture
def files(tmp_path):
    driver = tmp_path / 'motors.yaml'
    driver.write_text('motor_hardware:\n  motors:\n    a: {id: 8}\n    b: {id: 9}\n'
                      '    c: {id: 10, joint: elbow}\n')
    (tmp_path / 'transmissions.yaml').write_text(
        'transmissions:\n  wrist:\n'
        '    plugin: transmission_interface/DifferentialTransmission\n'
        '    actuators: [a, b]\n    joints: [yaw, pitch]\n')
    urdf = tmp_path / 'test.urdf'
    urdf.write_text('<robot name="test"><link name="base"/>' + ''.join(
        f'<link name="{name}_link"/><joint name="{name}" type="revolute">'
        f'<parent link="base"/><child link="{name}_link"/>'
        '<limit lower="-1" upper="1" velocity="2" effort="10"/></joint>'
        for name in ('yaw', 'pitch', 'elbow')) + '</robot>')
    return str(driver), str(urdf)


def feedback(stamp=NOW):
    msg = JointState(name=['yaw', 'pitch', 'elbow'], position=[.2, .1, .3],
                     velocity=[0., 0., 0.], effort=[0., 0., 0.])
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 10**9)
    return msg


def setpoint(stamp=NOW):
    return MotorCommandArray(sync_time_ns=stamp, count=2, targets=[
        MotorCommandTarget(motor_id=8, position=.4, velocity=.2, effort=1.),
        MotorCommandTarget(motor_id=9, position=.2, velocity=.0, effort=1.)])


def trajectory(position=.4):
    return JointTrajectory(joint_names=['yaw'], points=[JointTrajectoryPoint(
        positions=[position], time_from_start=wc.Duration(sec=1))])


def segment(index=0, count=1, trajectory_id=41, times=(1000,)):
    points = [MotorTrajectoryPoint(time_from_start_ms=t, motor_id=i,
                                  position=.2, velocity=0., effort=0.)
              for t in times for i in (8, 9)]
    return MotorTrajectory(sync_time_ns=NOW, session_id=3, sequence=7,
                           trajectory_id=trajectory_id, segment_index=index,
                           segment_count=count, count=len(points), target_count=2,
                           target_ids=[8, 9], points=points)


def motor_plan(times, positions, velocities=None):
    velocities = [0.] * len(times) if velocities is None else velocities
    message = MotorTrajectory(sync_time_ns=NOW, session_id=3, trajectory_id=41,
        segment_count=1, count=len(times), target_count=1, target_ids=[10],
        points=[MotorTrajectoryPoint(motor_id=10, time_from_start_ms=t, position=p, velocity=v)
                for t, p, v in zip(times, positions, velocities)])
    wire = deserialize_message(serialize_message(message), MotorTrajectory)
    return wc.MotorPlan(41, [10], NOW, list(wire.points))


def test_manual_uses_model_and_fresh_measured_holds(files):
    model = wc.WorkbenchModel(*files)
    measured = model.feedback(feedback(), NOW)
    values = model.manual(setpoint(), measured, NOW)
    assert values == pytest.approx([.3, .1, 2., .1, .1, 0., .3, 0., 0.])
    assert model.joint_names == ['yaw', 'pitch', 'elbow']


@pytest.mark.parametrize('change', ['partial', 'duplicate', 'unknown', 'count',
                                  'nan', 'position', 'velocity', 'effort', 'stale', 'future'])
def test_manual_rejects_whole_invalid_input(files, change):
    model = wc.WorkbenchModel(*files)
    msg = setpoint()
    if change == 'partial':
        msg.targets.pop(); msg.count = 1
    elif change == 'duplicate':
        msg.targets[1].motor_id = 8
    elif change == 'unknown':
        msg.targets[1].motor_id = 42
    elif change == 'count':
        msg.count = 1
    elif change in ('nan', 'position'):
        msg.targets[0].position = math.nan if change == 'nan' else 3.
    elif change == 'velocity':
        msg.targets[0].velocity = 8.
    elif change == 'effort':
        msg.targets[0].effort = 30.
    else:
        msg.sync_time_ns += -1_000_000_000 if change == 'stale' else 1_000_000_000
    with pytest.raises(ValueError):
        model.manual(msg, model.feedback(feedback(), NOW), NOW)


@pytest.mark.parametrize('channel', wc.CHANNELS)
@pytest.mark.parametrize('sign', [-1., 1.])
def test_manual_normalizes_only_float32_boundary_rounding(files, channel, sign):
    model = wc.WorkbenchModel(*files)
    model.limits['elbow'] = (-.14, .14, .14, .14)
    target = MotorCommandTarget(motor_id=10)
    setattr(target, channel, sign * .14)
    message = MotorCommandArray(sync_time_ns=NOW, count=1, targets=[target])
    wire = deserialize_message(serialize_message(message), MotorCommandArray)
    assert abs(getattr(wire.targets[0], channel)) > .14
    values = model.manual(wire, model.feedback(feedback(), NOW), NOW)
    assert values[6 + wc.CHANNELS.index(channel)] == sign * .14
    setattr(target, channel, sign * (.14 + .0001))
    wire = deserialize_message(serialize_message(message), MotorCommandArray)
    with pytest.raises(ValueError):
        model.manual(wire, model.feedback(feedback(), NOW), NOW)


@pytest.mark.parametrize('bound', [.14, 2.7])
@pytest.mark.parametrize('sign', [-1., 1.])
def test_motor_trajectory_normalizes_wire_endpoints_to_current_host_limits(files, bound, sign):
    model = wc.WorkbenchModel(*files)
    model.limits['elbow'] = (-bound, bound, 2., 10.)
    state = feedback()
    state.position[2] = 0.
    measured = model.feedback(state, NOW)
    for times in [(2500,), (2500, 5000)]:
        message = MotorTrajectory(sync_time_ns=NOW, session_id=3, trajectory_id=41,
            segment_count=1, count=len(times), target_count=1, target_ids=[10],
            points=[MotorTrajectoryPoint(motor_id=10, time_from_start_ms=t,
                                        position=sign * bound) for t in times])
        wire = deserialize_message(serialize_message(message), MotorTrajectory)
        assert abs(wire.points[0].position) > bound
        plan = wc.TrajectoryAssembly().add(wire, NOW, 1.)
        result = model.motor_trajectory(plan, measured, NOW)
        assert all(point.positions[2] == sign * bound for point in result.points)
        message.points[0].position = sign * (bound + .0001)
        wire = deserialize_message(serialize_message(message), MotorTrajectory)
        plan = wc.TrajectoryAssembly().add(wire, NOW, 1.)
        with pytest.raises(ValueError):
            model.motor_trajectory(plan, measured, NOW)


def test_motor_trajectory_uses_position_time_instead_of_overshooting_diagnostic_velocity(files):
    model = wc.WorkbenchModel(*files)
    model.limits['elbow'] = (-.12, .14, 2., 10.)
    state = feedback()
    state.position[2] = .14
    measured = model.feedback(state, NOW)
    plan = motor_plan([2500, 2542, 2584, 2625],
        [.14, .14, .13959224309518609, .1339594744709353],
        [0., 0., -.060231347449353254, -.285235839990286])
    original = [(p.position, p.velocity) for p in plan.points]
    explicit = JointTrajectory(joint_names=['elbow'], points=[JointTrajectoryPoint(
        positions=[p.position], velocities=[p.velocity],
        time_from_start=wc.Duration(sec=p.time_from_start_ms // 1000,
                                   nanosec=p.time_from_start_ms % 1000 * 10**6)) for p in plan.points])
    with pytest.raises(ValueError, match='插值越过位置限位'):
        model.joint_trajectory(explicit, measured, NOW)
    result = model.motor_trajectory(plan, measured, NOW)
    assert [p.positions[2] for p in result.points] == pytest.approx([p for p, _ in original])
    assert result.points[2].velocities[2] < 0.
    assert result.points[-1].velocities[2] == 0.
    assert [(p.position, p.velocity) for p in plan.points] == original


@pytest.mark.parametrize('diagnostic_velocity', [0., 1e30])
def test_motor_trajectory_derives_continuous_velocity_with_unequal_intervals(files, diagnostic_velocity):
    model = wc.WorkbenchModel(*files)
    state = feedback()
    state.position[2] = 0.
    plan = motor_plan([1000, 2000, 4000], [0., .1, .5], [diagnostic_velocity] * 3)
    result = model.motor_trajectory(plan, model.feedback(state, NOW), NOW)
    assert [p.velocities[2] for p in result.points] == pytest.approx([0., 9./70., 0.])
    assert [wc.stamp_ns(p.time_from_start) // 10**6 for p in result.points] == [1000, 2000, 4000]


def test_motor_trajectory_stops_tangent_at_extrema_and_plateaus(files):
    model = wc.WorkbenchModel(*files)
    state = feedback()
    state.position[2] = 0.
    plan = motor_plan([1000, 2000, 4000, 5000, 6000, 7000], [0., .1, .3, .2, .2, 0.])
    result = model.motor_trajectory(plan, model.feedback(state, NOW), NOW)
    assert [p.velocities[2] for p in result.points] == pytest.approx([0., .1, 0., 0., 0., 0.])


def test_motor_trajectory_maps_complete_coupled_groups_before_deriving_velocity(files):
    model = wc.WorkbenchModel(*files)
    message = segment(times=(1000, 2000, 4000))
    for index, point in enumerate(message.points):
        point.position = .2 + .2 * (index // 2) if point.motor_id == 8 else .2
        point.velocity = 1e30
    wire = deserialize_message(serialize_message(message), MotorTrajectory)
    plan = wc.TrajectoryAssembly().add(wire, NOW, 1.)
    measured = model.feedback(feedback(), NOW)
    result = model.motor_trajectory(plan, measured, NOW)
    for point, expected in zip(result.points, [[.2, 0., .3], [.3, .1, .3], [.4, .2, .3]]):
        assert list(point.positions) == pytest.approx(expected)
    assert list(result.points[1].velocities) == pytest.approx([9./130., 9./130., 0.])
    plan.points.pop(3)
    with pytest.raises(ValueError, match='完整传动组'):
        model.motor_trajectory(plan, measured, NOW)


def test_raw_motor_trajectory_preserves_linear_time_without_cubic_speed_peak(files):
    model = wc.WorkbenchModel(*files)
    model.limits['elbow'] = (-1., 1., 3.1416, 10.)
    state = feedback()
    state.position[2] = 0.
    measured = model.feedback(state, NOW)
    plan = motor_plan([1000, 1042], [0., .12])
    with pytest.raises(ValueError, match='插值越过速度限位'):
        model.motor_trajectory(plan, measured, NOW)
    result = model.motor_trajectory(plan, measured, NOW, linear=True)
    assert [p.positions[2] for p in result.points] == pytest.approx([0., .12])
    assert [wc.stamp_ns(p.time_from_start) // 10**6 for p in result.points] == [1000, 1042]
    assert all(not p.velocities for p in result.points)


@pytest.mark.parametrize('invalid', ['zero_mismatch', 'initial_outside'])
def test_raw_motor_trajectory_requires_continuous_in_bounds_start(files, invalid):
    model = wc.WorkbenchModel(*files)
    state = feedback()
    measured = model.feedback(state, NOW)
    if invalid == 'zero_mismatch':
        plan = motor_plan([0, 1000], [.3002, .4])
    else:
        model.limits['elbow'] = (-.2, .2, 2., 10.)
        plan = motor_plan([1000, 2000], [.1, .1])
    with pytest.raises(ValueError):
        model.motor_trajectory(plan, measured, NOW, linear=True)


@pytest.mark.parametrize('linear', [False, True])
@pytest.mark.parametrize('invalid', ['position', 'speed', 'nan_velocity', 'nan_position',
                                    'effort', 'time'])
def test_motor_trajectory_position_time_conversion_keeps_validation(files, invalid, linear):
    model = wc.WorkbenchModel(*files)
    plan = motor_plan([1000, 2000, 3000], [0., .1, .2])
    if invalid == 'position':
        plan.points[-1].position = 1.0001
    elif invalid == 'speed':
        plan.points[1].time_from_start_ms = 1001
        plan.points[2].time_from_start_ms = 1002
    elif invalid == 'nan_velocity':
        plan.points[1].velocity = math.nan
    elif invalid == 'nan_position':
        plan.points[1].position = math.nan
    elif invalid == 'effort':
        plan.points[1].effort = .1
    else:
        plan.points[1].time_from_start_ms = 500
    with pytest.raises(ValueError):
        model.motor_trajectory(plan, model.feedback(feedback(), NOW), NOW, linear=linear)


@pytest.mark.parametrize('change', ['stale', 'future', 'nan', 'partial', 'duplicate', 'zero'])
def test_feedback_fails_closed(files, change):
    model = wc.WorkbenchModel(*files)
    msg = feedback()
    if change == 'stale':
        msg.header.stamp.sec -= 1
    elif change == 'future':
        msg.header.stamp.sec += 1
    elif change == 'nan':
        msg.position[0] = math.nan
    elif change == 'partial':
        msg.name.pop(); msg.position.pop(); msg.velocity.pop(); msg.effort.pop()
    elif change == 'duplicate':
        msg.name[1] = 'yaw'
    else:
        msg.header.stamp.sec = 0
    with pytest.raises(ValueError):
        model.feedback(msg, NOW)


def test_joint_trajectory_fills_every_joint_from_measurement(files):
    model = wc.WorkbenchModel(*files)
    result = model.joint_trajectory(trajectory(), model.feedback(feedback(), NOW), NOW)
    assert result.joint_names == model.joint_names
    assert result.points[0].positions == pytest.approx([.4, .1, .3])
    assert list(result.points[0].velocities) == [0., 0., 0.]
    assert not result.points[0].effort


def test_zero_time_point_normalizes_quantization_error_to_measured_state(files):
    model = wc.WorkbenchModel(*files)
    state = feedback()
    state.velocity[0] = .01
    message = trajectory()
    message.points.insert(0, JointTrajectoryPoint(positions=[.2002]))
    result = model.joint_trajectory(message, model.feedback(state, NOW), NOW)
    assert list(result.points[0].positions) == pytest.approx(state.position)
    assert list(result.points[0].velocities) == pytest.approx(state.velocity)
    message.points[0].positions = [.202]
    with pytest.raises(ValueError):
        model.joint_trajectory(message, model.feedback(state, NOW), NOW)
    state.position[0] = .9999
    message.points[0].positions = [1.0001]
    with pytest.raises(ValueError):
        model.joint_trajectory(message, model.feedback(state, NOW), NOW)


@pytest.mark.parametrize('change', ['shape', 'duplicate', 'unknown', 'time', 'position',
                                  'velocity', 'effort', 'nan', 'speed', 'acceleration'])
def test_joint_trajectory_rejects_invalid_or_unsupported_data(files, change):
    model = wc.WorkbenchModel(*files)
    msg = trajectory()
    point = msg.points[0]
    if change == 'shape':
        point.velocities = [0., 0.]
    elif change == 'duplicate':
        msg.joint_names = ['yaw', 'yaw']; point.positions = [.4, .4]
    elif change == 'unknown':
        msg.joint_names = ['other']
    elif change == 'time':
        point.time_from_start.sec = -1
    elif change == 'position':
        point.positions = [1.1]
    elif change == 'velocity':
        point.velocities = [2.1]
    elif change == 'effort':
        point.effort = [.1]
    elif change == 'nan':
        point.positions = [math.nan]
    elif change == 'speed':
        point.time_from_start = wc.Duration(nanosec=10_000_000)
    else:
        point.accelerations = [1.]
    with pytest.raises(ValueError):
        model.joint_trajectory(msg, model.feedback(feedback(), NOW), NOW)


def test_segment_assembly_is_atomic_and_exceeds_single_message_capacity(files):
    model = wc.WorkbenchModel(*files)
    assembly = wc.TrajectoryAssembly()
    result = None
    for index in range(3):
        msg = segment(index, 3, times=range(1000 + index * 1280, 1000 + (index + 1) * 1280, 10))
        result = assembly.add(msg, NOW, 1. + index * .01)
        if index < 2:
            assert result is None
    converted = model.motor_trajectory(result, model.feedback(feedback(), NOW), NOW)
    assert len(converted.points) == 384
    assert converted.points[-1].time_from_start.sec == 4
    with pytest.raises(ValueError):
        assembly.add(msg, NOW, 1.1)


@pytest.mark.parametrize('overflow', ['points', 'segments'])
def test_segment_assembly_rejects_total_capacity_overflow(overflow):
    assembly = wc.TrajectoryAssembly()
    if overflow == 'points':
        for index in range(32):
            msg = segment(index, 33, times=range(1000 + index * 1280,
                                                1000 + (index + 1) * 1280, 10))
            assert assembly.add(msg, NOW, 1.) is None
        assert len(assembly.pending.points) == 8192
        msg = segment(32, 33, times=(50000,))
    else:
        msg = segment(0, 8193)
    with pytest.raises(ValueError, match='8192'):
        assembly.add(msg, NOW, 1.)
    assert assembly.pending is None
    assert (msg.session_id, msg.trajectory_id) in assembly.closed


@pytest.mark.parametrize('linear', [False, True])
def test_segment_assembly_rejects_mixed_linear_modes(linear):
    assembly = wc.TrajectoryAssembly()
    assert assembly.add(segment(0, 2), NOW, 1., linear=linear) is None
    last = segment(1, 2, times=(2000,))
    with pytest.raises(ValueError, match='不一致'):
        assembly.add(last, NOW, 1.1, linear=not linear)
    assert assembly.pending is None
    with pytest.raises(ValueError):
        assembly.add(last, NOW, 1.1, linear=linear)


@pytest.mark.parametrize('change', ['order', 'duplicate', 'metadata', 'target_ids', 'stale',
                                  'expired', 'missing_motor', 'time', 'nonfinite'])
def test_segment_assembly_discards_invalid_group(files, change):
    assembly = wc.TrajectoryAssembly()
    first = segment(0, 2)
    assert assembly.add(first, NOW, 1.) is None
    second = segment(1, 2, times=(2000,))
    if change == 'order':
        second.segment_index = 0; second.trajectory_id = 42
    elif change == 'duplicate':
        second = first
    elif change == 'metadata':
        second.segment_count = 3
    elif change == 'target_ids':
        second.target_ids = [9, 8]
    elif change == 'stale':
        second.sync_time_ns -= 1_000_000_000
    elif change == 'missing_motor':
        second.points.pop(); second.count = 1
    elif change == 'time':
        second.points[1].time_from_start_ms -= 1
    elif change == 'nonfinite':
        second.points[0].effort = math.inf
    with pytest.raises(ValueError):
        assembly.add(second, NOW, 4. if change == 'expired' else 1.1)
    with pytest.raises(ValueError):
        assembly.add(segment(1, 2, times=(2000,)), NOW, 4.1)


def test_invalid_urdf_or_mapping_fails_closed(files):
    driver, urdf = files
    text = Path(urdf).read_text()
    for bad in (text.replace('effort="10"', 'effort="nan"'),
                text.replace('name="yaw"', 'name="missing"')):
        Path(urdf).write_text(bad)
        with pytest.raises(ValueError):
            wc.WorkbenchModel(driver, urdf)
    Path(urdf).write_text(text)
    Path(driver).write_text(Path(driver).read_text().replace('id: 9', 'id: 8'))
    with pytest.raises(ValueError):
        wc.WorkbenchModel(driver, urdf)


def wait_for(predicate, seconds=4.):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(.01)
    return bool(predicate())


def cancel_race():
    cancel, result = Future(), Future()
    run = wc.Run(MotorTrajectoryState(active=True), 1,
                 handle=SimpleNamespace(status=GoalStatus.STATUS_EXECUTING,
                                        cancel_goal_async=lambda: cancel))
    run.result_future = result
    errors, continued = [], []
    gateway = SimpleNamespace(_epoch=2, _runs=[run], _waits=[], _stopping=False,
                              _assembly=wc.TrajectoryAssembly(), _locked=lambda fn: fn,
                              _fail=errors.append, _stop_error=errors.append)
    for method in ('_watch', '_cancel_one', '_cancel_handles', '_tick', '_goal_error'):
        setattr(gateway, method, MethodType(getattr(wc.WorkbenchControl, method), gateway))
    gateway._cancel_handles(2, lambda: continued.append(True))
    return gateway, cancel, result, errors, continued


@pytest.mark.parametrize('return_code', [CancelGoal.Response.ERROR_NONE,
    CancelGoal.Response.ERROR_REJECTED, CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID,
    CancelGoal.Response.ERROR_GOAL_TERMINATED])
def test_cancel_empty_ack_waits_for_terminal_result_with_stale_local_status(return_code):
    gateway, cancel, result, errors, continued = cancel_race()
    cancel.set_result(CancelGoal.Response(return_code=return_code))
    assert not errors and not continued
    result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    assert continued == [True] and not errors
    assert gateway._runs[0].handle.status == GoalStatus.STATUS_EXECUTING


def test_cancel_without_ack_or_terminal_result_still_times_out(monkeypatch):
    monkeypatch.setattr(wc, 'RPC_TIMEOUT', .1)
    monkeypatch.setattr(wc.time, 'monotonic', lambda: 1.)
    gateway, cancel, result, errors, continued = cancel_race()
    cancel.set_result(CancelGoal.Response(return_code=CancelGoal.Response.ERROR_REJECTED))
    assert not errors and not continued
    monkeypatch.setattr(wc.time, 'monotonic', lambda: 1.2)
    gateway._tick()
    assert len(errors) == 1 and '超时' in errors[0]
    result.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    assert not continued


def test_cancel_late_terminal_cannot_continue_superseded_epoch():
    gateway, cancel, result, errors, continued = cancel_race()
    cancel.set_result(CancelGoal.Response(return_code=CancelGoal.Response.ERROR_GOAL_TERMINATED))
    assert not errors
    gateway._epoch += 1
    result.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    assert not continued and not errors


@pytest.mark.parametrize('status', [None, GoalStatus.STATUS_EXECUTING])
def test_cancel_empty_ack_requires_a_valid_terminal_result(status):
    _, cancel, result, errors, continued = cancel_race()
    cancel.set_result(CancelGoal.Response(return_code=CancelGoal.Response.ERROR_REJECTED))
    result.set_result(None if status is None else SimpleNamespace(status=status))
    assert not continued and len(errors) == 1 and '终态未确认' in errors[0]


@pytest.mark.parametrize('phase', ['rpc', 'terminal'])
@pytest.mark.parametrize('stopping', [False, True])
def test_cancel_timeout_after_old_result_still_fails_current_transition(monkeypatch, phase, stopping):
    monkeypatch.setattr(wc, 'RPC_TIMEOUT', .1)
    monkeypatch.setattr(wc.time, 'monotonic', lambda: 1.)
    gateway, cancel, _, errors, continued = cancel_race()
    gateway._stopping, gateway._stop_deadline, gateway._mode = stopping, 100., 'switching'
    gateway._stop_error = lambda reason: errors.append('stop:' + reason)
    gateway._fail = lambda reason: errors.append('fail:' + reason)
    if phase == 'terminal':
        cancel.set_result(CancelGoal.Response(return_code=CancelGoal.Response.ERROR_GOAL_TERMINATED))
    # 原结果回调先清理旧 run，取消等待回调尚未得到执行机会。
    gateway._runs[:] = [wc.Run(MotorTrajectoryState(), gateway._epoch)]
    monkeypatch.setattr(wc.time, 'monotonic', lambda: 1.2)
    gateway._tick()
    assert len(errors) == 1 and errors[0].startswith('stop:' if stopping else 'fail:')
    assert not continued


class FakeControl(Node):
    def __init__(self, context):
        super().__init__('test_workbench_peer', context=context)
        self.names = [f'manual_controller_{i}' for i in range(3)]
        self.states = {name: 'inactive' for name in self.names +
                       ['trajectory_controller', 'gravity_controller', 'follow_controller']}
        self.switches, self.commands, self.goals = [], [[], [], []], []
        self.follow_targets = []
        self.target_pub = self.create_publisher(JointState, '/control/joint_target', 1)
        self.create_subscription(JointState, '/host_control/follow_controller/target', self.follow_targets.append, 1)
        self.canceled_goals = []
        self.canceling_goals = []
        self.goal_gate = threading.Event()
        self.goal_gate.set()
        self.cancel_terminal_gate = threading.Event()
        self.cancel_terminal_gate.set()
        self.status, self.receipts, self.joints, self.transmissions = [], [], [], []
        self.policies = []
        self.hardware_stops = []
        self.switch_delay = 0.
        self.switch_ok = True
        self.goal_delay = 0.
        self.goal_accept = True
        self.outcome = None
        self.bad_interfaces = False
        group = ReentrantCallbackGroup()
        self.create_service(ListControllers, '/host_control/controller_manager/list_controllers',
                            self.list_controllers, callback_group=group)
        self.create_service(SwitchController, '/host_control/controller_manager/switch_controller',
                            self.switch, callback_group=group)
        self.create_service(GetParameters, '/host_control/gravity_controller/get_parameters',
                            self.parameters, callback_group=group)
        self.server = ActionServer(self, FollowJointTrajectory,
            '/host_control/trajectory_controller/follow_joint_trajectory', self.execute,
            goal_callback=self.goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group)
        self.state_pub = self.create_publisher(JointState, '/host_control/joint_states', 10)
        self.setpoint_pub = self.create_publisher(MotorCommandArray, '/control/setpoint', 10)
        self.trajectory_pub = self.create_publisher(MotorTrajectory, '/control/trajectory', 10)
        self.joint_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.stop_pub = self.create_publisher(Empty, '/control/stop', 10)
        self.count_pub = self.create_publisher(Int32, '/client_count',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        for i, name in enumerate(self.names):
            self.create_subscription(Float64MultiArray, f'/host_control/{name}/commands',
                                     lambda msg, i=i: self.commands[i].append(msg), 10)
        self.create_subscription(String, '/control/status',
                                 lambda msg: self.status.append(json.loads(msg.data)), 10)
        self.create_subscription(String, '/robot_transmission/status',
                                 lambda msg: self.transmissions.append(json.loads(msg.data)), 10)
        self.create_subscription(MotorTrajectoryState, '/control/trajectory_state',
                                 self.receipts.append, 10)
        self.create_subscription(JointState, '/joint_states', self.joints.append, 10)
        def policy(message):
            self.policies.append(message)
        self.create_subscription(MotorPolicyArray, '/motor/policy_array', policy, 10)
        self.create_subscription(Empty, '/host_control/stop', self.hardware_stops.append, 10)
        self.publish_feedback = True
        self.limit_requests = []
        self.limits_ok = True
        self.limits_delay = 0.
        self.create_service(SetParametersAtomically,
            '/micro_ros_system_micro_ros_system/set_parameters_atomically',
            self.set_hardware_limits, callback_group=group)
        self.create_timer(.02, self.publish_state)

    def set_hardware_limits(self, request, response):
        self.limit_requests.append(copy.deepcopy(request))
        time.sleep(self.limits_delay)
        response.result = SetParametersResult(successful=self.limits_ok,
            reason='' if self.limits_ok else '硬件限位更新被拒绝')
        return response

    def publish_state(self):
        if self.publish_feedback:
            self.state_pub.publish(feedback(self.get_clock().now().nanoseconds))

    def list_controllers(self, _, response):
        joints = ['yaw', 'pitch', 'elbow']
        for name, state in self.states.copy().items():
            if name in self.names:
                required = [joints[self.names.index(name)] + '/' + x
                            for x in ('position', 'velocity', 'effort')]
            else:
                required = [j + '/' + x for j in joints for x in
                            (('effort',) if name == 'gravity_controller' else
                             ('position', 'velocity', 'effort') if name == 'follow_controller' else ('position', 'velocity'))]
            if self.bad_interfaces:
                required.reverse()
            response.controller.append(ControllerState(name=name, state=state,
                required_command_interfaces=required,
                claimed_interfaces=required if state == 'active' else []))
        return response

    def switch(self, request, response):
        self.switches.append(copy.deepcopy(request))
        delay, self.switch_delay = self.switch_delay, 0.
        time.sleep(delay)
        response.ok = self.switch_ok or not request.activate_controllers
        if response.ok:
            for name in request.deactivate_controllers:
                self.states[name] = 'inactive'
            for name in request.activate_controllers:
                self.states[name] = 'active'
        return response

    def parameters(self, _, response):
        response.values = [ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=False),
                           ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=.75)]
        return response

    def goal(self, request):
        self.goals.append(copy.deepcopy(request))
        time.sleep(self.goal_delay)
        self.goal_gate.wait(timeout=4.)
        return GoalResponse.ACCEPT if self.goal_accept else GoalResponse.REJECT

    def execute(self, handle):
        while rclpy.ok(context=self.context):
            if handle.is_cancel_requested:
                if not self.cancel_terminal_gate.is_set():
                    self.canceling_goals.append(handle.goal_id)
                    time.sleep(.02)
                    continue
                self.canceled_goals.append(handle.goal_id)
                handle.canceled()
                return FollowJointTrajectory.Result()
            if self.outcome:
                if self.outcome == 'succeeded':
                    handle.succeed()
                    return FollowJointTrajectory.Result(error_code=0)
                handle.abort()
                return FollowJointTrajectory.Result(error_code=-4, error_string='path tolerance')
            handle.publish_feedback(FollowJointTrajectory.Feedback(
                desired=JointTrajectoryPoint(time_from_start=wc.Duration(nanosec=100_000_000))))
            time.sleep(.02)
        return FollowJointTrajectory.Result()


@pytest.fixture
def graph(files, monkeypatch, request):
    monkeypatch.setenv('ROS_DOMAIN_ID', '183')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setattr(wc, 'RPC_TIMEOUT', .4)
    context = Context()
    rclpy.init(context=context)
    peer = FakeControl(context)
    settings = getattr(request, 'param', {})
    if 'client_count' in settings:
        peer.count_pub.publish(Int32(data=settings['client_count']))
    gateway = wc.WorkbenchControl(context=context, parameter_overrides=[
        Parameter('config_file', value=files[0]), Parameter('urdf_file', value=files[1]),
        Parameter('model_name', value='test'),
        Parameter('joint_limits_json', value=settings.get('joint_limits_json', '')),
        Parameter('publish_joint_states', value=settings.get('publish_joint_states', True))])
    executor = MultiThreadedExecutor(num_threads=6, context=context)
    executor.add_node(peer); executor.add_node(gateway)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    assert wait_for(lambda: peer.setpoint_pub.get_subscription_count() == 1
                    and peer.state_pub.get_subscription_count() == 1)
    if settings.get('publish_joint_states', True):
        assert wait_for(lambda: peer.joints)
    else:
        assert wait_for(lambda: peer.status and peer.status[-1]['feedback_fresh'])
    try:
        yield peer, gateway
    finally:
        peer.goal_gate.set()
        peer.cancel_terminal_gate.set()
        peer.outcome = 'aborted'
        executor.shutdown(timeout_sec=3.)
        thread.join(timeout=3.)
        peer.server.destroy()
        gateway._action.destroy()
        gateway.destroy_node(); peer.destroy_node(); context.shutdown()


def send_manual(peer):
    peer.setpoint_pub.publish(setpoint(peer.get_clock().now().nanoseconds))


def send_trajectory(peer, trajectory_id=41):
    msg = segment(trajectory_id=trajectory_id)
    msg.sync_time_ns = peer.get_clock().now().nanoseconds
    peer.trajectory_pub.publish(msg)


def stopped(peer):
    return all(state == 'inactive' for state in peer.states.values())


def test_ros_manual_switch_is_atomic_and_commands_are_one_shot(graph):
    peer, gateway = graph
    send_manual(peer)
    assert wait_for(lambda: all(peer.commands))
    switch = next(r for r in peer.switches if r.activate_controllers)
    assert switch.strictness == SwitchController.Request.STRICT
    assert switch.activate_controllers == peer.names
    assert not set(peer.names).intersection(switch.deactivate_controllers)
    assert [list(messages[-1].data) for messages in peer.commands] == [
        pytest.approx([.3, .1, 2.]), pytest.approx([.1, .1, 0.]), pytest.approx([.3, 0., 0.])]
    time.sleep(.15)
    assert [len(messages) for messages in peer.commands] == [1, 1, 1]
    assert wait_for(lambda: peer.status and peer.status[-1]['mode'] == 'manual')
    assert all(p.topic_name != '/motor/host_command_array' for p in gateway.publishers)


def test_ros_manual_updates_same_active_mode_without_switching(graph):
    peer, _ = graph
    send_manual(peer)
    assert wait_for(lambda: all(peer.commands))
    switches = len(peer.switches)
    send_manual(peer)
    assert wait_for(lambda: all(len(messages) == 2 for messages in peer.commands))
    assert len(peer.switches) == switches


@pytest.mark.parametrize('outcome', ['succeeded', 'aborted', 'rejected', 'canceled'])
def test_ros_action_receipts_come_from_jtc(graph, outcome):
    peer, _ = graph
    peer.goal_accept = outcome != 'rejected'
    send_trajectory(peer)
    assert wait_for(lambda: peer.goals)
    assert not peer.goals[0].path_tolerance
    assert not peer.goals[0].goal_tolerance
    assert wc.stamp_ns(peer.goals[0].goal_time_tolerance) == 0
    if outcome == 'rejected':
        assert wait_for(lambda: peer.status and 'reject' in peer.status[-1]
                        and peer.status[-1]['reject'])
    else:
        assert wait_for(lambda: any(s.active for s in peer.receipts))
        assert not any(s.completed for s in peer.receipts)
        if outcome == 'canceled':
            peer.joint_pub.publish(JointTrajectory())
        else:
            peer.outcome = outcome
    assert wait_for(lambda: any(s.trajectory_id == 41 and not s.active for s in peer.receipts))
    receipt = [s for s in peer.receipts if s.trajectory_id == 41][-1]
    assert receipt.completed == (outcome == 'succeeded')
    assert list(receipt.target_ids) == [8, 9]
    if outcome in ('aborted', 'rejected'):
        assert wait_for(lambda: stopped(peer))
        assert peer.status[-1]['reject']


def test_ros_stop_during_switch_cannot_publish_or_restore_target(graph):
    peer, _ = graph
    peer.switch_delay = .2
    send_manual(peer)
    assert wait_for(lambda: any(r.activate_controllers for r in peer.switches))
    peer.stop_pub.publish(Empty())
    assert wait_for(lambda: any(not r.activate_controllers for r in peer.switches))
    time.sleep(.4)
    assert wait_for(lambda: stopped(peer))
    assert not any(peer.commands)


def test_ros_stop_during_goal_acceptance_cancels_late_goal(graph):
    peer, _ = graph
    peer.goal_delay = .2
    send_trajectory(peer)
    assert wait_for(lambda: peer.goals)
    peer.stop_pub.publish(Empty())
    assert wait_for(lambda: stopped(peer))
    assert wait_for(lambda: peer.canceled_goals)
    assert wait_for(lambda: any(not s.active and s.trajectory_id == 41 for s in peer.receipts))
    time.sleep(.25)
    assert not peer.receipts[-1].completed
    assert stopped(peer)


def test_ros_supersede_waits_for_terminal_when_cancel_ack_is_empty(graph, monkeypatch):
    peer, gateway = graph
    peer.cancel_terminal_gate.clear()
    send_trajectory(peer)
    assert wait_for(lambda: any(state.active for state in peer.receipts))
    with gateway._lock:
        handle = gateway._runs[0].handle
        original = handle.cancel_goal_async
        def empty_ack():
            response = Future()
            original().add_done_callback(lambda _: response.set_result(CancelGoal.Response(
                return_code=CancelGoal.Response.ERROR_GOAL_TERMINATED)))
            return response
        monkeypatch.setattr(handle, 'cancel_goal_async', empty_ack)
    send_trajectory(peer, trajectory_id=42)
    assert wait_for(lambda: peer.canceling_goals)
    time.sleep(.05)
    assert len(peer.goals) == 1 and not peer.hardware_stops
    peer.cancel_terminal_gate.set()
    assert wait_for(lambda: len(peer.goals) == 2)
    assert wait_for(lambda: any(state.trajectory_id == 42 and state.active for state in peer.receipts))
    assert not peer.hardware_stops and not gateway._stopping
    assert gateway._runs[0].state.trajectory_id == 42


def test_ros_stop_barrier_waits_for_pending_goal_and_cancel_terminal(graph):
    peer, _ = graph
    peer.goal_gate.clear()
    peer.cancel_terminal_gate.clear()
    send_trajectory(peer)
    assert wait_for(lambda: peer.goals)
    peer.stop_pub.publish(Empty())
    assert wait_for(lambda: peer.hardware_stops and stopped(peer))
    assert wait_for(lambda: peer.status[-1]['active_controllers'] == [])
    assert peer.status[-1]['mode'] in ('stopping', 'error')
    peer.count_pub.publish(Int32(data=1))
    send_manual(peer)
    assert wait_for(lambda: peer.status[-1]['reject'])
    assert not any(peer.commands) and stopped(peer)
    assert wait_for(lambda: peer.status[-1]['mode'] == 'error')
    assert '超时' in peer.status[-1]['reason']
    assert not any(s.trajectory_id == 41 and not s.active for s in peer.receipts)

    peer.goal_gate.set()
    assert wait_for(lambda: peer.canceling_goals)
    send_manual(peer)
    time.sleep(.1)
    assert peer.status[-1]['mode'] in ('stopping', 'error')
    assert not peer.canceled_goals and not any(peer.commands) and stopped(peer)
    assert not any(s.trajectory_id == 41 and not s.active for s in peer.receipts)

    peer.cancel_terminal_gate.set()
    assert wait_for(lambda: peer.canceled_goals)
    assert wait_for(lambda: peer.status[-1]['mode'] == 'stopped')
    assert wait_for(lambda: any(s.trajectory_id == 41 and not s.active for s in peer.receipts))
    assert not any(peer.commands) and len(peer.goals) == 1
    send_manual(peer)
    assert wait_for(lambda: all(peer.commands))


@pytest.mark.parametrize('failure', ['switch', 'timeout', 'mapping', 'feedback'])
def test_ros_failed_transition_stops_all_motion_controllers(graph, failure):
    peer, _ = graph
    if failure == 'switch':
        peer.switch_ok = False
    elif failure == 'timeout':
        peer.switch_delay = .65
    elif failure == 'mapping':
        peer.bad_interfaces = True
    else:
        peer.publish_feedback = False
        time.sleep(.6)
    send_manual(peer)
    assert wait_for(lambda: peer.status and peer.status[-1]['reject'])
    time.sleep(.7 if failure == 'timeout' else .1)
    assert wait_for(lambda: stopped(peer))
    assert not any(peer.commands)
    if failure != 'feedback':
        assert peer.hardware_stops
        assert not peer.policies


def test_ros_client_zero_is_latched_and_reconnect_does_not_replay(graph):
    peer, _ = graph
    send_trajectory(peer)
    assert wait_for(lambda: any(s.active for s in peer.receipts))
    peer.count_pub.publish(Int32(data=0))
    assert wait_for(lambda: stopped(peer))
    count = len(peer.switches)
    send_manual(peer)
    assert wait_for(lambda: peer.status[-1]['reject'])
    peer.count_pub.publish(Int32(data=1))
    time.sleep(.15)
    assert len(peer.switches) == count and not any(peer.commands)
    send_manual(peer)
    assert wait_for(lambda: all(peer.commands))


def test_ros_feedback_forwarding_qos_and_truthful_gravity_status(graph):
    peer, gateway = graph
    assert wait_for(lambda: peer.status and peer.status[-1]['effort_gain'] == .75, seconds=3.)
    assert peer.status[-1]['gravity_enabled'] is False
    assert peer.status[-1]['mode'] == 'stopped'
    assert peer.joints[-1].name == ['yaw', 'pitch', 'elbow']
    assert wait_for(lambda: peer.transmissions)
    assert peer.transmissions[-1]['model'] == 'test'
    count_sub = next(sub for sub in gateway.subscriptions if sub.topic_name == '/client_count')
    assert count_sub.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL


@pytest.mark.parametrize('graph', [{'publish_joint_states': False}], indirect=True)
def test_ros_feedback_forwarding_can_be_disabled(graph):
    peer, gateway = graph
    assert not any(pub.topic_name == '/joint_states' for pub in gateway.publishers)
    assert not peer.joints
    send_manual(peer)
    assert wait_for(lambda: all(peer.commands))


@pytest.mark.parametrize('graph', [{'client_count': 0}], indirect=True)
def test_ros_latched_client_zero_blocks_first_target(graph):
    peer, _ = graph
    assert wait_for(lambda: peer.switches)
    send_manual(peer)
    assert wait_for(lambda: peer.status and peer.status[-1]['reject'])
    assert not any(request.activate_controllers for request in peer.switches)


def test_ros_segments_commit_only_once_and_stop_discards_partial_group(graph):
    peer, _ = graph
    start = peer.get_clock().now().nanoseconds
    first, last = segment(0, 2), segment(1, 2, times=(2000,))
    first.sync_time_ns = last.sync_time_ns = start
    peer.trajectory_pub.publish(first)
    assert wait_for(lambda: peer.status[-1]['reason'] == '轨迹分段收集中')
    assert not peer.goals and not peer.switches
    peer.trajectory_pub.publish(last)
    assert wait_for(lambda: len(peer.goals) == 1)
    assert len(peer.goals[0].trajectory.points) == 2
    assert not peer.goals[0].trajectory.points[0].effort
    assert wait_for(lambda: any(state.active for state in peer.receipts))
    second = segment(0, 2, trajectory_id=42)
    second.sync_time_ns = peer.get_clock().now().nanoseconds
    peer.trajectory_pub.publish(second)
    assert wait_for(lambda: peer.status[-1]['reason'] == '轨迹分段收集中')
    peer.stop_pub.publish(Empty())
    assert wait_for(lambda: stopped(peer))
    assert wait_for(lambda: any(s.trajectory_id == 42 and not s.active for s in peer.receipts))
    second.segment_index = 1
    peer.trajectory_pub.publish(second)
    assert wait_for(lambda: peer.status[-1]['reject'])
    assert len(peer.goals) == 1


def test_ros_segment_burst_preserves_every_point_in_one_goal(graph):
    peer, gateway = graph
    publisher = peer.create_publisher(MotorTrajectory, '/control/trajectory', 64)
    assert wait_for(lambda: publisher.get_subscription_count() == 1)
    messages = [segment(index, 12, times=range(1000 + index * 1280,
                                              1000 + (index + 1) * 1280, 10))
                for index in range(12)]
    start = peer.get_clock().now().nanoseconds
    for message in messages:
        message.sync_time_ns = start
    with gateway._lock:
        for message in messages:
            publisher.publish(message)
        time.sleep(.05)
    assert wait_for(lambda: len(peer.goals) == 1)
    goal = peer.goals[0].trajectory
    assert len(goal.points) == 1536
    assert [wc.stamp_ns(point.time_from_start) // 10**6 for point in goal.points] == [
        t for message in messages for t in sorted({point.time_from_start_ms for point in message.points})]
    assert all(list(point.positions) == pytest.approx([.2, 0., .3]) for point in goal.points)
    assert wait_for(lambda: any(receipt.active for receipt in peer.receipts))
    assert len(peer.goals) == 1


def test_ros_raw_segments_make_one_linear_goal_and_normal_cancel_stops_it(graph):
    peer, gateway = graph
    publisher = peer.create_publisher(MotorTrajectory, '/control/trajectory_raw', 64)
    assert wait_for(lambda: publisher.get_subscription_count() == 1)
    points = [MotorTrajectoryPoint(motor_id=motor, time_from_start_ms=2500 + index * 42,
                                   position=.2, velocity=0.)
              for index in range(300) for motor in (8, 9)]
    start = peer.get_clock().now().nanoseconds
    with gateway._lock:
        for index in range(3):
            chunk = points[index*256:(index+1)*256]
            publisher.publish(MotorTrajectory(sync_time_ns=start, session_id=3, trajectory_id=41,
                segment_index=index, segment_count=3, count=len(chunk), target_count=2,
                target_ids=[8, 9], points=chunk))
    assert wait_for(lambda: len(peer.goals) == 1)
    goal = peer.goals[0].trajectory
    assert [(t.name, t.position, t.velocity, t.acceleration)
            for t in peer.goals[0].path_tolerance] == [
        (name, .2, 0., 0.) for name in goal.joint_names]
    assert not peer.goals[0].goal_tolerance
    assert wc.stamp_ns(peer.goals[0].goal_time_tolerance) == 0
    assert len(goal.points) == 300
    assert [wc.stamp_ns(p.time_from_start) // 10**6 for p in goal.points] == [
        2500 + index * 42 for index in range(300)]
    assert all(not p.velocities for p in goal.points)
    assert wait_for(lambda: any(s.active and s.trajectory_id == 41 for s in peer.receipts))
    peer.trajectory_pub.publish(MotorTrajectory())
    assert wait_for(lambda: peer.canceled_goals)
    assert wait_for(lambda: any(not s.active and s.trajectory_id == 41 for s in peer.receipts))
    assert len(peer.goals) == 1
    partial = segment(0, 2, trajectory_id=42)
    partial.sync_time_ns = peer.get_clock().now().nanoseconds
    publisher.publish(partial)
    assert wait_for(lambda: gateway._assembly.pending is not None)
    peer.stop_pub.publish(Empty())
    assert wait_for(lambda: gateway._assembly.pending is None and stopped(peer))
    partial.segment_index = 1
    publisher.publish(partial)
    assert wait_for(lambda: peer.status[-1]['reject'])
    assert len(peer.goals) == 1


def test_ros_native_joint_trajectory_keeps_controller_default_tolerances(graph):
    peer, _ = graph
    message = trajectory()
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(
        peer.get_clock().now().nanoseconds, 10**9)
    peer.joint_pub.publish(message)
    assert wait_for(lambda: len(peer.goals) == 1)
    assert not peer.goals[0].path_tolerance
    assert not peer.goals[0].goal_tolerance
    assert wc.stamp_ns(peer.goals[0].goal_time_tolerance) == 0


def test_ros_normal_and_raw_segments_cannot_commit_a_mixed_goal(graph):
    peer, gateway = graph
    publisher = peer.create_publisher(MotorTrajectory, '/control/trajectory_raw', 64)
    assert wait_for(lambda: publisher.get_subscription_count() == 1)
    first, last = segment(0, 2), segment(1, 2, times=(2000,))
    first.sync_time_ns = last.sync_time_ns = peer.get_clock().now().nanoseconds
    peer.trajectory_pub.publish(first)
    assert wait_for(lambda: gateway._assembly.pending is not None)
    publisher.publish(last)
    assert wait_for(lambda: peer.status[-1]['reject'])
    assert gateway._assembly.pending is None
    assert not peer.goals and not any(r.activate_controllers for r in peer.switches)


def test_ros_unreachable_switch_service_rejects_without_any_command(graph):
    peer, gateway = graph
    service = next(service for service in peer.services
                   if service.srv_name == '/host_control/controller_manager/switch_controller')
    peer.destroy_service(service)
    probe = peer.create_client(SwitchController, service.srv_name)
    assert wait_for(lambda: not probe.service_is_ready())
    send_trajectory(peer)
    assert wait_for(lambda: peer.status[-1]['reject'])
    assert wait_for(lambda: any(s.trajectory_id == 41 and not s.active for s in peer.receipts))
    assert not peer.goals and not any(peer.commands)


def test_ros_stop_closes_hardware_gate_when_switch_service_is_unavailable(graph):
    peer, _ = graph
    send_manual(peer)
    assert wait_for(lambda: all(peer.commands))
    service = next(service for service in peer.services
                   if service.srv_name == '/host_control/controller_manager/switch_controller')
    peer.destroy_service(service)
    probe = peer.create_client(SwitchController, service.srv_name)
    assert wait_for(lambda: not probe.service_is_ready())
    peer.stop_pub.publish(Empty())
    assert wait_for(lambda: peer.hardware_stops)
    assert wait_for(lambda: peer.status[-1]['mode'] == 'error')
    assert '显式重新使能' in peer.status[-1]['safety_note']
    send_manual(peer)
    time.sleep(.1)
    assert [len(commands) for commands in peer.commands] == [1, 1, 1]
    assert not peer.policies


def test_ros_normal_stop_closes_hardware_gate_and_deactivates(graph):
    peer, gateway = graph
    send_manual(peer)
    assert wait_for(lambda: all(peer.commands))
    peer.stop_pub.publish(Empty())
    assert wait_for(lambda: peer.hardware_stops)
    assert wait_for(lambda: peer.status[-1]['mode'] == 'stopped')
    # MCU 租约撤销由监督器负责，本节点只关闭主机硬件门和控制器。
    assert not peer.policies
    assert all(pub.topic_name != '/motor/policy_array' for pub in gateway.publishers)
    assert all(sub.topic_name != '/host_control/stop' for sub in gateway.subscriptions)


def test_ros_joint_limit_parameters_apply_runtime_overrides_and_block_active_updates(graph):
    peer, gateway = graph
    def submit(lower, upper):
        return gateway.set_parameters_atomically([Parameter('joint_limits_json', value=json.dumps({
            'model': 'test', 'joint_limits': {'yaw': {'lower': lower, 'upper': upper}}}))])
    assert submit(-.5, .5).successful
    assert wait_for(lambda: peer.transmissions and peer.transmissions[-1].get('joint_limits', {}).get('yaw') ==
                    {'lower': -.5, 'upper': .5})
    assert peer.transmissions[-1]['control_service'] == '/workbench_control/set_parameters_atomically'
    assert submit(-2., 2.).successful
    assert wait_for(lambda: peer.transmissions[-1]['joint_limits']['yaw'] == {'lower': -2., 'upper': 2.})
    sent = {p.name: p.value for p in peer.limit_requests[-1].parameters}
    assert list(sent['position_limit_names'].string_array_value) == ['yaw', 'pitch', 'elbow']
    assert list(sent['position_limit_bounds'].double_array_value) == [-2., 2., -1., 1., -1., 1.]
    with gateway._lock:
        plan = gateway.mapping.joint_trajectory(trajectory(1.5), gateway._measured(),
                                               gateway.get_clock().now().nanoseconds)
        assert plan.points[-1].positions[0] == 1.5
    before = len(peer.limit_requests)
    assert not submit(float('nan'), 2.).successful
    assert len(peer.limit_requests) == before
    assert submit(-.5, .5).successful
    with gateway._lock:
        gateway._apply_joint_limits()
        with pytest.raises(ValueError, match='限位'):
            gateway.mapping.joint_trajectory(trajectory(.6), gateway._measured(),
                                            gateway.get_clock().now().nanoseconds)
        gateway._busy = True
        assert not submit(-.4, .4).successful
        gateway._busy = False
        gateway._active = ['trajectory_controller']
        assert not submit(-.4, .4).successful
        gateway._active = []
    assert gateway.mapping.limits['yaw'] == (-.5, .5, 2., 10.)


def test_rejected_hardware_limit_update_keeps_previous_host_limits(graph):
    peer, gateway = graph
    peer.limits_ok = False
    response = gateway.set_parameters_atomically([Parameter('joint_limits_json', value=json.dumps({
        'model': 'test', 'joint_limits': {'yaw': {'lower': -2., 'upper': 2.}}}))])
    assert not response.successful and '硬件限位' in response.reason
    assert gateway.get_parameter('joint_limits_json').value == ''
    assert gateway.mapping.limits['yaw'] == (-1., 1., 2., 10.)
    assert not any(peer.commands) and not peer.goals


def test_ros_limit_service_waits_for_hardware_ack_without_executor_deadlock(graph):
    peer, gateway = graph
    client = peer.create_client(SetParametersAtomically, '/workbench_control/set_parameters_atomically')
    assert client.wait_for_service(timeout_sec=1.)
    future = client.call_async(SetParametersAtomically.Request(parameters=[
        Parameter('joint_limits_json', value=json.dumps({'model': 'test',
            'joint_limits': {'yaw': {'lower': -2., 'upper': 2.}}})).to_parameter_msg()]))
    assert wait_for(future.done)
    assert future.result().result.successful
    assert len(peer.limit_requests) == 1
    assert wait_for(lambda: peer.transmissions[-1]['joint_limits']['yaw'] == {'lower': -2., 'upper': 2.})
    assert not any(peer.commands) and not peer.goals


def test_hardware_limit_timeout_blocks_control_until_confirmed_retry(graph):
    peer, gateway = graph
    peer.limits_delay = .65
    parameter = Parameter('joint_limits_json', value=json.dumps({
        'model': 'test', 'joint_limits': {'yaw': {'lower': -2., 'upper': 2.}}}))
    response = gateway.set_parameters_atomically([parameter])
    assert not response.successful and '未确认' in response.reason
    with gateway._lock:
        with pytest.raises(ValueError, match='限位更新未确认'):
            gateway._measured()
    peer.limits_delay = 0.
    assert gateway.set_parameters_atomically([parameter]).successful
    with gateway._lock:
        assert gateway._measured()
    assert not any(peer.commands) and not peer.goals


@pytest.mark.parametrize('graph', [{'joint_limits_json': json.dumps({
    'model': 'test', 'joint_limits': {'yaw': {'lower': -2., 'upper': 2.}}})}], indirect=True)
def test_startup_limit_override_requires_explicit_hardware_confirmation(graph):
    peer, gateway = graph
    assert not peer.limit_requests
    with gateway._lock, pytest.raises(ValueError, match='限位更新未确认'):
        gateway._measured()
    value = gateway.get_parameter('joint_limits_json').value
    assert gateway.set_parameters_atomically([Parameter('joint_limits_json', value=value)]).successful
    with gateway._lock:
        assert gateway._measured()
    assert not any(peer.commands) and not peer.goals


def test_stop_rejects_pre_stop_inputs_after_stop_completes(graph):
    peer, gateway = graph
    old=setpoint(peer.get_clock().now().nanoseconds)
    plan=segment();plan.sync_time_ns=old.sync_time_ns
    joint=trajectory();joint.header.stamp=peer.get_clock().now().to_msg()
    peer.stop_pub.publish(Empty())
    assert wait_for(lambda:peer.hardware_stops and peer.status[-1]['mode']=='stopped')
    switches=len(peer.switches)
    peer.setpoint_pub.publish(old);peer.trajectory_pub.publish(plan);peer.joint_pub.publish(joint)
    peer.joint_pub.publish(trajectory())
    time.sleep(.1)
    assert len(peer.switches)==switches
    assert stopped(peer)
    send_manual(peer)
    assert wait_for(lambda:peer.status[-1]['mode']=='manual')


def joint_target(stamp=NOW, name='yaw', position=.4):
    result = JointState(name=[name], position=[position])
    result.header.stamp.sec, result.header.stamp.nanosec = divmod(stamp, 10**9)
    return result


@pytest.mark.parametrize('invalid', ['duplicate', 'unknown', 'length', 'nan', 'limit', 'stale', 'velocity'])
def test_joint_target_rejects_invalid_input(files, invalid):
    model = wc.WorkbenchModel(*files)
    message = joint_target()
    if invalid == 'duplicate': message.name = ['yaw', 'yaw']; message.position = [.1, .2]
    if invalid == 'unknown': message.name = ['missing']
    if invalid == 'length': message.position = []
    if invalid == 'nan': message.position = [math.nan]
    if invalid == 'limit': message.position = [2.]
    if invalid == 'stale': message.header.stamp.sec -= 1
    if invalid == 'velocity': message.velocity = [1.]
    with pytest.raises(ValueError):
        model.joint_target(message, model.feedback(feedback(), NOW), NOW)


def test_jtc_admission_time_does_not_shorten_validated_trajectory(files):
    model = wc.WorkbenchModel(*files)
    message = trajectory()
    message.header.stamp.sec = 10
    output = model.joint_trajectory(message, model.feedback(feedback(), NOW), NOW + 200_000_000)
    assert wc.stamp_ns(output.header.stamp) == 0
    assert wc.stamp_ns(message.header.stamp) == NOW


def send_follow(peer, value=.4, name='yaw'):
    message = joint_target(peer.get_clock().now().nanoseconds, name, value)
    peer.target_pub.publish(message)
    return message


def test_ros_follow_coalesces_switch_and_then_updates_without_jtc(graph):
    peer, gateway = graph
    peer.switch_delay = .18
    send_follow(peer)
    assert wait_for(lambda: any(r.activate_controllers for r in peer.switches))
    send_follow(peer, .6)
    send_follow(peer, -.2)
    assert wait_for(lambda: peer.follow_targets)
    assert peer.follow_targets[-1].position == pytest.approx([-.2, .1, .3])
    assert peer.states['follow_controller'] == 'active'
    assert not peer.goals
    send_follow(peer, .5, 'elbow')
    assert wait_for(lambda: len(peer.follow_targets) == 2)
    assert peer.follow_targets[-1].position == pytest.approx([-.2, .1, .5])
    assert len([r for r in peer.switches if r.activate_controllers]) == 1
    assert gateway._mode == 'follow'


def test_ros_follow_stop_during_switch_discards_pending_and_old_targets(graph):
    peer, gateway = graph
    peer.switch_delay = .18
    old = send_follow(peer)
    assert wait_for(lambda: any(r.activate_controllers for r in peer.switches))
    send_follow(peer, -.3)
    peer.stop_pub.publish(Empty())
    assert wait_for(lambda: gateway._mode == 'stopped' and stopped(peer))
    peer.target_pub.publish(old)
    time.sleep(.15)
    assert not peer.follow_targets
    send_follow(peer, .5, 'elbow')
    assert wait_for(lambda: peer.follow_targets)
    assert peer.follow_targets[-1].position == pytest.approx([.2, .1, .5])


def test_ros_follow_client_disconnect_and_action_handover_do_not_replay(graph):
    peer, gateway = graph
    send_follow(peer)
    assert wait_for(lambda: peer.follow_targets)
    send_trajectory(peer)
    assert wait_for(lambda: peer.goals and gateway._mode == 'trajectory')
    assert peer.states['follow_controller'] == 'inactive'
    send_follow(peer, -.2)
    assert wait_for(lambda: len(peer.follow_targets) == 2)
    assert peer.canceled_goals
    assert peer.states['trajectory_controller'] == 'inactive'
    peer.count_pub.publish(Int32(data=0))
    assert wait_for(lambda: gateway._mode == 'stopped' and stopped(peer))
    peer.count_pub.publish(Int32(data=1))
    time.sleep(.1)
    assert len(peer.follow_targets) == 2
    assert stopped(peer)


def test_ros_follow_controller_error_closes_hardware_and_discards_target(graph):
    peer, gateway = graph
    send_follow(peer)
    assert wait_for(lambda: peer.follow_targets)
    peer.states['follow_controller'] = 'inactive'
    assert wait_for(lambda: peer.hardware_stops and gateway._mode == 'stopped')
    assert '连续跟随控制器意外停止' in gateway._reject
    assert not gateway._follow_targets
    assert len(peer.follow_targets) == 1
