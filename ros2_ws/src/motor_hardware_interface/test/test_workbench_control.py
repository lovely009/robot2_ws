"""公开转换接口及隔离 ROS 图；不启动 Agent、设备或运行服务。"""
import copy
import json
import math
from pathlib import Path
import sys
import threading
import time

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from can_bridge_msgs.msg import (MotorCommandArray, MotorCommandTarget, MotorTrajectory,
                                 MotorPolicyArray, MotorTrajectoryPoint, MotorTrajectoryState)
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers, SwitchController
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
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


def test_segment_assembly_is_atomic_and_has_no_total_point_cap(files):
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


class FakeControl(Node):
    def __init__(self, context):
        super().__init__('test_workbench_peer', context=context)
        self.names = [f'manual_controller_{i}' for i in range(3)]
        self.states = {name: 'inactive' for name in self.names +
                       ['trajectory_controller', 'gravity_controller']}
        self.switches, self.commands, self.goals = [], [[], [], []], []
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
        self.create_timer(.02, self.publish_state)

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
                            (('effort',) if name == 'gravity_controller' else ('position', 'velocity'))]
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
        assert peer.policies and peer.hardware_stops


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


def test_ros_stop_revokes_mcu_when_switch_service_is_unavailable(graph):
    peer, _ = graph
    send_manual(peer)
    assert wait_for(lambda: all(peer.commands))
    service = next(service for service in peer.services
                   if service.srv_name == '/host_control/controller_manager/switch_controller')
    peer.destroy_service(service)
    probe = peer.create_client(SwitchController, service.srv_name)
    assert wait_for(lambda: not probe.service_is_ready())
    peer.stop_pub.publish(Empty())
    assert wait_for(lambda: peer.policies)
    policy = peer.policies[-1]
    assert policy.count == 3 and [p.motor_id for p in policy.targets] == [8, 9, 10]
    assert policy.request_id and policy.policy_generation
    assert all(not p.enabled and p.kp == p.kd == 0 for p in policy.targets)
    assert wait_for(lambda: peer.status[-1]['mode'] == 'error')
    assert '显式重新使能' in peer.status[-1]['safety_note']
    send_manual(peer)
    time.sleep(.1)
    assert [len(commands) for commands in peer.commands] == [1, 1, 1]
    assert len(peer.policies) == 1


def test_ros_normal_stop_closes_hardware_gate_and_revokes_mcu_and_deactivates(graph):
    peer, gateway = graph
    send_manual(peer)
    assert wait_for(lambda: all(peer.commands))
    peer.stop_pub.publish(Empty())
    assert wait_for(lambda: peer.policies)
    assert wait_for(lambda: peer.hardware_stops)
    assert wait_for(lambda: peer.status[-1]['mode'] == 'stopped')
    assert all(not p.enabled for policy in peer.policies for p in policy.targets)
    assert all(sub.topic_name != '/host_control/stop' for sub in gateway.subscriptions)
