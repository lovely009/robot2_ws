"""隔离 ROS 图上的真实 JTC → Topic 硬件 → V3 桥；设备端完全由本测试提供。"""
import os
import json
from pathlib import Path
import signal
import subprocess
import threading
import time

import pytest

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers, SwitchController
import rclpy
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.qos import DurabilityPolicy
from std_msgs.msg import String, Empty, Int32
from trajectory_msgs.msg import JointTrajectoryPoint
from can_bridge_msgs.msg import (AgentCommand, AgentConfigRequest, AgentDeviceInfo,
                                 AgentInfo, AgentMotion, AgentMotorStatus, AgentStatus,
                                 AgentWireContract, MotorControlState)
from can_bridge_msgs.msg import MotorCommandArray, MotorCommandTarget, MotorTrajectory, MotorTrajectoryPoint


class TopicDevice(Node):
    def __init__(self, context):
        super().__init__('host_trajectory_test_device', context=context)
        self.position = 0.0
        self.velocity = 0.0
        self.armed = False
        self.commands = []
        self.sequence = 0
        self.command_sequence = 0
        self.began = time.monotonic()
        self.lock = threading.Lock()
        self.status = self.create_publisher(AgentStatus, AgentWireContract.TOPIC_STATUS, 10)
        self.info = self.create_publisher(AgentInfo, AgentWireContract.TOPIC_INFO, 10)
        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.motion = self.create_publisher(AgentMotion, AgentWireContract.TOPIC_MOTION, best_effort)
        self.create_subscription(AgentCommand, AgentWireContract.TOPIC_COMMAND,
                                 self.command, best_effort)
        self.create_subscription(AgentConfigRequest, AgentWireContract.TOPIC_CONFIG,
                                 self.config, 10)
        self.create_timer(.01, self.publish_state)

    def config(self, request):
        if request.operation == AgentConfigRequest.STOP_ALL:
            self.armed = False
        elif (request.operation == AgentConfigRequest.SET_POLICY and request.policies
              and all(not p.enabled for p in request.policies)):
            self.armed = False
        elif request.operation == AgentConfigRequest.QUERY_INFO:
            reply = AgentInfo()
            reply.session_id = 100
            reply.clock_valid = True
            reply.snapshot_stamp = self.get_clock().now().nanoseconds
            reply.query_id = request.query_id
            reply.revision = 1
            reply.cursor = request.cursor
            reply.next_cursor = 1
            reply.complete = True
            device = AgentDeviceInfo()
            device.motor_id = 1
            device.protocol = 1
            device.model_name = 'cerb'
            device.online = True
            device.parameters.motor_id = 1
            reply.devices = [device]
            self.info.publish(reply)

    def command(self, message):
        with self.lock:
            self.commands.append(message)
            self.command_sequence = message.sequence
            if message.count and self.armed:
                self.position = message.position_q[0] * AgentWireContract.SCALE_POSITION / 32767
                self.velocity = message.velocity_q[0] * AgentWireContract.SCALE_VELOCITY / 32767

    def publish_state(self):
        with self.lock:
            self.sequence += 1
            stamp = self.get_clock().now().nanoseconds
            tick = int((time.monotonic() - self.began) * 1000) + 1000
            status = AgentStatus()
            status.schema_version = AgentWireContract.SCHEMA_VERSION
            status.session_id = 100
            status.sequence = self.sequence
            status.clock_valid = True
            status.snapshot_stamp = stamp
            status.snapshot_tick_ms = tick
            status.info_revision = 1
            status.command_sequence = self.command_sequence
            motor = AgentMotorStatus()
            motor.motor_id = 1
            motor.online = True
            motor.status_flags = 1
            motor.policy_enabled = self.armed
            motor.lifecycle = (MotorControlState.LIFECYCLE_ACTIVE if self.armed
                               else MotorControlState.LIFECYCLE_INACTIVE)
            motor.execution_state = MotorControlState.EXECUTION_IDLE
            status.motors = [motor]
            self.status.publish(status)
            motion = AgentMotion()
            motion.schema_version = AgentWireContract.SCHEMA_VERSION
            motion.session_id = 100
            motion.sequence = self.sequence
            motion.snapshot_stamp = stamp
            motion.snapshot_tick_ms = tick
            motion.group_count = 1
            motion.count = 1
            motion.motor_ids = [1]
            motion.position_q = [round(self.position * 32767 / AgentWireContract.SCALE_POSITION)]
            motion.velocity_q = [round(self.velocity * 32767 / AgentWireContract.SCALE_VELOCITY)]
            motion.effort_q = [0]
            motion.state_age_ms = [0]
            motion.target_source = [0]
            self.motion.publish(motion)


def wait_for(predicate, seconds=10):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(.01)
    return bool(predicate())


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


def _exercise_workbench_client(device):
    """模拟网页低频请求，实际运动帧仍只能来自 controller_manager。"""
    status, progress = {}, []
    device.create_subscription(String, '/control/status',
                               lambda m: status.update(json.loads(m.data)), 10)
    from can_bridge_msgs.msg import MotorTrajectoryState
    device.create_subscription(MotorTrajectoryState, '/control/trajectory_state',
                               progress.append, 10)
    manual = device.create_publisher(MotorCommandArray, '/control/setpoint', 10)
    trajectory = device.create_publisher(MotorTrajectory, '/control/trajectory', 10)
    soft_stop = device.create_publisher(Empty, '/control/stop', 10)
    clients = device.create_publisher(Int32, '/client_count', QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    controllers = device.create_client(ListControllers, '/host_control/controller_manager/list_controllers')
    pending = None
    def ready():
        nonlocal pending
        if pending is None:
            pending = controllers.call_async(ListControllers.Request())
        if not pending.done():
            return False
        names = {c.name for c in pending.result().controller if c.state in ('active', 'inactive')}
        pending = None
        return {'trajectory_controller', 'gravity_controller', 'manual_controller_0'}.issubset(names)
    assert wait_for(lambda: manual.get_subscription_count() and status.get('feedback_fresh')
                    and ready()), json.dumps(status)
    clients.publish(Int32(data=1))
    request = MotorCommandArray(count=1, sequence=1,
        sync_time_ns=device.get_clock().now().nanoseconds,
        targets=[MotorCommandTarget(motor_id=1, position=.04, velocity=0., effort=1.)])
    manual.publish(request)
    assert wait_for(lambda: status.get('mode') == 'manual'), json.dumps(status)
    assert wait_for(lambda: any(m.count and m.effort_q[0] != 0 for m in device.commands)), status
    assert abs(device.position - .04) < .01
    soft_stop.publish(Empty())
    assert wait_for(lambda: status.get('mode') == 'stopped'), status
    time.sleep(.1)
    positive = sum(bool(m.count) for m in device.commands)
    time.sleep(.15)
    assert sum(bool(m.count) for m in device.commands) == positive

    # 显式停止撤销设备授权；后续新动作由测试设备模拟新的人工授权。
    device.armed = True

    def publish_trajectory(identifier, position, milliseconds):
        message = MotorTrajectory(sync_time_ns=device.get_clock().now().nanoseconds,
            sequence=identifier, trajectory_id=identifier, segment_count=1,
            count=2, target_count=1, target_ids=[1], points=[
                MotorTrajectoryPoint(motor_id=1, time_from_start_ms=0,
                                     position=device.position, velocity=0., effort=0.),
                MotorTrajectoryPoint(motor_id=1, time_from_start_ms=milliseconds,
                                     position=position, velocity=0., effort=0.)])
        trajectory.publish(message)

    publish_trajectory(77, .08, 1000)
    assert wait_for(lambda: any(m.trajectory_id == 77 and m.completed for m in progress), 8), status
    assert abs(device.position - .08) < .01
    gravity = device.create_client(SetParametersAtomically,
                                    '/host_control/gravity_controller/set_parameters_atomically')
    assert wait_for(gravity.service_is_ready)
    changed = gravity.call_async(SetParametersAtomically.Request(parameters=[
        Parameter('enabled', value=True).to_parameter_msg(),
        Parameter('effort_gain', value=.5).to_parameter_msg()]))
    assert wait_for(changed.done) and changed.result().result.successful
    assert wait_for(lambda: status.get('gravity_enabled') is True), json.dumps(status)
    assert wait_for(lambda: any(m.count and abs(m.effort_q[0]) > 400
                               for m in device.commands[-15:])), json.dumps(status)
    changed = gravity.call_async(SetParametersAtomically.Request(parameters=[
        Parameter('enabled', value=False).to_parameter_msg()]))
    assert wait_for(changed.done) and changed.result().result.successful
    assert wait_for(lambda: status.get('gravity_enabled') is False), json.dumps(status)
    assert wait_for(lambda: device.commands[-1].count and device.commands[-1].effort_q[0] == 0)
    publish_trajectory(78, .3, 3000)
    assert wait_for(lambda: device.position > .09), status
    trajectory.publish(MotorTrajectory(trajectory_id=78, segment_count=1,
                                      sync_time_ns=device.get_clock().now().nanoseconds))
    assert wait_for(lambda: any(m.trajectory_id == 78 and not m.active and not m.completed
                               for m in progress)), status
    clients.publish(Int32(data=0))
    assert wait_for(lambda: status.get('mode') == 'stopped'), status
    time.sleep(.1)
    positive = sum(bool(m.count) for m in device.commands)
    request.sequence += 1
    request.sync_time_ns = device.get_clock().now().nanoseconds
    manual.publish(request)
    time.sleep(.25)
    assert sum(bool(m.count) for m in device.commands) == positive, status


@pytest.mark.parametrize('via_workbench', [False, True])
def test_real_jtc_executes_cancels_and_cannot_renew_after_control_loop_stops(
        tmp_path, monkeypatch, via_workbench):
    monkeypatch.setenv('ROS_DOMAIN_ID', '179')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    context = Context()
    rclpy.init(context=context)
    device = TopicDevice(context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(device)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    driver = tmp_path / 'motors.yaml'
    driver.write_text('motor_hardware:\n  motors:\n    axis: {id: 1, joint: shoulder}\n')
    urdf = tmp_path / 'robot.urdf'
    urdf.write_text('<robot name="test"><link name="base"/><link name="arm">'
                    '<inertial><origin xyz="1 0 0"/><mass value="1"/>'
                    '<inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>'
                    '</inertial></link>'
                    '<joint name="shoulder" type="revolute"><parent link="base"/>'
                    '<child link="arm"/><axis xyz="0 1 0"/>'
                    '<limit lower="-1" upper="1" velocity="1" effort="10"/></joint></robot>')
    bridge_executable = Path(get_package_prefix('micro_ros_bridge')) / 'lib/micro_ros_bridge/micro_ros_bridge_node'
    launch_file = Path(get_package_share_directory('motor_hardware_interface')) / 'launch/host_trajectory.launch.py'
    children, logs = [], []
    client = ActionClient(device, FollowJointTrajectory,
                          '/host_control/trajectory_controller/follow_joint_trajectory')
    try:
        for name, command in (
            ('bridge', [str(bridge_executable), '--ros-args', '-p', 'trajectory_execution:=host']),
            ('controller', ['ros2', 'launch', str(launch_file),
                            f'motor_config_file:={driver}', f'urdf_file:={urdf}',
                            f'start_controllers:={str(not via_workbench).lower()}']),
        ):
            log = open(tmp_path / f'{name}.log', 'w')
            logs.append(log)
            children.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                             start_new_session=True))
        assert wait_for(lambda: client.server_is_ready()), '\n'.join(
            (tmp_path / f'{name}.log').read_text() for name in ('bridge', 'controller'))
        states = device.create_client(ListControllers, '/host_control/controller_manager/list_controllers')
        assert wait_for(states.service_is_ready)
        pending = None
        def controller_active():
            nonlocal pending
            if pending is None:
                pending = states.call_async(ListControllers.Request())
            if not pending.done():
                return False
            active = any(c.name == 'trajectory_controller' and c.state == 'active'
                         for c in pending.result().controller)
            pending = None
            return active
        assert wait_for(lambda: any(m.count == 0 for m in device.commands))
        # 仅模拟设备状态，不向任何真实电机发送使能请求。
        device.armed = True
        if via_workbench:
            _exercise_workbench_client(device)
            stop(children[1])
            time.sleep(.1)
            count = len(device.commands)
            time.sleep(.15)
            assert len(device.commands) == count
            return
        assert wait_for(controller_active)
        assert wait_for(lambda: any(m.count for m in device.commands))

        def goal(position, seconds):
            request = FollowJointTrajectory.Goal()
            request.trajectory.joint_names = ['shoulder']
            point = JointTrajectoryPoint(positions=[position], velocities=[0.0])
            point.time_from_start.sec = seconds
            request.trajectory.points = [point]
            future = client.send_goal_async(request)
            assert wait_for(future.done)
            handle = future.result()
            assert handle.accepted
            return handle

        completed = goal(.08, 1).get_result_async()
        assert wait_for(completed.done, 5)
        assert completed.result().status == GoalStatus.STATUS_SUCCEEDED
        assert completed.result().result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        assert abs(device.position - .08) < .01
        handle = goal(.3, 3)
        assert wait_for(lambda: device.position > .09, 3)
        canceled = handle.cancel_goal_async()
        assert wait_for(canceled.done)
        assert canceled.result().goals_canceling
        result = handle.get_result_async()
        assert wait_for(result.done)
        assert result.result().status == GoalStatus.STATUS_CANCELED
        time.sleep(.2)
        assert device.position < .2
        switches = device.create_client(SwitchController,
                                        '/host_control/controller_manager/switch_controller')
        assert wait_for(switches.service_is_ready)
        request = SwitchController.Request()
        request.deactivate_controllers = ['trajectory_controller']
        request.strictness = SwitchController.Request.STRICT
        request.timeout.sec = 2
        switched = switches.call_async(request)
        assert wait_for(switched.done)
        assert switched.result().ok
        time.sleep(.1)
        positive_count = sum(bool(m.count) for m in device.commands)
        time.sleep(.15)
        assert sum(bool(m.count) for m in device.commands) == positive_count, (
            'JTC 停用、manager 仍运行时，硬件不得继续给旧目标续租')
        stop(children[1])
        time.sleep(.1)
        count = len(device.commands)
        time.sleep(.15)
        assert len(device.commands) == count, '控制循环停止后桥不得独立重发目标或续租'
    finally:
        for child in reversed(children):
            stop(child)
        for log in logs:
            log.close()
        executor.shutdown()
        thread.join(timeout=3)
        client.destroy()
        device.destroy_node()
        context.shutdown()
