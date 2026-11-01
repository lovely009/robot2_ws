"""低频工作台客户端；运动周期、插值和重力均由 ros2_control 拥有。"""
from dataclasses import dataclass, field
import json
import math
import signal
import threading
import time
import xml.etree.ElementTree as ET

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from can_bridge_msgs.msg import (AgentWireContract, MotorCommandArray, MotorPolicyArray, MotorPolicyTarget,
                                 MotorTrajectory, MotorTrajectoryState)
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers, SwitchController
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from robot_transmission import Axis, Model
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float64MultiArray, Int32, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

FRESH_NS = 500_000_000
RPC_TIMEOUT = 2.
ASSEMBLY_TIMEOUT = 2.
ACTION = '/host_control/trajectory_controller/follow_joint_trajectory'
CHANNELS = ('position', 'velocity', 'effort')


def stamp_ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


def fresh(stamp, now):
    if stamp <= 0 or not -50_000_000 <= now - stamp <= FRESH_NS:
        raise ValueError('输入或反馈时间戳无效、过期或来自未来')


class WorkbenchModel:
    """公开转换接口；所有未指定关节只从本次新鲜实测生成静止保持。"""
    def __init__(self, config_file, urdf_file):
        try:
            self.model = Model.load(config_file)
        except (ValueError, RuntimeError) as error:
            raise ValueError(str(error)) from error
        self.joint_names = list(self.model.joint_names)
        self.ids = dict(self.model.actuator_ids)
        self.names_by_id = {value: key for key, value in self.ids.items()}
        if (not self.joint_names or not 0 < len(self.ids) <= 48
                or len(self.names_by_id) != len(self.ids)
                or any(not 1 <= value <= 64 for value in self.ids.values())):
            raise ValueError('模型 motor_id 必须唯一、位于 1..64 且最多 48 台')
        root = ET.parse(urdf_file).getroot()
        joints = root.findall('joint')
        nodes = {joint.get('name'): joint for joint in joints}
        if (root.tag != 'robot' or len(nodes) != len(joints)
                or any('xacro' in element.tag for element in root.iter())):
            raise ValueError('必须提供关节名唯一、展开后的 URDF')
        self.limits = {}
        for name in self.joint_names:
            joint = nodes.get(name)
            if (joint is None or joint.get('type') not in ('revolute', 'prismatic', 'continuous')
                    or joint.find('mimic') is not None):
                raise ValueError(f'{name} 必须是 URDF 中独立的活动关节')
            limit = joint.find('limit')
            try:
                velocity, effort = float(limit.get('velocity')), float(limit.get('effort'))
                lower, upper = -math.inf, math.inf
                if joint.get('type') != 'continuous':
                    lower, upper = float(limit.get('lower')), float(limit.get('upper'))
                    if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
                        raise ValueError()
                if not all(math.isfinite(v) and v > 0 for v in (velocity, effort)):
                    raise ValueError()
            except (AttributeError, TypeError, ValueError) as error:
                raise ValueError(f'{name} 缺少有效的 URDF P/V/E 限位') from error
            self.limits[name] = lower, upper, velocity, effort
        zero = self.model.actuator_to_joint({a: Axis() for a in self.ids})
        self.position_tolerance = dict.fromkeys(self.joint_names, 1e-7)
        for actuator in self.ids:
            delta = self.model.actuator_to_joint({a: Axis(
                AgentWireContract.SCALE_POSITION / 32767 if a == actuator else 0.) for a in self.ids})
            for name in self.joint_names:
                # 两次量化测量的差最多一格，再由同一传动 Model 换算到关节。
                self.position_tolerance[name] += abs(delta[name].position - zero[name].position)

    def feedback(self, message, now_ns):
        fresh(stamp_ns(message.header.stamp), now_ns)
        names = list(message.name)
        if (len(names) != len(set(names)) or not set(self.joint_names).issubset(names)
                or any(len(values) != len(names) for values in
                       (message.position, message.velocity, message.effort))):
            raise ValueError('反馈必须包含全部受控关节及完整 P/V/E')
        result = {name: Axis(p, v, e) for name, p, v, e in zip(
            names, message.position, message.velocity, message.effort) if name in self.limits}
        if not all(math.isfinite(v) for axis in result.values()
                   for v in (axis.position, axis.velocity, axis.effort)):
            raise ValueError('反馈包含非有限值')
        return result

    def _complete(self, targets, measured):
        if set(measured) != set(self.joint_names):
            raise ValueError('缺少全模型新鲜反馈')
        result = {name: targets.get(name, Axis(measured[name].position))
                  for name in self.joint_names}
        for name, axis in result.items():
            lower, upper, velocity, effort = self.limits[name]
            if (not all(math.isfinite(v) for v in (axis.position, axis.velocity, axis.effort))
                    or not lower <= axis.position <= upper
                    or abs(axis.velocity) > velocity or abs(axis.effort) > effort):
                raise ValueError(f'{name} 目标非有限或超过 URDF P/V/E 限位')
        # 同一 Model 验证完整传动组，禁止在客户端再实现混控。
        self.model.joint_to_actuator(result)
        return result

    def _motors(self, targets):
        ids = [target.motor_id for target in targets]
        if len(ids) != len(set(ids)) or not set(ids).issubset(self.names_by_id):
            raise ValueError('motor_id 重复或不属于模型')
        values = {self.names_by_id[p.motor_id]: Axis(p.position, p.velocity, p.effort)
                  for p in targets}
        for group in self.model.groups:
            present = set(group.actuators).intersection(values)
            if present and present != set(group.actuators):
                raise ValueError(f'{group.name} 必须提供完整传动组')
        try:
            return self.model.actuator_to_joint(values)
        except (ValueError, RuntimeError) as error:
            raise ValueError(str(error)) from error

    def manual(self, message, measured, now_ns):
        fresh(message.sync_time_ns, now_ns)
        if not 0 < message.count == len(message.targets) <= 64:
            raise ValueError('手动快照数量无效')
        values = self._complete(self._motors(message.targets), measured)
        return [getattr(values[name], channel) for name in self.joint_names for channel in CHANNELS]

    def joint_trajectory(self, message, measured, now_ns):
        if stamp_ns(message.header.stamp):
            fresh(stamp_ns(message.header.stamp), now_ns)
        names = list(message.joint_names)
        if (not names or not message.points or len(names) != len(set(names))
                or not set(names).issubset(self.joint_names)):
            raise ValueError('轨迹为空、关节名重复或不属于模型')
        output = JointTrajectory(header=message.header, joint_names=self.joint_names)
        previous, previous_values = 0, measured
        for index, point in enumerate(message.points):
            t = stamp_ns(point.time_from_start)
            if (not 0 <= point.time_from_start.nanosec < 10**9 or t < 0
                    or t > 0xFFFFFFFF * 10**6 or (index and t <= previous)):
                raise ValueError('轨迹时间必须非负、严格递增且可由状态毫秒表示')
            if (len(point.positions) != len(names)
                    or any(values and len(values) != len(names)
                           for values in (point.velocities, point.effort, point.accelerations))):
                raise ValueError('轨迹通道长度不匹配')
            if point.accelerations:
                raise ValueError('当前 P/V 轨迹入口不支持 acceleration')
            if any(not math.isfinite(e) or e != 0 for e in point.effort):
                raise ValueError('JTC 仅占 P/V；轨迹非零 effort 无执行接口，请使用手动 P/V/E')
            values = self._complete({name: Axis(point.positions[i],
                point.velocities[i] if point.velocities else 0.) for i, name in enumerate(names)}, measured)
            if t == 0:
                if any(abs(values[n].position - measured[n].position) > self.position_tolerance[n]
                       for n in self.joint_names):
                    raise ValueError('零时刻目标未对齐实测位置，请改用未来路点')
                values = self._complete({n: Axis(measured[n].position, measured[n].velocity)
                                         for n in self.joint_names}, measured)
            self._interval(previous_values, values, (t - previous) / 10**9)
            output.points.append(JointTrajectoryPoint(
                positions=[values[n].position for n in self.joint_names],
                velocities=[values[n].velocity for n in self.joint_names],
                time_from_start=Duration(sec=t // 10**9, nanosec=t % 10**9)))
            previous, previous_values = t, values
        return output

    def _interval(self, before, after, dt):
        # JTC P/V 使用三次 Hermite；端点合法还不足以保证段内位置/速度合法。
        for name in self.joint_names:
            p, q = before[name], after[name]
            if dt == 0:
                if abs(p.position - q.position) > 1e-7 or abs(p.velocity - q.velocity) > 1e-7:
                    raise ValueError('零时刻目标必须与实测 P/V 连续')
                continue
            a = 2*p.position - 2*q.position + dt*(p.velocity + q.velocity)
            b = -3*p.position + 3*q.position - dt*(2*p.velocity + q.velocity)
            c = dt*p.velocity
            position_times = [0., 1.]
            if abs(a) > 1e-14:
                discriminant = 4*b*b - 12*a*c
                if discriminant >= 0:
                    position_times.extend(((-2*b + math.sqrt(discriminant))/(6*a),
                                           (-2*b - math.sqrt(discriminant))/(6*a)))
            elif abs(b) > 1e-14:
                position_times.append(-c/(2*b))
            lower, upper, velocity, _ = self.limits[name]
            if any(not lower-1e-7 <= ((a*s+b)*s+c)*s+p.position <= upper+1e-7
                   for s in position_times if 0 <= s <= 1):
                raise ValueError(f'{name} JTC 插值越过位置限位')
            velocity_times = [0., 1.] + ([-b/(3*a)] if abs(a) > 1e-14 else [])
            if any(abs((3*a*s*s + 2*b*s + c)/dt) > velocity+1e-7
                   for s in velocity_times if 0 <= s <= 1):
                raise ValueError(f'{name} JTC 插值越过速度限位')

    def motor_trajectory(self, plan, measured, now_ns):
        fresh(plan.sync_time_ns, now_ns)
        message = JointTrajectory(joint_names=self.joint_names)
        frames = {}
        for point in plan.points:
            frames.setdefault(point.time_from_start_ms, []).append(point)
        for t, points in frames.items():
            values = self._complete(self._motors(points), measured)
            message.points.append(JointTrajectoryPoint(
                positions=[values[n].position for n in self.joint_names],
                velocities=[values[n].velocity for n in self.joint_names],
                effort=[values[n].effort for n in self.joint_names],
                time_from_start=Duration(sec=t // 1000, nanosec=t % 1000 * 10**6)))
        return self.joint_trajectory(message, measured, now_ns)


@dataclass
class MotorPlan:
    trajectory_id: int
    target_ids: list
    sync_time_ns: int
    points: list = field(default_factory=list)


class TrajectoryAssembly:
    def __init__(self):
        self.pending = None
        self.closed = set()

    def abort(self):
        if self.pending:
            self.closed.add(self.key)
        self.pending = None

    def add(self, message, now_ns, now):
        key = (message.session_id, message.trajectory_id)
        try:
            fresh(message.sync_time_ns, now_ns)
            if (not 0 < message.count == len(message.points) <= 256
                    or not 0 < message.target_count == len(message.target_ids) <= 64
                    or len(set(message.target_ids)) != message.target_count
                    or not 0 <= message.segment_index < message.segment_count
                    or key in self.closed):
                raise ValueError('轨迹分段数量、ID 或序号无效/重复')
            signature = (key, message.sequence, message.segment_count,
                         tuple(message.target_ids), message.sync_time_ns)
            if self.pending is None:
                if message.segment_index != 0:
                    raise ValueError('必须从第 0 段开始')
                self.key, self.signature, self.started = key, signature, now
                self.next_index, self.last_time, self.frame_ids = 0, -1, set()
                self.pending = MotorPlan(message.trajectory_id, list(message.target_ids),
                                         message.sync_time_ns)
            if (signature != self.signature or message.segment_index != self.next_index
                    or now - self.started > ASSEMBLY_TIMEOUT):
                raise ValueError('轨迹分段乱序、重复、不一致或收集超时')
            ids = set(message.target_ids)
            for point in message.points:
                t = point.time_from_start_ms
                if (t < self.last_time or point.motor_id not in ids
                        or not all(math.isfinite(v) for v in
                                   (point.position, point.velocity, point.effort))):
                    raise ValueError('轨迹路点乱序、ID 无效或包含非有限值')
                if t != self.last_time:
                    if self.last_time >= 0 and self.frame_ids != ids:
                        raise ValueError('每个时间点必须覆盖全部轨迹电机')
                    self.frame_ids = set()
                    self.last_time = t
                if point.motor_id in self.frame_ids:
                    raise ValueError('同一电机时间戳重复')
                self.frame_ids.add(point.motor_id)
            self.pending.points.extend(message.points)
            self.next_index += 1
            if self.next_index != message.segment_count:
                return None
            if self.frame_ids != ids:
                raise ValueError('末时间点缺少轨迹电机')
            result = self.pending
            self.abort()
            return result
        except ValueError:
            self.abort()
            self.closed.add(key)
            raise


@dataclass
class Run:
    state: MotorTrajectoryState
    epoch: int
    handle: object = None
    submitted: bool = False


class WorkbenchControl(Node):
    def __init__(self, **kwargs):
        super().__init__('workbench_control', **kwargs)
        self._lock = threading.RLock()
        immutable = ParameterDescriptor(read_only=True)
        self.config_file = self.declare_parameter('config_file', '', immutable).value
        self.urdf_file = self.declare_parameter('urdf_file', '', immutable).value
        self.model_name = self.declare_parameter('model_name', '', immutable).value
        publish_states = self.declare_parameter('publish_joint_states', True, immutable).value
        self.mapping = WorkbenchModel(self.config_file, self.urdf_file)
        self.manual_names = [f'manual_controller_{i}' for i in range(len(self.mapping.joint_names))]
        self.motion_names = self.manual_names + ['trajectory_controller', 'gravity_controller']
        self._epoch, self._id = 0, 0
        self._busy, self._stopping = False, False
        self._stop_confirmed, self._stop_deadline = False, 0.
        self._connected_clients = None
        self._input_floor_ns = 0
        self._stop_policy_id = 0
        self._feedback = None
        self._feedback_received = 0.
        self._assembly = TrajectoryAssembly()
        self._waits, self._runs, self._switching = [], [], set()
        self._stop_future = self._poll_future = self._params_future = None
        self._mode, self._reason, self._reject = 'stopped', '等待新目标', ''
        self._gravity_enabled = self._effort_gain = None
        self._active = []
        self._controllers_ready = False
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._status_pub = self.create_publisher(String, '/control/status', latched)
        self._transmission_pub = self.create_publisher(String, '/robot_transmission/status', latched)
        self._trajectory_pub = self.create_publisher(MotorTrajectoryState, '/control/trajectory_state', 10)
        self._hardware_stop_pub = self.create_publisher(Empty, '/host_control/stop', 10)

        self._joint_pub = self.create_publisher(JointState, '/joint_states', 10) if publish_states else None
        self._manual_pubs = [self.create_publisher(Float64MultiArray,
            f'/host_control/{name}/commands', 10) for name in self.manual_names]
        self._list = self.create_client(ListControllers, '/host_control/controller_manager/list_controllers')
        self._switch = self.create_client(SwitchController, '/host_control/controller_manager/switch_controller')
        self._params = self.create_client(GetParameters, '/host_control/gravity_controller/get_parameters')
        self._action = ActionClient(self, FollowJointTrajectory, ACTION)
        for kind, topic, callback, qos in (
            (MotorCommandArray, '/robot/managed/control/setpoint', self._on_setpoint, 10),
            (MotorTrajectory, '/robot/managed/control/trajectory', self._on_motor_trajectory, 10),
            (JointTrajectory, '/robot/managed/control/joint_trajectory', self._on_joint_trajectory, 10),
            (Empty, '/robot/managed/control/stop', lambda _: self._stop('显式停止'), 10),
            (Int32, '/client_count', self._on_client_count, latched),
            (JointState, '/host_control/joint_states', self._on_feedback, qos_profile_sensor_data),
        ):
            self.create_subscription(kind, topic, self._locked(callback), qos)
        self.create_timer(.05, self._locked(self._tick))
        self.create_timer(1., self._locked(self._poll))
        self._publish_status()

    def _locked(self, callback):
        def invoke(*args):
            with self._lock:
                return callback(*args)
        return invoke

    def _measured(self):
        if self._feedback is None or time.monotonic() - self._feedback_received > FRESH_NS / 10**9:
            raise ValueError('缺少新鲜关节反馈')
        return self.mapping.feedback(self._feedback, self.get_clock().now().nanoseconds)

    def _on_feedback(self, message):
        self._feedback, self._feedback_received = message, time.monotonic()
        if self._joint_pub:
            self._joint_pub.publish(message)

    def _on_client_count(self, message):
        self._connected_clients = max(0, message.data)
        if self._connected_clients == 0:
            self._stop('客户端归零')

    def _reject_input(self, reason, state=None):
        self._reject = str(reason)
        self.get_logger().warning('控制输入被拒绝：' + self._reject)
        if state:
            state.active = state.completed = False
            self._receipt(state)
        self._publish_status()

    def _on_setpoint(self, message):
        if message.count == 0 and not message.targets:
            self._stop('空手动快照停止')
            return
        if message.sync_time_ns < self._input_floor_ns:
            self._reject_input('目标早于最近一次停止')
            return
        self._submit('manual', lambda measured, now: self.mapping.manual(message, measured, now))

    def _state(self, trajectory_id, ids, total_ms=0):
        return MotorTrajectoryState(trajectory_id=trajectory_id, target_ids=list(ids),
                                     target_count=len(ids), total_ms=total_ms)

    def _on_motor_trajectory(self, message):
        if message.count == 0 and message.target_count == 0 and not message.points and not message.target_ids:
            self._cancel()
            return
        state = self._state(message.trajectory_id, message.target_ids)
        try:
            if self._connected_clients == 0 or self._stopping or self._busy:
                raise ValueError('客户端归零或控制切换尚未完成')
            if message.sync_time_ns < self._input_floor_ns:
                raise ValueError('轨迹早于最近一次停止')
            self._measured()
            plan = self._assembly.add(message, self.get_clock().now().nanoseconds, time.monotonic())
            if plan is None:
                self._reason = '轨迹分段收集中'
                self._publish_status()
                return
        except ValueError as error:
            self._assembly.abort()
            self._reject_input(error, state)
            return
        self._id = max(self._id, plan.trajectory_id)
        self._submit('trajectory', lambda measured, now: self.mapping.motor_trajectory(plan, measured, now), state)

    def _on_joint_trajectory(self, message):
        if not message.joint_names and not message.points:
            self._cancel()
            return
        self._id = self._id % 0xFFFFFFFF + 1
        state = self._state(self._id, sorted(self.mapping.ids.values()))
        if stamp_ns(message.header.stamp) and stamp_ns(message.header.stamp) < self._input_floor_ns:
            self._reject_input('轨迹早于最近一次停止', state)
            return
        self._submit('trajectory', lambda measured, now: self.mapping.joint_trajectory(message, measured, now), state)

    def _submit(self, mode, build, state=None):
        try:
            if self._connected_clients == 0 or self._stopping or self._busy:
                raise ValueError('客户端归零或控制切换尚未完成')
            if self._assembly.pending:
                raise ValueError('尚有未收齐轨迹分段')
            result = build(self._measured(), self.get_clock().now().nanoseconds)
            if state:
                state.total_ms = stamp_ns(result.points[-1].time_from_start) // 10**6
            if not self._list.service_is_ready() or not self._switch.service_is_ready():
                raise RuntimeError('controller manager 服务不可达')
            if mode == 'trajectory' and not self._action.server_is_ready():
                raise RuntimeError('JTC action 服务不可达')
        except ValueError as error:
            self._reject_input(error, state)
            return
        except RuntimeError as error:
            self._reject_input(error, state)
            self._stop(str(error), str(error))
            return
        self._epoch += 1
        epoch = self._epoch
        self._busy, self._mode, self._reason, self._reject = True, 'switching', '校验并切换控制器', ''
        run = Run(state, epoch) if state else None
        if run:
            self._runs.append(run)
        def ready():
            self._watch(self._list.call_async(ListControllers.Request()), epoch, '查询控制器',
                lambda response: self._prepare(response, mode, build, run, epoch))
        self._cancel_handles(epoch, ready)
        self._publish_status()

    def _controllers(self, response, mode):
        controllers = {c.name: c for c in response.controller}
        if not set(self.motion_names).issubset(controllers):
            raise ValueError('controller manager 缺少配置的运动控制器')
        desired = self.manual_names if mode == 'manual' else ['trajectory_controller', 'gravity_controller']
        for name in desired:
            c = controllers[name]
            if c.state not in ('active', 'inactive'):
                raise ValueError(f'{name} 未配置或不可切换')
            if name in self.manual_names:
                joint = self.mapping.joint_names[self.manual_names.index(name)]
                expected = [joint + '/' + channel for channel in CHANNELS]
                if list(c.required_command_interfaces) != expected:
                    raise ValueError(f'{name} P/V/E 接口顺序与模型不一致')
            else:
                expected = [j + '/' + channel for j in self.mapping.joint_names for channel in
                            (('effort',) if name == 'gravity_controller' else ('position', 'velocity'))]
                if set(c.required_command_interfaces) != set(expected):
                    raise ValueError(f'{name} 接口集合与模型不一致')
            if c.state == 'active' and set(c.claimed_interfaces) != set(expected):
                raise ValueError(f'{name} 实际占用接口与配置不一致')
        return controllers, desired

    def _prepare(self, response, mode, build, run, epoch):
        controllers, desired = self._controllers(response, mode)
        build(self._measured(), self.get_clock().now().nanoseconds)
        activate = [n for n in desired if controllers[n].state != 'active']
        deactivate = [n for n in self.motion_names if n not in desired and controllers[n].state == 'active']
        def confirm(_=None):
            self._watch(self._list.call_async(ListControllers.Request()), epoch, '确认控制器状态',
                lambda result: self._execute(result, mode, build, run, epoch))
        if not activate and not deactivate:
            confirm()
            return
        request = SwitchController.Request(activate_controllers=activate, deactivate_controllers=deactivate,
            strictness=SwitchController.Request.STRICT, timeout=Duration(sec=1))
        future = self._switch.call_async(request)
        self._switching.add(future)
        def switched(result):
            self._switching.discard(future)
            if not result.ok:
                raise ValueError('STRICT 控制器切换失败')
            confirm()
        def late(_):
            self._switching.discard(future)
            self._request_stop()
        def failed(reason):
            if future.done():
                self._switching.discard(future)
            self._fail(reason)
        self._watch(future, epoch, '切换控制器', switched, late=late, on_error=failed)

    def _execute(self, response, mode, build, run, epoch):
        controllers, desired = self._controllers(response, mode)
        self._active = [name for name in self.motion_names if controllers[name].state == 'active']
        if set(self._active) != set(desired):
            raise ValueError('实际 active 控制器不符合互斥控制模式')
        output = build(self._measured(), self.get_clock().now().nanoseconds)
        if mode == 'manual':
            if not all(p.get_subscription_count() for p in self._manual_pubs):
                raise ValueError('手动控制器 commands 订阅不可达')
            for i, publisher in enumerate(self._manual_pubs):
                publisher.publish(Float64MultiArray(data=output[3*i:3*i+3]))
            self._busy, self._mode, self._reason = False, mode, '手动控制器已激活，已发送单次目标'
        else:
            if not self._action.server_is_ready():
                raise ValueError('JTC action 服务不可达')
            run.submitted = True
            future = self._action.send_goal_async(FollowJointTrajectory.Goal(trajectory=output),
                feedback_callback=self._locked(lambda msg: self._action_feedback(run, msg)))
            self._watch(future, None, 'JTC goal 受理', lambda handle: self._accepted(run, handle),
                        late=lambda handle: self._accepted(run, handle),
                        on_error=lambda reason: self._goal_error(run, reason))
            self._reason = '等待 JTC goal 受理'
        self._publish_status()

    def _accepted(self, run, handle):
        if handle is None:
            self._goal_error(run, 'JTC goal 受理服务失败，服务端状态未确认')
            return
        if not handle.accepted:
            self._finish(run, False, 'JTC 拒绝 goal', failed=True)
            return
        run.handle = handle
        if run.epoch != self._epoch:
            self._cancel_one(run, self._epoch)
        else:
            self._busy, self._mode, self._reason = False, 'trajectory', 'JTC goal 已受理'
            run.state.active = True
            self._receipt(run.state)
            self._publish_status()
        self._watch(handle.get_result_async(), None, 'JTC 执行结果',
            lambda result: self._action_result(run, result),
            timeout=run.state.total_ms/1000 + RPC_TIMEOUT + 2.,
            late=lambda result: self._action_result(run, result),
            on_error=lambda reason: self._goal_error(run, reason))

    def _action_feedback(self, run, message):
        if run.epoch == self._epoch and run.handle:
            elapsed = stamp_ns(message.feedback.desired.time_from_start) // 10**6
            run.state.elapsed_ms = min(run.state.total_ms, max(0, elapsed))
            self._receipt(run.state)

    def _action_result(self, run, result):
        if result is None or result.status not in (
                GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED):
            self._goal_error(run, 'JTC 执行终态未确认')
            return
        success = (result.status == GoalStatus.STATUS_SUCCEEDED
                   and result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL)
        canceled = result.status == GoalStatus.STATUS_CANCELED or run.epoch != self._epoch
        reason = ('JTC 已取消' if canceled else 'JTC 已完成' if success else
                  f'JTC 中止/失败 {result.result.error_code}: {result.result.error_string}')
        self._finish(run, success and not canceled, reason, failed=not success and not canceled)

    def _finish(self, run, success, reason, failed=False):
        run.state.active, run.state.completed = False, success
        if success:
            run.state.elapsed_ms = run.state.total_ms
        self._receipt(run.state)
        if run in self._runs:
            self._runs.remove(run)
        if run.epoch == self._epoch:
            self._busy, self._reason = False, reason
            if failed:
                self._stop(reason, reason)
            else:
                self._publish_status()
        if self._stopping:
            self._complete_stop()

    def _goal_error(self, run, reason):
        if run not in self._runs:
            return
        if self._stopping:
            self._stop_error(reason)
        else:
            self._fail(reason)

    def _cancel_one(self, run, epoch, done=lambda: None):
        def canceled(response):
            # 已到终态的 goal 可以没有 goals_canceling；终态回调拥有最终回执。
            if not response.goals_canceling and run.handle.status not in (
                    GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED):
                raise ValueError('JTC 拒绝取消 goal')
            done()
        self._watch(run.handle.cancel_goal_async(), epoch, '取消 JTC goal', canceled,
                    on_error=lambda reason: self._goal_error(run, reason))

    def _cancel_handles(self, epoch, done):
        runs = [run for run in self._runs if run.handle and run.state.active]
        remaining = len(runs)
        def canceled():
            nonlocal remaining
            remaining -= 1
            if remaining == 0:
                done()
        if not runs:
            done()
        for run in runs:
            self._cancel_one(run, epoch, canceled)

    def _cancel(self):
        self._abort_assembly()
        if self._busy or self._stopping:
            self._stop('切换期间取消轨迹')
            return
        self._epoch += 1
        self._busy = True
        def done():
            self._busy, self._reason = False, '已请求 JTC 取消并保持'
            self._publish_status()
        self._cancel_handles(self._epoch, done)

    def _stop(self, reason, reject=''):
        if not self._stopping:
            self._stop_deadline = time.monotonic() + RPC_TIMEOUT
        self._stop_confirmed = False
        self._epoch += 1
        self._hardware_stop_pub.publish(Empty())
        self._stop_policy_id = self._epoch % 0xFFFFFFFF or 1
        self._busy = self._stopping = True
        self._mode, self._reason, self._reject = 'stopping', reason, reject
        for run in list(self._runs):
            if run.handle:
                self._cancel_one(run, self._epoch)
            elif not run.submitted:
                self._finish(run, False, reason)
        self._request_stop()
        self._publish_status()

    def _abort_assembly(self):
        if self._assembly.pending:
            plan = self._assembly.pending
            self._receipt(self._state(plan.trajectory_id, plan.target_ids))
        self._assembly.abort()

    def _request_stop(self):
        if not self._stopping:
            return
        if self._stop_future is not None and not self._stop_future.done():
            return
        if not self._switch.service_is_ready():
            self._stop_error('停止失败：controller manager switch 服务不可达')
            return
        epoch = self._epoch
        self._stop_confirmed = False
        request = SwitchController.Request(deactivate_controllers=self.motion_names,
            strictness=SwitchController.Request.BEST_EFFORT, timeout=Duration(sec=1))
        self._stop_future = self._switch.call_async(request)
        def stopped(response):
            if not response.ok or not self._list.service_is_ready():
                self._stop_error('停止失败或无法确认控制器状态')
                return
            self._watch(self._list.call_async(ListControllers.Request()), epoch, '确认全部停止',
                self._confirm_stopped, on_error=self._stop_error)
        self._watch(self._stop_future, epoch, '停用全部运动控制器', stopped,
                    on_error=self._stop_error, late=lambda _: self._request_stop())

    def _confirm_stopped(self, response):
        if not set(self.motion_names).issubset(c.name for c in response.controller):
            self._stop_error('停止未确认：缺少运动控制器状态')
            return
        self._active = [c.name for c in response.controller
                        if c.name in self.motion_names and c.state == 'active']
        if self._active:
            self._stop_error('停止未确认：仍有 active 运动控制器 ' + ','.join(self._active))
            return
        self._stop_confirmed = True
        self._complete_stop()

    def _complete_stop(self):
        # 受理响应与 cancel ack 都不是终态，旧请求未决时不得重新开放输入。
        if self._stop_confirmed and not self._switching and not self._runs:
            self._busy = self._stopping = False
            self._mode = 'stopped'
            self._reason = '控制器及旧 JTC goal 已确认停止'
        elif self._mode != 'error':
            self._reason = ('停止屏障等待旧 JTC goal 受理或取消终态' if self._runs
                            else '等待控制器停用确认')
        self._publish_status()

    def _stop_error(self, reason):
        self._busy = self._stopping = True
        self._mode, self._reject = 'error', self._reject or str(reason)
        self._reason = str(reason)
        self._publish_status()

    def _watch(self, future, epoch, label, done, timeout=None, late=None, on_error=None):
        record = [future, time.monotonic() + (RPC_TIMEOUT if timeout is None else timeout),
                  epoch, label, False, on_error]
        self._waits.append(record)
        def finish(completed):
            if record in self._waits:
                self._waits.remove(record)
            stale = (epoch is not None and epoch != self._epoch) or record[4]
            try:
                result = completed.result()
            except Exception as error:
                if stale and late:
                    late(None)
                elif not stale:
                    (on_error or self._fail)(f'{label}: {error}')
                return
            try:
                if stale:
                    if late:
                        late(result)
                else:
                    done(result)
            except Exception as error:
                (on_error or self._fail)(f'{label}: {error}')
        future.add_done_callback(self._locked(finish))

    def _fail(self, reason):
        self._stop(reason, reason)

    def _tick(self):
        now = time.monotonic()
        if self._stopping and self._runs and now >= self._stop_deadline and self._mode != 'error':
            self._stop_error('停止超时：旧 JTC goal 受理或取消终态未确认')
        for record in list(self._waits):
            if now >= record[1] and record in self._waits:
                self._waits.remove(record)
                record[4] = True
                if record[2] is None or record[2] == self._epoch:
                    reason = record[3] + '超时'
                    if record[5]:
                        record[5](reason)
                    else:
                        self._stop(reason, reason)
        if self._assembly.pending and now - self._assembly.started > ASSEMBLY_TIMEOUT:
            pending = self._assembly.pending
            self._assembly.abort()
            self._reject_input('轨迹分段收集超时', self._state(pending.trajectory_id, pending.target_ids))

    def _poll(self):
        epoch = self._epoch
        if not self._busy and self._list.service_is_ready() and (
                self._poll_future is None or self._poll_future.done()):
            self._poll_future = self._list.call_async(ListControllers.Request())
            def observed(response):
                configured = {c.name for c in response.controller if c.state in ('active', 'inactive')}
                self._controllers_ready = set(self.motion_names).issubset(configured)
                self._active = [c.name for c in response.controller
                                if c.name in self.motion_names and c.state == 'active']
                if not self._active:
                    self._mode = 'stopped'
                elif set(self._active) == set(self.manual_names):
                    self._mode = 'manual'
                elif set(self._active) in ({'trajectory_controller'}, {'trajectory_controller', 'gravity_controller'}):
                    self._mode = 'trajectory'
                else:
                    raise ValueError('检测到非互斥的 active 控制器集合')
                self._publish_status()
            self._watch(self._poll_future, epoch, '读取控制器状态', observed)
        if self._params.service_is_ready() and (self._params_future is None or self._params_future.done()):
            self._params_future = self._params.call_async(GetParameters.Request(names=['enabled', 'effort_gain']))
            def parameters(response):
                self._gravity_enabled = self._effort_gain = None
                if len(response.values) == 2:
                    enabled, gain = response.values
                    if enabled.type == ParameterType.PARAMETER_BOOL:
                        self._gravity_enabled = enabled.bool_value
                    if gain.type == ParameterType.PARAMETER_DOUBLE and math.isfinite(gain.double_value):
                        self._effort_gain = gain.double_value
                self._publish_status()
            self._watch(self._params_future, None, '读取重力配置', parameters,
                        on_error=lambda _: self._unknown_parameters())
        elif not self._params.service_is_ready():
            self._unknown_parameters()
        self._publish_status()

    def _unknown_parameters(self):
        self._gravity_enabled = self._effort_gain = None

    def _receipt(self, state):
        state.sync_time_ns = self.get_clock().now().nanoseconds
        self._trajectory_pub.publish(state)

    def _publish_status(self):
        try:
            self._measured()
            feedback_fresh = True
        except ValueError:
            feedback_fresh = False
        status = dict(model=self.model_name, joint_names=self.mapping.joint_names,
            mode=self._mode, reason=self._reason, reject=self._reject,
            output_topic='/motor/host_command_array', trajectory_output_topic=ACTION,
            gravity_enabled=self._gravity_enabled, effort_gain=self._effort_gain,
            active_controllers=self._active, feedback_fresh=feedback_fresh,
            controllers_ready=self._controllers_ready,
            pending_goal_ids=[run.state.trajectory_id for run in self._runs if run.submitted],
            stop_policy_request_id=self._stop_policy_id,
            safety_note=('已请求全部电机失能；恢复运动前需显式重新使能'
                         if self._stop_policy_id else ''))
        self._status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))
        self._transmission_pub.publish(String(data=json.dumps(dict(status,
            driver_file=self.config_file, trajectory_enabled=True,
            trajectory_topic='/robot/managed/control/trajectory', control_state_source='ros2_control',
            last_error=self._reject), ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WorkbenchControl()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            node._action.destroy()
            node.destroy_node()
        rclpy.try_shutdown()
