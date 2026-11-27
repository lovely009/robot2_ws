"""低频工作台客户端；运动周期、插值和重力均由 ros2_control 拥有。"""
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import signal
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from controller_manager_msgs.srv import ListControllers, SwitchController
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from robot_transmission import Axis, Model
from robot_transmission.coupled_limits import CoupledLimits
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Int32, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .feedback_publisher import build_transmission_status
from .transmission_bridge import parse_joint_limits

FRESH_NS = 500_000_000
RPC_TIMEOUT = 2.
ASSEMBLY_TIMEOUT = 2.
DYNAMIC_STATUS_PERIOD = 0.2
DYNAMIC_LIMIT_EPS = 1e-4
TRAJECTORY_MAX_POINTS = 8192  # 与网页一致，按展开后的电机路点计数。
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
        with open(config_file, encoding='utf-8') as handle:
            hardware = yaml.safe_load(handle)['motor_hardware']
        motors = hardware['motors']
        self.policy_defaults = {}
        for name, motor_id in self.ids.items():
            command = motors[name].get('command') or {}
            gains = {key: command.get(key) for key in ('kp', 'kd')}
            if all(type(value) in (int, float) and math.isfinite(value) and value >= 0
                   for value in gains.values()):
                self.policy_defaults[motor_id] = gains
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
        workspace_config = {}
        if hardware.get('coupled_limits_file'):
            path = Path(config_file).resolve().parent / hardware['coupled_limits_file']
            workspace_config = yaml.safe_load(path.read_text(encoding='utf-8'))
            for name, bounds in workspace_config['joint_limits'].items():
                if name not in self.limits:
                    continue
                lower, upper, velocity, effort = self.limits[name]
                if len(bounds) != 2 or not all(math.isfinite(v) for v in bounds):
                    raise ValueError(f'{name} 的耦合模型位置限位无效')
                lower, upper = max(lower, bounds[0]), min(upper, bounds[1])
                if lower >= upper:
                    raise ValueError(f'{name} 的耦合模型与 URDF 位置范围无交集')
                self.limits[name] = lower, upper, velocity, effort
        # 部分注册组使用输入轴回退，只为会话中完整的输出关节对加载耦合区域。
        groups = {name: group for name, group in workspace_config.get('coupled_limits', {}).items()
                  if set(group['joints']).issubset(self.joint_names)}
        self.workspace = CoupledLimits(groups, self.joint_names)
        coupled_names = {joint for group in groups.values() for joint in group['joints']}
        self.dynamic_joint_names = [name for name in self.joint_names if name in coupled_names]
        zero = self.model.actuator_to_joint({a: Axis() for a in self.ids})
        self.position_tolerance = dict.fromkeys(self.joint_names, 1e-7)
        for actuator in self.ids:
            delta = self.model.actuator_to_joint({a: Axis(
                12.5 / 32767 if a == actuator else 0.) for a in self.ids})
            for name in self.joint_names:
                # 两次量化测量的差最多一格，再由同一传动 Model 换算到关节。
                self.position_tolerance[name] += abs(delta[name].position - zero[name].position)

    def active_limits(self, positions, *, groups=None):
        selected = self.workspace.groups if groups is None else groups
        if not all(name in positions and math.isfinite(positions[name])
                   for group in selected for name in self.workspace.groups[group][0]):
            raise ValueError('动态限位缺少有效关节位置')
        self.workspace.validate(positions, groups=selected)
        return self.workspace.active_limits(
            {name: bounds[:2] for name, bounds in self.limits.items()}, positions, groups=selected)

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
        # MCU 量化可能把边界回读推到相邻无效网格。仅在一格量化误差内
        # 归一实测保持位置；显式目标仍使用原值执行严格的耦合区域校验。
        for joints, lower, step, _ in self.workspace.groups.values():
            for name, origin in zip(joints, lower):
                axis = result[name]
                tolerance = self.position_tolerance[name]
                minimum, maximum = self.limits[name][:2]
                if not minimum - tolerance <= axis.position <= maximum + tolerance:
                    continue
                grid_position = float(origin + round((axis.position - origin) / step) * step)
                if abs(axis.position - grid_position) <= tolerance:
                    result[name] = Axis(grid_position, axis.velocity, axis.effort)
        return result

    def _complete(self, targets, measured):
        if set(measured) != set(self.joint_names):
            raise ValueError('缺少全模型新鲜反馈')
        result = {name: targets.get(name, Axis(measured[name].position))
                  for name in self.joint_names}
        for name, axis in result.items():
            lower, upper, velocity, effort = self.limits[name]
            values = []
            for value, low, high in ((axis.position, lower, upper),
                                    (axis.velocity, -velocity, velocity),
                                    (axis.effort, -effort, effort)):
                if not math.isfinite(value):
                    raise ValueError(f'{name} 目标非有限或超过 URDF P/V/E 限位')
                if low <= value <= high:
                    values.append(value)
                else:
                    bounded = low if value < low else high
                    # 只消除 float32 传输的舍入误差，下游仍使用原限位严格校验。
                    if not math.isclose(value, bounded, rel_tol=2**-24, abs_tol=2**-150):
                        raise ValueError(f'{name} 目标非有限或超过 URDF P/V/E 限位')
                    values.append(bounded)
            result[name] = Axis(*values)
        # 其他组可原位保持；只校验本次会运动或施力的耦合组。
        moving_groups = [name for name, (joints, _, _, _) in self.workspace.groups.items()
                         if any(result[joint].position != measured[joint].position
                                or result[joint].velocity != 0. or result[joint].effort != 0.
                                for joint in joints)]
        self.workspace.validate({name: axis.position for name, axis in result.items()}, groups=moving_groups)
        # 同一 Model 验证完整传动组，禁止在客户端再实现混控。
        self.model.joint_to_actuator(result)
        return result

    def _motors(self, targets, measured=None):
        ids = [target.motor_id for target in targets]
        if len(ids) != len(set(ids)) or not set(ids).issubset(self.names_by_id):
            raise ValueError('motor_id 重复或不属于模型')
        values = {self.names_by_id[p.motor_id]: Axis(p.position, p.velocity, p.effort)
                  for p in targets}
        if measured is not None:
            holds = self.model.joint_to_actuator(
                {name: Axis(axis.position) for name, axis in measured.items()})
            values = {name: values.get(name, axis) for name, axis in holds.items()}
        for group in self.model.groups:
            present = set(group.actuators).intersection(values)
            if present and present != set(group.actuators):
                raise ValueError(f'{group.name} 必须提供完整传动组')
        try:
            return self.model.actuator_to_joint(values)
        except (ValueError, RuntimeError) as error:
            raise ValueError(str(error)) from error

    def manual(self, message, measured, now_ns):
        fresh(stamp_ns(message.header.stamp), now_ns)
        if not 0 < message.count == len(message.targets) <= 37:
            raise ValueError('手动快照数量无效')
        values = self._complete(self._motors(message.targets, measured), measured)
        return [getattr(values[name], channel) for name in self.joint_names for channel in CHANNELS]

    def joint_target(self, message, measured, now_ns, previous=None):
        fresh(stamp_ns(message.header.stamp), now_ns)
        names = list(message.name)
        if (not names or len(names) != len(set(names)) or not set(names).issubset(self.joint_names)
                or len(message.position) != len(names) or message.velocity or message.effort):
            raise ValueError('连续目标必须为模型内唯一关节及完整位置，不接受速度/力矩')
        targets = dict(previous or {})
        targets.update({name: Axis(value) for name, value in zip(names, message.position)})
        values = self._complete(targets, measured)
        output = JointState(name=self.joint_names, position=[values[n].position for n in self.joint_names])
        output.header.stamp = message.header.stamp
        return output

    def joint_trajectory(self, message, measured, now_ns, *, linear=False):
        if stamp_ns(message.header.stamp):
            fresh(stamp_ns(message.header.stamp), now_ns)
        names = list(message.joint_names)
        if (not names or not message.points or len(names) != len(set(names))
                or not set(names).issubset(self.joint_names)):
            raise ValueError('轨迹为空、关节名重复或不属于模型')
        if set(measured) != set(self.joint_names):
            raise ValueError('缺少全模型新鲜反馈')
        # Input stamp checks freshness; admission/switch latency must not shorten validated segments.
        J = len(self.joint_names)
        N = len(message.points)
        joint_index = {n: j for j, n in enumerate(self.joint_names)}
        col_idx = [joint_index[n] for n in names]
        m_pos = np.array([measured[n].position for n in self.joint_names], dtype=np.float64)
        m_vel = np.array([measured[n].velocity for n in self.joint_names], dtype=np.float64)
        P = np.broadcast_to(m_pos, (N, J)).copy()
        V = np.zeros((N, J), dtype=np.float64)
        times_ns = np.empty(N, dtype=np.int64)
        expected_len = len(names)
        previous = 0
        for index, point in enumerate(message.points):
            t = stamp_ns(point.time_from_start)
            if (not 0 <= point.time_from_start.nanosec < 10**9 or t < 0
                    or t > 0xFFFFFFFF * 10**6 or (index and t <= previous)):
                raise ValueError('轨迹时间必须非负、严格递增且可由状态毫秒表示')
            if (len(point.positions) != expected_len
                    or any(values and len(values) != expected_len
                           for values in (point.velocities, point.effort, point.accelerations))):
                raise ValueError('轨迹通道长度不匹配')
            if point.accelerations:
                raise ValueError('当前 P/V 轨迹入口不支持 acceleration')
            if any(not math.isfinite(e) or e != 0 for e in point.effort):
                raise ValueError('JTC 仅占 P/V；轨迹非零 effort 无执行接口，请使用手动 P/V/E')
            times_ns[index] = t
            previous = t
            P[index, col_idx] = point.positions
            if point.velocities:
                V[index, col_idx] = point.velocities

        lo = np.array([self.limits[n][0] for n in self.joint_names], dtype=np.float64)
        hi = np.array([self.limits[n][1] for n in self.joint_names], dtype=np.float64)
        vl = np.array([self.limits[n][2] for n in self.joint_names], dtype=np.float64)

        if not np.isfinite(P).all():
            i, j = np.argwhere(~np.isfinite(P))[0]
            raise ValueError(f'{self.joint_names[j]} 目标非有限或超过 URDF P/V/E 限位')
        if not np.isfinite(V).all():
            i, j = np.argwhere(~np.isfinite(V))[0]
            raise ValueError(f'{self.joint_names[j]} 目标非有限或超过 URDF P/V/E 限位')

        bad_p = (P < lo) | (P > hi)
        if bad_p.any():
            for i, j in np.argwhere(bad_p):
                bounded = float(lo[j] if P[i, j] < lo[j] else hi[j])
                if not math.isclose(float(P[i, j]), bounded, rel_tol=2**-24, abs_tol=2**-150):
                    raise ValueError(f'{self.joint_names[j]} 目标非有限或超过 URDF P/V/E 限位')
                P[i, j] = bounded
        bad_v = (V < -vl) | (V > vl)
        if bad_v.any():
            for i, j in np.argwhere(bad_v):
                bounded = float(-vl[j] if V[i, j] < -vl[j] else vl[j])
                if not math.isclose(float(V[i, j]), bounded, rel_tol=2**-24, abs_tol=2**-150):
                    raise ValueError(f'{self.joint_names[j]} 目标非有限或超过 URDF P/V/E 限位')
                V[i, j] = bounded

        if times_ns[0] == 0:
            pos_tol = (np.full(J, 1e-7, dtype=np.float64) if linear
                       else np.array([self.position_tolerance[n] for n in self.joint_names], dtype=np.float64))
            if np.any(np.abs(P[0] - m_pos) > pos_tol):
                raise ValueError('零时刻目标未对齐实测位置，请改用未来路点')
            P[0] = m_pos
            V[0] = m_vel
            for j, name in enumerate(self.joint_names):
                for arr, low, high in ((P, lo[j], hi[j]), (V, -vl[j], vl[j])):
                    val = float(arr[0, j])
                    if not math.isfinite(val):
                        raise ValueError(f'{name} 目标非有限或超过 URDF P/V/E 限位')
                    if not low <= val <= high:
                        bounded = low if val < low else high
                        if not math.isclose(val, bounded, rel_tol=2**-24, abs_tol=2**-150):
                            raise ValueError(f'{name} 目标非有限或超过 URDF P/V/E 限位')
                        arr[0, j] = bounded

        for group_name, (joints, lower, step, mask) in self.workspace.groups.items():
            j0, j1 = joint_index[joints[0]], joint_index[joints[1]]
            moving = (P[:, j0] != m_pos[j0]) | (P[:, j1] != m_pos[j1]) | (V[:, j0] != 0.0) | (V[:, j1] != 0.0)
            if moving.any():
                c0 = (P[moving, j0] - float(lower[0])) / step
                c1 = (P[moving, j1] - float(lower[1])) / step
                if not (np.isfinite(c0).all() and np.isfinite(c1).all()):
                    raise ValueError(f'{group_name} 两轴组合超出 CAD 可动区域，请先将另一轴移回零位附近')
                lo0 = np.floor(c0 + 1e-8).astype(int)
                hi0 = np.ceil(c0 - 1e-8).astype(int)
                lo1 = np.floor(c1 + 1e-8).astype(int)
                hi1 = np.ceil(c1 - 1e-8).astype(int)
                h0, h1 = mask.shape
                if ((lo0 < 0).any() or (lo1 < 0).any() or (hi0 >= h0).any() or (hi1 >= h1).any()
                        or not (mask[lo0, lo1] & mask[lo0, hi1] & mask[hi0, lo1] & mask[hi0, hi1]).all()):
                    raise ValueError(f'{group_name} 两轴组合超出 CAD 可动区域，请先将另一轴移回零位附近')

        for i in range(N):
            self.model.joint_to_actuator(
                {name: Axis(float(P[i, j]), float(V[i, j])) for j, name in enumerate(self.joint_names)})

        P_prev = np.vstack([m_pos[None, :], P[:-1]])
        V_prev = np.vstack([m_vel[None, :], V[:-1]])
        dt_arr = np.diff(np.concatenate(([0], times_ns))) / 1e9
        nz = dt_arr > 0
        if nz.any():
            p0, p1 = P_prev[nz], P[nz]
            v0, v1 = V_prev[nz], V[nz]
            dt = dt_arr[nz, None]
            if linear:
                bad_pos = (p0 < lo) | (p0 > hi) | (p1 < lo) | (p1 > hi)
                if bad_pos.any():
                    _, j = np.argwhere(bad_pos)[0]
                    raise ValueError(f'{self.joint_names[j]} 线性插值越过位置限位')
                speed = (p1 - p0) / dt
                bad_vel = ~np.isfinite(speed) | (np.abs(speed) > vl + 1e-7)
                if bad_vel.any():
                    _, j = np.argwhere(bad_vel)[0]
                    raise ValueError(f'{self.joint_names[j]} 线性插值越过速度限位')
            else:
                a = 2.0 * p0 - 2.0 * p1 + dt * (v0 + v1)
                b = -3.0 * p0 + 3.0 * p1 - dt * (2.0 * v0 + v1)
                c = dt * v0
                lo_tol, hi_tol = lo - 1e-7, hi + 1e-7
                bad_pos = (p0 < lo_tol) | (p0 > hi_tol) | (p1 < lo_tol) | (p1 > hi_tol)
                has_a = np.abs(a) > 1e-14
                disc = 4.0 * b * b - 12.0 * a * c
                has_roots = has_a & (disc >= 0.0)
                sqrt_disc = np.sqrt(np.where(has_roots, disc, 0.0))
                denom_a = np.where(has_a, 6.0 * a, 1.0)
                for s in ((-2.0 * b + sqrt_disc) / denom_a, (-2.0 * b - sqrt_disc) / denom_a):
                    in_seg = has_roots & (s >= 0.0) & (s <= 1.0)
                    if in_seg.any():
                        pos_s = ((a * s + b) * s + c) * s + p0
                        bad_pos |= in_seg & ((pos_s < lo_tol) | (pos_s > hi_tol))
                has_b = (~has_a) & (np.abs(b) > 1e-14)
                sb = -c / np.where(has_b, 2.0 * b, 1.0)
                in_seg_b = has_b & (sb >= 0.0) & (sb <= 1.0)
                if in_seg_b.any():
                    pos_sb = ((a * sb + b) * sb + c) * sb + p0
                    bad_pos |= in_seg_b & ((pos_sb < lo_tol) | (pos_sb > hi_tol))
                if bad_pos.any():
                    _, j = np.argwhere(bad_pos)[0]
                    raise ValueError(f'{self.joint_names[j]} JTC 插值越过位置限位')

                v_tol = vl + 1e-7
                bad_vel = (np.abs(v0) > v_tol) | (np.abs(v1) > v_tol)
                sv = -b / np.where(has_a, 3.0 * a, 1.0)
                in_seg_v = has_a & (sv >= 0.0) & (sv <= 1.0)
                if in_seg_v.any():
                    vel_sv = np.abs((3.0 * a * sv * sv + 2.0 * b * sv + c) / dt)
                    bad_vel |= in_seg_v & (vel_sv > v_tol)
                if bad_vel.any():
                    _, j = np.argwhere(bad_vel)[0]
                    raise ValueError(f'{self.joint_names[j]} JTC 插值越过速度限位')

            dt1 = dt_arr[nz]
            for group, (joints, lower, step, mask) in self.workspace.groups.items():
                j0, j1 = joint_index[joints[0]], joint_index[joints[1]]
                o0, o1 = float(lower[0]), float(lower[1])
                h0, h1 = mask.shape
                x0, y0 = P_prev[nz, j0], P_prev[nz, j1]
                x3, y3 = P[nz, j0], P[nz, j1]
                if linear:
                    x1, y1, x2, y2 = x0, y0, x3, y3
                else:
                    x1 = x0 + dt1 * V_prev[nz, j0] / 3.0
                    y1 = y0 + dt1 * V_prev[nz, j1] / 3.0
                    x2 = x3 - dt1 * V[nz, j0] / 3.0
                    y2 = y3 - dt1 * V[nz, j1] / 3.0
                x_min = np.minimum(np.minimum(x0, x1), np.minimum(x2, x3))
                x_max = np.maximum(np.maximum(x0, x1), np.maximum(x2, x3))
                y_min = np.minimum(np.minimum(y0, y1), np.minimum(y2, y3))
                y_max = np.maximum(np.maximum(y0, y1), np.maximum(y2, y3))
                lo0 = np.floor((x_min - o0) / step + 1e-8).astype(int)
                hi0 = np.ceil((x_max - o0) / step - 1e-8).astype(int)
                lo1 = np.floor((y_min - o1) / step + 1e-8).astype(int)
                hi1 = np.ceil((y_max - o1) / step - 1e-8).astype(int)
                for k in range(len(dt1)):
                    if (x0[k] == x3[k] == x1[k] == x2[k]
                            and y0[k] == y3[k] == y1[k] == y2[k]):
                        continue
                    if (0 <= lo0[k] <= hi0[k] < h0 and 0 <= lo1[k] <= hi1[k] < h1
                            and mask[lo0[k]:hi0[k] + 1, lo1[k]:hi1[k] + 1].all()):
                        continue
                    pending = [(((float(x0[k]), float(y0[k])), (float(x1[k]), float(y1[k])),
                                 (float(x2[k]), float(y2[k])), (float(x3[k]), float(y3[k]))), 0)]
                    while pending:
                        controls, depth = pending.pop()
                        c0, c1, c2, c3 = controls
                        bx_min = min(c0[0], c1[0], c2[0], c3[0])
                        bx_max = max(c0[0], c1[0], c2[0], c3[0])
                        by_min = min(c0[1], c1[1], c2[1], c3[1])
                        by_max = max(c0[1], c1[1], c2[1], c3[1])
                        blo0 = math.floor((bx_min - o0) / step + 1e-8)
                        bhi0 = math.ceil((bx_max - o0) / step - 1e-8)
                        blo1 = math.floor((by_min - o1) / step + 1e-8)
                        bhi1 = math.ceil((by_max - o1) / step - 1e-8)
                        if (0 <= blo0 <= bhi0 < h0 and 0 <= blo1 <= bhi1 < h1
                                and mask[blo0:bhi0 + 1, blo1:bhi1 + 1].all()):
                            continue
                        if depth == 12:
                            raise ValueError(f'{group} JTC 插值两轴组合超出 CAD 可动区域')
                        m01x, m01y = (c0[0] + c1[0]) * 0.5, (c0[1] + c1[1]) * 0.5
                        m12x, m12y = (c1[0] + c2[0]) * 0.5, (c1[1] + c2[1]) * 0.5
                        m23x, m23y = (c2[0] + c3[0]) * 0.5, (c2[1] + c3[1]) * 0.5
                        q01x, q01y = (m01x + m12x) * 0.5, (m01y + m12y) * 0.5
                        q12x, q12y = (m12x + m23x) * 0.5, (m12y + m23y) * 0.5
                        mid = ((q01x + q12x) * 0.5, (q01y + q12y) * 0.5)
                        pending.append(((mid, (q12x, q12y), (m23x, m23y), c3), depth + 1))
                        pending.append(((c0, (m01x, m01y), (q01x, q01y), mid), depth + 1))

        output = JointTrajectory(joint_names=self.joint_names)
        output.header.frame_id = message.header.frame_id
        P_list = P.tolist()
        V_list = [] if linear else V.tolist()
        times_list = times_ns.tolist()
        output.points = [
            JointTrajectoryPoint(
                positions=P_list[i],
                velocities=[] if linear else V_list[i],
                time_from_start=Duration(sec=t // 10**9, nanosec=t % 10**9),
            )
            for i, t in enumerate(times_list)
        ]
        return output

    def _interval(self, before, after, dt, *, linear=False):
        # JTC P/V 使用三次 Hermite；端点合法还不足以保证段内位置/速度合法。
        for name in self.joint_names:
            p, q = before[name], after[name]
            if dt == 0:
                if abs(p.position - q.position) > 1e-7 or abs(p.velocity - q.velocity) > 1e-7:
                    raise ValueError('零时刻目标必须与实测 P/V 连续')
                continue
            if linear:
                lower, upper, velocity, _ = self.limits[name]
                if not lower <= p.position <= upper or not lower <= q.position <= upper:
                    raise ValueError(f'{name} 线性插值越过位置限位')
                speed = (q.position - p.position)/dt
                if not math.isfinite(speed) or abs(speed) > velocity+1e-7:
                    raise ValueError(f'{name} 线性插值越过速度限位')
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
            lo_tol, hi_tol = lower - 1e-7, upper + 1e-7
            p0 = p.position
            for s in position_times:
                if 0 <= s <= 1:
                    pos = ((a*s + b)*s + c)*s + p0
                    if not lo_tol <= pos <= hi_tol:
                        raise ValueError(f'{name} JTC 插值越过位置限位')
            velocity_times = [0., 1.] + ([-b/(3*a)] if abs(a) > 1e-14 else [])
            v_tol = velocity + 1e-7
            for s in velocity_times:
                if 0 <= s <= 1:
                    if abs((3*a*s*s + 2*b*s + c)/dt) > v_tol:
                        raise ValueError(f'{name} JTC 插值越过速度限位')
        if dt > 0:
            self._workspace_interval(before, after, dt, linear=linear)

    def _workspace_interval(self, before, after, dt, *, linear):
        for group, (joints, lower, step, mask) in self.workspace.groups.items():
            j0, j1 = joints
            o0, o1 = float(lower[0]), float(lower[1])
            h0, h1 = mask.shape
            start = (before[j0].position, before[j1].position)
            end = (after[j0].position, after[j1].position)
            control_start = start if linear else (
                before[j0].position + dt * before[j0].velocity / 3,
                before[j1].position + dt * before[j1].velocity / 3)
            control_end = end if linear else (
                after[j0].position - dt * after[j0].velocity / 3,
                after[j1].position - dt * after[j1].velocity / 3)
            if start == end == control_start == control_end:
                continue
            # Hermite 转 Bezier 后，控制点的包围框包含整段曲线。
            # 仅当框内全部网格有效才放行，二分收紧框，无法证明合法则拒绝。
            pending = [((start, control_start, control_end, end), 0)]
            while pending:
                controls, depth = pending.pop()
                c0, c1, c2, c3 = controls
                x_min = min(c0[0], c1[0], c2[0], c3[0])
                x_max = max(c0[0], c1[0], c2[0], c3[0])
                y_min = min(c0[1], c1[1], c2[1], c3[1])
                y_max = max(c0[1], c1[1], c2[1], c3[1])
                lo0 = math.floor((x_min - o0) / step + 1e-8)
                hi0 = math.ceil((x_max - o0) / step - 1e-8)
                lo1 = math.floor((y_min - o1) / step + 1e-8)
                hi1 = math.ceil((y_max - o1) / step - 1e-8)
                if (0 <= lo0 <= hi0 < h0 and 0 <= lo1 <= hi1 < h1
                        and mask[lo0:hi0 + 1, lo1:hi1 + 1].all()):
                    continue
                if depth == 12:
                    raise ValueError(f'{group} JTC 插值两轴组合超出 CAD 可动区域')
                m01x, m01y = (c0[0] + c1[0]) * 0.5, (c0[1] + c1[1]) * 0.5
                m12x, m12y = (c1[0] + c2[0]) * 0.5, (c1[1] + c2[1]) * 0.5
                m23x, m23y = (c2[0] + c3[0]) * 0.5, (c2[1] + c3[1]) * 0.5
                q01x, q01y = (m01x + m12x) * 0.5, (m01y + m12y) * 0.5
                q12x, q12y = (m12x + m23x) * 0.5, (m12y + m23y) * 0.5
                rx, ry = (q01x + q12x) * 0.5, (q01y + q12y) * 0.5
                mid = (rx, ry)
                pending.append(((mid, (q12x, q12y), (m23x, m23y), c3), depth + 1))
                pending.append(((c0, (m01x, m01y), (q01x, q01y), mid), depth + 1))

    def motor_trajectory(self, plan, measured, now_ns, *, linear=False):
        fresh(plan.sync_time_ns, now_ns)
        message = JointTrajectory(joint_names=self.joint_names)
        frames = {}
        for point in plan.points:
            frames.setdefault(point.time_from_start_ms, []).append(point)
        times = list(frames)
        if (any(not 0 <= t <= 0xFFFFFFFF for t in times)
                or any(b <= a for a, b in zip(times, times[1:]))):
            raise ValueError('轨迹时间必须非负、严格递增且可由状态毫秒表示')
        for t, points in frames.items():
            if any(not math.isfinite(p.velocity) for p in points):
                raise ValueError('轨迹诊断速度包含非有限值')
            # MotorTrajectory 的 velocity 仅诊断；先映射位置，再在关节空间生成切矢。
            targets = [MotorCommandTarget(motor_id=p.motor_id, position=p.position,
                                         velocity=0., effort=p.effort) for p in points]
            values = self._complete(self._motors(targets), measured)
            message.points.append(JointTrajectoryPoint(
                positions=[values[n].position for n in self.joint_names],
                velocities=[values[n].velocity for n in self.joint_names],
                effort=[values[n].effort for n in self.joint_names],
                time_from_start=Duration(sec=t // 1000, nanosec=t % 1000 * 10**6)))
        if linear:
            return self.joint_trajectory(message, measured, now_ns, linear=True)
        for index in range(1, len(message.points) - 1):
            before, point, after = message.points[index-1:index+2]
            dt0, dt1 = (times[index] - times[index-1])/1000., (times[index+1] - times[index])/1000.
            w1, w2 = 2*dt1 + dt0, dt1 + 2*dt0
            for joint in range(len(self.joint_names)):
                d0 = (point.positions[joint] - before.positions[joint])/dt0
                d1 = (after.positions[joint] - point.positions[joint])/dt1
                if not math.isfinite(d0) or not math.isfinite(d1):
                    raise ValueError('轨迹差分速度非有限')
                if d0*d1 > 0.:
                    limit = 3*min(abs(d0), abs(d1))
                    point.velocities[joint] = max(-limit, min(limit, (w1+w2)/(w1/d0+w2/d1)))
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

    def add(self, message, now_ns, now, *, linear=False):
        key = (message.session_id, message.trajectory_id)
        try:
            fresh(message.sync_time_ns, now_ns)
            if message.segment_count > TRAJECTORY_MAX_POINTS:
                raise ValueError(f'轨迹分段数超过 {TRAJECTORY_MAX_POINTS}')
            if (not 0 < message.count == len(message.points) <= 256
                    or not 0 < message.target_count == len(message.target_ids) <= 64
                    or len(set(message.target_ids)) != message.target_count
                    or not 0 <= message.segment_index < message.segment_count
                    or key in self.closed):
                raise ValueError('轨迹分段数量、ID 或序号无效/重复')
            signature = (key, message.sequence, message.segment_count,
                         tuple(message.target_ids), message.sync_time_ns, linear)
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
            if len(self.pending.points) + len(message.points) > TRAJECTORY_MAX_POINTS:
                raise ValueError(f'轨迹总路点数超过 {TRAJECTORY_MAX_POINTS}')
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
class TrajectoryStateSnapshot:
    trajectory_id: int = 0
    active: bool = False
    completed: bool = False
    elapsed_ms: int = 0
    total_ms: int = 0


@dataclass
class Run:
    state: TrajectoryStateSnapshot
    epoch: int
    handle: object = None
    submitted: bool = False
    result_future: object = None
    raw_linear: bool = False


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
        self._urdf_limits = dict(self.mapping.limits)
        self.declare_parameter('joint_limits_json', '')
        self._limits_confirmed = not bool(self.get_parameter('joint_limits_json').value)
        self._apply_joint_limits()
        self.add_on_set_parameters_callback(self._validate_joint_limits)
        self.motion_names = ['trajectory_controller', 'gravity_controller', 'follow_controller']
        self._epoch, self._id = 0, 0
        self._busy, self._stopping = False, False
        self._stop_confirmed, self._stop_deadline = False, 0.
        self._connected_clients = None
        self._input_floor_ns = 0
        self._stop_policy_id = 0
        self._feedback = None
        self._feedback_received = 0.
        self._pending_follow = None
        self._follow_targets = {}
        self._follow_stamp = 0
        self._last_trajectory_state = TrajectoryStateSnapshot()
        self._waits, self._runs, self._switching = [], [], set()
        self._stop_future = self._poll_future = self._params_future = None
        self._mode, self._reason, self._reject = 'stopped', '等待新目标', ''
        self._gravity_enabled = self._effort_gain = None
        self._active = []
        self._controllers_ready = False
        self._last_dynamic_check = 0.0
        self._last_status_json = None
        self._last_limit_errors = None
        self._last_dynamic_limits = None
        self._last_publish_mono = 0.0
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._status_pub = self.create_publisher(String, '/control/status', latched)
        self._transmission_pub = self.create_publisher(String, '/robot_transmission/status', latched)
        self._stop_pub = self.create_publisher(Empty, '/control/stop', 10)
        self._internal_stop_emitted = False

        self._joint_pub = self.create_publisher(JointState, '/joint_states', 10) if publish_states else None
        self._follow_pub = self.create_publisher(JointState, '/host_control/follow_controller/target', 1)
        self._list = self.create_client(ListControllers, '/host_control/controller_manager/list_controllers')
        self._switch = self.create_client(SwitchController, '/host_control/controller_manager/switch_controller')
        self._params = self.create_client(GetParameters, '/host_control/gravity_controller/get_parameters')
        self._action = ActionClient(self, FollowJointTrajectory, ACTION)
        self._feedback_lock = threading.Lock()
        self._feedback_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(
            JointState, '/host_control/joint_states', self._on_feedback,
            qos_profile_sensor_data, callback_group=self._feedback_group)
        for kind, topic, callback, qos in (
            (JointState, '/control/joint_target', self._on_joint_target, 1),
            (JointTrajectory, '/control/trajectory', self._on_joint_trajectory, 10),
            (Empty, '/control/stop', self._on_stop_topic, 10),
            (Int32, '/client_count', self._on_client_count, latched),
            (DiagnosticArray, '/host_control/follow_controller/diagnostics',
             self._on_follow_diagnostics, 10),
        ):
            self.create_subscription(kind, topic, self._locked(callback), qos)
        self.create_timer(.05, self._locked(self._tick))
        self.create_timer(1., self._locked(self._poll))
        self._publish_status()

    def _joint_limits(self, value):
        limits = parse_joint_limits(value, self.model_name,
            {name: bounds[:2] for name, bounds in self._urdf_limits.items()})
        if any(limits[name] != self._urdf_limits[name][:2]
               for name in self.mapping.dynamic_joint_names):
            raise ValueError('耦合关节的动态限位由模型和姿态计算，不能覆盖为静态限位')
        return limits

    def _sync_hardware_limits(self, limits):
        finite = {name: bounds for name, bounds in limits.items()
                  if all(math.isfinite(value) for value in bounds)}
        if not finite:
            raise ValueError('请先设置有限的关节上下限')
        # 参数回调运行于主执行器；独立临时节点等待硬件确认，避免回调等待自身而死锁。
        client_node = Node('workbench_limit_update', context=self.context,
                           use_global_arguments=False, enable_rosout=False,
                           start_parameter_services=False)
        executor = SingleThreadedExecutor(context=self.context)
        executor.add_node(client_node)
        try:
            client = client_node.create_client(SetParametersAtomically,
                '/micro_ros_system_micro_ros_system/set_parameters_atomically')
            if not client.wait_for_service(timeout_sec=RPC_TIMEOUT):
                raise ValueError('主机硬件限位服务不可达，请重建控制会话')
            request = SetParametersAtomically.Request(parameters=[
                Parameter('position_limit_names', value=list(finite)).to_parameter_msg(),
                Parameter('position_limit_bounds', value=[float(value)
                    for bounds in finite.values() for value in bounds]).to_parameter_msg()])
            future = client.call_async(request)
            executor.spin_until_future_complete(future, timeout_sec=RPC_TIMEOUT)
            if not future.done() or future.exception() is not None:
                self._limits_confirmed = False
                raise ValueError('主机硬件限位更新未确认，暂不能控制；请重新提交限位')
            reply = future.result().result
            if not reply.successful:
                raise ValueError(reply.reason or '主机硬件拒绝更新限位')
            self._limits_confirmed = True
        finally:
            executor.shutdown()
            client_node.destroy_node()

    def _validate_joint_limits(self, parameters):
        with self._lock:
            try:
                for parameter in parameters:
                    if parameter.name == 'joint_limits_json':
                        if len(parameters) != 1:
                            raise ValueError('关节限位需单独原子提交')
                        if self._busy or self._runs or self._active:
                            raise ValueError('请先停止控制器，再修改关节限位')
                        self._sync_hardware_limits(self._joint_limits(parameter.value))
            except ValueError as error:
                return SetParametersResult(successful=False, reason=str(error))
            return SetParametersResult(successful=True)

    def _apply_joint_limits(self):
        limits = self._joint_limits(self.get_parameter('joint_limits_json').value)
        self.mapping.limits = {name: (*bounds, *self._urdf_limits[name][2:])
                               for name, bounds in limits.items()}

    def _locked(self, callback):
        def invoke(*args):
            with self._lock:
                return callback(*args)
        return invoke

    def _measured(self):
        if not self._limits_confirmed:
            raise ValueError('主机硬件限位更新未确认，请重新提交限位')
        self._apply_joint_limits()
        with self._feedback_lock:
            feedback, received = self._feedback, self._feedback_received
        if feedback is None or time.monotonic() - received > FRESH_NS / 10**9:
            raise ValueError('缺少新鲜关节反馈')
        now_ns = self.get_clock().now().nanoseconds
        try:
            return self.mapping.feedback(feedback, now_ns)
        except ValueError as error:
            if str(error) == '输入或反馈时间戳无效、过期或来自未来':
                fb_stamp = stamp_ns(feedback.header.stamp)
                self.get_logger().warning(
                    f'feedback 时间戳拒绝: stamp={fb_stamp} now={now_ns} '
                    f'diff_ms={(now_ns - fb_stamp) / 1e6:+.1f}')
            raise

    def _on_feedback(self, message):
        now = time.monotonic()
        with self._feedback_lock:
            self._feedback, self._feedback_received = message, now
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

    def _state(self, trajectory_id, total_ms=0):
        return TrajectoryStateSnapshot(trajectory_id=trajectory_id, total_ms=total_ms)

    def _clear_follow(self):
        self._pending_follow = None
        self._follow_targets = {}
        self._follow_stamp = 0

    def _build_follow(self, measured, now):
        if self._pending_follow is None:
            raise ValueError('连续目标已取消')
        return self.mapping.joint_target(self._pending_follow, measured, now)

    def _publish_follow(self, output):
        if not self._follow_pub.get_subscription_count():
            raise ValueError('连续跟随控制器订阅不可达')
        output.header.stamp = self.get_clock().now().to_msg()
        self._follow_pub.publish(output)
        self._pending_follow = None

    def _on_joint_target(self, message):
        try:
            if self._connected_clients == 0 or self._stopping or (
                    self._busy and self._pending_follow is None):
                raise ValueError('客户端归零或其他控制切换尚未完成')
            stamp = stamp_ns(message.header.stamp)
            if stamp < self._input_floor_ns or stamp <= self._follow_stamp:
                raise ValueError('连续目标早于停止或已接收目标')
            output = self.mapping.joint_target(message, self._measured(),
                self.get_clock().now().nanoseconds, self._follow_targets)
        except ValueError as error:
            self._reject_input(error)
            return
        self._pending_follow = output
        self._follow_targets = {name: Axis(value) for name, value in zip(output.name, output.position)}
        self._follow_stamp = stamp
        if self._busy:
            return  # The in-flight switch will publish only this latest admitted target.
        if self._mode == 'follow':
            try:
                self._publish_follow(output)
                self._reject = ''
            except ValueError as error:
                self._fail(str(error))
        elif not self._submit('follow', self._build_follow):
            self._clear_follow()

    def _on_joint_trajectory(self, message):
        if not message.joint_names and not message.points:
            self._cancel()
            return
        identity = message.header.frame_id
        raw_linear = False
        trajectory_id = self._id % 0xFFFFFFFF + 1
        for prefix, is_linear in (('workbench-trajectory-linear/', True), ('workbench-trajectory/', False)):
            if identity.startswith(prefix):
                value = identity.removeprefix(prefix)
                if (not 1 <= len(value) <= 10 or not value.isascii() or not value.isdigit()
                        or not 0 < int(value) <= 0xFFFFFFFF):
                    self._reject_input('关节轨迹播放编号必须为非零 uint32')
                    return
                trajectory_id = int(value)
                raw_linear = is_linear
                if any(run.state.trajectory_id == trajectory_id for run in self._runs):
                    self._reject_input('关节轨迹播放编号正在使用')
                    return
                break
        self._id = trajectory_id
        state = self._state(trajectory_id)
        if not stamp_ns(message.header.stamp) or stamp_ns(message.header.stamp) < self._input_floor_ns:
            self._reject_input('轨迹早于最近一次停止', state)
            return
        received_at_ns = self.get_clock().now().nanoseconds
        in_stamp = stamp_ns(message.header.stamp)
        cached = None

        def build(measured, _now):
            nonlocal cached
            if cached is not None:
                output, first_point_values, first_dt_s = cached
                self.mapping._interval(measured, first_point_values, first_dt_s, linear=False)
                return output
            try:
                output = self.mapping.joint_trajectory(message, measured, received_at_ns)
            except ValueError as error:
                if str(error) == '输入或反馈时间戳无效、过期或来自未来':
                    self.get_logger().warning(
                        f'trajectory input 时间戳拒绝: stamp={in_stamp} now={received_at_ns} '
                        f'diff_ms={(received_at_ns - in_stamp) / 1e6:+.1f}')
                raise
            if (set(message.joint_names) == set(self.mapping.joint_names)
                    and output.points
                    and stamp_ns(output.points[0].time_from_start) > 0):
                pt0 = output.points[0]
                first_point_values = {
                    name: Axis(pt0.positions[i], pt0.velocities[i] if pt0.velocities else 0.)
                    for i, name in enumerate(output.joint_names)
                }
                first_dt_s = stamp_ns(pt0.time_from_start) / 10**9
                cached = (output, first_point_values, first_dt_s)
            return output

        self._submit('trajectory', build, state, raw_linear=raw_linear)

    def _submit(self, mode, build, state=None, *, raw_linear=False):
        try:
            if self._connected_clients == 0 or self._stopping or self._busy:
                raise ValueError('客户端归零或控制切换尚未完成')
            result = build(self._measured(), self.get_clock().now().nanoseconds)
            if state:
                state.total_ms = stamp_ns(result.points[-1].time_from_start) // 10**6
            if not self._list.service_is_ready() or not self._switch.service_is_ready():
                raise RuntimeError('controller manager 服务不可达')
            if mode == 'trajectory' and not self._action.server_is_ready():
                raise RuntimeError('JTC action 服务不可达')
        except ValueError as error:
            self._reject_input(error, state)
            return False
        except RuntimeError as error:
            self._reject_input(error, state)
            self._stop(str(error), str(error))
            return False
        if mode != 'follow':
            self._clear_follow()
        self._epoch += 1
        epoch = self._epoch
        self._busy, self._mode, self._reason, self._reject = True, 'switching', '校验并切换控制器', ''
        run = Run(state, epoch, raw_linear=raw_linear) if state else None
        if run:
            self._runs.append(run)
        def ready():
            self._watch(self._list.call_async(ListControllers.Request()), epoch, '查询控制器',
                lambda response: self._prepare(response, mode, build, run, epoch))
        self._cancel_handles(epoch, ready)
        self._publish_status()
        return True

    def _controllers(self, response, mode):
        controllers = {c.name: c for c in response.controller}
        if not set(self.motion_names).issubset(controllers):
            raise ValueError('controller manager 缺少配置的运动控制器')
        desired = (['follow_controller', 'gravity_controller'] if mode == 'follow'
                   else ['trajectory_controller', 'gravity_controller'])
        for name in desired:
            c = controllers[name]
            if c.state not in ('active', 'inactive'):
                raise ValueError(f'{name} 未配置或不可切换')
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
        if mode == 'follow':
            self._publish_follow(output)
            self._busy, self._mode, self._reason = False, mode, '连续目标跟随中'
        else:
            if not self._action.server_is_ready():
                raise ValueError('JTC action 服务不可达')
            run.submitted = True
            goal = FollowJointTrajectory.Goal(trajectory=output)
            if run.raw_linear:
                goal.path_tolerance = [JointTolerance(name=name, position=.2)
                                       for name in output.joint_names]
            future = self._action.send_goal_async(goal,
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
        run.result_future = handle.get_result_async()
        if run.epoch == self._epoch:
            self._busy, self._mode, self._reason = False, 'trajectory', 'JTC goal 已受理'
            run.state.active = True
            self._receipt(run.state)
            self._publish_status()
        self._watch(run.result_future, None, 'JTC 执行结果',
            lambda result: self._action_result(run, result),
            timeout=run.state.total_ms/1000 + RPC_TIMEOUT + 2.,
            late=lambda result: self._action_result(run, result),
            on_error=lambda reason: self._goal_error(run, reason))
        if run.epoch != self._epoch and run in self._runs:
            self._cancel_one(run, self._epoch)

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
        terminal = (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED)
        def failed(reason):
            if self._stopping:
                self._stop_error(reason)
            else:
                self._fail(reason)
        def finished(result):
            if result is None or result.status not in terminal:
                raise ValueError('JTC 取消终态未确认')
            done()
        def canceled(response):
            # 服务端可能已终结，客户端 status/结果尚未到达；等同一结果 Future，超时仍停止。
            if not response.goals_canceling and run.handle.status not in terminal:
                self._watch(run.result_future, epoch, '等待 JTC 取消终态', finished,
                            on_error=failed)
                return
            done()
        self._watch(run.handle.cancel_goal_async(), epoch, '取消 JTC goal', canceled,
                    on_error=failed)

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
        if self._mode == 'follow':
            self._stop('取消连续目标')
            return
        if self._busy or self._stopping:
            self._stop('切换期间取消轨迹')
            return
        self._epoch += 1
        self._busy = True
        def done():
            self._busy, self._reason = False, '已请求 JTC 取消并保持'
            self._publish_status()
        self._cancel_handles(self._epoch, done)

    def _on_stop_topic(self, _message):
        if self._internal_stop_emitted:
            self._internal_stop_emitted = False
            return
        self._stop('显式停止', broadcast=False)

    def _stop(self, reason, reject='', *, broadcast=True):
        self._clear_follow()
        self._input_floor_ns = self.get_clock().now().nanoseconds + 50_000_000
        if not self._stopping:
            self._stop_deadline = time.monotonic() + RPC_TIMEOUT
        self._stop_confirmed = False
        self._epoch += 1
        if broadcast:
            self._internal_stop_emitted = True
            self._stop_pub.publish(Empty())
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

    def _on_follow_diagnostics(self, message):
        stamp = stamp_ns(message.header.stamp)
        reason = next((status.message for status in message.status
                       if status.level == DiagnosticStatus.ERROR and status.message), '')
        if self._mode != 'follow' and self._pending_follow is None:
            return
        if self._stopping or not reason:
            return
        if stamp <= 0 or stamp < self._input_floor_ns or stamp > (
                self.get_clock().now().nanoseconds + 50_000_000):
            return
        self._fail('跟随控制器故障：' + reason)

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
        if self.mapping.dynamic_joint_names and now - self._last_dynamic_check >= DYNAMIC_STATUS_PERIOD:
            self._last_dynamic_check = now
            self._publish_status(only_if_changed=True)

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
                if self._mode in ('follow', 'trajectory') and set(self._active) != {
                        self._mode + '_controller', 'gravity_controller'}:
                    reason = ('连续跟随' if self._mode == 'follow' else '轨迹') + '控制器意外停止，请检查反馈或控制器日志'
                    self._stop(reason, reason)
                    return
                if not self._active:
                    self._mode = 'stopped'
                    self._clear_follow()
                elif set(self._active) == {'follow_controller', 'gravity_controller'}:
                    self._mode = 'follow'
                elif set(self._active) == {'trajectory_controller', 'gravity_controller'}:
                    self._mode = 'trajectory'
                else:
                    raise ValueError('检测到非互斥的 active 控制器集合')
                self._publish_status(only_if_changed=True)
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
                self._publish_status(only_if_changed=True)
            self._watch(self._params_future, None, '读取重力配置', parameters,
                        on_error=lambda _: self._unknown_parameters())
        elif not self._params.service_is_ready():
            self._unknown_parameters()
        self._publish_status()

    def _unknown_parameters(self):
        self._gravity_enabled = self._effort_gain = None

    def _receipt(self, state):
        self._last_trajectory_state = TrajectoryStateSnapshot(
            trajectory_id=state.trajectory_id,
            active=state.active,
            completed=state.completed,
            elapsed_ms=state.elapsed_ms,
            total_ms=state.total_ms,
        )

    def _dynamic_limits_changed(self, limits):
        """Return True if any coupled dynamic joint bound shifted beyond deadband."""
        if self._last_dynamic_limits is None:
            return True
        for name in self.mapping.dynamic_joint_names:
            prev = self._last_dynamic_limits.get(name)
            curr = limits.get(name)
            if prev is None or curr is None:
                return True
            for p_val, c_val in zip(prev, curr):
                if math.isnan(p_val) != math.isnan(c_val):
                    return True
                if not math.isnan(c_val) and abs(c_val - p_val) > DYNAMIC_LIMIT_EPS:
                    return True
        return False

    def _publish_status(self, *, only_if_changed=False):
        feedback_error = ''
        try:
            measured = self._measured()
            feedback_fresh = True
        except ValueError as error:
            feedback_fresh = False
            feedback_error = str(error)
        limits = {name: bounds[:2] for name, bounds in self.mapping.limits.items()}
        limit_errors = {}
        positions = {name: axis.position for name, axis in
                     (self._follow_targets or measured).items()} if feedback_fresh else {}
        for group, (joints, _, _, _) in self.mapping.workspace.groups.items():
            try:
                if not feedback_fresh:
                    raise ValueError(feedback_error)
                active = self.mapping.active_limits(positions, groups=[group])
                limits.update({name: active[name] for name in joints})
            except ValueError as error:
                limits.update(dict.fromkeys(joints, (math.nan, math.nan)))
                limit_errors.update(dict.fromkeys(joints, str(error)))
        traj = self._last_trajectory_state
        status = dict(model=self.model_name, joint_names=self.mapping.joint_names,
            mode=self._mode, reason=self._reason, reject=self._reject,
            output_topic='/motor/command', trajectory_output_topic=ACTION,
            gravity_enabled=self._gravity_enabled, effort_gain=self._effort_gain,
            active_controllers=self._active, feedback_fresh=feedback_fresh,
            controllers_ready=self._controllers_ready,
            joint_trajectory_ids=True,
            pending_goal_ids=[run.state.trajectory_id for run in self._runs if run.submitted],
            trajectory_state=dict(
                trajectory_id=traj.trajectory_id,
                active=traj.active,
                completed=traj.completed,
                elapsed_ms=traj.elapsed_ms,
                total_ms=traj.total_ms,
            ),
            stop_policy_request_id=self._stop_policy_id,
            safety_note=('已请求主机停止；MCU 撤权由桥接层确认，恢复运动前需显式重新使能'
                         if self._stop_policy_id else ''))
        status_json = json.dumps(status, ensure_ascii=False)
        dynamic_limits = {name: limits[name] for name in self.mapping.dynamic_joint_names}
        now_mono = time.monotonic()
        if (only_if_changed and status_json == self._last_status_json
                and limit_errors == self._last_limit_errors
                and not self._dynamic_limits_changed(dynamic_limits)
                and now_mono - self._last_publish_mono < 1.0):
            return
        self._last_status_json = status_json
        self._last_limit_errors = dict(limit_errors)
        self._last_dynamic_limits = dynamic_limits
        self._last_publish_mono = now_mono
        self._status_pub.publish(String(data=status_json))
        self._transmission_pub.publish(String(data=build_transmission_status(
            model=self.model_name,
            driver_file=self.config_file,
            trajectory_enabled=True,
            control_state_source='ros2_control',
            control_service=self.get_fully_qualified_name() + '/set_parameters_atomically',
            joint_names=self.mapping.joint_names,
            joint_limits=limits,
            last_error=self._reject,
            extra={**status,
                   'control_groups': [{
                       'name': group.name,
                       'joints': list(group.joints),
                       'motors': [{'id': self.mapping.model.actuator_ids[name]}
                                  for name in group.actuators],
                   } for group in self.mapping.model.groups],
                   'policy_defaults': self.mapping.policy_defaults,
                   'dynamic_joint_names': self.mapping.dynamic_joint_names,
                   'joint_limit_error': '；'.join(dict.fromkeys(limit_errors.values())),
                   'joint_limit_errors': limit_errors,
                   'joint_velocity_limits': {name: bounds[2] for name, bounds in self.mapping.limits.items()},
                   'trajectory_topic': '/control/trajectory'},
        )))


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = WorkbenchControl()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor:
            executor.shutdown()
        if node:
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            node._action.destroy()
            node.destroy_node()
        rclpy.try_shutdown()
