"""传动组反馈与轨迹适配；时间以主机单调秒表示，报文年龄来自 MCU。"""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from robot_transmission import Axis, Model
from can_bridge_msgs.msg import WireContract
from .topic_bridge import BridgeError, MAX_TRAJECTORY_MOTORS, MAX_TRAJECTORY_POINTS


def parse_joint_limits(value: str, model_name: str, defaults: dict) -> dict:
    if not isinstance(value, str):
        raise BridgeError('joint_limits_json 必须是 JSON 字符串')
    try:
        data = json.loads(value) if value else {}
    except (ValueError, RecursionError) as exc:
        raise BridgeError('关节限位 JSON 格式无效') from exc
    limits = dict(defaults)
    if not isinstance(data, dict):
        raise BridgeError('关节限位必须是 JSON 对象')
    if not data:
        return limits
    if set(data) != {'model', 'joint_limits'} or data['model'] != model_name:
        raise BridgeError('关节限位模型与主机当前模型不一致或字段无效')
    if not isinstance(data['joint_limits'], dict):
        raise BridgeError('joint_limits 必须是关节范围对象')
    for name, bounds in data['joint_limits'].items():
        if name not in limits:
            raise BridgeError(f'关节 {name} 不属于当前主机传动模型')
        if not isinstance(bounds, dict) or set(bounds) != {'lower', 'upper'}:
            raise BridgeError(f'关节 {name} 限位必须包含 lower 和 upper')
        lower, upper = bounds['lower'], bounds['upper']
        try:
            valid = all(type(v) in (int, float) and math.isfinite(v) for v in (lower, upper))
        except OverflowError:
            valid = False
        if not valid or lower >= upper:
            raise BridgeError(f'关节 {name} 限位必须是有限数值，且下限小于上限')
        limits[name] = (lower, upper)
    return limits


class TransmissionBridge:
    def __init__(self, driver_file: str, transmissions_file: str = '',
                 urdf_file: str = '', feedback_timeout: float = .5,
                 max_skew: float = .05):
        self.model = Model.load(driver_file, transmissions_file)
        self.ids = dict(self.model.actuator_ids)
        if len(set(self.ids.values())) != len(self.ids):
            raise BridgeError('固件传动映射的裸 motor_id 不能重复')
        self.names_by_id = {v: k for k, v in self.ids.items()}
        if not math.isfinite(feedback_timeout) or feedback_timeout <= 0:
            raise BridgeError('feedback_timeout 必须为有限正值')
        if not math.isfinite(max_skew) or max_skew < 0:
            raise BridgeError('max_skew 必须为有限非负值')
        self.timeout, self.max_skew = feedback_timeout, max_skew
        self.samples = {}
        self.held = {}
        self.state_stamp = None
        self.joint_limits = {}
        if urdf_file:
            joints = {j.get('name'): j for j in ET.parse(Path(urdf_file)).getroot().findall('joint')}
            for name in self.model.joint_names:
                joint = joints.get(name)
                if joint is None or joint.get('type') == 'fixed' or joint.find('mimic') is not None:
                    raise BridgeError(f'传动输出 {name} 必须是 URDF 中独立的活动关节')
                limit = joint.find('limit')
                if limit is not None and joint.get('type') != 'continuous':
                    self.joint_limits[name] = (float(limit.get('lower', '-inf')),
                                               float(limit.get('upper', 'inf')))

    def joint_limits_from_json(self, value: str, model_name: str) -> dict:
        return parse_joint_limits(value, model_name,
            {name: self.joint_limits.get(name, (-math.inf, math.inf))
             for name in self.model.joint_names})

    def _fresh(self, now: float) -> dict:
        samples = {name: sample for name, (stamp, sample) in self.samples.items()
                   if 0 <= now-stamp <= self.timeout}
        result = {}
        for group in self.model.groups:
            if not all(a in samples for a in group.actuators):
                continue
            stamps = [self.samples[a][0] for a in group.actuators]
            if max(stamps)-min(stamps) > self.max_skew:
                continue
            result.update({a: samples[a] for a in group.actuators})
        return result

    def update(self, states: list[dict], now: float) -> dict:
        before = self._fresh(now)
        for group in self.model.groups:
            if not all(a in before for a in group.actuators):
                for joint in group.joints:
                    self.held.pop(joint, None)
        for state in states:
            name = self.names_by_id.get(state.get('motor_id'))
            if name is None:
                continue
            values = [state.get(k) for k in ('position', 'velocity', 'effort')]
            age = state.get('state_age_ms', 0xFFFFFFFF)
            if (not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)
                    or not isinstance(age, (int, float)) or not math.isfinite(age)
                    or age < 0 or age/1000 > self.timeout):
                self.samples.pop(name, None)
                continue
            self.samples[name] = (now-age/1000, Axis(*values))
        fresh = self._fresh(now)
        result = {}
        for group in self.model.groups:
            valid = all(a in fresh for a in group.actuators)
            if valid:
                try:
                    result.update(self.model.actuator_to_joint({a: fresh[a] for a in group.actuators}))
                except (ValueError, RuntimeError):
                    valid = False
                    for actuator in group.actuators:
                        self.samples.pop(actuator, None)
                        fresh.pop(actuator, None)
            if not valid:
                for joint in group.joints:
                    self.held.pop(joint, None)
        self.state_stamp = min((self.samples[a][0] for a in fresh), default=None)
        return result

    def trajectory(self, joint_names: list[str], samples: list[dict], now: float,
                   limits: dict | None = None):
        if not joint_names or len(set(joint_names)) != len(joint_names) or not samples:
            raise BridgeError('轨迹关节/路点为空或关节名重复')
        if set(joint_names)-set(self.model.joint_names):
            raise BridgeError('轨迹包含模型未知关节')
        fresh = self.model.actuator_to_joint(self._fresh(now))
        touched = [g for g in self.model.groups if set(g.joints).intersection(joint_names)]
        required = [j for g in touched for j in g.joints]
        if not all(j in fresh for j in required):
            raise BridgeError('传动组未收齐新鲜反馈或组内测量时间差过大')
        held = {j: self.held.get(j, fresh[j].position) for j in required}
        flat, previous = [], -1
        last_targets = {}
        for sample in samples:
            t = sample.get('t_ms')
            if not isinstance(t, int) or isinstance(t, bool) or not previous < t <= 0xFFFFFFFF:
                raise BridgeError('轨迹时间必须为非负、严格递增的整数毫秒')
            previous = t
            channels = {}
            for label in ('positions', 'velocities', 'effort'):
                channel = sample.get(label)
                if label == 'positions' and channel is None:
                    raise BridgeError('轨迹缺少 positions')
                if channel is not None and len(channel) != len(joint_names):
                    raise BridgeError(f'{label} 长度与关节数量不符')
                if channel is not None and not all(isinstance(v, (int, float)) and math.isfinite(v) for v in channel):
                    raise BridgeError(f'{label} 包含非有限值')
                channels[label] = channel
            targets = {j: Axis(held[j]) for j in required}
            for i, name in enumerate(joint_names):
                targets[name] = Axis(channels['positions'][i],
                                     channels['velocities'][i] if channels['velocities'] is not None else 0.,
                                     channels['effort'][i] if channels['effort'] is not None else 0.)
            # shortcut: 未来实际姿态未知，明确设计轨迹前馈或实时固件转换后再扩展。
            for group in touched:
                if group.position_dependent and any(targets[j].effort != 0 for j in group.joints):
                    raise BridgeError('非线性传动的非零轨迹力矩需要实时姿态，整条轨迹拒绝')
            for name, axis in targets.items():
                lower, upper = (limits or self.joint_limits).get(
                    name, self.joint_limits.get(name, (-math.inf, math.inf)))
                if not lower <= axis.position <= upper:
                    raise BridgeError(f'关节 {name} 目标超出主机关节限位 [{lower}, {upper}]，整组拒绝')
            try:
                motor_targets = self.model.joint_to_actuator(targets)
            except (ValueError, RuntimeError) as exc:
                raise BridgeError(str(exc)) from exc
            for actuator, axis in sorted(motor_targets.items(), key=lambda e: self.ids[e[0]]):
                # 线上位置/速度定点量程；越界必须整组拒绝，不能逐电机裁剪。
                if (abs(axis.position) > WireContract.SCALE_POSITION_RAD
                        or abs(axis.velocity) > WireContract.SCALE_VELOCITY_RAD_S
                        or abs(axis.effort) > WireContract.SCALE_EFFORT_NM):
                    raise BridgeError('换算后的执行器目标超过线上量程，整组拒绝')
                flat.append((t, self.ids[actuator], axis.position, axis.velocity, axis.effort))
            last_targets = {j: axis.position for j, axis in targets.items()}
        ids = sorted({p[1] for p in flat})
        if len(ids) > MAX_TRAJECTORY_MOTORS or len(flat) > MAX_TRAJECTORY_POINTS:
            raise BridgeError('成组展开后超过固件轨迹容量')
        self.held.update(last_targets)
        return ids, flat, []
