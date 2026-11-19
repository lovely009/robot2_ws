"""传动组反馈换算，使用主机单调时间和 MCU 报文年龄。"""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from robot_transmission import Axis, Model
from .topic_bridge import BridgeError


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
            if not all(a in fresh for a in group.actuators):
                continue
            try:
                result.update(self.model.actuator_to_joint({a: fresh[a] for a in group.actuators}))
            except (ValueError, RuntimeError):
                for actuator in group.actuators:
                    self.samples.pop(actuator, None)
                    fresh.pop(actuator, None)
        self.state_stamp = min((self.samples[a][0] for a in fresh), default=None)
        return result
