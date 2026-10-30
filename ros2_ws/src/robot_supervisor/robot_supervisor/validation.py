"""Validate managed motor intents against the installed robot configuration.

Trajectory support deliberately accepts one complete segment with synchronized
samples. Position interpolation follows the MCU's monotone Hermite algorithm;
joint and actuator extrema are checked between samples as well as at samples.
The velocity message field is diagnostic on the MCU, so this first interface
requires it to be zero rather than accepting an apparent velocity command.
"""
from __future__ import annotations

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

from robot_calibration.validation import load_model, resolve_reference
from robot_transmission import Axis


_LINEAR_PLUGINS = {
    'transmission_interface/SimpleTransmission',
    'transmission_interface/DifferentialTransmission',
}
_WIRE_POSITION, _WIRE_VELOCITY, _WIRE_EFFORT = 12.5, 64.0, 64.0
_EPSILON = 1e-6


def _number(value, label: str) -> float:
    try:
        finite = type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        finite = False
    if not finite:
        raise ValueError(f'{label} must be a finite number')
    return float(value)


def _integer(value, label: str, maximum: int) -> int:
    if type(value) is not int or not 0 <= value <= maximum:
        raise ValueError(f'{label} must be an integer in 0..{maximum}')
    return value


def _items(message, field: str, count_field: str, maximum: int) -> list:
    try:
        result = list(getattr(message, field))
        count = _integer(getattr(message, count_field), count_field, maximum)
    except (AttributeError, TypeError) as exc:
        raise ValueError(f'missing or invalid {field}') from exc
    if count != len(result):
        raise ValueError(f'{count_field} does not match {field}')
    return result


def _within(value: float, lower: float, upper: float, label: str) -> None:
    if not math.isfinite(value) or value < lower - _EPSILON or value > upper + _EPSILON:
        raise ValueError(f'{label} exceeds configured limits')


def _poly(coefficients, u):
    a, b, c, d = coefficients
    return ((a*u + b)*u + c)*u + d


def _roots(a, b, c):
    """Real roots of a quadratic; used only to locate extrema in [0, 1]."""
    if abs(a) < 1e-15:
        return [] if abs(b) < 1e-15 else [-c/b]
    discriminant = b*b - 4*a*c
    if discriminant < 0:
        return []
    root = math.sqrt(discriminant)
    return [(-b-root)/(2*a), (-b+root)/(2*a)]


def _extrema(coefficients, seconds):
    a, b, c, _ = coefficients
    position_u = [0., 1.] + [u for u in _roots(3*a, 2*b, c) if 0 < u < 1]
    velocity_u = [0., 1.]
    if abs(a) >= 1e-15:
        u = -b/(3*a)
        if 0 < u < 1:
            velocity_u.append(u)
    return ([_poly(coefficients, u) for u in position_u],
            [(3*a*u*u + 2*b*u + c)/seconds for u in velocity_u])


def _tangents(times, positions):
    """Same endpoint and weighted harmonic rules as motor_control.c."""
    result = [0.] * len(times)
    for i in range(1, len(times)-1):
        h0 = (times[i]-times[i-1])/1000.
        h1 = (times[i+1]-times[i])/1000.
        d0 = (positions[i]-positions[i-1])/h0
        d1 = (positions[i+1]-positions[i])/h1
        if d0*d1 <= 0:
            continue
        w1, w2 = 2*h1+h0, h1+2*h0
        tangent = (w1+w2)/(w1/d0+w2/d1)
        limit = 3*min(abs(d0), abs(d1))
        result[i] = max(-limit, min(limit, tangent))
    return result


class CommandValidator:
    def __init__(self, driver_file: str, urdf_file: str,
                 max_trajectory_ms: int = 2000):
        self.max_trajectory_ms = _integer(max_trajectory_ms, 'max_trajectory_ms', 0xFFFFFFFF)
        if self.max_trajectory_ms == 0:
            raise ValueError('max_trajectory_ms must be positive')
        try:
            self.model = load_model(driver_file)
            ids = dict(self.model.actuator_ids)
            if len(set(ids.values())) != len(ids):
                raise ValueError('motor IDs must be globally unique')
            self.motor_ids = sorted(ids.values())
            self._name_of_id = {motor_id: name for name, motor_id in ids.items()}
            self._groups = list(self.model.groups)
            self._group_ids = [{ids[a] for a in group.actuators} for group in self._groups]
            self.config_files = [str(Path(driver_file).resolve()), str(Path(urdf_file).resolve())]
            document = yaml.safe_load(Path(driver_file).read_text(encoding='utf-8'))
            hardware = document['motor_hardware']
            specs_path = resolve_reference(driver_file, hardware['motor_specs_file'])
            self.config_files.append(str(specs_path))
            specs = yaml.safe_load(specs_path.read_text(encoding='utf-8'))['motor_types']
            transmission_ref = hardware.get('transmissions_file')
            if transmission_ref:
                self.config_files.append(str(resolve_reference(driver_file, transmission_ref)))
            elif Path(driver_file).with_name('transmissions.yaml').is_file():
                self.config_files.append(str(Path(driver_file).with_name('transmissions.yaml').resolve()))
            self._motor_limits = {}
            self._gains = {}
            for name, motor_id in ids.items():
                motor = hardware['motors'][name]
                parameters = dict(specs[motor['type']].get('parameters', {}))
                parameters.update(motor.get('parameters', {}))
                limits = []
                for wire, keys in ((
                        _WIRE_POSITION, ('mit_position_limit_rad',)),
                        (_WIRE_VELOCITY, ('mit_velocity_limit_rad_s',)),
                        (_WIRE_EFFORT, ('mit_torque_limit_nm', 'peak_torque_nm'))):
                    bound = wire
                    for key in keys:
                        if key in parameters:
                            value = _number(parameters[key], f'{name}.{key}')
                            if value <= 0:
                                raise ValueError(f'{name}.{key} must be positive')
                            bound = min(bound, value)
                    limits.append(bound)
                self._motor_limits[motor_id] = tuple(limits)
                gain_limits = []
                for key in ('mit_kp_limit', 'mit_kd_limit'):
                    value = _number(parameters[key], f'{name}.{key}')
                    if value <= 0:
                        raise ValueError(f'{name}.{key} must be positive')
                    gain_limits.append(value)
                self._gains[motor_id] = tuple(gain_limits)
            urdf = ET.parse(urdf_file).getroot()
            joints = urdf.findall('joint')
            names = [joint.get('name') for joint in joints]
            if len(names) != len(set(names)):
                raise ValueError('URDF joint names must be unique')
            by_name = {joint.get('name'): joint for joint in joints}
            self._joint_limits = {}
            for name in self.model.joint_names:
                joint = by_name.get(name)
                if (joint is None or joint.get('type') not in ('revolute', 'prismatic')
                        or joint.find('mimic') is not None):
                    raise ValueError(f'{name} must be an independent bounded URDF joint')
                limit = joint.find('limit')
                if limit is None:
                    raise ValueError(f'{name} requires explicit URDF limits')
                values = tuple(_number(float(limit.attrib[key]), f'{name}.{key}')
                               for key in ('lower', 'upper', 'velocity', 'effort'))
                if values[0] >= values[1] or values[2] <= 0 or values[3] <= 0:
                    raise ValueError(f'{name} has invalid URDF limits')
                self._joint_limits[name] = values
        except (OSError, KeyError, TypeError, AttributeError, RuntimeError,
                ET.ParseError, yaml.YAMLError) as exc:
            raise ValueError(f'invalid robot configuration: {exc}') from exc

    def _identities(self, targets):
        ids = []
        for target in targets:
            try:
                motor_id = _integer(target.motor_id, 'motor_id', 65535)
            except AttributeError as exc:
                raise ValueError('target requires motor_id') from exc
            if motor_id not in self._name_of_id or motor_id in ids:
                raise ValueError('unknown or duplicate motor_id')
            ids.append(motor_id)
        return ids

    def _complete_groups(self, ids):
        selected = set(ids)
        for group_ids in self._group_ids:
            if selected & group_ids and not group_ids <= selected:
                raise ValueError('intent must cover its complete transmission group')

    def _axes(self, targets):
        ids = self._identities(targets)
        self._complete_groups(ids)
        axes = {}
        for target, motor_id in zip(targets, ids):
            try:
                values = tuple(_number(getattr(target, name), f'm{motor_id}.{name}')
                               for name in ('position', 'velocity', 'effort'))
            except AttributeError as exc:
                raise ValueError('target requires position, velocity and effort') from exc
            for value, maximum, name in zip(values, self._motor_limits[motor_id],
                                             ('position', 'velocity', 'effort')):
                _within(value, -maximum, maximum, f'm{motor_id}.{name}')
            axes[self._name_of_id[motor_id]] = Axis(*values)
        try:
            joints = self.model.actuator_to_joint(axes)
        except (RuntimeError, ValueError) as exc:
            raise ValueError(f'transmission conversion failed: {exc}') from exc
        for name, sample in joints.items():
            lower, upper, velocity, effort = self._joint_limits[name]
            _within(sample.position, lower, upper, f'{name}.position')
            _within(sample.velocity, -velocity, velocity, f'{name}.velocity')
            _within(sample.effort, -effort, effort, f'{name}.effort')
        return joints

    def command(self, message) -> None:
        self._axes(_items(message, 'targets', 'count', 64))

    def policy(self, message) -> None:
        targets = _items(message, 'targets', 'count', 64)
        ids = self._identities(targets)
        self._complete_groups(ids)
        enabled_by_id = {}
        for target, motor_id in zip(targets, ids):
            if (type(getattr(target, 'enabled', None)) is not bool
                    or type(getattr(target, 'confirm_unprotected', None)) is not bool):
                raise ValueError('policy flags must be booleans')
            enabled_by_id[motor_id] = target.enabled
            for name, maximum in zip(('kp', 'kd'), self._gains[motor_id]):
                value = _number(getattr(target, name, None), f'm{motor_id}.{name}')
                _within(value, 0., maximum, f'm{motor_id}.{name}')
        for group_ids in self._group_ids:
            if group_ids & set(ids) and len({enabled_by_id[i] for i in group_ids}) != 1:
                raise ValueError('transmission group policy enable flags must agree')

    def trajectory(self, message) -> None:
        points = _items(message, 'points', 'count', 256)
        ids = _items(message, 'target_ids', 'target_count', 64)
        if not points and not ids:
            return  # Cancellation does not depend on segment fields.
        if not points or not ids:
            raise ValueError('trajectory requires targets and points')
        if (getattr(message, 'segment_index', None) != 0
                or type(getattr(message, 'segment_index', None)) is not int
                or getattr(message, 'segment_count', None) != 1
                or type(getattr(message, 'segment_count', None)) is not int):
            raise ValueError('managed trajectories require one complete segment')
        for motor_id in ids:
            _integer(motor_id, 'target motor_id', 65535)
            if motor_id not in self._name_of_id:
                raise ValueError('unknown target motor_id')
        if len(set(ids)) != len(ids):
            raise ValueError('duplicate target motor_id')
        self._complete_groups(ids)
        for group, group_ids in zip(self._groups, self._group_ids):
            if set(ids) & group_ids and group.plugin not in _LINEAR_PLUGINS:
                raise ValueError('trajectory bounds require a supported linear transmission')
        by_time = {}
        previous = -1
        for point in points:
            try:
                time_ms = _integer(point.time_from_start_ms, 'time_from_start_ms', self.max_trajectory_ms)
                motor_id = _integer(point.motor_id, 'point motor_id', 65535)
                velocity = _number(point.velocity, 'point.velocity')
            except AttributeError as exc:
                raise ValueError('incomplete trajectory point') from exc
            if time_ms < previous or motor_id not in ids:
                raise ValueError('trajectory times must be ordered and target IDs declared')
            if velocity != 0:
                raise ValueError('managed trajectory velocity diagnostics must be zero')
            previous = time_ms
            samples = by_time.setdefault(time_ms, {})
            if motor_id in samples:
                raise ValueError('each motor needs strictly increasing waypoint times')
            samples[motor_id] = point
        times = list(by_time)
        if times[0] != 0:
            raise ValueError('trajectory must contain measured start targets at t=0')
        for samples in by_time.values():
            if set(samples) != set(ids):
                raise ValueError('every trajectory time must cover every target motor')
            self._axes(list(samples.values()))
        tangents = {i: _tangents(times, [by_time[t][i].position for t in times]) for i in ids}
        for index, (left, right) in enumerate(zip(times, times[1:])):
            seconds = (right-left)/1000.
            coefficients = {}
            for motor_id in ids:
                p0, p1 = by_time[left][motor_id].position, by_time[right][motor_id].position
                m0, m1 = tangents[motor_id][index], tangents[motor_id][index+1]
                coeff = (2*p0-2*p1+seconds*(m0+m1),
                         -3*p0+3*p1-seconds*(2*m0+m1), seconds*m0, p0)
                coefficients[motor_id] = coeff
                positions, velocities = _extrema(coeff, seconds)
                position_limit, velocity_limit, _ = self._motor_limits[motor_id]
                for value in positions:
                    _within(value, -position_limit, position_limit, f'm{motor_id}.interpolated position')
                for value in velocities:
                    _within(value, -velocity_limit, velocity_limit, f'm{motor_id}.interpolated velocity')
            joint_coefficients = {}
            for degree in range(4):
                # Position transforms include offsets only for the constant
                # coefficient; velocity transforms are the linear part.
                axes = {self._name_of_id[i]: Axis(coeff[degree], 0., 0.) if degree == 3
                        else Axis(0., coeff[degree], 0.) for i, coeff in coefficients.items()}
                try:
                    converted = self.model.actuator_to_joint(axes)
                except (ValueError, RuntimeError) as exc:
                    raise ValueError(f'trajectory transmission conversion failed: {exc}') from exc
                for name, sample in converted.items():
                    joint_coefficients.setdefault(name, []).append(
                        sample.position if degree == 3 else sample.velocity)
            for name, coeff in joint_coefficients.items():
                lower, upper, velocity, _ = self._joint_limits[name]
                positions, velocities = _extrema(coeff, seconds)
                for value in positions:
                    _within(value, lower, upper, f'{name}.interpolated position')
                for value in velocities:
                    _within(value, -velocity, velocity, f'{name}.interpolated velocity')
