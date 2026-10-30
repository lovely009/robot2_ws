"""Bounded position-only trajectory conversion and MCU interpolation model.

The MCU is the time-line authority. This model is used to report desired state
and measure tracking error at MCU sample ticks; it never streams setpoints.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

from can_bridge_msgs.msg import MotorTrajectory, MotorTrajectoryPoint
from robot_transmission import Axis


def _finite(values, label):
    if any(not math.isfinite(value) for value in values):
        raise ValueError(f'{label} must contain finite numbers')


def duration_ms(value):
    if value.sec < 0 or not 0 <= value.nanosec < 1_000_000_000:
        raise ValueError('time_from_start must be a nonnegative normalized duration')
    nanoseconds = value.sec * 1_000_000_000 + value.nanosec
    if nanoseconds % 1_000_000:
        raise ValueError('MCU waypoints require exact integer-millisecond times')
    return nanoseconds // 1_000_000


def _tangents(times, values):
    result = [0.] * len(times)
    for index in range(1, len(times) - 1):
        h0, h1 = (times[index] - times[index - 1]) / 1000., (times[index + 1] - times[index]) / 1000.
        d0, d1 = (values[index] - values[index - 1]) / h0, (values[index + 1] - values[index]) / h1
        if d0 * d1 <= 0:
            continue
        w0, w1 = 2 * h1 + h0, h1 + 2 * h0
        value = (w0 + w1) / (w0 / d0 + w1 / d1)
        limit = 3 * min(abs(d0), abs(d1))
        result[index] = max(-limit, min(limit, value))
    return result


def signed_tick_difference(new, old):
    """Signed modular MCU millisecond interval; goals are much shorter than 2^31 ms."""
    value = (int(new) - int(old)) & 0xFFFFFFFF
    return value if value < 0x80000000 else value - 0x100000000


@dataclass(frozen=True)
class Tolerances:
    path_position: dict
    path_velocity: dict
    goal_position: dict
    goal_velocity: dict
    goal_time_s: float


class CompiledTrajectory:
    def __init__(self, model, joint_names, times, message, tolerances):
        self.model, self.joint_names = model, tuple(joint_names)
        self.times, self.message, self.tolerances = tuple(times), message, tolerances
        self.motor_ids = tuple(message.target_ids)
        self._actuators = {mid: name for name, mid in model.actuator_ids.items()}
        self._positions = {mid: [p.position for p in message.points if p.motor_id == mid]
                           for mid in self.motor_ids}
        self._tangents = {mid: _tangents(times, values)
                          for mid, values in self._positions.items()}

    @property
    def total_ms(self):
        return self.times[-1]

    def desired_actuator(self, motor_id, elapsed_ms):
        """Expected value for one actuator at its own MCU measurement time."""
        elapsed_ms = min(self.total_ms, max(0., elapsed_ms))
        values = self._positions[motor_id]
        if elapsed_ms >= self.total_ms:
            return Axis(values[-1], 0., 0.)
        index = next(i for i in range(len(self.times) - 1) if elapsed_ms < self.times[i + 1])
        seconds = (self.times[index + 1] - self.times[index]) / 1000.
        u = (elapsed_ms - self.times[index]) / (seconds * 1000.)
        p0, p1 = values[index:index + 2]
        m0, m1 = self._tangents[motor_id][index:index + 2]
        a = 2 * p0 - 2 * p1 + seconds * (m0 + m1)
        b = -3 * p0 + 3 * p1 - seconds * (2 * m0 + m1)
        c = seconds * m0
        return Axis(((a * u + b) * u + c) * u + p0,
                    (3 * a * u * u + 2 * b * u + c) / seconds, 0.)

    def desired_actuators(self, elapsed_ms):
        return {self._actuators[mid]: self.desired_actuator(mid, elapsed_ms)
                for mid in self.motor_ids}

    def desired(self, elapsed_ms):
        return self.model.actuator_to_joint(self.desired_actuators(elapsed_ms))


class TrajectoryCompiler:
    def __init__(self, model, validate_trajectory, *, max_trajectory_ms=2000,
                 path_tolerance=.2, goal_tolerance=.02, stopped_velocity_tolerance=.01):
        self.model, self.validate = model, validate_trajectory
        self.max_trajectory_ms = max_trajectory_ms
        self.path_tolerance, self.goal_tolerance = path_tolerance, goal_tolerance
        self.stopped_velocity_tolerance = stopped_velocity_tolerance
        if not isinstance(max_trajectory_ms, int) or not 1 <= max_trajectory_ms <= 0x7FFFFFFF:
            raise ValueError('max_trajectory_ms must be a positive signed uint32 interval')
        if any(not math.isfinite(value) or value <= 0 for value in
               (path_tolerance, goal_tolerance, stopped_velocity_tolerance)):
            raise ValueError('default position and stopped-velocity tolerances must be positive')

    def _tolerances(self, goal, names):
        mappings = []
        for incoming, default_position, default_velocity in (
                (goal.path_tolerance, self.path_tolerance, 0.),
                (goal.goal_tolerance, self.goal_tolerance, self.stopped_velocity_tolerance)):
            positions = {name: default_position for name in names}
            velocities = {name: default_velocity for name in names}
            seen = set()
            for tolerance in incoming:
                if tolerance.name not in positions or tolerance.name in seen:
                    raise ValueError('tolerances require known, unique joint names')
                seen.add(tolerance.name)
                values = (tolerance.position, tolerance.velocity, tolerance.acceleration)
                _finite(values, 'tolerances')
                if any(value < 0 for value in values):
                    raise ValueError('disabling tracking tolerances is unsupported')
                if tolerance.acceleration != 0:
                    raise ValueError('acceleration tolerance is unsupported by MCU feedback')
                if tolerance.position:
                    positions[tolerance.name] = tolerance.position
                if tolerance.velocity:
                    velocities[tolerance.name] = tolerance.velocity
            mappings.extend((positions, velocities))
        duration = goal.goal_time_tolerance
        if duration.sec < 0 or not 0 <= duration.nanosec < 1_000_000_000:
            raise ValueError('goal_time_tolerance must be a normalized nonnegative duration')
        seconds = duration.sec + duration.nanosec / 1e9
        if seconds > 10:
            raise ValueError('goal_time_tolerance exceeds the 10 second execution bound')
        return Tolerances(*mappings, seconds or 1.)

    def compile(self, goal, trajectory_id):
        trajectory = goal.trajectory
        if trajectory.header.stamp.sec or trajectory.header.stamp.nanosec:
            raise ValueError('scheduled starts are unsupported; use a zero trajectory header stamp')
        if goal.multi_dof_trajectory.joint_names or goal.multi_dof_trajectory.points:
            raise ValueError('multi-DOF trajectories are unsupported')
        names = list(trajectory.joint_names)
        if len(set(names)) != len(names) or set(names) != set(self.model.joint_names):
            raise ValueError('trajectory must name every configured joint exactly once')
        if not 2 <= len(trajectory.points):
            raise ValueError('trajectory needs a measured t=0 point and at least one endpoint')
        tolerances = self._tolerances(goal, names)
        motor_ids = sorted(self.model.actuator_ids.values())
        if len(trajectory.points) * len(motor_ids) > 256:
            raise ValueError('expanded trajectory exceeds 256 motor waypoints')
        message = MotorTrajectory(trajectory_id=trajectory_id, segment_index=0, segment_count=1,
                                  target_count=len(motor_ids), target_ids=motor_ids)
        times = []
        for point in trajectory.points:
            if len(point.positions) != len(names):
                raise ValueError('every waypoint needs one position per joint')
            _finite(point.positions, 'positions')
            if point.velocities or point.accelerations or point.effort:
                raise ValueError('MCU position trajectories require empty velocity, acceleration and effort arrays')
            tick = duration_ms(point.time_from_start)
            if tick > self.max_trajectory_ms or (times and tick <= times[-1]) or (not times and tick != 0):
                raise ValueError('waypoint times must start at zero, strictly increase and fit the duration bound')
            times.append(tick)
            axes = self.model.joint_to_actuator(
                {name: Axis(position, 0., 0.) for name, position in zip(names, point.positions)})
            for actuator, axis in sorted(axes.items(), key=lambda item: self.model.actuator_ids[item[0]]):
                message.points.append(MotorTrajectoryPoint(
                    motor_id=self.model.actuator_ids[actuator], time_from_start_ms=tick,
                    position=axis.position, velocity=0., effort=0.))
        message.count = len(message.points)
        self.validate(message)
        return CompiledTrajectory(self.model, names, times, message, tolerances)
