"""ros2_control 重力参数适配；运行状态只认新鲜的 workbench_control 回读。"""

from __future__ import annotations

import math
from pathlib import Path
import re
import time

from robot_transmission import Model

CONTROL_STATUS_TIMEOUT_S = 2.5
PARAMETER_TIMEOUT_S = 3.0
_MODEL = re.compile(r'[A-Za-z0-9][A-Za-z0-9_.-]*\Z')


def _gain(value):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not 0.0 <= value <= 1.0 or not math.isfinite(value)):
        raise ValueError('effort_gain must be a finite number in 0..1')
    return float(value)


def validate_joint_map(spec: str) -> dict[int, str]:
    """兼容字段只做模型断言，不再配置控制器。"""
    if not isinstance(spec, str) or not spec:
        raise ValueError('joint_map must contain motor_id:joint entries')
    result = {}
    for entry in spec.split(','):
        motor_id, sep, joint = entry.partition(':')
        try:
            numeric_id = int(motor_id)
        except ValueError:
            raise ValueError(f'invalid joint_map motor_id: {motor_id!r}') from None
        if (not sep or not joint or joint != joint.strip() or ':' in joint
                or not 0 <= numeric_id <= 65535 or numeric_id in result):
            raise ValueError(f'invalid or duplicate joint_map entry: {entry!r}')
        result[numeric_id] = joint
    return result


class GravityFeedforwardManager:
    def __init__(self, models_root=None, parameter_writer=None, clock=time.monotonic):
        if models_root is None:
            from .model_catalog import default_models_root
            models_root = default_models_root()
        self._models_root = Path(models_root) if models_root else None
        self._parameter_writer = parameter_writer
        self._clock = clock
        self._control = None
        self._observed_at = None
        self._revision = 0
        self._pending = None
        self._error = ''
        self._gain_error = ''
        self._status_error = ''

    def observe(self, status):
        if not isinstance(status, dict):
            raise ValueError('/control/status must be a JSON object')
        model, joints, mode = (status.get(k) for k in ('model', 'joint_names', 'mode'))
        if (not isinstance(model, str) or not _MODEL.fullmatch(model)
                or not isinstance(joints, list) or not joints
                or any(not isinstance(j, str) or not j for j in joints)
                or len(set(joints)) != len(joints)
                or not isinstance(mode, str) or not mode
                or type(status.get('gravity_enabled')) is not bool):
            raise ValueError('invalid /control/status model, joints, mode or gravity_enabled')
        gain = _gain(status.get('effort_gain'))
        self._control = dict(model=model, joint_names=list(joints), mode=mode,
                             gravity_enabled=status['gravity_enabled'], effort_gain=gain)
        self._observed_at = self._clock()
        self._revision += 1
        self._status_error = ''
        self.tick()
        self._confirm()

    def invalidate_status(self, message):
        self._observed_at = None
        self._status_error = str(message)

    def _fresh(self):
        return (self._observed_at is not None
                and 0 <= self._clock() - self._observed_at <= CONTROL_STATUS_TIMEOUT_S)

    def status(self):
        self.tick()
        fresh = self._fresh()
        control = self._control
        running = bool(fresh and control['gravity_enabled'] and control['mode'] in ('trajectory', 'follow'))
        return {
            'state': 'running' if running else 'stopped',
            'config': ({'model': control['model'], 'effort_gain': control['effort_gain']}
                       if control else None),
            'returncode': None,
            'gain_error': self._gain_error,
            'error': (self._error or self._status_error
                      or ('' if fresh else '/control/status unavailable or stale')),
            'pending': self._pending['action'] if self._pending else None,
            'fresh': fresh,
        }

    def _require_control(self):
        if not self._fresh():
            raise RuntimeError('/control/status unavailable or stale')
        return self._control

    def start(self, request):
        if not isinstance(request, dict):
            raise ValueError('request must be a JSON object')
        control = self._require_control()
        model = request.get('model')
        if model != control['model']:
            raise ValueError('requested model must match the current /control/status model')
        if not self._models_root:
            raise ValueError('model transmission directory is unavailable')
        transmission = Model.load(str(self._models_root / model / 'config' / 'motor_driver.yaml'))
        if set(transmission.joint_names) != set(control['joint_names']):
            raise ValueError('/control/status joint_names do not match the transmission model')
        spec = request.get('joint_map', '')
        if spec != '':
            ids = transmission.actuator_ids
            direct = {ids[g.actuators[0]]: g.joints[0] for g in transmission.groups
                      if len(g.actuators) == len(g.joints) == 1}
            supplied = validate_joint_map(spec)
            if any(direct.get(motor_id) != joint for motor_id, joint in supplied.items()):
                raise ValueError('joint_map does not match direct transmission groups; '
                                 'coupled groups cannot be flattened')
        return self._write('start', {'enabled': True,
                                     'effort_gain': _gain(request.get('effort_gain', 1.0))})

    def stop(self):
        return self._write('stop', {'enabled': False, 'effort_gain': 0.0})

    def set_gain(self, value):
        gain = _gain(value)
        if self.status()['state'] != 'running':
            raise RuntimeError('gravity controller is not running')
        return self._write('gain', {'effort_gain': gain})

    def _write(self, action, values):
        self.tick()
        if self._pending:
            raise RuntimeError('gravity controller parameter request is pending')
        if self._parameter_writer is None:
            raise RuntimeError('gravity controller parameter service is unavailable')
        identity = ((self._control['model'], frozenset(self._control['joint_names']))
                    if self._fresh() else None)
        operation = dict(action=action, values=values, identity=identity, revision=self._revision,
                         deadline=self._clock() + PARAMETER_TIMEOUT_S,
                         acknowledged=False, future=None)
        self._pending = operation
        self._error = self._gain_error = ''
        try:
            future = self._parameter_writer(values)
            operation['future'] = future
            future.add_done_callback(lambda result: self._written(operation, result))
        except Exception as exc:
            self.note_failed(str(exc), action)
            self._pending = None
            raise
        return self.status()

    def _written(self, operation, future):
        self.tick()
        if self._pending is not operation:
            return  # 超时或 cleanup 后的回执不能认领后来的请求。
        try:
            result = future.result().result
            if not result.successful:
                raise RuntimeError(result.reason or 'gravity controller rejected parameters')
        except Exception as exc:
            self.note_failed(str(exc), operation['action'])
            self._pending = None
            return
        operation['acknowledged'] = True
        self._confirm()

    def _confirm(self):
        operation = self._pending
        if not operation or not self._fresh():
            return
        identity = (self._control['model'], frozenset(self._control['joint_names']))
        if operation['identity'] is not None and operation['identity'] != identity:
            self._pending = None
            self.note_failed('control model or joint_names changed during parameter request',
                             operation['action'])
            if operation['future'] is not None:
                operation['future'].cancel()
            return
        if not operation['acknowledged'] or self._revision <= operation['revision']:
            return
        actual = {'enabled': self._control['gravity_enabled'],
                  'effort_gain': self._control['effort_gain']}
        if all(actual[k] == v for k, v in operation['values'].items()):
            self._pending = None

    def tick(self):
        operation = self._pending
        if operation and self._clock() >= operation['deadline']:
            self._pending = None
            self.note_failed('gravity controller parameter request timed out '
                             '(waiting for response or /control/status confirmation)',
                             operation['action'])
            if operation['future'] is not None:
                operation['future'].cancel()

    def note_failed(self, message, action=''):
        self._error = str(message) or 'gravity controller parameter request failed'
        if action == 'gain':
            self._gain_error = self._error

    def close(self):
        operation, self._pending = self._pending, None
        if operation and operation['future'] is not None:
            operation['future'].cancel()
        self.invalidate_status('/control/status subscription is closed')
