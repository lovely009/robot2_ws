"""Robot readiness, exclusive control leases and host authorization.

This module has no ROS dependency. All expiries use the supplied monotonic clock;
ROS message timestamps are an additional ingress check performed by the node.
Neither ``armed`` nor a stop request claims a physical drive acknowledgement.
"""
from __future__ import annotations

import copy
import math
import secrets
import time
from collections.abc import Mapping

_U32 = (1 << 32) - 1
_U64 = (1 << 64) - 1
_LIFECYCLES = {0, 1, 2, 3, 4, 10, 13, 14, 15}


def _uint(value, maximum, minimum=0):
    return type(value) is int and minimum <= value <= maximum


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _result(success, reason='', **extra):
    return {'success': bool(success), 'reason': reason, **extra}


class SupervisorCore:
    BOOTING, NOT_READY, READY, ARMED, RUNNING, HOLD, FAULT = range(7)

    def __init__(self, robot_id='', model_name='', clock=time.monotonic,
                 control_timeout_s=.5, calibration_timeout_s=1.5,
                 min_lease_ms=100, max_lease_ms=2000, epoch=None):
        for label, value in (('control_timeout_s', control_timeout_s),
                             ('calibration_timeout_s', calibration_timeout_s)):
            if not _finite(value) or value <= 0:
                raise ValueError(f'{label} must be finite and positive')
        if (not _uint(min_lease_ms, _U32, 1)
                or not _uint(max_lease_ms, _U32, min_lease_ms)):
            raise ValueError('invalid control lease range')
        self.robot_id, self.model_name = robot_id, model_name
        self.clock = clock
        self.control_timeout_s = float(control_timeout_s)
        self.calibration_timeout_s = float(calibration_timeout_s)
        self.min_lease_ms, self.max_lease_ms = min_lease_ms, max_lease_ms
        # Public generation travels through rosbridge JSON. The independent
        # cryptographic lease token remains the actual authorization secret.
        self.epoch = (secrets.randbits(53) or 1) if epoch is None else epoch
        if not _uint(self.epoch, _U64, 1):
            raise ValueError('epoch must be a nonzero uint64')
        self.session_id = 0
        self.armed = False
        self.fault_latched = False
        self._fault_reason = ''
        self._owner = ''
        self._mode = 0
        self._token = ''
        self._expires = 0.
        self._last_input_sequence = -1
        self._calibration = None
        self._calibration_at = None
        self._expected = {}
        self._control = {}
        self._control_valid = False
        self._control_at = None
        self._control_sequence = None
        self._control_revision = 0
        self._topology = None
        self._params = None
        self._known_ids = set()
        self._generation = 0
        self._stop_pending = None
        self._stopping_at = None
        self._stopping_revision = -1
        self._stop_targets = set()
        self._stop_reasons = set()
        self._external_blockers = []

    @property
    def known_motor_ids(self):
        return sorted(self._known_ids)

    @property
    def policy_generation(self):
        return self._generation

    def _now(self):
        now = self.clock()
        if not _finite(now):
            raise ValueError('monotonic clock must return finite seconds')
        return float(now)

    def _request_stop(self, reason, now):
        if self._stopping_at is None:
            self._stopping_at = now
            self._stopping_revision = self._control_revision
            self._stop_reasons.clear()
        # Retransmission cadence belongs to the transport adapter. Merely
        # waiting for an acknowledgement must not produce another large stop
        # batch at every control-loop tick.
        if reason != 'awaiting_disabled' and reason not in self._stop_reasons:
            if self._stop_pending is None:
                self._stop_pending = reason
            self._stop_reasons.add(reason)
        self._stop_targets.update(self._expected)
        self._stop_targets.update(self._control)

    def _clear_lease(self):
        self._token, self._owner, self._mode = '', '', 0
        self._expires, self._last_input_sequence = 0., -1
        self.armed = False

    def _latch_fault(self, reason, now):
        if not self.fault_latched:
            self._fault_reason = reason
        self.fault_latched = True
        self._request_stop(reason, now)
        self._clear_lease()

    def _advance_session(self, session, now):
        if session == self.session_id:
            return
        previous = self.session_id
        self.session_id = session
        self._control, self._control_valid = {}, False
        self._control_at, self._control_sequence = None, None
        self._topology, self._params = None, None
        # A new bridge/Agent session does not prove that the MCU restarted.
        # Preserve the process high-water mark so a stop sent before the first
        # new control report cannot write generation zero back into live drives.
        if previous:
            self._latch_fault('mcu_session_changed', now)

    @staticmethod
    def _calibration_expected(report):
        fields = [report.get(name) for name in
                  ('motor_ids', 'bus_indices', 'protocols', 'model_names')]
        if (not all(isinstance(value, (list, tuple)) for value in fields)
                or not 1 <= len(fields[0]) <= 64
                or any(len(value) != len(fields[0]) for value in fields)):
            return None
        ids, buses, protocols, models = fields
        if (any(not _uint(mid, 65535) for mid in ids) or len(set(ids)) != len(ids)
                or any(not _uint(bus, 2) for bus in buses)
                or any(type(protocol) is not int or protocol not in (0, 1)
                       for protocol in protocols)
                or any(not isinstance(model, str) or not model or len(model) > 31
                       for model in models)):
            return None
        return {mid: {'bus_index': bus, 'protocol': protocol, 'model_name': model}
                for mid, bus, protocol, model in zip(ids, buses, protocols, models)}

    def update_calibration(self, report):
        now = self._now()
        if not isinstance(report, Mapping):
            self._calibration = None
            self._calibration_at = now
            self.tick()
            return False
        incoming = copy.deepcopy(dict(report))
        expected = self._calibration_expected(incoming)
        previous = self._calibration
        identity_fields = ('robot_id', 'model_name', 'calibration_id', 'config_hash',
                           'motor_ids', 'bus_indices', 'protocols', 'model_names')
        changed = previous is not None and any(
            previous.get(field) != incoming.get(field) for field in identity_fields)
        if changed and (self._token or self.armed):
            self._latch_fault('calibration_changed', now)
        self._calibration = incoming
        self._calibration_at = now
        if expected is not None:
            self._expected = expected
            self._known_ids.update(expected)
        self.tick()
        return expected is not None

    def update_control(self, report):
        now = self._now()
        if not isinstance(report, Mapping):
            self._control_valid = False
            self.tick()
            return False
        session, sequence = report.get('session_id'), report.get('sequence')
        if _uint(session, _U64, 1) and session < self.session_id:
            return False
        states = report.get('states')
        if (report.get('schema_version') != 2
                or type(report.get('schema_version')) is not int
                or not _uint(session, _U64, 1) or not _uint(sequence, _U32)
                or not isinstance(states, (list, tuple)) or len(states) > 64):
            self._control_valid = False
            self.tick()
            return False
        fresh = {}
        for state in states:
            if not isinstance(state, Mapping):
                self._control_valid = False
                self.tick()
                return False
            mid = state.get('motor_id')
            if (not _uint(mid, 65535) or mid in fresh
                    or type(state.get('control_lifecycle')) is not int
                    or state.get('control_lifecycle') not in _LIFECYCLES
                    or not _uint(state.get('execution_state'), 7)
                    or any(type(state.get(field)) is not bool for field in
                           ('online', 'policy_enabled', 'motion_enabled'))
                    or any(not _uint(state.get(field), maximum) for field, maximum in
                           (('state_age_ms', _U32), ('unified_fault_code', 65535),
                            ('vendor_fault_code', _U32), ('last_failure_reason', 65535),
                            ('policy_generation', _U32)))):
                self._control_valid = False
                self.tick()
                return False
            fresh[mid] = copy.deepcopy(dict(state))
        if session == self.session_id and self._control_sequence is not None:
            advance = (sequence - self._control_sequence) & _U32
            if advance == 0 or advance >= 0x80000000:
                return False
        self._advance_session(session, now)
        self._control = fresh
        self._known_ids.update(fresh)
        self._control_valid, self._control_at = True, now
        self._control_sequence = sequence
        self._control_revision += 1
        self._generation = max([self._generation] +
                               [state['policy_generation'] for state in fresh.values()])
        self.tick()
        return True

    def update_topology(self, report):
        now = self._now()
        if not isinstance(report, Mapping):
            self._topology = None
            self.tick()
            return False
        session = report.get('session_id')
        if _uint(session, _U64, 1) and session < self.session_id:
            return False
        entries = report.get('entries')
        valid = (type(report.get('schema_version')) is int
                 and report.get('schema_version') == 3
                 and _uint(session, _U64, 1)
                 and isinstance(entries, (list, tuple)) and len(entries) <= 64)
        mapped = {}
        if valid:
            for entry in entries:
                if not isinstance(entry, Mapping):
                    valid = False
                    break
                mid = entry.get('motor_id')
                if (not _uint(mid, 65535) or mid in mapped
                        or not _uint(entry.get('bus_index'), 2)
                        or type(entry.get('protocol')) is not int
                        or entry.get('protocol') not in (0, 1)
                        or not isinstance(entry.get('model_name'), str)
                        or len(entry['model_name']) > 31
                        or any(type(entry.get(key)) is not bool
                               for key in ('online', 'conflict'))):
                    valid = False
                    break
                mapped[mid] = copy.deepcopy(dict(entry))
        if not valid:
            self._topology = None
            self.tick()
            return False
        self._advance_session(session, now)
        self._topology = mapped
        self._known_ids.update(mapped)
        self.tick()
        return True

    def update_device_params(self, report):
        now = self._now()
        if not isinstance(report, Mapping):
            self._params = None
            self.tick()
            return False
        session = report.get('session_id')
        if _uint(session, _U64, 1) and session < self.session_id:
            return False
        entries = report.get('entries')
        valid = (_uint(session, _U64, 1) and isinstance(entries, (list, tuple))
                 and len(entries) <= 64 and type(report.get('count')) is int
                 and report['count'] == len(entries))
        mapped = {}
        if valid:
            for entry in entries:
                if not isinstance(entry, Mapping):
                    valid = False
                    break
                mid = entry.get('motor_id')
                if (not _uint(mid, 65535) or mid in mapped
                        or not _uint(entry.get('dev_init_state'), 5)
                        or type(entry.get('record_dirty')) is not bool
                        or not _uint(entry.get('mismatch_mask'), 255)):
                    valid = False
                    break
                mapped[mid] = copy.deepcopy(dict(entry))
        if not valid:
            self._params = None
            self.tick()
            return False
        self._advance_session(session, now)
        self._params = mapped
        self._known_ids.update(mapped)
        self.tick()
        return True

    def _base_blockers(self, now):
        blockers = list(self._external_blockers)
        calibration = self._calibration
        if calibration is None:
            blockers.append('calibration_missing')
        else:
            if (self._calibration_at is None or not
                    0 <= now - self._calibration_at <= self.calibration_timeout_s):
                blockers.append('calibration_stale')
            if calibration.get('valid') is not True:
                blockers.append('calibration_invalid')
            if calibration.get('commissioned') is not True:
                blockers.append('calibration_not_commissioned')
            if (not isinstance(calibration.get('blockers'), (list, tuple))
                    or calibration.get('blockers')):
                blockers.append('calibration_has_blockers')
            if self.robot_id and calibration.get('robot_id') != self.robot_id:
                blockers.append('robot_id_mismatch')
            if self.model_name and calibration.get('model_name') != self.model_name:
                blockers.append('model_name_mismatch')
            if any(not isinstance(calibration.get(key), str)
                   or not calibration[key].strip()
                   for key in ('robot_id', 'model_name', 'calibration_id', 'config_hash')):
                blockers.append('calibration_identity_missing')
            if self._calibration_expected(calibration) is None:
                blockers.append('calibration_motor_map_invalid')
        if not self.session_id:
            blockers.append('mcu_session_missing')
        if not self._control_valid or self._control_at is None:
            blockers.append('control_state_missing_or_invalid')
        else:
            age = now - self._control_at
            if not 0 <= age <= self.control_timeout_s:
                blockers.append('control_state_stale')
            for mid in self._expected:
                state = self._control.get(mid)
                if state is None:
                    blockers.append(f'motor_{mid}_control_missing')
                    continue
                allowed_lifecycle = (2, 3, 13, 14) if self._token else (2, 3)
                if (not state['online'] or state['unified_fault_code']
                        or state['vendor_fault_code']
                        or state['control_lifecycle'] not in allowed_lifecycle):
                    blockers.append(f'motor_{mid}_not_healthy')
                if not 0 <= age + state['state_age_ms'] / 1000 <= self.control_timeout_s:
                    blockers.append(f'motor_{mid}_feedback_stale')
            for mid, state in self._control.items():
                if mid not in self._expected and state['online']:
                    blockers.append(f'motor_{mid}_unexpected')
        if self._topology is None:
            blockers.append('topology_missing_or_invalid')
        else:
            for mid, expected in self._expected.items():
                entry = self._topology.get(mid)
                if entry is None:
                    blockers.append(f'motor_{mid}_topology_missing')
                elif (entry['conflict'] or any(entry.get(key) != expected[key]
                                               for key in expected)):
                    blockers.append(f'motor_{mid}_identity_mismatch')
            for mid, entry in self._topology.items():
                if entry['conflict'] or (mid not in self._expected and entry['online']):
                    blockers.append(f'motor_{mid}_topology_conflict_or_unexpected')
        for mid, expected in self._expected.items():
            if expected['protocol'] != 1:
                continue
            entry = self._params.get(mid) if self._params is not None else None
            if (entry is None or entry['dev_init_state'] != 2 or entry['record_dirty']
                    or entry['mismatch_mask'] or not _finite(entry.get('dev_safety'))
                    or entry['dev_safety'] != 0):
                blockers.append(f'motor_{mid}_device_parameters_unverified')
        return blockers

    def _all_disabled(self):
        required = set(self._expected)
        if self._stopping_at is not None:
            required.update(self._stop_targets)
        return bool(self._control_valid and self._control
                    and required.issubset(self._control) and all(
            not state['policy_enabled'] and not state['motion_enabled']
            for state in self._control.values()))

    def _any_enabled(self):
        return self._control_valid and any(state['policy_enabled'] or state['motion_enabled']
                                           for state in self._control.values())

    def _stable(self):
        return all(self._control.get(mid, {}).get('control_lifecycle') in (2, 3)
                   for mid in self._expected)

    def tick(self):
        now = self._now()
        if self._token and now >= self._expires:
            self._latch_fault('control_lease_expired', now)
        blockers = self._base_blockers(now)
        if blockers and (self._token or self.armed):
            self._latch_fault('readiness_lost:' + blockers[0], now)
        if self._stopping_at is not None:
            # A pre-stop disabled sample cannot acknowledge the stop: an
            # enable may already be in flight but not yet reflected by the MCU.
            if (self._control_revision > self._stopping_revision
                    and self._all_disabled()):
                self._stopping_at = None
                self._stop_targets.clear()
            else:
                self._request_stop('awaiting_disabled', now)
                if now - self._stopping_at > self.control_timeout_s:
                    self._latch_fault('disable_not_confirmed', now)
        if not self._token and self._any_enabled() and self._stopping_at is None:
            self._latch_fault('unexpected_motion_without_owner', now)

    def snapshot(self):
        self.tick()
        now = self._now()
        blockers = self._base_blockers(now)
        if self.fault_latched:
            blockers.insert(0, self._fault_reason or 'fault_latched')
        if self._stopping_at is not None:
            blockers.append('awaiting_disabled')
        ready = not blockers
        if self.fault_latched:
            state = self.FAULT
        elif self._stopping_at is not None:
            state = self.HOLD
        elif self.armed:
            state = self.RUNNING if any(s['execution_state'] in (3, 4)
                                       for s in self._control.values()) else self.ARMED
        elif ready:
            state = self.READY
        else:
            state = self.BOOTING if not self.session_id else self.NOT_READY
        calibration = self._calibration or {}
        return {'state': state, 'ready': ready, 'armed': self.armed,
                'fault_latched': self.fault_latched, 'epoch': self.epoch,
                'session_id': self.session_id, 'owner': self._owner, 'mode': self._mode,
                'lease_remaining_ms': max(0, math.ceil((self._expires-now)*1000))
                if self._token else 0,
                'calibration_id': str(calibration.get('calibration_id', '')),
                'config_hash': str(calibration.get('config_hash', '')),
                'motor_ids': sorted(self._expected), 'blockers': list(dict.fromkeys(blockers))}

    def _lease_error(self, lease_id, epoch, now):
        if not self._token:
            return 'no_active_control_lease'
        if (not isinstance(lease_id, str) or not lease_id or not lease_id.isascii()
                or not secrets.compare_digest(lease_id, self._token)
                or not _uint(epoch, _U64, 1) or epoch != self.epoch):
            return 'control_lease_mismatch'
        if now >= self._expires:
            return 'control_lease_expired'
        return ''

    def set_external_blockers(self, blockers):
        """Fail closed on runtime conditions checked by the ROS adapter.

        For example, duplicate publishers or missing authoritative sources are
        graph-level properties; the pure engine has no DDS discovery access.
        """
        if (not isinstance(blockers, (list, tuple))
                or any(not isinstance(reason, str) or not reason.strip()
                       for reason in blockers)):
            incoming = ['invalid_external_blockers']
        else:
            incoming = list(dict.fromkeys(blockers))
        if incoming != self._external_blockers:
            self._external_blockers = incoming
            self.tick()

    def _valid_lease_ms(self, lease_ms):
        return _uint(lease_ms, self.max_lease_ms, self.min_lease_ms)

    def acquire(self, owner, mode, lease_ms):
        self.tick()
        if (not isinstance(owner, str) or not owner.strip() or len(owner) > 128
                or any(ord(char) < 32 for char in owner)):
            return _result(False, 'owner_must_be_nonempty_printable_text')
        if type(mode) is not int or mode not in (1, 2, 3):
            return _result(False, 'invalid_control_mode')
        if not self._valid_lease_ms(lease_ms):
            return _result(False, 'lease_duration_out_of_range')
        if self._token:
            return _result(False, 'control_already_owned')
        status = self.snapshot()
        if not status['ready'] or not self._stable() or not self._all_disabled():
            return _result(False, 'robot_not_ready:' + ','.join(status['blockers']))
        self._owner, self._mode = owner.strip(), mode
        self._token = secrets.token_urlsafe(32)
        self._expires = self._now() + lease_ms / 1000.
        self._last_input_sequence = -1
        return _result(True, lease_id=self._token, epoch=self.epoch,
                       session_id=self.session_id, lease_ms=lease_ms)

    def renew(self, lease_id, epoch, lease_ms):
        self.tick()
        now = self._now()
        error = self._lease_error(lease_id, epoch, now)
        if error:
            return _result(False, error)
        if not self._valid_lease_ms(lease_ms):
            return _result(False, 'lease_duration_out_of_range')
        self._expires = now + lease_ms / 1000.
        return _result(True, lease_remaining_ms=lease_ms)

    def release(self, lease_id, epoch):
        self.tick()
        now = self._now()
        error = self._lease_error(lease_id, epoch, now)
        if error:
            return _result(False, error)
        self._request_stop('control_released', now)
        self._clear_lease()
        return _result(True)

    def set_armed(self, lease_id, epoch, armed):
        self.tick()
        now = self._now()
        error = self._lease_error(lease_id, epoch, now)
        if error:
            return _result(False, error)
        if type(armed) is not bool:
            return _result(False, 'armed_must_be_boolean')
        if not armed:
            self.armed = False
            self._request_stop('operator_disarmed', now)
            return _result(True)
        if self.armed:
            return _result(True)
        status = self.snapshot()
        if (not status['ready'] or not self._stable() or not self._all_disabled()):
            return _result(False, 'robot_not_stably_disabled')
        self.armed = True
        return _result(True)

    def stop(self, reason='operator_stop'):
        self._latch_fault(str(reason) or 'operator_stop', self._now())

    def reset_fault(self):
        self.tick()
        now = self._now()
        if self._token or self.armed:
            return _result(False, 'release_control_before_reset')
        if (self._base_blockers(now) or not self._stable() or not self._all_disabled()
                or self._stopping_at is not None):
            return _result(False, 'fresh_healthy_disabled_state_required')
        self.fault_latched, self._fault_reason = False, ''
        self._stopping_at = None
        return _result(True)

    def lease_matches(self, lease_id, epoch, session_id):
        self.tick()
        return bool(not self._lease_error(lease_id, epoch, self._now())
                    and _uint(session_id, _U64, 1) and session_id == self.session_id
                    and not self.fault_latched and not self._base_blockers(self._now()))

    @property
    def input_sequence(self):
        return self._last_input_sequence

    def authorize(self, lease_id, epoch, session_id, sequence, channel):
        self.tick()
        if not self.lease_matches(lease_id, epoch, session_id):
            return _result(False, 'invalid_or_unhealthy_control_lease')
        if not self.armed:
            return _result(False, 'robot_not_armed')
        if channel not in ('command', 'policy', 'trajectory'):
            return _result(False, 'invalid_command_channel')
        if channel == 'trajectory' and self._mode == 1:
            return _result(False, 'trajectory_requires_trajectory_or_task_mode')
        if not _uint(sequence, _U64) or sequence <= self._last_input_sequence:
            return _result(False, 'stale_input_sequence')
        self._last_input_sequence = sequence
        return _result(True)

    def next_policy_generation(self):
        if self._generation >= _U32:
            raise ValueError('policy_generation_exhausted_restart_after_mcu_reset_required')
        self._generation += 1
        return self._generation

    def take_stop_request(self):
        reason, self._stop_pending = self._stop_pending, None
        return reason
