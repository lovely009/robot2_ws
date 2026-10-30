"""Deterministic safety and lease tests; no ROS, processes or hardware."""
import copy

import pytest

from robot_supervisor.core import SupervisorCore


class Clock:
    def __init__(self):
        self.value = 100.

    def __call__(self):
        return self.value

    def advance(self, seconds):
        self.value += seconds


def calibration():
    return {'robot_id': 'bench-01', 'model_name': 'arm',
            'calibration_id': 'accepted-001', 'config_hash': 'abc123',
            'valid': True, 'commissioned': True,
            'motor_ids': [1, 2], 'bus_indices': [0, 1], 'protocols': [1, 0],
            'model_names': ['cerb_r48_101', 'vaxor_2550'],
            'joint_names': ['shoulder', 'wrist'], 'blockers': []}


def state(mid, **changes):
    result = {'motor_id': mid, 'online': True, 'control_lifecycle': 2,
              'execution_state': 1, 'policy_enabled': False, 'motion_enabled': False,
              'state_age_ms': 0, 'unified_fault_code': 0, 'vendor_fault_code': 0,
              'last_failure_reason': 0, 'policy_generation': 9}
    result.update(changes)
    return result


def control(sequence=1, session=1_800_000_000_000, states=None):
    return {'schema_version': 2, 'session_id': session, 'sequence': sequence,
            'states': [state(1), state(2)] if states is None else states}


def topology(session=1_800_000_000_000):
    return {'schema_version': 3, 'session_id': session, 'entries': [
        {'motor_id': 1, 'bus_index': 0, 'protocol': 1, 'model_name': 'cerb_r48_101',
         'online': True, 'conflict': False},
        {'motor_id': 2, 'bus_index': 1, 'protocol': 0, 'model_name': 'vaxor_2550',
         'online': True, 'conflict': False}]}


def parameters(session=1_800_000_000_000):
    return {'session_id': session, 'count': 1, 'entries': [
        {'motor_id': 1, 'dev_init_state': 2, 'record_dirty': False,
         'mismatch_mask': 0, 'dev_safety': 0.0}]}


def setup_core():
    clock = Clock()
    core = SupervisorCore('bench-01', 'arm', clock=clock, epoch=17)
    core.update_calibration(calibration())
    core.update_topology(topology())
    core.update_device_params(parameters())
    core.update_control(control())
    assert core.snapshot()['ready']
    return core, clock


def own(core, mode=1, duration=1000):
    lease = core.acquire('operator', mode, duration)
    assert lease['success'], lease
    return lease


def arm(core, lease):
    assert core.set_armed(lease['lease_id'], lease['epoch'], True)['success']


def authorize(core, lease, sequence, channel='command'):
    return core.authorize(lease['lease_id'], lease['epoch'], lease['session_id'],
                          sequence, channel)


def test_boot_requires_fresh_verified_calibration_and_control():
    core = SupervisorCore(clock=Clock())
    status = core.snapshot()
    assert status['state'] == core.BOOTING
    assert not status['ready'] and not status['armed']
    assert not core.acquire('operator', 1, 1000)['success']


def test_owner_is_exclusive_and_arm_is_separate_from_acquisition():
    core, _ = setup_core()
    lease = own(core)
    assert not core.snapshot()['armed']
    assert not authorize(core, lease, 0)['success']
    assert not core.acquire('second browser', 2, 1000)['success']
    arm(core, lease)
    assert authorize(core, lease, 0)['success']
    assert core.snapshot()['state'] == core.ARMED
    assert 'lease_id' not in core.snapshot()


def test_command_sequence_is_shared_across_channels_and_never_replayed():
    core, _ = setup_core()
    lease = own(core, mode=3)
    arm(core, lease)
    assert authorize(core, lease, 10, 'policy')['success']
    assert not authorize(core, lease, 10, 'command')['success']
    assert not authorize(core, lease, 9, 'trajectory')['success']
    assert not authorize(core, lease, True)['success']
    assert authorize(core, lease, 11, 'trajectory')['success']


def test_manual_owner_cannot_start_trajectory():
    core, _ = setup_core()
    lease = own(core)
    arm(core, lease)
    assert not authorize(core, lease, 1, 'trajectory')['success']
    assert authorize(core, lease, 1, 'policy')['success']


@pytest.mark.parametrize('owner,mode,lease_ms', [('', 1, 1000), ('x\ny', 1, 1000),
    ('x', True, 1000), ('x', 4, 1000), ('x', 1, 99), ('x', 1, 2001),
    ('x', 1, True), ('x', 1, float('nan'))])
def test_acquire_rejects_invalid_owner_mode_or_lease_duration(owner, mode, lease_ms):
    core, _ = setup_core()
    assert not core.acquire(owner, mode, lease_ms)['success']


def test_current_token_epoch_and_session_all_required():
    core, _ = setup_core()
    lease = own(core)
    arm(core, lease)
    assert not core.authorize('wrong', lease['epoch'], lease['session_id'], 1,
                              'command')['success']
    assert not core.authorize('非ASCII租约', lease['epoch'], lease['session_id'], 1,
                              'command')['success']
    assert not core.authorize(lease['lease_id'], lease['epoch'] + 1,
                              lease['session_id'], 1, 'command')['success']
    assert not core.authorize(lease['lease_id'], lease['epoch'],
                              lease['session_id'] + 1, 1, 'command')['success']
    assert authorize(core, lease, 1)['success']


def test_renew_cannot_extend_an_expired_lease_or_rearm():
    core, clock = setup_core()
    lease = own(core, duration=200)
    assert core.renew(lease['lease_id'], lease['epoch'], 500)['success']
    assert not core.snapshot()['armed']
    clock.advance(.501)
    assert not core.renew(lease['lease_id'], lease['epoch'], 500)['success']
    assert core.snapshot()['fault_latched']
    assert not core.snapshot()['owner']
    assert core.take_stop_request() == 'control_lease_expired'


def test_lease_expiry_needs_reset_and_fresh_explicit_acquisition():
    core, clock = setup_core()
    lease = own(core, duration=200)
    arm(core, lease)
    clock.advance(.201)
    core.tick()
    assert not authorize(core, lease, 1)['success']
    assert core.snapshot()['fault_latched']
    assert not core.reset_fault()['success']
    core.update_control(control(2))
    assert core.reset_fault()['success']
    assert not core.snapshot()['armed']
    replacement = own(core)
    assert replacement['lease_id'] != lease['lease_id']
    assert not authorize(core, replacement, 1)['success']


def test_readiness_loss_stops_owner_even_before_arming():
    core, _ = setup_core()
    lease = own(core)
    core.update_control(control(2, states=[state(1, online=False), state(2)]))
    assert core.snapshot()['fault_latched']
    assert not core.lease_matches(lease['lease_id'], lease['epoch'], lease['session_id'])
    assert core.take_stop_request().startswith('readiness_lost:')


def test_duplicate_and_old_control_messages_do_not_renew_freshness():
    core, clock = setup_core()
    clock.advance(.4)
    assert not core.update_control(control())
    assert not core.update_control(control(sequence=0))
    clock.advance(.101)
    assert 'control_state_stale' in core.snapshot()['blockers']
    assert core.update_control(control(sequence=2))
    assert core.snapshot()['ready']


def test_control_sequence_wrap_is_accepted_but_session_does_not_truncate():
    core, _ = setup_core()
    # More than half a uint32 cycle is correctly treated as stale, so set up a
    # fresh session with the high sequence before testing its valid wrap.
    session = 1_800_000_000_001
    assert core.update_control(control(sequence=0xFFFFFFFF, session=session))
    assert core.update_control(control(sequence=0, session=session))
    assert core.session_id == session
    assert not core.update_control(control(sequence=0xFFFFFFFF, session=session))


def test_new_session_retires_lease_and_all_old_session_caches():
    core, _ = setup_core()
    lease = own(core)
    arm(core, lease)
    session = lease['session_id'] + 1
    core.update_control(control(sequence=1, session=session))
    status = core.snapshot()
    assert status['session_id'] == session and status['fault_latched']
    assert not status['owner'] and not status['armed']
    assert 'topology_missing_or_invalid' in status['blockers']
    assert not core.update_control(control(sequence=99))
    assert not core.update_topology(topology())
    core.update_topology(topology(session))
    core.update_device_params(parameters(session))
    assert core.reset_fault()['success']
    assert not authorize(core, lease, 100)['success']


@pytest.mark.parametrize('field,value', [('valid', False), ('commissioned', False),
    ('robot_id', 'another robot'), ('model_name', 'another model'),
    ('blockers', ['zero_not_verified']), ('motor_ids', [1, 1]),
    ('protocols', [1, 3]), ('bus_indices', [0]), ('calibration_id', '')])
def test_calibration_rejections_block_startup(field, value):
    core, _ = setup_core()
    report = calibration()
    report[field] = value
    core.update_calibration(report)
    assert not core.snapshot()['ready']
    assert not core.acquire('operator', 1, 1000)['success']


def test_calibration_file_change_revokes_existing_owner():
    core, _ = setup_core()
    lease = own(core)
    changed = calibration()
    changed['config_hash'] = 'different-config'
    core.update_calibration(changed)
    assert core.snapshot()['fault_latched']
    assert core.take_stop_request() == 'calibration_changed'
    assert not core.lease_matches(lease['lease_id'], lease['epoch'], lease['session_id'])


def test_calibration_heartbeat_expires_separately_from_control():
    core, clock = setup_core()
    for sequence in range(2, 7):
        clock.advance(.31)
        core.update_control(control(sequence))
    assert 'calibration_stale' in core.snapshot()['blockers']


@pytest.mark.parametrize('change', ['bus', 'protocol', 'model', 'conflict', 'extra'])
def test_topology_identity_conflict_or_unexpected_motor_prevents_motion(change):
    core, _ = setup_core()
    report = topology()
    if change == 'extra':
        report['entries'].append({**report['entries'][0], 'motor_id': 3})
    else:
        key, value = {'bus': ('bus_index', 2), 'protocol': ('protocol', 0),
                      'model': ('model_name', 'wrong'), 'conflict': ('conflict', True)}[change]
        report['entries'][0][key] = value
    core.update_topology(report)
    assert not core.snapshot()['ready']


@pytest.mark.parametrize('field,value', [('dev_init_state', 3), ('record_dirty', True),
    ('mismatch_mask', 32), ('dev_safety', 1.0), ('dev_safety', float('nan'))])
def test_cerb_parameter_mismatch_dirty_or_disabled_protection_blocks(field, value):
    core, _ = setup_core()
    report = parameters()
    report['entries'][0][field] = value
    core.update_device_params(report)
    assert not core.snapshot()['ready']


def test_device_params_and_topology_are_events_not_heartbeats():
    core, clock = setup_core()
    for sequence in range(2, 22):
        clock.advance(.2)
        core.update_control(control(sequence))
        core.update_calibration(calibration())
    assert core.snapshot()['ready']


def test_malformed_new_control_does_not_reuse_previous_healthy_authorization():
    core, _ = setup_core()
    lease = own(core)
    arm(core, lease)
    report = control(2)
    report['states'][0]['online'] = 1  # bool is a distinct part of the contract.
    assert not core.update_control(report)
    assert core.snapshot()['fault_latched']
    assert not authorize(core, lease, 1)['success']


def test_feedback_age_is_added_to_receipt_age():
    core, clock = setup_core()
    core.update_control(control(2, states=[state(1, state_age_ms=300), state(2)]))
    assert core.snapshot()['ready']
    clock.advance(.201)
    assert 'motor_1_feedback_stale' in core.snapshot()['blockers']


def test_initial_enabled_drive_without_owner_is_stopped_and_reset_requires_ack():
    core, _ = setup_core()
    core.update_control(control(2, states=[state(1, policy_enabled=True), state(2)]))
    assert core.snapshot()['fault_latched']
    assert core.take_stop_request() == 'unexpected_motion_without_owner'
    assert not core.reset_fault()['success']
    core.update_control(control(3))
    assert core.reset_fault()['success']
    assert not core.snapshot()['armed']


def test_expected_transition_lifecycles_do_not_revoke_valid_owner():
    core, _ = setup_core()
    lease = own(core)
    arm(core, lease)
    core.update_control(control(2, states=[state(1, control_lifecycle=13), state(2)]))
    assert not core.snapshot()['fault_latched']
    assert core.snapshot()['armed']
    assert core.set_armed(lease['lease_id'], lease['epoch'], False)['success']
    core.update_control(control(3, states=[state(1, control_lifecycle=14), state(2)]))
    assert not core.set_armed(lease['lease_id'], lease['epoch'], True)['success']
    core.update_control(control(4))
    assert core.set_armed(lease['lease_id'], lease['epoch'], True)['success']


def test_release_sends_stop_and_waits_for_new_disabled_ack_without_false_fault():
    core, _ = setup_core()
    lease = own(core)
    arm(core, lease)
    core.update_control(control(2, states=[state(1, policy_enabled=True,
                                                 motion_enabled=True), state(2)]))
    assert core.release(lease['lease_id'], lease['epoch'])['success']
    assert core.take_stop_request() == 'control_released'
    assert not core.snapshot()['fault_latched']
    assert not core.acquire('new-owner', 1, 1000)['success']
    core.update_control(control(3))
    assert core.snapshot()['ready']
    assert own(core)['lease_id'] != lease['lease_id']


def test_pending_enable_cannot_use_pre_stop_disabled_snapshot_to_transfer_control():
    core, _ = setup_core()
    lease = own(core)
    arm(core, lease)
    # The policy is authorized, but the MCU's latest report still describes the
    # disabled state from before the enable was put on the wire.
    assert authorize(core, lease, 1, 'policy')['success']
    assert core.release(lease['lease_id'], lease['epoch'])['success']
    assert core.snapshot()['state'] == core.HOLD
    assert not core.acquire('next-owner', 1, 1000)['success']
    # A duplicate old report is not a stop acknowledgement either.
    assert not core.update_control(control())
    assert not core.acquire('next-owner', 1, 1000)['success']
    core.update_control(control(2, states=[state(1, policy_enabled=True), state(2)]))
    assert not core.acquire('next-owner', 1, 1000)['success']
    core.update_control(control(3))
    assert core.acquire('next-owner', 1, 1000)['success']


def test_partial_disabled_report_cannot_acknowledge_stop_for_a_missing_axis():
    core, clock = setup_core()
    lease = own(core)
    arm(core, lease)
    core.release(lease['lease_id'], lease['epoch'])
    core.take_stop_request()
    core.update_control(control(2, states=[state(1)]))
    assert 'awaiting_disabled' in core.snapshot()['blockers']
    assert core.take_stop_request() is None
    clock.advance(.501)
    core.tick()
    assert core.snapshot()['fault_latched']
    core.update_control(control(3))
    assert core.reset_fault()['success']


def test_waiting_for_disable_ack_does_not_request_a_new_stop_each_tick():
    core, clock = setup_core()
    lease = own(core)
    arm(core, lease)
    core.release(lease['lease_id'], lease['epoch'])
    assert core.take_stop_request() == 'control_released'
    for _ in range(10):
        clock.advance(.01)
        core.tick()
        assert core.take_stop_request() is None
        assert core.snapshot()['state'] == core.HOLD
    clock.advance(.401)
    core.tick()
    assert core.take_stop_request() == 'disable_not_confirmed'
    for _ in range(3):
        core.tick()
        assert core.take_stop_request() is None
    assert core.snapshot()['fault_latched']


def test_stop_is_unconditional_latched_and_reset_never_auto_enables():
    core, _ = setup_core()
    lease = own(core)
    arm(core, lease)
    core.stop()
    assert not core.snapshot()['owner'] and not core.snapshot()['armed']
    assert core.take_stop_request() == 'operator_stop'
    assert not core.reset_fault()['success']
    core.update_control(control(2))
    assert core.reset_fault()['success']
    assert not authorize(core, lease, 1)['success']
    assert not core.snapshot()['armed']


def test_policy_generation_tracks_authority_does_not_wrap_and_is_retained_for_stop():
    core, _ = setup_core()
    assert core.policy_generation == 9
    assert core.next_policy_generation() == 10
    core.update_control(control(2))
    assert core.next_policy_generation() == 11
    core.update_control(control(3, states=[state(1, policy_generation=0xFFFFFFFF), state(2)]))
    with pytest.raises(ValueError, match='exhausted'):
        core.next_policy_generation()
    core.stop()
    assert core.policy_generation == 0xFFFFFFFF


def test_new_agent_session_preserves_policy_high_water_before_control_arrives():
    core, _ = setup_core()
    lease = own(core)
    arm(core, lease)
    assert core.next_policy_generation() == 10
    core.update_topology(topology(session=lease['session_id'] + 1))
    assert core.take_stop_request() == 'mcu_session_changed'
    assert core.policy_generation == 10
    core.update_control(control(sequence=1, session=lease['session_id'] + 1))
    assert core.next_policy_generation() == 11


def test_restart_epoch_and_random_token_retire_all_previous_client_envelopes():
    core, _ = setup_core()
    old = own(core)
    replacement, _ = setup_core()
    replacement.epoch = 18
    current = own(replacement)
    arm(replacement, current)
    assert not authorize(replacement, old, 1)['success']
    assert current['lease_id'] != old['lease_id']


def test_monotonic_clock_rollback_cannot_renew_health():
    core, clock = setup_core()
    lease = own(core)
    clock.advance(-1.)
    core.tick()
    assert core.snapshot()['fault_latched']
    assert not core.renew(lease['lease_id'], lease['epoch'], 1000)['success']


@pytest.mark.parametrize('method,report', [('update_control', {}),
    ('update_topology', {'schema_version': 3, 'session_id': 123, 'entries': [None]}),
    ('update_device_params', {'session_id': 123, 'count': 1, 'entries': [None]}),
    ('update_calibration', None)])
def test_malformed_reports_do_not_raise_or_grant_readiness(method, report):
    core = SupervisorCore(clock=Clock())
    assert getattr(core, method)(copy.deepcopy(report)) is False
    assert not core.snapshot()['ready']


def test_external_runtime_blocker_revokes_owner_and_requires_explicit_reset():
    core, _ = setup_core()
    lease = own(core)
    arm(core, lease)
    core.set_external_blockers(['duplicate_bridge_publisher'])
    assert core.snapshot()['fault_latched']
    assert 'duplicate_bridge_publisher' in core.snapshot()['blockers']
    assert not authorize(core, lease, 1)['success']
    core.set_external_blockers([])
    assert core.snapshot()['fault_latched']
    assert not core.reset_fault()['success']
    core.update_control(control(2))
    assert core.reset_fault()['success']
    assert core.snapshot()['ready'] and not core.snapshot()['armed']


@pytest.mark.parametrize('value', [None, [''], [False]])
def test_malformed_external_conditions_fail_closed(value):
    core, _ = setup_core()
    core.set_external_blockers(value)
    assert 'invalid_external_blockers' in core.snapshot()['blockers']
