"""重力参数适配的离线测试：真实传动解析、假时钟和假参数 future。"""
from concurrent.futures import Future
from types import SimpleNamespace

import pytest

from bench_manager.gravity_feedforward_manager import (
    CONTROL_STATUS_TIMEOUT_S, PARAMETER_TIMEOUT_S,
    GravityFeedforwardManager, validate_joint_map,
)

_MODEL = 'arm'
_JOINTS = ['yaw', 'wrist_a', 'wrist_b']


class Writer:
    def __init__(self):
        self.calls = []
        self.futures = []

    def __call__(self, values):
        self.calls.append(values)
        future = Future()
        self.futures.append(future)
        return future

    def reply(self, successful=True, reason=''):
        self.futures[-1].set_result(SimpleNamespace(result=SimpleNamespace(
            successful=successful, reason=reason)))


@pytest.fixture
def models(tmp_path):
    config = tmp_path / _MODEL / 'config'
    config.mkdir(parents=True)
    (config / 'motor_driver.yaml').write_text('''motor_hardware:
  motors:
    a: {id: 1, joint: yaw}
    b: {id: 2}
    c: {id: 3}
''')
    (config / 'transmissions.yaml').write_text('''transmissions:
  wrist:
    plugin: transmission_interface/DifferentialTransmission
    actuators: [b, c]
    joints: [wrist_a, wrist_b]
''')
    return tmp_path


def _control(**overrides):
    return dict(model=_MODEL, joint_names=_JOINTS, mode='trajectory',
                gravity_enabled=False, effort_gain=1.0) | overrides


def _manager(models):
    writer, now = Writer(), [100.0]
    manager = GravityFeedforwardManager(models, writer, clock=lambda: now[0])
    manager.observe(_control())
    return manager, writer, now


def _request(**overrides):
    return dict(model=_MODEL, effort_gain=0.6) | overrides


def test_start_writes_only_runtime_parameters_and_waits_for_real_status(models):
    manager, writer, _ = _manager(models)
    result = manager.start(_request(joint_map='1:yaw'))
    assert writer.calls == [{'enabled': True, 'effort_gain': 0.6}]
    assert result['pending'] == 'start'
    assert result['state'] == 'stopped'
    assert result['config']['effort_gain'] == 1.0
    writer.reply()
    assert manager.status()['pending'] == 'start'
    manager.observe(_control(gravity_enabled=True, effort_gain=0.6))
    assert manager.status()['pending'] is None
    assert manager.status()['state'] == 'running'
    assert manager.status()['config'] == {'model': _MODEL, 'effort_gain': 0.6}


@pytest.mark.parametrize('spec', ['', '1:yaw'])
def test_joint_map_is_optional_and_validated_when_supplied(models, spec):
    manager, writer, _ = _manager(models)
    manager.start(_request(joint_map=spec))
    assert len(writer.calls) == 1


@pytest.mark.parametrize('spec', [None, '1:wrong', '2:wrist_a', '1:yaw,2:wrist_a',
                                   '9:yaw', '1:yaw,1:yaw', '1', '1:', ':yaw',
                                   'x:j', '70000:yaw', '-1:yaw', '1: yaw', '1:yaw:x'])
def test_joint_map_rejects_malformed_wrong_or_flattened_coupled_groups(models, spec):
    manager, writer, _ = _manager(models)
    with pytest.raises(ValueError):
        manager.start(_request(joint_map=spec))
    assert not writer.calls


def test_validate_joint_map_returns_numeric_ids():
    assert validate_joint_map('1:a,10:b') == {1: 'a', 10: 'b'}


@pytest.mark.parametrize('gain', [-0.1, 1.01, float('nan'), float('inf'), 10**400, True, '0.5', None])
def test_invalid_gain_never_reaches_the_controller(models, gain):
    manager, writer, _ = _manager(models)
    with pytest.raises(ValueError, match='0..1'):
        manager.start(_request(effort_gain=gain))
    assert not writer.calls


@pytest.mark.parametrize('model', ['', None, 'other', '../arm'])
def test_start_cannot_change_models(models, model):
    manager, writer, _ = _manager(models)
    with pytest.raises(ValueError, match='model'):
        manager.start(_request(model=model))
    assert not writer.calls


def test_start_rejects_mismatched_joint_topology(models):
    manager, writer, _ = _manager(models)
    manager.observe(_control(joint_names=['other']))
    with pytest.raises(ValueError, match='joint_names'):
        manager.start(_request())
    assert not writer.calls


def test_start_requires_fresh_real_status(models):
    manager, writer, now = _manager(models)
    now[0] += CONTROL_STATUS_TIMEOUT_S + 0.01
    with pytest.raises(RuntimeError, match='stale'):
        manager.start(_request())
    assert not writer.calls


@pytest.mark.parametrize('mode,enabled,state', [
    ('trajectory', True, 'running'), ('trajectory', False, 'stopped'),
    ('follow', True, 'running'), ('follow', False, 'stopped'),
    ('manual', True, 'stopped'), ('stopped', True, 'stopped'),
])
def test_restart_restores_only_observed_active_enabled_state(models, mode, enabled, state):
    manager = GravityFeedforwardManager(models)
    manager.observe(_control(mode=mode, gravity_enabled=enabled, effort_gain=0.25))
    assert manager.status()['state'] == state
    assert manager.status()['config'] == {'model': _MODEL, 'effort_gain': 0.25}


def test_running_state_expires_at_2_5_seconds(models):
    manager, _, now = _manager(models)
    manager.observe(_control(gravity_enabled=True))
    now[0] += CONTROL_STATUS_TIMEOUT_S
    assert manager.status()['state'] == 'running'
    now[0] += 0.01
    assert manager.status()['state'] == 'stopped'
    assert not manager.status()['fresh']
    assert 'stale' in manager.status()['error']


def test_stop_disables_and_zeros_effort_without_fresh_status(models):
    writer = Writer()
    manager = GravityFeedforwardManager(models, writer)
    result = manager.stop()
    assert writer.calls == [{'enabled': False, 'effort_gain': 0.0}]
    assert result['pending'] == 'stop'


def test_stop_waits_for_controller_readback_not_the_response(models):
    manager, writer, _ = _manager(models)
    manager.observe(_control(gravity_enabled=True))
    manager.stop()
    writer.reply()
    assert manager.status()['state'] == 'running'
    assert manager.status()['pending'] == 'stop'
    manager.observe(_control(gravity_enabled=False, effort_gain=0.0))
    assert manager.status()['state'] == 'stopped'
    assert manager.status()['pending'] is None


def test_gain_requires_running_controller(models):
    manager, writer, _ = _manager(models)
    with pytest.raises(RuntimeError, match='not running'):
        manager.set_gain(0.4)
    assert not writer.calls


def test_gain_is_not_cached_from_intent_or_successful_response(models):
    manager, writer, _ = _manager(models)
    manager.observe(_control(gravity_enabled=True))
    manager.set_gain(0.4)
    assert writer.calls == [{'effort_gain': 0.4}]
    writer.reply()
    assert manager.status()['config']['effort_gain'] == 1.0
    manager.observe(_control(gravity_enabled=True, effort_gain=0.4))
    assert manager.status()['config']['effort_gain'] == 0.4
    assert manager.status()['pending'] is None


def test_rejection_reports_error_and_never_changes_observed_values(models):
    manager, writer, _ = _manager(models)
    manager.observe(_control(gravity_enabled=True))
    manager.set_gain(0.4)
    writer.reply(False, 'refused')
    assert manager.status()['error'] == manager.status()['gain_error'] == 'refused'
    assert manager.status()['config']['effort_gain'] == 1.0
    assert manager.status()['pending'] is None


def test_future_exception_is_reported(models):
    manager, writer, _ = _manager(models)
    manager.start(_request())
    writer.futures[-1].set_exception(RuntimeError('transport gone'))
    assert manager.status()['error'] == 'transport gone'
    assert manager.status()['state'] == 'stopped'


@pytest.mark.parametrize('acknowledge', [False, True])
def test_unanswered_or_unconfirmed_requests_time_out(models, acknowledge):
    manager, writer, now = _manager(models)
    manager.start(_request())
    if acknowledge:
        writer.reply()
    now[0] += PARAMETER_TIMEOUT_S
    manager.tick()
    assert manager.status()['pending'] is None
    assert 'timed out' in manager.status()['error']
    assert manager.status()['state'] == 'stopped'


def test_timeout_late_reply_cannot_complete_a_new_request(models):
    manager, writer, now = _manager(models)
    manager.start(_request())
    old = manager._pending
    now[0] += PARAMETER_TIMEOUT_S
    manager.tick()
    manager.observe(_control())
    manager.stop()
    late = Future()
    late.set_result(SimpleNamespace(result=SimpleNamespace(successful=True)))
    manager._written(old, late)
    assert manager.status()['pending'] == 'stop'
    assert writer.calls[-1] == {'enabled': False, 'effort_gain': 0.0}
    assert writer.futures[0].cancelled()


def test_busy_request_does_not_send_an_overlapping_write(models):
    manager, writer, _ = _manager(models)
    manager.start(_request())
    with pytest.raises(RuntimeError, match='pending'):
        manager.stop()
    assert len(writer.calls) == 1


def test_status_before_ack_still_requires_successful_response(models):
    manager, writer, _ = _manager(models)
    manager.start(_request())
    manager.observe(_control(gravity_enabled=True, effort_gain=0.6))
    assert manager.status()['pending'] == 'start'
    writer.reply(False, 'refused')
    assert manager.status()['error'] == 'refused'


def test_close_cancels_pending_and_invalidates_running_state(models):
    manager, writer, _ = _manager(models)
    manager.observe(_control(gravity_enabled=True))
    manager.set_gain(0.5)
    manager.close()
    assert writer.futures[-1].cancelled()
    assert manager.status()['state'] == 'stopped'
    assert manager.status()['pending'] is None


@pytest.mark.parametrize('changed', [dict(model='other'), dict(joint_names=['other'])])
@pytest.mark.parametrize('acknowledge', [False, True])
def test_model_or_topology_change_cannot_confirm_a_previous_request(models, changed, acknowledge):
    manager, writer, _ = _manager(models)
    manager.start(_request())
    if acknowledge:
        writer.reply()
    manager.observe(_control(gravity_enabled=True, effort_gain=0.6, **changed))
    assert manager.status()['pending'] is None
    assert 'changed' in manager.status()['error']
    assert writer.futures[-1].done()


def test_empty_exception_still_reports_a_failure(models):
    manager, writer, _ = _manager(models)
    manager.start(_request())
    writer.futures[-1].set_exception(RuntimeError())
    assert manager.status()['error']
    assert manager.status()['pending'] is None
