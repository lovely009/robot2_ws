"""模型反馈只读起停与发布者交接；不启动节点或 systemd。"""
from pathlib import Path

import pytest

from conftest import FakeSystemd
from bench_manager.feedback_manager import FirmwareFeedbackManager
from bench_manager.systemd_units import SystemdUnits, UNIT_FEEDBACK


FeedbackSetup = tuple[FirmwareFeedbackManager, FakeSystemd]


@pytest.fixture
def setup_feedback(tmp_path: Path) -> FeedbackSetup:
    for name in ('arm_a', 'arm_b'):
        model = tmp_path / name
        (model / 'config').mkdir(parents=True)
        (model / 'urdf').mkdir()
        (model / 'config' / 'motor_driver.yaml').write_text(
            'motor_hardware:\n  motors:\n    m1: {id: 1, joint: elbow}\n')
        (model / 'urdf' / f'{name}.urdf').write_text(
            '<robot name="arm"><link name="base"/><link name="tip"/>'
            '<joint name="elbow" type="continuous"><parent link="base"/>'
        '<child link="tip"/><limit velocity="1" effort="10"/></joint></robot>')
    fake = FakeSystemd({UNIT_FEEDBACK: {'LoadState': 'not-found'}})
    return FirmwareFeedbackManager(tmp_path, SystemdUnits(runner=fake)), fake


def test_agent_and_model_start_control_session_without_enabling_once(setup_feedback: FeedbackSetup) -> None:
    manager, fake = setup_feedback
    manager.reconcile('arm_a', agent_running=False)
    assert manager.status()['state'] == 'stopped'
    manager.reconcile('arm_a', agent_running=True)
    manager.reconcile('arm_a', agent_running=True)
    starts = [c for c in fake.commands() if c[0] == 'systemd-run']
    assert len(starts) == 1
    assert 'model:=arm_a' in starts[0]
    assert 'host_trajectory.launch.py' in starts[0]
    assert 'start_controllers:=false' in starts[0]
    assert not any('enable' in arg and 'true' in arg for arg in starts[0])
    assert manager.status()['model'] == 'arm_a'
    assert manager.status()['state'] == 'running'


def test_model_switch_replaces_only_feedback_unit(setup_feedback: FeedbackSetup) -> None:
    manager, fake = setup_feedback
    manager.reconcile('arm_a', agent_running=True)
    manager.reconcile('arm_b', agent_running=True)
    assert manager.status()['model'] == 'arm_b'
    stops = [c for c in fake.commands() if c[:3] == ['systemctl', '--user', 'stop']]
    assert stops == [['systemctl', '--user', 'stop', UNIT_FEEDBACK]]


@pytest.mark.parametrize('context', [
    {'agent_running': False},
    {'agent_running': True, 'simulation_running': True},
    {'agent_running': True, 'external_publishers': ['external_bridge']},
])
def test_other_owner_or_stopped_agent_releases_feedback(setup_feedback: FeedbackSetup, context: dict) -> None:
    manager, _fake = setup_feedback
    manager.reconcile('arm_a', agent_running=True)
    manager.reconcile('arm_a', **context)
    assert manager.status()['state'] != 'running'
    manager.reconcile('arm_a', agent_running=True)
    assert manager.status()['state'] == 'running'


def test_invalid_new_model_stops_old_feedback_and_reports_error(setup_feedback: FeedbackSetup) -> None:
    manager, fake = setup_feedback
    manager.reconcile('arm_a', agent_running=True)
    manager.reconcile('missing', agent_running=True)
    assert manager.status()['state'] == 'failed'
    assert manager.status()['desired_model'] == 'missing'
    assert manager.status()['model'] == ''
    assert manager.status()['error']
    before = len(fake.commands())
    manager.reconcile('missing', agent_running=True)
    assert not [c for c in fake.commands()[before:] if c[0] == 'systemd-run']


def test_manager_restart_adopts_matching_unit(setup_feedback: FeedbackSetup) -> None:
    manager, fake = setup_feedback
    manager.reconcile('arm_a', agent_running=True)
    adopted = FirmwareFeedbackManager(manager.models_root, SystemdUnits(runner=fake))
    adopted.reconcile('arm_a', agent_running=True)
    assert adopted.status()['model'] == 'arm_a'
    assert len([c for c in fake.commands() if c[0] == 'systemd-run']) == 1


def test_old_feedback_unit_is_replaced_instead_of_adopted_as_control(setup_feedback: FeedbackSetup):
    manager, fake = setup_feedback
    manager.reconcile('arm_a', agent_running=True)
    fake.properties[UNIT_FEEDBACK]['Environment'] = 'BENCH_FEEDBACK_MODEL=arm_a'
    restarted = FirmwareFeedbackManager(manager.models_root, SystemdUnits(runner=fake))
    restarted.reconcile('arm_a', agent_running=True)
    assert restarted.status()['execution'] == 'ros2_control'
    assert len([c for c in fake.commands() if c[0] == 'systemd-run']) == 2


def test_failed_unit_retries_with_cooldown_and_bounded_attempts(setup_feedback: FeedbackSetup, monkeypatch) -> None:
    clock = [100.]
    monkeypatch.setattr('bench_manager.feedback_manager.time.monotonic', lambda: clock[0])
    manager, fake = setup_feedback
    stops = []
    manager._stop_control = lambda: stops.append(True)
    manager.reconcile('arm_a', agent_running=True)
    fake.properties[UNIT_FEEDBACK].update(ActiveState='failed', ExecMainStatus='1')
    for _ in range(3):
        manager.reconcile('arm_a', agent_running=True)
    assert manager.status()['state'] == 'failed'
    assert len([c for c in fake.commands() if c[0] == 'systemd-run']) == 1
    for now, expected in [(103., 2), (106., 3)]:
        clock[0] = now
        manager.reconcile('arm_a', agent_running=True)
        assert len([c for c in fake.commands() if c[0] == 'systemd-run']) == expected
        fake.properties[UNIT_FEEDBACK].update(ActiveState='failed', ExecMainStatus='1')
    clock[0] = 120.
    manager.reconcile('arm_a', agent_running=True)
    assert len([c for c in fake.commands() if c[0] == 'systemd-run']) == 3
    assert len(stops) == 3
    assert manager.status()['state'] == 'failed'


def test_stable_session_resets_retry_budget_without_enabling(setup_feedback, monkeypatch):
    clock = [100.]
    monkeypatch.setattr('bench_manager.feedback_manager.time.monotonic', lambda: clock[0])
    manager, fake = setup_feedback
    manager.reconcile('arm_a', agent_running=True)
    fake.properties[UNIT_FEEDBACK].update(ActiveState='failed', ExecMainStatus='1')
    clock[0] = 103.
    manager.reconcile('arm_a', agent_running=True)
    manager.observe_control_status({'model':'arm_a','feedback_fresh':True,'controllers_ready':True})
    for now in (105., 107., 109.):
        clock[0] = now
        manager.observe_control_status({'model':'arm_a','feedback_fresh':True,'controllers_ready':True})
    manager.reconcile('arm_a', agent_running=True)
    fake.properties[UNIT_FEEDBACK].update(ActiveState='failed', ExecMainStatus='1')
    manager.reconcile('arm_a', agent_running=True)
    starts = [c for c in fake.commands() if c[0] == 'systemd-run']
    assert len(starts) == 3
    fake.properties[UNIT_FEEDBACK].update(ActiveState='failed', ExecMainStatus='1')
    clock[0] = 112.
    manager.reconcile('arm_a', agent_running=True)
    assert len([c for c in fake.commands() if c[0] == 'systemd-run']) == 4
    assert all('start_controllers:=false' in command for command in starts)


def test_launch_running_without_ready_feedback_does_not_reset_retry_budget(setup_feedback, monkeypatch):
    clock = [100.]
    monkeypatch.setattr('bench_manager.feedback_manager.time.monotonic', lambda: clock[0])
    manager, fake = setup_feedback
    for now in (100., 103., 106.):
        clock[0] = now
        manager.reconcile('arm_a', agent_running=True)
        if now != 106.:
            fake.properties[UNIT_FEEDBACK].update(ActiveState='failed', ExecMainStatus='1')
    clock[0] = 120.
    manager.reconcile('arm_a', agent_running=True)
    fake.properties[UNIT_FEEDBACK].update(ActiveState='failed', ExecMainStatus='1')
    manager.reconcile('arm_a', agent_running=True)
    assert len([c for c in fake.commands() if c[0] == 'systemd-run']) == 3


def test_ready_status_gap_restarts_the_health_window(setup_feedback, monkeypatch):
    clock = [100.]
    monkeypatch.setattr('bench_manager.feedback_manager.time.monotonic', lambda: clock[0])
    manager, fake = setup_feedback
    manager.reconcile('arm_a', agent_running=True)
    for now in (101., 107.):
        clock[0] = now
        manager.observe_control_status({'model':'arm_a','feedback_fresh':True,'controllers_ready':True})
    manager.reconcile('arm_a', agent_running=True)
    assert manager._attempt_count == 1


def test_running_but_unhealthy_session_is_stopped_before_retry(setup_feedback, monkeypatch):
    clock = [100.]
    monkeypatch.setattr('bench_manager.feedback_manager.time.monotonic', lambda: clock[0])
    manager, fake = setup_feedback
    stops = [];manager._stop_control = lambda: stops.append(True)
    manager.reconcile('arm_a', agent_running=True)
    for now in (101., 103., 105., 107., 109.):
        clock[0] = now
        manager.observe_control_status({'model':'arm_a','feedback_fresh':False,'controllers_ready':True})
    manager.reconcile('arm_a', agent_running=True)
    starts = [c for c in fake.commands() if c[0] == 'systemd-run']
    assert len(starts) == 2 and len(stops) == 2
    assert all('start_controllers:=false' in command for command in starts)


def test_online_subset_session_changes_stop_before_restart_and_never_enable(setup_feedback):
    manager,fake=setup_feedback
    stops=[];manager._stop_control=lambda:stops.append(len(fake.commands()))
    manager.reconcile('arm_a',agent_running=True,motor_ids=[1],session_id=100)
    manager.reconcile('arm_a',agent_running=True,motor_ids=[1],session_id=100)
    starts=[c for c in fake.commands() if c[0]=='systemd-run']
    assert len(starts)==1 and 'motor_ids:=1' in starts[0]
    assert manager.status()['session_id']=='100'
    manager.reconcile('arm_a',agent_running=True,motor_ids=[1],session_id=101)
    assert len(stops)==2
    assert len([c for c in fake.commands() if c[0]=='systemd-run'])==2
    manager.reconcile('arm_a',agent_running=True,motor_ids=[],session_id=101)
    assert manager.status()['state']=='stopped'
    assert len(stops)==3


def test_no_complete_group_never_starts_a_control_session(setup_feedback):
    manager,fake=setup_feedback
    manager.reconcile('arm_a',agent_running=True,motor_ids=[2],session_id=100)
    assert not [c for c in fake.commands() if c[0]=='systemd-run']
    assert manager.status()['source']=='waiting'


def test_feedback_inventory_uses_source_and_reception_age_and_rejects_replay():
    from bench_manager.feedback_manager import FeedbackInventory
    from can_bridge_msgs.msg import MotorControlStateArray,MotorControlState
    inventory=FeedbackInventory();now=1_000_000_000
    frame=MotorControlStateArray(schema_version=2,session_id=100,sequence=1,sync_time_ns=now,
        states=[MotorControlState(motor_id=2,online=True,state_age_ms=10)])
    assert inventory.observe(frame,now,1.)
    assert inventory.current(now,1.)==([2],100)
    import copy
    frame=copy.deepcopy(frame);frame.sequence=2;frame.states[0].policy_enabled=True
    assert inventory.observe(frame,now,1.1)
    assert inventory.current(now,1.1)==([2],100)
    assert not inventory.observe(frame,now,1.1)
    assert inventory.current(now+301_000_000,1.2)==([],100)
    assert inventory.current(now,1.5)==([],100)
    frame=copy.deepcopy(frame);frame.sequence=3;frame.states[0].state_age_ms=290
    assert inventory.observe(frame,now,1.6)
    assert inventory.current(now+20_000_000,1.6)==([],100)


def test_inventory_rejects_old_session_duplicate_ids_and_future_source():
    import copy
    from bench_manager.feedback_manager import FeedbackInventory
    from can_bridge_msgs.msg import MotorControlStateArray,MotorControlState
    inventory=FeedbackInventory();now=1_000_000_000
    frame=MotorControlStateArray(schema_version=2,session_id=100,sequence=1,sync_time_ns=now,
        states=[MotorControlState(motor_id=2,online=True,state_age_ms=10)])
    assert inventory.observe(frame,now,1.)
    bad=copy.deepcopy(frame);bad.sequence=2;bad.session_id=99
    assert not inventory.observe(bad,now,1.)
    bad=copy.deepcopy(frame);bad.sequence=2;bad.states.append(copy.deepcopy(bad.states[0]))
    assert not inventory.observe(bad,now,1.)
    bad=copy.deepcopy(frame);bad.sequence=2;bad.sync_time_ns=now+20_000_001
    assert not inventory.observe(bad,now,1.)
    empty=MotorControlStateArray(schema_version=2,session_id=100,sequence=2,sync_time_ns=now)
    assert inventory.observe(empty,now,1.)
    assert inventory.current(now,1.)==([],100)
