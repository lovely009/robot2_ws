"""模型反馈只读起停与发布者交接；不启动节点或 systemd。"""
from pathlib import Path

import pytest

from conftest import FakeSystemd
from bench_manager.feedback_manager import FirmwareFeedbackManager
from bench_manager.systemd_units import SystemdUnits, UNIT_FEEDBACK


FeedbackSetup = tuple[FirmwareFeedbackManager, FakeSystemd]


@pytest.fixture
def clock(monkeypatch):
    now = [100.]
    monkeypatch.setattr('bench_manager.feedback_manager.time.monotonic', lambda: now[0])
    return now


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


def _settle(manager, clock, model='arm_a', **context):
    manager.reconcile(model, **context)
    clock[0] += 1.5
    manager.reconcile(model, **context)


@pytest.fixture
def inventory_feedback(setup_feedback):
    manager, fake = setup_feedback
    model = manager.models_root / 'arm_a'
    (model / 'config/motor_driver.yaml').write_text(
        'motor_hardware:\n  motors:\n'
        '    m1: {id: 1, joint: elbow}\n'
        '    m2: {id: 2, joint: wrist}\n')
    (model / 'urdf/arm_a.urdf').write_text(
        '<robot name="arm"><joint name="elbow" type="continuous"/>'
        '<joint name="wrist" type="continuous"/></robot>')
    return manager, fake


def test_inventory_start_waits_for_motors_to_finish_arriving(inventory_feedback, clock):
    manager, fake = inventory_feedback
    manager.reconcile('arm_a', agent_running=True, motor_ids=[2], session_id=100)
    assert manager.status()['state'] == 'stopped'
    clock[0] = 100.5
    manager.reconcile('arm_a', agent_running=True, motor_ids=[1, 2], session_id=100)
    clock[0] = 101.5
    manager.reconcile('arm_a', agent_running=True, motor_ids=[1, 2], session_id=100)
    assert not [c for c in fake.commands() if c[0] == 'systemd-run']
    clock[0] = 102.
    manager.reconcile('arm_a', agent_running=True, motor_ids=[1, 2], session_id=100)
    starts = [c for c in fake.commands() if c[0] == 'systemd-run']
    assert len(starts) == 1
    assert 'motor_ids:=1,2' in starts[0]
    assert 'start_controllers:=false' in starts[0]


@pytest.mark.parametrize('missing', [[2], []])
def test_brief_inventory_dropout_keeps_existing_session(inventory_feedback, clock, missing):
    manager, fake = inventory_feedback
    stops = []
    manager._stop_control = lambda: stops.append(clock[0])
    _settle(manager, clock, agent_running=True, motor_ids=[1, 2], session_id=100)
    before = len(stops)
    clock[0] = 102.
    manager.reconcile('arm_a', agent_running=True, motor_ids=missing, session_id=100)
    assert manager.status()['state'] == 'running'
    clock[0] = 102.5
    manager.reconcile('arm_a', agent_running=True, motor_ids=[1, 2], session_id=100)
    assert len(stops) == before
    assert len([c for c in fake.commands() if c[0] == 'systemd-run']) == 1
    assert manager.status()['motor_ids'] == '1,2'


@pytest.mark.parametrize('initial,changed', [([1, 2], [2]), ([2], [1, 2])])
def test_persistent_inventory_change_replaces_session_once(inventory_feedback, clock, initial, changed):
    manager, fake = inventory_feedback
    stops = []
    manager._stop_control = lambda: stops.append(len(fake.commands()))
    _settle(manager, clock, agent_running=True, motor_ids=initial, session_id=100)
    before = len(stops)
    clock[0] = 102.
    manager.reconcile('arm_a', agent_running=True, motor_ids=changed, session_id=100)
    clock[0] = 103.
    manager.reconcile('arm_a', agent_running=True, motor_ids=changed, session_id=100)
    assert len(stops) == before
    clock[0] = 103.5
    manager.reconcile('arm_a', agent_running=True, motor_ids=changed, session_id=100)
    manager.reconcile('arm_a', agent_running=True, motor_ids=changed, session_id=100)
    starts = [c for c in fake.commands() if c[0] == 'systemd-run']
    assert len(starts) == 2
    assert len(stops) == before + 1
    assert stops[-1] < fake.commands().index(starts[-1])
    assert manager.status()['motor_ids'] == ','.join(map(str, changed))
    assert all('start_controllers:=false' in c for c in starts)


def test_persistent_empty_inventory_stops_session_once(inventory_feedback, clock):
    manager, fake = inventory_feedback
    stops = []
    manager._stop_control = lambda: stops.append(clock[0])
    _settle(manager, clock, agent_running=True, motor_ids=[1, 2], session_id=100)
    before = len(stops)
    clock[0] = 102.
    manager.reconcile('arm_a', agent_running=True, motor_ids=[], session_id=100)
    assert manager.status()['state'] == 'running'
    clock[0] = 103.5
    manager.reconcile('arm_a', agent_running=True, motor_ids=[], session_id=100)
    manager.reconcile('arm_a', agent_running=True, motor_ids=[], session_id=100)
    assert manager.status()['state'] == 'stopped'
    assert len(stops) == before + 1
    assert len([c for c in fake.commands() if c[0] == 'systemd-run']) == 1


def test_new_mcu_session_stops_old_control_before_settling(inventory_feedback, clock):
    manager, fake = inventory_feedback
    stops = []
    manager._stop_control = lambda: stops.append(clock[0])
    _settle(manager, clock, agent_running=True, motor_ids=[1, 2], session_id=100)
    before = len(stops)
    clock[0] = 102.
    manager.reconcile('arm_a', agent_running=True, motor_ids=[2], session_id=101)
    assert manager.status()['state'] == 'stopped'
    assert len(stops) == before + 1
    clock[0] = 102.5
    manager.reconcile('arm_a', agent_running=True, motor_ids=[1, 2], session_id=101)
    assert len([c for c in fake.commands() if c[0] == 'systemd-run']) == 1
    clock[0] = 104.
    manager.reconcile('arm_a', agent_running=True, motor_ids=[1, 2], session_id=101)
    starts = [c for c in fake.commands() if c[0] == 'systemd-run']
    assert len(starts) == 2
    assert 'motor_ids:=1,2' in starts[-1]
    assert manager.status()['session_id'] == '101'
    assert all('start_controllers:=false' in c for c in starts)


@pytest.mark.parametrize('session', [0, 100])
def test_restart_adopts_live_inventory_without_stopping(inventory_feedback, clock, session):
    manager, fake = inventory_feedback
    _settle(manager, clock, agent_running=True, motor_ids=[1, 2], session_id=100)
    stops = []
    adopted = FirmwareFeedbackManager(manager.models_root, SystemdUnits(runner=fake),
                                      stop_control=lambda: stops.append(True))
    adopted.reconcile('arm_a', agent_running=True, motor_ids=[], session_id=session)
    assert adopted.status()['state'] == 'running'
    clock[0] += .5
    adopted.reconcile('arm_a', agent_running=True, motor_ids=[1, 2], session_id=100)
    assert adopted.status()['motor_ids'] == '1,2'
    assert not stops
    assert len([c for c in fake.commands() if c[0] == 'systemd-run']) == 1


def test_restart_waits_for_first_inventory_without_treating_unknown_as_empty(inventory_feedback, clock):
    manager, fake = inventory_feedback
    _settle(manager, clock, agent_running=True, motor_ids=[1, 2], session_id=100)
    stops = []
    adopted = FirmwareFeedbackManager(manager.models_root, SystemdUnits(runner=fake),
                                      stop_control=lambda: stops.append(True))
    for now in (102., 104., 106.):
        clock[0] = now
        adopted.reconcile('arm_a', agent_running=True, motor_ids=[], session_id=0)
        assert adopted.status()['state'] == 'running'
    adopted.reconcile('arm_a', agent_running=True, motor_ids=[1, 2], session_id=100)
    assert not stops
    assert len([c for c in fake.commands() if c[0] == 'systemd-run']) == 1


def test_exited_session_still_sends_stop_once_when_inventory_stays_empty(inventory_feedback, clock):
    manager, fake = inventory_feedback
    stops = []
    manager._stop_control = lambda: stops.append(clock[0])
    _settle(manager, clock, agent_running=True, motor_ids=[1, 2], session_id=100)
    before = len(stops)
    fake.properties[UNIT_FEEDBACK] = {'LoadState': 'not-found'}
    _settle(manager, clock, agent_running=True, motor_ids=[], session_id=100)
    manager.reconcile('arm_a', agent_running=True, motor_ids=[], session_id=100)
    assert len(stops) == before + 1
    assert manager.status()['state'] == 'stopped'
    assert len([c for c in fake.commands() if c[0] == 'systemd-run']) == 1


def test_agent_exit_cancels_pending_inventory_start(inventory_feedback, clock):
    manager, fake = inventory_feedback
    manager.reconcile('arm_a', agent_running=True, motor_ids=[1, 2], session_id=100)
    clock[0] += 1.
    manager.reconcile('arm_a', agent_running=False, motor_ids=[1, 2], session_id=100)
    clock[0] += 1.
    manager.reconcile('arm_a', agent_running=True, motor_ids=[1, 2], session_id=100)
    assert not [c for c in fake.commands() if c[0] == 'systemd-run']
    clock[0] += 1.5
    manager.reconcile('arm_a', agent_running=True, motor_ids=[1, 2], session_id=100)
    assert len([c for c in fake.commands() if c[0] == 'systemd-run']) == 1


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
    assert f'motor_config_file:={manager.models_root / "arm_a/config/motor_driver.yaml"}' in starts[0]
    assert f'urdf_file:={manager.models_root / "arm_a/urdf/arm_a.urdf"}' in starts[0]
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


@pytest.mark.parametrize('topology', ['model_motor_topology.yaml', 'motor_topology.yaml'])
def test_topology_only_model_releases_joint_session_and_keeps_motor_monitoring(
        setup_feedback, topology):
    manager, fake = setup_feedback
    manager.reconcile('arm_a', agent_running=True)
    config = manager.models_root / 'arm_b/config'
    (config / 'motor_driver.yaml').unlink()
    (config / topology).write_text('motors: {1: {can_node_id: 1}}\n')
    manager.reconcile('arm_b', agent_running=True, motor_ids=[1], session_id=100)
    status = manager.status()
    assert status['state'] == 'stopped' and status['error'] == ''
    assert 'MCU 电机反馈' in status['reason']
    assert len([c for c in fake.commands() if c[0] == 'systemd-run']) == 1
    before = len(fake.commands())
    manager.reconcile('arm_b', agent_running=True, motor_ids=[1], session_id=100)
    assert not [c for c in fake.commands()[before:] if 'stop' in c]


def test_manager_restart_adopts_matching_unit(setup_feedback: FeedbackSetup) -> None:
    manager, fake = setup_feedback
    manager.reconcile('arm_a', agent_running=True)
    adopted = FirmwareFeedbackManager(manager.models_root, SystemdUnits(runner=fake))
    adopted.reconcile('arm_a', agent_running=True)
    assert adopted.status()['model'] == 'arm_a'
    assert len([c for c in fake.commands() if c[0] == 'systemd-run']) == 1


def test_topology_only_model_cancels_inherited_starting_unit(setup_feedback):
    manager, fake = setup_feedback
    config = manager.models_root / 'arm_b/config'
    (config / 'motor_driver.yaml').unlink()
    (config / 'model_motor_topology.yaml').write_text('motors: {}\n')
    fake.properties[UNIT_FEEDBACK] = {'LoadState': 'loaded', 'ActiveState': 'activating'}
    for _ in range(2):
        manager.reconcile('arm_b', agent_running=True, motor_ids=[1], session_id=100)
    assert len([c for c in fake.commands() if 'stop' in c and UNIT_FEEDBACK in c]) == 1
    assert not [c for c in fake.commands() if c[0] == 'systemd-run']
    assert manager.status()['error'] == ''


def test_old_feedback_unit_is_replaced_instead_of_adopted_as_control(setup_feedback: FeedbackSetup):
    manager, fake = setup_feedback
    manager.reconcile('arm_a', agent_running=True)
    fake.properties[UNIT_FEEDBACK]['Environment'] = 'BENCH_FEEDBACK_MODEL=arm_a'
    restarted = FirmwareFeedbackManager(manager.models_root, SystemdUnits(runner=fake))
    restarted.reconcile('arm_a', agent_running=True)
    assert restarted.status()['execution'] == 'ros2_control'
    assert len([c for c in fake.commands() if c[0] == 'systemd-run']) == 2


@pytest.mark.parametrize('old_env', [
    'FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/old.xml',
    'BENCH_FEEDBACK_PUBLISH=false',
])
def test_restart_replaces_old_special_session(setup_feedback, old_env):
    manager, fake = setup_feedback
    manager.reconcile('arm_a', agent_running=True)
    fake.properties[UNIT_FEEDBACK]['Environment'] += ' ' + old_env
    restarted = FirmwareFeedbackManager(manager.models_root, SystemdUnits(runner=fake))
    restarted.reconcile('arm_a', agent_running=True)
    starts = [c for c in fake.commands() if c[0] == 'systemd-run']
    assert len(starts) == 2
    assert not any('FASTRTPS_DEFAULT_PROFILES_FILE' in arg for arg in starts[-1])
    assert 'publish_joint_states:=true' in starts[-1]


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


def test_online_subset_session_changes_stop_before_restart_and_never_enable(setup_feedback, clock):
    manager, fake = setup_feedback
    stops = []
    manager._stop_control = lambda: stops.append(len(fake.commands()))
    _settle(manager, clock, agent_running=True, motor_ids=[1], session_id=100)
    starts = [c for c in fake.commands() if c[0] == 'systemd-run']
    assert len(starts) == 1 and 'motor_ids:=1' in starts[0]
    assert manager.status()['session_id'] == '100'
    manager.reconcile('arm_a', agent_running=True, motor_ids=[1], session_id=101)
    assert len(stops) == 2
    assert manager.status()['state'] == 'stopped'
    _settle(manager, clock, agent_running=True, motor_ids=[1], session_id=101)
    assert len([c for c in fake.commands() if c[0] == 'systemd-run']) == 2
    _settle(manager, clock, agent_running=True, motor_ids=[], session_id=101)
    assert manager.status()['state'] == 'stopped'
    assert all('start_controllers:=false' in c for c in fake.commands() if c[0] == 'systemd-run')


def test_no_complete_group_never_starts_a_control_session(setup_feedback):
    manager,fake=setup_feedback
    manager.reconcile('arm_a',agent_running=True,motor_ids=[2],session_id=100)
    assert not [c for c in fake.commands() if c[0]=='systemd-run']
    assert manager.status()['source']=='waiting'


def test_partial_transmission_group_waits_until_complete(setup_feedback, clock):
    manager, fake = setup_feedback
    model = manager.models_root / 'arm_a'
    (model / 'config/motor_driver.yaml').write_text(
        'motor_hardware:\n  transmissions_file: transmissions.yaml\n  motors:\n'
        '    a: {id: 1, joint: input_a}\n    b: {id: 2, joint: input_b}\n')
    (model / 'config/transmissions.yaml').write_text(
        'transmissions:\n  pair:\n    plugin: transmission_interface/DifferentialTransmission\n'
        '    actuators: [a, b]\n    joints: [roll, pitch]\n')
    (model / 'urdf/arm_a.urdf').write_text(
        '<robot name="arm"><joint name="roll" type="continuous"/>'
        '<joint name="pitch" type="continuous"/></robot>')
    _settle(manager, clock, agent_running=True, motor_ids=[1], session_id=100)
    assert not [c for c in fake.commands() if c[0] == 'systemd-run']
    assert manager.status()['reason'] == '等待完整传动组的电机反馈'
    _settle(manager, clock, agent_running=True, motor_ids=[1, 2], session_id=100)
    starts = [c for c in fake.commands() if c[0] == 'systemd-run']
    assert len(starts) == 1
    assert 'motor_ids:=1,2' in starts[0]


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
