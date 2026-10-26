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
            '<child link="tip"/></joint></robot>')
    fake = FakeSystemd({UNIT_FEEDBACK: {'LoadState': 'not-found'}})
    return FirmwareFeedbackManager(tmp_path, SystemdUnits(runner=fake)), fake


def test_agent_and_model_start_readonly_feedback_once(setup_feedback: FeedbackSetup) -> None:
    manager, fake = setup_feedback
    manager.reconcile('arm_a', agent_running=False)
    assert manager.status()['state'] == 'stopped'
    manager.reconcile('arm_a', agent_running=True)
    manager.reconcile('arm_a', agent_running=True)
    starts = [c for c in fake.commands() if c[0] == 'systemd-run']
    assert len(starts) == 1
    assert 'model:=arm_a' in starts[0]
    assert 'enable_trajectory:=false' in starts[0]
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


def test_failed_unit_is_not_restarted_every_tick(setup_feedback: FeedbackSetup) -> None:
    manager, fake = setup_feedback
    manager.reconcile('arm_a', agent_running=True)
    fake.properties[UNIT_FEEDBACK].update(ActiveState='failed', ExecMainStatus='1')
    for _ in range(3):
        manager.reconcile('arm_a', agent_running=True)
    assert manager.status()['state'] == 'failed'
    assert len([c for c in fake.commands() if c[0] == 'systemd-run']) == 1
