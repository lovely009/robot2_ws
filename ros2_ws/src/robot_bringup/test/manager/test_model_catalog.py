"""真机管理器读取公共模型目录时，不依赖仿真进程管理模块。"""
from concurrent.futures import Future
import os
from pathlib import Path
import sys

import pytest
from ament_index_python.search_paths import get_search_paths

from conftest import FakeSystemd
from bench_manager.feedback_manager import FirmwareFeedbackManager
from bench_manager.gravity_feedforward_manager import GravityFeedforwardManager
from bench_manager.systemd_units import SystemdUnits


@pytest.fixture
def isolated_models(tmp_path, monkeypatch):
    index = tmp_path / 'share/ament_index/resource_index/packages'
    index.mkdir(parents=True)
    marker = index / 'robot_description'
    marker.touch()
    root = tmp_path / 'share/robot_description/models'
    model = root / 'arm'
    (model / 'config').mkdir(parents=True)
    (model / 'urdf').mkdir()
    (model / 'config/motor_driver.yaml').write_text(
        'motor_hardware:\n  motors:\n    motor: {id: 1, joint: elbow}\n')
    (model / 'urdf/arm.urdf').write_text(
        '<robot name="arm"><link name="base"/><link name="tip"/>'
        '<joint name="elbow" type="continuous"><parent link="base"/>'
        '<child link="tip"/><limit velocity="1" effort="10"/></joint></robot>')
    # 隔离模型包，但保留真实 transmission_interface 插件所需的安装前缀。
    prefixes = [str(tmp_path)] + [prefix for prefix in get_search_paths()
        if not (Path(prefix) / 'share/ament_index/resource_index/packages/robot_description').is_file()]
    monkeypatch.setenv('AMENT_PREFIX_PATH', os.pathsep.join(prefixes))
    # 拒绝加载可选仿真管理模块，但保留真实 ament 查找与传动解析。
    monkeypatch.setitem(sys.modules, 'bench_manager.simulation_manager', None)
    return root, marker


def _control():
    return dict(model='arm', joint_names=['elbow'], mode='trajectory',
                gravity_enabled=False, effort_gain=1.0)


@pytest.mark.parametrize('explicit_root', [False, True], ids=['installed', 'explicit'])
def test_feedback_resolves_model_without_simulation_manager(isolated_models, explicit_root):
    root, marker = isolated_models
    if explicit_root:
        marker.unlink()
    systemd = FakeSystemd()
    manager = FirmwareFeedbackManager(
        root if explicit_root else None, SystemdUnits(runner=systemd))

    manager.reconcile('arm', agent_running=True)

    assert manager.status()['state'] == 'running'
    assert manager.status()['model'] == 'arm'
    starts = [call for call in systemd.commands() if call[0] == 'systemd-run']
    assert len(starts) == 1
    assert f'motor_config_file:={root / "arm/config/motor_driver.yaml"}' in starts[0]
    assert 'start_controllers:=false' in starts[0]


@pytest.mark.parametrize('explicit_root', [False, True], ids=['installed', 'explicit'])
def test_gravity_resolves_model_without_simulation_manager(isolated_models, explicit_root):
    root, marker = isolated_models
    if explicit_root:
        marker.unlink()
    writes = []

    def write(values):
        writes.append(values)
        return Future()

    manager = GravityFeedforwardManager(root if explicit_root else None, write)
    manager.observe(_control())

    status = manager.start({'model': 'arm', 'effort_gain': 0.6})

    assert writes == [{'enabled': True, 'effort_gain': 0.6}]
    assert status['pending'] == 'start'
    assert status['state'] == 'stopped'


def test_missing_model_package_does_not_start_feedback(isolated_models):
    _root, marker = isolated_models
    marker.unlink()
    systemd = FakeSystemd()
    manager = FirmwareFeedbackManager(systemd=SystemdUnits(runner=systemd))

    manager.reconcile('arm', agent_running=True)

    assert manager.status()['state'] == 'failed'
    assert '找不到模型根目录' in manager.status()['error']
    assert not any(call[0] == 'systemd-run' for call in systemd.commands())


def test_missing_model_package_rejects_gravity_start(isolated_models):
    _root, marker = isolated_models
    marker.unlink()
    manager = GravityFeedforwardManager()
    manager.observe(_control())

    with pytest.raises(ValueError, match='model transmission directory is unavailable'):
        manager.start({'model': 'arm'})
