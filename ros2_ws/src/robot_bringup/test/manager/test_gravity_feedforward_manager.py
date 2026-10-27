"""`GravityFeedforwardProcessManager` 的测试（`bench-gravity-feedforward.service`）。

同其他 manager 测试：不起真进程、不碰真 systemd（`conftest.FakeSystemd`），
`urdf_provider` 与增益写入通道都注入假体。
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest

from conftest import FakeSystemd

from bench_manager.gravity_feedforward_manager import (
    GravityFeedforwardProcessManager,
    validate_joint_map,
)
from bench_manager.systemd_units import (
    AUTO_STARTED_ENV, UNIT_GRAVITY_FEEDFORWARD, SystemdUnits,
)

_MODEL = 'half_body_robot'
_OTHER_MODEL = 'other_robot'
_JOINT_MAP = '1:joint_link1,2:joint_link2'


@pytest.fixture
def urdfs(tmp_path: Path) -> dict[str, str]:
    """两个合法模型：`_OTHER_MODEL` 用来验「已在跑另一个模型」那条分支。"""
    paths: dict[str, str] = {}
    for model in (_MODEL, _OTHER_MODEL):
        model_dir = tmp_path / 'models' / model / 'urdf'
        model_dir.mkdir(parents=True)
        path = model_dir / f'{model}.urdf'
        path.write_text('<robot name="x"/>', encoding='utf-8')
        paths[model] = str(path)
    return paths


def _systemd() -> FakeSystemd:
    return FakeSystemd({UNIT_GRAVITY_FEEDFORWARD: {'LoadState': 'not-found'}})


def _creations(systemd: FakeSystemd) -> list[list[str]]:
    return [call for call in systemd.commands() if call[0] == 'systemd-run']


def _exec_start(systemd: FakeSystemd) -> list[str]:
    create = _creations(systemd)[0]
    return create[create.index('ros2'):]


def _env(systemd: FakeSystemd) -> dict[str, str]:
    create = _creations(systemd)[0]
    pairs = [a.split('=', 1)[1] for a in create if a.startswith('--setenv=')]
    return dict(pair.split('=', 1) for pair in pairs)


def _manager(urdfs: dict[str, str], systemd: FakeSystemd,
             written: list[float] | None = None) -> GravityFeedforwardProcessManager:
    return GravityFeedforwardProcessManager(
        urdf_provider=lambda model: urdfs.get(model, ''),
        gain_writer=(written.append if written is not None else None),
        systemd=SystemdUnits(runner=systemd),
    )


def _request(**overrides: Any) -> dict[str, Any]:
    request: dict[str, Any] = {
        'model': _MODEL,
        'joint_map': _JOINT_MAP,
        'effort_gain': 1.0,
    }
    request.update(overrides)
    return request


# --- 启动前的校验 -----------------------------------------------------------

@pytest.mark.parametrize('model', ['', 'no_such_model', '../etc', 'a/b', None])
def test_start_rejects_a_model_without_a_urdf(urdfs: dict[str, str], model: Any) -> None:
    """模型名直接进文件路径，`../` 之类必须在服务这一层就被拒。"""
    systemd = _systemd()
    manager = _manager(urdfs, systemd)

    with pytest.raises(ValueError):
        manager.start(_request(model=model))

    assert not _creations(systemd)


@pytest.mark.parametrize('joint_map', ['', '1', '1:', ':joint_link1', 'x:j',
                                       '1:a,1:b', '70000:a'])
def test_start_rejects_a_malformed_joint_map(urdfs: dict[str, str], joint_map: str) -> None:
    """畸形表在服务里就拒：让 unit 起来再退出，页面只会看到一条 `failed`。"""
    systemd = _systemd()
    manager = _manager(urdfs, systemd)

    with pytest.raises(ValueError):
        manager.start(_request(joint_map=joint_map))

    assert not _creations(systemd)


@pytest.mark.parametrize('gain', [-1.0, float('nan'), float('inf'), 'x'])
def test_start_rejects_a_bad_gain(urdfs: dict[str, str], gain: Any) -> None:
    systemd = _systemd()
    manager = _manager(urdfs, systemd)

    with pytest.raises(ValueError):
        manager.start(_request(effort_gain=gain))

    assert not _creations(systemd)


def test_joint_map_accepts_the_contract_shape() -> None:
    spec = '1:joint_link1,10:joint_link1.1'
    assert validate_joint_map(spec) == spec


# --- 启动 -------------------------------------------------------------------

def test_start_runs_the_overlay_with_the_models_urdf(urdfs: dict[str, str]) -> None:
    systemd = _systemd()
    manager = _manager(urdfs, systemd)

    status = manager.start(_request(effort_gain=0.8))

    assert _exec_start(systemd) == [
        'ros2', 'run', 'robot_dynamics', 'gravity_feedforward_overlay',
        '--ros-args',
        '-p', f'urdf_path:={urdfs[_MODEL]}',
        '-p', f'joint_map:={_JOINT_MAP}',
        '-p', 'effort_gain:=0.8',
    ]
    assert status['state'] == 'running'
    assert status['config'] == {
        'model': _MODEL, 'urdf_path': urdfs[_MODEL], 'joint_map': _JOINT_MAP,
        'effort_gain': 0.8}


def test_start_does_not_mark_the_unit_as_auto_started(urdfs: dict[str, str]) -> None:
    """不随会话自动起：归属标记必须是 0，否则停仿真会连带把它停了。"""
    systemd = _systemd()
    manager = _manager(urdfs, systemd)

    manager.start(_request())

    assert _env(systemd)[AUTO_STARTED_ENV] == '0'


def test_start_is_idempotent_for_the_same_configuration(urdfs: dict[str, str]) -> None:
    systemd = _systemd()
    manager = _manager(urdfs, systemd)

    manager.start(_request())
    manager.start(_request())

    assert len(_creations(systemd)) == 1


def test_start_refuses_another_model_without_replacing_it(urdfs: dict[str, str]) -> None:
    """换模型静默拿到旧模型的 τ_g 是这个按钮最危险的失效模式：先停再起。"""
    systemd = _systemd()
    manager = _manager(urdfs, systemd)
    manager.start(_request())

    with pytest.raises(RuntimeError, match='another model'):
        manager.start(_request(model=_OTHER_MODEL))

    assert len(_creations(systemd)) == 1


def test_start_applies_a_new_gain_without_restarting(urdfs: dict[str, str]) -> None:
    """同模型下改增益走参数通道——重起会断流，撞固件的命令超时软停。"""
    systemd = _systemd()
    written: list[float] = []
    manager = _manager(urdfs, systemd, written)
    manager.start(_request())

    status = manager.start(_request(effort_gain=0.5))

    assert len(_creations(systemd)) == 1, '不该重建 unit'
    assert written == [0.5]
    assert status['config'] is not None, '复用分支也要回状态'


# --- 状态 -------------------------------------------------------------------

def test_status_reports_the_units_own_description(urdfs: dict[str, str]) -> None:
    """状态从 unit 环境派生：bench_manager 重启后仍说得清在跑什么。"""
    systemd = _systemd()
    manager = _manager(urdfs, systemd)
    manager.start(_request(effort_gain=0.25))

    status = _manager(urdfs, systemd).status()

    assert status['state'] == 'running'
    assert status['config']['model'] == _MODEL
    assert status['config']['effort_gain'] == pytest.approx(0.25)


def test_status_before_any_start_has_no_config(urdfs: dict[str, str]) -> None:
    status = _manager(urdfs, _systemd()).status()

    assert status == {'state': 'stopped', 'config': None, 'returncode': None,
                      'gain_error': '', 'command_topic': None, 'route_error': ''}


def test_stop_clears_the_gain_override(urdfs: dict[str, str]) -> None:
    systemd = _systemd()
    written: list[float] = []
    manager = _manager(urdfs, systemd, written)
    manager.start(_request())
    manager.set_gain(0.5)
    manager.note_gain_applied(0.5)

    manager.stop()
    manager.start(_request(effort_gain=1.0))

    assert manager.status()['config']['effort_gain'] == pytest.approx(1.0)


# --- 增益通道 ---------------------------------------------------------------

def test_set_gain_requires_a_running_unit(urdfs: dict[str, str]) -> None:
    written: list[float] = []
    manager = _manager(urdfs, _systemd(), written)

    with pytest.raises(RuntimeError, match='not running'):
        manager.set_gain(0.5)

    assert written == []


def test_only_a_confirmed_gain_reaches_the_status(urdfs: dict[str, str]) -> None:
    """只有确认写进去的值才进状态；失败要说出来，不静默。"""
    systemd = _systemd()
    manager = _manager(urdfs, systemd, [])
    manager.start(_request(effort_gain=1.0))
    manager.set_gain(0.5)

    assert manager.status()['config']['effort_gain'] == pytest.approx(1.0)

    manager.note_gain_failed('parameter service is gone')
    status = manager.status()

    assert status['config']['effort_gain'] == pytest.approx(1.0)
    assert 'parameter service is gone' in status['gain_error']

    manager.note_gain_applied(0.5)
    status = manager.status()

    assert status['config']['effort_gain'] == pytest.approx(0.5)
    assert status['gain_error'] == ''
