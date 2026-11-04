"""`SimulationProcessManager` 的单测（仿真会话）。

不产生真实进程、不启任何 unit：systemd 走 `conftest.FakeSystemd`，CAN 接口 / 串口 /
模型列表全部用桩，所以本文件既不启 Gazebo、也不开 MuJoCo 窗口、更不碰 vcan 与串口。
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest

from conftest import FakeSystemd

from bench_manager.model_catalog import (
    enumerate_mujoco_models,
    enumerate_simulation_models,
)
from bench_manager.simulation_manager import SimulationProcessManager
from bench_manager.systemd_units import UNIT_SIMULATION, SystemdUnits


def _systemd() -> FakeSystemd:
    # 首次运行：unit 从未创建过（`LoadState=not-found`，与「干净停过」在 ActiveState
    # 上无法区分，所以这里必须显式建模）。
    return FakeSystemd({UNIT_SIMULATION: {'LoadState': 'not-found'}})


def _creations(systemd: FakeSystemd) -> list[list[str]]:
    """建过几次 unit——`systemd-run` 的调用。空列表＝什么都没起。"""
    return [call for call in systemd.commands() if call[0] == 'systemd-run']


def _exec_start(systemd: FakeSystemd) -> list[str]:
    """unit 的 ExecStart：`systemd-run` 的 argv 里 `--setenv` 之后的那截。"""
    create = _creations(systemd)[0]
    return create[create.index('ros2'):]


def _manager(systemd: FakeSystemd,
             models: tuple[str, ...] = ('half_body_robot',),
             mujoco_models: tuple[str, ...] = ('half_body_robot',),
             ) -> SimulationProcessManager:
    return SimulationProcessManager(
        can_provider=lambda: [],
        serial_provider=lambda: [],
        model_provider=lambda: list(models),
        mujoco_model_provider=lambda: list(mujoco_models),
        systemd=SystemdUnits(runner=systemd),
    )


def _request(**overrides: Any) -> dict[str, Any]:
    request: dict[str, Any] = {
        'backend': 'gazebo',
        'transport': 'vcan',
        'controller_mode': 'external',
        'model': 'half_body_robot',
        'vcan_interface': 'robot_vcan0',
    }
    request.update(overrides)
    return request


def test_options_report_both_model_lists() -> None:
    manager = _manager(_systemd(), models=('half_body_robot', 'other'),
                       mujoco_models=('half_body_robot',))

    options = manager.options()

    assert options['models'] == ['half_body_robot', 'other']
    assert options['mujoco_models'] == ['half_body_robot']


@pytest.mark.parametrize('transport', ['vcan', 'socketcan', 'slcan'])
def test_retired_host_mode_is_rejected_before_unit_or_device_preparation(transport: str) -> None:
    systemd = _systemd()
    manager = _manager(systemd)

    with pytest.raises(ValueError, match='host.*retired.*firmware.*external'):
        manager.start(_request(controller_mode='host', transport=transport))

    # Validation happens before even probing systemd, selecting a physical
    # endpoint or asking it to create a unit that would fail in launch later.
    assert systemd.commands() == []


@pytest.mark.parametrize('backend', [None, '', 'gazebo ', 'isaac', 7])
def test_start_rejects_unknown_backend_without_spawning(backend: Any) -> None:
    systemd = _systemd()
    manager = _manager(systemd)
    request = _request()
    if backend is None:
        del request['backend']
    else:
        request['backend'] = backend

    with pytest.raises(ValueError, match='backend'):
        manager.start(request)

    assert _creations(systemd) == []


def test_start_rejects_mujoco_backend_for_model_without_mjcf() -> None:
    systemd = _systemd()
    manager = _manager(systemd, models=('half_body_robot', 'gazebo_only'),
                       mujoco_models=('half_body_robot',))

    with pytest.raises(ValueError, match='MuJoCo'):
        manager.start(_request(backend='mujoco', model='gazebo_only'))

    assert _creations(systemd) == []


def test_start_passes_backend_and_endpoint_to_launch() -> None:
    systemd = _systemd()
    manager = _manager(systemd)

    manager.start(_request(backend='mujoco'))

    assert _exec_start(systemd) == [
        'ros2', 'launch', 'robot_sim', 'simulation.launch.py',
        'backend:=mujoco',
        'transport:=vcan',
        'controller_mode:=external',
        'model:=half_body_robot',
        'vcan_interface:=robot_vcan0',
    ]


def test_start_persists_the_session_description_into_the_unit() -> None:
    """会话描述随 unit 持久化——`bench_manager` 重启后页面仍说得清在跑什么。

    以前 config 活在本对象的字段里，进程一换就只剩「在跑」两个字。
    """
    systemd = _systemd()

    status = _manager(systemd).start(_request(backend='mujoco'))

    assert '--setenv=BENCH_SIM_BACKEND=mujoco' in _creations(systemd)[0]
    assert status['config']['backend'] == 'mujoco'
    assert status['state'] == 'running'


def test_same_config_is_idempotent_but_backend_change_conflicts() -> None:
    systemd = _systemd()
    manager = _manager(systemd)

    first = manager.start(_request(backend='gazebo'))
    again = manager.start(_request(backend='gazebo'))

    assert len(_creations(systemd)) == 1, '同配置重入不该重建 unit（会打断正在跑的仿真）'
    assert again == first
    assert again['config']['backend'] == 'gazebo'

    with pytest.raises(RuntimeError, match='already running'):
        manager.start(_request(backend='mujoco'))
    assert len(_creations(systemd)) == 1


# --- 模型级仿真参数（生成高度 / 重力 / 固定基座）-----------------------------
#
# 前端「模型」抽屉调的就是这三个：只有 Gazebo 有实现，MuJoCo 在 MJCF 生成期固定
# （launch 侧同样拒绝），所以缺省不放命令行、给了 Mujoco 就显式报错。

def test_start_forwards_model_parameters_to_launch() -> None:
    systemd = _systemd()
    manager = _manager(systemd)

    status = manager.start(_request(gravity=False, fixed_base=False, spawn_z=0.0027))

    command = _exec_start(systemd)
    assert 'gravity:=false' in command
    assert 'fixed_base:=false' in command
    assert 'spawn_z:=0.0027' in command
    # 布尔与浮点经环境变量走了一圈仍要还原成原类型，否则幂等比较永远不相等
    assert status['config']['spawn_z'] == 0.0027
    assert status['config']['gravity'] is False


def test_start_omits_model_parameters_when_absent() -> None:
    systemd = _systemd()
    manager = _manager(systemd)

    manager.start(_request())

    command = _exec_start(systemd)
    assert not [
        arg for arg in command
        if arg.startswith(('gravity:', 'fixed_base:', 'spawn_z:'))
    ], '缺省的模型参数不该进命令行（否则「不设置」会变成「改行为」）'


@pytest.mark.parametrize('key, value', [
    ('gravity', 'true'),
    ('gravity', 1),
    ('fixed_base', 0),
    ('spawn_z', '0.03'),
    ('spawn_z', True),
    ('spawn_z', float('inf')),
    ('spawn_z', 100.0),
])
def test_start_rejects_bad_model_parameters(key: str, value: Any) -> None:
    systemd = _systemd()
    manager = _manager(systemd)

    with pytest.raises(ValueError, match=key):
        manager.start(_request(**{key: value}))

    assert _creations(systemd) == []


def test_start_rejects_model_parameters_for_mujoco() -> None:
    systemd = _systemd()
    manager = _manager(systemd)

    with pytest.raises(ValueError, match='mujoco'):
        manager.start(_request(backend='mujoco', gravity=False))

    assert _creations(systemd) == []


def test_stop_stops_the_simulation_unit() -> None:
    systemd = _systemd()
    manager = _manager(systemd)
    manager.start(_request())

    status = manager.stop()

    assert ['systemctl', '--user', 'stop', UNIT_SIMULATION] in systemd.commands()
    assert status['state'] == 'stopped'
    assert status['config'] is None


def test_failed_unit_reports_the_exit_code_and_keeps_the_config() -> None:
    """非零退出的 unit 留得住（干净退出的会被卸载，那两种在 `UnitState` 里已分开）。"""
    systemd = FakeSystemd({UNIT_SIMULATION: {
        'LoadState': 'loaded', 'ActiveState': 'failed', 'SubState': 'failed',
        'ExecMainStatus': '1', 'Environment': 'BENCH_SIM_MODEL=half_body_robot'}})

    status = _manager(systemd).status()

    assert status['state'] == 'failed'
    assert status['returncode'] == 1
    assert status['config'] == {'model': 'half_body_robot'}


# --- 模型枚举（root 可配置）--------------------------------------------------
#
# 模型列表来自节点参数 models_dir（可指源码目录，改模型不必重构建）；root 缺省
# 时回落 robot_description 安装树。臂视图的映射/枚举**不在这里**——那批只读文件的函数
# 归 web_host/models.py（见 web_host/test/test_models.py）。

def test_enumerate_simulation_models_uses_root(tmp_path: Path) -> None:
    full = tmp_path / 'full_model'
    (full / 'config').mkdir(parents=True)
    (full / 'config' / 'simulation_topology.yaml').write_text(
        'simulation:\n', encoding='utf-8')
    (tmp_path / 'no_topology').mkdir()
    assert enumerate_simulation_models(tmp_path) == ['full_model']


def test_enumerate_mujoco_models_uses_root(tmp_path: Path) -> None:
    full = tmp_path / 'mj_model'
    (full / 'mujoco').mkdir(parents=True)
    (full / 'urdf').mkdir()
    (full / 'mujoco' / 'mj_model.xml').write_text('<mujoco/>', encoding='utf-8')
    (full / 'urdf' / 'mj_model.standalone.urdf').write_text(
        '<robot/>', encoding='utf-8')
    (tmp_path / 'no_mjcf').mkdir()
    assert enumerate_mujoco_models(tmp_path) == ['mj_model']


# --- 电机后端（HIL 的 FDCAN 模拟器）不再由仿真 launch 内嵌 -------------------
#
# HIL 电机后端由 bench_manager 独立管理；仿真 launch 不内嵌启动（start_fdcan_sim:=false）。

def _firmware_request(**overrides: Any) -> dict[str, Any]:
    request = _request(controller_mode='firmware', transport='socketcan',
                       sim_interface='can0', physical_bus_isolated=True)
    request.update(overrides)
    return request


def _firmware_manager(systemd: FakeSystemd) -> SimulationProcessManager:
    return SimulationProcessManager(
        can_provider=lambda: [{'name': 'can0', 'up': True, 'label': 'can0'}],
        serial_provider=lambda: [],
        model_provider=lambda: ['half_body_robot'],
        mujoco_model_provider=lambda: [],
        systemd=SystemdUnits(runner=systemd),
    )


def test_firmware_start_defers_motor_backend_to_bench_manager() -> None:
    systemd = _systemd()
    manager = _firmware_manager(systemd)

    status = manager.start(_firmware_request())

    command = _exec_start(systemd)
    assert 'start_fdcan_sim:=false' in command
    assert 'start_host_control:=false' in command
    assert not any(arg.startswith('sim_control_socket:=') for arg in command)
    assert status['config']['controller_mode'] == 'firmware'


# --- 全链：服务消息 → _simulation_request → _validate -------------------------
#
# 从真实服务消息验证 physical_bus_isolated 的透传：确认后允许启动，未确认必须拒绝。


def _start_simulation_message(**fields: Any):
    from bench_msgs.srv import StartSimulation

    message = StartSimulation.Request()
    for key, value in fields.items():
        setattr(message, key, value)
    return message


def test_service_message_clears_validation_on_a_physical_bus() -> None:
    from bench_manager.node import _simulation_request

    systemd = _systemd()
    manager = _firmware_manager(systemd)

    manager.start(_simulation_request(_start_simulation_message(
        backend='gazebo', transport='socketcan', controller_mode='firmware',
        model='half_body_robot', sim_interface='can0',
        physical_bus_isolated=True)))

    assert _creations(systemd), '确认位传到了却仍被拒——物理总线上的仿真起不来'


def test_service_message_without_confirmation_is_rejected() -> None:
    from bench_manager.node import _simulation_request

    systemd = _systemd()
    manager = _firmware_manager(systemd)

    with pytest.raises(ValueError, match='no real motor load'):
        manager.start(_simulation_request(_start_simulation_message(
            backend='gazebo', transport='socketcan', controller_mode='firmware',
            model='half_body_robot', sim_interface='can0')))

    assert not _creations(systemd), '被拒了却起了 unit'
