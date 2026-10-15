"""`SimulationProcessManager` 的单测（仿真进程管理）。

不产生真实子进程：`popen_factory` 注入假体，CAN 接口 / 串口 / 模型列表全部用桩，
所以本文件既不启 Gazebo、也不开 MuJoCo 窗口、更不碰 vcan 与串口。
"""

from __future__ import annotations

import signal
from pathlib import Path
from typing import Any, Callable

import pytest

from bench_manager.simulation_manager import (
    SimulationProcessManager,
    enumerate_mujoco_models,
    enumerate_simulation_models,
)


class FakeProcess:
    pid = 4321

    def __init__(self) -> None:
        self.returncode: int | None = None
        self.stdout = None

    def poll(self) -> int | None:
        return self.returncode

    def wait(self, timeout: float | None = None) -> int:
        self.returncode = -signal.SIGTERM
        return self.returncode


def _manager(
    spawned: list[tuple[list[str], dict[str, Any]]],
    models: tuple[str, ...] = ("half_body_robot",),
    mujoco_models: tuple[str, ...] = ("half_body_robot",),
) -> SimulationProcessManager:
    def fake_popen(command: list[str], **kwargs: Any) -> FakeProcess:
        spawned.append((command, kwargs))
        return FakeProcess()

    return SimulationProcessManager(
        can_provider=lambda: [],
        serial_provider=lambda: [],
        model_provider=lambda: list(models),
        mujoco_model_provider=lambda: list(mujoco_models),
        popen_factory=fake_popen,
    )


def _request(**overrides: Any) -> dict[str, Any]:
    request: dict[str, Any] = {
        'backend': 'gazebo',
        'transport': 'vcan',
        'controller_mode': 'host',
        'model': 'half_body_robot',
        'vcan_interface': 'robot_vcan0',
    }
    request.update(overrides)
    return request


def test_options_report_both_model_lists() -> None:
    manager = _manager([], models=('half_body_robot', 'other'),
                       mujoco_models=('half_body_robot',))

    options = manager.options()

    assert options['models'] == ['half_body_robot', 'other']
    assert options['mujoco_models'] == ['half_body_robot']


@pytest.mark.parametrize('backend', [None, '', 'gazebo ', 'isaac', 7])
def test_start_rejects_unknown_backend_without_spawning(backend: Any) -> None:
    spawned: list[tuple[list[str], dict[str, Any]]] = []
    manager = _manager(spawned)
    request = _request()
    if backend is None:
        del request['backend']
    else:
        request['backend'] = backend

    with pytest.raises(ValueError, match='backend'):
        manager.start(request)

    assert spawned == []


def test_start_rejects_mujoco_backend_for_model_without_mjcf() -> None:
    spawned: list[tuple[list[str], dict[str, Any]]] = []
    manager = _manager(spawned, models=('half_body_robot', 'gazebo_only'),
                       mujoco_models=('half_body_robot',))

    with pytest.raises(ValueError, match='MuJoCo'):
        manager.start(_request(backend='mujoco', model='gazebo_only'))

    assert spawned == []


def test_start_passes_backend_and_endpoint_to_launch() -> None:
    spawned: list[tuple[list[str], dict[str, Any]]] = []
    manager = _manager(spawned)

    manager.start(_request(backend='mujoco'))

    command, kwargs = spawned[0]
    assert command == [
        'ros2', 'launch', 'robot_sim', 'simulation.launch.py',
        'backend:=mujoco',
        'transport:=vcan',
        'controller_mode:=host',
        'model:=half_body_robot',
        'vcan_interface:=robot_vcan0',
    ]
    assert kwargs['start_new_session'] is True


def test_same_config_is_idempotent_but_backend_change_conflicts() -> None:
    spawned: list[tuple[list[str], dict[str, Any]]] = []
    manager = _manager(spawned)

    first = manager.start(_request(backend='gazebo'))
    again = manager.start(_request(backend='gazebo'))

    assert len(spawned) == 1
    assert again == first
    assert again['config']['backend'] == 'gazebo'

    with pytest.raises(RuntimeError, match='already running'):
        manager.start(_request(backend='mujoco'))
    assert len(spawned) == 1


# --- 模型级仿真参数（生成高度 / 重力 / 固定基座）-----------------------------
#
# 前端「模型」抽屉调的就是这三个：只有 Gazebo 有实现，MuJoCo 在 MJCF 生成期固定
# （launch 侧同样拒绝），所以缺省不放命令行、给了 Mujoco 就显式报错。

def test_start_forwards_model_parameters_to_launch() -> None:
    spawned: list[tuple[list[str], dict[str, Any]]] = []
    manager = _manager(spawned)

    status = manager.start(_request(gravity=False, fixed_base=False, spawn_z=0.0027))

    command, _kwargs = spawned[0]
    assert 'gravity:=false' in command
    assert 'fixed_base:=false' in command
    assert 'spawn_z:=0.0027' in command
    assert status['config']['spawn_z'] == 0.0027
    assert status['config']['gravity'] is False


def test_start_omits_model_parameters_when_absent() -> None:
    spawned: list[tuple[list[str], dict[str, Any]]] = []
    manager = _manager(spawned)

    manager.start(_request())

    command, _kwargs = spawned[0]
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
    spawned: list[tuple[list[str], dict[str, Any]]] = []
    manager = _manager(spawned)

    with pytest.raises(ValueError, match=key):
        manager.start(_request(**{key: value}))

    assert spawned == []


def test_start_rejects_model_parameters_for_mujoco() -> None:
    spawned: list[tuple[list[str], dict[str, Any]]] = []
    manager = _manager(spawned)

    with pytest.raises(ValueError, match='mujoco'):
        manager.start(_request(backend='mujoco', gravity=False))

    assert spawned == []


# --- 模型枚举（root 可配置）--------------------------------------------------
#
# 模型列表来自节点参数 models_dir（可指源码目录，改模型不必重构建）；root 缺省
# 时回落 robot_sim 安装树。臂视图的映射/枚举**不在这里**——那批只读文件的函数
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
# 2026-10-14 起：HIL 启动仿真时模拟器由 bench_manager 的独立进程管理（「电机后端」，
# 可单独起停、随仿真联动；见 motor_sim_manager.py 与 test_motor_sim_manager.py），
# 本 launch 只编排被控对象/驱动端/Agent（start_fdcan_sim:=false）。

def _firmware_request(**overrides: Any) -> dict[str, Any]:
    request = _request(controller_mode='firmware', transport='socketcan',
                       sim_interface='can0', physical_bus_isolated=True)
    request.update(overrides)
    return request


def _firmware_manager(
    spawned: list[tuple[list[str], dict[str, Any]]],
) -> SimulationProcessManager:
    def fake_popen(command: list[str], **kwargs: Any) -> FakeProcess:
        spawned.append((command, kwargs))
        return FakeProcess()

    return SimulationProcessManager(
        can_provider=lambda: [{'name': 'can0', 'up': True, 'label': 'can0'}],
        serial_provider=lambda: [],
        model_provider=lambda: ['half_body_robot'],
        mujoco_model_provider=lambda: [],
        popen_factory=fake_popen,
    )


def test_firmware_start_defers_motor_backend_to_bench_manager() -> None:
    spawned: list[tuple[list[str], dict[str, Any]]] = []
    manager = _firmware_manager(spawned)

    status = manager.start(_firmware_request())

    command, _ = spawned[0]
    assert 'start_fdcan_sim:=false' in command
    assert not any(arg.startswith('sim_control_socket:=') for arg in command)
    assert status['config']['controller_mode'] == 'firmware'


# --- 全链：服务消息 → _simulation_request → _validate -------------------------
#
# 2026-10-16：这两段之间的缝漏过一次真错。_validate 要求非 vcan 请求带
# physical_bus_isolated，而 StartSimulation.srv 当时**没有**这个字段、
# _simulation_request 也不搬它——于是物理总线上的仿真永远起不来（socketcan/slcan
# 下 host / external / firmware 三种模式全被拒，只有 vcan 能起）。既有测试都直接
# 构造 dict 并预先塞好这个键，所以一路全绿。下面两条从**真实服务消息**出发，
# 把缝焊上：一条证明确认位能传到校验、一条证明不确认仍然被拒。


def _start_simulation_message(**fields: Any):
    from bench_msgs.srv import StartSimulation

    message = StartSimulation.Request()
    for key, value in fields.items():
        setattr(message, key, value)
    return message


def test_service_message_clears_validation_on_a_physical_bus() -> None:
    from bench_manager.node import _simulation_request

    spawned: list[tuple[list[str], dict[str, Any]]] = []
    manager = _firmware_manager(spawned)

    manager.start(_simulation_request(_start_simulation_message(
        backend='gazebo', transport='socketcan', controller_mode='firmware',
        model='half_body_robot', sim_interface='can0',
        physical_bus_isolated=True)))

    assert spawned, '确认位传到了却仍被拒——物理总线上的仿真起不来'


def test_service_message_without_confirmation_is_rejected() -> None:
    from bench_manager.node import _simulation_request

    spawned: list[tuple[list[str], dict[str, Any]]] = []
    manager = _firmware_manager(spawned)

    with pytest.raises(ValueError, match='no real motor load'):
        manager.start(_simulation_request(_start_simulation_message(
            backend='gazebo', transport='socketcan', controller_mode='firmware',
            model='half_body_robot', sim_interface='can0')))

    assert not spawned, '被拒了却起了进程'
