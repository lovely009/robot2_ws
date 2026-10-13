"""`SimulationProcessManager` 的单测（仿真面板后端）。

不产生真实子进程：`popen_factory` 注入假体，CAN 接口 / 串口 / 模型列表全部用桩，
所以本文件既不启 Gazebo、也不开 MuJoCo 窗口、更不碰 vcan 与串口。
"""

from __future__ import annotations

import signal
from typing import Any, Callable

import pytest

from web_host.simulation_manager import (
    SimulationProcessManager,
    enumerate_simulation_models,
    simulation_model_joints,
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


# --- simulation_model_joints：臂视图映射的权威来源 ---------------------------
#
# 背景：这张 motor_id → 关节名 映射原先硬编码在前端 store.js、写死成
# half_body_robot 的 18 台，换模型就**静默失效**（关节名对不上 → 那些关节角恒为 0、
# 臂上小球点不中，表现得像"电机没注册"）。现在改为向模型自己的
# config/motor_driver.yaml 取。这几条守的就是"换模型不再静默失效"。

def test_model_joints_covers_every_installed_model() -> None:
    models = enumerate_simulation_models()
    if not models:
        pytest.skip('安装树里没有完整模型，测试前提不成立')
    for model in models:
        joints = simulation_model_joints(model)
        assert joints, f'{model} 取不出 id→关节映射'
        for motor_id, joint in joints.items():
            assert isinstance(motor_id, int) and motor_id > 0
            assert isinstance(joint, str) and joint, f'{model} 的 id {motor_id} 关节名为空'


def test_model_joints_pins_current_half_body_model() -> None:
    """钉住当前唯一模型的内容，与 simulation_topology.yaml 逐条一致。"""
    joints = simulation_model_joints('half_body_robot')
    assert set(joints) == set(range(1, 19)), '必须是 1..18 连续——曾经漏掉 10..18'
    assert joints[1] == 'joint_link1'
    assert joints[9] == 'joint_link9'
    assert joints[10] == 'joint_link1.1'   # 第二链首台
    assert joints[18] == 'joint_link9.9'   # 第二链末台
    # 两条链的关节名不能混：第一链无点号，第二链有点号
    assert all('.' not in joints[i] for i in range(1, 10))
    assert all('.' in joints[i] for i in range(10, 19))


@pytest.mark.parametrize('bad', ['', '..', '../etc', 'a/b', 'nope_does_not_exist'])
def test_model_joints_rejects_bad_model_names(bad: str) -> None:
    """非法模型名必须显式报错，不能悄悄返回空表或读到目录外。"""
    with pytest.raises(ValueError):
        simulation_model_joints(bad)
