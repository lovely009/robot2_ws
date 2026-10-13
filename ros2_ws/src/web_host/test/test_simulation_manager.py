"""`SimulationProcessManager` 的单测（仿真面板后端）。

不产生真实子进程：`popen_factory` 注入假体，CAN 接口 / 串口 / 模型列表全部用桩，
所以本文件既不启 Gazebo、也不开 MuJoCo 窗口、更不碰 vcan 与串口。
"""

from __future__ import annotations

import signal
from typing import Any, Callable

import pytest

from web_host.simulation_manager import SimulationProcessManager


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
