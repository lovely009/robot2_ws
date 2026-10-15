"""测试守卫：bench_manager 的单测**不得**真起进程。

bench_manager 能起 `ros2 launch robot_sim simulation.launch.py`（会拉起 Gazebo 或
MuJoCo 窗口、创建 vcan），`AgentProcessManager` 能起 micro-ROS Agent 抢串口——一个误写
的测试就能真占端口、真开窗口。这里把 `subprocess.Popen` 换成「一调用就报错」，逼测试
显式注入 `popen_factory` 假体（现有测试都这么做）。某个测试确实需要真进程时，请在那个
测试里显式 monkeypatch 回来，并说明为什么。
"""

from __future__ import annotations

import subprocess
from typing import Any

import pytest


@pytest.fixture(autouse=True)
def forbid_real_subprocess(monkeypatch: pytest.MonkeyPatch) -> None:
    def guarded(*args: Any, **kwargs: Any) -> None:
        target = args[0] if args else kwargs.get('args')
        raise AssertionError(
            f'test is not allowed to spawn a real process: {target!r}'
            '（请注入 popen_factory 假体）'
        )

    monkeypatch.setattr(subprocess, 'Popen', guarded)
