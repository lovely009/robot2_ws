"""测试守卫：web_host 的单测**不得**真起进程。

真实 Agent 进程管理由 bench_manager 负责，
那边有自己的守卫；web_host 现在只剩静态服务与文件域，本守卫防的是将来误加的
测试真起进程。确实需要真进程时请在那个测试里显式 monkeypatch 回来并说明原因。
"""

from __future__ import annotations

import subprocess
from pathlib import Path
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


@pytest.fixture
def legacy_model_reference(monkeypatch):
    # 旧半身映射仅在隔离夹具中验证，不作为发布模型或整机映射。
    reference = Path(__file__).parent / 'fixtures/robot_description'
    monkeypatch.setattr('web_host.shoulder_retarget.get_package_share_directory',
                        lambda _: str(reference))
    return reference
