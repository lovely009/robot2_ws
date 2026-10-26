"""测试守卫与共享假体。

禁止测试创建真实进程、触碰 systemd、串口或仿真窗口。
Popen/run 默认报错，测试须显式注入假体；确需真进程时须局部解除并说明原因。
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
            '（请注入 popen_factory / systemd 假体）'
        )

    monkeypatch.setattr(subprocess, 'Popen', guarded)
    monkeypatch.setattr(subprocess, 'run', guarded)


class FakeSystemd:
    """最小 systemd 假体，建模命令与 unit 属性。

    properties[unit] 对应 systemctl show 属性，用于验证认领、复用和失败状态。
    fail[argv] 指定退出码与 stderr，覆盖 unit 未加载等边界。
    """

    def __init__(self, properties: dict[str, dict[str, str]] | None = None,
                 fail: dict[tuple[str, ...], tuple[int, str]] | None = None) -> None:
        self.calls: list[list[str]] = []
        self.properties = dict(properties or {})
        self.fail = dict(fail or {})

    def __call__(self, argv: list[str], timeout_s: float) -> subprocess.CompletedProcess:
        self.calls.append(list(argv))
        rc, err = self.fail.get(tuple(argv), (0, ''))
        if rc == 0:
            self._apply(argv)
        out = ''
        if argv[1:3] == ['--user', 'show']:
            out = ''.join(f'{k}={v}\n'
                          for k, v in self.properties.get(argv[3], {}).items())
        return subprocess.CompletedProcess(argv, rc, out, err)

    def _apply(self, argv: list[str]) -> None:
        """建模 transient unit 状态迁移：创建后 active；
        stop 或 reset-failed 后卸载为 not-found，允许再次创建。
        """
        def setenv() -> str:
            return ' '.join(a.split('=', 1)[1] for a in argv
                            if a.startswith('--setenv='))

        if argv[0] == 'systemd-run':
            unit = next(a.split('=', 1)[1] for a in argv if a.startswith('--unit='))
            self.properties[unit] = {
                'LoadState': 'loaded', 'ActiveState': 'active',
                'SubState': 'running', 'ExecMainStatus': '0',
                'Environment': setenv(),
            }
        elif argv[0] == 'systemctl' and argv[1:3] == ['--user', 'stop']:
            self.properties[argv[3]] = {'LoadState': 'not-found'}
        elif (argv[0] == 'systemctl' and len(argv) > 3
              and argv[1:3] == ['--user', 'reset-failed']):
            self.properties[argv[3]] = {'LoadState': 'not-found'}

    def commands(self) -> list[list[str]]:
        return self.calls
