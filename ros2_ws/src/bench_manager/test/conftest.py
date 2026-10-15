"""测试守卫与共享假体。

守卫：bench_manager 的单测**不得**真起进程、真碰 systemd。它既能起
`ros2 launch robot_sim simulation.launch.py`（会拉起 Gazebo 或 MuJoCo 窗口、创建 vcan），
也能经 `systemd-run` 在用户会话里留下常驻 unit，还能起 micro-ROS Agent 抢串口——一个
误写的测试就能真占端口、真开窗口、真在系统里留东西。这里把 `subprocess.Popen` 与
`subprocess.run` 都换成「一调用就报错」，逼测试显式注入假体。某个测试确实需要真进程时，
请在那个测试里显式 monkeypatch 回来，并说明为什么。
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
    """最小 systemd 假体：只建模本仓库用到的命令与 unit 属性。

    它**不是**只记 argv 的哑替身——`properties[unit]` 建模的是 `systemctl --user show`
    会答的属性表，所以认领 / 复用 / 失败态这些分支才测得出来（只记 argv 的话，状态权威
    本身就没被验证）。

    `fail[argv]` 指定某条命令的退出码与 stderr，用来建模 "Unit not loaded." 这类边界。
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
        """建模**状态迁移**——这是本假体存在的理由。

        只记 argv 的哑替身答不出 `ActiveState`，于是「认领不重启」「失败带退出码」
        这些分支根本测不到，而它们正是把状态权威从 `poll()` 换成 systemd 的全部意义。

        迁移按 transient unit 的真实行为建模：`systemd-run` 建出来就是 active；
        `stop` 之后 unit 被卸载（回到 not-found）；`reset-failed` 清掉失败态后同样
        被卸载——所以下一次 `systemd-run` 才建得起来。
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
