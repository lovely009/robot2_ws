"""台架会话的 systemd 调用层——本包**唯一**碰 systemctl / systemd-run / journalctl 的地方。

为什么要有这一层：会话进程改由 systemd user transient unit 承载后，起停与状态查询全是外部
命令调用。收成一个可注入的 runner，单测才能在不启真 systemd 的前提下覆盖认领、复用、失败态
这些分支——CI 与容器里往往没有 user manager。

三条实测事实决定了这里的形状（2026-10-16 在本机核过）：

* 不存在的 unit 与「干净停过」的 unit 在 `ActiveState` 上**无法区分**——两者都答
  `inactive/dead/ExecMainStatus=0`。必须一起看 `LoadState`（`not-found` vs `loaded`）。
* `systemctl --user stop` 对不存在的 **service** 返回 exit 5「Unit not loaded.」，
  对不存在的 **slice** 返回 exit 0。两者都得当成功，否则 `deactivate` 卡在 deactivating。
* transient **service** 跑在 "clean and detached execution environment" 里，**不继承调用者
  环境**（user manager 自己的 47 个变量里零个 ROS 变量）。ROS 的发现路径只能靠
  `ros_environment()` 白名单显式带进去。
"""

from __future__ import annotations

import os
import subprocess
from enum import Enum
from typing import Callable, Mapping, NamedTuple

# unit 名与 slice 名是**契约**（web 面板、文档、ADR 都引用它们）：字面量只在这里定义一次。
UNIT_SIMULATION = "bench-simulation.service"
UNIT_MOTOR_BACKEND = "bench-motor-backend.service"
UNIT_AGENT = "bench-agent.service"
SLICE = "bench.slice"

# auto 归属（「随仿真拉起」）持久化在 unit 的环境里：只活在内存里的话，bench_manager
# 一重启就把随仿真起的误判成手动起的，停仿真时不敢停它。
AUTO_STARTED_ENV = "BENCH_AUTO_STARTED"

_TIMEOUT_S = 10.0
# `systemctl stop` 对不存在的 unit 的退出码（"Unit not loaded."）。当「已经停了」。
_NOT_LOADED_RC = 5

# 环境白名单：transient service 不继承调用者环境，ROS 的发现路径全靠这里带进去。
_ENV_PREFIXES = ("AMENT_", "ROS_", "RMW_", "RCUTILS_", "COLCON_",
                 "CYCLONEDDS_", "FASTDDS_")
_ENV_EXACT = ("PATH", "LD_LIBRARY_PATH", "PYTHONPATH")


class UnitState(Enum):
    ABSENT = "absent"    # 从未创建（LoadState=not-found）
    STOPPED = "stopped"  # 创建过、停着
    RUNNING = "running"
    FAILED = "failed"    # 上次非零退出；returncode 带退出码


class UnitStatus(NamedTuple):
    state: UnitState
    returncode: int | None = None


def ros_environment(environ: Mapping[str, str]) -> dict[str, str]:
    """从 `environ` 里挑出 unit 内跑 ROS 必需的变量。

    白名单而非黑名单：漏掉一个变量会在 unit 里表现成「包找不到」或「加进了另一个 DDS 域」，
    都是要翻 systemd 日志才看得见的错，所以宁可显式列全。
    """
    return {name: value for name, value in environ.items()
            if name in _ENV_EXACT or name.startswith(_ENV_PREFIXES)}


def _default_runner(argv: list[str], timeout_s: float) -> subprocess.CompletedProcess:
    return subprocess.run(argv, capture_output=True, text=True,
                          timeout=timeout_s, check=False)


def _parse_properties(text: str) -> dict[str, str]:
    out: dict[str, str] = {}
    for line in text.splitlines():
        key, sep, value = line.partition("=")
        if sep:
            out[key] = value
    return out


def _parse_environment(raw: str) -> dict[str, str]:
    """`systemctl show -p Environment` 的格式是空格分隔的 `K=V`（值带空格时加引号）。"""
    out: dict[str, str] = {}
    for token in raw.split():
        key, sep, value = token.partition("=")
        if sep:
            out[key] = value.strip('"')
    return out


def _exit_code(raw: str | None) -> int | None:
    try:
        return int(raw)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None


class SystemdUnits:
    """三个台架会话 unit 的起停与状态查询。

    `runner` 的契约是 `(argv, timeout_s) -> CompletedProcess`（默认 `subprocess.run`）；
    单测注入假体，生产不换。
    """

    def __init__(self, runner: Callable[..., subprocess.CompletedProcess] | None = None,
                 environ: Mapping[str, str] | None = None) -> None:
        self._run = runner or _default_runner
        self._environ = dict(os.environ if environ is None else environ)

    # --- 探测 ---------------------------------------------------------------

    def available(self) -> bool:
        """user manager / session bus 是否可达。`configure` 阶段拿它 fail-fast。"""
        return self._run(["systemctl", "--user", "show-environment"],
                         _TIMEOUT_S).returncode == 0

    def probe(self, unit: str) -> UnitStatus:
        result = self._run(["systemctl", "--user", "show", unit, "-p",
                            "LoadState,ActiveState,SubState,ExecMainStatus"],
                           _TIMEOUT_S)
        props = _parse_properties(result.stdout)
        # LoadState 是关键：只看 ActiveState 的话，「从未创建」与「干净停过」逐字相同。
        if props.get("LoadState") == "not-found":
            return UnitStatus(UnitState.ABSENT)
        active = props.get("ActiveState")
        if active == "active":
            return UnitStatus(UnitState.RUNNING)
        if active == "failed":
            return UnitStatus(UnitState.FAILED,
                              _exit_code(props.get("ExecMainStatus")))
        return UnitStatus(UnitState.STOPPED)

    def environment(self, unit: str) -> dict[str, str]:
        """读 unit 的环境——`AUTO_STARTED_ENV` 等归属标记就存在这里。"""
        result = self._run(["systemctl", "--user", "show", unit, "-p", "Environment"],
                           _TIMEOUT_S)
        return _parse_environment(_parse_properties(result.stdout).get("Environment", ""))

    # --- 起停 ---------------------------------------------------------------

    def start(self, unit: str, argv: list[str], auto: bool = False) -> UnitStatus:
        """让 unit 以**当前** argv 跑起来；已在跑则原样认领，不重启。

        重启会打断正在跑的仿真，所以 RUNNING 直接返回。这也正是 systemd 免费给的
        「单写者所有权」：unit 名被占用时第二个 bench_manager 根本建不出来，
        「在跑」就意味着「有人管着」。

        注意没有 `systemctl start` 分支：transient unit 干净退出后会被卸载，非干净退出
        留在 failed 态；两种情况都该按当前 argv **重建**，而不是拿旧定义启动。
        """
        status = self.probe(unit)
        if status.state is UnitState.RUNNING:
            return status
        if status.state is not UnitState.ABSENT:
            # 名字仍被占着（failed 态，或加载着但停着）：不先清干净，systemd-run 会
            # 直接报 "Unit already exists"。
            self.stop(unit)
            self._reset_failed(unit)
        self._create(unit, argv, auto)
        return self.probe(unit)

    def stop(self, unit: str) -> None:
        result = self._run(["systemctl", "--user", "stop", unit], _TIMEOUT_S)
        if result.returncode not in (0, _NOT_LOADED_RC):
            detail = (result.stderr or result.stdout).strip()
            raise RuntimeError(f"无法停止 {unit}：{detail}")

    def stop_slice(self) -> None:
        """停掉整个会话（`bench.slice` 下全部 unit）。

        这是「安全停止」的唯一显式语义，也是 lifecycle `deactivate` 的实现。首次运行时
        slice 不存在，`stop` 返回 exit 0——不必特判。
        """
        result = self._run(["systemctl", "--user", "stop", SLICE], _TIMEOUT_S)
        if result.returncode not in (0, _NOT_LOADED_RC):
            detail = (result.stderr or result.stdout).strip()
            raise RuntimeError(f"无法停止 {SLICE}：{detail}")

    def log(self, unit: str, lines: int) -> str:
        result = self._run(["journalctl", "--user", "-u", unit, "-n", str(lines),
                            "--output=cat", "--no-pager"], _TIMEOUT_S)
        return result.stdout if result.returncode == 0 else ""

    # --- 内部 ---------------------------------------------------------------

    def _reset_failed(self, unit: str) -> None:
        # 失败态清掉后 unit 才会被卸载；此时再 systemd-run 才不会撞名字。
        # 它本身失败不算错（unit 可能已经不在了）。
        self._run(["systemctl", "--user", "reset-failed", unit], _TIMEOUT_S)

    def _create(self, unit: str, argv: list[str], auto: bool) -> None:
        command = ["systemd-run", "--user", f"--unit={unit}", f"--slice={SLICE}"]
        for name, value in ros_environment(self._environ).items():
            command.append(f"--setenv={name}={value}")
        command.append(f"--setenv={AUTO_STARTED_ENV}={'1' if auto else '0'}")
        command.extend(argv)
        result = self._run(command, _TIMEOUT_S)
        if result.returncode == 0:
            return
        detail = (result.stderr or result.stdout).strip()
        if "already exists" in detail:
            raise RuntimeError(
                f"{unit} 已被另一个 bench_manager 持有（systemd 拒绝重名 unit）。"
                "同一台机器上只允许一个台架会话编排者——先停掉另一个再试。")
        raise RuntimeError(f"无法创建 {unit}：{detail}")
