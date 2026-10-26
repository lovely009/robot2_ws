"""台架会话的 systemd 调用层——本包**唯一**碰 systemctl / systemd-run / journalctl 的地方。

通过可注入 runner 调用 systemd user transient unit；测试无需真实 user manager。

systemd 249 的状态约束：

* 不存在的 unit 与「干净停过」的 unit 在 `ActiveState` 上**无法区分**——两者都答
  `inactive/dead/ExecMainStatus=0`。必须一起看 `LoadState`（`not-found` vs `loaded`）。
* **干净退出与失败退出的持久性不同**：退出码为 0 的 transient unit 当场被卸载
  （`LoadState=not-found`，`Environment` 一并消失）；非零退出的**留得住**
  （`LoadState=loaded` / `ActiveState=failed`，`ExecMainStatus` 与 `Environment` 都在）。
  所以「报退出码」只对失败态有意义，而失败态恰好是页面唯一需要报退出码的情形。
* `systemctl --user stop` 对不存在的 **service** 返回 exit 5「Unit not loaded.」，
  对不存在的 **slice** 返回 exit 0。两者都得当成功，否则 `deactivate` 卡在 deactivating。
* transient **service** 跑在 "clean and detached execution environment" 里，**不继承调用者
  环境**。ROS 的发现路径只能靠
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
UNIT_GRAVITY_FEEDFORWARD = "bench-gravity-feedforward.service"
UNIT_FEEDBACK = "bench-firmware-feedback.service"
SLICE = "bench.slice"

# 自动启动归属随 unit 环境持久化，重启后仍按归属联动停止。
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


# 页面与 srv 契约里的三个词。`absent`（从未创建）与 `stopped`（干净停过）同义：
# 页面不必区分，而 `LoadState` 已经把区分做在了 UnitState 这一层。
STATE_NAMES = {
    UnitState.ABSENT: "stopped",
    UnitState.STOPPED: "stopped",
    UnitState.RUNNING: "running",
    UnitState.FAILED: "failed",
}


def _encode(value: object) -> str:
    return ("true" if value else "false") if isinstance(value, bool) else str(value)


def config_env(prefix: str, config: Mapping[str, object]) -> dict[str, str]:
    """会话描述（跑的是哪个模型、哪条总线、哪台设备）→ unit 的环境变量。

    会话描述随 unit 持久化，管理器重启后可恢复。

    **值必须是无空白、无引号的单词**：`systemctl show -p Environment` 对含空格或 `"`
    的值做 C 风格转义（实测 `{"a":"b c"}` 出来是 `"{\\"a\\":\\"b c\\"}"`），而读回是按
    空白切词的，塞 JSON 会静默解析成半个对象。所以一个键一个变量，编码失败宁可当场报错。
    """
    env: dict[str, str] = {}
    for key, value in config.items():
        text = _encode(value)
        if any(char.isspace() or char == '"' for char in text):
            raise ValueError(
                f"{key} 的值含空白或引号，systemd 会转义它、读回来会散架：{text!r}")
        env[prefix + key.upper()] = text
    return env


def config_from_env(prefix: str, env: Mapping[str, str],
                    types: Mapping[str, Callable[[str], object]] | None = None
                    ) -> dict[str, object] | None:
    """`config_env()` 的逆；环境里没有本前缀的键时返回 None（＝没在跑）。

    `types` 给出需要还原类型的键（bool / float）；其余按字符串读回。类型信息只能由
    调用方给——环境变量本身是无类型的。
    """
    decoders = types or {}
    config: dict[str, object] = {}
    for name, value in env.items():
        if not name.startswith(prefix):
            continue
        key = name[len(prefix):].lower()
        config[key] = decoders.get(key, str)(value)
    return config or None


def ros_environment(environ: Mapping[str, str]) -> dict[str, str]:
    """从 `environ` 里挑出 unit 内跑 ROS 必需的变量。

    白名单而非黑名单：漏掉一个变量会在 unit 里表现成「包找不到」或「加进了另一个 DDS 域」，
    都是要翻 systemd 日志才看得见的错，所以宁可显式列全。
    """
    return {name: value for name, value in environ.items()
            if name in _ENV_EXACT or name.startswith(_ENV_PREFIXES)}


def _default_runner(argv: list[str], timeout_s: float) -> subprocess.CompletedProcess:
    try:
        return subprocess.run(argv, capture_output=True, text=True,
                              timeout=timeout_s, check=False)
    except subprocess.TimeoutExpired:
        # user manager 卡住时不能让服务回调炸掉（`TimeoutExpired` 不是 `OSError`，
        # 会穿透调用方的 except）。当成一条退出码 124 的失败命令交回去，由各调用点
        # 按自己的语义处理：`available()` 变 False（configure fail-fast），
        # `stop()` 报无法停止，`probe()` 落到 STOPPED。
        return subprocess.CompletedProcess(
            argv, 124, "",
            f"{' '.join(argv)} 在 {timeout_s:g}s 内没有返回")


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
    """台架会话 unit 的起停与状态查询。

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

    def start(self, unit: str, argv: list[str], auto: bool = False,
              session_env: Mapping[str, str] | None = None) -> UnitStatus:
        """让 unit 以**当前** argv 跑起来；已在跑则原样认领，不重启。

        RUNNING 时直接返回，避免打断会话；unit 名唯一，防止重复创建。

        `session_env` 持久化设备、模型与总线等会话描述，通过 `environment()` 读回。

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
        self._create(unit, argv, auto, session_env or {})
        return self.probe(unit)

    def stop(self, unit: str) -> None:
        result = self._run(["systemctl", "--user", "stop", unit], _TIMEOUT_S)
        if result.returncode not in (0, _NOT_LOADED_RC):
            detail = (result.stderr or result.stdout).strip()
            raise RuntimeError(f"无法停止 {unit}：{detail}")

    def stop_slice(self) -> None:
        """停掉整个会话（`bench.slice` 下全部 unit），并清掉失败态的残留。

        这是「安全停止」的唯一显式语义，也是 lifecycle `deactivate` 的实现。首次运行时
        slice 不存在，`stop` 返回 exit 0——不必特判。

        `stop` 只停**活跃**的 unit：非零退出的那份已经 inactive，会留在 `failed` 态
        （systemd 249）。必须清理残留，避免下次激活时认领过期失败。
        """
        result = self._run(["systemctl", "--user", "stop", SLICE], _TIMEOUT_S)
        if result.returncode not in (0, _NOT_LOADED_RC):
            detail = (result.stderr or result.stdout).strip()
            raise RuntimeError(f"无法停止 {SLICE}：{detail}")
        for unit in (UNIT_SIMULATION, UNIT_MOTOR_BACKEND, UNIT_AGENT,
                     UNIT_GRAVITY_FEEDFORWARD, UNIT_FEEDBACK):
            self._reset_failed(unit)

    def log(self, unit: str, lines: int) -> str:
        result = self._run(["journalctl", "--user", "-u", unit, "-n", str(lines),
                            "--output=cat", "--no-pager"], _TIMEOUT_S)
        return result.stdout if result.returncode == 0 else ""

    # --- 内部 ---------------------------------------------------------------

    def _reset_failed(self, unit: str) -> None:
        # 失败态清掉后 unit 才会被卸载；此时再 systemd-run 才不会撞名字。
        # 它本身失败不算错（unit 可能已经不在了）。
        self._run(["systemctl", "--user", "reset-failed", unit], _TIMEOUT_S)

    def _create(self, unit: str, argv: list[str], auto: bool,
                session_env: Mapping[str, str]) -> None:
        command = ["systemd-run", "--user", f"--unit={unit}", f"--slice={SLICE}"]
        for name, value in ros_environment(self._environ).items():
            command.append(f"--setenv={name}={value}")
        command.append(f"--setenv={AUTO_STARTED_ENV}={'1' if auto else '0'}")
        for name, value in session_env.items():
            command.append(f"--setenv={name}={value}")
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
