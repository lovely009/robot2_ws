"""监督 `bench-gravity-feedforward.service`（重力前馈叠加节点）。

按需起停，**不随会话自动起**：HIL 会话常常是操作者在终端手起的，随会话起的方案在
那条路上不存在。会话描述（模型 / URDF / 关节映射 / 增益）存进 unit 环境，bench_manager
重启后仍读得回来。

适用面是「STM32 在环」：叠加节点输出话题 `/micro_ros_do/*` 的唯一订阅者是固件，纯
Gazebo / 纯 MuJoCo / `controller_mode:=host` 三路没有订阅者。边界见 ADR 0013。
"""

from __future__ import annotations

import math
import re
from collections.abc import Callable, Mapping
from pathlib import Path

from .systemd_units import (
    STATE_NAMES, UNIT_GRAVITY_FEEDFORWARD, SystemdUnits, UnitState,
    config_env, config_from_env,
)

# 命令流的两个落点（ADR 0014）：直连固件，或经叠加节点。**本文件是这两个名字的权威**
# ——切换由本包执行（改 command_streamer 的 output_topic），前端只读回执核对，所以
# web_host 的静态契约测试从这里对账，而不是在前端再抄一份字面量。
DIRECT_COMMAND_TOPIC = "/micro_ros_do/motor_command_array"
OVERLAY_COMMAND_TOPIC = "/robot_dynamics/command_array"

_CONFIG_ENV_PREFIX = "BENCH_FEEDFORWARD_"
# 模型名直接进文件路径：只认单段、无分隔符的名字，挡掉 `../../` 这类穿越。
_MODEL_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")
_MAX_MOTOR_ID = 65535


def default_urdf_provider(model: str) -> str:
    """`robot_sim` 安装树里该模型的 URDF 路径；找不到返回空串。

    用 `<model>.urdf` 而非 `<model>.standalone.urdf`：前者正是 web 侧模型枚举的判据，
    这样「面板里选得到」与「叠加节点起得来」是同一个谓词。两个文件的动力学内容实测
    等价（同关节集与轴、逐 link 同质量与惯量）。
    """
    from ament_index_python.packages import get_package_share_directory
    try:
        share = get_package_share_directory("robot_sim")
    except Exception:
        return ""
    path = Path(share) / "models" / model / "urdf" / f"{model}.urdf"
    return str(path) if path.is_file() else ""


def validate_joint_map(spec: str) -> str:
    """校验 `motor_id:关节名` 逗号分隔表，返回原串；畸形即抛 ValueError。

    叠加节点自己也会 fail-fast，但那要等 unit 起来再退出——一顿 systemd 往返之后
    页面拿到的是「failed」，说不清是表错了还是模型错了。这里先拒。
    """
    if not isinstance(spec, str) or not spec.strip():
        raise ValueError("joint_map 不能为空（换模型必须显式给表）")
    seen: dict[int, str] = {}
    for entry in spec.split(","):
        motor_id, sep, joint = entry.partition(":")
        if not sep or not joint:
            raise ValueError(f"joint_map 条目应为 motor_id:关节名：{entry!r}")
        try:
            numeric_id = int(motor_id)
        except ValueError:
            raise ValueError(f"joint_map 的 motor_id 不是整数：{motor_id!r}") from None
        if not 0 <= numeric_id <= _MAX_MOTOR_ID:
            raise ValueError(f"joint_map 的 motor_id 超出 0..{_MAX_MOTOR_ID}：{numeric_id}")
        if numeric_id in seen:
            raise ValueError(f"joint_map 的 motor_id 重复：{numeric_id}")
        seen[numeric_id] = joint
    return spec


def _gain(value: object) -> float:
    """增益的取值口径：非负、有限。0 = 逐字段直通（多一跳但没补）。"""
    try:
        gain = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        raise ValueError("effort_gain must be a number") from None
    if not math.isfinite(gain) or gain < 0.0:
        raise ValueError("effort_gain must be finite and non-negative")
    return gain


def _decode_gain(value: str):
    """环境变量里的增益读回为 float；值坏了就原样带出来，不让状态快照炸掉。"""
    try:
        return _gain(value)
    except ValueError:
        return value


class GravityFeedforwardProcessManager:
    """Own the `bench-gravity-feedforward.service` unit."""

    def __init__(self, urdf_provider: Callable[[str], str] | None = None,
                 gain_writer: Callable[[float], None] | None = None,
                 systemd: SystemdUnits | None = None):
        self._urdf_provider = urdf_provider or default_urdf_provider
        self._gain_writer = gain_writer
        self._systemd = systemd or SystemdUnits()
        # 最近一次**确认写入**的增益；None = 沿用 unit 环境里启动时的值。
        self._gain_override: float | None = None
        # 写入失败要说出来：静默没生效正是这个按钮要消除的失效模式。
        self._gain_error = ""
        # 命令流当前发往哪里：None = 本次进程内还没确认过（进程重启后由页面从
        # /motor_command/status 读真值，这里只报自己确认过的）。同 gain，失败要说出来。
        self._command_topic: str | None = None
        self._route_error = ""

    def _status(self):
        unit = self._systemd.probe(UNIT_GRAVITY_FEEDFORWARD)
        env: Mapping[str, str] = (
            {} if unit.state is UnitState.ABSENT
            else self._systemd.environment(UNIT_GRAVITY_FEEDFORWARD))
        config = config_from_env(_CONFIG_ENV_PREFIX, env, {"effort_gain": _decode_gain})
        if config is not None and self._gain_override is not None:
            config["effort_gain"] = self._gain_override
        return {
            "state": STATE_NAMES[unit.state],
            "config": config,
            "returncode": unit.returncode,
            "gain_error": self._gain_error,
            "command_topic": self._command_topic,
            "route_error": self._route_error,
        }

    def status(self):
        return self._status()

    def _validate(self, request):
        if not isinstance(request, dict):
            raise ValueError("request must be a JSON object")
        model = request.get("model")
        if not isinstance(model, str) or not _MODEL_PATTERN.match(model):
            raise ValueError("model must be a plain model-directory name")
        urdf_path = self._urdf_provider(model)
        if not urdf_path or not Path(urdf_path).is_file():
            raise ValueError(f"model {model!r} has no urdf/<model>.urdf under robot_sim")
        return {
            "model": model,
            "urdf_path": urdf_path,
            "joint_map": validate_joint_map(request.get("joint_map")),
            "effort_gain": _gain(request.get("effort_gain", 1.0)),
        }

    def _argv(self, config):
        # 增益写成 float 字面量（`1.0` 而不是 `1`）：`-p` 的整数会被当成 integer 类型，
        # 节点声明的是 double，类型不匹配会直接拒绝覆盖。
        return ["ros2", "run", "robot_dynamics", "gravity_feedforward_overlay",
                "--ros-args",
                "-p", f"urdf_path:={config['urdf_path']}",
                "-p", f"joint_map:={config['joint_map']}",
                "-p", f"effort_gain:={config['effort_gain']!r}"]

    def start(self, request):
        """启动或复用叠加节点；同一模型下改增益走参数通道，不重起。"""
        config = self._validate(request)
        status = self._status()
        if status["state"] == "running":
            current = status["config"] or {}
            same_topology = all(
                current.get(key) == config[key]
                for key in ("model", "urdf_path", "joint_map"))
            if same_topology:
                if current.get("effort_gain") != config["effort_gain"]:
                    self.set_gain(config["effort_gain"])
                return self._status()
            raise RuntimeError(
                "gravity feedforward is already running for another model; "
                "stop it first")
        self._gain_override = None
        self._gain_error = ""
        self._systemd.start(UNIT_GRAVITY_FEEDFORWARD, self._argv(config),
                            session_env=config_env(_CONFIG_ENV_PREFIX, config))
        return self._status()

    def stop(self):
        self._systemd.stop(UNIT_GRAVITY_FEEDFORWARD)
        self._gain_override = None
        self._gain_error = ""
        return self._status()

    def set_gain(self, value):
        """运行期改增益（不重起）。未运行时即拒——写入通道只对活着的节点有意义。"""
        gain = _gain(value)
        if self._status()["state"] != "running":
            raise RuntimeError("gravity feedforward is not running")
        if self._gain_writer is None:
            raise RuntimeError("no gain channel is available")
        self._gain_writer(gain)
        return gain

    def note_gain_applied(self, gain: float) -> None:
        """写入成功后由调用方回报——只有确认过的值才进状态。"""
        self._gain_override = float(gain)
        self._gain_error = ""

    def note_gain_failed(self, message: str) -> None:
        """写入失败由调用方回报：状态里带出来，页面能看见，不静默。"""
        self._gain_error = str(message)

    def note_route_applied(self, topic: str) -> None:
        """命令流改道成功（调用方确认回执后回报）。"""
        self._command_topic = str(topic)
        self._route_error = ""

    def note_route_failed(self, message: str) -> None:
        """命令流改道失败：状态里带出来。

        失败的**处置方向**由调用方决定，两条路不同（见 node.py 的两个服务处理）：
        开启时切不过去就让它留在直连（有命令、只是没补偿），停止时切不回来就**不许**
        停 unit（否则命令落进没人订阅的话题，固件命令超时软停，而机构没有抱闸）。
        """
        self._route_error = str(message)
