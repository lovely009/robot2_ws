"""监督 `bench-motor-backend.service`（独立的 FDCAN 电机后端）。

「电机后端」= fdcan_motor_sim.py 的独立进程：页面/脚本经 StartMotorBackend 服务
按需单独起停（不依赖仿真——只连板子即可验证固件发现/离线/重连）；HIL（firmware）
+ 物理总线启动仿真时由 StartSimulation 服务默认联动拉起、停止时联动收起（见
hil_motor_backend_request 与 auto_started 标志）。电机表与增删断连命令经
query_motor_sim 转发到控制通道（协议见 micro_ros_agent/README.md）。

进程归属在 systemd（user transient unit）：控制通道路径因此是**固定**的——以前它
带 `bench_manager` 的 pid，unit 化之后那个 pid 跟 unit 没关系，而且重启就变。
"""

import json
import os
import socket

from .agent_manager import enumerate_serial_ports
from .simulation_manager import (
    enumerate_can_interfaces,
    enumerate_simulation_models,
)
from .systemd_units import (
    AUTO_STARTED_ENV, STATE_NAMES, UNIT_MOTOR_BACKEND, SystemdUnits, UnitState,
    config_env, config_from_env,
)

_CONTROL_MAX_RESPONSE_BYTES = 65536
# 电机后端只接物理/虚拟 CAN 端点；纯 vcan 软件闭环的总线电机是 motor_sim
# 节点（随仿真自带），不需要本后端。
_BACKEND_TRANSPORTS = ("socketcan", "slcan")
# 控制通道的固定路径（见模块 docstring）。终端路径与 unit 路径共用它，靠
# fdcan_motor_sim.py 的 flock 互斥，不靠路径区分。
_CONTROL_SOCKET = "/tmp/robot_sim_motor_backend.sock"
_CONFIG_ENV_PREFIX = "BENCH_MOTOR_"


def query_motor_sim(socket_path: str, request: dict,
                    timeout_s: float = 2.0) -> dict:
    """向 FDCAN 模拟器的控制通道发一条命令（Unix socket，一问一答）。

    模拟器一连接一命令（见 fdcan_motor_sim.py 的 _service_control_socket），
    所以每次调用新建连接；连接/超时/非法响应统一抛 RuntimeError。
    """
    payload = (json.dumps(request, ensure_ascii=False) + "\n").encode("utf-8")
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
            client.settimeout(timeout_s)
            client.connect(socket_path)
            client.sendall(payload)
            buffer = b""
            while (b"\n" not in buffer
                   and len(buffer) < _CONTROL_MAX_RESPONSE_BYTES):
                chunk = client.recv(65536)
                if not chunk:
                    break
                buffer += chunk
    except OSError as exc:
        raise RuntimeError(f"模拟器控制通道不可用：{exc}") from exc
    if b"\n" not in buffer:
        raise RuntimeError("模拟器控制通道无响应")
    try:
        response = json.loads(buffer.split(b"\n", 1)[0].decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise RuntimeError("模拟器控制通道返回了非法响应") from exc
    if not isinstance(response, dict):
        raise RuntimeError("模拟器控制通道返回了非法响应")
    return response


def hil_motor_backend_request(request) -> dict | None:
    """从仿真启动请求推出电机后端的启动请求；不需要时返回 None。

    规则（2026-10-14 与用户对齐）：仅 STM32 固件 HIL + 物理总线
    （socketcan/slcan）默认拉起；vcan 与其他控制端模式的总线电机是软件
    motor_sim 节点，不涉及 fdcan_motor_sim。
    """
    if not isinstance(request, dict):
        return None
    if request.get("controller_mode") != "firmware":
        return None
    transport = request.get("transport")
    if transport not in _BACKEND_TRANSPORTS:
        return None
    backend_request = {
        "model": request.get("model"),
        "transport": transport,
        # 与仿真请求同源：确认位由页面传下来，这里不再自己写死 True。写死会让
        # 未确认时后端先起、再被仿真那侧的校验拒掉后回滚——白起停一次进程，而
        # 回滚只兜 OSError。传下来则后端就地拒绝，报的还是同一条原因。
        "physical_bus_isolated": request.get("physical_bus_isolated") is True,
    }
    if transport == "socketcan":
        backend_request["sim_interface"] = request.get("sim_interface")
    else:
        backend_request["sim_tty"] = request.get("sim_tty")
    return backend_request


class MotorSimProcessManager:
    """Own the `bench-motor-backend.service` unit and forward its control commands."""

    def __init__(self, can_provider=None, serial_provider=None,
                 model_provider=None, systemd=None):
        self._can_provider = can_provider or enumerate_can_interfaces
        self._serial_provider = serial_provider or enumerate_serial_ports
        self._model_provider = model_provider or enumerate_simulation_models
        self._systemd = systemd or SystemdUnits()

    def _validate(self, request):
        if not isinstance(request, dict):
            raise ValueError("request must be a JSON object")
        transport = request.get("transport")
        if transport not in _BACKEND_TRANSPORTS:
            raise ValueError(
                "电机后端需要物理总线（socketcan 或 slcan）；vcan 纯软件"
                "闭环的电机由 motor_sim 节点扮演")
        model = request.get("model")
        if not isinstance(model, str) or model not in self._model_provider():
            raise ValueError("model has no installed full-simulation topology")
        if request.get("physical_bus_isolated") is not True:
            raise ValueError(
                "confirm that the physical CAN bus has no real motor load")
        config = {
            "model": model,
            "transport": transport,
            # 控制通道路径是常量；放进 config 让页面与幂等比较都从同一处取
            "motor_sim_control_socket": _CONTROL_SOCKET,
        }
        if transport == "socketcan":
            available = {
                item["name"]: item for item in self._can_provider()
                if isinstance(item, dict) and isinstance(item.get("name"), str)
            }
            name = request.get("sim_interface")
            if not isinstance(name, str) or name not in available:
                raise ValueError(
                    "select an enumerated CAN interface for the motor backend")
            if not available[name].get("up"):
                raise ValueError(f"SocketCAN interface {name} is down")
            config["sim_interface"] = name
        else:
            ports = {item["path"]: item for item in self._serial_provider()}
            path = request.get("sim_tty")
            if not isinstance(path, str) or path not in ports:
                raise ValueError(
                    "select an enumerated serial device for the motor backend")
            if not os.access(path, os.R_OK | os.W_OK):
                raise PermissionError(
                    f"bench_manager cannot read and write {path}")
            config["sim_tty"] = path
        return config

    def _status(self):
        """状态与配置都从 unit 派生（见 simulation_manager._status 的同款理由）。"""
        unit = self._systemd.probe(UNIT_MOTOR_BACKEND)
        env = ({} if unit.state is UnitState.ABSENT
               else self._systemd.environment(UNIT_MOTOR_BACKEND))
        return {
            "state": STATE_NAMES[unit.state],
            "config": config_from_env(_CONFIG_ENV_PREFIX, env),
            "returncode": unit.returncode,
            "auto_started": env.get(AUTO_STARTED_ENV) == "1",
        }

    def status(self):
        return self._status()

    def motor_request(self, request: dict) -> dict:
        """转发一条模拟器控制命令；后端没在跑时抛 RuntimeError。"""
        config = self._status()["config"]
        if not config:
            raise RuntimeError("电机后端未运行，模拟器控制通道不可用")
        return query_motor_sim(config["motor_sim_control_socket"], request)

    def start(self, request, auto=False):
        """启动或复用后端。auto 标记「随仿真拉起」，供停止联动判断。"""
        config = self._validate(request)
        status = self._status()
        if status["state"] == "running":
            if status["config"] == config:
                # 幂等复用；**不改** auto 归属（用户手动起的不能因停仿真被停）
                return status
            raise RuntimeError(
                "motor backend is already running with a different "
                "configuration; stop it first")

        command = [
            "ros2", "launch", "robot_sim", "simulation.launch.py",
            "simulator_only:=true",
            "controller_mode:=firmware",
            f"transport:={config['transport']}",
            f"model:={config['model']}",
            "start_web_host:=false",
            "start_micro_ros_agent:=false",
            f"sim_control_socket:={config['motor_sim_control_socket']}",
        ]
        if config["transport"] == "socketcan":
            command.append(f"sim_interface:={config['sim_interface']}")
        else:
            command.append(f"sim_tty:={config['sim_tty']}")

        self._systemd.start(UNIT_MOTOR_BACKEND, command, auto=auto,
                            session_env=config_env(_CONFIG_ENV_PREFIX, config))
        return self._status()

    def stop(self):
        self._systemd.stop(UNIT_MOTOR_BACKEND)
        return self._status()
