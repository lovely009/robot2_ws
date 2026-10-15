"""Supervise the standalone FDCAN motor backend (`simulator_only` launch)。

「电机后端」= fdcan_motor_sim.py 的独立进程：页面/脚本经 StartMotorBackend 服务
按需单独起停（不依赖仿真——只连板子即可验证固件发现/离线/重连）；HIL（firmware）
+ 物理总线启动仿真时由 StartSimulation 服务默认联动拉起、停止时联动收起（见
hil_motor_backend_request 与 auto_started 标志）。控制通道（Unix socket）路径由本
模块生成并随 launch 透传；电机表与增删断连命令经 query_motor_sim 转发（协议见
micro_ros_agent/README.md）。
"""

from collections import deque
import json
import os
import signal
import socket
import subprocess
import threading

from .agent_manager import enumerate_serial_ports
from .simulation_manager import (
    enumerate_can_interfaces,
    enumerate_simulation_models,
)

_LOG_LINES = 80
_LOG_LINE_CHARS = 1000
_CONTROL_MAX_RESPONSE_BYTES = 65536
# 电机后端只接物理/虚拟 CAN 端点；纯 vcan 软件闭环的总线电机是 motor_sim
# 节点（随仿真自带），不需要本后端。
_BACKEND_TRANSPORTS = ("socketcan", "slcan")
# 终端启动 HIL 时，simulation.launch.py 内嵌拉起模拟器并落在**固定路径**上
# （见 robot_sim/launch/simulation.launch.py 的同名路径，两边必须一致）。
# 本模块自己没有进程时接管它：只能转发命令，不能起停——那份进程的主人是 launch。
_EXTERNAL_CONTROL_SOCKET = "/tmp/robot_sim_motor_backend.sock"


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
        "physical_bus_isolated": True,
    }
    if transport == "socketcan":
        backend_request["sim_interface"] = request.get("sim_interface")
    else:
        backend_request["sim_tty"] = request.get("sim_tty")
    return backend_request


class MotorSimProcessManager:
    """Own at most one standalone FDCAN motor backend process and its log."""

    def __init__(self, can_provider=None, serial_provider=None,
                 model_provider=None, popen_factory=None, stop_timeout_s=8,
                 external_socket=_EXTERNAL_CONTROL_SOCKET):
        # external_socket=None 关掉「接管 launch 内嵌模拟器」——测试必须显式关，
        # 否则台架上真跑着 HIL 时 status() 会答 external，测试就随台架状态红绿。
        self._external_socket = external_socket
        self._can_provider = can_provider or enumerate_can_interfaces
        self._serial_provider = serial_provider or enumerate_serial_ports
        self._model_provider = model_provider or enumerate_simulation_models
        self._popen_factory = popen_factory or subprocess.Popen
        self._stop_timeout_s = stop_timeout_s
        self._lock = threading.RLock()
        self._process = None
        self._reader = None
        self._generation = 0
        self._config = None
        self._state = "stopped"
        self._returncode = None
        self._auto_started = False
        self._log = deque(maxlen=_LOG_LINES)

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
            # 控制通道路径随 bench_manager 进程恒定；放进 config 让幂等比较单一表达
            "motor_sim_control_socket":
                f"/tmp/bench_motor_backend_{os.getpid()}.sock",
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

    def _status_locked(self):
        if self._process is not None:
            returncode = self._process.poll()
            if returncode is not None:
                self._returncode = returncode
                if self._state != "stopping":
                    self._state = "stopped" if returncode == 0 else "failed"
                    self._process = None
                    self._reader = None
                    self._auto_started = False
            elif self._state == "starting":
                # This reports process liveness, not bus readiness.
                self._state = "running"
        elif self._external_alive():
            # launch 内嵌的那份。命令可转发，起停不归本模块（进程主人是 launch）。
            return {
                "state": "external",
                "config": {"motor_sim_control_socket": self._external_socket},
                "returncode": self._returncode,
                "auto_started": False,
                "log": list(self._log),
            }
        return {
            "state": self._state,
            "config": dict(self._config) if self._config else None,
            "returncode": self._returncode,
            "auto_started": self._auto_started,
            "log": list(self._log),
        }

    def _external_alive(self) -> bool:
        """固定路径上是否活着一个**不是本模块启动的**模拟器。

        真连一次而不是只看文件：异常退出的残留 socket 文件会一直在，存在性不作数。
        """
        if not self._external_socket or not os.path.exists(self._external_socket):
            return False
        try:
            query_motor_sim(self._external_socket, {"cmd": "status"}, timeout_s=0.5)
        except RuntimeError:
            return False
        return True

    def status(self):
        with self._lock:
            return self._status_locked()

    def motor_request(self, request: dict) -> dict:
        """转发一条模拟器控制命令；两个来源都没有时抛 RuntimeError。"""
        with self._lock:
            self._status_locked()
            path = None
            if self._process is not None and self._config:
                path = self._config.get("motor_sim_control_socket")
            elif self._external_alive():
                path = self._external_socket
        if not path:
            raise RuntimeError("电机后端未运行，模拟器控制通道不可用")
        return query_motor_sim(path, request)

    def _read_output(self, process, generation):
        stream = process.stdout
        if stream is None:
            return
        try:
            for line in stream:
                with self._lock:
                    if generation == self._generation:
                        self._log.append(
                            line.rstrip("\r\n")[-_LOG_LINE_CHARS:]
                        )
        finally:
            stream.close()

    def start(self, request, auto=False):
        """启动或复用后端。auto 标记「随仿真拉起」，供停止联动判断。"""
        config = self._validate(request)
        with self._lock:
            status = self._status_locked()
            if status.get("state") == "external":
                raise RuntimeError(
                    "固定路径上已有 launch 内嵌的模拟器在跑（终端启动的 HIL），"
                    "不能再起一个——同一条总线上两个模拟器会互相打断。"
                    "页面可直接对它增删电机；要换端点请先停那次仿真 launch。")
            if self._process is not None:
                if self._config == config:
                    # 幂等复用；**不改** auto 归属（用户手动起的不能因停仿真被停）
                    return status
                raise RuntimeError(
                    "motor backend is already running with a different "
                    "configuration; stop it first")

            control_socket = config["motor_sim_control_socket"]
            command = [
                "ros2", "launch", "robot_sim", "simulation.launch.py",
                "simulator_only:=true",
                "controller_mode:=firmware",
                f"transport:={config['transport']}",
                f"model:={config['model']}",
                "start_web_host:=false",
                "start_micro_ros_agent:=false",
                f"sim_control_socket:={control_socket}",
            ]
            if config["transport"] == "socketcan":
                command.append(f"sim_interface:={config['sim_interface']}")
            else:
                command.append(f"sim_tty:={config['sim_tty']}")

            self._log.clear()
            self._generation += 1
            generation = self._generation
            self._returncode = None
            self._state = "starting"
            self._config = config
            self._auto_started = bool(auto)
            try:
                self._process = self._popen_factory(
                    command,
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1,
                    start_new_session=True,
                )
            except OSError:
                self._state = "failed"
                self._config = None
                self._auto_started = False
                raise
            self._reader = threading.Thread(
                target=self._read_output,
                args=(self._process, generation),
                name="motor-backend-output",
                daemon=True,
            )
            self._reader.start()
            return self._status_locked()

    def stop(self):
        with self._lock:
            self._status_locked()
            process = self._process
            reader = self._reader
            generation = self._generation
            if process is None:
                return self._status_locked()
            self._state = "stopping"

        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            returncode = process.wait(timeout=self._stop_timeout_s)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                returncode = process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                returncode = process.wait()

        if reader is not None:
            reader.join(timeout=1)

        with self._lock:
            if self._process is process and self._generation == generation:
                self._returncode = returncode
                # A non-zero code is expected when ros2 launch receives SIGINT.
                self._state = "stopped"
                self._process = None
                self._reader = None
            self._auto_started = False
            # 模拟器正常退出会自己删控制 socket；这里兜底清异常退出的残留
            socket_path = (self._config or {}).get("motor_sim_control_socket")
            if socket_path:
                try:
                    os.unlink(socket_path)
                except OSError:
                    pass
            return self._status_locked()
