"""枚举 CAN 设备并监督一个统一的 robot_sim simulation.launch.py 进程。

本模块归 bench_manager（web 前端经 /bench/* 服务间接使用）；`backend` 选择被控对象
（gazebo | mujoco），只影响传给 simulation.launch.py 的参数与「模型是否有 MJCF」这条
校验；传输与控制端的校验对两个后端一致。

只读模型目录的**文件域**函数（simulation_model_joints / enumerate_arm_models）不在
这里——它们服务臂视图与前端静态域，归 web_host/models.py。
"""

from collections import deque
import json
import math
import os
import re
import signal
import subprocess
import threading
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml

from .agent_manager import enumerate_serial_ports


_TRANSPORTS = {"vcan", "socketcan", "slcan"}
_CONTROLLER_MODES = {"host", "external", "firmware"}
_BACKENDS = {"gazebo", "mujoco"}
_INTERFACE_NAME = re.compile(r"[A-Za-z0-9_.-]{1,15}$")
_LOG_LINES = 80
_LOG_LINE_CHARS = 1000


def enumerate_can_interfaces():
    """Return physical CAN interfaces and their current link state."""
    try:
        result = subprocess.run(
            ["ip", "-details", "-json", "link", "show"],
            capture_output=True,
            text=True,
            timeout=3,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return []
    if result.returncode != 0:
        return []
    try:
        links = json.loads(result.stdout)
    except json.JSONDecodeError:
        return []

    interfaces = []
    for link in links:
        if (link.get("linkinfo") or {}).get("info_kind") != "can":
            continue
        name = link.get("ifname")
        if not isinstance(name, str):
            continue
        is_up = "UP" in (link.get("flags") or [])
        info = (link.get("linkinfo") or {}).get("info_data") or {}
        bitrate = info.get("bitrate")
        dbitrate = info.get("dbitrate")
        detail = "UP" if is_up else "DOWN"
        if bitrate:
            detail += f" · {bitrate} bit/s"
        if dbitrate:
            detail += f" · data {dbitrate} bit/s"
        interfaces.append({
            "name": name,
            "up": is_up,
            "label": f"{name} ({detail})",
        })
    return interfaces


def _models_root():
    """默认模型根目录（安装树的 robot_sim share）；取不到返回 None。

    各枚举函数的 `root=` 参数可覆盖它：bench_manager 节点的 models_dir 参数
    就是把这个根显式传下来（可指源码目录，改模型不必重构建）。
    """
    try:
        return Path(get_package_share_directory("robot_sim")) / "models"
    except Exception:
        return None


def enumerate_simulation_models(root=None):
    """List installed models that provide a full simulation topology."""
    models_root = Path(root) if root is not None else _models_root()
    if models_root is None or not models_root.is_dir():
        return []
    return sorted(
        entry.name
        for entry in models_root.iterdir()
        if entry.is_dir()
        and (entry / "config" / "simulation_topology.yaml").is_file()
    )


def enumerate_mujoco_models(root=None):
    """List installed models that provide an MJCF for the MuJoCo backend.

    MuJoCo 后端不经过 ros2_control，被控对象是该模型自己的 `mujoco/<模型名>.xml`
    （由模型目录的 tools/build_mujoco_model.py 生成），缺它就只能跑 Gazebo。
    还要 `urdf/<模型名>.standalone.urdf`：mujoco.launch.py 用它渲染 robot_description，
    缺了虽然能起，但 rsp 拿到的是空模型——这种半残组合不该出现在下拉里。
    """
    models_root = Path(root) if root is not None else _models_root()
    if models_root is None or not models_root.is_dir():
        return []
    return sorted(
        entry.name
        for entry in models_root.iterdir()
        if entry.is_dir()
        and (entry / "mujoco" / f"{entry.name}.xml").is_file()
        and (entry / "urdf" / f"{entry.name}.standalone.urdf").is_file()
    )


def _model_parameters(request, backend):
    """模型级仿真参数：重力 / 生成高度 / 固定基座。

    只透传给 Gazebo——MuJoCo 的世界参数在 MJCF 生成期固定（launch 侧同样拒绝），
    给了就显式报错而不是静默忽略。键**缺省** = 用 launch 默认值，不进命令行。
    """
    params = {}
    for key in ("gravity", "fixed_base"):
        if key in request:
            value = request[key]
            if not isinstance(value, bool):
                raise ValueError(f"{key} must be a boolean")
            params[key] = value
    if "spawn_z" in request:
        value = request["spawn_z"]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError("spawn_z must be a number (metres)")
        if not math.isfinite(value) or not -10.0 <= value <= 10.0:
            raise ValueError("spawn_z must be a finite number within [-10, 10]")
        params["spawn_z"] = float(value)
    if params and backend == "mujoco":
        raise ValueError(
            "mujoco fixes gravity, spawn height and base welding in the MJCF at "
            "build time; model parameters apply to the gazebo backend only"
        )
    return params


class SimulationProcessManager:
    """Own a single ROS 2 simulation launch process and its recent output."""

    def __init__(self, can_provider=None, serial_provider=None,
                 model_provider=None, mujoco_model_provider=None,
                 popen_factory=None, stop_timeout_s=12):
        self._can_provider = can_provider or enumerate_can_interfaces
        self._serial_provider = serial_provider or enumerate_serial_ports
        self._model_provider = model_provider or enumerate_simulation_models
        self._mujoco_model_provider = (
            mujoco_model_provider or enumerate_mujoco_models
        )
        self._popen_factory = popen_factory or subprocess.Popen
        self._stop_timeout_s = stop_timeout_s
        self._lock = threading.RLock()
        self._process = None
        self._reader = None
        self._generation = 0
        self._config = None
        self._state = "stopped"
        self._returncode = None
        self._log = deque(maxlen=_LOG_LINES)

    def options(self):
        serial_ports = []
        for port in self._serial_provider():
            serial_ports.append({
                **port,
                "accessible": os.access(port["path"], os.R_OK | os.W_OK),
            })
        return {
            "can_interfaces": self._can_provider(),
            "serial_ports": serial_ports,
            "models": self._model_provider(),
            # 后端能力：MuJoCo 需要模型自带 MJCF，前端据此过滤模型下拉
            "mujoco_models": self._mujoco_model_provider(),
        }

    def _validate_and_build(self, request):
        if not isinstance(request, dict):
            raise ValueError("request must be a JSON object")
        transport = request.get("transport")
        mode = request.get("controller_mode")
        model = request.get("model")
        backend = request.get("backend")
        if not isinstance(transport, str) or transport not in _TRANSPORTS:
            raise ValueError("transport must be vcan, socketcan or slcan")
        if not isinstance(mode, str) or mode not in _CONTROLLER_MODES:
            raise ValueError("controller_mode must be host, external or firmware")
        if not isinstance(backend, str) or backend not in _BACKENDS:
            raise ValueError("backend must be gazebo or mujoco")
        if mode == "firmware" and transport == "vcan":
            raise ValueError("vcan is software-only and cannot connect to an MCU")
        if not isinstance(model, str) or model not in self._model_provider():
            raise ValueError("model has no installed full-simulation topology")
        if backend == "mujoco" and model not in self._mujoco_model_provider():
            raise ValueError(
                f"model '{model}' has no MuJoCo model "
                f"(models/{model}/mujoco/{model}.xml); use backend gazebo"
            )

        model_params = _model_parameters(request, backend)

        config = {
            "backend": backend,
            "transport": transport,
            "controller_mode": mode,
            "model": model,
        }
        config.update(model_params)
        launch_args = [
            f"backend:={backend}",
            f"transport:={transport}",
            f"controller_mode:={mode}",
            f"model:={model}",
        ]
        for key, value in model_params.items():
            if isinstance(value, bool):
                launch_args.append(f"{key}:={'true' if value else 'false'}")
            else:
                launch_args.append(f"{key}:={value}")

        if transport == "vcan":
            interface = request.get("vcan_interface", "robot_vcan0")
            if not isinstance(interface, str) or not _INTERFACE_NAME.fullmatch(interface):
                raise ValueError("vcan interface name must be 1-15 safe characters")
            config["vcan_interface"] = interface
            launch_args.append(f"vcan_interface:={interface}")
        else:
            if request.get("physical_bus_isolated") is not True:
                raise ValueError(
                    "confirm that the physical CAN bus has no real motor load"
                )

            if transport == "socketcan":
                available = {
                    item["name"]: item for item in self._can_provider()
                    if isinstance(item, dict) and isinstance(item.get("name"), str)
                }

                def require_can(name, field):
                    if not isinstance(name, str) or name not in available:
                        raise ValueError(f"select an enumerated CAN interface for {field}")
                    if not available[name].get("up"):
                        raise ValueError(f"SocketCAN interface {name} is down")

                sim_interface = request.get("sim_interface")
                require_can(sim_interface, "simulation endpoint")
                config["sim_interface"] = sim_interface
                launch_args.append(f"sim_interface:={sim_interface}")
                if mode == "host":
                    driver_interface = request.get("driver_interface")
                    require_can(driver_interface, "driver endpoint")
                    if driver_interface == sim_interface:
                        raise ValueError(
                            "SocketCAN host mode needs separate driver and "
                            "simulation interfaces"
                        )
                    config["driver_interface"] = driver_interface
                    launch_args.append(f"driver_interface:={driver_interface}")
            else:
                serial_ports = {
                    item["path"]: item for item in self._serial_provider()
                }

                def require_tty(path, field):
                    if not isinstance(path, str) or path not in serial_ports:
                        raise ValueError(f"select an enumerated serial device for {field}")
                    if not os.access(path, os.R_OK | os.W_OK):
                        raise PermissionError(f"bench_manager cannot read and write {path}")

                sim_tty = request.get("sim_tty")
                require_tty(sim_tty, "simulation endpoint")
                config["sim_tty"] = sim_tty
                launch_args.append(f"sim_tty:={sim_tty}")
                if mode == "host":
                    driver_tty = request.get("driver_tty")
                    require_tty(driver_tty, "driver endpoint")
                    if os.path.realpath(driver_tty) == os.path.realpath(sim_tty):
                        raise ValueError(
                            "SLCAN host mode needs two separate serial adapters"
                        )
                    config["driver_tty"] = driver_tty
                    launch_args.append(f"driver_tty:={driver_tty}")

        if mode == "firmware":
            # 模拟器（FDCAN 电机后端）由 bench_manager 的独立进程管理（可单独起停，
            # 见 motor_sim_manager.py）：本 launch 只编排被控对象/驱动端/Agent。
            # WebHost already manages its own Agent and HTTP/WebSocket listeners.
            launch_args.extend([
                "start_web_host:=false",
                "start_micro_ros_agent:=false",
                "start_fdcan_sim:=false",
            ])
        return config, launch_args

    def _status_locked(self):
        if self._process is not None:
            returncode = self._process.poll()
            if returncode is not None:
                self._returncode = returncode
                if self._state != "stopping":
                    self._state = "stopped" if returncode == 0 else "failed"
                    self._process = None
                    self._reader = None
            elif self._state == "starting":
                # This reports process liveness, not Gazebo/controller readiness.
                self._state = "running"
        return {
            "state": self._state,
            "config": dict(self._config) if self._config else None,
            "returncode": self._returncode,
            "log": list(self._log),
        }

    def status(self):
        with self._lock:
            return self._status_locked()

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

    def start(self, request):
        config, launch_args = self._validate_and_build(request)
        with self._lock:
            status = self._status_locked()
            if self._process is not None:
                if self._config == config:
                    return status
                raise RuntimeError("a simulation launch is already running")

            command = [
                "ros2", "launch", "robot_sim", "simulation.launch.py",
                *launch_args,
            ]
            self._log.clear()
            self._generation += 1
            generation = self._generation
            self._returncode = None
            self._state = "starting"
            self._config = config
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
                raise
            self._reader = threading.Thread(
                target=self._read_output,
                args=(self._process, generation),
                name="simulation-launch-output",
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
            return self._status_locked()
