"""Enumerate CAN devices and supervise one unified robot simulation launch.

`backend` 选择被控对象（gazebo | mujoco），只影响传给 simulation.launch.py 的参数与
「模型是否有 MJCF」这条校验；传输与控制端的校验对两个后端一致。
"""

from collections import deque
import json
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
    """模型根目录（安装树的 robot_sim share）；取不到返回 None。"""
    try:
        return Path(get_package_share_directory("robot_sim")) / "models"
    except Exception:
        return None


def enumerate_simulation_models():
    """List installed models that provide a full simulation topology."""
    models_root = _models_root()
    if models_root is None or not models_root.is_dir():
        return []
    return sorted(
        entry.name
        for entry in models_root.iterdir()
        if entry.is_dir()
        and (entry / "config" / "simulation_topology.yaml").is_file()
    )


def enumerate_mujoco_models():
    """List installed models that provide an MJCF for the MuJoCo backend.

    MuJoCo 后端不经过 ros2_control，被控对象是该模型自己的 `mujoco/<模型名>.xml`
    （由模型目录的 tools/build_mujoco_model.py 生成），缺它就只能跑 Gazebo。
    还要 `urdf/<模型名>.standalone.urdf`：mujoco.launch.py 用它渲染 robot_description，
    缺了虽然能起，但 rsp 拿到的是空模型——这种半残组合不该出现在下拉里。
    """
    models_root = _models_root()
    if models_root is None or not models_root.is_dir():
        return []
    return sorted(
        entry.name
        for entry in models_root.iterdir()
        if entry.is_dir()
        and (entry / "mujoco" / f"{entry.name}.xml").is_file()
        and (entry / "urdf" / f"{entry.name}.standalone.urdf").is_file()
    )


def simulation_model_joints(model):
    """读出某模型的**电机 id → 关节名**映射，供前端臂视图使用。

    前端靠这张表把 motor_id 落到关节上（关节角同步、状态选色、臂上小球点选）。
    它此前硬编码在 `web/js/core/store.js`，写死成 half_body_robot 的 18 台，
    于是**换模型就静默失效**：关节名对不上 → 那些关节角恒为 0、小球点不中，
    表现得像"电机没注册"。这里改为向模型配置取同一个真相源，加模型/改关节名
    不必再动前端。

    单一来源取 `config/motor_driver.yaml` 的 `motor_hardware.motors`：每个条目
    同时带 `joint:` 与 `id:`。不用 `simulation_topology.yaml`——那里只有电机名，
    还得再和本文件做一次连接。

    Raises:
        ValueError: 目录/模型缺失、非法模型名、解析不出条目、条目缺 id/joint、id 重复。
    """
    root = _models_root()
    if root is None or not root.is_dir():
        raise ValueError("找不到 robot_sim 的 models 目录（安装树不完整？）")
    # 防目录穿越：模型名只允许是单层目录名
    if not model or model != Path(model).name or model in (".", ".."):
        raise ValueError(f"非法模型名：{model!r}")
    config = root / model / "config" / "motor_driver.yaml"
    if not config.is_file():
        raise ValueError(f"模型 {model!r} 没有 config/motor_driver.yaml")

    with config.open("r", encoding="utf-8") as handle:
        doc = yaml.safe_load(handle) or {}
    motors = (doc.get("motor_hardware") or {}).get("motors")
    if not isinstance(motors, dict) or not motors:
        raise ValueError(f"{config} 里没有 motor_hardware.motors")

    joints = {}
    for name, spec in motors.items():
        if not isinstance(spec, dict):
            raise ValueError(f"电机 {name!r} 的条目不是映射")
        motor_id, joint = spec.get("id"), spec.get("joint")
        if motor_id is None or joint is None:
            raise ValueError(f"电机 {name!r} 缺 id 或 joint")
        motor_id = int(motor_id)
        if motor_id in joints:
            raise ValueError(
                f"电机 id {motor_id} 重复（{joints[motor_id]!r} 与 {joint!r}）："
                f"固件注册表以裸 motor_id 为键，重号会锁存 CONFIG_CONFLICT")
        joints[motor_id] = str(joint)
    return dict(sorted(joints.items()))


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

        config = {
            "backend": backend,
            "transport": transport,
            "controller_mode": mode,
            "model": model,
        }
        launch_args = [
            f"backend:={backend}",
            f"transport:={transport}",
            f"controller_mode:={mode}",
            f"model:={model}",
        ]

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
                        raise PermissionError(f"web_host cannot read and write {path}")

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
            # 模拟器（FDCAN 电机后端）由 web_host 的独立进程管理（可单独起停，
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
