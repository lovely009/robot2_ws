"""枚举 CAN 设备并监督 `bench-simulation.service`（robot_sim simulation.launch.py）。

进程归属在 systemd（user transient unit），不在本模块：状态由 unit 状态派生、停止靠
systemd 默认的 `KillMode=control-group`。本模块校验请求并生成 unit ExecStart 命令行。

本模块归 bench_manager（web 前端经 /bench/* 服务间接使用）；`backend` 选择被控对象
（gazebo | mujoco），只影响传给 simulation.launch.py 的参数与「模型是否有 MJCF」这条
校验；传输与控制端的校验对两个后端一致。没有仿真拓扑的 MuJoCo 模型仅以
external 模式直接启动 mujoco.launch.py，不经过 CAN 传输或控制端。

只读模型目录的**文件域**函数（simulation_model_joints / enumerate_arm_models）不在
这里——它们服务臂视图与前端静态域，归 web_host/models.py。
"""

import json
import math
import os
import re
import subprocess

from .agent_manager import enumerate_serial_ports
from .model_catalog import enumerate_mujoco_models, enumerate_simulation_models
from .systemd_units import (
    STATE_NAMES, UNIT_SIMULATION, SystemdUnits, UnitState, config_env,
    config_from_env,
)


_TRANSPORTS = {"vcan", "socketcan", "slcan"}
_CONTROLLER_MODES = {"external", "firmware"}
_BACKENDS = {"gazebo", "mujoco"}
_INTERFACE_NAME = re.compile(r"[A-Za-z0-9_.-]{1,15}$")

def _as_bool(text: str) -> bool:
    return text == "true"


# 会话描述在 unit 环境里的前缀；需要还原类型的键只有这三个——bool 由 config_env 统一
# 编码成 true/false，`spawn_z` 是唯一的浮点。
_CONFIG_ENV_PREFIX = "BENCH_SIM_"
_CONFIG_DECODERS = {"spawn_z": float, "gravity": _as_bool, "fixed_base": _as_bool}


def enumerate_can_interfaces():
    """返回物理 CAN 接口及当前链路状态。"""
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
    """管理单个 ROS 2 仿真会话 unit。"""

    def __init__(self, can_provider=None, serial_provider=None,
                 model_provider=None, mujoco_model_provider=None,
                 systemd=None):
        self._can_provider = can_provider or enumerate_can_interfaces
        self._serial_provider = serial_provider or enumerate_serial_ports
        self._model_provider = model_provider or enumerate_simulation_models
        self._mujoco_model_provider = (
            mujoco_model_provider or enumerate_mujoco_models
        )
        self._systemd = systemd or SystemdUnits()

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
        if mode == "host":
            raise ValueError(
                "controller_mode=host is retired; use firmware for MCU control "
                "or external for a simulator without a host motor driver"
            )
        if not isinstance(mode, str) or mode not in _CONTROLLER_MODES:
            raise ValueError("controller_mode must be external or firmware")
        if not isinstance(backend, str) or backend not in _BACKENDS:
            raise ValueError("backend must be gazebo or mujoco")
        if mode == "firmware" and transport == "vcan":
            raise ValueError("vcan is software-only and cannot connect to an MCU")

        standalone_mujoco = (
            backend == "mujoco"
            and mode == "external"
            and isinstance(model, str)
            and model not in self._model_provider()
            and model in self._mujoco_model_provider()
        )
        if standalone_mujoco:
            config = {
                "backend": backend,
                "transport": transport,
                "controller_mode": mode,
                "model": model,
            }
            _model_parameters(request, backend)
            return config, [f"model:={model}"], "mujoco.launch.py"

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

        if mode == "firmware":
            # 模拟器（FDCAN 电机后端）由 bench_manager 的独立进程管理（可单独起停，
            # 见 motor_sim_manager.py）；主机控制由反馈管理器等待有效 MCU 子集后启动。
            launch_args.extend([
                "start_web_host:=false",
                "start_micro_ros_agent:=false",
                "start_fdcan_sim:=false",
                "start_host_control:=false",
            ])
        return config, launch_args, "simulation.launch.py"

    def _status(self):
        """从 unit 派生状态，并恢复其环境中的会话配置。
        """
        unit = self._systemd.probe(UNIT_SIMULATION)
        env = ({} if unit.state is UnitState.ABSENT
               else self._systemd.environment(UNIT_SIMULATION))
        return {
            "state": STATE_NAMES[unit.state],
            "config": config_from_env(_CONFIG_ENV_PREFIX, env, _CONFIG_DECODERS),
            "returncode": unit.returncode,
        }

    def status(self):
        return self._status()

    def start(self, request):
        """按当前请求起 `bench-simulation.service`；已在跑则原样认领，不重启。

        重启会打断正在跑的仿真，所以 RUNNING 且配置相同直接返回。配置不同则拒绝：
        换后端/模型/端点不是「重启一下」，那是另一次会话，该先显式停掉。
        """
        config, launch_args, launch_file = self._validate_and_build(request)
        status = self._status()
        if status["state"] == "running":
            if status["config"] == config:
                return status
            raise RuntimeError(
                "a simulation launch is already running with a different "
                "configuration; stop it first")
        self._systemd.start(
            UNIT_SIMULATION,
            ["ros2", "launch", "robot_sim", launch_file,
             *launch_args],
            session_env=config_env(_CONFIG_ENV_PREFIX, config))
        return self._status()

    def stop(self):
        self._systemd.stop(UNIT_SIMULATION)
        return self._status()
