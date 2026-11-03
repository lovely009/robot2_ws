"""启动机械臂和 CAN 电机模拟器，接收外部控制器或 MCU 固件命令。

软件闭环（默认）：
  external CAN controller -> CAN transport
  -> motor_sim -> 被控对象 -> /joint_states -> motor_sim

「被控对象」由 backend:= 选：gazebo（gz sim + gz_ros2_control 的 effort 控制器）或
mujoco（自写力矩直通桥 + MuJoCo 物理）。两者对外契约相同——吃
/arm_effort_controller/commands、出 /joint_states——所以传输、控制端、被控电机模型
（motor_sim）、effort 门控、驱动端接线与运行期配置生成完全共用，只有被 include 的
launch 文件、以及命令数组的顺序来源（Gazebo 重写 controllers.yaml 的 joints；MuJoCo
把拓扑顺序的关节名交给桥按名字映射）不同。

backend:=mujoco 要求模型目录自带 mujoco/<模型名>.xml，且不支持 gravity:=false
（MJCF 的重力在生成期固定）。

transport:=vcan 默认自动创建私有虚拟总线；socketcan 与 slcan 用于接入外部
CAN 设备。controller_mode:=firmware 使用 FDCAN 台架模拟器，支持
单个 SocketCAN 接口或单 CANable + STM32 固件的 HIL 路径。
"""

from __future__ import annotations

import json
import math
import os
import re
import shutil
import socket
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import yaml
from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


# CERB τ_ff 的 12 位量化满量程厂家值（R58/R48 同值）。型号库/拓扑缺字段时的
# 缺省；不能回落 peak_torque_nm（45/11，物理峰值），否则前馈被整体缩放。
DEFAULT_CERB_MIT_TORQUE_LIMIT_NM = 300.0


def _load_simulation(path: Path) -> dict[str, Any]:
    document = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    simulation = document.get("simulation")
    if not isinstance(simulation, dict):
        raise RuntimeError(f"{path} requires a root 'simulation' map")
    motors = simulation.get("motors")
    profiles = simulation.get("profiles")
    if not isinstance(motors, list) or not motors:
        raise RuntimeError("simulation topology must contain at least one motor")
    if not isinstance(profiles, dict) or not profiles:
        raise RuntimeError("simulation topology must contain motor profiles")

    names: set[str] = set()
    joints: set[str] = set()
    for motor in motors:
        if not isinstance(motor, dict):
            raise RuntimeError("each simulation motor must be a map")
        name = motor.get("motor")
        joint = motor.get("joint")
        if not isinstance(name, str) or not name or not isinstance(joint, str) or not joint:
            raise RuntimeError(f"invalid simulation motor entry: {motor}")
        if name in names or joint in joints:
            raise RuntimeError(f"duplicate simulation motor mapping: {motor}")
        profile_name = motor.get("profile")
        if not isinstance(profile_name, str) or not profile_name:
            raise RuntimeError(f"motor '{name}' requires a simulation profile")
        profile = profiles.get(profile_name)
        if not isinstance(profile, dict) or not isinstance(profile.get("plant"), dict):
            raise RuntimeError(f"motor '{name}' references an invalid profile")
        names.add(name)
        joints.add(joint)

    return simulation


def _resolve_motor_specs_path(motor_driver_path: Path) -> Path:
    document = yaml.safe_load(motor_driver_path.read_text(encoding="utf-8")) or {}
    hardware = document.get("motor_hardware", {})
    specs_value = hardware.get("motor_specs_file")
    if not isinstance(specs_value, str) or not specs_value:
        raise RuntimeError(
            f"{motor_driver_path} must define motor_hardware.motor_specs_file"
        )
    specs_path = Path(specs_value)
    if not specs_path.is_absolute():
        specs_path = motor_driver_path.parent / specs_path
    return specs_path.resolve()


def _bind_motor_topology(
    simulation: dict[str, Any],
    motor_driver_path: Path,
    motor_specs_path: Path,
) -> dict[str, Any]:
    driver_document = yaml.safe_load(motor_driver_path.read_text(encoding="utf-8")) or {}
    hardware = driver_document.get("motor_hardware", {})
    source_motors = hardware.get("motors", {})
    source_buses = hardware.get("buses", {})
    specs_document = yaml.safe_load(motor_specs_path.read_text(encoding="utf-8")) or {}
    motor_types = specs_document.get("motor_types", {})
    if not isinstance(source_motors, dict) or not isinstance(source_buses, dict):
        raise RuntimeError(f"invalid motor topology in {motor_driver_path}")
    if not isinstance(motor_types, dict):
        raise RuntimeError(f"invalid motor types in {motor_specs_path}")

    selected_motors = []
    sim_profiles: dict[str, Any] = {}
    profile_motor_types: dict[str, str] = {}
    buses: list[str] = []
    addresses: set[tuple[str, str, int]] = set()
    for mapping in simulation["motors"]:
        name = mapping["motor"]
        source = source_motors.get(name)
        if not isinstance(source, dict):
            raise RuntimeError(
                f"simulation motor '{name}' is missing from {motor_driver_path}"
            )
        bus = str(source.get("bus", ""))
        motor_type = str(source.get("type", ""))
        motor_id = int(source.get("id", 0))
        type_spec = motor_types.get(motor_type)
        if bus not in source_buses or not isinstance(type_spec, dict):
            raise RuntimeError(f"motor '{name}' has an invalid bus or motor type")
        protocol = str(type_spec.get("protocol", motor_type))
        if protocol not in {"cerb", "vaxor"} or motor_id <= 0:
            raise RuntimeError(f"motor '{name}' has an unsupported protocol or ID")
        command = source.get("command")
        if not isinstance(command, dict) or not {"kp", "kd"}.issubset(command):
            raise RuntimeError(f"motor '{name}' requires command.kp and command.kd")
        address = (bus, protocol, motor_id)
        if address in addresses:
            raise RuntimeError(f"duplicate motor bus/protocol/ID mapping: {address}")
        addresses.add(address)
        if bus not in buses:
            buses.append(bus)

        profile_name = mapping["profile"]
        previous_type = profile_motor_types.setdefault(profile_name, motor_type)
        if previous_type != motor_type:
            raise RuntimeError(
                f"simulation profile '{profile_name}' cannot mix motor types "
                f"'{previous_type}' and '{motor_type}'"
            )
        type_parameters = dict(type_spec.get("parameters") or {})
        type_parameters.update(source.get("parameters") or {})
        # 关节侧峰值力矩：HIL（firmware）那条路上它**只**由模拟器的力矩饱和值
        # 体现——MJCF 的 <motor> 不带 forcerange、MuJoCo 桥直接写 data.ctrl 不夹，
        # 所以这里是唯一的执行器权限来源。取型号库的 peak_torque_nm（真机峰值，
        # r58=45 / r48=11）；没写该字段的型号（vaxor）留 None，模拟器退回全局值。
        peak_torque = type_parameters.get("peak_torque_nm")
        try:
            peak_torque = float(peak_torque)
        except (TypeError, ValueError):
            peak_torque = None
        if peak_torque is not None and peak_torque <= 0.0:
            peak_torque = None
        source_profile = simulation["profiles"][profile_name]
        # 逐台覆盖不能写回共享型号 profile，否则后续同型号电机会覆盖它。
        if source.get("parameters"):
            profile_name = f"{profile_name}__{name}"
        sim_profile = dict(source_profile)
        plant = dict(source_profile.get("plant", {}))
        plant["torque_constant_nm_per_a"] = type_parameters.get(
            "torque_constant_nm_per_amp", 0.0
        )
        plant["current_limit_a"] = plant.get(
            "current_limit_a",
            type_parameters.get(
                "peak_current_amp", type_parameters.get("rated_current_amp", 0.0)
            ),
        )
        if (
            float(plant["torque_constant_nm_per_a"]) <= 0.0
            or float(plant["current_limit_a"]) <= 0.0
        ):
            raise RuntimeError(
                f"simulation profile '{profile_name}' needs positive torque "
                "constant and current limit from its motor type or plant settings"
            )
        sim_profile["gear_ratio"] = type_parameters.get("gear_ratio", 1.0)
        sim_profile["efficiency"] = type_parameters.get("gear_efficiency", 1.0)
        sim_profile["plant"] = plant
        # 线协议量程与物理保护分离；显式坏值在生成配置前拒绝。
        mit_limits = {}
        if protocol == "cerb":
            for field, default in {
                "mit_position_limit_rad": 3.14,
                "mit_velocity_limit_rad_s": 3.14,
                "mit_torque_limit_nm": DEFAULT_CERB_MIT_TORQUE_LIMIT_NM,
                "mit_kp_limit": 2500.0,
                "mit_kd_limit": 500.0,
            }.items():
                raw = type_parameters.get(field, default)
                try:
                    value = float(raw)
                except (TypeError, ValueError):
                    value = float("nan")
                if not math.isfinite(value) or value <= 0:
                    raise ValueError(
                        f"motor '{name}' parameter {field} must be "
                        f"a positive finite number, got {raw!r}"
                    )
                mit_limits[field] = value
            sim_profile.update(mit_limits)
        mit_torque = mit_limits.get("mit_torque_limit_nm")
        # SDO 从站的对端契约：固件靠 0x1008:00 身份串绑型号、靠 0x6080:00 /
        # 0x2538:00 回读核验，三者必须同源。身份串没有合理的缺省，型号库缺字段
        # 就报错——否则节点会起一个固件永远绑不上的从站，而构建期毫无提示。
        if protocol == "cerb":
            for field in (
                "device_identity",
                "device_max_speed_rpm",
                "device_max_current_ma",
            ):
                value = type_parameters.get(field)
                if value is None:
                    raise RuntimeError(
                        f"motor type '{motor_type}' needs {field} for the "
                        "CERB SDO slave"
                    )
                sim_profile[field] = value
        sim_profiles[profile_name] = sim_profile

        selected_motors.append(
            {
                "name": name,
                "joint": mapping["joint"],
                "bus": bus,
                "protocol": protocol,
                "id": motor_id,
                "type": motor_type,
                "command": command,
                "driver_parameters": source.get("parameters", {}),
                "peak_torque_nm": peak_torque,
                # τ_ff 的 12 位量化满量程（vaxor 无此项），与 peak_torque_nm
                # （物理饱和）是两回事，分别进模拟器的两条 CLI。
                "mit_torque_limit_nm": mit_torque,
                **mit_limits,
                "profile": profile_name,
                # 反馈换算的两个系数：关节侧 N·m/A 与速比。与 C++ motor_sim 的
                # transmission + Kt 同式（joint_effort_to_motor_torque 后除 Kt），
                # 模拟器用它把「实际施加的关节力矩」还原成上报电流/转速。
                "joint_nm_per_amp": (
                    float(plant["torque_constant_nm_per_a"])
                    * float(sim_profile["gear_ratio"])
                    * float(sim_profile["efficiency"])
                ),
                "gear_ratio": float(sim_profile["gear_ratio"]),
            }
        )

    simulation["buses"] = buses
    if len(buses) != 1:
        raise RuntimeError(
            "the complete robot arm launcher currently supports one CAN bus; "
            "map every simulated motor to one bus in simulation_topology.yaml"
        )
    simulation["motors"] = selected_motors
    simulation["profiles"] = sim_profiles
    hardware_timing = hardware.get("timing", {})
    # 仿真驱动时序默认继承权威硬件配置；模型可在 simulation_topology.yaml 的
    # driver_timing 覆盖（例如仿真侧 CERB 只在收到命令帧后回帧，激活窗口需要
    # 比真机更宽）。只允许覆盖已知键，拼错即报错而不是静默忽略。
    driver_timing = {
        "feedback_timeout_s": hardware_timing.get("feedback_timeout_s", 0.10),
        "initial_feedback_wait_s": hardware_timing.get(
            "initial_feedback_wait_s", 1.0
        ),
    }
    override = simulation.get("driver_timing", {})
    if not isinstance(override, dict):
        raise RuntimeError("driver_timing in simulation_topology.yaml must be a mapping")
    unknown = set(override) - set(driver_timing)
    if unknown:
        raise RuntimeError(
            "unknown driver_timing keys in simulation_topology.yaml: "
            + ", ".join(sorted(str(key) for key in unknown))
        )
    driver_timing.update(override)
    simulation["driver_timing"] = driver_timing
    return simulation


def _model_joint_names(share: Path, model: str) -> set[str]:
    path = share / "models" / model / "config" / "controllers.yaml"
    if not path.is_file():
        raise RuntimeError(f"model controller config does not exist: {path}")
    document = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    for key, section in document.items():
        if str(key).endswith("/arm_effort_controller"):
            return set(section.get("ros__parameters", {}).get("joints", []))
    raise RuntimeError(f"{path} has no arm_effort_controller joint list")


def _mujoco_joint_names(path: Path) -> set[str]:
    """MJCF 里声明过的关节名（只取名字，顺序不参与校验）。

    这里刻意不 import mujoco：launch 跑在系统 python3，那里没有
    mujoco_ros2_control 的 venv；直接解析 XML 足够做「拓扑关节都在模型里」的校验。
    """
    try:
        root = ET.parse(path).getroot()
    except ET.ParseError as error:
        raise RuntimeError(f"cannot parse MJCF: {path}") from error
    return {joint.get("name") for joint in root.iter("joint") if joint.get("name")}


def _write_gazebo_controller_config(
    share: Path,
    model: str,
    joints: list[str],
    output_dir: Path,
) -> Path:
    source = share / "models" / model / "config" / "controllers.yaml"
    document = yaml.safe_load(source.read_text(encoding="utf-8")) or {}
    required_controllers = {"arm_position_controller", "arm_effort_controller"}
    found: set[str] = set()
    for name, section in document.items():
        controller = str(name).rsplit("/", 1)[-1]
        if controller not in required_controllers:
            continue
        parameters = section.setdefault("ros__parameters", {})
        parameters["joints"] = list(joints)
        found.add(controller)
    if found != required_controllers:
        missing = ", ".join(sorted(required_controllers - found))
        raise RuntimeError(f"{source} is missing controller configs: {missing}")

    output = output_dir / "gazebo_controllers.yaml"
    output.write_text(
        yaml.safe_dump(document, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    return output


def _absolute_transmissions_file(driver_path: Path) -> Path | None:
    if driver_path.is_file():
        document = yaml.safe_load(driver_path.read_text(encoding="utf-8")) or {}
        value = document.get("motor_hardware", {}).get("transmissions_file")
        if value:
            path = Path(str(value))
            if not path.is_absolute():
                path = driver_path.parent / path
            return path.resolve()
    sibling = driver_path.parent / "transmissions.yaml"
    return sibling.resolve() if sibling.is_file() else None


def _write_runtime_configs(
    simulation: dict[str, Any],
    output_dir: Path,
    transport: str,
    driver_endpoint: str,
    sim_endpoint: str,
    motor_specs_path: Path,
    motor_driver_path: Path,
) -> tuple[Path, Path]:
    timing = simulation.get("timing", {})
    driver_timing = simulation["driver_timing"]
    motors = simulation["motors"]
    profiles = simulation["profiles"]
    sim_motors = []
    driver_motors: dict[str, Any] = {}
    for motor in motors:
        sim_motors.append(
            {
                "joint": motor["joint"],
                "bus": motor["bus"],
                "protocol": motor["protocol"],
                "id": motor["id"],
                "profile": motor["profile"],
                # 保护开关 0x2650:00 与方向 0x2541:00 的真机上电初值。
                # 拓扑未提供覆盖入口，固定使用真机上电值。
                "sdo_overrides": {
                    "protection_enabled": 1,
                    "direction": 0,
                },
            }
        )
        driver_motors[motor["name"]] = {
            "joint": motor["joint"],
            "bus": motor["bus"],
            "type": motor["type"],
            "id": motor["id"],
            "command": motor["command"],
            "parameters": motor.get("driver_parameters", {}),
        }

    motor_sim_config = output_dir / "motor_sim.yaml"
    motor_sim_document = {
        "motor_simulation": {
            # 节拍缺省与 config.hpp 一致：0 表示跟随 control_hz 或不启用错峰。
            "timing": {
                "control_hz": timing.get("control_hz", 100),
                "vaxor_feedback_hz": timing.get("vaxor_feedback_hz", 50),
                "command_timeout_ms": timing.get("command_timeout_ms", 150),
                "can_period_ms": timing.get("can_period_ms", 0),
                "feedback_period_ms": timing.get("feedback_period_ms", 0),
                "stagger_ms": timing.get("stagger_ms", 0),
                "effort_rate_hz": timing.get("effort_rate_hz", 0),
            },
            # buses[] 是每条总线端点与传输的唯一宿主：节点不再收
            # transport_mode / <bus>_port / baud_rate / allow_external_canfd。
            "buses": [
                {
                    "name": simulation["buses"][0],
                    "transport": (
                        "serial_slcan" if transport == "slcan"
                        else "socket_can" if transport in {"socketcan", "vcan"}
                        else "usb_serial"
                    ),
                    "endpoint": sim_endpoint,
                    "baud_rate": 115200 if transport == "slcan" else 3000000,
                    "allow_external_canfd": transport == "slcan",
                }
            ],
            "profiles": profiles,
            "motors": sim_motors,
        }
    }
    motor_sim_config.write_text(
        yaml.safe_dump(motor_sim_document, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )

    driver_transport = "serial_slcan" if transport == "slcan" else "socketcan"
    bus_name = simulation["buses"][0]
    driver_bus = {"transport": driver_transport}
    if driver_transport == "socketcan":
        driver_bus["interface"] = driver_endpoint
    else:
        driver_bus["tty"] = driver_endpoint
    source_driver = yaml.safe_load(motor_driver_path.read_text(encoding="utf-8")) or {}
    transmissions_file = _absolute_transmissions_file(motor_driver_path)
    driver_config = output_dir / "motor_driver.yaml"
    driver_document = {
        "motor_hardware": {
            "motor_specs_file": str(motor_specs_path),
            **({"transmissions_file": str(transmissions_file)}
               if transmissions_file is not None else {}),
            "timing": driver_timing,
            "buses": {bus_name: driver_bus},
            "motors": driver_motors,
        }
    }
    driver_config.write_text(
        yaml.safe_dump(driver_document, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    return motor_sim_config, driver_config


def _ensure_vcan(name: str) -> tuple[bool, bool]:
    if not re.fullmatch(r"[A-Za-z0-9_.-]{1,15}", name):
        raise RuntimeError(f"invalid Linux CAN interface name: {name!r}")
    exists = subprocess.run(
        ["ip", "link", "show", "dev", name], capture_output=True, text=True
    ).returncode == 0
    created = False
    was_up = False
    if not exists:
        result = subprocess.run(
            ["ip", "link", "add", "dev", name, "type", "vcan"],
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            raise RuntimeError(
                f"cannot create {name}; check CAP_NET_ADMIN and the vcan kernel "
                f"module. Run 'sudo ip link add dev {name} type vcan' once, or "
                f"grant CAP_NET_ADMIN to the launch environment. {result.stderr.strip()}"
            )
        created = True
    else:
        details = subprocess.run(
            ["ip", "-details", "-json", "link", "show", "dev", name],
            capture_output=True,
            text=True,
        )
        try:
            link = json.loads(details.stdout)[0]
        except (IndexError, json.JSONDecodeError) as error:
            raise RuntimeError(
                f"cannot identify existing interface {name} as vcan"
            ) from error
        kind = (link.get("linkinfo") or {}).get("info_kind")
        if kind != "vcan":
            raise RuntimeError(
                f"interface '{name}' already exists as '{kind}', expected vcan"
            )
        was_up = "UP" in (link.get("flags") or [])
    if not was_up:
        result = subprocess.run(
            ["ip", "link", "set", "dev", name, "up"],
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            if created:
                subprocess.run(
                    ["ip", "link", "delete", "dev", name],
                    capture_output=True,
                    text=True,
                )
            raise RuntimeError(
                f"cannot bring {name} up; check CAP_NET_ADMIN. Run "
                f"'sudo ip link set dev {name} up' once, or grant CAP_NET_ADMIN "
                f"to the launch environment. {result.stderr.strip()}"
            )
    return created, was_up


def _validate_socketcan_interface(name: str) -> str:
    if not re.fullmatch(r"[A-Za-z0-9_.-]{1,15}", name):
        raise RuntimeError(f"invalid SocketCAN interface name: {name!r}")
    try:
        socket.if_nametoindex(name)
    except OSError as error:
        raise RuntimeError(
            f"SocketCAN interface '{name}' does not exist; configure it before launch"
        ) from error
    details = subprocess.run(
        ["ip", "-details", "-json", "link", "show", "dev", name],
        capture_output=True,
        text=True,
    )
    try:
        link = json.loads(details.stdout)[0]
    except (IndexError, json.JSONDecodeError) as error:
        raise RuntimeError(f"cannot inspect SocketCAN interface '{name}'") from error
    kind = (link.get("linkinfo") or {}).get("info_kind")
    if kind not in {"can", "vcan"}:
        raise RuntimeError(f"interface '{name}' is '{kind}', expected CAN or vcan")
    if "UP" not in (link.get("flags") or []):
        raise RuntimeError(f"SocketCAN interface '{name}' is down; bring it up first")
    return kind


def _required(context, key: str) -> str:
    value = LaunchConfiguration(key).perform(context).strip()
    if not value:
        raise RuntimeError(f"simulation.launch.py requires {key}:=<value>")
    return value


def _tau_limit_cli(simulation: dict[str, Any]) -> str:
    """物理饱和（peak_torque_nm）→ --tau-limits，只夹出力，不参与 MIT 解码。"""
    return ",".join(
        f"{motor['id']}:{motor['peak_torque_nm']:g}"
        for motor in simulation["motors"]
        if motor.get("peak_torque_nm")
    )


def _mit_limit_cli(simulation: dict[str, Any], field: str) -> str:
    return ",".join(
        f"{motor['id']}:{motor[field]:g}"
        for motor in simulation["motors"]
        if motor.get(field) is not None
    )


def _mit_torque_limit_cli(simulation: dict[str, Any]) -> str:
    return _mit_limit_cli(simulation, "mit_torque_limit_nm")


def launch_simulation(context):
    controller_mode = _required(context, "controller_mode")
    if controller_mode == "host":
        raise RuntimeError("controller_mode:=host has been retired; use firmware or external")
    transport = _required(context, "transport")
    backend = LaunchConfiguration("backend").perform(context).strip()
    simulator_only = _bool(
        LaunchConfiguration("simulator_only").perform(context))
    start_fdcan_sim = _bool(
        LaunchConfiguration("start_fdcan_sim").perform(context))
    if transport not in {"vcan", "socketcan", "slcan"}:
        raise RuntimeError("transport must be vcan, socketcan or slcan")
    if controller_mode not in {"external", "firmware"}:
        raise RuntimeError("controller_mode must be external or firmware")
    if backend not in {"gazebo", "mujoco"}:
        raise RuntimeError("backend must be gazebo or mujoco")
    try:
        spawn_z = float(LaunchConfiguration("spawn_z").perform(context))
    except ValueError:
        raise RuntimeError("spawn_z must be a number (metres)")
    if simulator_only and controller_mode != "firmware":
        raise RuntimeError(
            "simulator_only:=true 是 FDCAN 模拟器单起路径（web 上位机的"
            "「电机后端」），要求 controller_mode:=firmware"
        )
    if controller_mode == "firmware" and transport not in {"socketcan", "slcan"}:
        raise RuntimeError(
            "controller_mode:=firmware requires transport:=socketcan or slcan"
        )
    if backend == "mujoco" and not _bool(
        LaunchConfiguration("gravity").perform(context)
    ):
        # Gazebo 的 gravity 走 xacro 参数；MJCF 的重力在生成期就固定了，运行期没有开关。
        # 静默忽略会让人以为"已关重力"，所以直接拒绝。
        raise RuntimeError(
            "backend:=mujoco 不支持 gravity:=false：MJCF 的重力在生成期固定"
            "（models/<模型>/tools/build_mujoco_model.py），运行期没有对应开关"
        )
    if backend == "mujoco" and (
        not _bool(LaunchConfiguration("fixed_base").perform(context))
        or spawn_z != 0.0
    ):
        # 同理：生成高度与基座焊接都写进 MJCF，gazebo.launch.py 的这两个参数到不了 MuJoCo。
        raise RuntimeError(
            "backend:=mujoco 不支持 spawn_z/fixed_base：MJCF 的生成高度与基座"
            "在生成期固定（models/<模型>/tools/build_mujoco_model.py）"
        )

    share = Path(get_package_share_directory("robot_description"))
    model = LaunchConfiguration("model").perform(context)
    # 电机拓扑来源：显式 motor_config_file 优先；留空时优先**模型目录自带**的
    # config/motor_driver.yaml（模型专属仿真拓扑），否则回落权威整机配置。
    # 默认模型 half_body_robot 自带这份（第二条链也有电机，18 个关节全在拓扑里）。
    requested_config = LaunchConfiguration("motor_config_file").perform(context)
    if requested_config:
        motor_driver_path = Path(requested_config).resolve()
    else:
        model_local = share / "models" / model / "config" / "motor_driver.yaml"
        motor_driver_path = (
            model_local.resolve()
            if model_local.is_file()
            else Path(get_package_share_directory("motor_hardware_interface"))
            / "config"
            / "motor_driver.yaml"
        )
    if not motor_driver_path.is_file():
        raise RuntimeError(f"motor configuration file does not exist: {motor_driver_path}")
    motor_specs_path = _resolve_motor_specs_path(motor_driver_path)
    if not motor_specs_path.is_file():
        raise RuntimeError(f"motor specs file does not exist: {motor_specs_path}")
    hardware_share = Path(get_package_share_directory("motor_hardware_interface"))
    topology_file = share / "models" / model / "config" / "simulation_topology.yaml"
    if not topology_file.is_file():
        raise RuntimeError(
            f"full simulation topology does not exist for model '{model}': "
            f"{topology_file}"
        )
    simulation = _load_simulation(topology_file)
    simulation = _bind_motor_topology(
        simulation,
        motor_driver_path,
        motor_specs_path,
    )
    if model != simulation["model"]:
        raise RuntimeError(
            f"model '{model}' does not match simulation topology model "
            f"'{simulation['model']}'"
        )
    try:
        from robot_transmission import Model
    except ImportError as error:
        raise RuntimeError(
            "robot_transmission Python bindings are required; source the ROS workspace"
        ) from error
    try:
        transmission_model = Model.load(str(motor_driver_path))
    except Exception as error:
        raise RuntimeError(
            f"cannot load transmission model from {motor_driver_path}: {error}"
        ) from error
    model_joint_order = list(transmission_model.joint_names)
    topology_joint_order = [motor["joint"] for motor in simulation["motors"]]
    joint_order = (
        topology_joint_order
        if set(topology_joint_order) == set(model_joint_order)
        and len(topology_joint_order) == len(model_joint_order)
        else model_joint_order
    )
    # 关节集合的权威来源随后端切换：Gazebo 看控制器配置，MuJoCo 看 MJCF。
    # 两者都只做「拓扑关节 ⊆ 模型关节」的子集校验；顺序由 launch 显式交给下游
    # （Gazebo 重写 controllers.yaml 的 joints、MuJoCo 传 joint_names），不靠巧合。
    mujoco_model = None
    if backend == "mujoco":
        mujoco_model = share / "models" / model / "mujoco" / f"{model}.xml"
        if not mujoco_model.is_file():
            raise RuntimeError(
                f"model '{model}' has no MuJoCo model; expected {mujoco_model} "
                f"(生成：python3 models/{model}/tools/build_mujoco_model.py)"
            )
        # mujoco.launch.py 的 robot_state_publisher 用 standalone URDF 渲染
        # robot_description；缺它 rsp 会「起来了但模型是空的」，所以一并 fail fast。
        standalone_urdf = (
            share / "models" / model / "urdf" / f"{model}.standalone.urdf"
        )
        if not standalone_urdf.is_file():
            raise RuntimeError(
                f"model '{model}' has no standalone URDF for the MuJoCo backend: "
                f"{standalone_urdf}"
            )
        model_joints = _mujoco_joint_names(mujoco_model)
        model_description = f"MuJoCo model {mujoco_model}"
    else:
        model_joints = _model_joint_names(share, model)
        model_description = "Gazebo model"
    unknown_joints = set(joint_order) - model_joints
    if unknown_joints:
        raise RuntimeError(
            f"simulation topology references joints missing from the "
            f"{model_description}: {sorted(unknown_joints)}"
        )

    vcan_interface = None
    if transport == "vcan":
        vcan_interface = LaunchConfiguration("vcan_interface").perform(context)
        if not re.fullmatch(r"[A-Za-z0-9_.-]{1,15}", vcan_interface):
            raise RuntimeError(f"invalid Linux CAN interface name: {vcan_interface!r}")
        sim_endpoint = vcan_interface
        driver_endpoint = vcan_interface
    elif transport == "socketcan":
        sim_endpoint = _required(context, "sim_interface")
        driver_endpoint = sim_endpoint
        interface_kinds = {
            interface: _validate_socketcan_interface(interface)
            for interface in (sim_endpoint, driver_endpoint)
        }
        if (controller_mode == "firmware"
                and interface_kinds[sim_endpoint] != "can"):
            raise RuntimeError(
                "STM32 firmware HIL needs a physical CAN-FD interface; "
                "vcan cannot connect to an MCU"
            )
    else:
        sim_endpoint = _required(context, "sim_tty")
        driver_endpoint = sim_endpoint
        for tty in {sim_endpoint, driver_endpoint}:
            if not Path(tty).exists():
                raise RuntimeError(f"SLCAN device does not exist: {tty}")

    start_web_host = False
    start_agent = False
    sim_script = None
    web_share = None
    agent_tty = None
    agent_waiter_script = None
    if controller_mode == "firmware":
        motor_ids = [motor["id"] for motor in simulation["motors"]]
        if len(motor_ids) != len(set(motor_ids)):
            raise RuntimeError(
                "firmware simulator maps joints by motor ID; IDs must be unique "
                "across protocols on its single CAN bus"
            )
        start_web_host = _bool(
            LaunchConfiguration("start_web_host").perform(context)
        )
        start_agent = _bool(
            LaunchConfiguration("start_micro_ros_agent").perform(context)
        )
        if start_web_host and start_agent:
            raise RuntimeError(
                "start_web_host and start_micro_ros_agent cannot both be true: "
                "web_host manages the Agent when the browser connects"
            )
        if start_agent and not start_fdcan_sim:
            raise RuntimeError(
                "start_micro_ros_agent:=true 依赖模拟器写出的 ready-file 排序；"
                "start_fdcan_sim:=false 时模拟器由外部管理，请让外部起 Agent"
                "（或 start_web_host:=true）"
            )
        if start_fdcan_sim:
            sim_script_arg = (
                LaunchConfiguration("firmware_sim_script").perform(context).strip()
            )
            if sim_script_arg:
                sim_script = Path(sim_script_arg)
            else:
                prefix = Path(get_package_prefix("robot_sim")).resolve()
                candidates = [
                    parent / "micro_ros_agent" / "tools" / "fdcan_motor_sim.py"
                    for parent in (prefix, *prefix.parents)
                ]
                sim_script = next(
                    (candidate for candidate in candidates if candidate.is_file()), None
                )
                if sim_script is None:
                    raise RuntimeError(
                        "cannot locate micro_ros_agent/tools/fdcan_motor_sim.py; "
                        "set firmware_sim_script:=<path>"
                    )
            if not sim_script.is_file():
                raise RuntimeError(f"firmware simulator script not found: {sim_script}")
        if start_web_host:
            web_share = Path(get_package_share_directory("web_host"))
            if not (web_share / "launch" / "web_host.launch.py").is_file():
                raise RuntimeError(f"web_host launch file not found under {web_share}")
        if start_agent:
            agent_tty = _required(context, "agent_tty")
            if not Path(agent_tty).exists():
                raise RuntimeError(f"micro-ROS Agent tty does not exist: {agent_tty}")
            if Path(agent_tty).resolve() == Path(sim_endpoint).resolve():
                raise RuntimeError("agent_tty and sim_tty must refer to different devices")
            agent_waiter_script = (
                Path(get_package_prefix("robot_sim"))
                / "lib"
                / "robot_sim"
                / "agent_after_sim_ready.py"
            )
            # 必须连可执行位一起查：--symlink-install 下安装件是指向源文件的
            # 符号链接，源文件丢了 +x 就只剩「文件在、但起不来」这种假通过。
            if not (agent_waiter_script.is_file() and os.access(agent_waiter_script, os.X_OK)):
                raise RuntimeError(
                    f"installed micro-ROS Agent launcher helper not found or not "
                    f"executable: {agent_waiter_script}"
                )

    plant_launch_file = share / "launch" / (
        "mujoco.launch.py" if backend == "mujoco" else "gazebo.launch.py"
    )
    if not plant_launch_file.is_file():
        raise RuntimeError(f"被控对象 launch 文件不存在：{plant_launch_file}")
    vcan_created = False
    vcan_was_up = False
    vcan_prepared = False
    cleanup_vcan = False
    runtime_dir = None
    try:
        if vcan_interface is not None:
            vcan_created, vcan_was_up = _ensure_vcan(vcan_interface)
            vcan_prepared = True
            cleanup_vcan = (vcan_created and _bool(
                LaunchConfiguration("remove_vcan_on_exit").perform(context)
            )) or (not vcan_created and not vcan_was_up)
        runtime_dir = Path(tempfile.mkdtemp(prefix="robot_arm_sim_"))
        motor_sim_config, driver_config = _write_runtime_configs(
            simulation,
            runtime_dir,
            transport,
            driver_endpoint,
            sim_endpoint,
            motor_specs_path,
            motor_driver_path,
        )
        # Gazebo 的控制器参数只有 Gazebo 路需要；MuJoCo 桥自己就是被控对象，
        # 不做这件事也就不会凭空多一条对 models/<模型>/config/controllers.yaml 的启动期依赖。
        gazebo_controller_config = (
            _write_gazebo_controller_config(share, model, joint_order, runtime_dir)
            if backend == "gazebo"
            else None
        )
    except Exception:
        if runtime_dir is not None:
            shutil.rmtree(runtime_dir, ignore_errors=True)
        if vcan_created:
            subprocess.run(
                ["ip", "link", "delete", "dev", vcan_interface],
                capture_output=True,
                text=True,
            )
        elif vcan_prepared and vcan_interface is not None and not vcan_was_up:
            subprocess.run(
                ["ip", "link", "set", "dev", vcan_interface, "down"],
                capture_output=True,
                text=True,
            )
        raise

    if backend == "mujoco":
        # joint_names 用**拓扑顺序**，桥据此按名字映射命令数组：不再依赖
        # 「MJCF 关节顺序恰好等于拓扑顺序」（那是巧合，Gazebo 侧同类错位踩过一次）。
        plant_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(plant_launch_file)),
            launch_arguments={
                "model": model,
                "joint_names": ",".join(joint_order),
            }.items(),
        )
    else:
        plant_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(plant_launch_file)),
            launch_arguments={
                "model": model,
                "world_file": LaunchConfiguration("world_file"),
                "controllers_file": str(gazebo_controller_config),
                "control_mode": "effort",
                "gravity": LaunchConfiguration("gravity"),
                # 生成高度与基座焊接按模型几何定：换模型必须核对（见各模型 README）。
                "spawn_z": LaunchConfiguration("spawn_z"),
                "fixed_base": LaunchConfiguration("fixed_base"),
            }.items(),
        )

    if simulator_only:
        # 「电机后端」单起：被控对象/控制器/驱动端全部跳过，只跑模拟器
        # （plant_launch 仍被构造但不注册，不会执行）。
        actions = [
            LogInfo(
                msg=(
                    f"FDCAN motor backend only: transport={transport}, "
                    f"model={model}"
                )
            ),
        ]
    else:
        actions = [
            LogInfo(
                msg=(
                    f"Complete arm simulation: transport={transport}, "
                    f"controller_mode={controller_mode}, model={model}, "
                    f"backend={backend}"
                )
            ),
            plant_launch,
        ]

    if controller_mode == "firmware":
        cerb_ids = [
            str(motor["id"])
            for motor in simulation["motors"]
            if motor["protocol"] == "cerb"
        ]
        vaxor_ids = [
            str(motor["id"])
            for motor in simulation["motors"]
            if motor["protocol"] == "vaxor"
        ]
        joint_map = ",".join(
            f"{motor['id']}:{motor['joint']}" for motor in simulation["motors"]
        )
        # 物理饱和逐台取型号库 peak_torque_nm，与 MJCF 的峰值限制同源。
        tau_limits = _tau_limit_cli(simulation)
        # τ_ff 的解码量程与饱和值分开：固件按 mit_torque_limit_nm（CERB 300 N·m）
        # 编码，用峰值 45/11 解码会把前馈缩成 45/300。
        mit_torque_limits = _mit_torque_limit_cli(simulation)
        # 反馈电流/转速的换算系数（逐台，来自型号库）。模拟器没有第二个真相源：
        # 它把**实际施加的关节力矩**除以 joint_nm_per_amp 得到上报电流，把关节速度
        # 乘速比得到上报转速——与 C++ motor_sim 的 frame_router 同式，两个后端在
        # 页面上给出同一个数。缺了它们，模拟器只会报 0（宁可为 0 也不猜系数）。
        joint_nm_per_amp = ",".join(
            f"{motor['id']}:{motor['joint_nm_per_amp']:g}"
            for motor in simulation["motors"]
        )
        gear_ratios = ",".join(
            f"{motor['id']}:{motor['gear_ratio']:g}"
            for motor in simulation["motors"]
        )
        r58_ids = [
            str(motor["id"])
            for motor in simulation["motors"]
            if motor["protocol"] == "cerb"
            and motor["type"] == "cerb_r58_101"
        ]
        ready_file = runtime_dir / "motor_bus_sim_ready"
        # 透传给模拟器的额外参数（空格分隔）。用于台架对照实验，例如
        # --no-brs（FD 数据段也走 1M，隔离 BRS 数据相位）、--fb-period-ms 100
        # （降反馈负载）。不设默认：正常路径行为不变。
        extra_sim_args = LaunchConfiguration("sim_args").perform(context).split()
        # 控制通道默认路径须与 bench_manager/motor_sim_manager.py 的 _CONTROL_SOCKET 一致；
        # 残留 socket 由模拟器清理复用。
        control_socket = (
            LaunchConfiguration("sim_control_socket").perform(context).strip()
            or "/tmp/robot_sim_motor_backend.sock"
        )
        firmware_bus_args = (
            ["--interface", sim_endpoint]
            if transport == "socketcan"
            else ["--tty", sim_endpoint]
        )
        # start_fdcan_sim:=false 时不构造模拟器动作（web 上位机「电机后端」
        # 以独立的 simulator_only launch 管它，避免两个实例争同一条总线）。
        if start_fdcan_sim:
            simulator = ExecuteProcess(
                cmd=[
                    "python3",
                    "-u",
                    str(sim_script),
                    *firmware_bus_args,
                    "--cerb",
                    ",".join(cerb_ids),
                    "--vaxor",
                    ",".join(vaxor_ids),
                    "--cerb-r58",
                    ",".join(r58_ids),
                    "--joint-map",
                    joint_map,
                    "--joint-names",
                    ",".join(joint_order),
                    "--driver-file",
                    str(driver_config),
                    "--tau-limits",
                    tau_limits,
                    "--mit-torque-limits",
                    mit_torque_limits,
                    "--mit-position-limits",
                    _mit_limit_cli(simulation, "mit_position_limit_rad"),
                    "--mit-velocity-limits",
                    _mit_limit_cli(simulation, "mit_velocity_limit_rad_s"),
                    "--mit-kp-limits",
                    _mit_limit_cli(simulation, "mit_kp_limit"),
                    "--mit-kd-limits",
                    _mit_limit_cli(simulation, "mit_kd_limit"),
                    "--joint-nm-per-amp",
                    joint_nm_per_amp,
                    "--gear",
                    gear_ratios,
                    "--gazebo",
                    "--gazebo-control-mode",
                    "effort",
                    "--effort-topic",
                    "/arm_effort_controller/commands",
                    "--ready-file",
                    str(ready_file),
                    # 40 ms 周期、18 台约 450 帧/s，低于 100 ms 离线阈值，保留 2.5 倍余量。
                    # 37.4 ms 错峰窗口降低突发负载；位时序错误导致的 bus-off 不能靠改周期修复。
                    # 排在 extra_sim_args 之前：命令行后出现者胜，故 sim_args 仍可覆盖。
                    "--fb-period-ms",
                    "40",
                    *(["--control-socket", control_socket] if control_socket else []),
                    *extra_sim_args,
                ],
                output="screen",
                sigterm_timeout="5",
            )
            actions.extend(
                [
                    simulator,
                    RegisterEventHandler(
                        OnProcessExit(
                            target_action=simulator,
                            on_exit=[
                                EmitEvent(
                                    event=Shutdown(reason="FDCAN simulator exited")
                                )
                            ],
                        )
                    ),
                ]
            )
        # MCU 仍为设备后端，工作台轨迹与手动调试统一由主机控制器执行。
        if not simulator_only:
            control_urdf = share / "models" / model / "urdf" / f"{model}.urdf"
            if not control_urdf.is_file():
                raise RuntimeError(f"host control URDF does not exist: {control_urdf}")
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(
                    hardware_share / "launch" / "host_trajectory.launch.py")),
                launch_arguments={
                    "motor_config_file": str(driver_config), "urdf_file": str(control_urdf),
                    "model": model, "start_controllers": "false",
                    "publish_joint_states": "false", "publish_robot_state": "false",
                }.items()))
        if start_web_host:
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        str(web_share / "launch" / "web_host.launch.py")
                    )
                )
            )
        if start_agent:
            agent_waiter = ExecuteProcess(
                cmd=[
                    "python3",
                    str(agent_waiter_script),
                    str(ready_file),
                    agent_tty,
                ],
                output="screen",
                sigterm_timeout="5",
            )
            actions.extend(
                [
                    agent_waiter,
                    RegisterEventHandler(
                        OnProcessExit(
                            target_action=agent_waiter,
                            on_exit=[
                                EmitEvent(
                                    event=Shutdown(reason="micro-ROS Agent exited")
                                )
                            ],
                        )
                    ),
                ]
            )
    else:
        motor_sim = Node(
            package="robot_sim",
            executable="motor_sim_node",
            name="motor_sim",
            output="screen",
            parameters=[
                {
                    "config_file": str(motor_sim_config),
                    "driver_file": str(motor_driver_path),
                    "joint_names": joint_order,
                    "joint_states_topic": "/joint_states",
                    "effort_topic": "/arm_effort_controller/commands",
                    "drive_state_topic": "/motor_sim/drive_state",
                    "control_gate_topic": "/motor_sim/control_gate",
                    "fault_latching_enabled": True,
                }
            ],
        )
        gate = Node(
            package="robot_sim",
            executable="simulation_gate.py",
            name="motor_sim_gate",
            output="screen",
        )
        actions.extend(
            [
                motor_sim,
                gate,
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=motor_sim,
                        on_exit=[
                            EmitEvent(event=Shutdown(reason="motor_sim exited"))
                        ],
                    )
                ),
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=gate,
                        on_exit=[
                            EmitEvent(event=Shutdown(reason="simulation gate exited"))
                        ],
                    )
                ),
            ]
        )

    cleanup_actions = [
        ExecuteProcess(
            cmd=[
                "python3",
                "-c",
                "import shutil,sys; shutil.rmtree(sys.argv[1], ignore_errors=True)",
                str(runtime_dir),
            ],
            output="log",
        )
    ]
    if cleanup_vcan:
        cleanup_actions.append(
            ExecuteProcess(
                cmd=[
                    "ip",
                    "link",
                    "delete" if vcan_created else "set",
                    "dev",
                    LaunchConfiguration("vcan_interface"),
                    *([] if vcan_created else ["down"]),
                ],
                output="screen",
            )
        )
    actions.append(RegisterEventHandler(OnShutdown(on_shutdown=cleanup_actions)))
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "transport",
                default_value="vcan",
                description="CAN backend: vcan | socketcan | slcan",
            ),
            DeclareLaunchArgument(
                "controller_mode",
                default_value="external",
                description=(
                    "external (external CAN controller), "
                    "firmware (STM32 HIL over SocketCAN or SLCAN)"
                ),
            ),
            DeclareLaunchArgument(
                "backend",
                default_value="gazebo",
                description=(
                    "被控对象后端：gazebo（gz sim + gz_ros2_control）或 "
                    "mujoco（自写力矩直通桥）。只替换被控对象这一层，"
                    "传输/控制端/被控电机模型/门控完全共用"
                ),
            ),
            DeclareLaunchArgument(
                "model",
                default_value="half_body_robot",
                description="models/ 下含 simulation_topology.yaml 的模型目录",
            ),
            DeclareLaunchArgument(
                "motor_config_file",
                default_value="",
                description=(
                    "电机拓扑配置；留空则优先模型目录自带的 "
                    "config/motor_driver.yaml，其次用权威整机配置"
                ),
            ),
            DeclareLaunchArgument("world_file", default_value="empty.sdf"),
            DeclareLaunchArgument("gravity", default_value="true"),
            DeclareLaunchArgument(
                "spawn_z",
                default_value="0.0",
                description="Gazebo 模型生成高度（m），按模型几何底面定"
                            "（默认模型底面 z = 0）；mujoco 后端拒绝非 0",
            ),
            DeclareLaunchArgument(
                "fixed_base",
                default_value="true",
                description="Gazebo 把基座焊到世界原点（false = 自由浮动基座）；"
                            "mujoco 后端拒绝 false",
            ),
            DeclareLaunchArgument("vcan_interface", default_value="robot_vcan0"),
            DeclareLaunchArgument("remove_vcan_on_exit", default_value="true"),
            DeclareLaunchArgument("sim_interface", default_value="can1"),
            DeclareLaunchArgument("sim_tty", default_value=""),
            DeclareLaunchArgument("agent_tty", default_value=""),
            DeclareLaunchArgument("start_web_host", default_value="true"),
            DeclareLaunchArgument("start_micro_ros_agent", default_value="false"),
            DeclareLaunchArgument("firmware_sim_script", default_value=""),
            DeclareLaunchArgument(
                "simulator_only",
                default_value="false",
                description="只起 FDCAN 模拟器（跳过被控对象/控制器/驱动端），"
                            "要求 controller_mode:=firmware + 非 vcan 传输；"
                            "供 web 上位机把「电机后端」作为独立进程管理",
            ),
            DeclareLaunchArgument(
                "start_fdcan_sim",
                default_value="true",
                description="firmware 流程内嵌启动 FDCAN 模拟器；false = 由"
                            "外部（web 上位机）负责，本 launch 不拉模拟器",
            ),
            DeclareLaunchArgument(
                "sim_control_socket",
                default_value="",
                description="FDCAN 模拟器的运行期控制通道（Unix socket 路径，"
                            "JSON 行协议）；留空 = 不开。web 上位机的仿真页经"
                            "它做加/减电机与断连，见 micro_ros_agent/README.md",
            ),
            DeclareLaunchArgument(
                "sim_args",
                default_value="",
                description="追加给 FDCAN 模拟器的额外参数（空格分隔），"
                            "如 --no-brs / --fb-period-ms 100",
            ),
            OpaqueFunction(function=launch_simulation),
        ]
    )
