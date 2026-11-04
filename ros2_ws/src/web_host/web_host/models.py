"""模型目录、传动元数据与电机型号库的只读文件接口。"""

import importlib
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml

# motor_id → 关节映射的来源文件，按 simulation_model_joints 的解析优先级排列。
_DRIVER_CONFIG = "config/motor_driver.yaml"
_TOPOLOGY_CONFIG = "config/simulation_topology.yaml"


def _models_root():
    """默认读取 robot_description 安装树；调用方可用 root 覆盖。"""
    try:
        return Path(get_package_share_directory("robot_description")) / "models"
    except Exception:
        return None


def enumerate_arm_models(root=None):
    """列出具有主 URDF 的模型；几何预览不要求电机映射或仿真配置。"""
    models_root = Path(root) if root is not None else _models_root()
    if models_root is None or not models_root.is_dir():
        return []
    return sorted(
        entry.name
        for entry in models_root.iterdir()
        if entry.is_dir() and (entry / "urdf" / f"{entry.name}.urdf").is_file()
    )


def model_transmissions(model, root=None):
    """由 robot_transmission.Model 解析传动组，返回 UI 元数据与可直连关节。"""
    models_root = Path(root) if root is not None else _models_root()
    if models_root is None or not models_root.is_dir():
        raise ValueError("找不到模型根目录（models/ 不存在或安装树不完整？）")
    if not model or model != Path(model).name or model in (".", ".."):
        raise ValueError(f"非法模型名：{model!r}")
    driver_file = models_root / model / _DRIVER_CONFIG
    if not driver_file.is_file():
        raise ValueError(f"模型 {model!r} 没有 {_DRIVER_CONFIG}")
    try:
        model_class = importlib.import_module("robot_transmission").Model
        parsed = model_class.load(str(driver_file))
        ids = dict(parsed.actuator_ids)
        if len(set(ids.values())) != len(ids):
            raise ValueError("Web 设备与传动接口要求裸 motor_id 唯一")
        groups = list(parsed.groups)
    except (ImportError, RuntimeError, OSError, ValueError) as exc:
        raise ValueError(f"无法读取模型 {model!r} 的传动组：{exc}") from exc
    hardware = yaml.safe_load(driver_file.read_text(encoding="utf-8"))["motor_hardware"]
    policy_defaults = {}
    for name, motor in hardware.get("motors", {}).items():
        command = motor.get("command", {})
        if name in ids and all(key in command for key in ("kp", "kd")):
            policy_defaults[int(ids[name])] = {key: float(command[key]) for key in ("kp", "kd")}
    encoded = []
    for group in groups:
        motors = []
        for actuator in group.actuators:
            if actuator not in ids:
                raise ValueError(
                    f"传动组 {group.name!r} 的电机 {actuator!r} 没有 motor id")
            motors.append({"name": actuator, "id": int(ids[actuator])})
        encoded.append({
            "name": str(group.name),
            "plugin": str(group.plugin),
            "motors": motors,
            "joints": [str(joint) for joint in group.joints],
        })
    direct = {}
    for group in encoded:
        if len(group["motors"]) != 1 or len(group["joints"]) != 1:
            continue
        motor, joint = group["motors"][0], group["joints"][0]
        old = direct.get(motor["id"])
        if old is not None and old != joint:
            raise ValueError(
                f"电机 {motor['name']!r} 在多个单电机组中关联不同关节")
        direct[motor["id"]] = joint
    for group in encoded:
        motor, joint = group["motors"], group["joints"]
        group["direct_mapping"] = (len(motor) == 1 and len(joint) == 1
                                   and direct.get(motor[0]["id"]) == joint[0])
        if group["direct_mapping"]:
            group["legacy_mapping_reason"] = None
        else:
            group["legacy_mapping_reason"] = "coupled_group_not_flattened"
    return {
        "model": model,
        "driver_file": str(driver_file),
        "policy_defaults": policy_defaults,
        "groups": encoded,
        "direct_joints": dict(sorted(direct.items())),
    }


def has_joint_mapping_source(model, root=None):
    """检查模型是否声明驱动或拓扑文件；已声明但解析失败应由调用方报错。"""
    models_root = Path(root) if root is not None else _models_root()
    if (models_root is None or not model or model != Path(model).name
            or model in (".", "..")):
        return False
    return any((models_root / model / rel).is_file()
               for rel in (_DRIVER_CONFIG, _TOPOLOGY_CONFIG))


def simulation_model_joints(model, root=None):
    """读取电机 id → 关节名：优先模型驱动配置，否则用拓扑关联权威驱动配置。

    非法模型名、配置缺失、条目不完整或 id 重复时抛出 ValueError。"""
    models_root = Path(root) if root is not None else _models_root()
    if models_root is None or not models_root.is_dir():
        raise ValueError("找不到模型根目录（models/ 不存在或安装树不完整？）")
    # 防目录穿越：模型名只允许是单层目录名
    if not model or model != Path(model).name or model in (".", ".."):
        raise ValueError(f"非法模型名：{model!r}")

    driver_config = models_root / model / _DRIVER_CONFIG
    if driver_config.is_file():
        return _joints_from_driver_config(driver_config)
    return _joints_from_topology(model, models_root / model)


def _joints_from_driver_config(config):
    """一路：驱动配置里每个电机同时带 `id:` 与 `joint:`。"""
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
        _register_joint(joints, motor_id, joint)
    return dict(sorted(joints.items()))


def _joints_from_topology(model, model_dir):
    """二路：拓扑给 (电机名 → 关节)，id 由电机名回查权威驱动配置。"""
    topology_file = model_dir / _TOPOLOGY_CONFIG
    if not topology_file.is_file():
        raise ValueError(
            f"模型 {model!r} 既没有 config/motor_driver.yaml，"
            f"也没有 config/simulation_topology.yaml")
    try:
        driver_config = (
            Path(get_package_share_directory("motor_hardware_interface"))
            / "config" / "motor_driver.yaml")
    except Exception as exc:
        raise ValueError(
            f"找不到 motor_hardware_interface 的权威驱动配置：{exc}") from exc
    if not driver_config.is_file():
        raise ValueError(f"权威驱动配置不存在：{driver_config}")

    with topology_file.open("r", encoding="utf-8") as handle:
        topology = yaml.safe_load(handle) or {}
    entries = (topology.get("simulation") or {}).get("motors")
    if not isinstance(entries, list) or not entries:
        raise ValueError(f"{topology_file} 里没有 simulation.motors")

    with driver_config.open("r", encoding="utf-8") as handle:
        driver = yaml.safe_load(handle) or {}
    motors = (driver.get("motor_hardware") or {}).get("motors")
    if not isinstance(motors, dict):
        raise ValueError(f"{driver_config} 里没有 motor_hardware.motors")

    joints = {}
    for entry in entries:
        if not isinstance(entry, dict):
            raise ValueError(f"{topology_file} 的 motors 条目不是映射：{entry!r}")
        name, joint = entry.get("motor"), entry.get("joint")
        if name is None or joint is None:
            raise ValueError(f"拓扑条目缺 motor 或 joint：{entry!r}")
        spec = motors.get(name)
        if not isinstance(spec, dict) or spec.get("id") is None:
            raise ValueError(f"拓扑电机 {name!r} 在 {driver_config} 里没有 id")
        _register_joint(joints, spec["id"], joint)
    return dict(sorted(joints.items()))


def _register_joint(joints, motor_id, joint):
    motor_id = int(motor_id)
    if motor_id in joints:
        raise ValueError(
            f"电机 id {motor_id} 重复（{joints[motor_id]!r} 与 {joint!r}）："
            f"固件注册表以裸 motor_id 为键，重号会锁存 CONFIG_CONFLICT")
    joints[motor_id] = str(joint)


def _motor_config_dir():
    """权威型号库所在目录（安装树的 robot_description/config）。"""
    try:
        return (Path(get_package_share_directory("robot_description"))
                / "config")
    except Exception:
        return None


def motor_library(root=None):
    """读取 motor.yaml，返回型号参数及协议；缺失时返回空表。

    gear_ratio 用于将关节速度换算为设备对象 0x6080 的电机端 RPM。"""
    config_dir = Path(root) if root is not None else _motor_config_dir()
    if config_dir is None:
        return {}
    path = config_dir / "motor.yaml"
    if not path.is_file():
        return {}
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    types = data.get("motor_types") or {}
    out = {}
    for name, spec in types.items():
        spec = spec or {}
        params = spec.get("parameters") or {}
        out[str(name)] = {"protocol": str(spec.get("protocol") or name), **params}
    return out
