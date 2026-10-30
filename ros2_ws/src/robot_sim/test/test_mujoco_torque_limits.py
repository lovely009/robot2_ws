"""MJCF 夹取使用机械峰值，而非 MIT 协议量程；无峰值时沿用 URDF。"""

import importlib.util
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

MODELS_DIR = Path(__file__).parents[2] / "robot_description" / "models"
MODEL_DIR = MODELS_DIR / "half_body_robot"
TOOL = MODEL_DIR / "tools" / "build_mujoco_model.py"
TOPOLOGY = MODEL_DIR / "config" / "motor_driver.yaml"
GENERATED_MJCF = MODEL_DIR / "mujoco" / "half_body_robot.xml"

# 型号 → 机械峰值（型号库 peak_torque_nm 的独立手抄值）
PEAK_BY_TYPE = {"cerb_r48_101": 11.0, "cerb_r58_101": 45.0}


def _load_builder():
    spec = importlib.util.spec_from_file_location("build_mujoco_model", TOOL)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _topology_motors():
    section = yaml.safe_load(TOPOLOGY.read_text(encoding="utf-8"))["motor_hardware"]
    return section["motors"]


def test_torque_limits_use_model_peak_not_protocol_range():
    limits = _load_builder().torque_limits_by_joint()

    checked = 0
    for spec in _topology_motors().values():
        joint, motor_type = spec.get("joint"), spec.get("type")
        if motor_type not in PEAK_BY_TYPE:
            continue
        assert limits[joint] == PEAK_BY_TYPE[motor_type], joint
        checked += 1
    assert checked > 0, "拓扑里没有可校验的 CERB 电机"


def test_vaxor_falls_back_to_urdf_rated_effort():
    """vaxor 型号库没有 peak_torque_nm，生成器不写 actuatorfrcrange，沿用 URDF 1.15。"""
    limits = _load_builder().torque_limits_by_joint()
    vaxor_joints = [
        spec["joint"] for spec in _topology_motors().values()
        if str(spec.get("type", "")).startswith("vaxor")
    ]
    assert vaxor_joints
    for joint in vaxor_joints:
        assert joint not in limits

    root = ET.parse(GENERATED_MJCF).getroot()
    by_name = {joint.get("name"): joint for joint in root.iter("joint")}
    for joint in vaxor_joints:
        assert by_name[joint].get("actuatorfrcrange") == "-1.15 1.15", joint
