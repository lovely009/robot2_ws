"""静态验证电机、URDF 与 effort controller 的 30 轴 ABI。"""

import math
from pathlib import Path
import subprocess
from typing import Dict, List
from xml.etree import ElementTree

import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
MOTORS_YAML = PACKAGE / "config/motors.yaml"
CONTROLLERS_YAML = PACKAGE / "config/controllers.yaml"
HUMANOID_XACRO = PACKAGE / "urdf/humanoid.urdf.xacro"
MECHANICS_YAML = PACKAGE / "config/mechanics.yaml"


def _motor_document() -> dict:
    return yaml.safe_load(MOTORS_YAML.read_text())["motor_simulation"]


def _motor_nodes() -> List[dict]:
    return _motor_document()["motors"]


def _expanded_robot() -> ElementTree.Element:
    result = subprocess.run(
        [
            "xacro",
            str(HUMANOID_XACRO),
            f"controllers_file:={CONTROLLERS_YAML}",
            f"mechanics_file:={MECHANICS_YAML}",
            f"motor_config_file:={MOTORS_YAML}",
        ],
        check=True,
        text=True,
        capture_output=True,
    )
    return ElementTree.fromstring(result.stdout)


def test_motor_contract_has_exact_bus_protocol_and_id_topology() -> None:
    document = _motor_document()
    motors = _motor_nodes()
    assert document["buses"] == ["left_arm", "right_arm", "legs"]
    assert len(motors) == 30
    assert sum(node["protocol"] == "rp" for node in motors) == 20
    assert sum(node["protocol"] == "motor_2530" for node in motors) == 10

    assert [(m["protocol"], m["id"]) for m in motors[:9]] == [
        *(('rp', motor_id) for motor_id in range(1, 7)),
        *(('motor_2530', motor_id) for motor_id in range(1, 4)),
    ]
    assert [(m["protocol"], m["id"]) for m in motors[9:18]] == [
        *(('rp', motor_id) for motor_id in range(1, 7)),
        *(('motor_2530', motor_id) for motor_id in range(1, 4)),
    ]
    assert [(m["protocol"], m["id"]) for m in motors[18:]] == [
        *(('rp', motor_id) for motor_id in range(1, 5)),
        ('motor_2530', 1),
        ('motor_2530', 2),
        *(('rp', motor_id) for motor_id in range(5, 9)),
        ('motor_2530', 3),
        ('motor_2530', 4),
    ]


def test_motor_urdf_and_controller_joint_order_matches() -> None:
    expected = [node["joint"] for node in _motor_nodes()]
    robot = _expanded_robot()
    actuated = [
        joint.attrib["name"]
        for joint in robot.findall("joint")
        if joint.attrib["type"] == "revolute"
    ]
    ros2_control = [
        joint.attrib["name"] for joint in robot.findall(".//ros2_control/joint")
    ]
    controller = yaml.safe_load(CONTROLLERS_YAML.read_text())
    controlled = controller["effort_controller"]["ros__parameters"]["joints"]
    assert actuated == expected
    assert ros2_control == expected
    assert controlled == expected


def test_urdf_limits_are_generated_from_motor_profiles() -> None:
    document = _motor_document()
    profiles: Dict[str, dict] = document["profiles"]
    robot = _expanded_robot()
    joints = {node.attrib["name"]: node for node in robot.findall("joint")}
    for motor in _motor_nodes():
        limit = joints[motor["joint"]].find("limit")
        assert limit is not None
        profile = profiles[motor["profile"]]
        assert float(limit.attrib["lower"]) == pytest.approx(
            math.radians(profile["hard_lower_deg"])
        )
        assert float(limit.attrib["upper"]) == pytest.approx(
            math.radians(profile["hard_upper_deg"])
        )
        assert float(limit.attrib["velocity"]) == pytest.approx(
            profile["velocity_rad_s"]
        )
        assert float(limit.attrib["effort"]) == pytest.approx(profile["effort_nm"])


def test_left_and_right_joint_axes_share_anatomical_command_sign() -> None:
    robot = _expanded_robot()
    joints = {node.attrib["name"]: node for node in robot.findall("joint")}
    same_axes = [
        "shoulder_pitch",
        "elbow_pitch",
        "wrist_flexion",
        "hip_pitch",
        "knee_pitch",
        "ankle_pitch",
    ]
    mirrored_axes = [
        "scapula_protraction",
        "scapula_elevation",
        "shoulder_roll",
        "shoulder_yaw",
        "forearm_roll",
        "wrist_deviation",
        "hip_yaw",
        "hip_roll",
        "ankle_roll",
    ]
    for name in same_axes:
        left = joints[f"left_{name}_joint"].find("axis")
        right = joints[f"right_{name}_joint"].find("axis")
        assert left is not None and right is not None
        assert left.attrib["xyz"] == right.attrib["xyz"]
    for name in mirrored_axes:
        left = joints[f"left_{name}_joint"].find("axis")
        right = joints[f"right_{name}_joint"].find("axis")
        assert left is not None and right is not None
        left_axis = tuple(float(value) for value in left.attrib["xyz"].split())
        right_axis = tuple(float(value) for value in right.attrib["xyz"].split())
        assert left_axis == tuple(-value for value in right_axis)

    left_origin = joints["left_scapula_protraction_joint"].find("origin")
    right_origin = joints["right_scapula_protraction_joint"].find("origin")
    assert left_origin is not None and right_origin is not None
    left_y = float(left_origin.attrib["xyz"].split()[1])
    right_y = float(right_origin.attrib["xyz"].split()[1])
    assert left_y == pytest.approx(-right_y)
