import math
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DRIVEN = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def expanded_robot(**arguments: str) -> ET.Element:
    xacro_arguments = [f"{name}:={value}" for name, value in arguments.items()]
    result = subprocess.run(
        ["xacro", str(PACKAGE_ROOT / "urdf/ur5e.urdf.xacro"), *xacro_arguments],
        check=True,
        capture_output=True,
        text=True,
    )
    return ET.fromstring(result.stdout)


def final_sdf(**arguments: str) -> ET.Element:
    robot = expanded_robot(**arguments)
    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf") as urdf:
        urdf.write(ET.tostring(robot, encoding="unicode"))
        urdf.flush()
        result = subprocess.run(
            ["ign", "sdf", "-p", urdf.name],
            check=True,
            capture_output=True,
            text=True,
        )
    return ET.fromstring(result.stdout)


def test_model_has_fixed_base_tool_and_six_effort_interfaces() -> None:
    robot = expanded_robot()
    joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}
    assert joints["base_joint"].attrib["type"] == "fixed"
    assert robot.find("link[@name='tool0']") is not None
    control = robot.find("ros2_control")
    assert control is not None
    assert [joint.attrib["name"] for joint in control.findall("joint")] == DRIVEN
    assert all(
        [interface.attrib["name"] for interface in joint.findall("command_interface")]
        == ["effort"]
        for joint in control.findall("joint")
    )


def test_model_inertias_are_positive_and_finite() -> None:
    robot = expanded_robot()
    masses = [float(node.attrib["value"]) for node in robot.findall(".//inertial/mass")]
    assert masses
    assert all(math.isfinite(value) and value > 0.0 for value in masses)


def test_joint_damping_is_split_between_arm_and_wrist() -> None:
    robot = expanded_robot(arm_joint_damping="12.0", wrist_joint_damping="1.5")
    joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}
    for name in DRIVEN[:3]:
        assert joints[name].find("dynamics").attrib["damping"] == "12.0"
    for name in DRIVEN[3:]:
        assert joints[name].find("dynamics").attrib["damping"] == "1.5"


def test_model_self_collision_is_enabled_by_default_and_can_be_overridden() -> None:
    robot = expanded_robot()
    assert len(robot.findall(".//collision")) == 7

    enabled = final_sdf()
    assert enabled.findtext("model/self_collide") == "true"

    disabled = final_sdf(self_collision_enabled="false")
    assert disabled.findtext("model/self_collide") == "false"


def test_gz_ros2_control_namespace_can_be_overridden() -> None:
    robot = expanded_robot(robot_namespace="test_arm")
    plugin = robot.find("gazebo/plugin[@name='gz_ros2_control::GazeboSimROS2ControlPlugin']")
    assert plugin is not None
    assert plugin.findtext("ros/namespace") == "/test_arm"
