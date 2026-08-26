import math
import subprocess
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


def expanded_robot() -> ET.Element:
    result = subprocess.run(
        ["xacro", str(PACKAGE_ROOT / "urdf/ur5e.urdf.xacro")],
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
    assert all(joint.find("command_interface").attrib["name"] == "effort" for joint in control.findall("joint"))


def test_model_inertias_are_positive_and_finite() -> None:
    robot = expanded_robot()
    masses = [float(node.attrib["value"]) for node in robot.findall(".//inertial/mass")]
    assert masses
    assert all(math.isfinite(value) and value > 0.0 for value in masses)
