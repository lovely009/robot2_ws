"""验证悬空人形模型的质量、尺寸、支架和惯量。"""

from pathlib import Path
import subprocess
from typing import Dict, Tuple
from xml.etree import ElementTree

import pytest


PACKAGE = Path(__file__).resolve().parents[1]


def _robot() -> ElementTree.Element:
    result = subprocess.run(
        [
            "xacro",
            str(PACKAGE / "urdf/humanoid.urdf.xacro"),
            f"controllers_file:={PACKAGE / 'config/controllers.yaml'}",
            f"mechanics_file:={PACKAGE / 'config/mechanics.yaml'}",
            f"motor_config_file:={PACKAGE / 'config/motors.yaml'}",
        ],
        check=True,
        text=True,
        capture_output=True,
    )
    return ElementTree.fromstring(result.stdout)


def _xyz(element: ElementTree.Element | None) -> Tuple[float, float, float]:
    if element is None:
        return (0.0, 0.0, 0.0)
    values = element.attrib.get("xyz", "0 0 0").split()
    return tuple(float(value) for value in values)  # type: ignore[return-value]


def _zero_pose_link_origins(robot: ElementTree.Element) -> Dict[str, Tuple[float, float, float]]:
    remaining = list(robot.findall("joint"))
    origins: Dict[str, Tuple[float, float, float]] = {"world": (0.0, 0.0, 0.0)}
    while remaining:
        progressed = False
        for joint in list(remaining):
            parent = joint.find("parent").attrib["link"]  # type: ignore[union-attr]
            if parent not in origins:
                continue
            child = joint.find("child").attrib["link"]  # type: ignore[union-attr]
            offset = _xyz(joint.find("origin"))
            origins[child] = tuple(
                origins[parent][index] + offset[index] for index in range(3)
            )  # type: ignore[assignment]
            remaining.remove(joint)
            progressed = True
        assert progressed, "URDF joint tree contains an unreachable link"
    return origins


def test_dynamic_link_mass_is_exactly_45_kg() -> None:
    robot = _robot()
    masses = [
        float(node.attrib["value"])
        for node in robot.findall("./link/inertial/mass")
    ]
    assert sum(masses) == pytest.approx(45.0, abs=1e-9)


def test_model_is_fixed_with_feet_10_cm_high_and_height_170_cm() -> None:
    robot = _robot()
    joints = {node.attrib["name"]: node for node in robot.findall("joint")}
    assert joints["world_to_support_joint"].attrib["type"] == "fixed"
    assert joints["support_to_pelvis_joint"].attrib["type"] == "fixed"
    origins = _zero_pose_link_origins(robot)

    foot = next(
        link for link in robot.findall("link") if link.attrib["name"] == "left_foot_link"
    )
    foot_visual = foot.find("visual")
    foot_center_z = origins["left_foot_link"][2] + _xyz(
        foot_visual.find("origin")  # type: ignore[union-attr]
    )[2]
    foot_height = float(
        foot_visual.find("geometry/box").attrib["size"].split()[2]  # type: ignore[union-attr]
    )
    foot_bottom = foot_center_z - foot_height / 2.0

    head = next(
        link for link in robot.findall("link") if link.attrib["name"] == "head_link"
    )
    head_radius = float(
        head.find("visual/geometry/sphere").attrib["radius"]  # type: ignore[union-attr]
    )
    head_top = origins["head_link"][2] + head_radius
    assert foot_bottom == pytest.approx(0.10, abs=0.005)
    assert head_top == pytest.approx(1.80, abs=0.005)
    assert head_top - foot_bottom == pytest.approx(1.70, abs=0.01)


def test_every_dynamic_link_has_positive_diagonal_inertia() -> None:
    for link in _robot().findall("link"):
        mass = link.find("inertial/mass")
        if mass is None:
            continue
        assert float(mass.attrib["value"]) > 0.0
        inertia = link.find("inertial/inertia")
        assert inertia is not None
        assert float(inertia.attrib["ixx"]) > 0.0
        assert float(inertia.attrib["iyy"]) > 0.0
        assert float(inertia.attrib["izz"]) > 0.0
