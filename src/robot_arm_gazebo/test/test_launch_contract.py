import importlib.util
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_ROOT / "launch/gazebo.launch.py"


def load_launch_module():
    spec = importlib.util.spec_from_file_location("robot_arm_gazebo_launch", LAUNCH_FILE)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_gazebo_launch_uses_robot_arm_namespace_and_fail_fast_chain() -> None:
    text = LAUNCH_FILE.read_text(encoding="utf-8")
    assert 'PushRosNamespace("robot_arm")' in text
    assert '"-name", "ur5e"' in text
    assert '"/robot_arm/controller_manager"' in text
    assert "joint_state_spawner" in text
    assert "effort_spawner" in text
    assert "Shutdown(" in text


@pytest.mark.parametrize(
    "raw",
    (
        "0,0,0,0,0",
        "0,0,0,0,0,0,0",
        "0,nan,0,0,0,0",
        "0,inf,0,0,0,0",
        "0,nope,0,0,0,0",
    ),
)
def test_initial_positions_reject_malformed_vectors(raw: str) -> None:
    module = load_launch_module()
    with pytest.raises(ValueError):
        module.parse_initial_positions(raw, [(-6.28, 6.28)] * 6)


def test_initial_positions_reject_values_outside_hard_limits() -> None:
    module = load_launch_module()
    with pytest.raises(ValueError, match="hard limits"):
        module.parse_initial_positions(
            "0,0,3.2,0,0,0",
            [(-6.28, 6.28)] * 2 + [(-3.14, 3.14)] + [(-6.28, 6.28)] * 3,
        )


def test_initial_positions_accept_six_finite_radians() -> None:
    module = load_launch_module()
    assert module.parse_initial_positions(
        "0, -1.57, 0, -1.57, 0, 0", [(-6.28, 6.28)] * 6
    ) == [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
