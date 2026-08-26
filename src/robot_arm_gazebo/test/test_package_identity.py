import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_workspace_contains_only_three_approved_packages() -> None:
    result = subprocess.run(
        ["colcon", "list", "--names-only"],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    assert result.stdout.splitlines() == [
        "humanoid_gazebo",
        "motor_sim",
        "robot_arm_gazebo",
    ]
