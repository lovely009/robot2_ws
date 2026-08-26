from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
EXPECTED = [
    ("shoulder_pan_joint", "arm", "rp", 1),
    ("shoulder_lift_joint", "arm", "rp", 2),
    ("elbow_joint", "arm", "rp", 3),
    ("wrist_1_joint", "arm", "motor_2530", 4),
    ("wrist_2_joint", "arm", "motor_2530", 5),
    ("wrist_3_joint", "arm", "motor_2530", 6),
]


def test_motor_and_controller_order_match() -> None:
    motors = yaml.safe_load((PACKAGE_ROOT / "config/motors.yaml").read_text())
    rows = motors["motor_simulation"]["motors"]
    actual = [(row["joint"], row["bus"], row["protocol"], row["id"]) for row in rows]
    assert actual == EXPECTED

    controllers = yaml.safe_load(
        (PACKAGE_ROOT / "config/controllers.yaml").read_text()
    )
    assert controllers["effort_controller"]["ros__parameters"]["joints"] == [
        row[0] for row in EXPECTED
    ]


def test_six_axis_command_limits_stay_inside_hard_limits() -> None:
    config = yaml.safe_load((PACKAGE_ROOT / "config/motors.yaml").read_text())[
        "motor_simulation"
    ]
    assert config["buses"] == ["arm"]
    assert len(config["motors"]) == 6
    for profile in config["profiles"].values():
        assert profile["hard_lower_deg"] < profile["command_lower_deg"]
        assert profile["command_lower_deg"] < profile["command_upper_deg"]
        assert profile["command_upper_deg"] < profile["hard_upper_deg"]
