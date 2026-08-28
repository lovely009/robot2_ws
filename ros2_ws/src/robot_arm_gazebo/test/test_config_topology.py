from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONTROLLER_WILDCARD = "/**"
EXPECTED = [
    ("shoulder_pan_joint", "arm", "cerb", 1),
    ("shoulder_lift_joint", "arm", "cerb", 2),
    ("elbow_joint", "arm", "cerb", 3),
    ("wrist_1_joint", "arm", "vaxor", 4),
    ("wrist_2_joint", "arm", "vaxor", 5),
    ("wrist_3_joint", "arm", "vaxor", 6),
]


def test_motor_and_controller_order_match() -> None:
    motors = yaml.safe_load((PACKAGE_ROOT / "config/motors.yaml").read_text())
    rows = motors["motor_simulation"]["motors"]
    actual = [(row["joint"], row["bus"], row["protocol"], row["id"]) for row in rows]
    assert actual == EXPECTED

    controllers = yaml.safe_load(
        (PACKAGE_ROOT / "config/controllers.yaml").read_text()
    )
    assert controllers[f"{CONTROLLER_WILDCARD}/effort_controller"][
        "ros__parameters"
    ]["joints"] == [row[0] for row in EXPECTED]


def test_controller_yaml_keys_match_namespaced_nodes() -> None:
    """参数文件顶层键按节点全限定名匹配，裸键在 /robot_arm 命名空间下整文件失效。"""

    controllers = yaml.safe_load(
        (PACKAGE_ROOT / "config/controllers.yaml").read_text()
    )
    assert controllers[f"{CONTROLLER_WILDCARD}/controller_manager"][
        "ros__parameters"
    ]["update_rate"] == 100
    for name in ("joint_state_broadcaster", "effort_controller", "brake_controller"):
        assert (
            controllers[f"{CONTROLLER_WILDCARD}/controller_manager"][
                "ros__parameters"
            ][name]["type"]
            != ""
        )


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
