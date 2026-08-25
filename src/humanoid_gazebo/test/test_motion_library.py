import math

import pytest

from humanoid_gazebo.motion_library import (
    ACTIONS,
    JOINT_LIMITS,
    JOINT_ORDER,
    apply_shoulder_rhythm,
    smoothstep_interpolate,
)


def test_action_names_cover_the_thirty_axis_acceptance_workflow() -> None:
    assert tuple(ACTIONS) == (
        "joint_scan",
        "shoulder_rhythm",
        "arm_combo",
        "suspended_legs",
        "mirror_check",
    )


def test_every_action_has_exact_thirty_joint_values_in_abi_order() -> None:
    assert len(JOINT_ORDER) == 30
    for action in ACTIONS.values():
        assert action.positions == JOINT_ORDER
        assert action.keyframes
        for frame in action.keyframes:
            assert len(frame.values) == 30
            for joint, value in zip(JOINT_ORDER, frame.values):
                lower, upper = JOINT_LIMITS[joint]
                assert lower <= value <= upper


def test_mirror_action_uses_same_anatomical_command_sign() -> None:
    pose = ACTIONS["mirror_check"].keyframes[0].values
    by_joint = dict(zip(JOINT_ORDER, pose))
    for joint in (
        "scapula_protraction_joint",
        "shoulder_roll_joint",
        "shoulder_yaw_joint",
        "elbow_pitch_joint",
        "hip_roll_joint",
        "ankle_roll_joint",
    ):
        assert by_joint[f"left_{joint}"] == pytest.approx(
            by_joint[f"right_{joint}"]
        )


def test_shoulder_rhythm_is_off_by_default_and_uses_requested_ratio() -> None:
    assert apply_shoulder_rhythm(60.0, enabled=False) == (0.0, 0.0)
    assert apply_shoulder_rhythm(60.0, enabled=True) == pytest.approx((7.5, 3.6))
    elevation, protraction = apply_shoulder_rhythm(180.0, enabled=True)
    assert elevation == pytest.approx(30.0)
    assert protraction == pytest.approx(18.0)


def test_smoothstep_has_zero_endpoint_velocity_without_overshoot() -> None:
    start = tuple(0.0 for _ in JOINT_ORDER)
    target = tuple(math.radians(10.0) for _ in JOINT_ORDER)
    q0, dq0 = smoothstep_interpolate(start, target, 0.0, duration=2.0)
    q1, dq1 = smoothstep_interpolate(start, target, 1.0, duration=2.0)
    assert q0 == start
    assert q1 == target
    assert all(abs(value) < 1e-12 for value in (*dq0, *dq1))
    middle, _ = smoothstep_interpolate(start, target, 0.5, duration=2.0)
    assert all(0.0 <= value <= math.radians(10.0) for value in middle)
