import pytest

from robot_arm_gazebo.motion_library import JOINT_ORDER
from robot_arm_gazebo.ros_action_player import (
    bounded_pd_effort,
    validate_command_graph,
    validate_joint_state,
)


def test_pd_effort_is_six_axis_and_clamped_per_joint() -> None:
    target = (100.0,) * 6
    zero = (0.0,) * 6
    assert bounded_pd_effort(target, zero, zero, kp=30.0, kd=1.0) == [
        60.0,
        60.0,
        60.0,
        10.0,
        10.0,
        10.0,
    ]


def test_joint_state_requires_exact_order_finite_values_and_fresh_feedback() -> None:
    positions = [0.0] * 6
    velocities = [0.0] * 6
    validate_joint_state(JOINT_ORDER, positions, velocities, age_s=0.05, stale_after_s=0.15)
    with pytest.raises(RuntimeError, match="order"):
        validate_joint_state(
            tuple(reversed(JOINT_ORDER)),
            positions,
            velocities,
            age_s=0.05,
            stale_after_s=0.15,
        )
    with pytest.raises(RuntimeError, match="nonfinite"):
        validate_joint_state(
            JOINT_ORDER,
            [0.0, 0.0, float("nan"), 0.0, 0.0, 0.0],
            velocities,
            age_s=0.05,
            stale_after_s=0.15,
        )
    with pytest.raises(RuntimeError, match="stale"):
        validate_joint_state(JOINT_ORDER, positions, velocities, age_s=0.16, stale_after_s=0.15)


@pytest.mark.parametrize(
    ("controller_subscriptions", "effort_publishers", "reason"),
    ((0, 1, "controller"), (1, 2, "publisher")),
)
def test_command_graph_rejects_missing_controller_or_second_publisher(
    controller_subscriptions: int, effort_publishers: int, reason: str
) -> None:
    with pytest.raises(RuntimeError, match=reason):
        validate_command_graph(controller_subscriptions, effort_publishers)
