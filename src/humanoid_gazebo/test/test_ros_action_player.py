from humanoid_gazebo.motion_library import JOINT_ORDER
from humanoid_gazebo.ros_action_player import bounded_pd_effort


def test_pd_effort_preserves_controller_order_and_applies_damping() -> None:
    target = tuple(0.5 for _ in JOINT_ORDER)
    position = tuple(0.1 for _ in JOINT_ORDER)
    velocity = tuple(0.2 for _ in JOINT_ORDER)
    effort = bounded_pd_effort(target, position, velocity, kp=10.0, kd=2.0)
    assert len(effort) == 30
    assert effort == [3.6] * 30


def test_pd_effort_is_bounded_by_each_joint_limit() -> None:
    target = tuple(100.0 for _ in JOINT_ORDER)
    zero = tuple(0.0 for _ in JOINT_ORDER)
    effort = bounded_pd_effort(target, zero, zero, kp=100.0, kd=0.0)
    assert len(effort) == 30
    assert effort[0] == 30.0
    assert effort[2] == 60.0
    assert effort[6] == 4.0
    assert effort[22] == 10.0
