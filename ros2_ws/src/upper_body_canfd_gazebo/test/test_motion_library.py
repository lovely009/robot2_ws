from upper_body_canfd_gazebo.motion_library import (
    ACTIONS,
    CHEER,
    FORWARD,
    LEFT_RAISED,
    READY,
    RIGHT_RAISED,
    T_POSE,
    smoothstep_interpolate,
    validate_pose,
)


def test_action_numbers_are_stable_for_terminal_menu() -> None:
    assert sorted(ACTIONS) == list(range(14))


def test_every_action_keyframe_respects_urdf_limits() -> None:
    for action in ACTIONS.values():
        for frame in action.frames:
            validate_pose(frame.pose)


def test_reference_poses_have_clear_joint_signatures() -> None:
    # T 字姿态主要激励左右 shoulder_roll，且符号镜像。
    assert T_POSE.left[1] > 1.0
    assert T_POSE.right[1] < -1.0
    # 前伸主要使用负 shoulder_pitch，与 T 字动作的主轴不同。
    assert FORWARD.left[2] < -1.0
    assert FORWARD.right[2] < -1.0
    # 单臂辨识动作必须保证另一条 CAN 总线保持准备姿态。
    assert LEFT_RAISED.right == READY.right
    assert RIGHT_RAISED.left == READY.left
    # 欢呼姿态：双臂对称上举（roll 展开 + pitch 大角度前上），左右镜像。
    assert CHEER.left[1] > 0.5 and CHEER.left[2] < -1.2
    assert CHEER.right[1] < -0.5 and CHEER.right[2] < -1.2


def test_smoothstep_never_overshoots_between_ready_and_t_pose() -> None:
    for u in (0.0, 0.25, 0.5, 0.75, 1.0):
        position, _velocity = smoothstep_interpolate(READY, T_POSE, u, duration=2.0)
        for side in ("left", "right"):
            start = getattr(READY, side)
            target = getattr(T_POSE, side)
            actual = getattr(position, side)
            for q0, q1, q in zip(start, target, actual):
                assert min(q0, q1) - 1e-12 <= q <= max(q0, q1) + 1e-12


def test_smoothstep_has_zero_endpoint_velocity() -> None:
    q0, dq0 = smoothstep_interpolate(READY, FORWARD, 0.0, duration=2.0)
    q1, dq1 = smoothstep_interpolate(READY, FORWARD, 1.0, duration=2.0)
    assert q0 == READY
    assert q1 == FORWARD
    assert all(abs(value) < 1e-12 for arm in (dq0.left, dq0.right) for value in arm)
    assert all(abs(value) < 1e-12 for arm in (dq1.left, dq1.right) for value in arm)
