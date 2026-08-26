from robot_arm_gazebo.motion_library import ACTIONS, JOINT_LIMITS, JOINT_ORDER


def test_motion_library_has_bounded_six_axis_actions() -> None:
    assert tuple(ACTIONS) == ("home", "joint_scan", "combined", "zero")
    for action in ACTIONS.values():
        for target in action.targets:
            assert len(target) == 6
            assert all(
                JOINT_LIMITS[joint][0] <= value <= JOINT_LIMITS[joint][1]
                for joint, value in zip(JOINT_ORDER, target)
            )


def test_joint_scan_exercises_each_axis_without_changing_abi_order() -> None:
    scan = ACTIONS["joint_scan"]
    assert scan.positions == JOINT_ORDER
    moved_axes = {
        index
        for target in scan.targets
        for index, value in enumerate(target)
        if value != 0.0
    }
    assert moved_axes == set(range(6))
