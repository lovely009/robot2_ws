from upper_body_canfd_gazebo.motion_library import T_POSE, pose_to_command


def test_pose_maps_to_position_controller_joint_order() -> None:
    command = pose_to_command(T_POSE)
    assert len(command) == 14
    assert command[:7] == list(T_POSE.left)
    assert command[7:] == list(T_POSE.right)
