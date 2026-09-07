"""启动 RViz 模型预览（无 Gazebo）：滑杆驱动关节，验证模型外观与运动学。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("robot_arm_gazebo")
    urdf_file = os.path.join(pkg_share, "urdf", "zhoubuguanjie0.urdf")
    rviz_config = os.path.join(pkg_share, "config", "display.rviz")

    with open(urdf_file, "r", encoding="utf-8") as handle:
        robot_description = {"robot_description": handle.read()}

    return LaunchDescription(
        [
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[robot_description],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )
