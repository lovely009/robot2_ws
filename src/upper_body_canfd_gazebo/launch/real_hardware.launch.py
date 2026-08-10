"""Bring up the real upper-body motors through the CAN-FD hardware plugin.

No Gazebo: robot_state_publisher publishes the model, ros2_control loads
upper_body_canfd_hardware/CanFdMotorSystem for the left/right CAN-FD buses,
and joint_state_broadcaster + effort_controller are spawned.

Examples:
  # vcan bring-up without physical hardware (fake slaves must be running)
  ros2 launch upper_body_canfd_gazebo real_hardware.launch.py \
      left_interface:=vcan0 right_interface:=vcan1

  # physical bring-up (explicitly allows can0/can1)
  ros2 launch upper_body_canfd_gazebo real_hardware.launch.py \
      left_interface:=can0 right_interface:=can1 allow_hardware:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("upper_body_canfd_gazebo")
    xacro_file = PathJoinSubstitution([pkg_share, "urdf", "upper_body.urdf.xacro"])
    controller_file = PathJoinSubstitution([pkg_share, "config", "controllers.yaml"])

    left_interface = LaunchConfiguration("left_interface")
    right_interface = LaunchConfiguration("right_interface")
    allow_hardware = LaunchConfiguration("allow_hardware")

    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                xacro_file,
                " use_real_hardware:=true",
                " left_interface:=",
                left_interface,
                " right_interface:=",
                right_interface,
                " allow_hardware:=",
                allow_hardware,
                " controllers_file:=",
                controller_file,
            ]
        ),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, controller_file],
        output="screen",
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    effort_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["effort_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("left_interface", default_value="can0"),
            DeclareLaunchArgument("right_interface", default_value="can1"),
            DeclareLaunchArgument(
                "allow_hardware",
                default_value="true",
                description=(
                    "Explicitly allow physical CAN interfaces (can0/can1); "
                    "the hardware plugin refuses them otherwise."
                ),
            ),
            robot_state_publisher,
            controller_manager,
            joint_state_spawner,
            RegisterEventHandler(
                OnProcessExit(target_action=joint_state_spawner, on_exit=[effort_spawner])
            ),
        ]
    )
