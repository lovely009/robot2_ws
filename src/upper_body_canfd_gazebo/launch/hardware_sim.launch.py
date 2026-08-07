"""Full ros2_control stack against the CAN-FD hardware plugin on vcan.

Starts the fake motor slaves (RP broadcast + 25/30 mm wrist drives), the
robot model, ros2_control with upper_body_canfd_hardware/CanFdMotorSystem on
vcan0/vcan1, and the joint state + effort controllers. No Gazebo.

Example:
  ros2 launch upper_body_canfd_gazebo hardware_sim.launch.py

Then inspect the loop with:
  ros2 topic echo /joint_states
  ros2 topic pub -r 50 /effort_controller/commands \
      std_msgs/msg/Float64MultiArray "{data: [0,0,0,0,0,0,0,0,0,0,0,0,0,0]}"
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
                " allow_hardware:=false",
                " controllers_file:=",
                controller_file,
            ]
        ),
        value_type=str,
    )

    fake_slaves = Node(
        package="upper_body_canfd_hardware",
        executable="fake_slaves.py",
        arguments=["--left-interface", left_interface, "--right-interface", right_interface],
        output="screen",
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
            DeclareLaunchArgument("left_interface", default_value="vcan0"),
            DeclareLaunchArgument("right_interface", default_value="vcan1"),
            fake_slaves,
            robot_state_publisher,
            controller_manager,
            # The spawner blocks until /controller_manager is up, so it can be
            # started right away; effort_controller follows joint_state_broadcaster.
            joint_state_spawner,
            RegisterEventHandler(
                OnProcessExit(target_action=joint_state_spawner, on_exit=[effort_spawner])
            ),
        ]
    )
