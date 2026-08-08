"""Start ros2_control against the physical dual CAN-FD buses."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    share = FindPackageShare("upper_body_canfd_gazebo")
    description = ParameterValue(Command([
        FindExecutable(name="xacro"), " ",
        PathJoinSubstitution([share, "urdf", "upper_body.urdf.xacro"]),
        " hardware_backend:=canfd left_interface:=", LaunchConfiguration("left_interface"),
        " right_interface:=", LaunchConfiguration("right_interface"),
    ]), value_type=str)
    config = PathJoinSubstitution([share, "config", "hardware_controllers.yaml"])
    manager = Node(package="controller_manager", executable="ros2_control_node", output="screen",
                   parameters=[{"robot_description": description}, config])
    state_publisher = Node(package="robot_state_publisher", executable="robot_state_publisher",
                           output="screen", parameters=[{"robot_description": description}])
    jsb = Node(package="controller_manager", executable="spawner", output="screen",
               arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"])
    position = Node(package="controller_manager", executable="spawner", output="screen",
                    arguments=["position_controller", "--controller-manager", "/controller_manager"])
    return LaunchDescription([
        DeclareLaunchArgument("left_interface", default_value="can0"),
        DeclareLaunchArgument("right_interface", default_value="can1"),
        manager, state_publisher,
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[position])), jsb,
    ])
