"""Full C++ driver -> vcan -> C++ motor simulator -> Gazebo loopback."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    left_interface = LaunchConfiguration("left_interface")
    right_interface = LaunchConfiguration("right_interface")
    gui = LaunchConfiguration("gui")
    render_engine = LaunchConfiguration("render_engine")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("upper_body_canfd_gazebo"), "launch", "sim.launch.py"]
            )
        ),
        launch_arguments={
            "left_interface": left_interface,
            "right_interface": right_interface,
            "start_bridge": "false",
            "gui": gui,
            "render_engine": render_engine,
        }.items(),
    )

    cpp_bridge = Node(
        package="motor_canfd_sim",
        executable="motor_canfd_sim_node",
        name="motor_canfd_sim",
        output="screen",
        parameters=[
            PathJoinSubstitution([FindPackageShare("motor_canfd_sim"), "config", "sim.yaml"]),
            {"left_interface": left_interface, "right_interface": right_interface},
        ],
    )

    motor_config = PathJoinSubstitution(
        [FindPackageShare("motor_hardware_interface"), "config", "motor_canfd.yaml"]
    )
    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare("upper_body_canfd_gazebo"), "urdf", "upper_body.urdf.xacro"]
                ),
                " hardware_backend:=canfd left_interface:=",
                left_interface,
                " right_interface:=",
                right_interface,
                " motor_config_file:=",
                motor_config,
            ]
        ),
        value_type=str,
    )
    controller_config = PathJoinSubstitution(
        [FindPackageShare("motor_canfd_sim"), "config", "loopback_controllers.yaml"]
    )
    driver_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="driver",
        output="screen",
        parameters=[{"robot_description": robot_description}, controller_config],
    )
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        namespace="driver",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/driver/controller_manager",
        ],
    )
    position_controller = Node(
        package="controller_manager",
        executable="spawner",
        namespace="driver",
        output="screen",
        arguments=[
            "position_controller",
            "--controller-manager",
            "/driver/controller_manager",
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("left_interface", default_value="vcan0"),
            DeclareLaunchArgument("right_interface", default_value="vcan1"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("render_engine", default_value=""),
            gazebo,
            # Start the driver process only after the C++ motor emulator process exists.
            # The hardware plugin itself repeatedly sends zero-gain discovery queries during
            # activation, so small process-start skew does not lose the one-shot query.
            RegisterEventHandler(
                OnProcessStart(target_action=cpp_bridge, on_start=[driver_manager])
            ),
            RegisterEventHandler(
                OnProcessStart(
                    target_action=driver_manager,
                    on_start=[joint_state_broadcaster],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[position_controller],
                )
            ),
            cpp_bridge,
        ]
    )
