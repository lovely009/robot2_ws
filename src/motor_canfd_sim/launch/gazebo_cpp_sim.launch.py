"""Run the existing Fortress model with the new C++ CAN-FD motor bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    left_interface = LaunchConfiguration("left_interface")
    right_interface = LaunchConfiguration("right_interface")
    gui = LaunchConfiguration("gui")
    render_engine = LaunchConfiguration("render_engine")

    # 与真机驱动共用电机拓扑，只在启动时把物理 CAN 总线重定向到 vcan。
    motor_config = PathJoinSubstitution(
        [FindPackageShare("motor_hardware_interface"), "config", "motor_canfd.yaml"]
    )

    old_model_launch = IncludeLaunchDescription(
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
            {
                "motor_config_file": motor_config,
                "bus_override.left_arm": left_interface,
                "bus_override.right_arm": right_interface,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("left_interface", default_value="vcan0"),
            DeclareLaunchArgument("right_interface", default_value="vcan1"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("render_engine", default_value=""),
            old_model_launch,
            cpp_bridge,
        ]
    )
