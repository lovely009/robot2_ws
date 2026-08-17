"""Gazebo 模型 + C++ 电机模拟器，不含 ros2_control 驱动控制器。

供模型侧开发/测试使用；完整回环请用 driver_loopback.launch.py。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from motor_canfd_sim.helpers import (
    default_config_share_substitution,
    expected_default_config_path,
)
from motor_hardware_interface.helpers import read_buses

# 总线清单来自仿真拓扑配置：每条总线一个 interface_<总线名> 参数。
SIM_BUSES = read_buses(expected_default_config_path())


def generate_launch_description() -> LaunchDescription:
    gui = LaunchConfiguration("gui")
    render_engine = LaunchConfiguration("render_engine")
    motor_config_file = LaunchConfiguration("motor_config_file")
    bus_overrides = {
        f"bus_override.{name}": LaunchConfiguration(f"interface_{name}")
        for name, _ in SIM_BUSES
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("upper_body_canfd_gazebo"), "launch", "gazebo.launch.py"]
            )
        ),
        launch_arguments={
            "gui": gui,
            "render_engine": render_engine,
        }.items(),
    )

    # 模拟器只接收配置路径；默认使用 motor_canfd_sim 自带的仿真拓扑配置。
    simulator = Node(
        package="motor_canfd_sim",
        executable="motor_canfd_sim_node",
        name="motor_canfd_sim",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("motor_canfd_sim"), "config", "sim.yaml"]
            ),
            {
                "motor_config_file": motor_config_file,
                **bus_overrides,
            },
        ],
    )

    return LaunchDescription(
        [
            *[
                DeclareLaunchArgument(f"interface_{name}", default_value=interface)
                for name, interface in SIM_BUSES
            ],
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("render_engine", default_value="ogre"),
            DeclareLaunchArgument(
                "motor_config_file",
                default_value=default_config_share_substitution(),
                description="电机拓扑配置路径；默认使用仿真侧 motor_canfd_sim/config/motor_driver.yaml",
            ),
            gazebo,
            simulator,
        ]
    )
