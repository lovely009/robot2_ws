"""只启动 C++ 电机模拟器（虚拟 CAN 电机设备），不启动 Gazebo 与驱动控制器。

默认使用本包 config/motor_driver.yaml（仿真侧拓扑，经 motor_specs_file
引用 config/motor.yaml 型号库）；上层 launch 需要不同拓扑时用
motor_config_file 覆盖。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    motor_config_file = LaunchConfiguration("motor_config_file")
    default_motor_config = default_config_share_substitution()
    bus_overrides = {
        f"bus_override.{name}": LaunchConfiguration(f"interface_{name}")
        for name, _ in SIM_BUSES
    }

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
            DeclareLaunchArgument(
                "motor_config_file",
                default_value=default_motor_config,
                description="电机拓扑配置路径；默认使用本包的仿真侧拓扑配置",
            ),
            *[
                DeclareLaunchArgument(f"interface_{name}", default_value=interface)
                for name, interface in SIM_BUSES
            ],
            simulator,
        ]
    )
