"""只启动 C++ 电机模拟器（虚拟 CAN 电机设备），不启动 Gazebo 与驱动控制器。

默认使用本包 config/motor_canfd.yaml（仿真侧拓扑，来源于仿真模型）；
上层 launch 需要不同拓扑时用 motor_config_file 覆盖。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    motor_config_file = LaunchConfiguration("motor_config_file")
    left_interface = LaunchConfiguration("left_interface")
    right_interface = LaunchConfiguration("right_interface")
    default_motor_config = PathJoinSubstitution(
        [FindPackageShare("motor_canfd_sim"), "config", "motor_canfd.yaml"]
    )

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
                "bus_override.left_arm": left_interface,
                "bus_override.right_arm": right_interface,
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
            DeclareLaunchArgument("left_interface", default_value="vcan0"),
            DeclareLaunchArgument("right_interface", default_value="vcan1"),
            simulator,
        ]
    )
