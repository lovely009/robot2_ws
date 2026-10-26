"""motor_sim 通用独立启动入口。

与机器人无关的共享电机仿真。拓扑与总线端点完全由 motor_config_file 决定：每条总线的
传输类型与端点写在配置的 buses[] 里，节点不接受任何总线类参数。

独立使用时，节点从外部 /joint_states 读取当前位置/速度，
经 FrameRouter 计算 effort 并发布到 effort_topic，
同时把电机反馈帧通过所选 transport 发回 CAN 总线。

示例（socket_can，单总线 arm 挂 vcan0；端点写在 YAML 的 buses[] 里）：

  ros2 launch motor_sim motor_sim.launch.py \
    motor_config_file:=/path/to/motors.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "motor_config_file",
            description="电机 YAML 配置路径（必填；拓扑、总线端点、协议、ID、profile 都来自该文件）",
        ),
        DeclareLaunchArgument(
            "joint_states_topic",
            default_value="joint_states",
            description="外部关节状态输入话题",
        ),
        DeclareLaunchArgument(
            "effort_topic",
            default_value="effort_controller/commands",
            description="effort 输出话题（Float64MultiArray）",
        ),
        DeclareLaunchArgument(
            "fault_latching_enabled",
            default_value="true",
            description="故障是否锁存；false 仅记录，不清零输出",
        ),
    ]

    node = Node(
        package="motor_sim",
        executable="motor_sim_node",
        name="motor_sim",
        output="screen",
        parameters=[
            {
                "config_file": LaunchConfiguration("motor_config_file"),
                "joint_states_topic": LaunchConfiguration("joint_states_topic"),
                "effort_topic": LaunchConfiguration("effort_topic"),
                "fault_latching_enabled": LaunchConfiguration(
                    "fault_latching_enabled"
                ),
            },
        ],
    )

    return LaunchDescription(args + [node])
