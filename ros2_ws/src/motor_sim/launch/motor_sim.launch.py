"""motor_sim 通用独立启动入口。

与机器人无关的共享电机仿真。拓扑完全由 motor_config_file 决定，
bus→endpoint 映射必须显式传入，不提供任何机器人默认值。

独立使用时，节点从外部 /joint_states 读取当前位置/速度，
经 FrameRouter 计算 effort 并发布到 effort_topic，
同时把电机反馈帧通过所选 transport 发回 CAN 总线。

示例（socket_can，单总线 arm 挂 vcan0）：

  ros2 launch motor_sim motor_sim.launch.py \
    motor_config_file:=/path/to/motors.yaml \
    transport_mode:=socket_can \
    arm_port:=vcan0

示例（usb_serial 双总线）：

  ros2 launch motor_sim motor_sim.launch.py \
    motor_config_file:=/path/to/motors.yaml \
    transport_mode:=usb_serial \
    left_arm_port:=/dev/ttyUSB0 \
    right_arm_port:=/dev/ttyUSB1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 通用可选参数；bus 端口由调用方按配置文件中的 bus 名额外传入，
    # 节点会在运行时从 YAML 解析 bus 列表并逐一读取对应参数。
    args = [
        DeclareLaunchArgument(
            "motor_config_file",
            description="电机 YAML 配置路径（必填；拓扑、协议、ID、profile 都来自该文件）",
        ),
        DeclareLaunchArgument(
            "transport_mode",
            default_value="usb_serial",
            description="传输方式：usb_serial | serial_slcan | socket_can",
        ),
        DeclareLaunchArgument(
            "baud_rate",
            default_value="3000000",
            description="串口波特率（socket_can 忽略）",
        ),
        DeclareLaunchArgument(
            "allow_external_canfd",
            default_value="false",
            description="serial_slcan 时必须显式设为 true",
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

    # 将所有 bus 端口参数转发到节点。总线名完全由 YAML 决定，
    # 这里不声明具体 bus 参数，由节点在运行时按需读取 <bus>_port。
    # 调用方只需在 launch 命令行追加 <bus_name>_port:=<value>。
    node = Node(
        package="motor_sim",
        executable="motor_sim_node",
        name="motor_sim",
        output="screen",
        parameters=[
            {
                "config_file": LaunchConfiguration("motor_config_file"),
                "transport_mode": LaunchConfiguration("transport_mode"),
                "baud_rate": LaunchConfiguration("baud_rate"),
                "allow_external_canfd": LaunchConfiguration(
                    "allow_external_canfd"
                ),
                "joint_states_topic": LaunchConfiguration(
                    "joint_states_topic"
                ),
                "effort_topic": LaunchConfiguration("effort_topic"),
                "fault_latching_enabled": LaunchConfiguration(
                    "fault_latching_enabled"
                ),
            },
        ],
    )

    return LaunchDescription(args + [node])
