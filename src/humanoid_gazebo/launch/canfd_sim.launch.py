"""三路 USB-CANFD（SLCAN-FD 串口协议）的完整 Gazebo 仿真入口。"""

from launch import LaunchDescription

from humanoid_gazebo.launch_support import serial_sim_launch_description


def generate_launch_description() -> LaunchDescription:
    return serial_sim_launch_description("serial_slcan")
