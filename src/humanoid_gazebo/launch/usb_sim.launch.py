"""三路 USB 串口直连的完整 Gazebo 仿真入口。"""

from launch import LaunchDescription

from humanoid_gazebo.launch_support import serial_sim_launch_description


def generate_launch_description() -> LaunchDescription:
    return serial_sim_launch_description("usb_serial")
