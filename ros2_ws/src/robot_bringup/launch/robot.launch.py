"""工作台入口：模型由 bench_manager 按选择切换，启动时不使能电机。"""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='micro_ros_bridge', executable='micro_ros_bridge_node', output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(Path(__file__).with_name('bench.launch.py')))),
    ])
