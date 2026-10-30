"""Default robot entry: the managed MCU chain with the web client enabled.

The included launch requires an explicit model and robot_id. Neither this
entry nor the web client starts an Agent or arms motion by default.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', description='Commissioned model directory name'),
        DeclareLaunchArgument('robot_id', description='Identity of this robot assembly'),
        DeclareLaunchArgument('start_web', default_value='true',
                              description='Start the managed web client'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(Path(__file__).with_name('managed_robot.launch.py'))),
            launch_arguments={
                'model': LaunchConfiguration('model'),
                'robot_id': LaunchConfiguration('robot_id'),
                'start_web': LaunchConfiguration('start_web'),
            }.items()),
    ])
