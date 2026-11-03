from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    defaults = {'base_frame': 'world', 'frame_prefix': 'mocap', 'scale': '0.01',
                'up_axis': 'y_up', 'apply_root_offset': 'false', 'speed': '1.0', 'loop': 'false'}
    types = {'scale': float, 'speed': float, 'apply_root_offset': bool, 'loop': bool}
    args = [DeclareLaunchArgument('bvh_file', description='Absolute path to a BVH file')]
    args += [DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()]
    args.append(DeclareLaunchArgument('rviz', default_value='false'))
    params = {name: ParameterValue(LaunchConfiguration(name), value_type=types.get(name, str))
              for name in ['bvh_file', *defaults]}
    return LaunchDescription(args + [
        Node(package='bvh_broadcaster', executable='bvh_broadcaster', output='screen',
             parameters=[params]),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', os.path.join(get_package_share_directory('bvh_broadcaster'),
                                           'config', 'skeleton.rviz')]),
    ])
