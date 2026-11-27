from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def preview_nodes(context):
    from bvh_broadcaster.retarget import load_motion
    urdf = LaunchConfiguration('urdf').perform(context)
    motion = LaunchConfiguration('motion_file').perform(context)
    load_motion(motion, urdf)
    description = Path(urdf).read_text()
    robot = ET.fromstring(description)
    roots = {e.attrib['name'] for e in robot.findall('link')} - {
        e.attrib['link'] for e in robot.findall('joint/child')}
    if len(roots) != 1:
        raise ValueError('URDF must have exactly one root link')
    return [
        Node(package='bvh_broadcaster', executable='mocap_robot_preview', output='screen',
             parameters=[{'motion_file': motion, 'urdf': urdf}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace='mocap_robot', name='state_publisher', output='screen',
             parameters=[{'robot_description': description, 'frame_prefix': 'retarget_robot/',
                          'publish_frequency': 60.0}],
             remappings=[('joint_states', '/mocap_robot/joint_states')]),
        # CAD +Y forward → ROS +X forward; place the robot beside the human skeleton.
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='mocap_preview_anchor', arguments=[
                 '--x', '0', '--y', '1.5', '--z', '0', '--yaw', '-1.5707963267948966',
                 '--pitch', '0', '--roll', '0', '--frame-id', 'world',
                 '--child-frame-id', 'retarget_robot/' + roots.pop()]),
    ]


def generate_launch_description():
    package = Path(get_package_share_directory('bvh_broadcaster'))
    urdf = PathJoinSubstitution([
        FindPackageShare('robot_description'),
        'models', 'half_body_robot', 'urdf', 'half_body_robot.urdf',
    ])
    return LaunchDescription([
        DeclareLaunchArgument('motion_file', description='Validated retarget .npz file'),
        DeclareLaunchArgument('urdf', default_value=urdf),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=preview_nodes),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', str(package / 'config/retarget.rviz')]),
    ])
