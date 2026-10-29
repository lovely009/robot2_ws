"""主机 JTC → MicroRosSystem；由 robot.launch.py 组合唯一的主机桥。"""
import math
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnShutdown, OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from robot_transmission import Model


def hardware_description(config_file, urdf_file):
    config_file = str(Path(config_file).resolve(strict=True))
    model = Model.load(config_file)
    ids = list(model.actuator_ids.values())
    if not 0 < len(ids) <= 48 or len(set(ids)) != len(ids) or any(not 1 <= i <= 64 for i in ids):
        raise ValueError('MCU motor_id 必须全局唯一、在 1..64 内，且总数不超过 48')
    root = ET.parse(Path(urdf_file).resolve(strict=True)).getroot()
    if root.tag != 'robot' or any('xacro' in element.tag for element in root.iter()):
        raise ValueError('urdf_file 必须是展开后的 robot URDF')
    joint_nodes = root.findall('joint')
    joints_by_name = {joint.get('name'): joint for joint in joint_nodes}
    if len(joints_by_name) != len(joint_nodes):
        raise ValueError('URDF 关节名重复')
    for existing in root.findall('ros2_control'):
        root.remove(existing)
    control = ET.SubElement(root, 'ros2_control', {'name': 'micro_ros_system', 'type': 'system'})
    hardware = ET.SubElement(control, 'hardware')
    ET.SubElement(hardware, 'plugin').text = 'motor_hardware_interface/MicroRosSystem'
    ET.SubElement(hardware, 'param', {'name': 'config_file'}).text = config_file
    joints = list(model.joint_names)
    for name in joints:
        real_joint = joints_by_name.get(name)
        if (real_joint is None or real_joint.get('type') not in ('revolute', 'prismatic', 'continuous')
                or real_joint.find('mimic') is not None):
            raise ValueError(f'{name} 必须是 URDF 中独立的活动关节')
        limits = real_joint.find('limit')
        try:
            velocity = float(limits.get('velocity'))
            effort = float(limits.get('effort'))
            if not all(math.isfinite(v) and v > 0 for v in (velocity, effort)):
                raise ValueError()
            position = None
            if real_joint.get('type') != 'continuous':
                position = (float(limits.get('lower')), float(limits.get('upper')))
                if not all(math.isfinite(v) for v in position) or position[0] >= position[1]:
                    raise ValueError()
        except (AttributeError, TypeError, ValueError) as error:
            raise ValueError(f'{name} 缺少有效的位置/速度限位') from error
        joint = ET.SubElement(control, 'joint', {'name': name})
        for channel, bounds in (('position', position), ('velocity', (-velocity, velocity)),
                                ('effort', (-effort, effort))):
            interface = ET.SubElement(joint, 'command_interface', {'name': channel})
            if bounds:
                for key, value in zip(('min', 'max'), bounds):
                    ET.SubElement(interface, 'param', {'name': key}).text = str(value)
        for channel in ('position', 'velocity', 'effort'):
            ET.SubElement(joint, 'state_interface', {'name': channel})
    return ET.tostring(root, encoding='unicode'), joints


def controller_parameters(joints, urdf_file=''):
    parameters = {
        '/host_control/controller_manager': {'ros__parameters': {
            'update_rate': 100,
            'joint_state_broadcaster': {'type': 'joint_state_broadcaster/JointStateBroadcaster'},
            'trajectory_controller': {'type': 'joint_trajectory_controller/JointTrajectoryController'},
            'gravity_controller': {'type': 'robot_dynamics/GravityCompensationController'},
        }},
        '/host_control/trajectory_controller': {'ros__parameters': {
            'joints': list(joints),
            'command_interfaces': ['position', 'velocity'],
            'state_interfaces': ['position', 'velocity'],
            'open_loop_control': False,
            'allow_partial_joints_goal': False,
            'constraints': {
                'goal_time': 1.0, 'stopped_velocity_tolerance': 0.02,
                **{name: {'trajectory': 0.1, 'goal': 0.01} for name in joints},
            },
        }},
        '/host_control/gravity_controller': {'ros__parameters': {
            'joints': list(joints), 'urdf_file': urdf_file,
            'enabled': False, 'effort_gain': 1.0,
        }},
    }
    for index, joint in enumerate(joints):
        name = f'manual_controller_{index}'
        parameters['/host_control/controller_manager']['ros__parameters'][name] = {
            'type': 'forward_command_controller/MultiInterfaceForwardCommandController'}
        parameters[f'/host_control/{name}'] = {'ros__parameters': {
            'joint': joint, 'interface_names': ['position', 'velocity', 'effort']}}
    return parameters


def launch_control_stack(context):
    config_file = LaunchConfiguration('motor_config_file').perform(context)
    urdf_file = LaunchConfiguration('urdf_file').perform(context)
    description, joints = hardware_description(
        config_file, urdf_file)
    started = LaunchConfiguration('start_controllers').perform(context).lower() == 'true'
    temporary = tempfile.TemporaryDirectory(prefix='host-trajectory-')
    parameters_file = str(Path(temporary.name) / 'controllers.yaml')
    with open(parameters_file, 'w', encoding='utf-8') as handle:
        yaml.safe_dump(controller_parameters(joints, str(Path(urdf_file).resolve())), handle,
                       sort_keys=False)
    def cleanup(context):
        temporary.cleanup()
        return []
    manager = Node(package='controller_manager', executable='ros2_control_node',
                   namespace='host_control', output='screen',
                   parameters=[{'robot_description': description}, parameters_file])
    broadcaster = Node(package='controller_manager', executable='spawner', output='screen',
                       arguments=['joint_state_broadcaster', '--controller-manager',
                                  '/host_control/controller_manager'])
    motion = Node(package='controller_manager', executable='spawner', output='screen',
                  arguments=['trajectory_controller', 'gravity_controller',
                             '--controller-manager', '/host_control/controller_manager',
                             *(['--activate-as-group'] if started else ['--inactive'])])
    manual = Node(package='controller_manager', executable='spawner', output='screen',
                  arguments=[*[f'manual_controller_{i}' for i in range(len(joints))],
                             '--controller-manager', '/host_control/controller_manager', '--inactive'])
    client = Node(package='motor_hardware_interface', executable='workbench_control.py',
                  name='workbench_control', output='screen', parameters=[{
                      'config_file': config_file, 'urdf_file': urdf_file,
                      'model_name': LaunchConfiguration('model').perform(context),
                      'publish_joint_states': LaunchConfiguration(
                          'publish_joint_states').perform(context).lower() == 'true',
                  }])
    actions = [manager, client, broadcaster, motion, manual,
               RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
               RegisterEventHandler(OnProcessExit(target_action=manager, on_exit=[
                   EmitEvent(event=Shutdown(reason='host controller manager exited'))])),
               RegisterEventHandler(OnProcessExit(target_action=client, on_exit=[
                   EmitEvent(event=Shutdown(reason='workbench control client exited'))]))]
    if LaunchConfiguration('publish_robot_state').perform(context).lower() == 'true':
        actions.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                            name='host_robot_state_publisher', output='screen',
                            parameters=[{'robot_description': description}],
                            remappings=[('joint_states', '/joint_states')]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('motor_config_file', description='MCU 电机与传动映射，裸 ID 全局唯一'),
        DeclareLaunchArgument('urdf_file', description='对应模型展开后的 URDF，保留真实关节与限位'),
        DeclareLaunchArgument('model', default_value='', description='工作台模型身份'),
        DeclareLaunchArgument('start_controllers', default_value='false', choices=['false', 'true'],
                              description='显式激活 JTC；默认等待新的工作台目标'),
        DeclareLaunchArgument('publish_joint_states', default_value='true'),
        DeclareLaunchArgument('publish_robot_state', default_value='true'),
        OpaqueFunction(function=launch_control_stack),
    ])
