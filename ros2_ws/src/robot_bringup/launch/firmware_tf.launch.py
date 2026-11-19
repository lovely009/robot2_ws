"""只读固件实测关节 → TF；运动统一通过受监督的 MCU 入口。"""

from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from robot_transmission import Model


def firmware_tf_nodes(context: LaunchContext) -> list[Node]:
    enable_trajectory = context.launch_configurations.get('enable_trajectory', 'false').lower()
    if enable_trajectory != 'false':
        raise ValueError('enable_trajectory 必须为 false；运动请使用受监督的 '
                         '/robot/follow_joint_trajectory Action')
    model = LaunchConfiguration('model').perform(context)
    root = Path(LaunchConfiguration('model_dir').perform(context)) / 'models'
    if not model or Path(model).name != model or model in ('.', '..'):
        raise ValueError('model 必须是单层目录名')
    driver_file = root / model / 'config' / 'motor_driver.yaml'
    transmission = Model.load(str(driver_file))
    ids = list(transmission.actuator_ids.values())
    if len(set(ids)) != len(ids):
        raise ValueError('固件映射裸 motor_id 不能重复')
    urdf = root / model / 'urdf' / f'{model}.urdf'
    description = urdf.read_text(encoding='utf-8')
    joints = {j.attrib['name']: j for j in ET.fromstring(description).findall('joint')}
    names = list(transmission.joint_names)
    if len(set(names)) != len(names):
        raise ValueError('多个电机映射到同一关节；需要显式传动换算，不能直接重命名')
    for name in names:
        if name not in joints or joints[name].get('type') == 'fixed':
            raise ValueError(f'映射关节 {name!r} 不是模型中的活动关节：{urdf}')
        if joints[name].find('mimic') is not None:
            raise ValueError(f'映射关节 {name!r} 是 mimic 从动关节，应映射到主动关节')

    return [
        Node(
            package='motor_hardware_interface', executable='firmware_topic_bridge.py',
            name='firmware_topic_bridge', output='screen',
            parameters=[{'driver_file': str(driver_file), 'urdf_file': str(urdf),
                         'model_name': model,
                         'enable_trajectory': False}],
        ),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='firmware_robot_state_publisher', output='screen',
            parameters=[{'robot_description': description, 'use_sim_time': False}],
            remappings=[('joint_states', '/joint_states')],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('model', description='已配置电机映射的模型目录名'),
        DeclareLaunchArgument('model_dir', default_value=get_package_share_directory('robot_description'),
                              description='模型包根目录（含 models/）'),
        DeclareLaunchArgument('enable_trajectory', default_value='false',
                              description='兼容参数，仅允许 false；本入口只读反馈与 TF'),
        OpaqueFunction(function=firmware_tf_nodes),
    ])
