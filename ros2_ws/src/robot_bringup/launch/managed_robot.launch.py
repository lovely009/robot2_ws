"""受监督的固件入口：启动核验、独占租约与只读关节反馈。

所有 facade 写入口均与旧台架隔离。关键进程退出时结束整个 launch；
长轨迹只允许在本 launch 同时管理串口 Agent 时显式开启。
"""

import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
    OpaqueFunction, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


WRITE_REMAPPINGS = (
    ('/motor/command_array', '/robot/managed/motor/command_array'),
    ('/motor/policy_array', '/robot/managed/motor/policy_array'),
    ('/motor/trajectory', '/robot/managed/motor/trajectory'),
    ('/motor/control_lease', '/robot/managed/motor/control_lease'),
    ('/motor/param_set', '/robot/managed/maintenance/param_set'),
    ('/motor/system_command', '/robot/managed/maintenance/system_command'),
    ('/motor/execute_config', '/robot/managed/maintenance/execute_config'),
)


def _argument(context, name):
    return LaunchConfiguration(name).perform(context)


def _boolean(context, name):
    value = _argument(context, name).lower()
    if value not in ('true', 'false'):
        raise ValueError(f'{name} 必须是 true 或 false')
    return value == 'true'


def _feedback_nodes(context):
    # 与旧只读入口共用映射验证，避免另建一套电机/关节解释规则。
    source = Path(__file__).with_name('firmware_tf.launch.py')
    spec = importlib.util.spec_from_file_location('_managed_firmware_tf', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    feedback_context = LaunchContext()
    feedback_context.launch_configurations.update(
        model=_argument(context, 'model'),
        model_dir=_argument(context, 'model_dir'),
        enable_trajectory='false',
    )
    return module.firmware_tf_nodes(feedback_context)


def managed_nodes(context):
    model = _argument(context, 'model')
    if not model or Path(model).name != model or model in ('.', '..'):
        raise ValueError('model 必须是单层模型目录名')
    robot_id = _argument(context, 'robot_id').strip()
    if not robot_id:
        raise ValueError('robot_id 不能为空；应使用这台装配机器的固定标识')
    start_agent = _boolean(context, 'start_agent')
    start_web = _boolean(context, 'start_web')
    allow_trajectories = _boolean(context, 'allow_trajectories')
    device = _argument(context, 'agent_device').strip()
    if start_agent and (not device or not Path(device).is_absolute()):
        raise ValueError('start_agent=true 时须显式提供绝对路径 agent_device')
    if allow_trajectories and not start_agent:
        raise ValueError('轨迹必须由本 launch 管理 Agent：请设置 start_agent=true')

    model_root = Path(_argument(context, 'model_dir')) / 'models' / model
    driver = model_root / 'config' / 'motor_driver.yaml'
    urdf = model_root / 'urdf' / f'{model}.urdf'
    feedback = _feedback_nodes(context)  # 模型/传动错误必须在任何进程启动前拒绝。
    calibration = Node(
        package='robot_calibration', executable='calibration_validator',
        name='calibration_validator', output='screen',
        parameters=[{
            'driver_file': str(driver), 'urdf_file': str(urdf),
            'manifest_file': _argument(context, 'calibration_file'),
            'model_name': model, 'robot_id': robot_id,
        }],
        sigterm_timeout='1', sigkill_timeout='1',
    )
    supervisor = Node(
        package='robot_supervisor', executable='robot_supervisor',
        name='robot_supervisor', output='screen',
        parameters=[{
            'driver_file': str(driver), 'urdf_file': str(urdf),
            'model_name': model, 'robot_id': robot_id,
            'allow_trajectories': allow_trajectories,
        }],
        sigterm_timeout='1', sigkill_timeout='1',
    )
    bridge = Node(
        package='micro_ros_bridge', executable='micro_ros_bridge_node',
        name='micro_ros_bridge', output='screen',
        remappings=list(WRITE_REMAPPINGS),
        sigterm_timeout='1', sigkill_timeout='1',
    )
    critical = [calibration, supervisor, bridge, *feedback]
    if start_agent:
        critical.append(Node(
            package='micro_ros_agent', executable='micro_ros_agent',
            name='managed_micro_ros_agent', output='screen',
            arguments=['serial', '--dev', device],
            sigterm_timeout='1', sigkill_timeout='1',
        ))

    # 先注册事件再启动节点，覆盖启动失败/立即退出。结束会话不会自动重启或复权。
    handlers = [RegisterEventHandler(OnProcessExit(
        target_action=node,
        on_exit=[EmitEvent(event=Shutdown(
            reason='managed robot critical process exited'))],
    )) for node in critical]
    optional = []
    if start_web:
        optional.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(
                get_package_share_directory('web_host')) / 'launch' / 'web_host.launch.py')),
            launch_arguments={
                'model_dir': _argument(context, 'model_dir'),
                'robot_model': model,
                'robot_id': robot_id,
                'control_mode': 'managed',
                'web_port': _argument(context, 'web_port'),
                'rosbridge_port': _argument(context, 'rosbridge_port'),
            }.items(),
        ))
    # UI and rosbridge are clients. Their exit is handled by lease expiry,
    # without tearing down healthy feedback or MCU state monitoring.
    return [*handlers, *critical, *optional]


def generate_launch_description():
    try:
        model_dir = get_package_share_directory('robot_description')
    except Exception:
        model_dir = ''
    return LaunchDescription([
        DeclareLaunchArgument('model', description='具有已验收装配配置的模型目录名'),
        DeclareLaunchArgument('model_dir', default_value=model_dir,
                              description='模型包根目录，包含 models/'),
        DeclareLaunchArgument('robot_id', description='这台装配机器的固定标识'),
        DeclareLaunchArgument('calibration_file', default_value='',
                              description='标定验收记录；缺失时仅运行核验与反馈，不授权运动'),
        DeclareLaunchArgument('agent_device', default_value='',
                              description='由本 launch 管理的 Agent 串口绝对路径'),
        DeclareLaunchArgument('start_agent', default_value='false',
                              description='显式启动串口 Agent；默认适用于无硬件检查'),
        DeclareLaunchArgument('allow_trajectories', default_value='false',
                              description='显式开放轨迹；须同时由本 launch 管理 Agent'),
        DeclareLaunchArgument('start_web', default_value='false',
                              description='启动托管网页客户端；退出不会结束控制监督'),
        DeclareLaunchArgument('web_port', default_value='8080',
                              description='网页 HTTP 端口'),
        DeclareLaunchArgument('rosbridge_port', default_value='9090',
                              description='网页 ROS WebSocket 端口'),
        OpaqueFunction(function=managed_nodes),
    ])
