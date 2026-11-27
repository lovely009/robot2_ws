import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


READ_TOPICS = [
    '/diagnostics', '/joint_states',
    '/tf', '/tf_static', '/robot_transmission/status', '/imu/data', '/imu/temp_raw',
    '/motor/motion', '/motor/state', '/motor/events',
    '/control/status', '/bench/status',
]
CONTROL_TOPICS = ['/control/joint_target', '/control/trajectory', '/control/stop']
CONTROL_SERVICES = ['/motor/submit_policy', '/motor/execute_config',
                    '/motor/get_device_info', '/micro_ros_node/*', '/bench/*',
                    '/bench_manager/set_parameters_atomically',
                    '/workbench_control/set_parameters_atomically',
                    '/firmware_topic_bridge/set_parameters_atomically']


def web_nodes(context):
    mode = LaunchConfiguration('control_mode').perform(context)
    if mode not in ('bench', 'readonly'):
        raise ValueError('control_mode must be bench or readonly')
    backend = LaunchConfiguration('backend').perform(context)
    if backend not in ('bff', 'legacy'):
        raise ValueError('backend must be bff or legacy')
    control = mode == 'bench'
    pkg_share = get_package_share_directory('web_host')
    if backend == 'bff':
        args = [
            '--host', LaunchConfiguration('host'),
            '--port', LaunchConfiguration('web_port'),
            '--web-root', os.path.join(pkg_share, 'web'),
            '--model-dir', LaunchConfiguration('model_dir'),
            '--model-url-prefix', '/robot_description',
            '--control-mode', mode,
            '--robot-model', LaunchConfiguration('robot_model'),
        ]
        pin = LaunchConfiguration('pin').perform(context)
        if pin:
            args.extend(['--pin', pin])
        return [
            Node(
                package='web_host', executable='bff_server',
                name='web_bff_server', output='screen',
                arguments=args,
            ),
        ]
    return [
        Node(
            package='web_host', executable='rosbridge_shim',
            name='rosbridge_websocket', output='screen',
            additional_env={'WORKBENCH_WEB_PORT': LaunchConfiguration('web_port')},
            parameters=[{
                'port': LaunchConfiguration('rosbridge_port'), 'address': '127.0.0.1',
                'topics_glob': '[]',
                'topics_pub_glob': repr(CONTROL_TOPICS if control else []),
                'topics_sub_glob': repr(READ_TOPICS),
                'services_glob': repr(['/rosapi/*'] + (CONTROL_SERVICES if control else [])),
                'call_services_in_new_thread': True,
            }]),
        Node(
            package='web_host', executable='static_server',
            name='web_static_server', output='screen',
            parameters=[{
                'host': '127.0.0.1', 'port': LaunchConfiguration('web_port'),
                'web_root': os.path.join(pkg_share, 'web'),
                'model_url_prefix': '/robot_description', 'model_dir': LaunchConfiguration('model_dir'),
                'control_mode': mode, 'robot_model': LaunchConfiguration('robot_model'),
                'rosbridge_port': LaunchConfiguration('rosbridge_port'),
            }]),
    ]


def generate_launch_description():
    try:
        model_dir = get_package_share_directory('robot_description')
    except Exception:
        model_dir = ''

    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='bff',
                              description='Web backend: bff (single-port FastAPI+rclpy) or legacy (rosbridge+static_server).'),
        DeclareLaunchArgument('host', default_value='0.0.0.0',
                              description='HTTP/WS listen host for bff_server.'),
        DeclareLaunchArgument('pin', default_value='',
                              description='Optional control lease PIN for bff_server.'),
        DeclareLaunchArgument('control_mode', default_value='bench',
                              description='bench or readonly.'),
        DeclareLaunchArgument('robot_model', default_value='full_body_robot', description='Initial model selection.'),
        DeclareLaunchArgument('rosbridge_port', default_value='9090',
                              description='WebSocket port of legacy rosbridge.'),
        DeclareLaunchArgument('web_port', default_value='8080',
                              description='HTTP/WS port of the web UI.'),
        DeclareLaunchArgument('model_dir', default_value=model_dir,
                              description='模型包根目录（含 models/）：臂视图几何'
                                          '（URDF/STL）的挂载源与 /api/models 的枚举根；'
                                          '可指向源码目录，空 = 不挂载不枚举。'),

        OpaqueFunction(function=web_nodes),
    ])
