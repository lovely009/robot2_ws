import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


READ_TOPICS = [
    '/robot/state', '/robot/calibration_state', '/diagnostics', '/joint_states',
    '/tf', '/tf_static', '/robot_transmission/status', '/imu/data', '/imu/temp',
    '/motor/control_states', '/motor/motion_states', '/motor/topology',
    '/motor/trajectory_state', '/motor/status_array', '/motor/events',
    '/motor/device_param_report', '/motor/system_status', '/motor/config_result',
    '/control/status', '/control/trajectory_state',
]
CONTROL_TOPICS = ['/robot/control/command', '/robot/control/policy', '/robot/control/trajectory']
CONTROL_SERVICES = [
    '/robot/acquire_control', '/robot/renew_control', '/robot/set_armed',
    '/robot/release_control', '/robot/stop', '/robot/reset_fault',
    '/robot/authorize_motion', '/robot/execute_maintenance',
    '/robot/follow_joint_trajectory/_action/send_goal',
    '/robot/follow_joint_trajectory/_action/get_result',
    '/robot/follow_joint_trajectory/_action/cancel_goal',
]


def web_nodes(context):
    mode = LaunchConfiguration('control_mode').perform(context)
    if mode not in ('managed', 'readonly'):
        raise ValueError('control_mode must be managed or readonly')
    control = mode == 'managed'
    pkg_share = get_package_share_directory('web_host')
    return [
        Node(
            package='web_host', executable='rosbridge_shim',
            name='rosbridge_websocket', output='screen',
            parameters=[{
                'port': LaunchConfiguration('rosbridge_port'), 'address': '0.0.0.0',
                # Expose public intents and observations. Internal authorized
                # outputs and wire topics must never be writable through the UI.
                'topics_glob': '[]',
                'topics_pub_glob': repr(CONTROL_TOPICS if control else []),
                'topics_sub_glob': repr(READ_TOPICS),
                'services_glob': repr(['/rosapi/*'] + (CONTROL_SERVICES if control else [])),
                # GetResult may wait for an executing goal; it must not block
                # lease renewal or stop requests on the same WebSocket.
                'call_services_in_new_thread': True,
            }]),
        Node(
            package='web_host', executable='static_server',
            name='web_static_server', output='screen',
            parameters=[{
                'host': '0.0.0.0', 'port': LaunchConfiguration('web_port'),
                'web_root': os.path.join(pkg_share, 'web'),
                'model_url_prefix': '/robot_sim', 'model_dir': LaunchConfiguration('model_dir'),
                'control_mode': mode, 'robot_model': LaunchConfiguration('robot_model'),
                'rosbridge_port': LaunchConfiguration('rosbridge_port'),
                'robot_id': LaunchConfiguration('robot_id'),
            }]),
    ]


def generate_launch_description():
    # 模型包几何由 static_server 挂载到 /robot_sim；缺包时仅降级臂视图与模型列表。
    # 本 launch 提供 L1 页面服务；台架编排由 robot_bringup/robot.launch.py 负责，
    # 避免仅浏览页面时引入进程管理权限。
    try:
        model_dir = get_package_share_directory('robot_description')
    except Exception:
        model_dir = ''

    return LaunchDescription([
        DeclareLaunchArgument('control_mode', default_value='managed',
                              description='managed or readonly; real control always passes through the MCU supervisor.'),
        DeclareLaunchArgument('robot_model', default_value='', description='Managed robot model.'),
        DeclareLaunchArgument('robot_id', default_value='', description='Assembly identity for display.'),
        DeclareLaunchArgument('rosbridge_port', default_value='9090',
                              description='WebSocket port of rosbridge.'),
        DeclareLaunchArgument('web_port', default_value='8080',
                              description='HTTP port of the static web UI.'),
        DeclareLaunchArgument('model_dir', default_value=model_dir,
                              description='模型包根目录（含 models/）：臂视图几何'
                                          '（URDF/STL）的挂载源与 /api/models 的枚举根；'
                                          '可指向源码目录，空 = 不挂载不枚举。'),

        OpaqueFunction(function=web_nodes),
    ])
