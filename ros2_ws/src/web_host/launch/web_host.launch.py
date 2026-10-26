import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('web_host')
    web_root = os.path.join(pkg_share, 'web')

    # 模型包几何由 static_server 挂载到 /robot_sim；缺包时仅降级臂视图与模型列表。
    # 本 launch 提供 L1 页面服务；台架编排由 robot_bringup/robot.launch.py 负责，
    # 避免仅浏览页面时引入进程管理权限。
    try:
        model_dir = get_package_share_directory('robot_sim')
    except Exception:
        model_dir = ''

    rosbridge_port = LaunchConfiguration('rosbridge_port')
    web_port = LaunchConfiguration('web_port')

    return LaunchDescription([
        DeclareLaunchArgument('rosbridge_port', default_value='9090',
                              description='WebSocket port of rosbridge.'),
        DeclareLaunchArgument('web_port', default_value='8080',
                              description='HTTP port of the static web UI.'),
        DeclareLaunchArgument('model_dir', default_value=model_dir,
                              description='模型包根目录（含 models/）：臂视图几何'
                                          '（URDF/STL）的挂载源与 /api/models 的枚举根；'
                                          '可指向源码目录，空 = 不挂载不枚举。'),

        # 经 web_host 的垫片启动 rosbridge：兼容有界字符串 string<31>
        # （MotorTopology.model_name），见 web_host/rosbridge_shim.py。
        Node(
            package='web_host',
            executable='rosbridge_shim',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{
                'port': rosbridge_port,
                'address': '0.0.0.0',
            }],
        ),

        Node(
            package='web_host',
            executable='static_server',
            name='web_static_server',
            output='screen',
            parameters=[{
                'host': '0.0.0.0',
                'port': web_port,
                'web_root': web_root,
                'model_url_prefix': '/robot_sim',
                'model_dir': LaunchConfiguration('model_dir'),
            }],
        ),

        # 命令节拍持有者（ADR 0014）：页面只下**设定值**，100 Hz 的完整快照流由它出。
        # 节拍不能留在页面里——浏览器把后台标签页的定时器钳到 1 Hz，而固件的快照新鲜度
        # 线是 30 ms 软停 / 100 ms 失能，页面一失焦台架就整臂掉力。
        # 它只服务页面（页面在，它就该在），所以随 L1 常驻，不做成台架 unit；依赖单向
        # 无环，package.xml 里是真声明。
        Node(
            package='motor_command',
            executable='command_streamer',
            name='command_streamer',
            output='screen',
        ),
    ])
