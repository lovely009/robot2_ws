import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('web_host')
    web_root = os.path.join(pkg_share, 'web')

    # 臂视图的几何（URDF + 约 20 MB STL）在 robot_sim 的 share 目录里，
    # 由 static_server 挂到 /robot_sim 下（目录符号链接装不进包，
    # 见 static_server.py 的注释）。缺这个包时降级：静态页面照常，只有臂视图
    # 提示加载失败——所以这里吞掉查找异常。
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
                              description='臂视图几何（URDF/STL）所在的包 share 目录；'
                                          '空 = 不挂载（臂视图会提示加载失败）。'),

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
    ])
