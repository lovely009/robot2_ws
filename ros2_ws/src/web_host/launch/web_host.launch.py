import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('web_host')
    web_root = os.path.join(pkg_share, 'web')

    # 臂视图的几何（URDF + 约 20 MB STL）在模型包（默认 robot_sim）里，
    # 由 static_server 挂到 /robot_sim 下（目录符号链接装不进包，
    # 见 static_server.py 的注释）；同一个目录也是 static_server 与
    # bench_manager 的模型枚举根。缺这个包时降级：静态页面照常，只有臂视图提示
    # 加载失败、模型列表为空——所以这里吞掉查找异常。
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
                              description='模型包根目录（含 models/）：既是臂视图几何'
                                          '（URDF/STL）的挂载源，也是 /api/models 与'
                                          '仿真/电机后端的模型枚举根；可指向源码目录，'
                                          '空 = 不挂载不枚举。'),
        DeclareLaunchArgument('start_bench_manager', default_value='true',
                              description='同时启动 bench_manager（/bench/* 进程管理'
                                          '服务）；false = 只做静态服务与 rosbridge。'),

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

        # 台架进程管理（仿真 / micro-ROS Agent / 电机后端）走 ROS 服务；
        # 前端只依赖 rosbridge 与静态 HTTP 两个通用面。
        Node(
            package='bench_manager',
            executable='bench_manager',
            name='bench_manager',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_bench_manager')),
            parameters=[{
                'model_dir': LaunchConfiguration('model_dir'),
            }],
        ),
    ])
