import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('web_host')
    web_root = os.path.join(pkg_share, 'web')

    # 臂视图的几何（URDF + 约 20 MB STL）在模型包（默认 robot_sim）里，
    # 由 static_server 挂到 /robot_sim 下（目录符号链接装不进包，
    # 见 static_server.py 的注释）。缺这个包时降级：静态页面照常，只有臂视图提示
    # 加载失败、模型列表为空——所以这里吞掉查找异常。
    #
    # 本 launch 只做 L1（rosbridge + 静态页）。台架会话的编排者 bench_manager 归
    # `robot_bringup` 的 robot.launch.py——那里才是「起一套台架」的入口，两者分开
    # 是为了让「只想看页面」不必带上进程管理权限。
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
