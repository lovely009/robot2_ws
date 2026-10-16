"""台架唯一入口：L1（rosbridge + 静态页）+ L2（bench_manager）。

L3 的三个会话 unit（仿真 / 电机后端 / micro-ROS Agent）由 bench_manager 按需起停——
那是页面上「启动」按钮的事，本 launch 不碰它们。

lifecycle 推进：仓库里没有 lifecycle_manager，用 launch 事件把 configure → activate
串起来（节点一起来发 CONFIGURE，进 inactive 后发 ACTIVATE），于是「起 robot.launch」
就等于「页面立刻能用」。配置失败（例如没有 systemd user manager）时节点停在
unconfigured，服务不会注册——那是刻意的 fail-fast，日志里有原因。

停止：Ctrl-C 让节点走 `stop_sessions()`，停掉整个 `bench.slice`。异常死亡走不到那里，
那些 unit 留给下一个 bench_manager 认领（见 bench_manager/node.py 的模块 docstring）。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events.matchers import matches_node_name
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    # 模型枚举根与臂视图几何都来自模型包；缺包时降级（模型列表为空、臂视图提示
    # 加载失败），静态页面照常——所以这里吞掉查找异常。
    try:
        model_dir = get_package_share_directory('robot_sim')
    except Exception:
        model_dir = ''

    web_launch = os.path.join(
        get_package_share_directory('web_host'), 'launch', 'web_host.launch.py')

    bench_manager = LifecycleNode(
        package='bench_manager',
        executable='bench_manager',
        name='bench_manager',
        namespace='',
        output='screen',
        parameters=[{'model_dir': LaunchConfiguration('model_dir')}],
    )
    bench_matcher = matches_node_name('bench_manager')

    return LaunchDescription([
        DeclareLaunchArgument('model_dir', default_value=model_dir,
                              description='模型包根目录（含 models/）：臂视图几何与'
                                          '模型枚举的共同来源；可指向源码目录。'),
        DeclareLaunchArgument('rosbridge_port', default_value='9090',
                              description='rosbridge WebSocket 端口。'),
        DeclareLaunchArgument('web_port', default_value='8080',
                              description='静态页面 HTTP 端口。'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(web_launch),
            launch_arguments={
                'model_dir': LaunchConfiguration('model_dir'),
                'rosbridge_port': LaunchConfiguration('rosbridge_port'),
                'web_port': LaunchConfiguration('web_port'),
            }.items(),
        ),

        bench_manager,

        RegisterEventHandler(OnProcessStart(
            target_action=bench_manager,
            on_start=[
                LogInfo(msg='bench_manager 已启动，进入配置态'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=bench_matcher,
                    transition_id=Transition.TRANSITION_CONFIGURE)),
            ],
        )),
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=bench_manager,
            goal_state='inactive',
            entities=[
                LogInfo(msg='bench_manager 已配置，激活'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=bench_matcher,
                    transition_id=Transition.TRANSITION_ACTIVATE)),
            ],
        )),
    ])
