"""工作台会话由 bench_manager 按模型选择起停，启动时不使能电机。"""

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
from launch.conditions import LaunchConfigurationNotEquals
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
        model_dir = get_package_share_directory('robot_description')
    except Exception:
        model_dir = ''

    web_launch = os.path.join(
        get_package_share_directory('web_host'), 'launch', 'web_host.launch.py')
    firmware_tf_launch = os.path.join(
        get_package_share_directory('robot_bringup'), 'launch', 'firmware_tf.launch.py')

    bench_manager = LifecycleNode(
        package='robot_bringup',
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
        DeclareLaunchArgument('firmware_model', default_value='',
                              description='固件实测 TF 的模型；空值不启动，仿真时保持为空。'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(web_launch),
            launch_arguments={
                'model_dir': LaunchConfiguration('model_dir'),
                'rosbridge_port': LaunchConfiguration('rosbridge_port'),
                'web_port': LaunchConfiguration('web_port'),
                'control_mode': 'bench',
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(firmware_tf_launch),
            condition=LaunchConfigurationNotEquals('firmware_model', ''),
            launch_arguments={
                'model': LaunchConfiguration('firmware_model'),
                'model_dir': LaunchConfiguration('model_dir'),
                'enable_trajectory': 'false',
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
