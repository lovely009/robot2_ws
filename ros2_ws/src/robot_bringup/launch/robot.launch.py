"""台架入口：rosbridge、静态页、micro_ros_bridge 与 bench_manager。

Agent 默认由 bench_manager 自动启动，其余会话按需起停。
通过 launch 事件串联 configure → activate；配置失败则保持 unconfigured，
不注册服务，原因见日志。
干净退出停止 bench.slice；异常死亡保留 unit，供后继管理器认领。
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessStart
from launch.conditions import LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events.matchers import matches_node_name
from lifecycle_msgs.msg import Transition


def _validate_execution(context):
    execution = LaunchConfiguration('trajectory_execution').perform(context)
    if execution != 'host':
        raise ValueError('工作台只支持 trajectory_execution=host；旧固件轨迹入口已迁移到 ros2_control')
    config = LaunchConfiguration('host_motor_config_file').perform(context)
    urdf = LaunchConfiguration('host_urdf_file').perform(context)
    if config or urdf:
        for name in ('host_motor_config_file', 'host_urdf_file'):
            value = LaunchConfiguration(name).perform(context)
            if not value or not Path(value).is_file():
                raise ValueError(f'host 模式需要有效的 {name}')
    return []


def generate_launch_description():
    # 模型枚举根与臂视图几何都来自模型包；缺包时降级（模型列表为空、臂视图提示
    # 加载失败），静态页面照常——所以这里吞掉查找异常。
    try:
        model_dir = get_package_share_directory('robot_sim')
    except Exception:
        model_dir = ''

    web_launch = os.path.join(
        get_package_share_directory('web_host'), 'launch', 'web_host.launch.py')
    # 下行唯一的编码者：主机 /motor/* ↔ 线上 /micro_ros_*。纯 ROS 节点，
    # 不碰设备，所以随 L1 常驻；没有固件时话题空转，不报错。
    micro_ros_bridge = Node(
        package='micro_ros_bridge',
        executable='micro_ros_bridge_node',
        name='micro_ros_bridge',
        output='screen',
        parameters=[{'trajectory_execution': LaunchConfiguration('trajectory_execution')}],
    )

    bench_manager = LifecycleNode(
        package='robot_bringup',
        executable='bench_manager',
        name='bench_manager',
        namespace='',
        output='screen',
        parameters=[{
            'model_dir': LaunchConfiguration('model_dir'),
            'auto_start_agent': ParameterValue(
                LaunchConfiguration('start_micro_ros_agent'), value_type=bool),
            'agent_device': LaunchConfiguration('agent_tty'),
            'feedback_model': LaunchConfiguration('firmware_model'),
        }],
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
        DeclareLaunchArgument('start_micro_ros_agent', default_value='true',
                              description='激活台架管理器时自动启动 Agent。'),
        DeclareLaunchArgument('agent_tty', default_value='',
                              description='Agent 串口；空值只在唯一串口时自动选择。'),
        DeclareLaunchArgument('firmware_model', default_value='',
                              description='初始控制模型；由台架管理器按 Agent/模型归属启动控制栈。'),
        DeclareLaunchArgument('trajectory_execution', default_value='host',
                              choices=['host'], description='工作台运动统一由 ros2_control 执行'),
        DeclareLaunchArgument('host_motor_config_file', default_value='',
                              description='host 模式的 MCU 电机与传动映射'),
        DeclareLaunchArgument('host_urdf_file', default_value='',
                              description='host 模式对应的已展开 URDF'),
        OpaqueFunction(function=_validate_execution),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(web_launch),
            launch_arguments={
                'model_dir': LaunchConfiguration('model_dir'),
                'rosbridge_port': LaunchConfiguration('rosbridge_port'),
                'web_port': LaunchConfiguration('web_port'),
            }.items(),
        ),

        micro_ros_bridge,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('motor_hardware_interface'),
                'launch', 'host_trajectory.launch.py')),
            condition=LaunchConfigurationNotEquals('host_motor_config_file', ''),
            launch_arguments={
                'motor_config_file': LaunchConfiguration('host_motor_config_file'),
                'urdf_file': LaunchConfiguration('host_urdf_file'),
                'model': LaunchConfiguration('firmware_model'),
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
