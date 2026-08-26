"""启动命名空间隔离的 UR5e Gazebo effort 仿真。"""

import math

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from robot_arm_gazebo.launch_support import (
    parse_initial_positions as parse_initial_positions_impl,
)


HARD_LIMITS = (
    (-2.0 * math.pi, 2.0 * math.pi),
    (-2.0 * math.pi, 2.0 * math.pi),
    (-math.pi, math.pi),
    (-2.0 * math.pi, 2.0 * math.pi),
    (-2.0 * math.pi, 2.0 * math.pi),
    (-2.0 * math.pi, 2.0 * math.pi),
)


def parse_initial_positions(raw: str, limits=HARD_LIMITS) -> list[float]:
    """保留 launch 模块的公开校验入口，便于静态契约测试。"""

    return parse_initial_positions_impl(raw, limits)


def _start_after_success(next_action, stage: str):
    def _on_exit(event, _context):
        if event.returncode == 0:
            return [] if next_action is None else [next_action]
        reason = f"{stage} 失败（退出码 {event.returncode}），停止 UR5e 仿真"
        return [LogInfo(msg=reason), Shutdown(reason=reason)]

    return _on_exit


def _build_runtime(context):
    # OpaqueFunction 先完成严格解析；异常会在任何 Gazebo/ROS 进程创建前终止 launch。
    values = parse_initial_positions(
        LaunchConfiguration("initial_positions").perform(context)
    )
    package_share = FindPackageShare("robot_arm_gazebo")
    model = PathJoinSubstitution([package_share, "urdf", "ur5e.urdf.xacro"])
    controllers = PathJoinSubstitution([package_share, "config", "controllers.yaml"])
    world = PathJoinSubstitution([package_share, "worlds", "ur5e_world.sdf"])
    initial_names = (
        "shoulder_pan_initial",
        "shoulder_lift_initial",
        "elbow_initial",
        "wrist_1_initial",
        "wrist_2_initial",
        "wrist_3_initial",
    )
    xacro_initials = [
        part
        for name, value in zip(initial_names, values)
        for part in (" ", f"{name}:={value:.17g}")
    ]
    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                model,
                " controllers_file:=",
                controllers,
                *xacro_initials,
            ]
        ),
        value_type=str,
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={
            "on_exit_shutdown": "true",
            "gz_args": ["-r -v 2 ", world],
        }.items(),
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        namespace="robot_arm",
        output="screen",
        arguments=["-name", "ur5e", "-topic", "robot_description"],
    )
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/robot_arm/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )
    effort_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "effort_controller",
            "--controller-manager",
            "/robot_arm/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )
    return [
        gazebo,
        GroupAction([PushRosNamespace("robot_arm"), robot_state_publisher]),
        clock_bridge,
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot,
                on_exit=_start_after_success(joint_state_spawner, "创建 ur5e 模型"),
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_spawner,
                on_exit=_start_after_success(
                    effort_spawner, "启动 joint_state_broadcaster"
                ),
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=effort_spawner,
                on_exit=_start_after_success(None, "启动 effort_controller"),
            )
        ),
        spawn_robot,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "initial_positions", default_value="0,-1.57,0,-1.57,0,0"
            ),
            OpaqueFunction(function=_build_runtime),
        ]
    )
