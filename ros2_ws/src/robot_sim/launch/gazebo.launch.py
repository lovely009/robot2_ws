"""启动 Ignition Gazebo 仿真：空世界 + 生成模型 + gz_ros2_control 控制器。

模型由 `model:=` 选择（`models/<模型名>/`），默认 `half_body_robot`。
控制方式（启动完成后；命令个数 = 该模型 `arm_effort_controller` 的关节数，
half_body_robot 是 18 个 = joint_link1..9 → joint_link1.1..9.9）：

  ros2 topic pub --once /arm_effort_controller/commands \\
    std_msgs/msg/Float64MultiArray "{data: [1.0, 0.0, 0.0, 0.0]}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("robot_sim")
    model = LaunchConfiguration("model")
    control_mode = LaunchConfiguration("control_mode")
    gravity = LaunchConfiguration("gravity")
    xacro_file = PathJoinSubstitution(
        [pkg_share, "models", model, "urdf", [model, ".urdf.xacro"]]
    )
    controllers_file = LaunchConfiguration("controllers_file")
    world_file = LaunchConfiguration("world_file")
    verbosity = LaunchConfiguration("verbosity")

    # Ignition 用 model:// 解析 mesh（URDF 的 package:// 会被转换），须把
    # install/robot_sim/share 加入资源搜索路径（其下有 robot_sim/models）。
    # 模型挪进 models/ 后仍是同一个前缀，这条逻辑与模型无关。
    pkg_install_share = os.path.dirname(get_package_share_directory("robot_sim"))
    resource_path = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    ign_resource_path = (
        f"{pkg_install_share}{os.pathsep}{resource_path}" if resource_path
        else pkg_install_share
    )

    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                xacro_file,
                " controllers_file:=",
                controllers_file,
                " fixed_base:=",
                LaunchConfiguration("fixed_base"),
                " gravity:=",
                gravity,
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
            "gz_args": ["-v ", verbosity, " -r ", world_file],
        }.items(),
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )
    # /clock 桥接让 use_sim_time 的节点跟随仿真时间
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )
    # spawn_z 默认 0.0：默认模型 half_body_robot 的 base_link 网格
    # z ∈ [0.0000, 0.7029]，零位下全模型最低点恰为 z = 0.000000，故无需抬起。
    # 换模型要重新核这个值。
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", model,
            "-topic", "robot_description",
            "-z", LaunchConfiguration("spawn_z"),
        ],
    )
    # 依赖链：模型生成 -> joint_state_broadcaster -> arm_position_controller
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout", "60"],
    )
    position_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["arm_position_controller", "--controller-manager-timeout", "60"],
        condition=UnlessCondition(
            PythonExpression(["'", control_mode, "' == 'effort'"])
        ),
    )
    effort_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["arm_effort_controller", "--controller-manager-timeout", "60"],
        condition=IfCondition(
            PythonExpression(["'", control_mode, "' == 'effort'"])
        ),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value="half_body_robot",
                description="models/ 下的模型目录名",
            ),
            DeclareLaunchArgument(
                "world_file",
                default_value="empty.sdf",
                description="Ignition 世界文件（gz sim 可解析的路径）",
            ),
            DeclareLaunchArgument(
                "spawn_z",
                default_value="0.0",
                description="模型生成高度（m）；默认模型底面在 z = 0，故为 0",
            ),
            DeclareLaunchArgument(
                "fixed_base",
                default_value="true",
                description="把基座焊到世界原点（false = 自由浮动基座）",
            ),
            DeclareLaunchArgument(
                "gravity",
                default_value="true",
                description="第一链连杆是否受 Gazebo 重力（第二链恒关）；position 轨迹验收可设 false",
            ),
            DeclareLaunchArgument(
                "control_mode",
                default_value="effort",
                description="控制器模式：effort（默认，FDCAN/Gazebo 电机后端）或 position",
            ),
            DeclareLaunchArgument(
                "verbosity",
                default_value="2",
                description="Ignition Gazebo 日志级别；调试插件加载可设 4",
            ),
            DeclareLaunchArgument(
                "controllers_file",
                default_value=PathJoinSubstitution(
                    [pkg_share, "models", model, "config", "controllers.yaml"]
                ),
                description="controller_manager 参数文件",
            ),
            SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", ign_resource_path),
            gazebo,
            robot_state_publisher,
            clock_bridge,
            spawn_robot,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_robot,
                    on_exit=[joint_state_spawner],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_spawner,
                    on_exit=[position_spawner, effort_spawner],
                )
            ),
        ]
    )
