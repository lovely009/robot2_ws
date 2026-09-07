"""启动 Ignition Gazebo 仿真：空世界 + 生成 zhoubuguanjie0 + gz_ros2_control 控制器。

控制方式（启动完成后）：
  ros2 topic pub --once /arm_position_controller/commands \
    std_msgs/msg/Float64MultiArray "{data: [0.5, -0.3, 1.0, 0.2]}"
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("robot_arm_gazebo")
    xacro_file = PathJoinSubstitution([pkg_share, "urdf", "zhoubuguanjie0.urdf.xacro"])
    controllers_file = PathJoinSubstitution(
        [pkg_share, "config", "controllers.yaml"]
    )
    world_file = LaunchConfiguration("world_file")

    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                xacro_file,
                " controllers_file:=",
                controllers_file,
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
            "gz_args": ["-v 2 -r ", world_file],
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
    # z=0.1 抬高生成点，避免 base_link 下探的连杆穿地
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", "zhoubuguanjie0",
            "-topic", "robot_description",
            "-z", "0.1",
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
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world_file",
                default_value="empty.sdf",
                description="Ignition 世界文件（gz sim 可解析的路径）",
            ),
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
                    on_exit=[position_spawner],
                )
            ),
        ]
    )
