"""启动悬空 30 轴人形模型及 Gazebo effort controller。"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    Shutdown,
)
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


def _start_after_success(next_action, stage: str):
    """仅在前一启动阶段成功时继续，否则关闭整个 launch。"""

    def _on_exit(event, _context):
        if event.returncode == 0:
            return [] if next_action is None else [next_action]
        reason = f"{stage} 失败（退出码 {event.returncode}），停止整个仿真"
        return [LogInfo(msg=reason), Shutdown(reason=reason)]

    return _on_exit


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare("humanoid_gazebo")
    model = PathJoinSubstitution([package_share, "urdf", "humanoid.urdf.xacro"])
    controllers = PathJoinSubstitution(
        [package_share, "config", "controllers.yaml"]
    )
    mechanics = PathJoinSubstitution([package_share, "config", "mechanics.yaml"])
    motors = PathJoinSubstitution([package_share, "config", "motors.yaml"])
    world = PathJoinSubstitution([package_share, "worlds", "humanoid_world.sdf"])

    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                model,
                " controllers_file:=",
                controllers,
                " mechanics_file:=",
                mechanics,
                " motor_config_file:=",
                motors,
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
            "gz_args": [
                PythonExpression(
                    [
                        "'-s ' if '",
                        LaunchConfiguration("gui"),
                        "'.lower() == 'false' else ''",
                    ]
                ),
                PythonExpression(
                    [
                        "'--render-engine '",
                        " + '",
                        LaunchConfiguration("render_engine"),
                        "' + ' '",
                    ]
                ),
                "-r -v 2 ",
                world,
            ]
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
        output="screen",
        arguments=["-name", "humanoid", "-topic", "robot_description"],
    )
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
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
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("render_engine", default_value="ogre"),
            gazebo,
            robot_state_publisher,
            clock_bridge,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_robot,
                    on_exit=_start_after_success(
                        joint_state_spawner, "创建 humanoid 模型"
                    ),
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
    )
