"""完整回环：gazebo.launch.py 基础 + C++ 电机模拟器 + /driver ros2_control。

模拟器（model_sim.launch.py 内的 motor_canfd_sim 节点）启动后才拉起
/driver/controller_manager，避免硬件接口激活时因模拟器未就绪收不到反馈。
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
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
from motor_canfd_sim.helpers import default_config_share_substitution


def _is_simulator(action):
    return (
        isinstance(action, Node)
        and action.node_package == "motor_canfd_sim"
        and action.node_executable == "motor_canfd_sim_node"
    )


def generate_launch_description() -> LaunchDescription:
    left_interface = LaunchConfiguration("left_interface")
    right_interface = LaunchConfiguration("right_interface")
    gui = LaunchConfiguration("gui")
    render_engine = LaunchConfiguration("render_engine")
    motor_config_file = LaunchConfiguration("motor_config_file")

    model_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("upper_body_canfd_gazebo"), "launch", "model_sim.launch.py"]
            )
        ),
        launch_arguments={
            "left_interface": left_interface,
            "right_interface": right_interface,
            "gui": gui,
            "render_engine": render_engine,
            "motor_config_file": motor_config_file,
        }.items(),
    )

    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare("upper_body_canfd_gazebo"), "urdf", "upper_body.urdf.xacro"]
                ),
                " hardware_backend:=canfd left_interface:=",
                left_interface,
                " right_interface:=",
                right_interface,
                " motor_config_file:=",
                motor_config_file,
            ]
        ),
        value_type=str,
    )
    # 真机与回环共用同一份控制器配置，区别只有 controller_manager 命名空间。
    controller_config = PathJoinSubstitution(
        [FindPackageShare("motor_hardware_interface"), "config", "driver_controllers.yaml"]
    )
    # /driver 命名空间避免与 Gazebo 自己的 /controller_manager 重名；
    # 二者通过 CAN-FD/vcan 和关节状态形成完整回环。
    driver_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="driver",
        output="screen",
        parameters=[{"robot_description": robot_description}, controller_config],
    )
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        namespace="driver",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/driver/controller_manager",
        ],
    )
    position_controller = Node(
        package="controller_manager",
        executable="spawner",
        namespace="driver",
        output="screen",
        arguments=[
            "position_controller",
            "--controller-manager",
            "/driver/controller_manager",
        ],
    )

    # 事件处理器必须先注册，再执行 model_sim（其中的模拟器节点才会触发
    # OnProcessStart(_is_simulator)），否则会漏掉模拟器启动事件。
    return LaunchDescription(
        [
            DeclareLaunchArgument("left_interface", default_value="vcan0"),
            DeclareLaunchArgument("right_interface", default_value="vcan1"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("render_engine", default_value="ogre"),
            DeclareLaunchArgument(
                "motor_config_file",
                default_value=default_config_share_substitution(),
                description="电机拓扑配置路径；默认使用仿真侧 motor_canfd_sim/config/motor_canfd.yaml",
            ),
            RegisterEventHandler(
                OnProcessStart(target_action=_is_simulator, on_start=[driver_manager])
            ),
            RegisterEventHandler(
                OnProcessStart(target_action=driver_manager, on_start=[joint_state_broadcaster])
            ),
            RegisterEventHandler(
                OnProcessExit(target_action=joint_state_broadcaster, on_exit=[position_controller])
            ),
            # 模拟器退出即终止回环：模拟器不在 = 回环无意义。启动期失败（如 vcan
            # 未创建导致模拟器启动即退出）应 fail-fast，而不是让 spawner 永久重试。
            RegisterEventHandler(
                OnProcessExit(
                    target_action=_is_simulator,
                    on_exit=[
                        Shutdown(
                            reason="motor_canfd_sim 退出，完整回环终止；"
                            "请确认 vcan 已创建（bash src/upper_body_canfd_gazebo/scripts/setup_vcan.sh）"
                        )
                    ],
                )
            ),
            model_sim,
        ]
    )
