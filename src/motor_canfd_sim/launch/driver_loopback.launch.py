"""通用 C++ 驱动 -> vcan -> C++ 电机模拟器 -> Gazebo 全回环启动文件。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    left_interface = LaunchConfiguration("left_interface")
    right_interface = LaunchConfiguration("right_interface")
    gui = LaunchConfiguration("gui")
    render_engine = LaunchConfiguration("render_engine")

    # 真机硬件接口和电机模拟器共用这一份 YAML，确保 bus/motor/joint 映射一致。
    motor_config = PathJoinSubstitution(
        [FindPackageShare("motor_hardware_interface"), "config", "motor_canfd.yaml"]
    )

    # Gazebo 只负责机器人动力学；start_bridge=false 避免同时启动旧 Python CAN 桥。
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("upper_body_canfd_gazebo"), "launch", "sim.launch.py"]
            )
        ),
        launch_arguments={
            "left_interface": left_interface,
            "right_interface": right_interface,
            "start_bridge": "false",
            "gui": gui,
            "render_engine": render_engine,
        }.items(),
    )

    # C++ 模拟器扮演“电机侧”。这里把逻辑总线重定向到 vcan0/vcan1。
    cpp_bridge = Node(
        package="motor_canfd_sim",
        executable="motor_canfd_sim_node",
        name="motor_canfd_sim",
        output="screen",
        parameters=[
            PathJoinSubstitution([FindPackageShare("motor_canfd_sim"), "config", "sim.yaml"]),
            {
                "motor_config_file": motor_config,
                "bus_override.left_arm": left_interface,
                "bus_override.right_arm": right_interface,
            },
        ],
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
                motor_config,
            ]
        ),
        value_type=str,
    )
    controller_config = PathJoinSubstitution(
        [FindPackageShare("motor_canfd_sim"), "config", "loopback_controllers.yaml"]
    )
    # 新驱动的 controller_manager 放在 /driver 命名空间，避免与 Gazebo 自己的
    # /controller_manager 重名。二者通过 CAN-FD/vcan 和关节状态形成完整回环。
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

    # 启动顺序：Gazebo/C++ 模拟器 -> 真机驱动 manager -> state broadcaster
    # -> position controller。后一级等前一级就绪，可减少 controller 启动竞态。
    return LaunchDescription(
        [
            DeclareLaunchArgument("left_interface", default_value="vcan0"),
            DeclareLaunchArgument("right_interface", default_value="vcan1"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("render_engine", default_value=""),
            gazebo,
            RegisterEventHandler(OnProcessStart(target_action=cpp_bridge, on_start=[driver_manager])),
            RegisterEventHandler(
                OnProcessStart(target_action=driver_manager, on_start=[joint_state_broadcaster])
            ),
            RegisterEventHandler(
                OnProcessExit(target_action=joint_state_broadcaster, on_exit=[position_controller])
            ),
            cpp_bridge,
        ]
    )
