"""启动命名空间隔离的 UR5e Gazebo effort 仿真。"""

import math

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
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

# 集中默认值便于长期调试；调用方仍可用 launch 参数逐项临时覆盖。
LAUNCH_DEFAULTS = {
    "initial_positions": "0,-1.57,0,-1.57,0,0",
    "world_file": "ur5e_world.sdf",
    "world_name": "ur5e_world",
    "gz_verbosity": "2",
    "transport_mode": "socket_can",
    "arm_port": "vcan0",
    "baud_rate": "3000000",
    "allow_external_canfd": "true",
    "fault_latching_enabled": "false",
    "motor_config_file": "motors.yaml",
    "robot_namespace": "robot_arm",
    "joint_states_topic": "joint_states",
    "effort_commands_topic": "effort_controller/commands",
    "diagnostics_topic": "diagnostics",
    "drive_state_topic": "drive_state",
    "control_gate_topic": "control_gate",
    "reset_fault_service": "reset_fault",
    "controller_switch_timeout": "2.0",
    "arm_joint_damping": "20.0",
    "wrist_joint_damping": "2.0",
    "self_collision_enabled": "true",
}


def parse_initial_positions(raw: str, limits=HARD_LIMITS) -> list[float]:
    """保留 launch 模块的公开校验入口，便于静态契约测试。"""

    return parse_initial_positions_impl(raw, limits)


def validate_transport_mode(value: str) -> str:
    """在启动任何进程前拒绝未知传输模式。"""

    supported = {"usb_serial", "serial_slcan", "socket_can"}
    if value not in supported:
        raise ValueError(
            "transport_mode must be usb_serial, serial_slcan or socket_can"
        )
    return value


def _start_after_success(next_action, stage: str):
    def _on_exit(event, _context):
        if event.returncode == 0:
            return [] if next_action is None else [next_action]
        reason = f"{stage} 失败（退出码 {event.returncode}），停止 UR5e 仿真"
        return [LogInfo(msg=reason), Shutdown(reason=reason)]

    return _on_exit


def _shutdown_on_exit(stage: str):
    def _on_exit(event, _context):
        reason = f"{stage} 已退出（退出码 {event.returncode}），停止 UR5e 仿真后端"
        return [LogInfo(msg=reason), Shutdown(reason=reason)]

    return _on_exit


def _build_runtime(context):
    # OpaqueFunction 先完成严格解析；异常会在任何 Gazebo/ROS 进程创建前终止 launch。
    values = parse_initial_positions(
        LaunchConfiguration("initial_positions").perform(context)
    )
    validate_transport_mode(LaunchConfiguration("transport_mode").perform(context))
    package_share = FindPackageShare("robot_arm_gazebo")
    model = PathJoinSubstitution([package_share, "urdf", "ur5e.urdf.xacro"])
    controllers = PathJoinSubstitution([package_share, "config", "controllers.yaml"])
    robot_namespace = LaunchConfiguration("robot_namespace")
    controller_manager = PythonExpression(
        ["'/' + '", robot_namespace, "'.strip('/') + '/controller_manager'"]
    )
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
                " robot_namespace:=",
                robot_namespace,
                " arm_joint_damping:=",
                LaunchConfiguration("arm_joint_damping"),
                " wrist_joint_damping:=",
                LaunchConfiguration("wrist_joint_damping"),
                " self_collision_enabled:=",
                LaunchConfiguration("self_collision_enabled"),
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
            "gz_args": [
                "-v ",
                LaunchConfiguration("gz_verbosity"),
                " ",
                LaunchConfiguration("world_file"),
            ],
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
        namespace=robot_namespace,
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
            controller_manager,
            "--controller-manager-timeout",
            "60",
            "--inactive",
        ],
    )
    effort_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "effort_controller",
            "--controller-manager",
            controller_manager,
            "--controller-manager-timeout",
            "60",
            "--inactive",
        ],
    )
    brake_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "brake_controller",
            "--controller-manager",
            controller_manager,
            "--controller-manager-timeout",
            "60",
            "--inactive",
        ],
    )
    motor_sim = Node(
        package="robot_arm_gazebo",
        executable="robot_arm_motor_sim_node",
        namespace=robot_namespace,
        output="screen",
        parameters=[
            {
                "transport_mode": LaunchConfiguration("transport_mode"),
                "arm_port": LaunchConfiguration("arm_port"),
                "baud_rate": ParameterValue(
                    LaunchConfiguration("baud_rate"), value_type=int
                ),
                "config_file": LaunchConfiguration("motor_config_file"),
                "allow_external_canfd": ParameterValue(
                    LaunchConfiguration("allow_external_canfd"), value_type=bool
                ),
                "fault_latching_enabled": ParameterValue(
                    LaunchConfiguration("fault_latching_enabled"), value_type=bool
                ),
                "drive_state_topic": LaunchConfiguration("drive_state_topic"),
                "control_gate_topic": LaunchConfiguration("control_gate_topic"),
                "reset_fault_service": LaunchConfiguration(
                    "reset_fault_service"
                ),
            }
        ],
        remappings=[
            ("joint_states", LaunchConfiguration("joint_states_topic")),
            (
                "effort_controller/commands",
                LaunchConfiguration("effort_commands_topic"),
            ),
            ("diagnostics", LaunchConfiguration("diagnostics_topic")),
        ],
    )
    brake_coordinator = Node(
        package="robot_arm_gazebo",
        executable="robot_arm_brake_coordinator_node",
        namespace=robot_namespace,
        output="screen",
        parameters=[
            {
                "drive_state_topic": LaunchConfiguration("drive_state_topic"),
                "control_gate_topic": LaunchConfiguration("control_gate_topic"),
                "diagnostics_topic": LaunchConfiguration("diagnostics_topic"),
                "controller_name": ParameterValue(
                    "effort_controller", value_type=str
                ),
                "brake_controller_name": ParameterValue(
                    "brake_controller", value_type=str
                ),
                "controller_manager_name": controller_manager,
                "controller_switch_timeout": ParameterValue(
                    LaunchConfiguration("controller_switch_timeout"),
                    value_type=float,
                ),
            }
        ],
    )
    backend_nodes = GroupAction([motor_sim, brake_coordinator])
    world_control_service = PythonExpression(
        ["'/world/' + '", LaunchConfiguration("world_name"), "' + '/control'"]
    )
    initialize_active_controllers = ExecuteProcess(
        cmd=[
            FindExecutable(name="bash"),
            "-c",
            [
                "set -e; ",
                "(",
                "while true; do ",
                FindExecutable(name="ign"),
                " service -s ",
                world_control_service,
                " --reqtype ignition.msgs.WorldControl",
                " --reptype ignition.msgs.Boolean --timeout 2000",
                " --req 'multi_step: 1' >/dev/null; sleep 0.05; done",
                ") & step_pid=$!; ",
                "trap 'kill $step_pid 2>/dev/null || true' EXIT; ",
                FindExecutable(name="ros2"),
                " control switch_controllers --activate ",
                "joint_state_broadcaster brake_controller ",
                "--strict --activate-asap --controller-manager ",
                controller_manager,
                "; ",
                "kill $step_pid 2>/dev/null || true; wait $step_pid 2>/dev/null || true; ",
                "trap - EXIT; ",
                FindExecutable(name="ign"),
                " service -s ",
                world_control_service,
                " --reqtype ignition.msgs.WorldControl",
                " --reptype ignition.msgs.Boolean --timeout 2000",
                " --req 'pause: false'",
            ],
        ],
        output="screen",
    )
    return [
        gazebo,
        GroupAction([PushRosNamespace(robot_namespace), robot_state_publisher]),
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
                on_exit=_start_after_success(
                    brake_spawner, "配置 inactive effort_controller"
                ),
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=brake_spawner,
                on_exit=_start_after_success(
                    initialize_active_controllers,
                    "配置 inactive brake_controller",
                ),
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=initialize_active_controllers,
                on_exit=_start_after_success(
                    backend_nodes, "激活初始抱闸并恢复 Gazebo 物理仿真"
                ),
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=motor_sim,
                on_exit=_shutdown_on_exit("电机仿真后端"),
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=brake_coordinator,
                on_exit=_shutdown_on_exit("仿真抱闸协调器"),
            )
        ),
        spawn_robot,
    ]


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare("robot_arm_gazebo")
    return LaunchDescription(
        [
            # 六轴仿真初始关节角，严格使用 6 个逗号分隔的有限弧度值。
            DeclareLaunchArgument(
                "initial_positions",
                default_value=LAUNCH_DEFAULTS["initial_positions"],
                description="六轴初始关节角，格式为 6 个逗号分隔的弧度值。",
            ),
            # Gazebo world 文件；默认使用包内 UR5e 空场景，可传绝对路径覆盖。
            DeclareLaunchArgument(
                "world_file",
                default_value=PathJoinSubstitution(
                    [package_share, "worlds", "ur5e_world.sdf"]
                ),
                description="Gazebo world 文件路径。",
            ),
            # world 的 SDF 名称，用于启动编排阶段恢复暂停的物理仿真。
            DeclareLaunchArgument(
                "world_name",
                default_value=LAUNCH_DEFAULTS["world_name"],
                description="Gazebo world 名称，必须与 SDF 中的 world name 一致。",
            ),
            # Gazebo 日志详细度，数值越大输出越详细。
            DeclareLaunchArgument(
                "gz_verbosity",
                default_value=LAUNCH_DEFAULTS["gz_verbosity"],
                description="Gazebo 日志详细度。",
            ),
            # 外部电机协议端点的传输方式；三种模式互斥。
            DeclareLaunchArgument(
                "transport_mode",
                default_value=LAUNCH_DEFAULTS["transport_mode"],
                description=(
                    "电机协议传输方式：usb_serial、serial_slcan 或 socket_can。"
                ),
            ),
            # arm 总线端点；串口模式填 tty 路径，SocketCAN 模式填接口名。
            DeclareLaunchArgument(
                "arm_port",
                default_value=LAUNCH_DEFAULTS["arm_port"],
                description=(
                    "arm 总线端点：串口路径（如 /dev/ttyUSB0）或 SocketCAN 接口名。"
                ),
            ),
            # 串口波特率；socket_can 模式不使用该值。
            DeclareLaunchArgument(
                "baud_rate",
                default_value=LAUNCH_DEFAULTS["baud_rate"],
                description="串口波特率；socket_can 模式忽略。",
            ),
            # 外部 CAN-FD 授权；serial_slcan 必须显式设为 true。
            DeclareLaunchArgument(
                "allow_external_canfd",
                default_value=LAUNCH_DEFAULTS["allow_external_canfd"],
                description="允许外部 CAN-FD；serial_slcan 模式必须设为 true。",
            ),
            # 故障锁存安全开关；false 仍会抱闸，但可通过全轴 disable 后重新使能恢复。
            DeclareLaunchArgument(
                "fault_latching_enabled",
                default_value=LAUNCH_DEFAULTS["fault_latching_enabled"],
                description=(
                    "启用故障锁存；false 仍清零抱闸，但允许全轴 disable 后恢复。"
                ),
            ),
            # 六轴协议、ID、限位、传动和 plant 参数的唯一配置源。
            DeclareLaunchArgument(
                "motor_config_file",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "motors.yaml"]
                ),
                description="电机仿真 YAML 配置文件路径。",
            ),
            # 所有机器人 ROS 资源的 namespace，不包含前导斜杠。
            DeclareLaunchArgument(
                "robot_namespace",
                default_value=LAUNCH_DEFAULTS["robot_namespace"],
                description="机器人 ROS namespace，默认 robot_arm。",
            ),
            # 电机节点订阅的关节状态 topic，可使用相对或绝对名称。
            DeclareLaunchArgument(
                "joint_states_topic",
                default_value=LAUNCH_DEFAULTS["joint_states_topic"],
                description="namespace 内的关节状态 topic，亦可传绝对名称。",
            ),
            # 电机节点发布的六轴 effort 命令 topic。
            DeclareLaunchArgument(
                "effort_commands_topic",
                default_value=LAUNCH_DEFAULTS["effort_commands_topic"],
                description="namespace 内的 effort controller 命令 topic。",
            ),
            # 电机节点发布连接和锁存状态的 diagnostics topic。
            DeclareLaunchArgument(
                "diagnostics_topic",
                default_value=LAUNCH_DEFAULTS["diagnostics_topic"],
                description="namespace 内的 diagnostics topic。",
            ),
            # motor node 发布、抱闸协调器订阅的五态驱动生命周期 topic。
            DeclareLaunchArgument(
                "drive_state_topic",
                default_value=LAUNCH_DEFAULTS["drive_state_topic"],
                description="namespace 内的驱动生命周期状态 topic。",
            ),
            # 协调器确认 effort controller 已就绪后打开的控制 gate topic。
            DeclareLaunchArgument(
                "control_gate_topic",
                default_value=LAUNCH_DEFAULTS["control_gate_topic"],
                description="namespace 内的 controller 就绪确认 gate topic。",
            ),
            # 锁存故障的显式复位服务；复位前必须全轴 disable。
            DeclareLaunchArgument(
                "reset_fault_service",
                default_value=LAUNCH_DEFAULTS["reset_fault_service"],
                description="namespace 内的锁存故障显式复位服务。",
            ),
            # Controller Manager STRICT 切换确认超时，单位秒。
            DeclareLaunchArgument(
                "controller_switch_timeout",
                default_value=LAUNCH_DEFAULTS["controller_switch_timeout"],
                description="effort controller STRICT 切换超时，单位秒。",
            ),
            # 肩、肘三轴粘性阻尼，单位 N·m·s/rad；用于近似传动阻力并稳定仿真。
            DeclareLaunchArgument(
                "arm_joint_damping",
                default_value=LAUNCH_DEFAULTS["arm_joint_damping"],
                description="肩、肘三轴粘性阻尼，单位 N·m·s/rad。",
            ),
            # 腕部三轴粘性阻尼，单位 N·m·s/rad。
            DeclareLaunchArgument(
                "wrist_joint_damping",
                default_value=LAUNCH_DEFAULTS["wrist_joint_damping"],
                description="腕部三轴粘性阻尼，单位 N·m·s/rad。",
            ),
            # 模型级自碰撞开关；相邻 joint 连接 link 由 SDFormat 自动排除。
            DeclareLaunchArgument(
                "self_collision_enabled",
                default_value=LAUNCH_DEFAULTS["self_collision_enabled"],
                description="启用模型级自碰撞；默认 true。",
            ),
            OpaqueFunction(function=_build_runtime),
        ]
    )
