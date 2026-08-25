"""USB 仿真 launch 的公共组合与启动前安全校验。"""

from typing import Mapping

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


BUS_NAMES = ("left_arm", "right_arm", "legs")
STABLE_PORT_PREFIX = "/dev/serial/by-id/"


def _as_bool(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in ("true", "1", "yes", "on"):
        return True
    if normalized in ("false", "0", "no", "off"):
        return False
    raise ValueError(f"布尔参数必须为 true 或 false，收到：{value}")


def validate_serial_ports(
    ports: Mapping[str, str],
    transport_mode: str,
    *,
    allow_external_canfd: bool,
) -> None:
    """校验三路稳定设备名、互斥模式与 CAN-FD 显式授权。"""

    if set(ports) != set(BUS_NAMES):
        raise ValueError("ports 必须且只能包含 left_arm、right_arm、legs")
    if transport_mode not in ("usb_serial", "serial_slcan"):
        raise ValueError("transport_mode 必须为 usb_serial 或 serial_slcan")
    for bus, port in ports.items():
        if not port.startswith(STABLE_PORT_PREFIX) or port == STABLE_PORT_PREFIX:
            raise ValueError(
                f"{bus} 必须使用非空 {STABLE_PORT_PREFIX} 稳定设备路径"
            )
    if len(set(ports.values())) != len(BUS_NAMES):
        raise ValueError("three serial ports must be unique")
    if transport_mode == "serial_slcan" and not allow_external_canfd:
        raise ValueError(
            "serial_slcan 必须显式设置 allow_external_canfd:=true"
        )


def _launch_setup(context: LaunchContext, expected_mode: str):
    mode = LaunchConfiguration("transport_mode").perform(context)
    if mode != expected_mode:
        raise ValueError(
            f"该入口固定使用 {expected_mode}，不能改为 {mode}"
        )
    ports = {
        bus: LaunchConfiguration(f"{bus}_port").perform(context)
        for bus in BUS_NAMES
    }
    allow_external_canfd = _as_bool(
        LaunchConfiguration("allow_external_canfd").perform(context)
    )
    validate_serial_ports(
        ports,
        mode,
        allow_external_canfd=allow_external_canfd,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("humanoid_gazebo"), "launch", "gazebo.launch.py"]
            )
        ),
        launch_arguments={
            "gui": LaunchConfiguration("gui"),
            "render_engine": LaunchConfiguration("render_engine"),
        }.items(),
    )
    simulator = Node(
        package="humanoid_gazebo",
        executable="humanoid_motor_sim_node",
        name="humanoid_motor_sim",
        output="screen",
        parameters=[
            {
                "transport_mode": mode,
                "left_arm_port": ports["left_arm"],
                "right_arm_port": ports["right_arm"],
                "legs_port": ports["legs"],
                "baud_rate": ParameterValue(
                    LaunchConfiguration("baud_rate"), value_type=int
                ),
                "config_file": LaunchConfiguration("config_file"),
                "allow_external_canfd": allow_external_canfd,
                "use_sim_time": True,
            }
        ],
    )
    shutdown = RegisterEventHandler(
        OnProcessExit(
            target_action=simulator,
            on_exit=[
                Shutdown(
                    reason="电机仿真节点退出或串口断开，停止整个 Gazebo 仿真"
                )
            ],
        )
    )
    return [gazebo, shutdown, simulator]


def serial_sim_launch_description(default_mode: str) -> LaunchDescription:
    """生成模式固定、端口启动前校验的完整仿真入口。"""

    if default_mode not in ("usb_serial", "serial_slcan"):
        raise ValueError(f"不支持的 transport mode：{default_mode}")
    return LaunchDescription(
        [
            DeclareLaunchArgument("transport_mode", default_value=default_mode),
            DeclareLaunchArgument("left_arm_port", default_value=""),
            DeclareLaunchArgument("right_arm_port", default_value=""),
            DeclareLaunchArgument("legs_port", default_value=""),
            DeclareLaunchArgument("baud_rate", default_value="3000000"),
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("humanoid_gazebo"), "config", "motors.yaml"]
                ),
            ),
            DeclareLaunchArgument("allow_external_canfd", default_value="false"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("render_engine", default_value="ogre"),
            OpaqueFunction(
                function=_launch_setup,
                kwargs={"expected_mode": default_mode},
            ),
        ]
    )
