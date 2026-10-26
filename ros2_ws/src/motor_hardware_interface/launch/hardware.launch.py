"""CAN-FD 真机 bring-up 入口（motor_hardware_interface 自足，不依赖仿真栈）。

启动 ros2_control（motor_hardware_interface/CanFdSystem）、robot_state_publisher、
joint_state_broadcaster 与 position_controller，直接驱动配置中的 CAN 总线。
默认为 serial_slcan 双总线（/dev/ttyACM0、/dev/ttyACM1），也可切 SocketCAN。

模型与控制器拓扑全部按运行时生效的 motor_config_file 生成：position controller
的 joints、URDF ros2_control 关节清单（control_joints）与 bus_overrides 均由
实际配置驱动（OpaqueFunction 延迟求值），因此覆盖 motor_config_file 时自动一致。
interface_<逻辑总线名> 参数按默认配置声明，运行时把每个逻辑总线覆盖为对应
设备路径（socketcan 总线为 can0 等接口名，serial_slcan 总线为 /dev/ttyACM0 等）。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from motor_hardware_interface.helpers import (
    motor_joints_from_config,
    read_buses,
    write_position_joints_params_file,
)


def _default_config_path() -> Path:
    return (
        Path(get_package_share_directory("motor_hardware_interface"))
        / "config"
        / "motor_driver.yaml"
    )


def generate_launch_description() -> LaunchDescription:
    share = FindPackageShare("motor_hardware_interface")
    default_model_xacro = PathJoinSubstitution(
        [share, "urdf", "upper_body.urdf.xacro"]
    )
    default_motor_config = PathJoinSubstitution(
        [share, "config", "motor_driver.yaml"]
    )

    # interface_<总线名> 参数按默认配置声明（默认值即默认设备路径）。
    # 覆盖 motor_config_file 时总线名集合应包含在默认配置中；未知总线会在
    # launch_control_stack 中报 LaunchError。
    default_buses = read_buses(_default_config_path())

    return LaunchDescription([
        DeclareLaunchArgument(
            "model_xacro",
            default_value=default_model_xacro,
            description="渲染 CAN-FD 后端 ros2_control 块的机器人模型 xacro 路径",
        ),
        *[
            DeclareLaunchArgument(f"interface_{name}", default_value=device)
            for name, device in default_buses
        ],
        DeclareLaunchArgument(
            "motor_config_file",
            default_value=default_motor_config,
            description="真机电机拓扑配置；默认 motor_hardware_interface/config/motor_driver.yaml",
        ),
        DeclareLaunchArgument(
            "shutdown_on_manager_exit",
            default_value="false",
            description="controller_manager 退出时是否关闭整个 launch",
        ),
        DeclareLaunchArgument(
            "controller_manager_name",
            default_value="/controller_manager",
            description="controller_manager 完整 ROS 名称",
        ),
        OpaqueFunction(function=launch_control_stack),
    ])


def launch_control_stack(context: LaunchContext):
    # 运行时求值：后续所有生成都基于实际生效的 motor_config_file。
    config_path = perform_substitutions(
        context, [LaunchConfiguration("motor_config_file")]
    )
    if not Path(config_path).is_file():
        raise RuntimeError(f"电机拓扑配置不存在：{config_path}")

    joints = motor_joints_from_config(config_path)
    buses = read_buses(config_path)

    overrides = []
    for name, _ in buses:
        parameter = f"interface_{name}"
        if parameter not in context.launch_configurations:
            raise RuntimeError(
                f"配置 '{config_path}' 包含未声明参数的总线 '{name}'；"
                f"请保持总线集合与默认配置一致，或在 launch 中提供 {parameter}。"
            )
        overrides.append(f"{name}:{context.launch_configurations[parameter]}")
    bus_overrides = ",".join(overrides)
    control_joints = ",".join(joints)
    # 与 driver_controllers.yaml 同构的临时 joints 参数文件，随 manager 一起加载。
    position_joints_param_file = write_position_joints_params_file(joints)
    controller_manager_name = perform_substitutions(
        context, [LaunchConfiguration("controller_manager_name")]
    ).strip()
    if not controller_manager_name:
        raise RuntimeError("controller_manager_name cannot be empty")
    if not controller_manager_name.startswith("/"):
        controller_manager_name = f"/{controller_manager_name}"

    model_xacro = perform_substitutions(
        context, [LaunchConfiguration("model_xacro")]
    )
    description = ParameterValue(Command([
        FindExecutable(name="xacro"), " ",
        model_xacro,
        " bus_overrides:=", bus_overrides,
        " motor_config_file:=", config_path,
        " control_joints:=", control_joints,
    ]), value_type=str)

    share = FindPackageShare("motor_hardware_interface")
    controller_config = PathJoinSubstitution(
        [share, "config", "driver_controllers.yaml"]
    )

    manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {"robot_description": description},
            controller_config,
            position_joints_param_file,
        ],
    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": description}],
    )
    jsb = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            controller_manager_name,
        ],
    )
    position = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "position_controller",
            "--controller-manager",
            controller_manager_name,
        ],
    )

    actions = [
        manager,
        state_publisher,
        RegisterEventHandler(
            OnProcessExit(target_action=jsb, on_exit=[position])
        ),
        jsb,
    ]
    shutdown_on_manager_exit = perform_substitutions(
        context, [LaunchConfiguration("shutdown_on_manager_exit")]
    ).strip().lower() in {"1", "true", "yes", "on"}
    if shutdown_on_manager_exit:
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=manager,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(
                                reason="motor_hardware_interface controller_manager exited"
                            )
                        )
                    ],
                )
            )
        )

    return actions
