"""Official ros2_control controllers against GenericSystem only.

This launch has no device transport arguments. The fully rendered description is
checked before creating any process; a hardware plugin other than the official
mock_components/GenericSystem is rejected. Real robots use robot.launch.py
and communicate through micro-ROS MCU.
"""

import math
from pathlib import Path, PurePosixPath
import re
import tempfile
import xml.etree.ElementTree as ET

import xacro

import yaml

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
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from motor_hardware_interface.helpers import (
    motor_joints_from_config,
)


def _positive_parameter(context, name):
    try:
        value = float(LaunchConfiguration(name).perform(context))
    except (TypeError, ValueError) as error:
        raise RuntimeError(f"{name} 必须为有限正数") from error
    if not math.isfinite(value) or value <= 0:
        raise RuntimeError(f"{name} 必须为有限正数")
    return value


def _controller_parameters(context, joints, mode, manager_name):
    """Inject the chosen controller's topology without copying a joint list.

    Keep the established position-controller parameter keys for existing
    simulation clients. Fully qualified keys additionally support
    a caller-selected controller-manager name and namespace.
    """
    if mode == "position":
        parameters = {
            "position_controller": {"ros__parameters": {"joints": list(joints)}},
            "/driver/position_controller": {"ros__parameters": {"joints": list(joints)}},
        }
        controller = "position_controller"
        plugin = "forward_command_controller/ForwardCommandController"
        controller_parameters = {"joints": list(joints), "interface_name": "position"}
    else:
        controller = "joint_trajectory_controller"
        plugin = "joint_trajectory_controller/JointTrajectoryController"
        path_tolerance = _positive_parameter(context, "trajectory_path_tolerance")
        goal_tolerance = _positive_parameter(context, "trajectory_goal_tolerance")
        constraints = {
            "goal_time": _positive_parameter(context, "trajectory_goal_time"),
            "stopped_velocity_tolerance": _positive_parameter(
                context, "trajectory_stopped_velocity_tolerance"),
        }
        constraints.update({joint: {"trajectory": path_tolerance, "goal": goal_tolerance}
                            for joint in joints})
        controller_parameters = {
            "joints": list(joints),
            "command_interfaces": ["position"],
            "state_interfaces": ["position", "velocity"],
            "allow_partial_joints_goal": False,
            "state_publish_rate": 50.0,
            "action_monitor_rate": 20.0,
            "constraints": constraints,
        }
        parameters = {}
    namespace = str(PurePosixPath(manager_name).parent)
    controller_name = (namespace.rstrip("/") + "/" + controller)
    parameters[manager_name] = {"ros__parameters": {
        "update_rate": 100,
        "joint_state_broadcaster": {"type": "joint_state_broadcaster/JointStateBroadcaster"},
        controller: {"type": plugin},
    }}
    parameters[controller_name] = {"ros__parameters": controller_parameters}
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False, encoding="utf-8") as handle:
        yaml.safe_dump(parameters, handle, allow_unicode=True, sort_keys=False)
        return controller, handle.name


def generate_launch_description() -> LaunchDescription:
    share = FindPackageShare("motor_hardware_interface")
    default_model_xacro = PathJoinSubstitution(
        [share, "config", "mock_control.urdf.xacro"]
    )
    default_motor_config = PathJoinSubstitution(
        [share, "config", "motor_driver.yaml"]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "model_xacro",
            default_value=default_model_xacro,
            description="仅允许 GenericSystem 的模拟模型 xacro 路径",
        ),
        DeclareLaunchArgument(
            "motor_config_file",
            default_value=default_motor_config,
            description="只读取关节拓扑；不会打开该配置中的 CAN/串口设备",
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
        DeclareLaunchArgument(
            "control_mode", default_value="trajectory", choices=["position", "trajectory"],
            description="模拟控制器：position 或标准 FollowJointTrajectory Action",
        ),
        DeclareLaunchArgument(
            "trajectory_path_tolerance", default_value="0.2",
            description="轨迹执行中允许的位置误差 rad（移动关节为 m），须为正数",
        ),
        DeclareLaunchArgument(
            "trajectory_goal_tolerance", default_value="0.02",
            description="终点允许的位置误差 rad（移动关节为 m），须为正数",
        ),
        DeclareLaunchArgument(
            "trajectory_goal_time", default_value="1.0",
            description="终点容差额外等待时间 s，须为正数",
        ),
        DeclareLaunchArgument(
            "trajectory_stopped_velocity_tolerance", default_value="0.01",
            description="终点静止速度容差 rad/s（移动关节为 m/s），须为正数",
        ),
        OpaqueFunction(function=launch_control_stack),
    ])


def _validate_mock_description(description, joints):
    """Reject every real/plugin backend before controller_manager can start."""
    robot = ET.fromstring(description)
    systems = robot.findall("ros2_control")
    if len(systems) != 1 or systems[0].get("type") != "system":
        raise RuntimeError("mock_control requires exactly one GenericSystem")
    hardware = systems[0].findall("hardware")
    plugins = systems[0].findall("hardware/plugin")
    if (len(hardware) != 1 or len(plugins) != 1
            or (plugins[0].text or "").strip() != "mock_components/GenericSystem"):
        raise RuntimeError(
            "mock_control only accepts mock_components/GenericSystem; "
            "real robots must use robot_bringup robot.launch.py via micro-ROS MCU")
    control_joints = [joint.get("name") for joint in systems[0].findall("joint")]
    if sorted(control_joints) != sorted(joints) or len(set(control_joints)) != len(joints):
        raise RuntimeError("mock ros2_control joints must match motor_config_file exactly")
    physical_joints = {joint.get("name") for joint in robot.findall("joint")}
    if not set(joints) <= physical_joints:
        raise RuntimeError("mock description is missing physical joints")


def launch_control_stack(context: LaunchContext):
    # 运行时求值：后续所有生成都基于实际生效的 motor_config_file。
    config_path = perform_substitutions(
        context, [LaunchConfiguration("motor_config_file")]
    )
    if not Path(config_path).is_file():
        raise RuntimeError(f"电机拓扑配置不存在：{config_path}")

    joints = motor_joints_from_config(config_path)
    control_joints = ",".join(joints)
    mode = LaunchConfiguration("control_mode").perform(context)
    if mode not in {"position", "trajectory"}:
        raise RuntimeError("control_mode 必须是 position 或 trajectory")
    controller_manager_name = perform_substitutions(
        context, [LaunchConfiguration("controller_manager_name")]
    ).strip()
    if not controller_manager_name:
        raise RuntimeError("controller_manager_name cannot be empty")
    if not controller_manager_name.startswith("/"):
        controller_manager_name = f"/{controller_manager_name}"
    if not re.fullmatch(r"(?:/[A-Za-z_][A-Za-z0-9_]*)+", controller_manager_name):
        raise RuntimeError("controller_manager_name 必须是有效的 ROS 节点完整名称")
    selected_controller, joints_param_file = _controller_parameters(
        context, joints, mode, controller_manager_name)
    manager_path = PurePosixPath(controller_manager_name)
    manager_namespace = str(manager_path.parent)

    model_xacro = perform_substitutions(
        context, [LaunchConfiguration("model_xacro")]
    )
    rendered = xacro.process_file(model_xacro, mappings={
        "motor_config_file": config_path,
        "control_joints": control_joints,
    }).toxml()
    _validate_mock_description(rendered, joints)
    description = ParameterValue(rendered, value_type=str)

    share = FindPackageShare("motor_hardware_interface")
    controller_config = PathJoinSubstitution(
        [share, "config", "driver_controllers.yaml"]
    )

    manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=manager_namespace,
        # Controller plugins create their own ROS nodes in this process.
        # A global __node remap (Node(name=...)) would rename those nodes too,
        # causing their per-controller YAML parameters to miss. Scope the
        # manager rename to its original node name instead.
        arguments=["--ros-args", "-r", f"controller_manager:__node:={manager_path.name}"],
        output="screen",
        parameters=[
            {"robot_description": description},
            controller_config,
            joints_param_file,
        ],
    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=manager_namespace,
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
    controller = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            selected_controller,
            "--controller-manager",
            controller_manager_name,
        ],
    )

    actions = [
        manager,
        state_publisher,
        RegisterEventHandler(
            OnProcessExit(target_action=jsb, on_exit=[controller])
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
