import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
from launch import LaunchContext
from launch.actions import (
    ExecuteProcess,
    GroupAction,
    RegisterEventHandler,
    Shutdown,
)
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_ROOT / "launch/gazebo.launch.py"
EXPECTED_ARGUMENTS = {
    "initial_positions",
    "world_file",
    "world_name",
    "gz_verbosity",
    "transport_mode",
    "arm_port",
    "baud_rate",
    "allow_external_canfd",
    "fault_latching_enabled",
    "motor_config_file",
    "robot_namespace",
    "joint_states_topic",
    "effort_commands_topic",
    "diagnostics_topic",
    "drive_state_topic",
    "control_gate_topic",
    "reset_fault_service",
    "controller_switch_timeout",
    "arm_joint_damping",
    "wrist_joint_damping",
    "self_collision_enabled",
}


def load_launch_module():
    spec = importlib.util.spec_from_file_location("robot_arm_gazebo_launch", LAUNCH_FILE)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def build_runtime(module):
    context = LaunchContext()
    context.launch_configurations.update(module.LAUNCH_DEFAULTS)
    context.launch_configurations["world_file"] = str(
        PACKAGE_ROOT / "worlds/ur5e_world.sdf"
    )
    context.launch_configurations["motor_config_file"] = str(
        PACKAGE_ROOT / "config/motors.yaml"
    )
    return context, module._build_runtime(context)


def target_action(handler: RegisterEventHandler):
    return handler.event_handler._OnActionEventBase__action_matcher


def actions_on_exit(handler: RegisterEventHandler, returncode: int):
    callback = handler.event_handler._OnActionEventBase__on_event
    return callback(SimpleNamespace(returncode=returncode), None)


def resolve_parameters(node: Node, context: LaunchContext) -> dict[str, object]:
    resolved = {}
    for name_parts, value in node._Node__parameters[0].items():
        name = perform_substitutions(context, list(name_parts))
        if isinstance(value, ParameterValue):
            resolved[name] = value.evaluate(context)
        else:
            resolved[name] = perform_substitutions(context, list(value))
    return resolved


def test_gazebo_launch_uses_robot_arm_namespace_and_fail_fast_chain() -> None:
    module = load_launch_module()
    context, actions = build_runtime(module)
    spawn_robot = next(
        action
        for action in actions
        if isinstance(action, Node) and action.node_executable == "create"
    )
    assert spawn_robot._Node__node_namespace.perform(context) == "robot_arm"
    assert spawn_robot._Node__arguments == [
        "-name",
        "ur5e",
        "-topic",
        "robot_description",
    ]

    group = next(action for action in actions if isinstance(action, GroupAction))
    pushed_namespace = group._GroupAction__actions[0]
    assert perform_substitutions(context, pushed_namespace.namespace) == "robot_arm"

    handlers = [action for action in actions if isinstance(action, RegisterEventHandler)]
    staged_handlers = [
        handler
        for handler in handlers
        if getattr(target_action(handler), "node_executable", None)
        not in {
            "robot_arm_motor_sim_node",
            "robot_arm_brake_coordinator_node",
        }
    ]
    assert len(staged_handlers) == 5
    assert all(
        any(isinstance(action, Shutdown) for action in actions_on_exit(handler, 1))
        for handler in staged_handlers
    )


def test_gazebo_launch_declares_documented_backend_arguments() -> None:
    module = load_launch_module()
    launch_description = module.generate_launch_description()
    declarations = {
        entity.name: entity
        for entity in launch_description.entities
        if entity.__class__.__name__ == "DeclareLaunchArgument"
    }
    assert declarations.keys() == EXPECTED_ARGUMENTS
    assert all(declaration.description for declaration in declarations.values())


def test_gazebo_launch_has_expected_connection_defaults() -> None:
    module = load_launch_module()
    assert module.LAUNCH_DEFAULTS["transport_mode"] in {
        "usb_serial",
        "serial_slcan",
        "socket_can",
    }
    assert module.LAUNCH_DEFAULTS["arm_port"]
    assert module.LAUNCH_DEFAULTS["baud_rate"] == "3000000"
    assert module.LAUNCH_DEFAULTS["allow_external_canfd"] in {"true", "false"}
    assert module.LAUNCH_DEFAULTS["fault_latching_enabled"] in {"true", "false"}
    assert module.LAUNCH_DEFAULTS["self_collision_enabled"] == "true"


def test_transport_mode_rejects_unknown_value() -> None:
    module = load_launch_module()
    with pytest.raises(ValueError, match="transport_mode"):
        module.validate_transport_mode("automatic")


def test_paused_world_initializes_controllers_before_backend_nodes_start() -> None:
    module = load_launch_module()
    context, actions = build_runtime(module)
    handlers = [action for action in actions if isinstance(action, RegisterEventHandler)]
    spawner_handlers = [
        handler
        for handler in handlers
        if isinstance(target_action(handler), Node)
        and target_action(handler).node_executable == "spawner"
    ]
    assert len(spawner_handlers) == 3
    assert all(
        "--inactive" in target_action(handler)._Node__arguments
        for handler in spawner_handlers
    )

    gazebo = next(
        action
        for action in actions
        if action.__class__.__name__ == "IncludeLaunchDescription"
    )
    launch_arguments = dict(
        gazebo._IncludeLaunchDescription__launch_arguments
    )
    gz_args = "".join(
        part if isinstance(part, str) else part.perform(context)
        for part in launch_arguments["gz_args"]
    )
    assert "-r" not in gz_args.split()

    brake_handler = next(
        handler
        for handler in spawner_handlers
        if "brake_controller" in target_action(handler)._Node__arguments
    )
    initialization = actions_on_exit(brake_handler, returncode=0)[0]
    assert isinstance(initialization, ExecuteProcess)
    # Humble 的 ros2controlcli 未把显式 timeout 转为 float，会在 Duration 中
    # 重复字符串并崩溃；启动阶段使用其数值型默认值，运行期协调器仍使用 launch 参数。
    initialization_script = perform_substitutions(context, initialization.cmd[2])
    assert "--switch-timeout" not in initialization_script
    assert "joint_state_broadcaster brake_controller" in initialization_script
    assert "multi_step: 1" in initialization_script
    assert initialization_script.index("multi_step: 1") < initialization_script.index(
        "pause: false"
    )

    activation_handler = next(
        handler
        for handler in handlers
        if target_action(handler) is initialization
    )

    started = actions_on_exit(activation_handler, returncode=0)
    assert len(started) == 1
    backend_group = started[0]
    assert isinstance(backend_group, GroupAction)
    backend_nodes = [
        action
        for action in backend_group._GroupAction__actions
        if isinstance(action, Node)
    ]
    assert len(backend_nodes) == 2
    motor_node = next(
        node for node in backend_nodes if node.node_executable == "robot_arm_motor_sim_node"
    )
    coordinator = next(
        node
        for node in backend_nodes
        if node.node_executable == "robot_arm_brake_coordinator_node"
    )
    assert motor_node.node_package == "robot_arm_gazebo"
    assert motor_node._Node__node_namespace.perform(context) == "robot_arm"
    assert coordinator._Node__node_namespace.perform(context) == "robot_arm"

    parameters = resolve_parameters(motor_node, context)
    assert parameters["transport_mode"] == module.LAUNCH_DEFAULTS["transport_mode"]
    assert parameters["arm_port"] == module.LAUNCH_DEFAULTS["arm_port"]
    assert parameters["config_file"].endswith("/config/motors.yaml")
    assert parameters["baud_rate"] == 3000000
    assert parameters["drive_state_topic"] == "drive_state"
    assert parameters["control_gate_topic"] == "control_gate"
    assert parameters["reset_fault_service"] == "reset_fault"

    coordinator_parameters = resolve_parameters(coordinator, context)
    assert coordinator_parameters["controller_manager_name"] == (
        "/robot_arm/controller_manager"
    )
    assert coordinator_parameters["brake_controller_name"] == "brake_controller"
    assert coordinator_parameters["controller_switch_timeout"] == 2.0

    remappings = motor_node._Node__remappings
    assert [
        perform_substitutions(context, list(target)) for _, target in remappings
    ] == [
        "joint_states",
        "effort_controller/commands",
        "diagnostics",
    ]


def test_motor_backend_exit_shuts_down_unified_launch() -> None:
    module = load_launch_module()
    _, actions = build_runtime(module)
    handlers = [action for action in actions if isinstance(action, RegisterEventHandler)]
    motor_handler = next(
        handler
        for handler in handlers
        if getattr(target_action(handler), "node_executable", None)
        == "robot_arm_motor_sim_node"
    )

    exit_actions = actions_on_exit(motor_handler, returncode=1)
    assert any(isinstance(action, Shutdown) for action in exit_actions)


def test_brake_coordinator_exit_shuts_down_unified_launch() -> None:
    module = load_launch_module()
    _, actions = build_runtime(module)
    handlers = [action for action in actions if isinstance(action, RegisterEventHandler)]
    coordinator_handler = next(
        handler
        for handler in handlers
        if getattr(target_action(handler), "node_executable", None)
        == "robot_arm_brake_coordinator_node"
    )

    exit_actions = actions_on_exit(coordinator_handler, returncode=1)
    assert any(isinstance(action, Shutdown) for action in exit_actions)


def test_runtime_overrides_namespace_transport_and_topics_together() -> None:
    module = load_launch_module()
    context = LaunchContext()
    context.launch_configurations.update(module.LAUNCH_DEFAULTS)
    context.launch_configurations.update(
        {
            "robot_namespace": "test_arm",
            "transport_mode": "socket_can",
            "arm_port": "vcan0",
            "baud_rate": "115200",
            "allow_external_canfd": "true",
            "fault_latching_enabled": "false",
            "motor_config_file": "/tmp/test-motors.yaml",
            "joint_states_topic": "/plant/joints",
            "effort_commands_topic": "/plant/efforts",
            "diagnostics_topic": "/plant/diagnostics",
            "drive_state_topic": "/plant/drive_state",
            "control_gate_topic": "/plant/control_gate",
            "reset_fault_service": "/plant/reset_fault",
            "controller_switch_timeout": "3.5",
            "self_collision_enabled": "false",
        }
    )
    actions = module._build_runtime(context)
    handlers = [action for action in actions if isinstance(action, RegisterEventHandler)]
    brake_handler = next(
        handler
        for handler in handlers
        if isinstance(target_action(handler), Node)
        and "brake_controller" in target_action(handler)._Node__arguments
    )
    initialization = actions_on_exit(brake_handler, returncode=0)[0]
    activation_handler = next(
        handler
        for handler in handlers
        if target_action(handler) is initialization
    )
    backend_group = actions_on_exit(activation_handler, returncode=0)[0]
    backend_nodes = [
        action
        for action in backend_group._GroupAction__actions
        if isinstance(action, Node)
    ]
    motor_node = next(
        node for node in backend_nodes if node.node_executable == "robot_arm_motor_sim_node"
    )
    coordinator = next(
        node
        for node in backend_nodes
        if node.node_executable == "robot_arm_brake_coordinator_node"
    )

    parameters = resolve_parameters(motor_node, context)
    assert parameters == {
        "transport_mode": "socket_can",
        "arm_port": "vcan0",
        "baud_rate": 115200,
        "config_file": "/tmp/test-motors.yaml",
        "allow_external_canfd": True,
        "fault_latching_enabled": False,
        "drive_state_topic": "/plant/drive_state",
        "control_gate_topic": "/plant/control_gate",
        "reset_fault_service": "/plant/reset_fault",
    }
    assert motor_node._Node__node_namespace.perform(context) == "test_arm"
    assert [
        (
            perform_substitutions(context, list(source)),
            perform_substitutions(context, list(target)),
        )
        for source, target in motor_node._Node__remappings
    ] == [
        ("joint_states", "/plant/joints"),
        ("effort_controller/commands", "/plant/efforts"),
        ("diagnostics", "/plant/diagnostics"),
    ]

    coordinator_parameters = resolve_parameters(coordinator, context)
    assert coordinator_parameters == {
        "drive_state_topic": "/plant/drive_state",
        "control_gate_topic": "/plant/control_gate",
        "diagnostics_topic": "/plant/diagnostics",
        "controller_name": "effort_controller",
        "brake_controller_name": "brake_controller",
        "controller_manager_name": "/test_arm/controller_manager",
        "controller_switch_timeout": 3.5,
    }

    spawners = [
        target_action(handler)
        for handler in handlers
        if getattr(target_action(handler), "node_executable", None) == "spawner"
    ]
    assert len(spawners) == 3
    for spawner in spawners:
        manager_index = spawner._Node__arguments.index("--controller-manager") + 1
        assert spawner._Node__arguments[manager_index].perform(context) == (
            "/test_arm/controller_manager"
        )


@pytest.mark.parametrize(
    "raw",
    (
        "0,0,0,0,0",
        "0,0,0,0,0,0,0",
        "0,nan,0,0,0,0",
        "0,inf,0,0,0,0",
        "0,nope,0,0,0,0",
    ),
)
def test_initial_positions_reject_malformed_vectors(raw: str) -> None:
    module = load_launch_module()
    with pytest.raises(ValueError):
        module.parse_initial_positions(raw, [(-6.28, 6.28)] * 6)


def test_initial_positions_reject_values_outside_hard_limits() -> None:
    module = load_launch_module()
    with pytest.raises(ValueError, match="hard limits"):
        module.parse_initial_positions(
            "0,0,3.2,0,0,0",
            [(-6.28, 6.28)] * 2 + [(-3.14, 3.14)] + [(-6.28, 6.28)] * 3,
        )


def test_initial_positions_accept_six_finite_radians() -> None:
    module = load_launch_module()
    assert module.parse_initial_positions(
        "0, -1.57, 0, -1.57, 0, 0", [(-6.28, 6.28)] * 6
    ) == [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
