"""Validate retired hardware entry and mock-only controllers without starting nodes."""

import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.events.process import ProcessExited
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from motor_hardware_interface.helpers import motor_joints_from_config

HERE = Path(__file__).parent


def _load_launch_module(name="mock_control"):
    launch_file = HERE.parent / "launch" / f"{name}.launch.py"
    spec = importlib.util.spec_from_file_location("hardware_launch", launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _execute_launch(overrides=None):
    """执行参数声明与 OpaqueFunction，返回 (actions, context)。"""
    module = _load_launch_module()
    description = module.generate_launch_description()
    context = LaunchContext()
    opaque_function = None
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
        elif isinstance(action, OpaqueFunction):
            opaque_function = action
    assert opaque_function is not None, "launch 必须包含 OpaqueFunction"
    context.launch_configurations.update(overrides or {})
    actions = opaque_function.execute(context)
    return actions, context


def _manager_node(actions):
    for action in actions:
        if (
            isinstance(action, Node)
            and action.node_package == "controller_manager"
            and action.node_executable == "ros2_control_node"
        ):
            return action
    raise AssertionError("缺少 ros2_control_node")


def _description(manager, context):
    for parameters in vars(manager)["_Node__parameters"]:
        if not isinstance(parameters, dict):
            continue
        for key, value in parameters.items():
            if perform_substitutions(context, list(key)) == "robot_description":
                return ET.fromstring(value.evaluate(context))
    raise AssertionError("manager is missing validated robot_description")


def _joints_param_file(manager, context, controller="position_controller"):
    # manager 参数：robot_description、controller_config（ParameterFile）、joints 参数文件。
    parameters_list = vars(manager)["_Node__parameters"]
    for parameters in parameters_list:
        if isinstance(parameters, ParameterFile):
            path = parameters._ParameterFile__param_file
            if isinstance(path, list):
                path = perform_substitutions(context, path)
            data = yaml.safe_load(Path(path).read_text())
            if (
                any(key.rstrip('/').split('/')[-1] == controller
                    and "joints" in value.get("ros__parameters", {})
                    for key, value in data.items())
            ):
                return path
    raise AssertionError("缺少 joints 临时参数文件")


def _selected_controller_spawner(actions, context):
    broadcaster = next(action for action in actions
                       if isinstance(action, Node)
                       and action.node_package == 'controller_manager'
                       and action.node_executable == 'spawner')
    event = ProcessExited(action=broadcaster, name='test', cmd=[], cwd=None,
                          env=None, pid=123, returncode=0)
    for registration in actions:
        if not isinstance(registration, RegisterEventHandler):
            continue
        handler = registration.event_handler
        if handler.matches(event):
            spawned = list(handler.handle(event, context))
            return next(action for action in spawned if isinstance(action, Node))
    raise AssertionError('joint state broadcaster must spawn the selected controller')


def test_old_hardware_launch_is_retired_before_any_node_is_created():
    module = _load_launch_module("hardware")
    with pytest.raises(RuntimeError, match="managed_robot.launch.py.*micro-ROS MCU"):
        module.generate_launch_description()


def test_mock_default_has_no_device_arguments_and_only_generic_system():
    actions, context = _execute_launch()
    assert not any(name.startswith("interface_") for name in context.launch_configurations)
    assert context.launch_configurations["control_mode"] == "trajectory"
    root = _description(_manager_node(actions), context)
    assert [p.text for p in root.findall("ros2_control/hardware/plugin")] == [
        "mock_components/GenericSystem"]
    expected_joints = motor_joints_from_config(
        Path(get_package_share_directory("motor_hardware_interface"))
        / "config" / "motor_driver.yaml")
    assert [j.get("name") for j in root.findall("ros2_control/joint")] == expected_joints
    assert not root.findall("ros2_control/hardware/param")


def test_mock_override_config_only_selects_joint_topology():
    override = HERE / "fixtures" / "override_motor_config.yaml"
    actions, context = _execute_launch({
        "motor_config_file": str(override), "control_mode": "position"})
    expected_joints = motor_joints_from_config(override)
    root = _description(_manager_node(actions), context)
    assert [j.get("name") for j in root.findall("ros2_control/joint")] == expected_joints
    assert root.find("ros2_control/hardware/plugin").text == "mock_components/GenericSystem"
    data = yaml.safe_load(Path(_joints_param_file(_manager_node(actions), context)).read_text())
    assert data["position_controller"]["ros__parameters"]["joints"] == expected_joints


def test_mock_rejects_real_canfd_xacro_before_node_construction(monkeypatch):
    module = _load_launch_module()
    context = LaunchContext()
    description = module.generate_launch_description()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations["model_xacro"] = str(
        HERE.parent / "urdf" / "upper_body.urdf.xacro")
    monkeypatch.setattr(module, "Node", lambda *args, **kwargs: pytest.fail(
        "Node constructed before hardware plugin validation"))
    with pytest.raises(RuntimeError, match="only accepts.*GenericSystem"):
        module.launch_control_stack(context)


@pytest.mark.parametrize("plugin", ["motor_hardware_interface/CanFdSystem", "vendor/Driver", ""])
def test_mock_rejects_non_mock_plugins(plugin):
    module = _load_launch_module()
    description = (f'<robot><ros2_control name="test" type="system"><hardware>'
                   f'<plugin>{plugin}</plugin></hardware></ros2_control></robot>')
    with pytest.raises(RuntimeError, match="only accepts.*GenericSystem"):
        module._validate_mock_description(description, [])


def test_mock_rejects_mixed_mock_and_real_components():
    module = _load_launch_module()
    description = ('<robot><ros2_control name="mock" type="system"><hardware>'
                   '<plugin>mock_components/GenericSystem</plugin></hardware></ros2_control>'
                   '<ros2_control name="real" type="system"><hardware>'
                   '<plugin>motor_hardware_interface/CanFdSystem</plugin>'
                   '</hardware></ros2_control></robot>')
    with pytest.raises(RuntimeError, match="exactly one GenericSystem"):
        module._validate_mock_description(description, [])


def test_mock_rejects_joint_config_mismatch():
    with pytest.raises(RuntimeError, match="joints must match"):
        _execute_launch({
            'model_xacro': str(HERE / 'fixtures' / 'standard_trajectory_mock.urdf.xacro')})


def test_trajectory_mode_uses_standard_jtc_and_actual_configured_joints():
    override = HERE / 'fixtures' / 'override_motor_config.yaml'
    actions, context = _execute_launch({
        'control_mode': 'trajectory', 'motor_config_file': str(override),
        'trajectory_path_tolerance': '0.15', 'trajectory_goal_tolerance': '0.01',
        'trajectory_goal_time': '0.8', 'trajectory_stopped_velocity_tolerance': '0.03',
    })
    joints = motor_joints_from_config(override)
    manager = _manager_node(actions)
    data = yaml.safe_load(Path(_joints_param_file(
        manager, context, 'joint_trajectory_controller')).read_text())
    manager_params = data['/controller_manager']['ros__parameters']
    assert manager_params['joint_trajectory_controller']['type'] == (
        'joint_trajectory_controller/JointTrajectoryController')
    assert 'position_controller' not in manager_params
    controller = data['/joint_trajectory_controller']['ros__parameters']
    assert controller['joints'] == joints
    assert controller['command_interfaces'] == ['position']
    assert controller['state_interfaces'] == ['position', 'velocity']
    assert controller['allow_partial_joints_goal'] is False
    assert controller['constraints']['goal_time'] == .8
    assert controller['constraints']['stopped_velocity_tolerance'] == .03
    assert {name: controller['constraints'][name] for name in joints} == {
        name: {'trajectory': .15, 'goal': .01} for name in joints}
    spawner = _selected_controller_spawner(actions, context)
    assert getattr(spawner, '_Node__arguments') == [
        'joint_trajectory_controller', '--controller-manager', '/controller_manager']
    assert [j.get("name") for j in _description(manager, context).findall(
        "ros2_control/joint")] == joints


@pytest.mark.parametrize('mode', ['position', 'trajectory'])
def test_controller_manager_namespace_matches_parameters_and_feedback(mode):
    actions, context = _execute_launch({
        'control_mode': mode, 'controller_manager_name': '/arm/can_manager',
    })
    manager = _manager_node(actions)
    assert manager._Node__node_name is None
    assert manager._Node__arguments == [
        '--ros-args', '-r', 'controller_manager:__node:=can_manager']
    assert perform_substitutions(context, normalize_to_list_of_substitutions(
        manager._Node__node_namespace)) == '/arm'
    selected = 'joint_trajectory_controller' if mode == 'trajectory' else 'position_controller'
    data = yaml.safe_load(Path(_joints_param_file(manager, context, selected)).read_text())
    assert data['/arm/can_manager']['ros__parameters'][selected]['type'].endswith(
        '/JointTrajectoryController' if mode == 'trajectory' else '/ForwardCommandController')
    assert data['/arm/' + selected]['ros__parameters']['joints']
    publisher = next(action for action in actions if isinstance(action, Node)
                     and action.node_package == 'robot_state_publisher')
    assert perform_substitutions(context, normalize_to_list_of_substitutions(
        publisher._Node__node_namespace)) == '/arm'
    spawner = _selected_controller_spawner(actions, context)
    assert getattr(spawner, '_Node__arguments')[-1] == '/arm/can_manager'


@pytest.mark.parametrize('name', [
    'trajectory_path_tolerance', 'trajectory_goal_tolerance',
    'trajectory_goal_time', 'trajectory_stopped_velocity_tolerance',
])
@pytest.mark.parametrize('value', ['0', '-1', 'nan', 'inf', 'not-a-number'])
def test_trajectory_tolerances_must_be_finite_and_positive(name, value):
    with pytest.raises(RuntimeError, match=name):
        _execute_launch({'control_mode': 'trajectory', name: value})


@pytest.mark.parametrize('name', ['', '/', '/bad//name', '/bad name', '/0invalid'])
def test_invalid_controller_manager_names_rejected(name):
    with pytest.raises(RuntimeError, match='controller_manager_name'):
        _execute_launch({'controller_manager_name': name})


def test_unknown_control_mode_rejected():
    with pytest.raises(RuntimeError, match='control_mode'):
        _execute_launch({'control_mode': 'invented'})


def test_standard_controller_runtime_dependency_declared():
    import xml.etree.ElementTree as ET
    package = ET.parse(HERE.parent / 'package.xml').getroot()
    dependencies = {item.text for item in package.findall('exec_depend')}
    assert {'joint_trajectory_controller', 'control_msgs'} <= dependencies


def test_simulation_host_mode_fails_before_transport_or_process_setup():
    path = HERE.parent.parent / "robot_sim" / "launch" / "simulation.launch.py"
    spec = importlib.util.spec_from_file_location("simulation_launch_retirement", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations["controller_mode"] = "host"
    with pytest.raises(RuntimeError, match="controller_mode:=host.*retired"):
        module.launch_simulation(context)


def test_legacy_plugin_export_is_shadowed_in_install():
    from ament_index_python.resources import get_resource
    content, _ = get_resource("hardware_interface__pluginlib__plugin", "motor_hardware_interface")
    assert content == ""
