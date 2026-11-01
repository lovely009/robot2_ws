"""Managed bringup contract tests; never execute nodes, serial devices or CAN."""

import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.events import Shutdown
from launch.events.process import ProcessExited
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters


def _module():
    source = Path(__file__).parents[1] / 'launch' / 'managed_robot.launch.py'
    spec = importlib.util.spec_from_file_location('managed_robot_launch', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def context(tmp_path):
    model = tmp_path / 'models' / 'test_arm'
    (model / 'config').mkdir(parents=True)
    (model / 'urdf').mkdir()
    (model / 'config' / 'motor_driver.yaml').write_text(
        'motor_hardware:\n  motors:\n    axis: {id: 23, joint: elbow}\n', encoding='utf-8')
    (model / 'urdf' / 'test_arm.urdf').write_text(
        '<robot name="test_arm"><link name="base"/><link name="tip"/>'
        '<joint name="elbow" type="continuous"><parent link="base"/>'
        '<child link="tip"/><axis xyz="0 0 1"/></joint></robot>', encoding='utf-8')
    context = LaunchContext()
    context.launch_configurations.update(
        model='test_arm', model_dir=str(tmp_path), robot_id='bench-01',
        calibration_file='', start_agent='false', agent_device='',
        allow_trajectories='false', start_web='false', web_port='8080', rosbridge_port='9090',
    )
    return context


def _nodes(module, context):
    return [action for action in module.managed_nodes(context) if isinstance(action, Node)]


def _parameters(context, node):
    return evaluate_parameters(context, getattr(node, '_Node__parameters'))[0]


def test_default_managed_graph_has_only_the_gated_writer_and_readonly_feedback(context):
    module = _module()
    nodes = _nodes(module, context)
    assert {node.node_package for node in nodes} == {
        'robot_calibration', 'robot_supervisor', 'micro_ros_bridge',
        'motor_hardware_interface', 'robot_state_publisher',
    }
    bridge = next(node for node in nodes if node.node_package == 'micro_ros_bridge')
    remaps = {
        perform_substitutions(context, source): perform_substitutions(context, target)
        for source, target in getattr(bridge, '_Node__remappings')
    }
    assert remaps == {
        '/motor/command_array': '/robot/managed/motor/command_array',
        '/motor/policy_array': '/robot/managed/motor/policy_array',
        '/motor/trajectory': '/robot/managed/motor/trajectory',
        '/motor/control_lease': '/robot/managed/motor/control_lease',
        '/motor/param_set': '/robot/managed/maintenance/param_set',
        '/motor/system_command': '/robot/managed/maintenance/system_command',
        '/motor/execute_config': '/robot/managed/maintenance/execute_config',
    }
    feedback = next(node for node in nodes if node.node_package == 'motor_hardware_interface')
    assert _parameters(context, feedback)['enable_trajectory'] is False
    supervisor = next(node for node in nodes if node.node_package == 'robot_supervisor')
    assert _parameters(context, supervisor) == {
        'allow_trajectories': False,
        'driver_file': str(Path(context.launch_configurations['model_dir']) /
                           'models/test_arm/config/motor_driver.yaml'),
        'urdf_file': str(Path(context.launch_configurations['model_dir']) /
                         'models/test_arm/urdf/test_arm.urdf'),
        'robot_id': 'bench-01', 'model_name': 'test_arm',
    }
    calibration = next(node for node in nodes if node.node_package == 'robot_calibration')
    calibration_params = _parameters(context, calibration)
    assert calibration_params['manifest_file'] == ''
    for key in ('driver_file', 'urdf_file', 'robot_id', 'model_name'):
        assert calibration_params[key] == _parameters(context, supervisor)[key]


def test_owned_agent_requires_explicit_device_and_trajectory_opt_in(context):
    module = _module()
    context.launch_configurations['start_agent'] = 'true'
    with pytest.raises(ValueError, match='agent_device'):
        module.managed_nodes(context)
    context.launch_configurations['agent_device'] = '/dev/serial/by-id/test-controller'
    nodes = _nodes(module, context)
    assert sum(node.node_package == 'micro_ros_agent' for node in nodes) == 1
    supervisor = next(node for node in nodes if node.node_package == 'robot_supervisor')
    assert _parameters(context, supervisor)['allow_trajectories'] is False
    context.launch_configurations['allow_trajectories'] = 'true'
    supervisor = next(node for node in _nodes(module, context)
                      if node.node_package == 'robot_supervisor')
    assert _parameters(context, supervisor)['allow_trajectories'] is True


def test_trajectories_cannot_outlive_an_external_agent(context):
    context.launch_configurations['allow_trajectories'] = 'true'
    with pytest.raises(ValueError, match='管理 Agent'):
        _module().managed_nodes(context)


@pytest.mark.parametrize('key,value', [
    ('model', '../outside'), ('model', ''), ('robot_id', ' '),
    ('start_agent', '1'), ('allow_trajectories', 'yes'), ('start_web', '1'),
])
def test_invalid_launch_arguments_fail_before_any_process_is_started(context, key, value):
    context.launch_configurations[key] = value
    with pytest.raises(ValueError):
        _module().managed_nodes(context)


def test_every_critical_process_exit_shuts_down_the_whole_managed_session(context):
    module = _module()
    context.launch_configurations.update(
        start_agent='true', agent_device='/dev/serial/by-id/test-controller')
    actions = module.managed_nodes(context)
    registrations = [action for action in actions if isinstance(action, RegisterEventHandler)]
    nodes = [action for action in actions if isinstance(action, Node)]
    assert len(registrations) == len(nodes) == 4
    stacks = [a for a in actions if isinstance(a, IncludeLaunchDescription)]
    assert len(stacks) == 1
    assert dict(stacks[0].launch_arguments)['start_controllers'] == 'true'
    assert actions[:len(registrations)] == registrations
    for node in nodes:
        event = ProcessExited(action=node, name='test', cmd=[], cwd=None, env=None,
                              pid=12345, returncode=1)
        matched = [registration.event_handler for registration in registrations
                   if registration.event_handler.matches(event)]
        assert len(matched) == 1
        on_exit = list(matched[0].handle(event, context))
        assert len(on_exit) == 1
        assert isinstance(on_exit[0], EmitEvent)
        assert isinstance(on_exit[0].event, Shutdown)


def test_web_is_a_managed_client_outside_the_critical_process_set(context):
    module = _module()
    context.launch_configurations['start_web'] = 'true'
    actions = module.managed_nodes(context)
    web = [action for action in actions if isinstance(action, IncludeLaunchDescription)]
    assert len(web) == 1
    assert dict(web[0].launch_arguments) == {
        'control_mode': 'managed', 'robot_model': 'test_arm', 'robot_id': 'bench-01',
        'model_dir': context.launch_configurations['model_dir'],
        'web_port': '8080', 'rosbridge_port': '9090',
    }
    handlers = [action.event_handler for action in actions
                if isinstance(action, RegisterEventHandler)]
    assert len(handlers) == 3
    optional_web_node = Node(package='web_host', executable='static_server')
    event = ProcessExited(action=optional_web_node, name='web', cmd=[], cwd=None,
                          env=None, pid=23456, returncode=1)
    assert not any(handler.matches(event) for handler in handlers)
