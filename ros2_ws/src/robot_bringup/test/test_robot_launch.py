"""Public bringup selects the managed MCU chain; bench stays read-only."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node


def _module(filename):
    path = Path(__file__).parent.parent / 'launch' / filename
    spec = importlib.util.spec_from_file_location(filename.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _defaults(actions, context):
    for action in actions:
        if isinstance(action, DeclareLaunchArgument) and action.default_value is not None:
            action.execute(context)


def test_default_robot_entry_includes_managed_with_web_enabled():
    actions = _module('robot.launch.py').generate_launch_description().entities
    context = LaunchContext()
    _defaults(actions, context)
    assert context.launch_configurations['start_web'] == 'true'
    includes = [action for action in actions if isinstance(action, IncludeLaunchDescription)]
    assert len(includes) == 1
    source = includes[0].launch_description_source
    source.get_launch_description(LaunchContext())
    assert 'managed_robot.launch.py' in source.location
    assert not any(isinstance(action, Node) for action in actions)
    managed = includes[0].launch_description_source.get_launch_description(context).entities
    _defaults(managed, context)
    assert context.launch_configurations['start_web'] == 'true'
    assert context.launch_configurations['start_agent'] == 'false'
    assert context.launch_configurations['allow_trajectories'] == 'false'
    assert {action.name for action in managed
            if isinstance(action, DeclareLaunchArgument) and action.default_value is None} == {
                'model', 'robot_id'}


def test_public_robot_respects_explicit_headless_selection():
    actions = _module('robot.launch.py').generate_launch_description().entities
    context = LaunchContext()
    context.launch_configurations['start_web'] = 'false'
    _defaults(actions, context)
    assert context.launch_configurations['start_web'] == 'false'


def test_bench_starts_no_motor_writer_or_agent():
    actions = _module('bench.launch.py').generate_launch_description().entities
    nodes = [action for action in actions if isinstance(action, Node)]
    assert {node.node_package for node in nodes} == {'robot_bringup'}
    includes = [action for action in actions if isinstance(action, IncludeLaunchDescription)]
    web = next(action for action in includes if 'control_mode' in dict(action.launch_arguments))
    assert dict(web.launch_arguments)['control_mode'] == 'readonly'
    tf = next(action for action in includes if 'enable_trajectory' in dict(action.launch_arguments))
    assert dict(tf.launch_arguments)['enable_trajectory'] == 'false'
    context = LaunchContext()
    _defaults(actions, context)
    assert context.launch_configurations['firmware_model'] == ''
    assert not tf.condition.evaluate(context)
    context.launch_configurations['firmware_model'] = 'half_body_robot'
    assert tf.condition.evaluate(context)
