import importlib.util
from ast import literal_eval
from pathlib import Path

import pytest


def test_workbench_exposes_controls_but_no_wire_writes():
    pytest.importorskip('launch')
    from launch import LaunchContext
    from launch_ros.utilities import evaluate_parameters
    path = Path(__file__).resolve().parents[1] / 'launch/web_host.launch.py'
    spec = importlib.util.spec_from_file_location('workbench_web_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(control_mode='bench', robot_model='fixture',
        model_dir='', rosbridge_port='9090', web_port='8080')
    nodes = module.web_nodes(context)
    assert {node.node_executable for node in nodes} == {'rosbridge_shim', 'static_server'}
    bridge = nodes[0]
    params = evaluate_parameters(context, getattr(bridge, '_Node__parameters'))[0]
    assert literal_eval(params['topics_pub_glob']) == module.CONTROL_TOPICS
    assert params['address'] == '127.0.0.1'
    web_params = evaluate_parameters(context, getattr(nodes[1], '_Node__parameters'))[0]
    assert web_params['host'] == '127.0.0.1'
    assert params['topics_glob'] == '[]'
    assert params['call_services_in_new_thread'] is True
    assert not any('/managed/' in topic or '/micro_ros_' in topic
                   for topic in literal_eval(params['topics_pub_glob']))
    assert {'/bench/status', '/motor/control_states', '/joint_states'} <= set(literal_eval(params['topics_sub_glob']))
    assert {'/control/status', '/control/trajectory_state'} <= set(literal_eval(params['topics_sub_glob']))
    assert '/micro_ros_up/status' in literal_eval(params['topics_sub_glob'])
    assert '/bench/*' in literal_eval(params['services_glob'])
    assert '/motor/submit_policy' in literal_eval(params['services_glob'])
    assert '/motor/policy_array' not in literal_eval(params['topics_pub_glob'])
    assert '/control/setpoint' in literal_eval(params['topics_pub_glob'])
    assert '/control/joint_target' in literal_eval(params['topics_pub_glob'])
    assert not any('/host_control/' in topic for topic in literal_eval(params['topics_pub_glob']))
    assert '/control/trajectory_raw' in literal_eval(params['topics_pub_glob'])
    context.launch_configurations['control_mode'] = 'readonly'
    params = evaluate_parameters(context, getattr(module.web_nodes(context)[0], '_Node__parameters'))[0]
    assert literal_eval(params['topics_pub_glob']) == []
    assert literal_eval(params['services_glob']) == ['/rosapi/*']
    assert '/micro_ros_up/status' in literal_eval(params['topics_sub_glob'])
