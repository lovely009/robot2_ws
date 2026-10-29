"""静态检查台架入口：bridge 必须随 L1 起，否则下行没有编码者。"""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import Node


def _load_launch_module():
    path = Path(__file__).parent.parent / 'launch' / 'robot.launch.py'
    spec = importlib.util.spec_from_file_location('robot_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_bridge_node_is_started():
    actions = _load_launch_module().generate_launch_description().entities
    found = [
        action for action in actions
        if isinstance(action, Node)
        and action.node_package == 'micro_ros_bridge'
        and action.node_executable == 'micro_ros_bridge_node'
    ]
    assert found, 'robot.launch.py 没有起 micro_ros_bridge：下行没有编码者'
    assert len(found) == 1, (
        f'robot.launch.py 起了 {len(found)} 个 micro_ros_bridge：下行只允许一个编码者')
    # Node.node_name 要 execute 后才可读（launch_ros 的实现），这里取构造参数。
    raw_name = getattr(found[0], '_Node__node_name')
    assert raw_name == 'micro_ros_bridge', (
        f'micro_ros_bridge 节点名是 {raw_name!r}，应为微固件侧约定的名字')


def test_model_selection_is_owned_by_the_manager() -> None:
    from launch_ros.utilities import evaluate_parameters
    actions = _load_launch_module().generate_launch_description().entities
    context = LaunchContext()
    for action in actions:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations['firmware_model'] = ''
    manager = next(a for a in actions if isinstance(a, Node)
                   and a.node_executable == 'bench_manager')
    assert evaluate_parameters(context, manager._Node__parameters)[0]['feedback_model'] == ''
    context.launch_configurations['firmware_model'] = 'half_body_robot'
    assert evaluate_parameters(context, manager._Node__parameters)[0]['feedback_model'] == 'half_body_robot'
    assert not [a for a in actions if isinstance(a, IncludeLaunchDescription)
                and 'enable_trajectory' in dict(a.launch_arguments)]


def test_manager_is_distributed_by_bringup() -> None:
    actions = _load_launch_module().generate_launch_description().entities
    managers = [a for a in actions if isinstance(a, Node)
                and a.node_executable == 'bench_manager']
    assert len(managers) == 1
    assert managers[0].node_package == 'robot_bringup'


def test_agent_autostart_defaults_on_and_is_forwarded_to_manager() -> None:
    from launch_ros.utilities import evaluate_parameters
    actions = _load_launch_module().generate_launch_description().entities
    context = LaunchContext()
    for action in actions:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    manager = next(a for a in actions if isinstance(a, Node)
                   and a.node_executable == 'bench_manager')
    parameters = evaluate_parameters(context, manager._Node__parameters)[0]
    assert parameters['auto_start_agent'] is True
    assert parameters['agent_device'] == ''
    context.launch_configurations['start_micro_ros_agent'] = 'false'
    context.launch_configurations['agent_tty'] = '/dev/ttyACM0'
    actions = _load_launch_module().generate_launch_description().entities
    manager = next(a for a in actions if isinstance(a, Node)
                   and a.node_executable == 'bench_manager')
    parameters = evaluate_parameters(context, manager._Node__parameters)[0]
    assert parameters['auto_start_agent'] is False
    assert parameters['agent_device'] == '/dev/ttyACM0'


def test_bringup_installs_manager_module_and_entrypoint(monkeypatch) -> None:
    import runpy
    import setuptools
    root = Path(__file__).parent.parent
    metadata = {}
    monkeypatch.chdir(root)
    monkeypatch.setattr(setuptools, 'setup', lambda **kw: metadata.update(kw))
    runpy.run_path(str(root / 'setup.py'))
    assert 'bench_manager' in metadata.get('packages', [])
    assert 'bench_manager = bench_manager.node:main' in metadata['entry_points']['console_scripts']


def test_only_host_execution_is_accepted_and_config_is_an_optional_pair(tmp_path):
    from launch_ros.utilities import evaluate_parameters
    module = _load_launch_module()
    actions = module.generate_launch_description().entities
    context = LaunchContext()
    for action in actions:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    bridge = next(a for a in actions if isinstance(a, Node)
                  and a.node_package == 'micro_ros_bridge')
    assert evaluate_parameters(context, bridge._Node__parameters)[0]['trajectory_execution'] == 'host'
    host = [a for a in actions if isinstance(a, IncludeLaunchDescription)
            and 'urdf_file' in dict(a.launch_arguments)]
    assert len(host) == 1
    assert not host[0].condition.evaluate(context)
    validate = next(a for a in actions if isinstance(a, OpaqueFunction))
    import pytest
    assert validate.execute(context) == []
    config = tmp_path / 'motors.yaml'; config.write_text('')
    urdf = tmp_path / 'model.urdf'; urdf.write_text('')
    context.launch_configurations['host_motor_config_file'] = str(config)
    with pytest.raises(ValueError, match='host_urdf_file'):
        validate.execute(context)
    context.launch_configurations['host_urdf_file'] = str(urdf)
    assert host[0].condition.evaluate(context)
    assert validate.execute(context) == []
    context.launch_configurations['trajectory_execution'] = 'firmware'
    with pytest.raises(ValueError, match='host'):
        validate.execute(context)
