"""静态检查台架入口：bridge 必须随 L1 起，否则下行没有编码者。"""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
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


def test_firmware_tf_requires_an_explicit_model() -> None:
    actions = _load_launch_module().generate_launch_description().entities
    includes = [a for a in actions if isinstance(a, IncludeLaunchDescription)
                and 'model' in dict(a.launch_arguments)]
    assert len(includes) == 1
    context = LaunchContext()
    context.launch_configurations['firmware_model'] = ''
    assert not includes[0].condition.evaluate(context)
    context.launch_configurations['firmware_model'] = 'half_body_robot'
    assert includes[0].condition.evaluate(context)


def test_manager_is_distributed_by_bringup() -> None:
    actions = _load_launch_module().generate_launch_description().entities
    managers = [a for a in actions if isinstance(a, Node)
                and a.node_executable == 'bench_manager']
    assert len(managers) == 1
    assert managers[0].node_package == 'robot_bringup'


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
