"""静态检查台架入口：bridge 必须随 L1 起，否则下行没有编码者。"""

import importlib.util
from pathlib import Path

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
