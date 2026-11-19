"""固件 TF 使用模型配置，启动前拒绝错误映射；不启动进程或硬件。"""

import importlib.util
from pathlib import Path
from types import ModuleType

from launch import LaunchContext
from launch.utilities import perform_substitutions
from launch_ros.utilities import evaluate_parameters
import pytest


def _load_launch_module() -> ModuleType:
    path = Path(__file__).parent.parent / 'launch' / 'firmware_tf.launch.py'
    spec = importlib.util.spec_from_file_location('firmware_tf_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _context(root: Path, model: str = 'test_arm') -> LaunchContext:
    context = LaunchContext()
    context.launch_configurations.update(model_dir=str(root), model=model)
    return context


def _model(root: Path, motors: str) -> None:
    model = root / 'models' / 'test_arm'
    (model / 'config').mkdir(parents=True)
    (model / 'urdf').mkdir()
    (model / 'config' / 'motor_driver.yaml').write_text(
        'motor_hardware:\n  motors:\n' + motors, encoding='utf-8')
    (model / 'urdf' / 'test_arm.urdf').write_text(
        '<robot name="test_arm"><link name="base"/><link name="tip"/>'
        '<joint name="elbow" type="continuous"><parent link="base"/>'
        '<child link="tip"/><axis xyz="0 0 1"/></joint></robot>', encoding='utf-8')


def test_single_motor_uses_model_mapping_and_disables_downlink(tmp_path: Path) -> None:
    _model(tmp_path, '    actuator: {id: 23, joint: elbow}\n')
    context = _context(tmp_path)
    nodes = _load_launch_module().firmware_tf_nodes(context)
    bridge = next(n for n in nodes if n.node_package == 'motor_hardware_interface')
    params = evaluate_parameters(context, getattr(bridge, '_Node__parameters'))[0]
    assert params['driver_file'] == str(tmp_path/'models'/'test_arm'/'config'/'motor_driver.yaml')
    assert params['model_name'] == 'test_arm'
    assert params['enable_trajectory'] is False
    publisher = next(n for n in nodes if n.node_package == 'robot_state_publisher')
    description = evaluate_parameters(
        context, getattr(publisher, '_Node__parameters'))[0]['robot_description']
    assert 'name="elbow"' in description
    assert ('joint_states', '/joint_states') in [
        (perform_substitutions(context, a), perform_substitutions(context, b))
        for a, b in getattr(publisher, '_Node__remappings')]


@pytest.mark.parametrize('motors', [
    '    a: {id: 1, joint: missing}\n',
    '    a: {id: 1, joint: elbow}\n    b: {id: 2, joint: elbow}\n',
    '    a: {id: 1, joint: elbow}\n    b: {id: 1, joint: elbow}\n',
])
def test_invalid_mapping_is_rejected_before_nodes_start(tmp_path: Path, motors: str) -> None:
    _model(tmp_path, motors)
    with pytest.raises(ValueError):
        _load_launch_module().firmware_tf_nodes(_context(tmp_path))


def test_model_path_cannot_escape_root(tmp_path: Path) -> None:
    with pytest.raises(ValueError):
        _load_launch_module().firmware_tf_nodes(_context(tmp_path, '../outside'))


@pytest.mark.parametrize('value', ['true', '1', 'yes'])
def test_legacy_trajectory_entry_is_rejected_before_reading_model(tmp_path, value):
    context = _context(tmp_path)
    context.launch_configurations['enable_trajectory'] = value
    with pytest.raises(ValueError, match='follow_joint_trajectory'):
        _load_launch_module().firmware_tf_nodes(context)
