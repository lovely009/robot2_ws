"""Exercise real MuJoCo limiting without starting ROS or connecting hardware."""
import importlib.util
from itertools import product
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest


def test_full_body_position_targets_reach_closed_loop_corners(monkeypatch):
    mujoco = pytest.importorskip('mujoco')
    pytest.importorskip('rclpy')
    package = Path(__file__).resolve().parents[1]
    monkeypatch.syspath_prepend(str(package / 'scripts'))
    from mujoco_effort_bridge import model_text
    from mujoco_joint_position import JointPositionTargets

    path = package.parent / 'robot_description/models/full_body_robot/mujoco/full_body_robot.xml'
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    config = json.loads(model_text(model, 'robot_sim_joint_position'))
    controller = JointPositionTargets(model, config)
    assert len(controller.joints) == 37
    assert len(controller.rods) == 10
    names = list(config['differential_limits'])
    pairs = list(zip(names[::2], names[1::2]))
    # All six groups at their corners, with the waist yaw also displaced.
    controller.apply(data, ['BN_Spine_yaw'], [0.15])
    for first, second in product((0, 1), repeat=2):
        for a, b in pairs:
            controller.apply(data, [a], [controller.limits[a][first]])
            controller.apply(data, [b], [controller.limits[b][second]])
        assert np.all(data.ctrl >= model.actuator_ctrlrange[:, 0])
        assert np.all(data.ctrl <= model.actuator_ctrlrange[:, 1])
        mujoco.mj_step(model, data, nstep=1800)
        actual = [data.qpos[address] for address in controller.joints.values()]
        np.testing.assert_allclose(actual, list(controller.targets.values()), atol=0.002)
    previous = data.ctrl.copy()
    with pytest.raises(ValueError, match='超出仿真范围'):
        controller.apply(data, ['BN_Spine_yaw', names[0]], [0., 1.])
    np.testing.assert_array_equal(data.ctrl, previous)
    assert controller.targets['BN_Spine_yaw'] == 0.15


def test_viewer_position_model_preserves_control_and_publishes_sim_time(tmp_path, monkeypatch):
    pytest.importorskip('mujoco')
    pytest.importorskip('rclpy')
    scripts = Path(__file__).resolve().parents[1] / 'scripts'
    monkeypatch.syspath_prepend(str(scripts))
    spec = importlib.util.spec_from_file_location('viewer_bridge_test', scripts / 'mujoco_effort_bridge.py')
    bridge = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(bridge)
    path = tmp_path / 'preview.xml'
    path.write_text('''<mujoco>
      <custom><text name="robot_sim_control_mode" data="viewer_position"/></custom>
      <worldbody><body><joint name="j" type="hinge"/>
        <geom type="sphere" size="0.1"/></body></worldbody>
      <actuator><position joint="j" kp="4" ctrlrange="-0.2 0.2"/></actuator>
    </mujoco>''')
    params = {'mujoco_model': str(path), 'headless': True}
    monkeypatch.setattr(bridge.Node, '__init__', lambda *args: None)
    monkeypatch.setattr(bridge.Node, 'declare_parameter',
                        lambda self, name, default: SimpleNamespace(value=params.get(name, default)))
    subscribe = Mock()
    monkeypatch.setattr(bridge.Node, 'create_subscription', subscribe)
    monkeypatch.setattr(bridge.Node, 'create_publisher', lambda *args: Mock())
    monkeypatch.setattr(bridge.Node, 'create_timer', lambda *args: None)
    monkeypatch.setattr(bridge.Node, 'get_logger', lambda self: Mock())
    node = bridge.MujocoEffortBridge()
    subscribe.assert_not_called()
    node.data.ctrl[:] = 0.15
    node._tick()
    np.testing.assert_allclose(node.data.ctrl, [0.15])
    state = node.publisher.publish.call_args.args[0]
    clock = node.clock_publisher.publish.call_args.args[0]
    assert state.name == ['j'] and state.position[0] > 0
    assert state.header.stamp == clock.clock
    params['joint_names'] = 'j'
    with pytest.raises(RuntimeError, match='viewer_position'):
        bridge.MujocoEffortBridge()


@pytest.mark.parametrize('joint_limited', [False, True])
@pytest.mark.parametrize('command', [-100.0, -0.25, 0.25, 100.0])
def test_optional_joint_limit_keeps_actuator_limits(tmp_path, monkeypatch, joint_limited, command):
    mujoco = pytest.importorskip('mujoco')
    pytest.importorskip('rclpy')
    scripts = Path(__file__).resolve().parents[1] / 'scripts'
    monkeypatch.syspath_prepend(str(scripts))
    spec = importlib.util.spec_from_file_location('effort_bridge_test', scripts / 'mujoco_effort_bridge.py')
    bridge = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(bridge)
    path = tmp_path / 'arm.xml'
    limit = 'actuatorfrclimited="true" actuatorfrcrange="-0.4 0.4"' if joint_limited else ''
    path.write_text(f'''<mujoco><worldbody><body>
      <joint name="j" type="hinge" {limit}/><geom type="sphere" size="0.1"/>
      </body></worldbody><actuator>
      <motor joint="j" ctrllimited="true" ctrlrange="-0.7 0.7"
             forcelimited="true" forcerange="-0.7 0.7"/>
      </actuator></mujoco>''')
    params = {'mujoco_model': str(path), 'headless': True}
    monkeypatch.setattr(bridge.Node, '__init__', lambda *args: None)
    monkeypatch.setattr(bridge.Node, 'declare_parameter',
                        lambda self, name, default: SimpleNamespace(value=params.get(name, default)))
    for name in ('create_subscription', 'create_publisher', 'create_timer'):
        monkeypatch.setattr(bridge.Node, name, lambda *args: None)
    monkeypatch.setattr(bridge.Node, 'get_logger', lambda self: Mock())
    node = bridge.MujocoEffortBridge()
    node._apply_effort([command])
    mujoco.mj_forward(node.model, node.data)
    cap = 0.4 if joint_limited else 0.7
    np.testing.assert_allclose(node.data.qfrc_actuator, [np.clip(command, -cap, cap)])
