"""Exercise real MuJoCo limiting without starting ROS or connecting hardware."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest


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
