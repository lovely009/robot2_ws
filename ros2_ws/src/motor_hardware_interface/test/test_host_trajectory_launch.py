"""主机轨迹配置必须保留模型、限位和唯一执行路径；不启动硬件。"""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


def module():
    path = Path(__file__).parents[1] / 'launch' / 'host_trajectory.launch.py'
    spec = importlib.util.spec_from_file_location('host_trajectory_launch', path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


@pytest.fixture
def model(tmp_path):
    driver = tmp_path / 'driver.yaml'
    driver.write_text('motor_hardware:\n  motors:\n'
                      '    a: {id: 1, joint: shoulder}\n'
                      '    b: {id: 64, joint: elbow}\n')
    urdf = tmp_path / 'robot.urdf'
    urdf.write_text('<robot name="test"><link name="base"/>'
                    '<joint name="shoulder" type="revolute">'
                    '<limit lower="-1.2" upper="1.3" velocity="2.5" effort="10"/>'
                    '</joint><joint name="elbow" type="continuous">'
                    '<limit velocity="3" effort="10"/></joint>'
                    '<ros2_control name="old_can" type="system">'
                    '<hardware><plugin>old/Can</plugin></hardware></ros2_control></robot>')
    return driver, urdf


def test_real_model_and_joint_limits_reach_topic_hardware(model):
    driver, urdf = model
    text, joints = module().hardware_description(str(driver), str(urdf))
    root = ET.fromstring(text)
    assert joints == ['shoulder', 'elbow']
    assert root.find('joint').get('type') == 'revolute'
    controls = root.findall('ros2_control')
    assert len(controls) == 1
    control = controls[0]
    assert control.findtext('hardware/plugin') == 'motor_hardware_interface/MicroRosSystem'
    assert control.findtext('hardware/param[@name="config_file"]') == str(driver)
    joint = control.find('joint[@name="shoulder"]')
    assert joint.findtext('command_interface[@name="position"]/param[@name="min"]') == '-1.2'
    assert joint.findtext('command_interface[@name="velocity"]/param[@name="max"]') == '2.5'
    assert control.find('joint[@name="elbow"]/command_interface[@name="position"]/param') is None
    assert len(joint.findall('state_interface')) == 3
    assert joint.findtext('command_interface[@name="effort"]/param[@name="max"]') == '10.0'


@pytest.mark.parametrize('change', ['missing_joint', 'missing_limit', 'invalid_velocity', 'mimic'])
def test_ambiguous_or_invalid_model_is_rejected(model, change):
    driver, urdf = model
    root = ET.fromstring(urdf.read_text())
    joint = root.find('joint')
    if change == 'missing_joint':
        root.remove(joint)
    elif change == 'missing_limit':
        joint.remove(joint.find('limit'))
    elif change == 'invalid_velocity':
        joint.find('limit').set('velocity', 'nan')
    else:
        ET.SubElement(joint, 'mimic', {'joint': 'elbow'})
    urdf.write_text(ET.tostring(root, encoding='unicode'))
    with pytest.raises(ValueError):
        module().hardware_description(str(driver), str(urdf))


def test_duplicate_motor_identity_is_rejected_before_any_node_starts(model):
    driver, urdf = model
    driver.write_text(driver.read_text().replace('id: 64', 'id: 1'))
    with pytest.raises(ValueError, match='motor_id'):
        module().hardware_description(str(driver), str(urdf))


def test_jtc_uses_measured_feedback_and_both_target_channels():
    params = module().controller_parameters(['shoulder', 'elbow'])
    jtc = params['/host_control/trajectory_controller']['ros__parameters']
    assert jtc['joints'] == ['shoulder', 'elbow']
    assert jtc['command_interfaces'] == ['position', 'velocity']
    assert jtc['state_interfaces'] == ['position', 'velocity']
    assert jtc['open_loop_control'] is False
    assert jtc['allow_partial_joints_goal'] is False
    assert jtc['constraints']['shoulder']['trajectory'] > 0


def test_manual_and_gravity_controllers_claim_distinct_interfaces():
    params = module().controller_parameters(['shoulder', 'elbow'], '/robot.urdf')
    manager = params['/host_control/controller_manager']['ros__parameters']
    assert manager['manual_controller_0']['type'] == (
        'forward_command_controller/MultiInterfaceForwardCommandController')
    assert params['/host_control/manual_controller_0']['ros__parameters'] == {
        'joint': 'shoulder', 'interface_names': ['position', 'velocity', 'effort']}
    assert params['/host_control/manual_controller_1']['ros__parameters']['joint'] == 'elbow'
    gravity = params['/host_control/gravity_controller']['ros__parameters']
    assert gravity == {'joints': ['shoulder', 'elbow'], 'urdf_file': '/robot.urdf',
                       'enabled': False, 'effort_gain': 1.0}
