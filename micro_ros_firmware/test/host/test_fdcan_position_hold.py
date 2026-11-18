"""位置 HIL 在板子命令失效时锁位，不能持续把零目标发送给 MuJoCo。"""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import time

import pytest

SCRIPT = Path(__file__).parents[3] / 'micro_ros_agent/tools/fdcan_motor_sim.py'


def test_feedback_snapshot_is_reused_until_update_or_expiry(monkeypatch, tmp_path):
    spec = importlib.util.spec_from_file_location('fdcan_motor_feedback_snapshot', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    driver = tmp_path / 'driver.yaml'
    driver.write_text('motor_hardware:\n  motors:\n    m: {id: 1, joint: input_axis}\n')
    bridge = module.GazeboBridge.__new__(module.GazeboBridge)
    bridge.joint_map = {1: 'input_axis'}
    bridge.joints = ['input_axis']
    bridge.enable_transmission(driver)
    clock = iter([1., 1.1, 1.2, 1.3, 1.4, 1.4, 1.91])
    monkeypatch.setattr(module, 'time', SimpleNamespace(monotonic=lambda: next(clock)))
    original_transform = bridge._actuator_feedback
    transforms = []

    def transform(now):
        transforms.append(now)
        return original_transform(now)

    monkeypatch.setattr(bridge, '_actuator_feedback', transform)
    bridge._on_state(SimpleNamespace(name=['input_axis'], position=[.3], velocity=[.2]))
    assert bridge.joint_position(1) == .3
    assert bridge.joint_velocity(1) == .2
    assert bridge.feedback_available(1)
    assert transforms == [1.1]

    bridge._on_state(SimpleNamespace(name=['input_axis'], position=[.6], velocity=[.4]))
    assert bridge.joint_position(1) == .6
    assert not bridge.feedback_available(1)
    assert transforms == [1.1, 1.4, 1.91]


@pytest.mark.parametrize('failure', ['timeout', 'disabled', 'nonfinite', 'firmware_offline',
                                     'neutral_enable', 'damping'])
def test_position_hil_latches_measured_input_on_invalid_command(tmp_path, failure):
    spec = importlib.util.spec_from_file_location('fdcan_motor_position_hold', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    driver = tmp_path / 'driver.yaml'
    driver.write_text('motor_hardware:\n  motors:\n    m: {id: 1, joint: input_axis}\n')
    bridge = module.GazeboBridge.__new__(module.GazeboBridge)
    bridge.joint_map = {1: 'input_axis'}
    bridge.joints = ['input_axis']
    bridge.enable_transmission(driver)
    bridge.control_mode = 'position'
    bridge.fw_lifecycle = {}
    bridge._msg_type = SimpleNamespace
    sent = []
    bridge.pub = SimpleNamespace(publish=lambda msg: sent.append(msg.data))
    motor = module.MotorModel(1, 'cerb')
    motor.enabled = True
    motor.last_cmd = {'pos_rad': .6, 'vel_rad_s': 0., 'kp': 1., 'kd': 1., 'tau_nm': 0.}
    now = time.monotonic()
    motor.last_cmd_at = now
    bridge.pos['input_axis'] = .3
    bridge.state_at['input_axis'] = now
    bridge.publish_transmission_commands({1: motor}, now)
    assert sent[-1] == [.6]
    if failure == 'timeout': motor.last_cmd_at = now - 1
    elif failure == 'disabled': motor.enabled = False
    elif failure == 'nonfinite': motor.last_cmd['pos_rad'] = float('nan')
    elif failure == 'firmware_offline':
        bridge.fw_lifecycle = {1: 0}
        bridge.fw_status_at = now
    else:
        # The symmetric encoding can decode zero as one half-LSB off zero.
        payload = bytes.fromhex('cc 80 00 80 00 00 00 08 00')
        if failure == 'damping':
            payload = bytes.fromhex('cc 80 00 80 00 00 01 08 00')
        motor.last_cmd = module.cerb_decode_command(payload)
        assert motor.last_cmd['enabled'] and motor.last_cmd['kp'] == 0
    bridge.publish_transmission_commands({1: motor}, now)
    assert sent[-1] == [.3]
    bridge.pos['input_axis'] = .4
    bridge.publish_transmission_commands({1: motor}, now)
    assert sent[-1] == [.3]
    bridge.fw_lifecycle = {}
    motor.enabled = True
    # The same zero position with nonzero Kp is a valid position command.
    motor.last_cmd = module.cerb_decode_command(bytes.fromhex('cc 80 00 80 00 31 00 08 00'))
    motor.last_cmd_at = now
    bridge.publish_transmission_commands({1: motor}, now)
    assert sent[-1] == [motor.last_cmd['pos_rad']]


def test_partial_motor_selection_preserves_full_joint_vector_and_holds_omitted_joint(tmp_path):
    spec = importlib.util.spec_from_file_location('fdcan_motor_partial_hold', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    driver = tmp_path / 'driver.yaml'
    driver.write_text('motor_hardware:\n  motors:\n'
                      '    selected: {id: 1, joint: selected_axis}\n'
                      '    omitted: {id: 2, joint: omitted_axis}\n')
    bridge = module.GazeboBridge.__new__(module.GazeboBridge)
    bridge.joint_map = {1: 'selected_axis', 2: 'omitted_axis'}
    bridge.joints = ['selected_axis', 'omitted_axis']
    bridge.enable_transmission(driver)
    bridge.control_mode = 'position'
    bridge.fw_lifecycle = {}
    bridge._msg_type = SimpleNamespace
    sent = []
    bridge.pub = SimpleNamespace(publish=lambda msg: sent.append(msg.data))
    motor = module.MotorModel(1, 'cerb')
    motor.enabled = True
    motor.last_cmd = {'pos_rad': .6, 'vel_rad_s': 0., 'kp': 1., 'kd': 1., 'tau_nm': 0.}
    now = time.monotonic()
    motor.last_cmd_at = now
    bridge.pos.update(selected_axis=.2, omitted_axis=.3)
    bridge.state_at = {joint: now for joint in bridge.joints}

    bridge.publish_transmission_commands({1: motor}, now)
    assert sent[-1] == [.6, .3]
    bridge.pos['omitted_axis'] = .4
    bridge.publish_transmission_commands({1: motor}, now)
    assert sent[-1] == [.6, .3]
