from pathlib import Path

import pytest
from robot_transmission import Model


@pytest.fixture
def model(tmp_path):
    driver = tmp_path / 'driver.yaml'
    driver.write_text('motor_hardware:\n  motors:\n'
                      '    a: {id: 8, joint: yaw}\n'
                      '    b: {id: 9, joint: pitch}\n', encoding='utf-8')
    (tmp_path / 'transmissions.yaml').write_text(
        'transmissions:\n  wrist:\n'
        '    plugin: transmission_interface/DifferentialTransmission\n'
        '    actuators: [a, b]\n    joints: [yaw, pitch]\n'
        '    actuator_reduction: [1, 1]\n    joint_reduction: [1, 1]\n'
        '    joint_offset: [0, 0]\n', encoding='utf-8')
    return Model.load(str(driver))
