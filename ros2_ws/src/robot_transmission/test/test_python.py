from pathlib import Path

import pytest
import yaml

from robot_transmission import Axis, Model


def test_position_aware_effort_binding(tmp_path):
    driver = tmp_path / 'driver.yaml'
    driver.write_text(yaml.safe_dump({'motor_hardware': {'motors': {
        'a': {'id': 1, 'joint': 'q1'}, 'b': {'id': 2, 'joint': 'q2'},
        'c': {'id': 3, 'joint': 'q3'},
    }}}))
    linear = Model.load(str(driver))
    assert linear.joint_effort_to_actuator({'q1': 2}) == {'a': 2}
    assert not linear.groups[0].position_dependent
    params = {
        'motor_axis_origin': '.033699999952 .04475 .4',
        'motor_axis_direction': '1 0 0',
        'motor_ball_zero': '.033699999952 .061556509578 .389158356406',
        'joint1_axis_origin': '-.0025 .06725 .4825',
        'joint1_axis_direction': '0 0 1',
        'joint2_axis_origin': '.0025 .06775 .5025',
        'joint2_axis_direction': '0 -1 0',
        'joint_ball_zero': '.035840440737 .06775 .494319840327',
        'rod_length': '.10536545216923923',
    }
    (tmp_path / 'transmissions.yaml').write_text(yaml.safe_dump({'transmissions': {
        'linkage': {'plugin': 'robot_transmission/SpatialLinkageTransmission',
                    'actuators': ['a', 'b'], 'joints': ['q1', 'q2'], 'parameters': params}}}))
    m = Model.load(str(driver))
    assert m.groups[0].position_dependent
    with pytest.raises(AttributeError):
        m.groups[0].position_dependent = False
    pose = {'q1': .17, 'q2': .08, 'q3': .4}
    effort = {'q1': 2, 'q2': 4}
    expected = m.joint_to_actuator({'q1': Axis(.17, 0, 2), 'q2': Axis(.08, 0, 4)})
    actual = m.joint_effort_to_actuator(effort, joint_positions=pose)
    assert actual == pytest.approx({n: a.effort for n, a in expected.items()})
    assert m.actuator_effort_to_joint(actual, actuator_positions={
        n: a.position for n, a in expected.items()}) == pytest.approx(effort)
    assert abs(actual['b'] - m.joint_effort_to_actuator(
        effort, joint_positions={'q1': 0, 'q2': 0})['b']) > .001
    for invalid in ({}, {'q1': 0}, {'q1': float('nan'), 'q2': 0}, {'q1': 0, 'q2': 0, 'bad': 0}):
        with pytest.raises(ValueError):
            m.joint_effort_to_actuator(effort, joint_positions=invalid)
    with pytest.raises(ValueError):
        m.joint_effort_to_actuator(effort)
    with pytest.raises(ValueError):
        m.actuator_effort_to_joint({'a': 0}, actuator_positions={'a': 0, 'b': 0})
    import robot_transmission._core as core
    assert 'install/robot_transmission/' in str(Path(core.__file__))
