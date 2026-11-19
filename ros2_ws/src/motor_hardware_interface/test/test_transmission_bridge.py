"""传动反馈的完整性、测量时间、几何换算和关节限位参数。"""

import json
from pathlib import Path

import pytest

from motor_hardware_interface.topic_bridge import BridgeError
from motor_hardware_interface.transmission_bridge import TransmissionBridge


@pytest.fixture
def driver(tmp_path):
    path = tmp_path / 'motor_driver.yaml'
    path.write_text(
        'motor_hardware:\n  motors:\n'
        '    a: {id: 8, joint: yaw}\n    b: {id: 9, joint: pitch}\n',
        encoding='utf-8',
    )
    (tmp_path / 'transmissions.yaml').write_text(
        'transmissions:\n  wrist:\n'
        '    plugin: transmission_interface/DifferentialTransmission\n'
        '    actuators: [a, b]\n    joints: [yaw, pitch]\n',
        encoding='utf-8',
    )
    return path


@pytest.fixture
def bridge(driver):
    return TransmissionBridge(str(driver))


@pytest.fixture
def spatial_bridge(driver):
    fixture = Path(__file__).parent / 'fixtures/spatial_transmissions.yaml'
    driver.with_name('transmissions.yaml').write_text(fixture.read_text())
    return TransmissionBridge(str(driver))


def states(a=.3, b=.1, age_b=0):
    return [
        dict(motor_id=8, position=a, velocity=.4, effort=3., state_age_ms=0),
        dict(motor_id=9, position=b, velocity=.2, effort=1., state_age_ms=age_b),
    ]


def test_feedback_converts_position_velocity_and_effort(bridge):
    result = bridge.update(states(), 1.)
    yaw, pitch = result['yaw'], result['pitch']
    assert (yaw.position, yaw.velocity, yaw.effort) == pytest.approx((.2, .3, 4.))
    assert (pitch.position, pitch.velocity, pitch.effort) == pytest.approx((.1, .1, 2.))


def test_feedback_waits_for_both_actuators(bridge):
    assert bridge.update(states()[:1], 1.) == {}
    assert bridge.state_stamp is None
    assert set(bridge.update(states()[1:], 1.02)) == {'yaw', 'pitch'}
    assert bridge.state_stamp == 1.


@pytest.mark.parametrize('age', [51, 700, -1, float('nan'), None])
def test_invalid_age_or_excessive_skew_omits_the_group(bridge, age):
    bridge.update(states(), 1.)
    assert bridge.update(states(age_b=age), 2.) == {}
    assert bridge.state_stamp is None
    assert set(bridge.update(states(), 3.)) == {'yaw', 'pitch'}


def test_feedback_stamp_includes_reported_age_and_expires(bridge):
    assert bridge.update(states(age_b=25), 1.)
    assert bridge.state_stamp == pytest.approx(.975)
    assert bridge.update([], 1.4)
    assert bridge.state_stamp == pytest.approx(.975)
    assert bridge.update([], 1.5) == {}
    assert bridge.state_stamp is None


@pytest.mark.parametrize('field', ['position', 'velocity', 'effort'])
def test_invalid_measurement_clears_cached_actuator(bridge, field):
    bridge.update(states(), 1.)
    invalid = {**states()[0], field: float('nan')}
    assert bridge.update([invalid], 1.01) == {}
    assert bridge.update(states()[1:], 1.02) == {}
    assert set(bridge.update(states()[:1], 1.03)) == {'yaw', 'pitch'}


def test_unknown_motor_has_no_effect(bridge):
    bridge.update(states(), 1.)
    unknown = {**states()[0], 'motor_id': 64, 'position': 9.}
    assert bridge.update([unknown], 1.01)['yaw'].position == pytest.approx(.2)
    assert bridge.state_stamp == 1.


def test_nonlinear_feedback_requires_synchronised_group(spatial_bridge):
    assert spatial_bridge.update(states(.2, .2)[:1], 1.) == {}
    assert set(spatial_bridge.update(states(.2, .2), 1.)) == {'yaw', 'pitch'}
    assert spatial_bridge.update(states(.2, .2, age_b=51), 2.) == {}
    assert spatial_bridge.state_stamp is None


def test_nonlinear_geometry_failure_discards_samples_and_recovers(spatial_bridge):
    spatial_bridge.update(states(.2, .2), 1.)
    assert spatial_bridge.update(states(.2, 4.), 1.2) == {}
    assert spatial_bridge.state_stamp is None
    assert spatial_bridge.samples == {}
    observed = spatial_bridge.update(states(.1, .1), 1.3)
    assert observed['yaw'].position == pytest.approx(.1)
    assert spatial_bridge.state_stamp == pytest.approx(1.3)


@pytest.mark.parametrize('options', [
    {'feedback_timeout': 0}, {'feedback_timeout': -1},
    {'feedback_timeout': float('inf')}, {'feedback_timeout': float('nan')},
    {'max_skew': -1}, {'max_skew': float('nan')},
])
def test_invalid_feedback_windows_are_rejected(driver, options):
    with pytest.raises(BridgeError):
        TransmissionBridge(str(driver), **options)


@pytest.fixture
def limited_bridge(tmp_path):
    driver = tmp_path / 'motor_driver.yaml'
    driver.write_text('motor_hardware:\n  motors:\n    m2: {id: 2, joint: joint_link2}\n')
    urdf = tmp_path / 'robot.urdf'
    urdf.write_text('<robot name="arm"><joint name="joint_link2" type="revolute">'
                    '<limit lower="-0.12" upper="0.14"/></joint></robot>')
    return TransmissionBridge(str(driver), urdf_file=str(urdf))


def test_joint_limit_override_keeps_urdf_defaults(limited_bridge):
    payload = json.dumps({
        'model': 'arm', 'joint_limits': {'joint_link2': {'lower': -.5, 'upper': .14}},
    })
    assert limited_bridge.joint_limits_from_json(payload, 'arm') == {'joint_link2': (-.5, .14)}
    assert limited_bridge.joint_limits == {'joint_link2': (-.12, .14)}
    assert limited_bridge.joint_limits_from_json('', 'arm') == limited_bridge.joint_limits
    assert limited_bridge.joint_limits_from_json('{}', 'arm') == limited_bridge.joint_limits


@pytest.mark.parametrize('payload', [
    '{bad', 'null', '[]', 'false', '{"joint_limits":{}}',
    '{"model":"other","joint_limits":{}}',
    '{"model":"arm","joint_limits":[],"extra":1}',
    '{"model":"arm","joint_limits":{"unknown":{"lower":-1,"upper":1}}}',
    '{"model":"arm","joint_limits":{"joint_link2":null}}',
    '{"model":"arm","joint_limits":{"joint_link2":{"lower":false,"upper":1}}}',
    '{"model":"arm","joint_limits":{"joint_link2":{"lower":0,"upper":0}}}',
    '{"model":"arm","joint_limits":{"joint_link2":{"lower":1,"upper":0}}}',
    '{"model":"arm","joint_limits":{"joint_link2":{"lower":"-.5","upper":1}}}',
    '{"model":"arm","joint_limits":{"joint_link2":{"lower":-1,"upper":Infinity}}}',
    json.dumps({'model': 'arm', 'joint_limits': {'joint_link2': {'lower': -1, 'upper': 10**1000}}}),
])
def test_invalid_joint_limit_override_preserves_defaults(limited_bridge, payload):
    with pytest.raises(BridgeError):
        limited_bridge.joint_limits_from_json(payload, 'arm')
    assert limited_bridge.joint_limits == {'joint_link2': (-.12, .14)}


def test_partial_override_keeps_other_joint_limits(bridge):
    bridge.joint_limits = {'yaw': (-1., 1.), 'pitch': (-.1, .1)}
    payload = json.dumps({
        'model': 'arm', 'joint_limits': {'yaw': {'lower': -2., 'upper': 2.}},
    })
    assert bridge.joint_limits_from_json(payload, 'arm') == {
        'yaw': (-2., 2.), 'pitch': (-.1, .1),
    }
