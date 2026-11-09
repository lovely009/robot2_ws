"""差动组的反馈新鲜度与成对轨迹；不启动 ROS 或硬件。"""
from pathlib import Path
import json

import pytest
from motor_hardware_interface.transmission_bridge import TransmissionBridge
from motor_hardware_interface.topic_bridge import BridgeError


def _spatial_bridge(tmp_path):
    _bridge(tmp_path)
    (tmp_path / 'transmissions.yaml').write_text(
        (Path(__file__).parent / 'fixtures/spatial_transmissions.yaml').read_text())
    return TransmissionBridge(str(tmp_path / 'motor_driver.yaml'))


def test_nonlinear_trajectory_rejects_nonzero_effort_atomically(tmp_path):
    bridge = _spatial_bridge(tmp_path)
    bridge.update(_states(), 1.)
    bridge.trajectory(['yaw', 'pitch'], [dict(t_ms=100, positions=[.1, .05])], 1.1)
    held = dict(bridge.held)
    with pytest.raises(BridgeError, match='实时姿态'):
        bridge.trajectory(['yaw', 'pitch'], [
            dict(t_ms=100, positions=[.2, .08], effort=[0., 0.]),
            dict(t_ms=200, positions=[.2, .08], effort=[0., 1.]),
        ], 1.1)
    assert bridge.held == held
    ids, flat, _ = bridge.trajectory(['yaw', 'pitch'], [
        dict(t_ms=100, positions=[.2, .08], effort=[0., 0.])], 1.1)
    assert ids == [8, 9] and all(v[4] == 0 for v in flat)


def test_nonlinear_feedback_requires_complete_synchronised_group(tmp_path):
    bridge = _spatial_bridge(tmp_path)
    assert not bridge.update(_states()[:1], 1.)
    assert len(bridge.update(_states(), 1.)) == 2
    bridge.trajectory(['yaw'], [dict(t_ms=100, positions=[.3])], 1.1)
    assert not bridge.update(_states(age_b=51), 2.)
    assert not bridge.held
    with pytest.raises(BridgeError):
        bridge.trajectory(['yaw'], [dict(t_ms=100, positions=[.3])], 2.1)
    assert not bridge.update(_states(age_b=700), 3.)
    observed = bridge.update(_states(.1, .1), 4.)
    _, flat, _ = bridge.trajectory(['pitch'], [dict(t_ms=100, positions=[.05])], 4.1)
    assert flat[0][2] == pytest.approx(observed['yaw'].position)
    with pytest.raises(BridgeError):
        bridge.trajectory(['pitch'], [dict(t_ms=100, positions=[.05])], 5.)


def test_nonlinear_geometric_feedback_failure_invalidates_held_and_recovers(tmp_path):
    bridge = _spatial_bridge(tmp_path)
    bridge.update(_states(), 1.)
    bridge.trajectory(['yaw'], [dict(t_ms=100, positions=[.3])], 1.1)
    assert bridge.held['yaw'] == .3
    assert not bridge.update(_states(.2, 4.), 1.2)
    assert not bridge.held and bridge.state_stamp is None
    with pytest.raises(BridgeError):
        bridge.trajectory(['pitch'], [dict(t_ms=100, positions=[.05])], 1.25)
    observed = bridge.update(_states(.1, .1), 1.3)
    _, flat, _ = bridge.trajectory(['pitch'], [dict(t_ms=100, positions=[.05])], 1.4)
    assert flat[0][2] == pytest.approx(observed['yaw'].position)
    assert flat[0][2] == pytest.approx(.1)


def test_linear_effort_trajectory_remains_supported(tmp_path):
    bridge = _bridge(tmp_path)
    bridge.update(_states(), 1.)
    _, flat, _ = bridge.trajectory(['yaw', 'pitch'], [
        dict(t_ms=100, positions=[.2, .1], effort=[2., 4.])], 1.1)
    assert [v[4] for v in flat] == pytest.approx([3., -1.])


def _bridge(tmp_path: Path) -> TransmissionBridge:
    (tmp_path/'motor_driver.yaml').write_text('motor_hardware:\n  motors:\n    a: {id: 8, joint: yaw}\n    b: {id: 9, joint: pitch}\n', encoding='utf-8')
    (tmp_path/'transmissions.yaml').write_text('transmissions:\n  wrist:\n    plugin: transmission_interface/DifferentialTransmission\n    actuators: [a,b]\n    joints: [yaw,pitch]\n', encoding='utf-8')
    return TransmissionBridge(str(tmp_path/'motor_driver.yaml'))


def _states(a: float = .2, b: float = .2, age_b: int = 0) -> list[dict]:
    return [dict(motor_id=8, position=a, velocity=0., effort=2., state_age_ms=0),
            dict(motor_id=9, position=b, velocity=0., effort=2., state_age_ms=age_b)]


def test_feedback_needs_fresh_complete_group(tmp_path: Path) -> None:
    bridge=_bridge(tmp_path)
    assert bridge.update(_states()[:1],1.) == {}
    states=bridge.update(_states(),1.)
    assert states['yaw'].position == pytest.approx(.2)
    assert states['pitch'].position == pytest.approx(0.)
    assert bridge.update(_states(age_b=700),2.) == {}
    assert bridge.update(_states(age_b=100),3.) == {}  # 超过同组采样时间差


def test_sparse_joint_target_preserves_observed_partner_and_pairs_times(tmp_path: Path) -> None:
    bridge=_bridge(tmp_path)
    with pytest.raises(BridgeError):
        bridge.trajectory(['yaw'],[dict(t_ms=100,positions=[.3])],1.)
    bridge.update(_states(.3,.1),1.)  # yaw=.2, pitch=.1
    ids,flat,warnings=bridge.trajectory(['yaw'],[dict(t_ms=100,positions=[.4])],1.1)
    assert ids == [8,9]
    assert flat == [(100,8,.5,0.,0.),(100,9,pytest.approx(.3),0.,0.)]
    assert not warnings
    with pytest.raises(BridgeError):
        bridge.trajectory(['yaw'],[dict(t_ms=100,positions=[.3])],2.)


def test_bad_shapes_and_partial_effects_do_not_advance_held_target(tmp_path: Path) -> None:
    bridge=_bridge(tmp_path); bridge.update(_states(),1.)
    with pytest.raises(BridgeError):
        bridge.trajectory(['yaw','pitch'],[dict(t_ms=100,positions=[.3,.1],velocities=[0.])],1.1)
    with pytest.raises(BridgeError):
        bridge.trajectory(['yaw','yaw'],[dict(t_ms=100,positions=[.3,.1])],1.1)
    ids,flat,_=bridge.trajectory(['pitch'],[dict(t_ms=100,positions=[.1])],1.1)
    assert ids == [8,9]
    assert flat[0][2] == pytest.approx(.3)
    assert flat[1][2] == pytest.approx(.1)


def test_group_rejects_combined_actuator_range_and_uses_measurement_time(tmp_path: Path) -> None:
    bridge=_bridge(tmp_path)
    bridge.update(_states(age_b=25),1.)
    assert bridge.state_stamp == pytest.approx(.975)
    with pytest.raises(BridgeError):
        bridge.trajectory(['yaw','pitch'],[dict(t_ms=100,positions=[8.,8.])],1.1)
    assert not bridge.held
    with pytest.raises(BridgeError):
        bridge.trajectory(['yaw','pitch'],[dict(t_ms=100,positions=[0.,0.])],2.)


def test_recovered_group_does_not_reuse_target_from_previous_feedback_epoch(tmp_path: Path) -> None:
    bridge=_bridge(tmp_path)
    bridge.update(_states(.3,.1),1.)
    bridge.trajectory(['yaw'],[dict(t_ms=100,positions=[.4])],1.1)
    bridge.update(_states(.9,.5),2.)  # 断链后 yaw=.7，不应保留旧目标 .4
    _,flat,_=bridge.trajectory(['pitch'],[dict(t_ms=100,positions=[.1])],2.1)
    assert flat[0][2] == pytest.approx(.8)
    assert flat[1][2] == pytest.approx(.6)


def _limited_bridge(tmp_path: Path) -> TransmissionBridge:
    driver = tmp_path/'motor_driver.yaml'
    driver.write_text('motor_hardware:\n  motors:\n    m2: {id: 2, joint: joint_link2}\n', encoding='utf-8')
    urdf = tmp_path/'robot.urdf'
    urdf.write_text('<robot name="arm"><joint name="joint_link2" type="revolute">'
                    '<limit lower="-0.12" upper="0.14"/></joint></robot>', encoding='utf-8')
    bridge = TransmissionBridge(str(driver), urdf_file=str(urdf))
    bridge.update([dict(motor_id=2, position=0., velocity=0., effort=0., state_age_ms=0)], 1.)
    return bridge


def test_explicit_joint_limits_override_urdf_without_disabling_range_checks(tmp_path: Path) -> None:
    bridge = _limited_bridge(tmp_path)
    sample = [dict(t_ms=100, positions=[-.249])]
    with pytest.raises(BridgeError, match='限位'):
        bridge.trajectory(['joint_link2'], sample, 1.1)
    limits = bridge.joint_limits_from_json(json.dumps({
        'model': 'arm', 'joint_limits': {'joint_link2': {'lower': -.5, 'upper': .14}},
    }), 'arm')
    assert limits == {'joint_link2': (-.5, .14)}
    _, flat, _ = bridge.trajectory(['joint_link2'], sample, 1.1, limits=limits)
    assert flat == [(100, 2, -.249, 0., 0.)]
    assert bridge.joint_limits == {'joint_link2': (-.12, .14)}
    assert bridge.joint_limits_from_json('', 'arm') == bridge.joint_limits
    assert bridge.joint_limits_from_json('{}', 'arm') == bridge.joint_limits
    with pytest.raises(BridgeError, match='限位'):
        bridge.trajectory(['joint_link2'], [dict(t_ms=100, positions=[-.501])], 1.1, limits=limits)
    for boundary in (-.5, .14):
        bridge.trajectory(['joint_link2'], [dict(t_ms=100, positions=[boundary])], 1.1, limits=limits)
    with pytest.raises(BridgeError, match='限位'):
        bridge.trajectory(['joint_link2'], sample, 1.1)


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
    '{"model":"arm","joint_limits":{"joint_link2":{"lower":NaN,"upper":1}}}',
    '{"model":"arm","joint_limits":{"joint_link2":{"lower":-1,"upper":Infinity}}}',
    '{"model":"arm","joint_limits":{"joint_link2":{"lower":-1}}}',
    '{"model":"arm","joint_limits":{"joint_link2":{"lower":-1,"upper":1,"extra":0}}}',
    json.dumps({'model': 'arm', 'joint_limits': {'joint_link2': {'lower': -1, 'upper': 10**1000}}}),
])
def test_invalid_joint_limit_overrides_do_not_change_defaults(tmp_path: Path, payload: str) -> None:
    bridge = _limited_bridge(tmp_path)
    with pytest.raises(BridgeError):
        bridge.joint_limits_from_json(payload, 'arm')
    assert bridge.joint_limits == {'joint_link2': (-.12, .14)}


def test_override_keeps_unmodified_group_limits_and_actuator_wire_range(tmp_path: Path) -> None:
    bridge = _bridge(tmp_path)
    bridge.joint_limits = {'yaw': (-1., 1.), 'pitch': (-.1, .1)}
    limits = bridge.joint_limits_from_json(json.dumps({
        'model': 'arm', 'joint_limits': {'yaw': {'lower': -20., 'upper': 20.}},
    }), 'arm')
    assert limits['pitch'] == (-.1, .1)
    bridge.update(_states(.3, .1), 1.)
    with pytest.raises(BridgeError, match='限位'):
        bridge.trajectory(['yaw', 'pitch'], [dict(t_ms=100, positions=[0., .2])], 1.1, limits=limits)
    with pytest.raises(BridgeError, match='线上量程'):
        bridge.trajectory(['yaw'], [dict(t_ms=100, positions=[13.])], 1.1, limits=limits)
    assert not bridge.held
