"""差动组的反馈新鲜度与成对轨迹；不启动 ROS 或硬件。"""
from pathlib import Path

import pytest
from motor_hardware_interface.transmission_bridge import TransmissionBridge
from motor_hardware_interface.topic_bridge import BridgeError


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
