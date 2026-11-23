"""Live poses are delivered before the recording ends and keep a fixed T-pose reference."""

from pathlib import Path

import numpy as np
import pytest

from test_vdmocap import capture
from web_host.vdmocap_live_pose import LivePose


@pytest.mark.parametrize('model,targets', [('full_body_robot', 16), ('half_body_robot', 6)])
def test_live_follows_individual_frames_with_consistent_robot_geometry(model, targets, tmp_path):
    root = Path(__file__).resolve().parents[2] / 'robot_description/models'
    sample = capture()
    solver = LivePose(model, root, tmp_path, sample['joints'], sample['rest'])
    assert solver.scene['matched_targets'] == targets
    assert solver.scene['preview_only']
    a, b = [solver.frame(f) for f in sample['frames']]
    assert a['index'] == 17 and b['index'] == 19
    np.testing.assert_allclose(b['source'], sample['frames'][-1]['positions'], atol=5.1e-7)
    assert np.isfinite(b['joint_positions']).all()
    assert len(b['joint_positions']) == len(solver.scene['robot']['joint_names'])
    assert len(b['robot']) == len(solver.scene['robot']['names'])
    assert not np.allclose(a['root_quaternion'], b['root_quaternion'])
    assert not list(tmp_path.iterdir())


def test_arm_movement_changes_robot_joints_and_limits_each_update(tmp_path):
    from scipy.spatial.transform import Rotation
    from test_vdmocap import NAMES, PARENTS, REST

    root = Path(__file__).resolve().parents[2] / 'robot_description/models'
    live = LivePose('full_body_robot', root, tmp_path, NAMES, REST.tolist())
    def pose(index, degrees):
        rotations = [Rotation.identity() for _ in NAMES]
        for name in ('LeftLowerArm', 'LeftHand'):
            rotations[NAMES.index(name)] = Rotation.from_euler('xyz', [degrees, degrees / 2, 0], degrees=True)
        points = REST.copy()
        for i, parent in enumerate(PARENTS[1:], 1):
            points[i] = points[parent] + rotations[parent].apply(REST[i] - REST[parent])
        return {'index': index, 'frequency': 60, 'positions': points.tolist(),
                'quaternions': [np.roll(r.as_quat(), 1).tolist() for r in rotations]}

    base = live.frame(pose(21, 0))
    poses = []
    for i in range(22, 47):
        prev = live.q.copy()
        f = live.frame(pose(i, min(40, (i - 21) * 4)))
        assert np.all(np.abs(live.q - prev) <= live.solver.velocity / 60 + 1e-7)
        assert np.all(live.q >= live.solver.lower - 1e-7) and np.all(live.q <= live.solver.upper + 1e-7)
        poses.append(f['joint_positions'])
    assert np.max(np.abs(np.asarray(poses[-1]) - base['joint_positions'])) > .1
    # Invalid packets cannot change the warm start or its source clock.
    previous = live.q.copy()
    bad = pose(47, 40)
    bad['positions'][1][2] += .2
    with pytest.raises(ValueError):
        live.frame(bad)
    np.testing.assert_array_equal(live.q, previous)
    assert live.previous_index == 46
    # Rewinding playback reacquires the same pose as joining at that index.
    reset = live.frame(pose(10, 0))
    np.testing.assert_allclose(reset['joint_positions'], base['joint_positions'], atol=1e-7)


def test_sdk_stream_survives_pause_disconnect_and_rewind(monkeypatch, tmp_path):
    from contextlib import contextmanager
    import threading
    from types import SimpleNamespace
    from web_host import vdmocap_live as transport, vdmocap_live_pose as pose

    source = capture()
    channel = {'index': 8}
    closed = threading.Event()
    class SDK:
        MocapData = SimpleNamespace

        @staticmethod
        def udp_recv_mocap_data(_i, _ip, _port, data):
            idx = channel['index']
            if idx is None:
                return False
            data.isUpdate, data.frameIndex, data.frequency = True, idx, 60
            data.position_body = source['frames'][0]['positions']
            data.quaternion_body = source['frames'][0]['quaternions']
            return True

        @staticmethod
        def udp_send_request_connect(*_):
            return True

    @contextmanager
    def connection(*_):
        try:
            yield SDK, (0, '127.0.0.1', 7000), source['joints'], source['rest']
        finally:
            closed.set()

    class Pose:
        scene = {'preview_only': True}

        def __init__(self, *_):
            pass

        def frame(self, frame, **_):
            return {'index': frame['index']}

    monkeypatch.setattr(transport, 'body_connection', connection)
    monkeypatch.setattr(pose, 'LivePose', Pose)
    stream = transport.LiveStream(tmp_path, '127.0.0.1', 7000)
    states, indices = [], []
    def emit(item):
        if item['type'] == 'frame':
            indices.append(item['index'])
            if item['index'] == 1:
                stream.shutdown.set()
        if item['type'] == 'state':
            states.append(item['state'])
            if item['state'] == 'paused':
                channel['index'] = None
            if item['state'] == 'disconnected':
                channel['index'] = 1

    # Bound a failed transition so a regression cannot hang the test runner.
    timer = threading.Timer(8, stream.shutdown.set)
    timer.start()
    try:
        stream.run('test', SimpleNamespace(models_root=None, profiles_dir=None), emit, threading.Event())
    finally:
        timer.cancel()
    assert indices == [8, 1] and closed.is_set()
    assert states[0] == 'waiting' and 'paused' in states and 'disconnected' in states


def test_source_clock_rejects_late_packet_recovers_at_same_index_and_accepts_real_rewind():
    from web_host.vdmocap_live import _FrameClock

    clock = _FrameClock()
    assert clock.accept(100, 60, 0) and clock.accept(102, 60, .02)
    assert not clock.accept(101, 60, .03)
    assert clock.index == 102 and clock.segment == 1
    assert not clock.accept(102, 60, .04)
    assert clock.accept(102, 60, 3) and clock.segment == 2
    assert not clock.accept(0, 60, 3.1)
    assert clock.accept(1, 60, 3.12) and clock.segment == 3
    clock = _FrameClock()
    assert clock.accept(2**32 - 1, 60, 0) and clock.accept(0, 60, .02)
    assert clock.segment == 1


def test_restart_preempts_only_the_same_tab_and_waits_for_udp_release(tmp_path):
    import threading
    from web_host.vdmocap_live import LiveStream

    stream = LiveStream(tmp_path, '127.0.0.1', 7000)
    first = stream.acquire('first')
    assert stream.acquire('second') is None and not first.is_set()
    def close_original():
        assert first.wait(1)
        stream.release()

    old = threading.Thread(target=close_original)
    old.start()
    new = stream.acquire('first')
    old.join(2)
    assert first.is_set() and new and not new.is_set()
    stream.release()
    stream.close()
    assert stream.acquire('third') is None
