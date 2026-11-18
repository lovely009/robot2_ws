"""HTTP adapter for full-resolution motion playback data."""

from io import BytesIO
import json
from pathlib import Path

import numpy as np
import pytest

from web_host.motion import MotionJobs
from web_host.motion_playback import handle_motion_playback


class Handler:
    def __init__(self):
        self.command = 'GET'
        self.path = '/api/motion/jobs'
        self.wfile = BytesIO()
        self.code = None

    def send_response(self, code):
        self.code = code

    def send_header(self, *args):
        pass

    def end_headers(self):
        pass


def make_job(jobs, name='joint_a', frames=1001, frame_time=0.01, status='done'):
    directory = Path(jobs.directory.name) / 'playback-fixture'
    directory.mkdir()
    positions = np.linspace(-1, 1, frames * 2, dtype=np.float64).reshape(frames, 2)
    np.savez_compressed(directory / 'motion.npz', positions=positions,
                        joint_names=[name, name + '_b'], frame_time=frame_time)
    jobs.jobs['fixture'] = {'directory': directory,
                            'status': {'state': status, 'progress': 100}}
    return positions


def test_http_returns_every_full_resolution_frame():
    manager = MotionJobs()
    try:
        expected = make_job(manager)
        handler = Handler()
        handler.path = '/api/motion/jobs/fixture/trajectory'
        assert handle_motion_playback(handler, manager)
        assert handler.code == 200
        payload = json.loads(handler.wfile.getvalue())
        assert payload['joint_names'] == ['joint_a', 'joint_a_b']
        assert payload['frame_time'] == pytest.approx(0.01)
        assert len(payload['positions']) == 1001
        np.testing.assert_allclose(np.asarray(payload['positions']), expected)
        assert payload['positions'][-1] == expected[-1].tolist()
    finally:
        manager.close()


@pytest.mark.parametrize('status', ['running', 'failed'])
def test_http_rejects_incomplete_job(status):
    manager = MotionJobs()
    try:
        make_job(manager, status=status)
        handler = Handler()
        handler.path = '/api/motion/jobs/fixture/trajectory'
        assert handle_motion_playback(handler, manager)
        assert handler.code == 400
        assert '尚未完成' in json.loads(handler.wfile.getvalue())['error']
    finally:
        manager.close()


def test_http_rejects_missing_job():
    manager = MotionJobs()
    try:
        handler = Handler()
        handler.path = '/api/motion/jobs/missing/trajectory'
        assert handle_motion_playback(handler, manager)
        assert handler.code == 404
        assert json.loads(handler.wfile.getvalue())['error']
    finally:
        manager.close()


@pytest.mark.parametrize('mutate', [
    lambda npz: np.savez_compressed(npz, positions=[[.1, .2]],
                                    joint_names=[''], frame_time=.01),
    lambda npz: np.savez_compressed(npz, positions=[[.1, .2], [.3, .4]],
                                    joint_names=['joint_a', 'joint_a'], frame_time=.01),
    lambda npz: np.savez_compressed(npz, positions=[[.1], [.2]],
                                    joint_names=['joint_a', 'joint_b'], frame_time=.01),
    lambda npz: np.savez_compressed(npz, positions=[[.1, np.nan]],
                                    joint_names=['joint_a', 'joint_b'], frame_time=.01),
    lambda npz: np.savez_compressed(npz, positions=[[.1, .2]],
                                    joint_names=['joint_a', 'joint_b'], frame_time=0),
    lambda npz: np.savez_compressed(npz, positions=np.full((2, 2), .1),
                                    joint_names=['joint_a', 'joint_b'],
                                    frame_time=1e100),
], ids=['empty-name', 'duplicate-name', 'bad-shape', 'nonfinite', 'zero-frame-time',
        'duration-overflow'])
def test_http_rejects_invalid_motion_data(mutate):
    manager = MotionJobs()
    try:
        make_job(manager)
        mutate(Path(manager.directory.name) / 'playback-fixture' / 'motion.npz')
        handler = Handler()
        handler.path = '/api/motion/jobs/fixture/trajectory'
        assert handle_motion_playback(handler, manager)
        assert handler.code == 400
        assert json.loads(handler.wfile.getvalue())['error']
    finally:
        manager.close()


def test_http_rejects_corrupt_motion_data():
    manager = MotionJobs()
    try:
        make_job(manager)
        (Path(manager.directory.name) / 'playback-fixture' / 'motion.npz').write_bytes(b'not-a-zip')
        handler = Handler()
        handler.path = '/api/motion/jobs/fixture/trajectory'
        assert handle_motion_playback(handler, manager)
        assert handler.code == 400
        assert json.loads(handler.wfile.getvalue())['error']
    finally:
        manager.close()
