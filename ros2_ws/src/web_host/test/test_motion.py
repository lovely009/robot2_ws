"""Conversion jobs exercise real IK on a short clip, with no ROS graph or subprocess."""
from email.message import Message
from io import BytesIO
import json
from pathlib import Path
import threading

import pytest

pytest.importorskip('pinocchio')
pytest.importorskip('bvh_broadcaster')
from web_host.motion import MotionJobs, handle_motion


@pytest.fixture
def payload():
    hierarchy = 'HIERARCHY\nROOT Spine3 { OFFSET 0 0 0 CHANNELS 6 Xposition Yposition Zposition Yrotation Xrotation Zrotation\n'
    for side, sign in [('Right', -1), ('Left', 1)]:
        for part, offset in [('UpperArm', 10), ('LowerArm', 30), ('Hand', 30)]:
            hierarchy += f'JOINT {side}{part} {{ OFFSET {sign * offset} 0 0 CHANNELS 3 Yrotation Xrotation Zrotation\n'
        hierarchy += '} } }\n'
    hierarchy += '}\nMOTION\nFrames: 3\nFrame Time: 0.02\n'
    rows = (' '.join(['0', '100'] + ['0'] * 22) + '\n') * 3
    urdf = Path(__file__).resolve().parents[2] / 'robot_sim/models/half_body_robot/urdf/half_body_robot.urdf'
    return {'bvh_text': hierarchy + rows, 'urdf_text': urdf.read_text(),
            'bvh_name': '测试.bvh', 'urdf_name': 'half_body_robot.urdf'}


@pytest.fixture
def jobs():
    manager = MotionJobs()
    yield manager
    manager.close()


def finish(jobs, job):
    jobs.thread.join(timeout=10)
    return jobs.status(job['id'])


def test_import_convert_preview_and_download(jobs, payload):
    job = jobs.create(payload)
    done = finish(jobs, job)
    assert done['state'] == 'done', done
    assert done['progress'] == 100 and done['joint_count'] == 18
    preview = json.loads(jobs.result(job['id'], 'preview.json'))
    assert len(preview['source']['frames']) == 3
    assert len(preview['robot']['frames'][0]) == 19
    assert '<robot' in preview['robot']['urdf']
    assert len(preview['robot']['joint_names']) == 18
    assert len(preview['robot']['joint_positions']) == len(preview['times'])
    assert all(len(q) == 18 for q in preview['robot']['joint_positions'])
    assert preview['duration'] == pytest.approx(.04)
    report = json.loads(jobs.result(job['id'], 'report.json'))
    assert report['preview_only'] and len(report['joint_names']) == 18
    assert jobs.result(job['id'], 'motion.npz').startswith(b'PK')
    with pytest.raises(FileNotFoundError):
        jobs.result(job['id'], '../robot.urdf')
    snapshot = jobs.status(job['id'])
    snapshot['mapping']['torso'] = 'changed'
    assert jobs.status(job['id'])['mapping']['torso'] == 'Spine3'


def test_unknown_model_requests_mapping_then_converts(jobs, payload):
    payload['urdf_text'] = payload['urdf_text'].replace('Joint_link4"', 'renamed_shoulder"')
    job = jobs.create(payload)
    assert job['state'] == 'mapping' and job['mapping'] is None
    import yaml
    template = Path(__file__).resolve().parents[2] / 'bvh_broadcaster/config/half_body_robot.yaml'
    mapping = yaml.safe_load(template.read_text())
    mapping['arms']['right']['robot'][0] = 'renamed_shoulder'
    jobs.start(job['id'], mapping)
    assert finish(jobs, job)['state'] == 'done'


def test_conversion_failure_is_terminal(jobs, payload, monkeypatch):
    def fail(*args):
        raise ValueError('fixture IK failure')
    monkeypatch.setattr('bvh_broadcaster.retarget.ArmRetargeter.convert', fail)
    done = finish(jobs, jobs.create(payload))
    assert done['state'] == 'failed' and done['error'] == 'fixture IK failure'
    with pytest.raises(ValueError, match='尚未完成'):
        jobs.result(done['id'], 'preview.json')


def test_only_one_conversion_can_run(jobs, payload, monkeypatch):
    started, release = threading.Event(), threading.Event()
    def blocked(*args):
        started.set()
        release.wait(timeout=5)
        raise ValueError('released')
    monkeypatch.setattr('bvh_broadcaster.retarget.ArmRetargeter.convert', blocked)
    try:
        job = jobs.create(payload)
        assert started.wait(timeout=2)
        with pytest.raises(ValueError, match='已有转换'):
            jobs.create(payload)
        with pytest.raises(ValueError, match='已开始'):
            jobs.start(job['id'], job['mapping'])
    finally:
        release.set()


@pytest.mark.parametrize('field,value,match', [
    ('bvh_text', '', '不能为空'), ('urdf_text', '<robot/>', '完整关节树'),
    ('urdf_text', '<!DOCTYPE robot><robot/>', 'DTD'), ('up_axis', 'bad', '坐标或单位')])
def test_invalid_import_rejected(jobs, payload, field, value, match):
    payload[field] = value
    with pytest.raises(ValueError, match=match):
        jobs.create(payload)


class Handler:
    command = 'POST'
    path = '/api/motion/jobs'
    def __init__(self, body=b'{}', origin='http://localhost', content_type='application/json'):
        self.headers = Message()
        for k, v in {'Host': 'localhost', 'Origin': origin, 'Content-Type': content_type,
                     'Content-Length': str(len(body))}.items():
            self.headers[k] = v
        self.rfile, self.wfile = BytesIO(body), BytesIO()
        self.code = None
    def send_response(self, code): self.code = code
    def send_header(self, *args): pass
    def end_headers(self): pass


@pytest.mark.parametrize('origin,content_type,code', [
    ('http://other.invalid', 'application/json', 403), ('http://localhost', 'text/plain', 415)])
def test_http_upload_boundary(jobs, origin, content_type, code):
    handler = Handler(origin=origin, content_type=content_type)
    assert handle_motion(handler, jobs) and handler.code == code


def test_http_missing_task_and_other_routes(jobs):
    handler = Handler()
    handler.command = 'GET'
    handler.path = '/api/motion/jobs/missing'
    assert handle_motion(handler, jobs) and handler.code == 404
    handler.path = '/api/actions'
    assert not handle_motion(handler, jobs)
