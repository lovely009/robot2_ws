"""SDK world poses must survive BVH FK and the normal saved-action lookup."""

import copy
import json

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from bvh_broadcaster.bvh import BvhClip
from web_host.vdmocap import body_bvh


NAMES = 'Hips RightUpperLeg RightLowerLeg RightFoot RightToe LeftUpperLeg LeftLowerLeg LeftFoot LeftToe Spine Spine1 Spine2 Spine3 Neck Head RightShoulder RightUpperArm RightLowerArm RightHand LeftShoulder LeftUpperArm LeftLowerArm LeftHand'.split()
PARENTS = [-1, 0, 1, 2, 3, 0, 5, 6, 7, 0, 9, 10, 11, 12, 13, 12, 15, 16, 17, 12, 19, 20, 21]
REST = np.array([[0, 0, 1], [.1, 0, .98], [.1, 0, .55], [.1, 0, .12], [.1, .15, .04],
                 [-.1, 0, .98], [-.1, 0, .55], [-.1, 0, .12], [-.1, .15, .04],
                 [0, 0, 1.1], [0, 0, 1.2], [0, 0, 1.3], [0, 0, 1.4], [0, 0, 1.5], [0, 0, 1.65],
                 [.1, 0, 1.45], [.2, 0, 1.45], [.45, 0, 1.45], [.65, 0, 1.45],
                 [-.1, 0, 1.45], [-.2, 0, 1.45], [-.45, 0, 1.45], [-.65, 0, 1.45]])


def capture():
    frames = []
    for index, angle in [(17, 10), (19, 35)]:
        rotations = Rotation.from_euler('xyz', [[4, -3, angle + i] for i in range(23)], degrees=True)
        points = np.empty_like(REST)
        points[0] = [.4, index / 100, .96]
        for i, parent in enumerate(PARENTS[1:], 1):
            points[i] = points[parent] + rotations[parent].apply(REST[i] - REST[parent])
        frames.append({'index': index, 'frequency': 60, 'positions': points.tolist(),
                       'quaternions': rotations.as_quat()[:, [3, 0, 1, 2]].tolist()})
    return {'world_space': 'WS_Geo', 'joints': NAMES[:], 'rest': REST.tolist(), 'frames': frames}


def test_world_pose_round_trip_after_joint_reordering_and_packet_loss(tmp_path):
    data = capture()
    reverse = copy.deepcopy(data)
    for items in [reverse['joints'], reverse['rest'], *(a[k] for a in reverse['frames'] for k in ('positions', 'quaternions'))]:
        items.reverse()
    text, info = body_bvh(reverse)
    path = tmp_path / 'body.bvh'
    path.write_text(text)
    clip = BvhClip.load(path, preserve_names=True)
    assert info['interpolated_frames'] == 1 and len(clip.frames) == 3
    assert clip.frame_time == pytest.approx(1 / 60)
    for index, source in [(0, data['frames'][0]), (2, data['frames'][1])]:
        points, rotations = [], []
        for joint, (p, q) in zip(clip.joints, clip.local_transforms(index, .01, 'z_up')):
            r = Rotation.from_quat(q)
            if joint.parent >= 0:
                p = points[joint.parent] + rotations[joint.parent].apply(p)
                r = rotations[joint.parent] * r
            points.append(p)
            rotations.append(r)
            if not joint.is_end:
                i = NAMES.index(joint.name)
                np.testing.assert_allclose(p, source['positions'][i], atol=1e-7)
                expected = Rotation.from_quat(np.roll(source['quaternions'][i], -1))
                assert (r.inv() * expected).magnitude() < 1e-7
    np.testing.assert_allclose(clip.frames[1, :3], np.mean(clip.frames[[0, 2], :3], axis=0))


@pytest.mark.parametrize('invalid', ['names', 'shape', 'nan', 'quaternion', 'geometry', 'units', 'space', 'frequency', 'duplicate', 'backward', 'gap'])
def test_rejects_invalid_recording(invalid):
    data = capture()
    f = data['frames'][1]
    if invalid == 'names': data['joints'][1] = data['joints'][2]
    if invalid == 'shape': f['positions'].pop()
    if invalid == 'nan': f['positions'][0][0] = float('nan')
    if invalid == 'quaternion': f['quaternions'][3] = [0] * 4
    if invalid == 'geometry': f['positions'][1][1] += .1
    if invalid == 'units': data['rest'] = (REST * 100).tolist()
    if invalid == 'space': data['world_space'] = 'WS_Unity'
    if invalid == 'frequency': f['frequency'] = 30
    if invalid == 'duplicate': f['index'] = 17
    if invalid == 'backward': f['index'] = 16
    if invalid == 'gap': f['index'] = 100
    with pytest.raises(ValueError):
        body_bvh(data)


def test_frame_counter_wrap_and_rejection_of_scaled_or_non_numeric_capture():
    data = capture()
    data['frames'][0]['index'] = 2**32 - 1
    data['frames'][1]['index'] = 1
    assert body_bvh(data)[1]['frames'] == 3

    invalid = copy.deepcopy(data)
    invalid['rest'] = (REST * 100).tolist()
    for frame in invalid['frames']:
        frame['positions'] = (np.array(frame['positions']) * 100).tolist()
    with pytest.raises(ValueError, match='米'):
        body_bvh(invalid)
    data['frames'][0]['quaternions'][0][0] = True
    with pytest.raises(ValueError, match='quaternions'):
        body_bvh(data)


def test_import_is_saved_for_normal_workbench_reopen_and_retry(tmp_path):
    from io import BytesIO
    from pathlib import Path
    from web_host.motion import MotionJobs
    from web_host.motion_catalog import source_statuses
    from web_host.vdmocap_import import import_capture

    models = Path(__file__).resolve().parents[2] / 'robot_description/models'
    opts = dict(actions_dir=tmp_path/'library', profiles_dir=tmp_path/'profiles',
                models_root=models, cache_dir=tmp_path/'results')
    request = {'name': '动作.bvh', 'capture': capture()}
    jobs = MotionJobs(**opts)
    try:
        result = import_capture(jobs, request)
        assert result['matched_targets'] == 16 and result['joint_count'] == 37
        assert result['preview_only'] and result['frames'] == 3
        assert import_capture(jobs, request)['result_cached']
        original = (opts['actions_dir']/request['name']).read_bytes()
        different = copy.deepcopy(request)
        different['capture']['frames'][1]['positions'] = (
            np.asarray(different['capture']['frames'][1]['positions']) + [.01, 0, 0]).tolist()
        with pytest.raises(FileExistsError):
            import_capture(jobs, different)
        assert original == (opts['actions_dir']/request['name']).read_bytes()
    finally:
        jobs.close()

    jobs = MotionJobs(**opts)
    try:
        params = {'model': 'full_body_robot', 'retargeting': 'generic', 'up_axis': 'auto', 'scale': .01,
                  'urdf_text': (models/'full_body_robot/urdf/full_body_robot.urdf').read_text()}
        assert source_statuses(jobs, params) == [{'name': request['name'], 'saved': True}]
        job = jobs.create({**params, 'bvh_source': request['name']})
        assert job['result_cached'] and job['mapping_origin'] == 'saved'
        with np.load(BytesIO(jobs.result(job['id'], 'motion.npz')), allow_pickle=False) as motion:
            assert motion['positions'].shape == (3, 37) and np.isfinite(motion['positions']).all()
            assert motion['frame_time'] == pytest.approx(1 / 60)
    finally:
        jobs.close()


def test_failed_solve_or_persistence_does_not_publish_source(tmp_path, monkeypatch):
    from pathlib import Path
    from web_host.motion import MotionJobs
    from web_host.vdmocap_import import import_capture

    root = Path(__file__).resolve().parents[2] / 'robot_description/models'
    jobs = MotionJobs(tmp_path/'actions', tmp_path/'profiles', root, tmp_path/'cache')

    def fail_save(*args):
        raise OSError('cache unavailable')

    try:
        monkeypatch.setattr(jobs.cache, 'save', fail_save)
        with pytest.raises(OSError, match='cache unavailable'):
            import_capture(jobs, {'name': 'failed.bvh', 'capture': capture()})
        assert jobs.sources() == []
        assert not jobs.profiles_dir.exists()
    finally:
        jobs.close()


def test_new_playback_skips_frozen_frame_and_stops_before_rewind():
    from types import SimpleNamespace
    from web_host.vdmocap_record import read_frames

    indices = iter([250, 250, 0, 1, 2, 0])
    def receive(index, host, port, data):
        data.frameIndex = next(indices)
        return True

    data = SimpleNamespace(frequency=60, isUpdate=True, position_body=REST, quaternion_body=[[1, 0, 0, 0]] * 23)
    sdk = SimpleNamespace(MocapData=lambda: data, udp_recv_mocap_data=receive)
    frames = read_frames(sdk, (0, '127.0.0.1', 7000), 5)
    assert [frame['index'] for frame in frames] == [0, 1, 2]


@pytest.fixture
def import_jobs(tmp_path):
    from pathlib import Path
    from web_host.motion import MotionJobs

    root = Path(__file__).resolve().parents[2] / 'robot_description/models'
    jobs = MotionJobs(tmp_path/'library', tmp_path/'profiles', root, tmp_path/'results')
    try:
        yield jobs
    finally:
        jobs.close()


def test_builtin_arm_template_and_unsupported_model_preserve_shared_jobs(import_jobs):
    from web_host.vdmocap_import import ImportConflict, import_capture

    jobs = import_jobs
    req = {'name': '双臂.bvh', 'model': 'half_body_robot', 'capture': capture()}
    for _ in range(3):
        result = import_capture(jobs, req)
        assert result['matched_targets'] == 6 and result['joint_count'] == 18
    previous_ids = tuple(jobs.jobs)
    with pytest.raises(ValueError, match='缺少已确认'):
        import_capture(jobs, {**req, 'model': 'trial_arm_7dof'})
    assert tuple(jobs.jobs) == previous_ids
    assert all(jobs.result(i, 'motion.npz') for i in previous_ids)
    jobs.jobs[previous_ids[-1]]['status']['state'] = 'running'
    with pytest.raises(ImportConflict, match='等待完成'):
        import_capture(jobs, req)


def test_retry_with_old_mapping_uses_read_only_library_and_preserves_invalid_profile(import_jobs, monkeypatch):
    from web_host import vdmocap_import as importer

    jobs = import_jobs
    req = {'name': '动作.bvh', 'capture': capture()}
    importer.import_capture(jobs, req)
    path, = jobs.profiles_dir.glob('*.json')
    profile = json.loads(path.read_text())
    for target in profile['mapping']['targets']:
        if not target.get('source'):
            target.pop('source', None)
    path.write_text(json.dumps(profile))
    def forbid_write(*args):
        pytest.fail('A cached retry should not write the action or its confirmed mapping')

    monkeypatch.setattr(importer, 'write_action_text', forbid_write)
    assert importer.import_capture(jobs, req)['result_cached']
    profile['reference'] = [999]
    invalid = json.dumps(profile)
    path.write_text(invalid)
    with pytest.raises(importer.ImportConflict, match='重新确认'):
        importer.import_capture(jobs, {**req, 'name': '其他名称.bvh'})
    assert path.read_text() == invalid and jobs.sources() == ['动作.bvh']


def test_conflicting_mapping_saved_during_solve_is_preserved(import_jobs, monkeypatch):
    from web_host import vdmocap_import as importer

    jobs = import_jobs
    save = jobs.cache.save
    operator_record = '{"version": 1, "mapping": {"targets": []}, "reference": []}'

    def concurrent_save(key, source):
        save(key, source)
        job, = jobs.jobs.values()
        jobs.profiles_dir.mkdir()
        (jobs.profiles_dir / (job['profile_key'] + '.json')).write_text(operator_record)

    monkeypatch.setattr(jobs.cache, 'save', concurrent_save)
    with pytest.raises(importer.ImportConflict, match='映射发生变化'):
        importer.import_capture(jobs, {'name': '冲突.bvh', 'capture': capture()})
    assert jobs.sources() == []
    path, = jobs.profiles_dir.glob('*.json')
    assert path.read_text() == operator_record
