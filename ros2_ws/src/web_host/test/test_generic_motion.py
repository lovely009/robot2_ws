"""Generic jobs persist only confirmed mappings for the exact input identity."""

import json
from io import BytesIO
from pathlib import Path

import numpy as np
import pytest
import yaml

from web_host.motion import MotionJobs


def payload():
    return {'retargeting': 'generic', 'up_axis': 'z_up', 'scale': .01,
            'urdf_text': '''<robot name="new_arm"><link name="base"/><link name="LeftArm"/>
            <joint name="axis" type="revolute"><parent link="base"/><child link="LeftArm"/>
            <axis xyz="0 0 1"/><limit lower="-2" upper="2" velocity="100" effort="1"/>
            </joint></robot>''',
            'bvh_text': '''HIERARCHY
ROOT Hips { OFFSET 0 0 0 CHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation
 JOINT LeftArm { OFFSET 0 0 0 CHANNELS 3 Zrotation Xrotation Yrotation
  End Site { OFFSET 100 0 0 }
 }
}
MOTION
Frames: 3
Frame Time: 0.02
0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 30 0 0
0 0 0 0 0 0 60 0 0
'''}


def finish(jobs, job):
    jobs.thread.join(10)
    return jobs.status(job['id'])


def test_registered_full_body_uses_simulation_limits_and_moving_cad_preview():
    model = Path(__file__).resolve().parents[2] / 'robot_description/models/full_body_robot'
    source = (model / 'urdf/full_body_robot.urdf').read_text()
    data = payload() | {'model': 'full_body_robot', 'urdf_text': source,
                        'urdf_name': 'full_body_robot.urdf'}
    jobs = MotionJobs()
    try:
        first = jobs.create(data)
        assert first['constraint_source'] == 'simulation'
        assert first['joint_count'] == 37 and not first['warnings']
        jobs.start(first['id'], first['mapping'])
        done = finish(jobs, first)
        assert done['state'] == 'done', done.get('error', done)
        assert done['summary']['simulation_limits_enforced']
        assert not done['summary']['mechanical_limits_enforced']
        preview = json.loads(jobs.result(first['id'], 'preview.json'))
        assert len(preview['robot']['joint_names']) == 67
        assert 'left_wrist_flange_proximal' in preview['robot']['joint_names']
        assert len(preview['robot']['joint_positions'][0]) == 67
        with np.load(BytesIO(jobs.result(first['id'], 'motion.npz'))) as motion:
            assert motion['positions'].shape == (3, 37)
            assert bool(motion['simulation_limits_enforced'])
            columns = [preview['robot']['joint_names'].index(name) for name in motion['joint_names']]
            np.testing.assert_allclose(np.asarray(preview['robot']['joint_positions'])[:, columns],
                                       motion['positions'], atol=1e-8)
        assert (model / 'urdf/full_body_robot.urdf').read_text() == source
        assert 'velocity="3.141592653589793"' in jobs.result(first['id'], 'robot.urdf').decode()
    finally:
        jobs.close()


@pytest.mark.parametrize('up_axis,expected', [
    ('y_up', [[0, -1, 0], [1, 0, 0], [0, 0, 1]]),
    ('z_up', [[-1, 0, 0], [0, -1, 0], [0, 0, 1]]),
])
def test_full_body_preset_accounts_for_source_anatomical_heading(up_axis, expected):
    model = Path(__file__).resolve().parents[2] / 'robot_description/models/full_body_robot'
    vector = lambda x, h: f'{x} {h} 0' if up_axis == 'y_up' else f'{x} 0 {h}'
    names = [('Spine', 0, 10), ('Head', 0, 50), ('LeftUpperLeg', 10, 0), ('RightUpperLeg', -10, 0)]
    children = ''.join(f'JOINT {name} {{ OFFSET {vector(x, h)} CHANNELS 3 Zrotation Xrotation Yrotation }}\n'
                       for name, x, h in names)
    source = ('HIERARCHY\nROOT Hips { OFFSET 0 0 0 CHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation\n'
              + children + '}\nMOTION\nFrames: 1\nFrame Time: .02\n' + ' '.join(['0'] * 18) + '\n')
    jobs = MotionJobs()
    try:
        job = jobs.create(payload() | {'model': 'full_body_robot', 'up_axis': up_axis,
            'urdf_text': (model / 'urdf/full_body_robot.urdf').read_text(), 'bvh_text': source})
        np.testing.assert_allclose(job['mapping']['source_to_robot'], expected)
    finally:
        jobs.close()


def test_registered_model_rejects_changed_urdf_and_invalidates_changed_constraints(tmp_path):
    model = Path(__file__).resolve().parents[2] / 'robot_description/models/full_body_robot'
    directory = tmp_path / 'full_body_robot'
    (directory / 'urdf').mkdir(parents=True)
    (directory / 'config').mkdir()
    source = (model / 'urdf/full_body_robot.urdf').read_text()
    (directory / 'urdf/full_body_robot.urdf').write_text(source)
    config = yaml.safe_load((model / 'config/simulation_limits.yaml').read_text())
    path = directory / 'config/simulation_limits.yaml'
    path.write_text(yaml.safe_dump(config))
    jobs = MotionJobs(models_root=tmp_path)
    try:
        data = payload() | {'model':'full_body_robot', 'urdf_text':source}
        first = jobs.create(data)
        key = jobs.jobs[first['id']]['profile_key']
        with pytest.raises(ValueError, match='不一致'):
            jobs.create(data | {'urdf_text':source.replace('name="full_body_robot"', 'name="changed"')})
        config['joint_limits']['BN_LeftUpperArm_1']['upper'] = 3.
        path.write_text(yaml.safe_dump(config))
        second = jobs.create(data)
        assert jobs.jobs[second['id']]['profile_key'] != key
    finally:
        jobs.close()


def test_registered_full_body_t_pose_matches_actual_bvh_arm_directions():
    package = Path(__file__).resolve().parents[1]
    model = package.parent / 'robot_description/models/full_body_robot'
    jobs = MotionJobs()
    try:
        first = jobs.create(payload() | {
            'model': 'full_body_robot', 'up_axis': 'y_up',
            'urdf_text': (model / 'urdf/full_body_robot.urdf').read_text(),
            'bvh_text': (package / 'action_library/演示样例_20261027141432.bvh').read_text()})
        clip = jobs.jobs[first['id']]['clip']
        clip.frames = clip.frames[:1]
        jobs.start(first['id'], first['mapping'])
        done = finish(jobs, first)
        assert done['state'] == 'done', done.get('error', done)
        assert done['summary']['pose_fit'] == 'bone_directions'
        preview = json.loads(jobs.result(first['id'], 'preview.json'))
        source, robot = preview['source'], preview['robot']
        basis = np.asarray(preview['source_to_robot'])
        for side in ('Left', 'Right'):
            direction = basis @ (np.asarray(source['frames'][0][source['names'].index(side + 'LowerArm')]) -
                                 source['frames'][0][source['names'].index(side + 'UpperArm')])
            actual = (np.asarray(robot['frames'][0][robot['names'].index(f'BN_{side}LowerArm_2_link')]) -
                      robot['frames'][0][robot['names'].index(f'BN_{side}UpperArm_3_link')])
            cosine = np.dot(actual, direction) / np.linalg.norm(actual) / np.linalg.norm(direction)
            assert cosine > np.cos(np.deg2rad(15)), (side, actual, direction)
    finally:
        jobs.close()


def test_confirm_then_reuse_across_service_restart(tmp_path):
    with_manager = MotionJobs(profiles_dir=tmp_path)
    try:
        first = with_manager.create(payload())
        assert first['state'] == 'mapping'
        assert first['mapping_origin'] == 'suggested'
        assert first['mapping']['mode'] == 'generic'
        with_manager.start(first['id'], first['mapping'], remember=True)
        done = finish(with_manager, first)
        assert done['state'] == 'done', done
        assert done['summary']['preview_only']
        preview = json.loads(with_manager.result(first['id'], 'preview.json'))
        assert preview['robot']['joint_names'] == ['axis']
        assert preview['robot']['joint_positions'][-1][0] == pytest.approx(np.pi / 3, abs=.002)
        assert len(list(tmp_path.glob('*.json'))) == 1
    finally:
        with_manager.close()
    restored = MotionJobs(profiles_dir=tmp_path)
    try:
        second = restored.create(payload())
        assert second['mapping_origin'] == 'saved'
        assert finish(restored, second)['state'] == 'done'
    finally:
        restored.close()


def test_preview_exposes_source_root_rotation_in_robot_coordinates():
    data = payload()
    data['bvh_text'] = data['bvh_text'].replace('0 0 0 0 0 0', '0 0 0 0 90 0')
    jobs = MotionJobs()
    try:
        job = jobs.create(data)
        job['mapping']['source_to_robot'] = [[0,-1,0],[1,0,0],[0,0,1]]
        jobs.start(job['id'], job['mapping'])
        assert finish(jobs, job)['state'] == 'done'
        preview = json.loads(jobs.result(job['id'], 'preview.json'))
        from scipy.spatial.transform import Rotation
        rotations = Rotation.from_quat(preview['robot']['root_quaternions']).as_matrix()
        # A source X rotation is represented about Y after the +90-degree Z mapping.
        expected = Rotation.from_euler('y', 90, degrees=True).as_matrix()
        np.testing.assert_allclose(rotations, np.tile(expected, (3, 1, 1)), atol=1e-8)
        assert preview['robot']['root_rotation_source'] == 'Hips'
        assert preview['robot']['root_rotation_frame'] == 'robot'
    finally:
        jobs.close()


def test_completed_results_reused_after_restart_without_solving(tmp_path, monkeypatch):
    from bvh_broadcaster.generic_retarget import GenericRetargeter
    options = dict(profiles_dir=tmp_path / 'profiles', cache_dir=tmp_path / 'results')
    jobs = MotionJobs(**options)
    try:
        first = jobs.create(payload())
        jobs.start(first['id'], first['mapping'], remember=True)
        assert finish(jobs, first)['result_saved']
        expected = jobs.result(first['id'], 'motion.npz')
    finally:
        jobs.close()
    def forbidden(*args, **kwargs):
        pytest.fail('unchanged cached motion must not run the solver')
    monkeypatch.setattr(GenericRetargeter, 'convert', forbidden)
    jobs = MotionJobs(**options)
    try:
        restored = jobs.create(payload())
        assert restored['state'] == 'done' and restored['result_cached']
        assert jobs.thread is None
        assert jobs.result(restored['id'], 'motion.npz') == expected
        # Reviewing the mapping still pauses, then reuses the same completed result.
        review = jobs.create(payload() | {'review_mapping': True})
        assert review['state'] == 'mapping'
        assert jobs.start(review['id'], review['mapping'])['result_cached']
    finally:
        jobs.close()


@pytest.mark.parametrize('change', ['motion', 'model', 'units', 'axes', 'mapping'])
def test_result_cache_invalidates_changed_inputs(tmp_path, change):
    jobs = MotionJobs(cache_dir=tmp_path)
    try:
        first = jobs.create(payload())
        jobs.start(first['id'], first['mapping'])
        assert finish(jobs, first)['state'] == 'done'
        first_key = jobs.jobs[first['id']]['cache_key']
        data = payload()
        if change == 'motion':
            data['bvh_text'] = data['bvh_text'].replace('30 0 0', '20 0 0')
        elif change == 'model':
            data['urdf_text'] = data['urdf_text'].replace('velocity="100"', 'velocity="50"')
        elif change == 'units':
            data['scale'] = 1.
        elif change == 'axes':
            data['up_axis'] = 'y_up'
        second = jobs.create(data)
        if change == 'mapping':
            second['mapping']['reference_frame'] = 1
        jobs.start(second['id'], second['mapping'])
        done = finish(jobs, second)
        assert done['state'] == 'done', done.get('error')
        assert not done.get('result_cached')
        assert jobs.jobs[second['id']]['cache_key'] != first_key
    finally:
        jobs.close()


@pytest.mark.parametrize('damage', ['missing', 'truncated', 'manifest'])
def test_damaged_result_cache_recomputes(tmp_path, damage):
    jobs = MotionJobs(cache_dir=tmp_path)
    try:
        first = jobs.create(payload())
        jobs.start(first['id'], first['mapping'])
        assert finish(jobs, first)['state'] == 'done'
        directory = tmp_path / jobs.jobs[first['id']]['cache_key']
        if damage == 'missing':
            (directory / 'motion.npz').unlink()
        elif damage == 'truncated':
            (directory / 'preview.json').write_text('{')
        else:
            (directory / 'manifest.json').write_text('null')
        second = jobs.create(payload())
        jobs.start(second['id'], second['mapping'])
        done = finish(jobs, second)
        assert done['state'] == 'done' and not done.get('result_cached')
    finally:
        jobs.close()


@pytest.mark.parametrize('implementation_module', ['bvh_broadcaster.generic_retarget', 'web_host.motion_source'])
def test_result_identity_tracks_algorithm_and_simulation_assets(tmp_path, monkeypatch, implementation_module):
    from types import SimpleNamespace
    from web_host import motion_cache
    jobs = MotionJobs()
    try:
        first = jobs.create(payload())
        job = jobs.jobs[first['id']]
        model = tmp_path / 'model'
        (model / 'mujoco').mkdir(parents=True)
        (model / 'urdf').mkdir()
        xml = model / 'mujoco/model.xml'
        xml.write_text('<mujoco/>')
        urdf = model / 'urdf/model.standalone.urdf'
        urdf.write_text('<robot/>')
        job['simulation'] = {'directory':model, 'config_sha256':'original'}
        implementation = tmp_path / 'implementation.py'
        implementation.write_text('original')
        find_spec = motion_cache.importlib.util.find_spec
        monkeypatch.setattr(motion_cache.importlib.util, 'find_spec', lambda name:
            SimpleNamespace(origin=str(implementation)) if name == implementation_module
            else find_spec(name))
        def key():
            return motion_cache.result_key(job, first['mapping'])
        previous = key()
        for change in (lambda: xml.write_text('<mujoco changed="1"/>'),
                       lambda: urdf.write_text('<robot changed="1"/>'),
                       lambda: job['simulation'].update(config_sha256='changed'),
                       lambda: implementation.write_text('changed')):
            change()
            current = key()
            assert current != previous
            previous = current
    finally:
        jobs.close()


def test_pre_heading_fix_mapping_requires_new_confirmation(tmp_path, monkeypatch):
    # Reproduce the old profile namespace, without the anatomical-basis version.
    dumps = json.dumps
    def legacy_dumps(document, *args, **kwargs):
        if isinstance(document, dict) and 'source_basis_version' in document:
            document = {key: value for key, value in document.items() if key != 'source_basis_version'}
        return dumps(document, *args, **kwargs)
    jobs = MotionJobs(profiles_dir=tmp_path)
    try:
        with monkeypatch.context() as patch:
            patch.setattr(json, 'dumps', legacy_dumps)
            old = jobs.create(payload())
        old_job = jobs.jobs[old['id']]
        old['mapping']['source_to_robot'] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
        jobs._remember_mapping(old_job, old['mapping'])
        assert (tmp_path / (old_job['profile_key'] + '.json')).is_file()
        current = jobs.create(payload())
        assert jobs.jobs[current['id']]['profile_key'] != old_job['profile_key']
        assert current['state'] == 'mapping' and current['mapping_origin'] == 'suggested'
    finally:
        jobs.close()


def test_cache_write_failure_keeps_completed_downloads(tmp_path):
    unavailable = tmp_path / 'not-a-directory'
    unavailable.write_text('preserve this file')
    jobs = MotionJobs(cache_dir=unavailable)
    try:
        first = jobs.create(payload())
        jobs.start(first['id'], first['mapping'])
        done = finish(jobs, first)
        assert done['state'] == 'done' and done['cache_warning']
        assert jobs.result(first['id'], 'motion.npz').startswith(b'PK')
        assert unavailable.read_text() == 'preserve this file'
    finally:
        jobs.close()


@pytest.mark.parametrize('change', [
    lambda p: p.update(urdf_text=p['urdf_text'].replace('velocity="100"', 'velocity="50"')),
    lambda p: p.update(bvh_text=p['bvh_text'].replace('OFFSET 100 0 0', 'OFFSET 110 0 0')),
    lambda p: p.update(bvh_text=p['bvh_text'].replace('0 0 0 0 0 0 0 0 0', '0 0 0 0 0 0 10 0 0')),
    lambda p: p.update(up_axis='y_up'),
    lambda p: p.update(scale=1),
])
def test_model_skeleton_reference_axis_or_units_change_invalidates_mapping(tmp_path, change):
    jobs = MotionJobs(profiles_dir=tmp_path)
    try:
        first = jobs.create(payload())
        jobs.start(first['id'], first['mapping'], remember=True)
        assert finish(jobs, first)['state'] == 'done'
        modified = payload()
        change(modified)
        assert jobs.create(modified)['state'] == 'mapping'
    finally:
        jobs.close()


def test_corrupt_profile_falls_back_to_confirmation(tmp_path):
    jobs = MotionJobs(profiles_dir=tmp_path)
    try:
        first = jobs.create(payload())
        jobs.start(first['id'], first['mapping'], remember=True)
        assert finish(jobs, first)['state'] == 'done'
        next(tmp_path.glob('*.json')).write_text('{broken')
        assert jobs.create(payload())['state'] == 'mapping'
    finally:
        jobs.close()


def test_cad_preview_is_explicit_exported_and_original_urdf_unchanged(tmp_path):
    data = payload()
    data['urdf_text'] = data['urdf_text'].replace('lower="-2" upper="2" velocity="100"',
                                                'lower="0" upper="0" velocity="0"')
    jobs = MotionJobs(profiles_dir=tmp_path)
    try:
        first = jobs.create(data)
        assert first['state'] == 'mapping' and first['warnings']
        with pytest.raises(ValueError, match='限位'):
            jobs.start(first['id'], first['mapping'], remember=True)
        assert not list(tmp_path.glob('*.json'))
        first['mapping']['limit_policy'] = 'preview'
        jobs.start(first['id'], first['mapping'], remember=True)
        done = finish(jobs, first)
        assert done['state'] == 'done', done
        assert done['summary']['mechanical_limits_enforced'] is False
        preview = json.loads(jobs.result(first['id'], 'preview.json'))
        assert preview['robot']['ignore_limits'] is True
        assert preview['robot']['urdf'] == data['urdf_text']
        assert preview['robot']['joint_positions'][-1][0] > 1
    finally:
        jobs.close()


def test_profile_can_be_reconfirmed_without_reusing_bad_choice(tmp_path):
    jobs = MotionJobs(profiles_dir=tmp_path)
    try:
        first = jobs.create(payload())
        jobs.start(first['id'], first['mapping'], remember=True)
        assert finish(jobs, first)['state'] == 'done'
        again = jobs.create({**payload(), 'review_mapping': True})
        assert again['state'] == 'mapping' and again['mapping_origin'] == 'saved'
    finally:
        jobs.close()


@pytest.mark.parametrize('missing', ['element', 'velocity', 'upper'])
def test_missing_limits_reach_explicit_preview_without_changing_urdf(tmp_path, missing):
    data = payload()
    if missing == 'element':
        data['urdf_text'] = data['urdf_text'].replace('<limit lower="-2" upper="2" velocity="100" effort="1"/>', '')
    else:
        data['urdf_text'] = data['urdf_text'].replace('velocity="100"' if missing == 'velocity' else 'upper="2"', '')
    jobs = MotionJobs(profiles_dir=tmp_path)
    try:
        job = jobs.create(data)
        assert job['state'] == 'mapping' and job['warnings']
        with pytest.raises(ValueError, match='限位'):
            jobs.start(job['id'], job['mapping'])
        job['mapping']['limit_policy'] = 'preview'
        jobs.start(job['id'], job['mapping'])
        assert finish(jobs, job)['state'] == 'done'
        preview = json.loads(jobs.result(job['id'], 'preview.json'))
        assert preview['robot']['urdf'] == data['urdf_text']
        assert not jobs.status(job['id'])['summary']['mechanical_limits_enforced']
    finally:
        jobs.close()


@pytest.mark.parametrize('missing,sign', [('lower', 1), ('upper', -1)])
def test_preview_keeps_the_other_valid_position_boundary(tmp_path, missing, sign):
    data = payload()
    data['urdf_text'] = data['urdf_text'].replace('lower="-2" upper="2"', 'lower="-0.2" upper="0.2"')
    data['urdf_text'] = data['urdf_text'].replace('lower="-0.2"' if missing == 'lower' else 'upper="0.2"', '')
    if sign < 0:
        data['bvh_text'] = data['bvh_text'].replace('30 0 0', '-30 0 0').replace('60 0 0', '-60 0 0')
    jobs = MotionJobs(profiles_dir=tmp_path)
    try:
        job = jobs.create(data)
        job['mapping']['limit_policy'] = 'preview'
        jobs.start(job['id'], job['mapping'])
        assert finish(jobs, job)['state'] == 'done'
        preview = json.loads(jobs.result(job['id'], 'preview.json'))
        q = np.asarray(preview['robot']['joint_positions'])[:, 0]
        assert np.all(sign * q <= .2 + 1e-8)
        assert sign * q[-1] == pytest.approx(.2, abs=.001)
    finally:
        jobs.close()


def test_non_ascii_joint_names_survive_generic_import(tmp_path):
    data = payload()
    data['bvh_text'] = data['bvh_text'].replace('LeftArm', '左臂:关节').replace('Hips', '人体根')
    data['urdf_text'] = data['urdf_text'].replace('LeftArm', '左臂:关节')
    jobs = MotionJobs(profiles_dir=tmp_path)
    try:
        job = jobs.create(data)
        assert job['source_joints'] == ['人体根', '左臂:关节']
        assert job['mapping']['targets'][0]['source'] == '左臂:关节'
        jobs.start(job['id'], job['mapping'])
        assert finish(jobs, job)['state'] == 'done'
    finally:
        jobs.close()


@pytest.mark.parametrize('filename', ['full_body_robot.urdf', 'full_body_robot.mapping.urdf'])
def test_actual_full_body_cad_uses_candidates_and_converts_without_modifying_urdf(tmp_path, filename):
    root = Path(__file__).resolve().parents[2]
    path = root / 'robot_description/models/full_body_robot/urdf' / filename
    import yaml
    presets = yaml.safe_load((root / 'bvh_broadcaster/config/generic_profiles.yaml').read_text())
    names = sorted(set(presets['profiles'][0]['targets'].values()) - {'Spine'})
    hierarchy = 'HIERARCHY\nROOT Spine { OFFSET 0 0 0 CHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation\n'
    for name in names:
        hierarchy += f'JOINT {name} {{ OFFSET 1 0 0 CHANNELS 3 Zrotation Xrotation Yrotation }}\n'
    frames = [' '.join(map(str, [0] * 6 + [angle, 0, 0] * len(names))) for angle in (0, 2, 4)]
    bvh = hierarchy + '}\nMOTION\nFrames: 3\nFrame Time: 0.02\n' + '\n'.join(frames)
    data = {**payload(), 'bvh_text': bvh, 'urdf_text': path.read_text()}
    jobs = MotionJobs(profiles_dir=tmp_path)
    try:
        job = jobs.create(data)
        assert job['state'] == 'mapping' and job['joint_count'] == 37
        assert sum(bool(t['source']) for t in job['mapping']['targets']) == 16
        assert next(t['target'] for t in job['mapping']['targets'] if t['source'] == 'LeftHand') == 'BN_LeftHand_pitch_link'
        job['mapping']['limit_policy'] = 'preview'
        jobs.start(job['id'], job['mapping'])
        done = finish(jobs, job)
        assert done['state'] == 'done', done
        preview = json.loads(jobs.result(job['id'], 'preview.json'))
        assert preview['robot']['urdf'] == path.read_text() == data['urdf_text']
        assert len(preview['robot']['joint_names']) == 37
        assert all(name.startswith('BN_') for name in preview['robot']['joint_names'])
        assert np.isfinite(preview['robot']['joint_positions']).all()
        assert not done['summary']['mechanical_limits_enforced']
        assert not done['summary']['unmapped_joints']
    finally:
        jobs.close()
