"""Generic jobs persist only confirmed mappings for the exact input identity."""

import json
from pathlib import Path

import numpy as np
import pytest

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


def test_actual_full_body_cad_uses_candidates_and_converts_without_modifying_urdf(tmp_path):
    root = Path(__file__).resolve().parents[2]
    path = root / 'robot_description/models/full_body_robot/urdf/full_body_robot.urdf'
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
        job['mapping']['limit_policy'] = 'preview'
        jobs.start(job['id'], job['mapping'])
        done = finish(jobs, job)
        assert done['state'] == 'done', done
        preview = json.loads(jobs.result(job['id'], 'preview.json'))
        assert preview['robot']['urdf'] == path.read_text() == data['urdf_text']
        assert len(preview['robot']['joint_names']) == 37
        assert np.isfinite(preview['robot']['joint_positions']).all()
        assert not done['summary']['mechanical_limits_enforced']
        assert done['summary']['unmapped_joints']
    finally:
        jobs.close()
