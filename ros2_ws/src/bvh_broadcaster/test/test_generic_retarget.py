"""Different robot trees use the same confirmed mapping and solver."""

from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from bvh_broadcaster.bvh import BvhClip
from bvh_broadcaster.generic_retarget import GenericRetargeter, _swing_rotation, analyze


def robot(tmp_path, kind='revolute', mimic=''):
    limit = '<limit lower="-2" upper="2" velocity="100" effort="1"/>'
    path = tmp_path / 'robot.urdf'
    path.write_text(f'''<robot name="unseen_robot"><link name="base"/>
      <link name="left_upper_arm"/><link name="left_forearm"/>
      <joint name="motor_a" type="{kind}"><parent link="base"/>
      <child link="left_upper_arm"/><axis xyz="0 0 1"/>{limit}</joint>
      <joint name="motor_b" type="revolute"><parent link="left_upper_arm"/>
      <child link="left_forearm"/><origin xyz="1 0 0"/><axis xyz="0 0 1"/>
      {limit}{mimic}</joint></robot>''')
    return path


def clip(tmp_path, rotations=(0, 30, 60)):
    path = tmp_path / 'source.bvh'
    path.write_text('''HIERARCHY
ROOT Hips { OFFSET 0 0 0 CHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation
 JOINT LeftArm { OFFSET 0 0 0 CHANNELS 3 Zrotation Xrotation Yrotation
  JOINT LeftForeArm { OFFSET 100 0 0 CHANNELS 3 Zrotation Xrotation Yrotation
   End Site { OFFSET 100 0 0 }
  }
 }
}
MOTION
Frames: ''' + str(len(rotations)) + '\nFrame Time: 0.02\n' +
                    '\n'.join(' '.join(map(str, [0] * 6 + [r, 0, 0, 0, 0, 0]))
                              for r in rotations))
    return BvhClip.load(path)


def mapping():
    return {'mode': 'generic', 'source_root': 'Hips', 'reference_frame': 0,
            'source_to_robot': np.eye(3).tolist(), 'limit_policy': 'urdf',
            'targets': [{'source': 'LeftArm', 'target': 'left_upper_arm'},
                        {'source': 'LeftForeArm', 'target': 'left_forearm'}]}


def test_semantic_suggestions_are_explicit_and_convert_unseen_model(tmp_path):
    path, source = robot(tmp_path), clip(tmp_path)
    result = analyze(path, source, up_axis='z_up')
    targets = {item['target']: item for item in result['mapping']['targets']}
    assert targets['left_upper_arm']['source'] == 'LeftArm'
    assert targets['left_forearm']['source'] == 'LeftForeArm'
    assert result['confirmation_required']
    solver = GenericRetargeter(path, mapping(), source, up_axis='z_up')
    positions, _, errors = solver.convert()
    np.testing.assert_allclose(positions[:, 0], np.deg2rad([0, 30, 60]), atol=.005)
    assert np.max(errors) < .5
    assert solver.validate_result(positions) is None


def test_unknown_names_require_mapping_and_do_not_claim_coverage(tmp_path):
    path, source = robot(tmp_path), clip(tmp_path)
    path.write_text(path.read_text().replace('left_upper_arm', 'piece_123').replace('left_forearm', 'piece_456'))
    result = analyze(path, source)
    assert result['confirmation_required']
    assert all(not t['source'] for t in result['mapping']['targets'])
    with pytest.raises(ValueError, match='至少'):
        GenericRetargeter(path, result['mapping'], source)


def test_bone_directions_match_first_pose_instead_of_robot_zero(tmp_path):
    path, source = robot(tmp_path), clip(tmp_path, (0, -1, -2))
    path.write_text(path.read_text().replace('<axis xyz="0 0 1"/>',
                    '<origin rpy="0 0 -1.5707963267948966"/><axis xyz="0 0 1"/>', 1)
                    .replace('velocity="100"', 'velocity="1"'))
    solver = GenericRetargeter(path, mapping(), source, up_axis='z_up', match_directions=True)
    positions, _, _ = solver.convert()
    # The source's horizontal arm must not become the robot's downward zero pose.
    np.testing.assert_allclose(positions[:, 0], np.deg2rad([90, 89, 88]), atol=.01)
    assert np.max(np.abs(np.diff(positions, axis=0))) <= .020001
    assert solver.report['direction_max_deg'] < 1
    assert solver.report['orientation_max_deg'] < 1


def test_direction_fit_without_segment_keeps_relative_orientation(tmp_path):
    path, source = robot(tmp_path), clip(tmp_path)
    selected = mapping()
    selected['targets'] = selected['targets'][:1]
    solver = GenericRetargeter(path, selected, source, up_axis='z_up', match_directions=True)
    positions, _, _ = solver.convert()
    np.testing.assert_allclose(positions[:, 0], np.deg2rad([0, 30, 60]), atol=.005)


@pytest.mark.parametrize('end', [(1., 0., 0.), (-1., 0., 0.), (0., 1., 0.)])
def test_swing_calibration_is_a_proper_rotation_including_opposite_vectors(end):
    rotation = _swing_rotation(np.array([1., 0., 0.]), np.array(end))
    np.testing.assert_allclose(rotation @ [1., 0., 0.], end, atol=1e-8)
    np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-8)
    assert np.linalg.det(rotation) == pytest.approx(1.)


def test_zero_limits_require_explicit_geometry_preview_and_keep_source(tmp_path):
    path, source = robot(tmp_path), clip(tmp_path)
    path.write_text(path.read_text().replace('lower="-2" upper="2" velocity="100"',
                                           'lower="0" upper="0" velocity="0"'))
    original = path.read_bytes()
    assert analyze(path, source)['warnings']
    with pytest.raises(ValueError, match='限位'):
        GenericRetargeter(path, mapping(), source)
    selected = {**mapping(), 'limit_policy': 'preview'}
    solver = GenericRetargeter(path, selected, source, up_axis='z_up')
    positions, _, _ = solver.convert()
    assert positions[-1, 0] > .9
    assert not solver.mechanical_limits_enforced
    assert path.read_bytes() == original


def test_continuous_joint_crosses_pi_without_a_jump(tmp_path):
    path, source = robot(tmp_path, 'continuous'), clip(tmp_path, (0, 90, 170, 190, 250))
    solver = GenericRetargeter(path, mapping(), source, up_axis='z_up')
    positions, _, _ = solver.convert()
    np.testing.assert_allclose(positions[:, 0], np.deg2rad([0, 90, 170, 190, 250]), atol=.01)
    assert positions.shape == (5, 2)
    assert solver.configuration(positions[-1]).shape == (3,)


def test_mimic_negative_ratio_is_preserved_and_bounds_intersect(tmp_path):
    path, source = robot(tmp_path, mimic='<mimic joint="motor_a" multiplier="-2" offset="0.1"/>'), clip(tmp_path)
    selected = mapping()
    selected['targets'] = selected['targets'][:1]
    solver = GenericRetargeter(path, selected, source, up_axis='z_up')
    positions, _, _ = solver.convert()
    np.testing.assert_allclose(positions[:, 1], -2 * positions[:, 0] + .1, atol=1e-10)
    assert np.all(positions <= 2 + 1e-8) and np.all(positions >= -2 - 1e-8)


def test_prismatic_target_uses_scaled_translation(tmp_path):
    path, source = robot(tmp_path, 'prismatic'), clip(tmp_path, (0, 0, 0))
    source.joints[1].channels = ('Zposition', 'Xrotation', 'Yrotation')
    source.frames[:, 6] = [0, 10, 20]
    selected = mapping()
    selected['targets'] = selected['targets'][:1]
    solver = GenericRetargeter(path, selected, source, up_axis='z_up', scale=.01)
    positions, _, _ = solver.convert()
    np.testing.assert_allclose(positions[:, 0], [0, .1, .2], atol=.002)
    assert solver.report['translation_targets'] == ['left_upper_arm']
    assert solver.report['translation_mean_m'][0] < .002


@pytest.mark.parametrize('change,match', [
    (lambda m: m.update(reference_frame=-1), '参考帧'),
    (lambda m: m.update(source_to_robot=[[0]*3]*3), '旋转'),
    (lambda m: m['targets'][0].update(source='absent'), '人体'),
    (lambda m: m['targets'][0].update(target='absent'), '机器人'),
    (lambda m: m['targets'].append(m['targets'][0].copy()), '重复'),
])
def test_invalid_mapping_is_rejected_before_conversion(tmp_path, change, match):
    selected = mapping()
    change(selected)
    with pytest.raises(ValueError, match=match):
        GenericRetargeter(robot(tmp_path), selected, clip(tmp_path))


def test_same_anatomical_pose_cannot_constrain_intermediate_serial_axes(tmp_path):
    selected = mapping()
    selected['targets'][1]['source'] = 'LeftArm'
    with pytest.raises(ValueError, match='重复.*LeftArm|LeftArm.*重复'):
        GenericRetargeter(robot(tmp_path), selected, clip(tmp_path))


def test_invalid_mimic_and_unsupported_joint_are_named(tmp_path):
    path, source = robot(tmp_path, mimic='<mimic joint="missing"/>'), clip(tmp_path)
    with pytest.raises(ValueError, match='mimic'):
        analyze(path, source)
    path = robot(tmp_path, kind='floating')
    with pytest.raises(ValueError, match='floating.*motor_a|motor_a.*floating'):
        analyze(path, source)
