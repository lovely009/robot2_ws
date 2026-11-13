import numpy as np
import pytest

from bvh_broadcaster.bvh import BvhClip
from web_host.motion_source import anatomical_basis, prepare_source


def clip(tmp_path, *, axis='Z', position_channels=True, repeated=True):
    vector = lambda n: f'0 {n} 0' if axis == 'Y' else f'0 0 {n}'
    channels = '6 Xposition Yposition Zposition Zrotation Xrotation Yrotation' if position_channels else '3 Zrotation Xrotation Yrotation'
    children = [('Head', 50), ('LeftFoot', -100), ('RightFoot', -100)]
    hierarchy = ''.join(f'JOINT {name} {{ OFFSET {vector(length)} CHANNELS {channels} }}\n' for name, length in children)
    row = '0 0 100 0 0 0 ' + ' '.join(
        ((vector(length) if repeated else '0 0 0') + ' ' if position_channels else '') + '0 0 0'
        for _, length in children)
    path = tmp_path / 'clip.bvh'
    path.write_text('HIERARCHY\nROOT Hips { OFFSET 0 0 100 CHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation\n'
                    + hierarchy + '}\nMOTION\nFrames: 2\nFrame Time: 0.02\n' + row + '\n' + row + '\n')
    return BvhClip.load(path)


def test_z_up_export_does_not_double_bone_lengths(tmp_path):
    source = clip(tmp_path)
    assert prepare_source(source, 'auto')[:2] == ('z_up', 'local_translation')
    poses = source.local_transforms(0, up_axis='z_up', apply_root_offset=False)
    np.testing.assert_allclose(poses[1][0], [0, 0, .5])
    np.testing.assert_allclose(poses[2][0], [0, 0, -1])
    np.testing.assert_allclose(poses[0][0], [0, 0, 1])


@pytest.mark.parametrize('position_channels', [True, False])
def test_standard_y_up_offsets_are_preserved(tmp_path, position_channels):
    source = clip(tmp_path, axis='Y', position_channels=position_channels, repeated=False)
    before = [joint.offset.copy() for joint in source.joints]
    assert prepare_source(source, 'auto')[:2] == ('y_up', 'offset_delta')
    np.testing.assert_allclose([j.offset for j in source.joints], before)


def test_manual_axis_is_respected(tmp_path):
    assert prepare_source(clip(tmp_path), 'y_up')[0] == 'y_up'


def test_varying_position_channels_are_not_reinterpreted(tmp_path):
    source = clip(tmp_path)
    source.frames[1, source.joints[1].start] += 2
    assert prepare_source(source, 'auto')[:2] == ('z_up', 'offset_delta')
    np.testing.assert_allclose(source.joints[1].offset, [0, 0, 50])


def test_export_rounding_noise_does_not_double_short_bones(tmp_path):
    source = clip(tmp_path)
    source.joints[1].offset = np.array([0., 0., 1.])
    source.frames[:, source.joints[1].start:source.joints[1].start + 3] = [0., 0., 1.00005]
    assert prepare_source(source, 'auto')[1] == 'local_translation'
    np.testing.assert_allclose(source.joints[1].offset, [0, 0, 0])


@pytest.mark.parametrize('pair', [('LeftUpperLeg', 'RightUpperLeg'), ('LeftShoulder', 'RightShoulder')])
@pytest.mark.parametrize('up_axis,expected', [
    ('y_up', np.eye(3)),
    ('z_up', np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])),
])
def test_same_humanoid_left_and_forward_axes_map_to_same_robot_direction(up_axis, expected, pair):
    names = list(pair)
    basis = anatomical_basis(names, [np.array([10., 0., 0.]), np.array([-10., 0., 0.])], up_axis)
    np.testing.assert_allclose(basis, expected)
    source_left = np.array([0, 1, 0]) if up_axis == 'y_up' else np.array([1, 0, 0])
    source_forward = np.cross(source_left, [0, 0, 1])
    np.testing.assert_allclose(basis @ source_left, [0, 1, 0])
    np.testing.assert_allclose(basis @ source_forward, [1, 0, 0])
    np.testing.assert_allclose(np.linalg.det(basis), 1)


def test_unknown_or_coincident_left_right_pair_keeps_existing_basis():
    for names in (['unknown', 'other'], ['LeftUpperLeg', 'RightUpperLeg']):
        np.testing.assert_allclose(anatomical_basis(names, [np.zeros(3), np.zeros(3)], 'z_up'), np.eye(3))
