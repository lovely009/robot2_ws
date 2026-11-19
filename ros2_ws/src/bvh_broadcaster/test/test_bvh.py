import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from bvh_broadcaster.bvh import BvhClip, playback_frame


SAMPLE = '''HIERARCHY
ROOT Hips {
 OFFSET 0 100 0
 CHANNELS 6 Xposition Yposition Zposition Yrotation Xrotation Zrotation
 JOINT Hand {
  OFFSET 0 10 0
  CHANNELS 3 Yrotation Xrotation Zrotation
  End Site { OFFSET 0 5 0 }
 }
}
MOTION
Frames: 1
Frame Time: 0.016667
10 20 30 90 90 0 0 0 0
'''


def load(tmp_path, text=SAMPLE):
    path = tmp_path / '动作.bvh'
    path.write_text(text, encoding='utf-8-sig')
    return BvhClip.load(path)


def test_intrinsic_channel_order_units_and_ros_axes(tmp_path):
    clip = load(tmp_path)
    assert [j.name for j in clip.joints] == ['Hips', 'Hand', 'Hand__end']
    poses = clip.local_transforms(0, apply_root_offset=False)
    root_position, root_quaternion = poses[0]
    np.testing.assert_allclose(root_position, [0.3, 0.1, 0.2])
    # R_y(90) R_x(90) maps the child's +Y offset to +X in BVH, i.e. ROS +Y.
    hand_world = root_position + Rotation.from_quat(root_quaternion).apply(poses[1][0])
    np.testing.assert_allclose(hand_world, [0.3, 0.2, 0.2], atol=1e-12)
    np.testing.assert_allclose(np.linalg.norm([q for _, q in poses], axis=1), 1)
    np.testing.assert_allclose(poses[2][1], [0, 0, 0, 1])


def test_root_offset_is_explicit_and_z_up_can_be_preserved(tmp_path):
    clip = load(tmp_path)
    included = clip.local_transforms(0)[0][0]
    ignored = clip.local_transforms(0, apply_root_offset=False)[0][0]
    np.testing.assert_allclose(included - ignored, [0, 0, 1])
    np.testing.assert_allclose(clip.local_transforms(0, up_axis='z_up')[0][0], [0.1, 1.2, 0.3])


@pytest.mark.parametrize('text', [
    SAMPLE.replace('Frames: 1', 'Frames: 2'),
    SAMPLE.replace('0.016667', '0'),
    SAMPLE.replace('10 20 30 90 90 0 0 0 0', '10 20 30 90 90 0 0 0'),
    SAMPLE.replace('10 20 30 90', 'nan 20 30 90'),
    SAMPLE.replace('Yrotation Xrotation Zrotation', 'Yrotation Xrotation Xrotation'),
    SAMPLE.replace('JOINT Hand', 'JOINT Hips'),
    SAMPLE.replace('End Site { OFFSET 0 5 0 }', 'End Site { OFFSET 0 5 0'),
    SAMPLE.replace('Yposition Zposition Yrotation', 'Yrotation Zposition Yposition'),
])
def test_invalid_input_is_rejected(tmp_path, text):
    with pytest.raises(ValueError):
        load(tmp_path, text)


@pytest.mark.parametrize('scale,axis', [(0, 'y_up'), (float('nan'), 'y_up'), (1, 'x')])
def test_invalid_conversion_is_rejected(tmp_path, scale, axis):
    with pytest.raises(ValueError):
        load(tmp_path).local_transforms(0, scale=scale, up_axis=axis)


def test_playback_timing_loop_and_final_hold():
    assert playback_frame(0, 0.1, 10, 1, False) == 0
    assert playback_frame(0.21, 0.1, 10, 2, False) == 4
    assert playback_frame(1.21, 0.1, 10, 1, True) == 2
    assert playback_frame(1.21, 0.1, 10, 1, False) == 9
    assert playback_frame(-1, 0.1, 10, 1, False) == 0
