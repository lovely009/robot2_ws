"""Keep the confirmed raw two-motor mapping identical in offline preview."""

import copy
import json
from pathlib import Path

import numpy as np
import pytest

pin = pytest.importorskip('pinocchio')
pytest.importorskip('bvh_broadcaster')
from bvh_broadcaster.bvh import BvhClip
from bvh_broadcaster.retarget import digest, load_motion
from web_host.shoulder_retarget import ShoulderRetargeter


ROOT = Path(__file__).resolve().parents[2]
URDF = ROOT / 'robot_description/models/half_body_robot/urdf/half_body_robot.urdf'
LIBRARY = ROOT / 'web_host/action_library'
MAPPING = {'mode': 'shoulders12', 'source': 'shoulder_r',
           'joints': ['joint_link1', 'joint_link2'],
           'source_to_robot': [[0, -1, 0], [1, 0, 0], [0, 0, 1]]}


@pytest.fixture
def clip():
    return BvhClip.load(LIBRARY / '1-4.bvh')


def test_all_raw_frames_match_confirmed_two_motor_action(clip, tmp_path):
    solver = ShoulderRetargeter(URDF, MAPPING, clip)
    progress = []
    positions, directions, orientations = solver.convert(lambda i, n: progress.append((i, n)))
    action = json.loads((LIBRARY / '1-4_shoulders12_raw.json').read_text())
    expected = np.array([p['position'] for p in action['points']]).reshape(-1, 2)
    assert positions.shape == (300, 18)
    np.testing.assert_allclose(positions[:, solver.indices], expected, atol=1e-10, rtol=0)
    np.testing.assert_array_equal(positions[:, 2:], 0)
    assert directions.shape == (300, 0) and orientations.shape == (300, 1)
    assert orientations.max() == pytest.approx(action['source']['orientation_projection_max_error_deg'])
    assert (len(positions) - 1) * clip.frame_time == action['source']['original_duration_s']
    assert progress[-1] == (299, 300)
    export = tmp_path / 'motion.npz'
    np.savez_compressed(export, positions=positions, joint_names=solver.names,
                        frame_time=clip.frame_time, urdf_sha256=digest(URDF))
    names, loaded, dt = load_motion(export, URDF)
    assert names == solver.names and dt == clip.frame_time
    np.testing.assert_array_equal(loaded, positions)


def test_unknown_model_or_source_is_not_automatically_mapped(clip):
    model = pin.buildModelFromUrdf(str(URDF))
    assert ShoulderRetargeter.supports(model, clip)
    model.lowerPositionLimit[0] -= .1
    assert not ShoulderRetargeter.supports(model, clip)
    model = pin.buildModelFromUrdf(str(URDF))
    clip.joints[3].parent = 1
    assert not ShoulderRetargeter.supports(model, clip)
    clip.joints[3].name = 'unknown'
    with pytest.raises(ValueError, match='已确认'):
        ShoulderRetargeter(URDF, MAPPING, clip)


@pytest.mark.parametrize('field,value', [
    ('mode', 'other'), ('source', 'arm_r'), ('joints', ['joint_link3', 'joint_link4']),
    ('source_to_robot', [[1, 0, 0], [0, 1, 0], [0, 0, -1]]),
    ('source_to_robot', [[float('nan'), 0, 0], [0, 1, 0], [0, 0, 1]]),
])
def test_invalid_mapping_is_rejected(clip, field, value):
    mapping = copy.deepcopy(MAPPING)
    mapping[field] = value
    with pytest.raises(ValueError):
        ShoulderRetargeter(URDF, mapping, clip)


def test_bvh_up_axis_changes_rotation_mapping(clip):
    # Axis conversion is conjugation of the same recorded shoulder rotation.
    y = ShoulderRetargeter(URDF, MAPPING, clip, up_axis='y_up')
    z = ShoulderRetargeter(URDF, MAPPING, clip, up_axis='z_up')
    basis = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])
    np.testing.assert_allclose(y._rotation(1), basis @ z._rotation(1) @ basis.T, atol=1e-12)
    assert not np.allclose(y._rotation(1), z._rotation(1))
    with pytest.raises(ValueError, match='BVH 坐标'):
        ShoulderRetargeter(URDF, MAPPING, clip, up_axis='unknown')
