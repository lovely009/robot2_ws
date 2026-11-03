"""The inspection model must preserve motion and distinguish uncalibrated datums."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

MODEL = Path(__file__).resolve().parents[1] / 'models/half_body_robot'
SPEC = importlib.util.spec_from_file_location('motor_frames', MODEL / 'tools/build_motor_frames.py')
frames = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(frames)


def test_frame_basis_is_right_handed_even_for_parallel_reference():
    for normal in ([1, 0, 0], [0, 0, -1], [0.03684, 0, 0.99932]):
        rotation = frames.basis(normal)
        np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-12)
        assert np.linalg.det(rotation) == pytest.approx(1)
        np.testing.assert_allclose(rotation[:, 2], np.array(normal) / np.linalg.norm(normal))


def test_circle_fit_and_degenerate_evidence():
    centers = np.array([[.02, 0, 0], [0, .02, 0], [-.02, 0, 0], [0, -.02, 0]])
    center, radius, residual = frames.fit_circle(centers, [0, 0, 1])
    np.testing.assert_allclose(center, [0, 0, 0], atol=1e-12)
    assert radius == pytest.approx(.02)
    assert residual < 1e-12
    with pytest.raises(ValueError):
        frames.fit_circle(centers[:2], [0, 0, 1])
    with pytest.raises(ValueError):
        frames.fit_circle(np.array([[0, 0, 0], [.01, 0, 0], [.02, 0, 0]]), [0, 0, 1])


def test_inspection_urdf_preserves_every_existing_link_and_joint():
    original = ET.parse(MODEL / 'urdf/half_body_robot.urdf').getroot()
    inspected = ET.parse(MODEL / 'cad/frames/half_body_robot.frames.urdf').getroot()
    for element in original:
        found = inspected.find(f"{element.tag}[@name='{element.get('name')}']")
        assert found is not None
        assert ET.tostring(element).strip() == ET.tostring(found).strip()
    additions = [j for j in inspected.findall('joint') if original.find(f"joint[@name='{j.get('name')}']") is None]
    assert len(additions) == 36
    assert all(j.get('type') == 'fixed' for j in additions)
    assert len({link.get('name') for link in inspected.findall('link')}) == len(inspected.findall('link'))
    for joint in original.findall('joint'):
        axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
        fixed = inspected.find(f"joint[@name='{joint.get('name')}_axis_fixed']")
        rotation = frames.transform(fixed)[:3, :3]
        np.testing.assert_allclose(rotation[:, 2], axis / np.linalg.norm(axis), atol=2e-11)


def test_geometric_evidence_and_uncommissioned_zero():
    import yaml
    report = yaml.safe_load((MODEL / 'cad/frames/measurements.yaml').read_text())
    assert len(report['joint_frames']) == 18
    assert len(report['motor_datums']) == 18
    for row in report['motor_datums']:
        assert row['encoder_zero'] == 'unverified'
        assert row['command_direction'] == 'unverified'
        if row['status'] == 'bolt_circle_verified':
            assert row['observed_holes'] >= 4
            assert row['pcd_error_mm'] < .05
            assert row['fit_residual_mm'] < .05
        else:
            assert row['motor_id'] is None
            assert 'candidate' in row['frame']
    # The shoulder linkage cannot be mistaken for a coaxial motor.
    shoulder = next(r for r in report['motor_datums'] if r['motor_id'] == 2)
    assert shoulder['flange_center_to_joint_axis_mm'] > 50
    assert shoulder['axis_angle_deg'] > 80


def test_measurements_are_current_and_locators_set_motor_clocking():
    import hashlib
    import yaml
    report = yaml.safe_load((MODEL / 'cad/frames/measurements.yaml').read_text())
    for relative, expected in report['source_sha256'].items():
        assert hashlib.sha256((MODEL / relative).read_bytes()).hexdigest() == expected, relative
    for row in report['motor_datums']:
        if row['clocking_status'] != 'locator_verified':
            assert row['locating_hole_parent'] is None
            continue
        locator = np.array(row['locating_hole_parent']) - row['parent_xyz']
        rotation = frames.Rotation.from_euler('xyz', row['parent_rpy']).as_matrix()
        locator -= (locator @ rotation[:, 2]) * rotation[:, 2]
        np.testing.assert_allclose(rotation[:, 0], locator / np.linalg.norm(locator), atol=1e-10)


def test_motor_tf_relations_cover_configured_motors_and_group_wrist_axes():
    import yaml
    relation = yaml.safe_load((MODEL / 'cad/frames/motor_tf.yaml').read_text())
    driver = yaml.safe_load((MODEL / 'config/motor_driver.yaml').read_text())['motor_hardware']['motors']
    transmissions = yaml.safe_load((MODEL / 'config/transmissions.yaml').read_text())['transmissions']
    inspected = ET.parse(MODEL / 'cad/frames' / relation['urdf']).getroot()
    links = {link.get('name') for link in inspected.findall('link')}
    assert set(relation['motors']) == {motor['id'] for motor in driver.values()}
    for name, motor in driver.items():
        row = relation['motors'][motor['id']]
        assert row['actuator'] == name
        assert row['motor_type'] == motor['type']
        assert set(row['joint_axis_tfs']) <= links
        if row['transmission_group'] is not None:
            group = transmissions[row['transmission_group']]
            assert name in group['actuators']
            assert row['relation'] == 'differential_group'
            assert row['joint_axis_tfs'] == [f'{joint}_axis' for joint in group['joints']]
            assert row['mount_reference_tf'] is None
        else:
            assert row['joint_axis_tfs'] == [f"{motor['joint']}_axis"]
            assert row['mount_reference_tf'] in links
            assert 'candidate' not in row['mount_reference_tf']
    assert relation['motors'][2]['relation'] == 'linkage'
    assert relation['motors'][11]['relation'] == 'linkage'
