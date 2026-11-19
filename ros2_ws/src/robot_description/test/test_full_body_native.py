"""Main model must use native mesh names and the verified assembly, not old ICP."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

MODEL = Path(__file__).parents[1] / 'models/full_body_robot'
SOURCE = MODEL / 'cad/robot_body/urdf/cnm.urdf'




def test_calibration_uses_current_source_files():
    import hashlib
    alignment = json.loads((MODEL / 'config/step_alignment.json').read_text())
    assert alignment['urdf_sha256'] == hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    assert alignment['step_sha256'] == hashlib.sha256((MODEL / 'cad/全身不带头.STEP').read_bytes()).hexdigest()
    assert json.loads((MODEL / 'config/display_meshes.json').read_text())['source'] == 'cad/robot_body'
    for entry in alignment['links'].values():
        path = SOURCE.parents[1] / 'meshes' / entry['source']
        assert entry['source_sha256'] == hashlib.sha256(path.read_bytes()).hexdigest()
    arm = json.loads((MODEL / 'config/native_assembly.json').read_text())['arm_inertia']
    assert arm['sha256'] == hashlib.sha256((MODEL / arm['source_urdf']).read_bytes()).hexdigest()


def test_mechanical_generator_preserves_calibrated_skeleton(monkeypatch):
    import sys
    monkeypatch.syspath_prepend(str(MODEL / 'tools'))
    import build_native_model as builder
    import joint_convention
    alignment = json.loads((MODEL / 'config/step_alignment.json').read_text())
    hand = json.loads((MODEL / 'config/left_hand_mount_alignment.json').read_text())
    source, _, _, _ = builder.source_tree(alignment, hand)
    rotation, coordinates = joint_convention.load(MODEL)
    joint_convention.convert_urdf(source, rotation, coordinates)
    runtime = ET.parse(MODEL / 'urdf/full_body_robot.urdf').getroot()
    for joint in source.findall('joint'):
        actual = runtime.find(f"joint[@name='{joint.get('name')}']")
        for field in ('origin', 'axis'):
            for key, value in joint.find(field).attrib.items():
                np.testing.assert_allclose(np.fromstring(value, sep=' '),
                                           np.fromstring(actual.find(field).get(key), sep=' '), atol=1e-12)
    for link in source.findall('link'):
        actual = runtime.find(f"link[@name='{link.get('name')}']/inertial")
        for field in ('mass', 'inertia', 'origin'):
            for key, value in link.find('inertial/' + field).attrib.items():
                np.testing.assert_allclose(np.fromstring(value, sep=' '),
                                           np.fromstring(actual.find(field).get(key), sep=' '), atol=1e-12)


def test_mechanical_generator_rejects_stale_source(monkeypatch):
    monkeypatch.syspath_prepend(str(MODEL / 'tools'))
    import build_native_model as builder
    alignment = json.loads((MODEL / 'config/step_alignment.json').read_text())
    alignment['urdf_sha256'] = 'stale'
    with pytest.raises(ValueError, match='STEP calibration'):
        builder.source_tree(alignment)


def test_mechanical_generator_preserves_calibrated_skeleton(monkeypatch):
    monkeypatch.syspath_prepend(str(MODEL / 'tools'))
    import build_native_model
    import joint_convention
    alignment = json.loads((MODEL / 'config/step_alignment.json').read_text())
    hand = json.loads((MODEL / 'config/left_hand_mount_alignment.json').read_text())
    generated, _, _, _ = build_native_model.source_tree(alignment, hand)
    rotation, coordinates = joint_convention.load(MODEL)
    joint_convention.convert_urdf(generated, rotation, coordinates)
    current = ET.parse(MODEL / 'urdf/full_body_robot.urdf').getroot()
    assert len(generated.findall('joint')) == 37
    for joint in generated.findall('joint'):
        reference = current.find(f"joint[@name='{joint.get('name')}']")
        assert reference is not None
        for tag, key in (('origin', 'xyz'), ('origin', 'rpy'), ('axis', 'xyz')):
            np.testing.assert_allclose(
                np.fromstring(joint.find(tag).get(key), sep=' '),
                np.fromstring(reference.find(tag).get(key), sep=' '), atol=1e-12)
