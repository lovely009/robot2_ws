"""Keep the tested 4.4 source intact and both runtime formats consistent."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

MODEL = Path(__file__).resolve().parents[1] / 'models/right_arm_4dof'
URDF = MODEL / 'urdf/right_arm_4dof.urdf'
MJCF = MODEL / 'mujoco/right_arm_4dof.xml'


def numbers(text):
    return np.fromstring(text, sep=' ')


def test_original_files_and_runtime_meshes_are_preserved():
    hashes = json.loads((MODEL / 'source_sha256.json').read_text())
    assert len(hashes) == 24
    assert set(hashes) == {str(p.relative_to(MODEL / 'cad'))
                           for p in (MODEL / 'cad').rglob('*') if p.is_file()}
    for relative, digest in hashes.items():
        assert hashlib.sha256((MODEL / 'cad' / relative).read_bytes()).hexdigest() == digest
    for mesh in (MODEL / 'meshes').glob('*.STL'):
        assert mesh.read_bytes() == (MODEL / 'cad/meshes' / mesh.name).read_bytes()
    assert len(list((MODEL / 'meshes').glob('*.STL'))) == 9


def test_mjcf_changes_only_model_name_and_standalone_matches_urdf():
    original = (MODEL / 'cad/urdf/4.4.mjcf.xml').read_bytes()
    assert MJCF.read_bytes() == original.replace(
        b'model="shoubi_4_4_full"', b'model="right_arm_4dof"', 1)
    assert (MODEL / 'urdf/right_arm_4dof.standalone.urdf').read_bytes() == URDF.read_bytes()


def test_urdf_physics_and_geometry_match_tested_mjcf():
    urdf, mjcf = ET.parse(URDF).getroot(), ET.parse(MJCF).getroot()
    original = ET.parse(MODEL / 'cad/urdf/4.4.urdf').getroot()
    bodies = {b.get('name'): b for b in mjcf.findall('.//body')}
    assert urdf.get('name') == 'right_arm_4dof'
    assert {link.get('name') for link in urdf.findall('link')} == set(bodies)
    assert len(bodies) == 9
    for link in urdf.findall('link'):
        name = link.get('name')
        inertia = link.find('inertial')
        reference = bodies[name].find('inertial')
        assert float(inertia.find('mass').get('value')) == float(reference.get('mass'))
        np.testing.assert_allclose(numbers(inertia.find('origin').get('xyz')),
                                   numbers(reference.get('pos')), atol=1e-14)
        np.testing.assert_allclose(numbers(inertia.find('origin').get('rpy')), 0, atol=1e-14)
        np.testing.assert_allclose([float(inertia.find('inertia').get(k))
                                   for k in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz')],
                                   numbers(reference.get('fullinertia')), atol=1e-14)
        source_link = original.find(f"link[@name='{name}']")
        for tag in ('visual', 'collision'):
            runtime = link.find(tag)
            source = source_link.find(tag)
            source.find('geometry/mesh').set('filename',
                f'package://robot_description/models/right_arm_4dof/meshes/{name}.STL')
            assert ET.tostring(runtime).strip() == ET.tostring(source).strip()
    parents = {b.get('name'): p.get('name') for p in bodies.values() for b in p.findall('body')}
    moving = []
    for joint in urdf.findall('joint'):
        body = bodies[joint.find('child').get('link')]
        assert joint.find('parent').get('link') == parents[body.get('name')]
        origin = joint.find('origin')
        np.testing.assert_allclose(numbers(origin.get('xyz')), numbers(body.get('pos')), atol=1e-14)
        quat = numbers(body.get('quat', '1 0 0 0'))
        np.testing.assert_allclose(Rotation.from_euler('xyz', numbers(origin.get('rpy'))).as_matrix(),
                                   Rotation.from_quat(quat[[1, 2, 3, 0]]).as_matrix(), atol=1e-14)
        reference = body.find('joint')
        if reference is None:
            assert joint.get('type') == 'fixed'
            continue
        moving.append(joint.get('name'))
        assert joint.get('type') == 'revolute'
        assert reference.get('name') == joint.get('name')
        np.testing.assert_allclose(numbers(joint.find('axis').get('xyz')), numbers(reference.get('axis')))
        limit = joint.find('limit')
        np.testing.assert_allclose([float(limit.get(k)) for k in ('lower', 'upper')],
                                   numbers(reference.get('range')), atol=1e-14)
        assert float(limit.get('velocity')) == 0  # No measured speed limit was supplied.
        actuator = mjcf.find(f"actuator/motor[@joint='{joint.get('name')}']")
        assert float(limit.get('effort')) == numbers(actuator.get('forcerange'))[1]
        for urdf_key, mjcf_key in (('damping', 'damping'), ('friction', 'frictionloss')):
            assert float(joint.find('dynamics').get(urdf_key)) == float(reference.get(mjcf_key))
    assert moving == [f'Joint_link{i}' for i in range(1, 5)]


def test_mujoco_loads_with_original_actuator_limits():
    mujoco = pytest.importorskip('mujoco')
    model = mujoco.MjModel.from_xml_path(str(MJCF))
    data = mujoco.MjData(model)
    assert (model.nbody, model.nq, model.nu) == (10, 4, 4)
    assert not model.jnt_actfrclimited.any()
    for sign in (-1, 1):
        data.ctrl[:] = sign * 100
        mujoco.mj_forward(model, data)
        np.testing.assert_allclose(data.qfrc_actuator, sign * np.array([10, 5, 5, 5]))
    for _ in range(100):
        mujoco.mj_step(model, data)
    assert np.isfinite(data.qpos).all()
    assert np.isfinite(data.qvel).all()
