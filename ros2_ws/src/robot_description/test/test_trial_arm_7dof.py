"""新手臂导入保真、资源解析与惯性必要条件检查。"""
import hashlib
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET
import json
import shutil

import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation
from robot_transmission import Axis, Model

PACKAGE = Path(__file__).resolve().parents[1]
MODEL = PACKAGE / 'models/trial_arm_7dof'


def shape(element):
    return element.tag, element.attrib, (element.text or '').strip(), [shape(c) for c in element]


def forward_kinematics(root, positions=None):
    positions = positions or {}
    world = {'base_link': np.eye(4)}
    pending = list(root.findall('joint'))
    while pending:
        ready = [j for j in pending if j.find('parent').get('link') in world]
        assert ready, 'invalid link tree'
        for joint in ready:
            origin = joint.find('origin')
            transform = np.eye(4)
            transform[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
            transform[:3, :3] = Rotation.from_euler(
                'xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
            if joint.get('type') == 'revolute':
                axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                transform[:3, :3] = transform[:3, :3] @ Rotation.from_rotvec(
                    axis * positions.get(joint.get('name'), 0)).as_matrix()
            world[joint.find('child').get('link')] = world[joint.find('parent').get('link')] @ transform
            pending.remove(joint)
    return world


def test_import_preserves_export_and_resolves_meshes():
    runtime = ET.parse(MODEL / 'urdf/trial_arm_7dof.urdf').getroot()
    original = ET.parse(MODEL / 'cad/mass_export/urdf/手臂_mass.urdf').getroot()
    assert len(runtime.findall('link')) == 15
    prefix = 'package://robot_description/models/trial_arm_7dof/'
    for mesh in original.iter('mesh'):
        mesh.set('filename', mesh.attrib['filename'].replace('package://手臂_mass/', prefix))
    old_world, new_world = forward_kinematics(original), forward_kinematics(runtime)
    for link in original.findall('link'):
        name = link.get('name')
        assert shape(link) == shape(runtime.find(f"link[@name='{name}']"))
        np.testing.assert_allclose(old_world[name], new_world[name], atol=1e-12, rtol=0)
    frame = runtime.find("link[@name='Joint_Link2_axis']")
    assert frame is not None and len(frame) == 0
    joints = runtime.findall('joint')
    assert len(joints) == 14
    assert [j.get('name') for j in joints if j.get('type') == 'revolute'] == [
        f'Joint_Link{i}' for i in range(1, 8)]
    assert sum(j.get('type') == 'fixed' for j in joints) == 7
    for old in original.findall('joint'):
        new = runtime.find(f"joint[@name='{old.get('name')}']")
        if old.get('name') != 'Joint_Link2':
            assert shape(new) == shape(old)
        elif old.get('type') == 'revolute':
            assert shape(new.find('limit')) == shape(old.find('limit'))
    for mesh in runtime.iter('mesh'):
        path = MODEL / mesh.attrib['filename'].removeprefix(prefix)
        assert path.is_file()
        assert hashlib.sha256(path.read_bytes()).digest() == hashlib.sha256(
            (MODEL / 'cad/mass_export/meshes' / path.name).read_bytes()).digest()


def test_pitch_axis_and_ball_trajectory_match_step():
    runtime = ET.parse(MODEL / 'urdf/trial_arm_7dof.urdf').getroot()
    zero = forward_kinematics(runtime)
    axis_frame = zero['Joint_Link2_axis']
    np.testing.assert_allclose(axis_frame[:3, 3], [.0025, .06775, .5025], atol=1e-12, rtol=0)
    np.testing.assert_allclose(axis_frame[:3, 2], [0, -1, 0], atol=1e-12, rtol=0)
    ball = np.array([.035840440737, .06775, .494319840327, 1])
    local_ball = np.linalg.solve(zero['Joint_Link2'], ball)
    o1, o2 = np.array([-.0025, .06725, .4825]), np.array([.0025, .06775, .5025])
    for q1, q2 in [(0, .2), (.17, -.1), (.17, .08)]:
        world = forward_kinematics(runtime, {'Joint_Link1': q1, 'Joint_Link2': q2})
        expected = o1 + Rotation.from_rotvec([0, 0, q1]).apply(
            o2-o1 + Rotation.from_rotvec([0, -q2, 0]).apply(ball[:3]-o2))
        np.testing.assert_allclose((world['Joint_Link2'] @ local_ball)[:3], expected, atol=1e-12, rtol=0)


def test_inertial_consistency():
    spec = importlib.util.spec_from_file_location('trial_arm_audit', MODEL / 'tools/audit_inertia.py')
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    result = module.audit()
    assert len(result['links']) == 14
    for link in result['links']:
        assert all(link['checks'].values()), link


def test_model_catalog_includes_preview(monkeypatch):
    monkeypatch.syspath_prepend(str(PACKAGE.parent / 'web_host'))
    models = importlib.import_module('web_host.models')
    assert MODEL.name in models.enumerate_arm_models(MODEL.parent)
    assert '手臂_mass' not in models.enumerate_arm_models(MODEL.parent)
    assert models.has_joint_mapping_source(MODEL.name, MODEL.parent)


def test_trial_mapping_is_offline_coupled_and_complete():
    driver = MODEL / 'config/motor_driver.yaml'
    hardware = yaml.safe_load(driver.read_text())['motor_hardware']
    assert set(hardware) == {'transmissions_file', 'motor_specs_file', 'motors'}
    assert (driver.parent / hardware['motor_specs_file']).resolve() == PACKAGE / 'config/motor.yaml'
    assert len(hardware['motors']) == 7
    for i in range(1, 8):
        assert hardware['motors'][f'arm_joint{i}'] == {
            'id': i, 'joint': f'Joint_Link{i}',
            'type': 'cerb_r58_101' if i in (3, 4) else 'cerb_r48_101'}
    model = Model.load(str(driver))
    assert model.actuator_ids == {f'arm_joint{i}': i for i in range(1, 8)}
    state = model.actuator_to_joint({n: Axis() for n in model.actuator_names})
    assert set(state) == {f'Joint_Link{i}' for i in range(1, 8)}
    assert all(abs(v.position) < 1e-9 for v in state.values())
    coupled, *direct = model.groups
    assert coupled.actuators == ['arm_joint1', 'arm_joint2']
    assert coupled.joints == ['Joint_Link1', 'Joint_Link2']
    assert coupled.position_dependent and len(direct) == 5
    assert all(not g.position_dependent and len(g.joints) == 1 for g in direct)
    assert coupled.actuator_reduction == [1, 1]
    expected = {
        'motor_axis_origin': [.033699999952, .04475, .4], 'motor_axis_direction': [1, 0, 0],
        'motor_ball_zero': [.033699999952, .061556509578, .389158356406],
        'joint1_axis_origin': [-.0025, .06725, .4825], 'joint1_axis_direction': [0, 0, 1],
        'joint2_axis_origin': [.0025, .06775, .5025], 'joint2_axis_direction': [0, -1, 0],
        'joint_ball_zero': [.035840440737, .06775, .494319840327],
        'rod_length': [.10536545216923923],
    }
    assert set(coupled.parameters) == set(expected)
    for key, values in expected.items():
        np.testing.assert_allclose(np.fromstring(coupled.parameters[key], sep=' '), values, atol=1e-14, rtol=0)
    evidence = json.loads((MODEL / 'cad/transmission_audit/2026-11-05-step.json').read_text())
    np.testing.assert_allclose(np.array(evidence['ball_centers_mm']) / 1000,
                               [expected['motor_ball_zero'], expected['joint_ball_zero']], atol=1e-14, rtol=0)
    assert not (MODEL / 'mjcf').exists()
    assert not list((MODEL / 'config').glob('*hil*'))


def test_audit_rejects_missing_physical_inertial(tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location('trial_arm_missing_audit', MODEL / 'tools/audit_inertia.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    (tmp_path / 'urdf').mkdir()
    (tmp_path / 'cad/mass_export/urdf').mkdir(parents=True)
    shutil.copy(MODEL / 'cad/mass_export/urdf/手臂_mass.csv', tmp_path / 'cad/mass_export/urdf')
    root = ET.parse(MODEL / 'urdf/trial_arm_7dof.urdf').getroot()
    link = root.find("link[@name='base_link']")
    link.remove(link.find('inertial'))
    ET.ElementTree(root).write(tmp_path / 'urdf/trial_arm_7dof.urdf')
    monkeypatch.setattr(module, 'MODEL', tmp_path)
    with pytest.raises(AssertionError, match='base_link'):
        module.audit()


def test_recheck_covers_split_motor_links_without_combined_mass_assumptions(monkeypatch):
    monkeypatch.syspath_prepend(str(MODEL / 'tools'))
    recheck_inertia = importlib.import_module('recheck_inertia')
    result = recheck_inertia.recheck()
    expected = {f'Joint_Link{i}' for i in range(3, 8)} | {
        f'Joint_Link{i}.{i}' for i in range(3, 8)}
    assert {row['link'] for row in result['links']} == expected
    for row in result['links']:
        assert 'remainder_mass_kg' not in row
        assert row['com_convex_hull_margin_m'] >= 0
        assert all(row['checks'].values())
