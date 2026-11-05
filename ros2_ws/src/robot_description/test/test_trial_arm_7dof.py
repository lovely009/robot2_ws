"""新手臂导入保真、资源解析与惯性必要条件检查。"""
import hashlib
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]
MODEL = PACKAGE / 'models/trial_arm_7dof'


def shape(element):
    return element.tag, element.attrib, (element.text or '').strip(), [shape(c) for c in element]


def test_import_preserves_export_and_resolves_meshes():
    runtime = ET.parse(MODEL / 'urdf/trial_arm_7dof.urdf').getroot()
    assert len(runtime.findall('link')) == 14
    original = ET.parse(MODEL / 'cad/mass_export/urdf/手臂_mass.urdf').getroot()
    original.set('name', MODEL.name)
    prefix = 'package://robot_description/models/trial_arm_7dof/'
    for mesh in original.iter('mesh'):
        mesh.set('filename', mesh.attrib['filename'].replace('package://手臂_mass/', prefix))
    assert shape(original) == shape(runtime)
    joints = runtime.findall('joint')
    assert len(joints) == 13
    assert [joint.get('name') for joint in joints if joint.get('type') == 'revolute'] == [
        f'Joint_Link{i}' for i in range(1, 8)]
    assert sum(joint.get('type') == 'fixed' for joint in joints) == 6
    parent = 'base_link'
    for joint in joints:
        parent_link, child_link = joint.find('parent'), joint.find('child')
        assert parent_link is not None and child_link is not None
        assert parent_link.attrib['link'] == parent
        parent = child_link.attrib['link']
        assert runtime.find(f"link[@name='{parent}']") is not None
    for mesh in runtime.iter('mesh'):
        path = MODEL / mesh.attrib['filename'].removeprefix(prefix)
        assert path.is_file()
        assert hashlib.sha256(path.read_bytes()).digest() == hashlib.sha256(
            (MODEL / 'cad/mass_export/meshes' / path.name).read_bytes()).digest()


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
    assert not models.has_joint_mapping_source(MODEL.name, MODEL.parent)


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
