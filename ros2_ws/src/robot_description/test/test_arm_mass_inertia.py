"""质量属性近似修复的回归：不把必要条件通过当成实物标定。"""
import csv
import hashlib
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
import pytest

MODEL = Path(__file__).resolve().parents[1] / 'models/trial_arm_7dof/cad/mass_export'
URDF = MODEL / 'urdf/手臂_mass.urdf'
CSV = MODEL / 'urdf/手臂_mass.csv'
MASSES = {'Joint_Link3.3': 0.46, 'Joint_Link4.4': 0.46, 'Joint_Link5.5': 0.244}


def required(element: ET.Element, path: str) -> ET.Element:
    found = element.find(path)
    assert found is not None, path
    return found


@pytest.mark.parametrize('name,mass', MASSES.items())
def test_motor_mass_properties_fit_geometry(name, mass):
    link = required(ET.parse(URDF).getroot(), f"link[@name='{name}']")
    inertial = required(link, 'inertial')
    assert float(required(inertial, 'mass').attrib['value']) == mass
    origin = required(inertial, 'origin')
    assert [float(value) for value in origin.attrib['rpy'].split()] == [0., 0., 0.]
    com = np.fromstring(origin.attrib['xyz'], sep=' ')
    a = {k: float(v) for k, v in required(inertial, 'inertia').attrib.items()}
    tensor = np.array([[a['ixx'], a['ixy'], a['ixz']],
                       [a['ixy'], a['iyy'], a['iyz']],
                       [a['ixz'], a['iyz'], a['izz']]])
    assert np.isfinite(tensor).all() and np.isfinite(com).all()
    eigen = np.linalg.eigvalsh(tensor)
    assert (eigen > 0).all()
    assert eigen[0] + eigen[1] >= eigen[2] - 1e-12
    data = (MODEL / 'meshes' / f'{name}.STL').read_bytes()
    count = struct.unpack_from('<I', data, 80)[0]
    assert len(data) == 84 + 50 * count
    dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
    vertices = np.frombuffer(data, dtype=dtype, count=count, offset=84)['vertices'].reshape(-1, 3)
    lower, upper = vertices.min(0), vertices.max(0)
    assert ((com >= lower) & (com <= upper)).all(), (name, com, lower, upper)
    variance = (np.trace(tensor) / 2 - np.diag(tensor)) / mass
    assert (variance >= -1e-12).all()
    assert (variance <= (upper - com) * (com - lower) + 1e-9).all()
    report = json.loads((MODEL / 'cad/inertial_audit/2026-11-05-approximation.json').read_text())
    recorded = report['links'][name]
    assert recorded['mass_kg'] == mass
    assert np.allclose(recorded['com_m'], com, rtol=1e-12, atol=1e-15)
    assert np.allclose(recorded['inertia_kg_m2'], tensor, rtol=1e-12, atol=1e-15)


def shape(element):
    return element.tag, element.attrib, (element.text or '').strip(), [shape(c) for c in element]


def test_only_selected_inertials_changed_and_sources_preserved():
    original = ET.parse(MODEL / 'cad/original/手臂_mass.urdf').getroot()
    repaired = ET.parse(URDF).getroot()
    for root in (original, repaired):
        for name in MASSES:
            link = required(root, f"link[@name='{name}']")
            link.remove(required(link, 'inertial'))
    assert shape(original) == shape(repaired)
    with (MODEL / 'cad/original/手臂_mass.csv').open(newline='') as stream:
        old = list(csv.DictReader(stream))
    with CSV.open(newline='') as stream:
        new = list(csv.DictReader(stream))
    assert len(old) == len(new) == 14
    allowed = {'Center of Mass X', 'Center of Mass Y', 'Center of Mass Z',
               'Mass', 'Moment Ixx', 'Moment Ixy', 'Moment Ixz',
               'Moment Iyy', 'Moment Iyz', 'Moment Izz'}
    for before, after in zip(old, new):
        changes = {key for key in before if before[key] != after[key]}
        assert changes <= (allowed if before['Link Name'] in MASSES else set())
    report = json.loads((MODEL / 'cad/inertial_audit/2026-11-05-approximation.json').read_text())
    assert report['physical_validation'] is False
    assert set(report['links']) == set(MASSES)
    for relative, digest in report['sha256'].items():
        assert hashlib.sha256((MODEL / relative).read_bytes()).hexdigest() == digest


def test_csv_inertials_match_repaired_urdf():
    with CSV.open(newline='') as stream:
        rows = {row['Link Name']: row for row in csv.DictReader(stream)}
    for link in ET.parse(URDF).getroot().findall('link'):
        row = rows[link.attrib['name']]
        inertial = required(link, 'inertial')
        assert float(row['Mass']) == float(required(inertial, 'mass').attrib['value'])
        assert [float(row[f'Center of Mass {axis}']) for axis in 'XYZ'] == [
            float(value) for value in required(inertial, 'origin').attrib['xyz'].split()]
        for key, value in required(inertial, 'inertia').attrib.items():
            assert float(row[f'Moment {key.capitalize()}']) == float(value)
