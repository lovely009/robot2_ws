"""离线检查声明参数与网格边界；不由均匀密度假设重算电机惯量。"""
import csv
import hashlib
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np

MODEL = Path(__file__).resolve().parents[1]


def audit():
    root = ET.parse(MODEL / 'urdf/trial_arm_7dof.urdf').getroot()
    with (MODEL / 'cad/mass_export/urdf/手臂_mass.csv').open(newline='') as stream:
        rows = {row['Link Name']: row for row in csv.DictReader(stream)}
    physical_names = {link.get('name') for link in root.findall('link')} - {'Joint_Link2_axis'}
    assert physical_names == set(rows), 'physical links must match the original CSV'
    results = []
    for link in root.findall('link'):
        name = link.get('name')
        if name == 'Joint_Link2_axis':
            assert len(link) == 0, 'Joint_Link2_axis must remain an empty frame'
            continue
        inertial = link.find('inertial')
        assert inertial is not None, f'{name}: missing physical inertial'
        mass = float(inertial.find('mass').get('value'))
        com = np.fromstring(inertial.find('origin').get('xyz'), sep=' ')
        a = inertial.find('inertia').attrib
        tensor = np.array([[float(a['ixx']), float(a['ixy']), float(a['ixz'])],
                           [float(a['ixy']), float(a['iyy']), float(a['iyz'])],
                           [float(a['ixz']), float(a['iyz']), float(a['izz'])]])
        eigen = np.linalg.eigvalsh(tensor)
        mesh = MODEL / 'meshes' / f'{name}.STL'
        data = mesh.read_bytes()
        count = struct.unpack_from('<I', data, 80)[0]
        assert len(data) == 84 + 50 * count, mesh
        dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
        vertices = np.frombuffer(data, dtype=dtype, count=count, offset=84)['vertices'].reshape(-1, 3)
        lower, upper = vertices.min(0).astype(float), vertices.max(0).astype(float)
        row = rows[name]
        csv_values = [float(row[key]) for key in ['Mass', 'Center of Mass X', 'Center of Mass Y',
                      'Center of Mass Z', 'Moment Ixx', 'Moment Ixy', 'Moment Ixz',
                      'Moment Iyy', 'Moment Iyz', 'Moment Izz']]
        urdf_values = [mass, *com, *[float(a[key]) for key in ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz']]]
        # 由质心惯量得到各轴坐标方差，检查包围盒所允许的方差上界。
        variance = (np.trace(tensor) / 2 - np.diag(tensor)) / mass
        checks = dict(finite=bool(np.isfinite(urdf_values).all()), positive_mass=mass > 0,
                      positive_definite=bool((eigen > 0).all()),
                      triangle_inequality=bool(eigen[0] + eigen[1] >= eigen[2] - 1e-12),
                      com_in_mesh_bounds=bool(((com >= lower - 1e-6) & (com <= upper + 1e-6)).all()),
                      variance_within_bounds=bool((variance <= (upper-com)*(com-lower) + 1e-9).all()),
                      csv_matches=bool(np.array_equal(csv_values, urdf_values)))
        results.append(dict(link=name, mass_kg=mass, com_m=com.tolist(), inertia_kg_m2=a,
                            principal_inertia_kg_m2=eigen.tolist(), mesh_bounds_m=[lower.tolist(), upper.tolist()],
                            mesh_sha256=hashlib.sha256(data).hexdigest(), components=row['SW Components'],
                            checks=checks))
    return dict(total_mass_kg=sum(row['mass_kg'] for row in results), links=results)


if __name__ == '__main__':
    result = audit()
    destination = MODEL / 'cad/inertial_audit'
    destination.mkdir(exist_ok=True)
    (destination / '2026-11-05-mass.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps({row['link']: row['checks'] for row in result['links']}, indent=2))
