"""右臂机械参数、网格与来源校验。"""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

MODEL = Path(__file__).resolve().parents[1] / 'models/right_arm_4dof'
URDF = MODEL / 'urdf/right_arm_4dof.urdf'


def test_source_hashes_and_meshes():
    hashes = json.loads((MODEL / 'source_sha256.json').read_text())
    assert set(hashes) == {str(p.relative_to(MODEL / 'cad')) for p in (MODEL / 'cad').rglob('*') if p.is_file()}
    for relative, digest in hashes.items():
        assert hashlib.sha256((MODEL / 'cad' / relative).read_bytes()).hexdigest() == digest
    meshes = list((MODEL / 'meshes').glob('*.STL'))
    assert len(meshes) == 9
    for mesh in meshes:
        assert mesh.read_bytes() == (MODEL / 'cad/meshes' / mesh.name).read_bytes()


def test_mechanical_inertias_and_standalone_are_preserved():
    assert (MODEL / 'urdf/right_arm_4dof.standalone.urdf').read_bytes() == URDF.read_bytes()
    original = ET.parse(MODEL / 'cad/urdf/4.4.urdf').getroot()
    runtime = ET.parse(URDF).getroot()
    assert len(runtime.findall('link')) == 9
    assert len(runtime.findall('joint')) == 8
    for link in runtime.findall('link'):
        source = original.find(f"link[@name='{link.get('name')}']")
        for field in ('mass', 'inertia', 'origin'):
            actual = link.find('inertial/' + field).attrib
            expected = source.find('inertial/' + field).attrib
            assert actual.keys() == expected.keys()
            for key in actual:
                np.testing.assert_allclose(np.fromstring(actual[key], sep=' '), np.fromstring(expected[key], sep=' '), atol=1e-14)
