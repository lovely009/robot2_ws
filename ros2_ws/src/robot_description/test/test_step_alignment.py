"""Regress shoulder plates and wrist alignment against actual STEP landmarks."""
import json
from pathlib import Path

import numpy as np
import pytest

MODEL = Path(__file__).parents[1] / 'models/full_body_robot'


def test_step_reference_matches_assembly_landmarks_without_motion():
    mujoco = pytest.importorskip('mujoco')
    vtk = pytest.importorskip('vtk')
    from vtk.util.numpy_support import vtk_to_numpy
    from scipy.spatial import cKDTree

    model = mujoco.MjModel.from_xml_path(str(MODEL / 'mujoco/full_body_robot.step-aligned.xml'))
    assert model.njnt == model.nu == model.neq == 0
    assert model.nmesh == model.ngeom == 38
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    manifest = json.loads((MODEL / 'config/step_alignment.json').read_text())
    fixture = json.loads((MODEL / 'cad/assembly_audit/step-landmarks.json').read_text())
    assert fixture['step_sha256'] == manifest['step_sha256']
    assert set(manifest['links']) == set(fixture['landmarks'])
    for name, entry in manifest['links'].items():
        reader = vtk.vtkSTLReader()
        reader.SetFileName(str(MODEL / 'mujoco/visual/step_aligned' / entry['source']))
        reader.Update()
        vertices = vtk_to_numpy(reader.GetOutput().GetPoints().GetData())
        body = data.body(name)
        world = vertices @ body.xmat.reshape(3, 3).T + body.xpos
        distance = cKDTree(world).query(fixture['landmarks'][name])[0]
        assert max(distance) < .002, (name, max(distance))
