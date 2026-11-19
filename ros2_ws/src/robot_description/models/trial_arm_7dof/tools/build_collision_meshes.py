#!/usr/bin/env python3
"""以 VTK 降至约4000面，再用固定种子 CoACD 凸分解；只写仿真碰撞件。

shortcut: 凸分解和降采样近似接触边界，需精确孔壁接触时改用CAD专用碰撞体。
"""
import hashlib
import json
from pathlib import Path
import tempfile

import coacd
import trimesh
import vtk
from vtk.util.numpy_support import vtk_to_numpy


PARAMETERS = dict(threshold=.03, max_convex_hull=32, resolution=2000,
                  mcts_iterations=50, seed=0)


def completed(path, output):
    marker = output / f'{path.stem}.complete.json'
    if not marker.exists():
        return False
    manifest = json.loads(marker.read_text())
    return (manifest['source_sha256'] == hashlib.sha256(path.read_bytes()).hexdigest()
            and manifest['parameters'] == PARAMETERS
            and set(manifest['parts']) == {p.name for p in output.glob(path.stem + '_*.obj')}
            and all(hashlib.sha256((output / name).read_bytes()).hexdigest() == digest
                    for name, digest in manifest['parts'].items()))


def build():
    model = Path(__file__).resolve().parents[1]
    output = model / 'meshes/collision'
    output.mkdir(exist_ok=True)
    coacd.set_log_level('error')
    for path in sorted((model / 'meshes').glob('*.STL')):
        if completed(path, output):
            continue
        reader = vtk.vtkSTLReader()
        reader.SetFileName(str(path))
        reader.Update()
        decimator = vtk.vtkQuadricDecimation()
        decimator.SetInputConnection(reader.GetOutputPort())
        decimator.SetTargetReduction(max(0, 1 - 4000 / reader.GetOutput().GetNumberOfCells()))
        decimator.Update()
        poly = decimator.GetOutput()
        vertices = vtk_to_numpy(poly.GetPoints().GetData()).astype(float)
        faces = vtk_to_numpy(poly.GetPolys().GetData()).reshape(-1, 4)[:, 1:]
        parts = coacd.run_coacd(coacd.Mesh(vertices, faces), **PARAMETERS)
        assert parts, f'{path}: convex decomposition produced no parts'
        with tempfile.TemporaryDirectory(dir=output) as temporary:
            staging = Path(temporary)
            for i, (vertices, faces) in enumerate(parts):
                trimesh.Trimesh(vertices, faces, process=False).export(staging / f'{path.stem}_{i:02}.obj')
            manifest = dict(source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                            parameters=PARAMETERS,
                            parts={p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                   for p in sorted(staging.glob('*.obj'))})
            # 完成标记最后替换；中途退出会在下次校验时重建，不能复用半套分件。
            for old in output.glob(path.stem + '_*.obj'):
                old.unlink()
            for part in staging.glob('*.obj'):
                part.replace(output / part.name)
            marker = staging / 'complete.json'
            marker.write_text(json.dumps(manifest, indent=2) + '\n')
            marker.replace(output / f'{path.stem}.complete.json')
        print(path.stem, len(parts), flush=True)


if __name__ == '__main__':
    build()
