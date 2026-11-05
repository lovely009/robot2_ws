"""复核3～7：凸包、网格闭合性，以及单结构件均匀密度假设下的惯性。"""
import hashlib
import json
import struct

import numpy as np
from scipy.spatial import ConvexHull

from audit_inertia import MODEL, audit


def uniform_properties(triangles, mass):
    """有向闭合三角面四面体积分；仅用于均匀密度假设对照。"""
    a, b, c = triangles.transpose(1, 0, 2)
    volumes = np.einsum('ij,ij->i', a, np.cross(b, c)) / 6
    volume = volumes.sum()
    sums = triangles.sum(axis=1)
    com = (volumes[:, None] * sums / 4).sum(axis=0) / volume
    second = np.einsum('i,ijk->jk', volumes,
                      np.einsum('ij,ik->ijk', sums, sums)
                      + np.einsum('nji,njk->nik', triangles, triangles)) / 20
    inertia = (np.trace(second) * np.eye(3) - second) * mass / volume
    inertia -= mass * ((com @ com) * np.eye(3) - np.outer(com, com))
    return dict(volume_m3=float(volume), density_kg_m3=float(mass / volume),
                com_m=com.tolist(), inertia_kg_m2=inertia.tolist())


def recheck():
    results = []
    for row in audit()['links']:
        if row['link'] in {'base_link', 'Joint_Link1.1', 'Joint_Link1', 'Joint_Link2'}:
            continue
        data = (MODEL / 'meshes' / f'{row["link"]}.STL').read_bytes()
        count = struct.unpack_from('<I', data, 80)[0]
        dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
        triangles = np.frombuffer(data, dtype=dtype, count=count, offset=84)['vertices'].astype(float)
        vertices, indices = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
        faces = indices.reshape(-1, 3)
        edges = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
        _, inverse, counts = np.unique(np.sort(edges, axis=1), axis=0, return_inverse=True, return_counts=True)
        winding = np.bincount(inverse, weights=np.where(edges[:, 0] < edges[:, 1], 1, -1))
        closed = bool((counts == 2).all() and (winding == 0).all())
        hull = ConvexHull(vertices)
        com = np.array(row['com_m'])
        row.update(mesh_closed_oriented_edges=closed, nonmanifold_or_boundary_edges=int((counts != 2).sum()),
                   com_convex_hull_margin_m=float(-(hull.equations[:, :3] @ com + hull.equations[:, 3]).max()))
        if row['link'] == 'Joint_Link7' and closed:
            row['uniform_density_hypothesis'] = uniform_properties(triangles, row['mass_kg'])
            predicted = np.diag(row['uniform_density_hypothesis']['inertia_kg_m2'])
            row['declared_to_uniform_diagonal_ratio'] = [float(row['inertia_kg_m2'][key]) / value
                                                        for key, value in zip(['ixx', 'iyy', 'izz'], predicted)]
        results.append(row)
    # 仓库根目录固定由模型路径定位，不读取网络规格的可变版本。
    docs = MODEL.parents[4] / 'docs/巨蟹电机'
    sources = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in docs.glob('*参数规格.pdf')}
    return dict(scope='Joint_Link3..7 and their fixed motor links; Joint_Link1/2 excluded by user',
                total_mass_kg=sum(row['mass_kg'] for row in results), spec_sha256=sources, links=results)


if __name__ == '__main__':
    # 自检四面体的解析结果，避免把积分公式的错误当成模型缺陷。
    vertices = np.array([[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.]])
    tetra = vertices[np.array([[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]])]
    result = uniform_properties(tetra, 1.)
    assert np.isclose(result['volume_m3'], 1 / 6)
    assert np.allclose(result['com_m'], [.25] * 3)
    assert np.allclose(result['inertia_kg_m2'], np.eye(3) / 16 + np.ones((3, 3)) / 80)
    result = recheck()
    assert len(result['spec_sha256']) == 2
    assert all(row['com_convex_hull_margin_m'] >= 0 for row in result['links'])
    assert all(all(row['checks'].values()) for row in result['links'])
    target = MODEL / 'cad/inertial_audit/2026-11-05-mass-recheck.json'
    target.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print('3～7惯性必要条件、凸包及积分公式自检通过；复核记录：', target)
