#!/usr/bin/env python3
"""顶点聚类降采样 STL（纯 numpy，无第三方依赖）。

用途：MuJoCo 的 STL 解码器**上限 20 万三角面**，而 `base_link.STL` 是 50 万面
（其余 18 个 mesh 都在限内）。这里把顶点量化到体素网格、每簇取均值、重映射三角形并
去掉退化/重复面，得到 ≤20 万面的等价外观网格。

只用于 **MuJoCo 的 visual 网格**（`meshes/base_link_simplified.STL`）；Gazebo/ROS 侧
仍用全分辨率 `meshes/base_link.STL`，碰撞体另有 `meshes/collision/*` 与 box。

用法（在模型目录内）：
  python3 tools/simplify_mesh.py meshes/base_link.STL meshes/base_link_simplified.STL [体素大小 m]
"""

from __future__ import annotations

import struct
import sys
from pathlib import Path

import numpy as np

MUJOCO_FACE_LIMIT = 200_000


def read_binary_stl(path: Path) -> np.ndarray:
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    usable = (min(len(data) - 84, count * 50) // 50) * 50
    if usable == 0:
        raise RuntimeError(f"{path.name}: 不是二进制 STL 或为空")
    records = np.frombuffer(data, dtype=np.uint8, count=usable, offset=84).reshape(-1, 50)
    return records[:, :48].copy().view("<f4").reshape(-1, 12)[:, 3:12].astype(float).reshape(-1, 3, 3)


def write_binary_stl(path: Path, triangles: np.ndarray) -> None:
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    lengths[lengths == 0.0] = 1.0
    normals = normals / lengths[:, None]
    record = np.zeros((len(triangles), 50), dtype=np.uint8)
    record[:, :48] = np.concatenate(
        [normals, triangles.reshape(-1, 9)], axis=1
    ).astype("<f4").view(np.uint8).reshape(-1, 48)
    header = f"vertex-clustered simplification of {path.name}".encode()[:79].ljust(80, b"\0")
    path.write_bytes(header + struct.pack("<I", len(triangles)) + record.tobytes())


def decimate(triangles: np.ndarray, voxel: float) -> np.ndarray:
    vertices = triangles.reshape(-1, 3)
    _, inverse = np.unique(np.floor(vertices / voxel).astype(np.int64),
                           axis=0, return_inverse=True)
    clustered = np.zeros((inverse.max() + 1, 3))
    np.add.at(clustered, inverse, vertices)
    clustered /= np.bincount(inverse)[:, None]
    faces = clustered[inverse].reshape(-1, 3, 3)
    edges = [np.linalg.norm(faces[:, a] - faces[:, b], axis=1) for a, b in ((0, 1), (1, 2), (2, 0))]
    faces = faces[np.all(np.array(edges) > 1e-9, axis=0)]
    _, unique = np.unique(np.round(faces, 6).reshape(len(faces), -1), axis=0, return_index=True)
    return faces[np.sort(unique)]


def main() -> int:
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    source, target = Path(sys.argv[1]), Path(sys.argv[2])
    voxel = float(sys.argv[3]) if len(sys.argv) > 3 else 0.004
    triangles = read_binary_stl(source)
    print(f"输入 {source.name}: {len(triangles)} 面")
    for _ in range(8):
        result = decimate(triangles, voxel)
        print(f"  voxel={voxel * 1000:.2f} mm → {len(result)} 面")
        if len(result) <= MUJOCO_FACE_LIMIT * 0.95:
            break
        voxel *= 1.4
    if len(result) > MUJOCO_FACE_LIMIT:
        print(f"仍超过 MuJoCo 限制 {MUJOCO_FACE_LIMIT}", file=sys.stderr)
        return 1
    write_binary_stl(target, result)
    print(f"写出 {target}（{len(result)} 面，{target.stat().st_size / 1e6:.1f} MB）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
