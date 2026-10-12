#!/usr/bin/env python3
"""零位碰撞体相交验收：自碰撞开启前必须为 0 相交（SAT，含交叉型相交）。

为什么需要它：模型的自碰撞是打开的（`<self_collide>true</self_collide>`）。如果两个
**非相邻** link 的碰撞体在零位就相交（常见于把有让位缺口的件简化成 box/凸包），dartsim
会施加巨大的分离力把关节顶死——2026-10-12 实测：`base_link` 的整高 box 与 `Joint_link2`
相交 15.8 mm 时，joint_link2 加 2 N·m 完全不动。

判定：AABB 粗筛候选对（跳过共享关节的相邻对，它们在 dartsim 里默认不检测）→
分离轴定理。轴集 = A 面法向 + B 面法向 + (A 边方向 × B 边方向)。

用法（在模型目录内，需 source 带 pinocchio 的环境，如 ROS 2 Humble）：
  python3 tools/check_collision_interpenetration.py            # 退出码 0 = 无相交
  python3 tools/check_collision_interpenetration.py <urdf路径>
"""

from __future__ import annotations

import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pinocchio as pin
from scipy.spatial import ConvexHull

MODEL_DIR = Path(__file__).resolve().parent.parent
DEFAULT_URDF = MODEL_DIR / "urdf" / f"{MODEL_DIR.name}.urdf"
MESH_PREFIX = "package://robot_arm_gazebo/"
PACKAGE_DIR = MODEL_DIR.parent.parent  # ros2_ws/src/robot_arm_gazebo
EPS = 1e-9


def read_binary_stl(path: Path) -> np.ndarray:
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    usable = (min(len(data) - 84, count * 50) // 50) * 50
    if usable == 0:
        raise RuntimeError(f"{path.name}: 不是二进制 STL 或为空")
    records = np.frombuffer(data, dtype=np.uint8, count=usable, offset=84).reshape(-1, 50)
    return records[:, :48].copy().view("<f4").reshape(-1, 12)[:, 3:12].astype(float).reshape(-1, 3)


def unit_directions(vectors: np.ndarray) -> np.ndarray:
    norms = np.linalg.norm(vectors, axis=1)
    vectors = vectors[norms > 1e-12] / norms[norms > 1e-12][:, None]
    _, index = np.unique(np.round(np.abs(vectors), 6), axis=0, return_index=True)
    return vectors[index]


def box_vertices(size: np.ndarray) -> np.ndarray:
    half = size / 2.0
    return np.array([[x, y, z] for x in (-half[0], half[0])
                     for y in (-half[1], half[1]) for z in (-half[2], half[2])])


def shape_of(collision: ET.Element) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    local = np.array([float(v) for v in collision.find("origin").get("xyz").split()])
    geometry = collision.find("geometry")
    box = geometry.find("box")
    if box is not None:
        points = box_vertices(np.array([float(v) for v in box.get("size").split()]))
    else:
        filename = geometry.find("mesh").get("filename")
        if not filename.startswith(MESH_PREFIX):
            raise RuntimeError(f"网格路径必须以 {MESH_PREFIX} 开头：{filename}")
        points = np.unique(read_binary_stl(PACKAGE_DIR / filename[len(MESH_PREFIX):]),
                           axis=0)
    points = points + local
    hull = ConvexHull(points)
    simplices = hull.simplices
    edges = np.vstack([points[simplices[:, 1]] - points[simplices[:, 0]],
                       points[simplices[:, 2]] - points[simplices[:, 1]],
                       points[simplices[:, 0]] - points[simplices[:, 2]]])
    return points, unit_directions(hull.equations[:, :3]), unit_directions(edges)


def penetration_depth(points_a, normals_a, edges_a, points_b, normals_b, edges_b) -> float:
    """返回穿透深度；<= 0 表示存在分离轴（不相交）。"""
    axes = unit_directions(np.vstack([
        normals_a, normals_b,
        np.cross(edges_a[:, None, :], edges_b[None, :, :]).reshape(-1, 3)]))
    depths = np.empty(len(axes))
    for index, axis in enumerate(axes):
        projection_a = points_a @ axis
        projection_b = points_b @ axis
        depths[index] = (min(projection_a.max(), projection_b.max())
                         - max(projection_a.min(), projection_b.min()))
    return depths.min()


def main() -> int:
    urdf = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_URDF
    root = ET.parse(urdf).getroot()
    model = pin.buildModelFromUrdf(str(urdf))
    data = model.createData()
    pin.forwardKinematics(model, data, pin.neutral(model))
    joint_of_child = {joint.find("child").get("link"): joint for joint in root.findall("joint")}
    adjacent = {frozenset((joint.find("parent").get("link"), joint.find("child").get("link")))
                for joint in root.findall("joint")}

    shapes = {}
    for link in root.findall("link"):
        if link.find("collision") is None:
            continue
        name = link.get("name")
        joint = joint_of_child.get(name)
        index = model.getJointId(joint.get("name")) if joint is not None else 0
        transform = np.eye(4) if index == 0 else np.asarray(data.oMi[index].homogeneous)
        points, normals, edges = shape_of(link.find("collision"))
        points = (transform[:3, :3] @ points.T).T + transform[:3, 3]
        normals = normals @ transform[:3, :3].T
        edges = edges @ transform[:3, :3].T
        shapes[name] = (points, normals, edges)

    print(f"{urdf.name}：{len(shapes)} 个碰撞体，检查非相邻对（零位）")
    hits = 0
    names = sorted(shapes)
    for index, first in enumerate(names):
        for second in names[index + 1:]:
            if frozenset((first, second)) in adjacent:
                continue
            points_a, normals_a, edges_a = shapes[first]
            points_b, normals_b, edges_b = shapes[second]
            if np.any(np.minimum(points_a.max(0), points_b.max(0))
                      - np.maximum(points_a.min(0), points_b.min(0)) <= 0):
                continue
            depth = penetration_depth(points_a, normals_a, edges_a,
                                      points_b, normals_b, edges_b)
            if depth > EPS:
                hits += 1
                print(f"  ✗ {first:16s} ↔ {second:16s} 相交，穿透 ≈ {depth * 1000:.2f} mm")
    if hits:
        print("自碰撞会因此产生假接触（关节可能被顶死），请改用更贴合的形状")
        return 1
    print("  ✅ 无非相邻相交")
    return 0


if __name__ == "__main__":
    sys.exit(main())
