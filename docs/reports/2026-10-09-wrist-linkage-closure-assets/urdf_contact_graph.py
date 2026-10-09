#!/usr/bin/env python3
"""装配位形接触图分析：
- 计算所有 link 对的最小间距（装配位形，降采样点云）；
- 列出 URDF 树中的边在各 link 对中的最小间距（判定树边是否几何相接）；
- 列出非树边但几何相接的对（判定闭环切口/固定连接缺失）。
"""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return (
        np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    )


def origin_matrix(element):
    origin = element.find("origin")
    xyz = np.zeros(3)
    rpy = np.zeros(3)
    if origin is not None:
        if origin.get("xyz"):
            xyz = np.array([float(v) for v in origin.get("xyz").split()])
        if origin.get("rpy"):
            rpy = np.array([float(v) for v in origin.get("rpy").split()])
    t = np.eye(4)
    t[:3, :3] = rpy_to_matrix(*rpy)
    t[:3, 3] = xyz
    return t


def read_stl_triangles(path):
    raw = path.read_bytes()
    n = int(np.frombuffer(raw[80:84], dtype="<u4")[0])
    if len(raw) == 84 + 50 * n:
        rec = np.frombuffer(
            raw[84:],
            dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]),
        )
        return rec["v"].astype(np.float64)
    lines = raw.decode("utf-8", "ignore").splitlines()
    return np.array(
        [[float(x) for x in ln.split()[1:4]] for ln in lines if ln.strip().startswith("vertex")],
        dtype=np.float64,
    ).reshape(-1, 3, 3)


def main(urdf_path, pkg_name, pkg_root, tol):
    root = ET.parse(urdf_path).getroot()
    edges = []
    joints = {}
    for j in root.findall("joint"):
        parent, child = j.find("parent").get("link"), j.find("child").get("link")
        edges.append((parent, child))
        joints[child] = {"parent": parent, "T": origin_matrix(j)}
    links = {l.get("name"): l for l in root.findall("link")}
    children = {}
    for child, j in joints.items():
        children.setdefault(j["parent"], []).append(child)
    roots = [n for n in links if n not in joints]
    pose = {}
    stack = [(roots[0], np.eye(4))]
    while stack:
        link, T = stack.pop()
        pose[link] = T
        for c in children.get(link, []):
            stack.append((c, T @ joints[c]["T"]))

    clouds, trees = {}, {}
    for link, elem in links.items():
        pts_all = []
        for visual in elem.findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                continue
            rel = mesh.get("filename").split(f"package://{pkg_name}/", 1)[1]
            tris = read_stl_triangles(pkg_root / rel)
            pts = tris.reshape(-1, 3)
            pts = pts[:: max(1, len(pts) // 4000)]
            h = np.hstack([pts, np.ones((len(pts), 1))])
            pts_all.append((origin_matrix(visual) @ h.T).T[:, :3])
        cloud = np.vstack(pts_all)
        h = np.hstack([cloud, np.ones((len(cloud), 1))])
        world = (pose[link] @ h.T).T[:, :3]
        clouds[link] = world
        trees[link] = cKDTree(world)

    names = list(links)
    dist = {}
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            d, _ = trees[a].query(clouds[b])
            dist[(a, b)] = d.min()

    print("=== URDF 树边的最小间距（应≈0：关节处几何相接）===")
    for a, b in edges:
        d = dist.get((a, b), dist.get((b, a)))
        flag = "" if d < tol else "   <-- 树边但几何不相接！"
        print(f"{a:16s} -> {b:16s} d={d*1000:9.2f} mm{flag}")

    print(f"\n=== 非树边但间距 < {tol*1000:.0f} mm 的 link 对（疑似缺失连接/闭环切口）===")
    for (a, b), d in sorted(dist.items(), key=lambda kv: kv[1]):
        if d >= tol:
            continue
        if (a, b) in edges or (b, a) in edges:
            continue
        print(f"{a:16s} <-> {b:16s} d={d*1000:9.2f} mm")


if __name__ == "__main__":
    urdf = Path(sys.argv[1])
    pkg_name = sys.argv[2]
    pkg_root = Path(sys.argv[3])
    tol = float(sys.argv[4]) if len(sys.argv) > 4 else 0.002
    main(urdf, pkg_name, pkg_root, tol)
