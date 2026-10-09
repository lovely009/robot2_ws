#!/usr/bin/env python3
"""装配位形下的 URDF 运动学 + STL 包围盒探针（只读分析，不入库）。

对给定 URDF：按 origin 做 FK（关节角全 0），把每个 link 的 STL 顶点变换到
base_link 系，输出包围盒/质心，供人工核对物理装配关系。
"""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def origin_matrix(element) -> np.ndarray:
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


def vec(text, default=(0, 0, 0)):
    return np.array([float(v) for v in text.split()]) if text else np.array(default)


def read_stl_vertices(path: Path) -> np.ndarray:
    raw = path.read_bytes()
    size = len(raw)
    if size >= 84:
        n = int(np.frombuffer(raw[80:84], dtype="<u4")[0])
        if size == 84 + 50 * n:
            rec = np.frombuffer(
                raw[84:],
                dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]),
            )
            return rec["v"].reshape(-1, 3).astype(np.float64)
    # ASCII fallback
    lines = raw.decode("utf-8", "ignore").splitlines()
    pts = [
        [float(x) for x in ln.split()[1:4]]
        for ln in lines
        if ln.strip().startswith("vertex")
    ]
    return np.array(pts, dtype=np.float64)


def main(urdf_path: Path, pkg_name: str, pkg_root: Path) -> None:
    root = ET.parse(urdf_path).getroot()
    joints = {}
    for j in root.findall("joint"):
        name = j.get("name")
        joints[name] = {
            "type": j.get("type"),
            "parent": j.find("parent").get("link"),
            "child": j.find("child").get("link"),
            "T": origin_matrix(j),
            "axis": vec(j.find("axis").get("xyz")) if j.find("axis") is not None else None,
            "limit": (j.find("limit").attrib if j.find("limit") is not None else {}),
        }
    links = {l.get("name"): l for l in root.findall("link")}

    children = {}
    child_of = {}
    for name, j in joints.items():
        children.setdefault(j["parent"], []).append(name)
        child_of[j["child"]] = name

    # FK from root link (no parent joint)
    roots = [name for name in links if name not in child_of]
    pose = {}
    order = []
    stack = [(roots[0], np.eye(4), None)]
    while stack:
        link, T, via = stack.pop()
        pose[link] = T
        order.append((link, via))
        for jname in children.get(link, []):
            stack.append((joints[jname]["child"], T @ joints[jname]["T"], jname))

    print(f"root link(s): {roots}")
    print(f"{'link':<16} {'via joint':<16} pos(base) / bbox min..max (m)")
    for link, via in order:
        label = f"{link:<16} {str(via):<16}"
        entries = []
        for visual in links[link].findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                continue
            rel = mesh.get("filename").split(f"package://{pkg_name}/", 1)[1]
            vis = origin_matrix(visual)
            verts = read_stl_vertices(pkg_root / rel)
            stem = np.hstack([verts, np.ones((len(verts), 1))])
            world = (pose[link] @ vis @ stem.T).T[:, :3]
            lo, hi = world.min(axis=0), world.max(axis=0)
            entries.append(f"min({lo[0]:+.3f},{lo[1]:+.3f},{lo[2]:+.3f}) max({hi[0]:+.3f},{hi[1]:+.3f},{hi[2]:+.3f})")
        print(f"{label} " + (" | ".join(entries) if entries else "(no mesh)"))

    print("\njoints (world pose of joint origin + axis dir in base frame):")
    for name, j in joints.items():
        T = pose[j["parent"]] @ j["T"]
        pos = T[:3, 3]
        axis_w = T[:3, :3] @ j["axis"] if j["axis"] is not None else np.zeros(3)
        lim = j["limit"]
        lim_s = (
            f"limit[{lim.get('lower')}..{lim.get('upper')}] e={lim.get('effort')} v={lim.get('velocity')}"
            if lim
            else ""
        )
        print(
            f"{name:<16} parent={j['parent']:<14} child={j['child']:<14} "
            f"pos=({pos[0]:+.4f},{pos[1]:+.4f},{pos[2]:+.4f}) "
            f"axis=({axis_w[0]:+.3f},{axis_w[1]:+.3f},{axis_w[2]:+.3f}) {lim_s}"
        )


if __name__ == "__main__":
    urdf = Path(sys.argv[1])
    pkg_name = sys.argv[2]
    pkg_root = Path(sys.argv[3]) if len(sys.argv) > 3 else urdf.parent.parent
    main(urdf, pkg_name, pkg_root)
