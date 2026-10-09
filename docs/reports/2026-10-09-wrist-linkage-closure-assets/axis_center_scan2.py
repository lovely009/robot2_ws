#!/usr/bin/env python3
"""轴心标定 v2：轴方向取"垂直于臂轴"的两个候选（竖直 z / 横向 y），
扫描轴心位置，统计 8 个腕位形的可解性。
"""

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

PKG = Path("/home/ly/robot_ws/ros2_ws/src/robot_arm_gazebo")
URDF = PKG / "models/zhoubuguanjie_youbi_001/urdf/zhoubuguanjie_youbi_001.urdf"


def rpy_to_R(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return (
        np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    )


def origin_T(el):
    o = el.find("origin")
    xyz = rpy = np.zeros(3)
    if o is not None:
        if o.get("xyz"):
            xyz = np.array([float(v) for v in o.get("xyz").split()])
        if o.get("rpy"):
            rpy = np.array([float(v) for v in o.get("rpy").split()])
    T = np.eye(4)
    T[:3, :3] = rpy_to_R(*rpy)
    T[:3, 3] = xyz
    return T


def axis_T(axis, q):
    axis = axis / np.linalg.norm(axis)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    T = np.eye(4)
    T[:3, :3] = np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * (K @ K)
    return T


def solve_T(joints, q):
    children = {}
    for name, j in joints.items():
        children.setdefault(j["parent"], []).append(name)
    roots = [l for l in {j["parent"] for j in joints.values()} if l not in {j["child"] for j in joints.values()}]
    T = {}
    stack = [(roots[0], np.eye(4))]
    while stack:
        link, Tw = stack.pop()
        T[link] = Tw
        for jn in children.get(link, []):
            j = joints[jn]
            Tj = Tw @ j["T"]
            if jn in q and j["type"] in ("revolute", "continuous"):
                Tj = Tj @ axis_T(j["axis"], q[jn])
            stack.append((j["child"], Tj))
    return T


root = ET.parse(URDF).getroot()
joints = {}
for j in root.findall("joint"):
    axis = j.find("axis")
    joints[j.get("name")] = {
        "parent": j.find("parent").get("link"),
        "child": j.find("child").get("link"),
        "T": origin_T(j),
        "axis": np.array([float(v) for v in axis.get("xyz").split()]) if axis is not None else np.zeros(3),
        "type": j.get("type"),
    }
T0 = solve_T(joints, {})

tests = [(0.0, 0.0), (0.2, 0.0), (0.0, 0.2), (0.4, 0.2), (-0.3, 0.3), (0.6, -0.3), (0.0, -0.3), (-0.4, 0.0), (0.3, 0.4), (-0.2, -0.4), (0.5, 0.5), (-0.5, 0.2)]


def scan_axis(tag, axis_dir, P9_0, P7s, L):
    """axis_dir: 单位方向（世界）。轴心扫描在其余两维上进行。"""
    d = np.array(axis_dir, dtype=float)
    d = d / np.linalg.norm(d)
    # 与 d 垂直的两个基
    ref = np.eye(3)[int(np.argmin(np.abs(d)))]
    e1 = np.cross(d, ref); e1 /= np.linalg.norm(e1)
    e2 = np.cross(d, e1)

    def perp_dist(p, c):
        r = p - c
        return np.linalg.norm(r - (r @ d) * d)

    results = []
    for u in np.arange(-0.04, 0.0401, 0.001):
        for v in np.arange(-0.04, 0.0401, 0.001):
            c = u * e1 + v * e2
            r = perp_dist(P9_0, c)
            if r < 3e-3:
                continue
            hits = 0
            gaps = []
            for P7 in P7s:
                dx = abs((P7 - P9_0) @ d)
                rho = perp_dist(P7, c)
                dmin = float(np.sqrt(dx * dx + max(rho - r, 0.0) ** 2))
                dmax = float(np.sqrt(dx * dx + (rho + r) ** 2))
                if dmin <= L <= dmax:
                    hits += 1
                else:
                    gaps.append(min(abs(L - dmin), abs(L - dmax)))
            results.append((hits, float(np.mean(gaps)) if gaps else 0.0, c.copy(), r))
    results.sort(key=lambda x: (-x[0], x[1]))
    print(f"\n===== {tag}: 轴方向 d=({d[0]:+.2f},{d[1]:+.2f},{d[2]:+.2f}) P9=({P9_0[0]:.4f},{P9_0[1]:.4f},{P9_0[2]:.4f}) L={L*1000:.2f}mm =====")
    for hits, gap, c, r in results[:5]:
        print(f"  hits={hits}/{len(P7s)} 缺口={gap*1000:6.2f}mm 轴心=({c[0]:+.4f},{c[1]:+.4f},{c[2]:+.4f}) r={r*1000:5.1f}mm")
    full = [x for x in results if x[0] == len(P7s)]
    if full:
        cs = np.array([x[2] for x in full]); rs = np.array([x[3] for x in full])
        print(f"  >>> 全解区域 {len(full)} 格点，轴心中心={np.round(cs.mean(axis=0),4)}，r 中位={np.median(rs)*1000:.1f}mm")
    else:
        print("  >>> 无全解区域")


for tag, bearing, rod in (("A", "Joint_link9", "Joint_link7"), ("B", "Joint_link9_0", "Joint_link7_0")):
    P9_0 = T0[bearing][:3, 3]
    L = np.linalg.norm(P9_0 - T0[rod][:3, 3])
    P7s = []
    for t5, t6 in tests:
        T = solve_T(joints, {"joint_link5": t5, "joint_link6": t6})
        P7s.append(T[rod][:3, 3])
    for axis_dir, name in (([0, 0, 1], "竖直 z"), ([0, 1, 0], "横向 y")):
        scan_axis(f"分支{tag} / {name}", axis_dir, P9_0, P7s, L)
