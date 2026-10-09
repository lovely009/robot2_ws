#!/usr/bin/env python3
"""腕部电机-轴承-杆局部特写渲染：多视角 3D + 正交视图，用于判断输出端/摇臂几何。"""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

PKG = Path("/home/ly/robot_ws/ros2_ws/src/robot_arm_gazebo")
URDF = PKG / "models/zhoubuguanjie_youbi_001/urdf/zhoubuguanjie_youbi_001.urdf"
MESH_DIR = PKG / "models/zhoubuguanjie_youbi_001/meshes"


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


def read_stl_tris(path):
    raw = path.read_bytes()
    n = int(np.frombuffer(raw[80:84], dtype="<u4")[0])
    rec = np.frombuffer(raw[84:], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    return rec["v"].astype(np.float64).reshape(-1, 3)


def main(wanted, out_png, stride=12):
    root = ET.parse(URDF).getroot()
    joints = {}
    for j in root.findall("joint"):
        joints[j.find("child").get("link")] = {
            "parent": j.find("parent").get("link"), "T": origin_T(j)
        }
    links = {l.get("name"): l for l in root.findall("link")}
    children = {}
    for c, j in joints.items():
        children.setdefault(j["parent"], []).append(c)
    roots = [n for n in links if n not in joints]
    pose = {}
    stack = [(roots[0], np.eye(4))]
    while stack:
        link, T = stack.pop()
        pose[link] = T
        for c in children.get(link, []):
            stack.append((c, T @ joints[c]["T"]))

    cols = {"Joint_link4": "0.88", "Joint_link7": "tab:green", "Joint_link9": "tab:orange",
            "Joint_link10": "red", "Joint_link6": "0.45", "Joint_link5": "0.65",
            "Joint_link7_0": "tab:green", "Joint_link9_0": "tab:orange", "Joint_link10_0": "darkred"}
    clouds = {}
    for link in wanted:
        pts = []
        for visual in links[link].findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                continue
            fname = mesh.get("filename").split("/meshes/", 1)[1]
            v = read_stl_tris(MESH_DIR / fname)
            v = v[::stride]
            h = np.hstack([v, np.ones((len(v), 1))])
            pts.append((pose[link] @ origin_T(visual) @ h.T).T[:, :3])
        clouds[link] = np.vstack(pts)

    # 裁剪小臂：只保留腕部附近一段，放大细节
    if "Joint_link4" in clouds:
        p = clouds["Joint_link4"]
        clouds["Joint_link4"] = p[p[:, 0] > 0.33]
    all_pts = np.vstack(list(clouds.values()))
    lo, hi = all_pts.min(axis=0), all_pts.max(axis=0)
    pad = 0.008

    fig = plt.figure(figsize=(22, 15))
    views = [
        (0, 1, "x", "y", "top (xy)", None),
        (0, 2, "x", "z", "side (xz)", None),
        (1, 2, "y", "z", "front (yz)", "left"),
    ]
    for idx, (i, j, la, lb, title, _) in enumerate(views):
        ax = fig.add_subplot(2, 3, idx + 1)
        for link, pts in clouds.items():
            ax.scatter(pts[:, i], pts[:, j], s=0.6, color=cols.get(link, "k"), rasterized=True, label=link)
        ax.set_title(title)
        ax.set_xlabel(la); ax.set_ylabel(lb)
        ax.set_aspect("equal")
        ax.set_xlim(lo[i] - pad, hi[i] + pad)
        ax.set_ylim(lo[j] - pad, hi[j] + pad)
        ax.grid(True, linewidth=0.2)
        if idx == 0:
            for link, label in (("Joint_link10", "X25S A (Joint_link10)"),
                                ("Joint_link10_0", "X25S B (Joint_link10_0)")):
                c = clouds[link].mean(axis=0)
                ax.annotate(label, (c[0], c[1]), color="red", fontsize=9,
                            xytext=(10, 10), textcoords="offset points",
                            arrowprops=dict(arrowstyle="->", color="red", lw=1.0))
    # 3 个 3D 视角
    for k, (elev, azim, title) in enumerate([(25, -60, "3D view 1"), (25, 30, "3D view 2"), (60, -90, "3D view 3")]):
        ax = fig.add_subplot(2, 3, 4 + k, projection="3d")
        for link, pts in clouds.items():
            ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], s=0.4, color=cols.get(link, "k"), rasterized=True)
        ax.view_init(elev=elev, azim=azim)
        ax.set_title(title)
        ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z")
        ax.set_xlim(lo[0] - pad, hi[0] + pad)
        ax.set_ylim(lo[1] - pad, hi[1] + pad)
        ax.set_zlim(lo[2] - pad, hi[2] + pad)
    handles = [plt.Line2D([], [], marker="o", ls="", color=cols.get(l, "k"), label=l) for l in clouds]
    fig.legend(handles=handles, loc="lower right", fontsize=9)
    fig.suptitle("wrist assembly detail (zero pose) -- X25S motors in red")
    fig.tight_layout()
    fig.savefig(out_png, dpi=150, bbox_inches="tight")
    print("saved", out_png)


if __name__ == "__main__":
    out = Path(sys.argv[1])
    stride = int(sys.argv[2]) if len(sys.argv) > 2 else 12
    wanted = sys.argv[3].split(",") if len(sys.argv) > 3 else ["Joint_link4", "Joint_link6", "Joint_link7", "Joint_link9", "Joint_link10"]
    main(wanted, out, stride)
