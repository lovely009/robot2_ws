#!/usr/bin/env python3
"""闭环求解原型：由腕部姿态（joint_link5/6 等主链角）解出被动关节 joint_link7/9 的值，
使连杆（Joint_link7/7_0）落在"电机固定于小臂"的真实约束里。

约束：杆末端轴承（Joint_link9/9_0，连同电机 Joint_link10/10_0）的铰点必须落在
"以电机轴为轴、半径 r 的圆柱面"上（电机固定于 Joint_link4 ⇒ 摇臂端只能绕电机轴转）。

对每个分支：
- 拟合电机圆柱轴 (a, C)：方向用面法线零空间，轴心用顶点在垂轴平面的质心；
- r = 零位下 P9 到轴 (a, C) 的垂距；
- 位形 (θ5, θ6) 下：P9(j7) = T7(j7) 的平移，解 dist(P9(j7), axis) = r 的根（取近零位支）；
- 评估 (1) 杆长守恒，(2) 9/10 的目标姿态残差（j9 取最优绕轴补偿后的最小残差）。
"""

import xml.etree.ElementTree as ET
from pathlib import Path

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


def axis_T(axis, q):
    """绕单位轴 axis 转 q 的旋转矩阵（URDF: T_child = T_parent @ T_origin @ R(axis,q)）"""
    axis = axis / np.linalg.norm(axis)
    K = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0],
    ])
    T = np.eye(4)
    T[:3, :3] = np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * (K @ K)
    return T


def read_stl(path):
    raw = path.read_bytes()
    n = int(np.frombuffer(raw[80:84], dtype="<u4")[0])
    rec = np.frombuffer(raw[84:], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    tris = rec["v"].astype(np.float64)
    return tris.reshape(-1, 3)[::20]  # 采样：每 20 个顶点取 1，足够拟合圆柱轴


def load_urdf():
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
    links = {l.get("name"): l for l in root.findall("link")}
    return joints, links


def solve_T(joints, q):
    """给定各关节角，返回 {link: T_world}（关节角默认 0）"""
    children = {}
    for name, j in joints.items():
        children.setdefault(j["parent"], []).append(name)
    roots = [l for l in {j["parent"] for j in joints.values()} | {j["child"] for j in joints.values()}
             if l not in {j["child"] for j in joints.values()}]
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
    return T, children


def world_mesh(links, T, link):
    pts = []
    for visual in links[link].findall("visual"):
        mesh = visual.find("geometry/mesh")
        if mesh is None:
            continue
        fname = mesh.get("filename").split("/meshes/", 1)[1]
        v = read_stl(MESH_DIR / fname)
        h = np.hstack([v, np.ones((len(v), 1))])
        pts.append((T[link] @ origin_T(visual) @ h.T).T[:, :3])
    return np.vstack(pts)


def fit_axis(motor_pts):
    """返回 (a 单位向量, C 轴心)：轴向使"顶点到轴距离的方差"最小；
    轴心 = 垂轴平面内投影的圆心（Kasa 拟合）。"""
    allzero = {}
    c = motor_pts.mean(axis=0)
    u, s, vt = np.linalg.svd(motor_pts - c, full_matrices=False)

    def score(cand):
        d = motor_pts @ cand
        perp = motor_pts - np.outer(d, cand)
        r = np.linalg.norm(perp - perp.mean(axis=0), axis=1)
        return r.std()

    best = min((score(v) for v in vt))
    # 以 PCA 最优方向为起点做球面局部细化
    cand0 = vt[int(np.argmin([score(v) for v in vt]))]
    step = np.radians(2.0)
    for _ in range(40):
        improved = False
        for axis_try in (0, 1):
            for sgn in (+1, -1):
                # 构造垂直于 cand0 的一个方向
                ref = np.eye(3)[np.argmin(np.abs(cand0))]
                p1 = np.cross(cand0, ref); p1 /= np.linalg.norm(p1)
                p2 = np.cross(cand0, p1)
                rot_axis = p1 if axis_try == 0 else p2
                K = np.array([
                    [0, -rot_axis[2], rot_axis[1]],
                    [rot_axis[2], 0, -rot_axis[0]],
                    [-rot_axis[1], rot_axis[0], 0],
                ])
                cand = (np.eye(3) + np.sin(sgn * step) * K + (1 - np.cos(sgn * step)) * (K @ K)) @ cand0
                if score(cand) < best:
                    best = score(cand)
                    cand0 = cand
                    improved = True
        if not improved:
            step *= 0.5
        if step < np.radians(0.01):
            break
    a = cand0 / np.linalg.norm(cand0)

    # 轴心 C：把所有点投到 ⊥a 平面，对 (x, y, 1) 解圆方程 x²+y² + D x + E y + F = 0（Kasa）
    ref = np.eye(3)[np.argmin(np.abs(a))]
    e1 = np.cross(a, ref); e1 /= np.linalg.norm(e1)
    e2 = np.cross(a, e1)
    p2 = np.stack([motor_pts @ e1, motor_pts @ e2], axis=1)
    A = np.hstack([p2, np.ones((len(p2), 1))])
    b = -(p2 ** 2).sum(axis=1)
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    cx = -sol[0] / 2
    cy = -sol[1] / 2
    C = cx * e1 + cy * e2
    return a, C


def dist_to_axis(p, a, C):
    d = p - C
    return np.linalg.norm(d - (d @ a) * a)


def main():
    joints, links = load_urdf()
    q0 = {}
    T0, _ = solve_T(joints, q0)

    # 电机与分支参数
    branches = {
        "A": dict(motor="Joint_link10", rod="joint_link7", rod2="joint_link9", bearing="Joint_link9", rod_link="Joint_link7"),
        "B": dict(motor="Joint_link10_0", rod="joint_link7_0", rod2="joint_link9_0", bearing="Joint_link9_0", rod_link="Joint_link7_0"),
    }
    params = {}
    for tag, br in branches.items():
        pts = world_mesh(links, T0, br["motor"])
        Cm = (pts.min(axis=0) + pts.max(axis=0)) / 2
        a = np.array([0.0, 0.0, 1.0])   # 摇臂轴：过电机中心、竖直（装配位形下）
        C = Cm
        P9 = T0[br["bearing"]][:3, 3]
        r = dist_to_axis(P9, a, C)
        P7 = T0[br["rod_link"]][:3, 3]
        L = np.linalg.norm(P9 - P7)
        params[tag] = (a, C, r, P9, P7, L)
        print(f"[{tag}] arm axis a=({a[0]:+.3f},{a[1]:+.3f},{a[2]:+.3f}) Cm=({Cm[0]:+.4f},{Cm[1]:+.4f},{Cm[2]:+.4f})")
        print(f"     r(P9->axis)={r*1000:.2f} mm   rod length L={L*1000:.2f} mm   g(0)={dist_to_axis(P9, a, C)-r:+.4f} m")

    def solve_branch(tag, q):
        """球铰模型：杆端 P9 只需落在"以 P7 为球心、半径 L 的球面"上
        （关节轴承允许杆与铰轴间夹角变化）；摇臂端仍是绕电机轴的圆。
        同时给出"若用 revolute 近似杆-腕铰（j7）"的 FK 位置误差与离面角。"""
        a, C, r, P9_0, P7_0, L = params[tag]
        br = branches[tag]
        zero = {k: 0.0 for k, j in joints.items() if j["type"] == "revolute"}
        T, _ = solve_T(joints, {**zero, **q})
        P7 = T[br["rod_link"]][:3, 3]
        R6 = T["Joint_link6"][:3, :3]

        # 摇臂圆参数化（相位基准 = 零位 P9_0）
        O = C + ((P9_0 - C) @ a) * a
        u = (P9_0 - O) / r
        w = np.cross(a, u)

        def P9_of(phi):
            return O + r * (u * np.cos(phi) + w * np.sin(phi))

        def g(phi):
            return np.linalg.norm(P9_of(phi) - P7) - L

        xs = np.linspace(-np.pi, np.pi, 721)
        gs = np.array([g(x) for x in xs])
        roots = []
        for i in range(len(xs) - 1):
            if abs(gs[i]) < 1e-12:
                roots.append(xs[i])
            elif gs[i] * gs[i + 1] < 0:
                lo, hi = xs[i], xs[i + 1]
                for _ in range(60):
                    mid = (lo + hi) / 2
                    if g(lo) * g(mid) <= 0:
                        hi = mid
                    else:
                        lo = mid
                roots.append((lo + hi) / 2)
        if not roots:
            gmin = float(np.min(np.abs(gs)))
            return ("NONE", gmin)
        phi = min(roots, key=abs)
        P9 = P9_of(phi)

        # j7：把"P7→P9"投影到 6 系的 ⊥z 平面内取角（相对零位方向）
        d = P9 - P7
        d6 = R6.T @ d
        d0 = P9_0 - P7_0
        d06 = R6.T @ d0

        def ang(v):
            return np.arctan2(v[1], v[0])

        j7 = ang(d6) - ang(d06)
        # 杆偏离 ⊥(j7 轴) 平面的角度（球铰实际用掉的自由度）
        out_of_plane = np.degrees(np.arcsin(np.clip(abs(d6[2]) / np.linalg.norm(d6), -1, 1)))

        # revolute 近似下 P9 的 FK 位置误差
        Tw, _ = solve_T(joints, {**zero, **q, br["rod"]: j7})
        P9_fk = Tw[br["bearing"]][:3, 3]
        fk_err = np.linalg.norm(P9_fk - P9)

        # j9：目标姿态 = 9 的零位姿态（电机固定于小臂）
        R_target = T0[br["bearing"]][:3, :3]
        best = (1e9, 0.0)
        for j9 in np.linspace(-np.pi, np.pi, 361):
            Tj, _ = solve_T(joints, {**zero, **q, br["rod"]: j7, br["rod2"]: j9})
            R_cur = Tj[br["bearing"]][:3, :3]
            ang_err = np.degrees(np.arccos(np.clip((np.trace(R_target @ R_cur.T) - 1) / 2, -1, 1)))
            if ang_err < best[0]:
                best = (ang_err, j9)
        att_err, j9 = best
        return phi, j7, j9, fk_err, out_of_plane, att_err, len(roots)

    print("\n=== 位形扫描（θ5=joint_link5, θ6=joint_link6）===")
    for t5, t6 in [(0, 0), (0.2, 0), (0, 0.2), (0.4, 0.2), (-0.3, 0.3), (0.6, -0.3), (0.8, 0.5)]:
        q = {"joint_link5": t5, "joint_link6": t6}
        outs = []
        for tag in "AB":
            res = solve_branch(tag, q)
            if isinstance(res, tuple) and res[0] == "NONE":
                outs.append(f"{tag}: 无解(缺口 {res[1]*1000:.2f}mm)")
            else:
                phi, j7, j9, fk_err, oop, att_err, nroots = res
                outs.append(
                    f"{tag}: j7={j7:+.4f} j9={j9:+.4f} 摆臂φ={phi:+.4f} | "
                    f"FK误差={fk_err*1000:5.2f}mm 离面角={oop:4.1f}° 姿态残差={att_err:5.2f}° roots={nroots}"
                )
        print(f"θ5={t5:+.2f} θ6={t6:+.2f} | " + " | ".join(outs))


if __name__ == "__main__":
    main()
