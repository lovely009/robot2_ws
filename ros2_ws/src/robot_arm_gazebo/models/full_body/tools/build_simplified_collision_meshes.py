#!/usr/bin/env python3
"""为 full_body 生成简化碰撞网格（凸包），压掉 gazebo 侧接触检测开销。

原模型的 collision 直接复用可视化网格（全模型约 146 万三角面，base_link 一个
就 50 万面）。本脚本对**形状接近凸体**的 link 取顶点凸包，写出到 meshes/collision/，
供 URDF 的 <collision> 引用；visual 一律仍用原始网格。

两类 link 不用凸包（它们外形有让位缺口，实心简化形状会在零位就与邻近连杆假接触，
自碰撞一开就把关节顶死；2026-10-12 实测：base_link 的整高 box 与 Joint_link2 相交
15.8 mm，Joint_link7 的凸包与 Joint_link9 相交 12.3 mm，joint_link2 加 2 N·m 完全不动）：

  * PRIMITIVE_LINKS：base_link（URDF 里用**每边各内缩 10 mm** 的 box，避开躯干顶部两肩的
    两肩的让位区；整高 0.7029 会与肩部连杆假接触）与 Joint_link8/.8（近似方块）；
  * NO_COLLISION_LINKS：Joint_link9/.9 小法兰**不提供 collision**——它们被 Joint_link7
    的凸包整个包住，任何实心简化形状都会与 link7 假接触（实测缩到 bbox×0.8 仍穿透
    9.3 mm），而用原始网格做 trimesh 碰撞会让 dartsim 的 ODE 检测刷
    "hash table bucket overflow" 并拖垮控制器切换。

其余 14 个 link 用凸包（每个 390–554 面，顶点朝质心内缩 3 mm 留接触余量）。改这几个集合后必须重跑本脚本，并重新派生
URDF 的 collision、复跑仿真确认零位无假接触。

用法（在模型目录内）：
  python3 tools/build_simplified_collision_meshes.py            # 生成
  python3 tools/build_simplified_collision_meshes.py --check    # 只校验，不写文件

依赖 numpy 与 scipy（scipy.spatial.ConvexHull）。生成是确定性的：同样的输入 STL
得到同样的输出。
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull

MODEL_DIR = Path(__file__).resolve().parent.parent
MESH_DIR = MODEL_DIR / "meshes"
OUT_DIR = MESH_DIR / "collision"

# 凸包顶点朝质心统一内缩。凸包会把原网格的凹处填满（比实物「胖」几毫米），实测手臂下垂时
# 凸包与躯干 box 恰好贴住（0.00 / 1.24 mm），接触刚度下就把关节顶死了；内缩留余量。
SHRINK_M = 0.003

# URDF 里用 <box> 基元：base_link 的高度会截断到 0.68 m（脚本打印的是原始包围盒）。
PRIMITIVE_LINKS = ("base_link", "Joint_link8", "Joint_link8.8")

# URDF 里不提供 <collision>（小法兰，被 link7 的凸包包住，见文件头说明）。
NO_COLLISION_LINKS = ("Joint_link9", "Joint_link9.9")


def read_binary_stl(path: Path) -> np.ndarray:
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    usable = min(len(data) - 84, count * 50) // 50
    if usable != count:
        raise RuntimeError(f"{path.name}: 头部声明 {count} 面，文件只有 {usable} 面")
    records = np.frombuffer(data, dtype=np.uint8, count=usable * 50, offset=84)
    floats = records.reshape(-1, 50)[:, :48].copy().view("<f4").reshape(-1, 12)
    return floats[:, 3:12].reshape(-1, 3, 3).astype(np.float64)


def write_binary_stl(path: Path, triangles: np.ndarray) -> None:
    normals = np.cross(triangles[:, 1] - triangles[:, 0],
                       triangles[:, 2] - triangles[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    lengths[lengths == 0.0] = 1.0
    normals = normals / lengths[:, None]
    record = np.zeros((len(triangles), 50), dtype=np.uint8)
    record[:, :48] = np.concatenate(
        [normals, triangles.reshape(-1, 9)], axis=1).astype("<f4").view(np.uint8).reshape(-1, 48)
    header = b"full_body convex collision mesh".ljust(80, b"\0")
    payload = header + struct.pack("<I", len(triangles)) + record.tobytes()
    path.write_bytes(payload)


def convex_hull_mesh(triangles: np.ndarray) -> np.ndarray:
    points = np.unique(triangles.reshape(-1, 3), axis=0)
    try:
        hull = ConvexHull(points)
    except Exception:  # 退化点集（共面/重复点）时抖动重试
        hull = ConvexHull(points, qhull_options="QJ")
    faces = points[hull.simplices]
    if SHRINK_M <= 0.0:
        return faces
    # 每个顶点朝凸包顶点均值移动 SHRINK_M（保持面连接，内缩后仍为凸体）
    center = points.mean(axis=0)
    vertices = faces.reshape(-1, 3)
    direction = vertices - center
    norms = np.linalg.norm(direction, axis=1, keepdims=True)
    norms[norms < 1e-9] = 1.0
    return (vertices - SHRINK_M * direction / norms).reshape(-1, 3, 3)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true",
                        help="只校验已生成的凸包，不写文件")
    args = parser.parse_args()

    OUT_DIR.mkdir(exist_ok=True)
    # 跳过降采样件（MuJoCo 用，见 tools/simplify_mesh.py），它不是某个 link 的网格
    sources = sorted(p for p in MESH_DIR.glob("*.STL")
                     if not p.stem.endswith("_simplified"))
    if not sources:
        print(f"没有找到 {MESH_DIR}/*.STL", file=sys.stderr)
        return 1

    failures = 0
    for source in sources:
        link = source.stem
        triangles = read_binary_stl(source)
        bbox = triangles.reshape(-1, 3).max(axis=0) - triangles.reshape(-1, 3).min(axis=0)
        if link in NO_COLLISION_LINKS:
            print(f"{link:16s} 原网格 {len(triangles):>7d} 面；URDF 不提供 collision")
            continue
        if link in PRIMITIVE_LINKS:
            note = "（URDF 里每边内缩 10 mm）" if link == "base_link" else ""
            print(f"{link:16s} 原网格 {len(triangles):>7d} 面；URDF 用 box，"
                  f"size=\"{' '.join(f'{v:.4f}' for v in bbox)}\"{note}（min 角 "
                  f"{' '.join(f'{v:.4f}' for v in triangles.reshape(-1,3).min(axis=0))}）")
            continue

        simplified = convex_hull_mesh(triangles)
        target = OUT_DIR / f"{link}.STL"
        if args.check:
            if not target.is_file():
                print(f"{link:16s} 缺少凸包 {target.name}", file=sys.stderr)
                failures += 1
                continue
            existing = read_binary_stl(target)
            same = (len(existing) == len(simplified)
                    and np.allclose(existing, simplified, atol=1e-9))
            print(f"{link:16s} 凸包 {len(simplified):>5d} 面  "
                  f"{'与已生成件一致' if same else '与已生成件不一致'}")
            failures += 0 if same else 1
        else:
            write_binary_stl(target, simplified)
            print(f"{link:16s} 原网格 {len(triangles):>7d} 面 → 凸包 "
                  f"{len(simplified):>5d} 面（{target.relative_to(MODEL_DIR)}）")

    expected = {f"{source.stem}.STL" for source in sources
                if source.stem not in PRIMITIVE_LINKS + NO_COLLISION_LINKS}
    stale = sorted(path.name for path in OUT_DIR.glob("*.STL")
                   if path.name not in expected)
    if stale:
        print(f"{OUT_DIR.name}/ 里有多余凸包（对应 link 已改为 box/无碰撞），请删除："
              f"{', '.join(stale)}", file=sys.stderr)
        failures += len(stale)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
