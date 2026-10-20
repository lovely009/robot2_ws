#!/usr/bin/env python3
"""重生成派生件：`urdf/<名>.standalone.urdf` 与 `dist/<名>_urdf/`。

为什么要有这个脚本：standalone 与 dist 都与主 URDF 同源，但它们是**独立文件**。
手工改一处忘了重生成另一处，两份模型就会悄悄分叉——`test_model_consistency.py` 的
`test_standalone_matches_urdf` / `test_dist_artifact_matches_model` 正是为拦这个而写的，
但它们只报错、不修。改完主 URDF 后跑本脚本即可。

派生规则（与 test 的断言一一对应）：
  1. 渲染 `<名>.urdf.xacro`，**fixed_base:=false**（不带 world 固定连杆——那是 Gazebo 侧
     的仿真脚手架，MuJoCo 后端用这份 URDF 渲染 robot_description，带 world 会多一个浮空基座）；
  2. 剥掉根级 `<ros2_control>` 与 `<gazebo>` 两块，并去掉 `xmlns:xacro`；
  3. standalone 保留 `package://robot_sim/models/<名>/meshes/` 路径（须 source 工作空间）；
     dist 改成相对路径 `meshes/...`，因此整个目录可脱离 ROS 直接加载；
  4. dist 的网格用**硬链接**指向模型 `meshes/`（不占额外磁盘；改一处两边同时变，别单独改）。

输出格式（`ET.indent` + `<x/>`）与历史派生件一致，重跑本脚本不会产生无意义的格式 diff。

用法（在模型目录内，需 source /opt/ros/humble/setup.bash 以便找到 xacro）：
  python3 tools/regen_derived.py
  python3 tools/regen_derived.py --check    # 只校验已生成件与主 URDF 同源，不写文件
"""

from __future__ import annotations

import argparse
import copy
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

MODEL_DIR = Path(__file__).resolve().parent.parent
PACKAGE_DIR = MODEL_DIR.parent.parent                      # .../src/robot_sim
NAME = MODEL_DIR.name
XACRO = MODEL_DIR / "urdf" / f"{NAME}.urdf.xacro"
URDF = MODEL_DIR / "urdf" / f"{NAME}.urdf"
STANDALONE = MODEL_DIR / "urdf" / f"{NAME}.standalone.urdf"
DIST_DIR = PACKAGE_DIR / "dist" / f"{NAME}_urdf"
DIST_URDF = DIST_DIR / f"{NAME}.urdf"
MESHES = MODEL_DIR / "meshes"
DIST_MESHES = DIST_DIR / "meshes"
MESH_PREFIX = f"package://robot_sim/models/{NAME}/meshes/"

STANDALONE_HEADER = f"""<!-- {NAME}（单腿：骨盆 base_link → 髋 → 膝 → 踝，6 个主动关节，纯串联无闭环）纯模型 URDF，可直接加载。
     由 urdf/{NAME}.urdf.xacro 以 fixed_base:=false 渲染后剥掉 <ros2_control> 与
     <gazebo> 两块得到，因此不含 world 固定连杆（那是 Gazebo 侧的仿真脚手架）。
     网格走 {MESH_PREFIX}，加载前须 source 本工作空间；
     要脱离 ROS 使用，改用 ../../dist/{NAME}_urdf/ 那份相对路径版本。
     ⚠ 本文件由 tools/regen_derived.py 生成，改主 URDF 后重跑，勿手改。
     ⚠ link 名是 Joint_linkN（大写 L，导出器原样），joint 名是 joint_linkN（小写）——不可同名（SDFormat 里 link 与 joint 共用命名空间）。
     ⚠ 关节限位是**占位值**（导出件 <limit> 全是 0/0/0/0，CAD 未配角度配合），接真机前必须按机械限位替换；力矩/速度取自型号库 motor.yaml。 -->"""

DIST_HEADER = f"""<!-- {NAME}（单腿：骨盆 base_link → 髋 → 膝 → 踝，6 个主动关节，纯串联无闭环）的 ROS 无关发布件，可脱离 ROS 直接加载。
     由 tools/regen_derived.py 从模型主 URDF 派生，勿手改：
     1. 网格路径由 {MESH_PREFIX} 改为相对路径 meshes/...，整个目录可任意搬移；
     2. 不含 <ros2_control> 与 <gazebo> 仿真块，也不含 world 固定连杆。
     几何/惯性取自 SolidWorks 导出（未做真机标定）。关节限位是**占位值**（导出件 <limit> 全是 0/0/0/0，CAD 未配角度配合），接真机前必须按机械限位替换；力矩/速度取自型号库 motor.yaml。 -->"""


def render() -> ET.Element:
    """渲染薄壳并剥掉仿真脚手架，返回纯模型树。"""
    result = subprocess.run(
        ["xacro", str(XACRO), "fixed_base:=false"],
        capture_output=True, text=True, check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(f"xacro 渲染失败（先 source /opt/ros/humble/setup.bash）：\n{result.stderr}")
    root = ET.fromstring(result.stdout)
    root.attrib.pop("xmlns:xacro", None)
    for tag in ("ros2_control", "gazebo"):
        for element in root.findall(tag):
            root.remove(element)
    if root.find("link[@name='world']") is not None:
        raise RuntimeError("渲染件含 world 连杆：fixed_base 未生效")
    return root


def serialize(root: ET.Element, header: str, out: Path) -> None:
    ET.indent(root, space="  ")
    body = re.sub(r" />", "/>", ET.tostring(root, encoding="unicode"))
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text('<?xml version="1.0" encoding="utf-8"?>\n' + header + "\n" + body + "\n",
                   encoding="utf-8")
    print(f"wrote {out.relative_to(PACKAGE_DIR)}")


def link_mesh(src: Path, dst: Path) -> None:
    """硬链接（已存在同 inode 则跳过）；跨设备时退回拷贝。"""
    dst.parent.mkdir(parents=True, exist_ok=True)
    if dst.exists() and os.path.samefile(src, dst):
        return
    dst.unlink(missing_ok=True)
    try:
        os.link(src, dst)
    except OSError:
        import shutil
        shutil.copy2(src, dst)


def visual_meshes() -> list[Path]:
    """发布件里的网格 = 全分辨率可视网格（`*_simplified.STL` 是 MuJoCo 后端的解码上限
    变通件，随 `mujoco/` 一起走，不进对外发布件）。"""
    return sorted(p for p in MESHES.glob("*.STL") if not p.name.endswith("_simplified.STL"))


def sync_dist_meshes() -> None:
    wanted = {p.name for p in visual_meshes()}
    for stale in DIST_MESHES.glob("*.STL"):
        if stale.name not in wanted:
            stale.unlink()
    for src in visual_meshes():
        link_mesh(src, DIST_MESHES / src.name)
    collision = MESHES / "collision"
    if collision.is_dir():
        wanted_c = {p.name for p in collision.glob("*.STL")}
        for stale in (DIST_MESHES / "collision").glob("*.STL"):
            if stale.name not in wanted_c:
                stale.unlink()
        for src in sorted(collision.glob("*.STL")):
            link_mesh(src, DIST_MESHES / "collision" / src.name)
    print(f"linked {len(list(DIST_MESHES.glob('*.STL')))} visual + "
          f"{len(list((DIST_MESHES / 'collision').glob('*.STL')))} collision meshes into "
          f"{DIST_MESHES.relative_to(PACKAGE_DIR)}")


def generate() -> None:
    root = render()
    serialize(root, STANDALONE_HEADER, STANDALONE)

    dist_root = copy.deepcopy(root)
    for mesh in dist_root.iter("mesh"):
        filename = mesh.get("filename")
        if not filename.startswith(MESH_PREFIX):
            raise RuntimeError(f"网格路径不在模型目录内，无法转相对路径：{filename}")
        mesh.set("filename", "meshes/" + filename[len(MESH_PREFIX):])
    serialize(dist_root, DIST_HEADER, DIST_URDF)

    sync_dist_meshes()


def check() -> int:
    """只校验：standalone/dist 的关节定义与质量必须与主 URDF 同源。"""
    def shape(root: ET.Element):
        return [
            (j.get("name"), j.get("type"),
             tuple(sorted((j.find("limit").attrib if j.find("limit") is not None else {}).items())))
            for j in root.findall("joint")
        ]

    def masses(root: ET.Element):
        return {l.get("name"): l.find("inertial/mass").get("value") for l in root.findall("link")}

    main = ET.parse(URDF).getroot()
    problems = []
    for path, label in ((STANDALONE, "standalone"), (DIST_URDF, "dist")):
        if not path.is_file():
            problems.append(f"{label} 缺失：{path}")
            continue
        root = ET.parse(path).getroot()
        if shape(root) != shape(main):
            problems.append(f"{label} 的关节定义与主 URDF 不一致")
        if label == "dist" and masses(root) != masses(main):
            problems.append("dist 的质量参数与主 URDF 不一致")
    if problems:
        for p in problems:
            print(f"✗ {p}", file=sys.stderr)
        print("  → 跑 python3 tools/regen_derived.py 重生成", file=sys.stderr)
        return 1
    print("✓ standalone 与 dist 都与主 URDF 同源")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="只校验，不写文件")
    args = parser.parse_args()
    if args.check:
        return check()
    generate()
    return check()


if __name__ == "__main__":
    raise SystemExit(main())
