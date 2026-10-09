#!/usr/bin/env python3
"""从渲染后的 xacro 重生成 standalone 与 dist 派生件（一次性工具）。"""

import copy
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

PKG = Path("/home/ly/robot_ws/ros2_ws/src/robot_arm_gazebo")
MODEL = PKG / "models/zhoubuguanjie_youbi_001"
NAME = "zhoubuguanjie_youbi_001"
RENDERED = Path("/tmp/render.urdf")

STANDALONE_HEADER = f"""<!-- {NAME}（右臂：R58 肩座 + 6 关节主链 + 腕部远置驱动连杆机构）纯模型 URDF，可直接加载。
     由 urdf/{NAME}.urdf.xacro 渲染后剥掉 <ros2_control> 与 <gazebo> 块得到
     （渲染时 fixed_base:=false，因此不含 world 固定连杆——那是 Gazebo 侧的仿真脚手架）。
     网格走 package://robot_arm_gazebo/models/{NAME}/meshes/，加载前须 source 本工作空间；
     要脱离 ROS 使用，改用 ../../dist/{NAME}_urdf/ 那份相对路径版本。
     ⚠ 关节限位/力矩/速度是占位值，接真机前必须按实际机械限位与执行器参数替换。
     ⚠ 腕部（joint_link5 之后）为远置驱动连杆机构，物理闭环在 URDF 树中已被切开，见 README。 -->"""

DIST_HEADER = f"""<!-- {NAME}（右臂：R58 肩座 + 6 关节主链 + 腕部远置驱动连杆机构）纯模型 URDF，可直接加载。
     几何/惯性参数由 SolidWorks 模型导出，未做真机标定；
     关节限位/力矩/速度是占位值，接真机前必须按实际机械限位与执行器参数修改；
     网格走同目录 meshes/ 相对路径，整个文件夹可任意搬移。 -->"""


def strip_scaffolding(root: ET.Element) -> None:
    root.attrib.pop("xmlns:xacro", None)
    for tag in ("ros2_control", "gazebo"):
        for element in root.findall(tag):
            root.remove(element)


def dump(root: ET.Element, header: str, out: Path) -> None:
    ET.indent(root, space="  ")
    body = ET.tostring(root, encoding="unicode")
    out.write_text(
        '<?xml version="1.0" encoding="utf-8"?>\n' + header + "\n" + body + "\n",
        encoding="utf-8",
    )
    print(f"wrote {out}")


def main() -> None:
    root = ET.parse(RENDERED).getroot()
    strip_scaffolding(root)
    assert root.find("link[@name='world']") is None, "渲染件不应包含 world 连杆"
    assert root.find("ros2_control") is None and root.find("gazebo") is None

    dump(root, STANDALONE_HEADER, MODEL / "urdf" / f"{NAME}.standalone.urdf")

    dist_root = copy.deepcopy(root)
    for mesh in dist_root.iter("mesh"):
        filename = mesh.get("filename")
        mesh.set("filename", "meshes/" + filename.split("/meshes/", 1)[1])
    dist_dir = PKG / "dist" / f"{NAME}_urdf"
    dump(dist_root, DIST_HEADER, dist_dir / f"{NAME}.urdf")

    dist_meshes = dist_dir / "meshes"
    if dist_meshes.is_dir():
        for stale in dist_meshes.iterdir():
            stale.unlink()
    dist_meshes.mkdir(parents=True, exist_ok=True)
    for src in sorted((MODEL / "meshes").iterdir()):
        shutil.copyfile(src, dist_meshes / src.name)
    print(f"copied {len(list(dist_meshes.iterdir()))} meshes to {dist_meshes}")


if __name__ == "__main__":
    main()
