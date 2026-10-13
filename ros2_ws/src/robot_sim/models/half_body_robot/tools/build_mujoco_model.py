#!/usr/bin/env python3
"""生成 MuJoCo 用的 MJCF（`mujoco/half_body_robot.xml`）。

为什么需要后处理：`mujoco_ros2_control` 自带的 `robot_description_to_mjcf.sh` 只把
URDF 转成"body 树 + 关节 + 惯量 + **visual** geom"，**不带碰撞几何**（visual geom 是
`contype=0 conaffinity=0`），也不排除相邻连杆的接触、不带我们的摩擦系数。所以：

 1. 转换前：把 dist 版 URDF 与网格放进临时目录，`base_link` 换成**降采样网格**
    （MuJoCo 的 STL 解码器上限 20 万面，原网格 50 万面，见 tools/simplify_mesh.py），
    并去掉 XML 声明（转换器的 lxml 不接受带 encoding 声明的字符串）。
 2. 转换：调 `robot_description_to_mjcf.sh --save_only`。
 3. 后处理：
    * `<compiler meshdir>` 指回仓库的 `../meshes`（不复制 47 MB 的 assets）；
      visual mesh 用原网格（base_link 用降采样版）。
    * 按 URDF 的 `<collision>` 逐 link 补 **collision geom**（box / 凸包 mesh，
      `density=0` 不重复计质量、`friction="0.8 0.005 0.0001"`、`group=3`）；
      `Joint_link9/.9` 与 URDF 一致地**不给**碰撞体。
    * 加 18 个 `<contact><exclude>`：MuJoCo 不会自动排除相邻连杆（dartsim 会），
      不排除的话关节处贴合的网格会立刻自锁。
 4. 自检：用 mujoco_ros2_control 的 venv 加载 MJCF，跑几步物理并核对关节/限位/接触数。

用法（在模型目录内）：
  python3 tools/build_mujoco_model.py            # 生成 + 自检
  python3 tools/build_mujoco_model.py --check    # 只自检已生成件
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

MODEL_DIR = Path(__file__).resolve().parent.parent
PACKAGE_DIR = MODEL_DIR.parent.parent                      # .../src/robot_sim
DIST_DIR = PACKAGE_DIR / "dist" / f"{MODEL_DIR.name}_urdf"
URDF = DIST_DIR / f"{MODEL_DIR.name}.urdf"
OUTPUT = MODEL_DIR / "mujoco" / f"{MODEL_DIR.name}.xml"
CONVERTER = Path("/opt/ros/humble/lib/mujoco_ros2_control/robot_description_to_mjcf.sh")
VENV_PYTHON = Path.home() / ".ros/ros2_control/.venv/bin/python"
FRICTION = "0.8 0.005 0.0001"
# URDF 里没有碰撞体的 link（与 Gazebo 侧一致：被 link7 的凸包包住，实心形状会假接触）
NO_COLLISION = ("Joint_link9", "Joint_link9.9")


def convert(workdir: Path) -> Path:
    """把 dist 版 URDF 转成 MJCF，返回生成的 MJCF 路径。"""
    meshes = workdir / "meshes"
    meshes.mkdir(parents=True)
    for mesh in (DIST_DIR / "meshes").glob("*.STL"):
        (meshes / mesh.name).symlink_to(mesh)
    simplified = MODEL_DIR / "meshes" / "base_link_simplified.STL"
    if not simplified.is_file():
        raise RuntimeError("缺少 meshes/base_link_simplified.STL，先跑 tools/simplify_mesh.py")
    (meshes / "base_link.STL").unlink()
    shutil.copy2(simplified, meshes / "base_link.STL")
    # collision 网格保持 collision/ 层级（与 visual 同名，不能平铺到一起）
    (meshes / "collision").mkdir()
    for mesh in (DIST_DIR / "meshes" / "collision").glob("*.STL"):
        (meshes / "collision" / mesh.name).symlink_to(mesh)

    # 去掉 XML 声明：转换器的 lxml.etree.fromstring 不接受带 encoding 声明的字符串
    text = URDF.read_text(encoding="utf-8")
    text = "\n".join(line for line in text.splitlines() if not line.startswith("<?xml"))
    (workdir / "half_body_robot.urdf").write_text(text, encoding="utf-8")

    output = workdir / "out" / "half_body_robot.xml"
    # 转换器按 **当前工作目录** 解析 URDF 里的相对 mesh 路径，必须在工作目录里跑
    result = subprocess.run(
        [str(CONVERTER), "-u", "half_body_robot.urdf", "-o", str(output), "--save_only"],
        capture_output=True, text=True, check=False, cwd=str(workdir),
    )
    mjcf = output / "mujoco_description.xml"
    if result.returncode != 0 or not mjcf.is_file():
        raise RuntimeError(f"转换失败：\n{result.stdout[-2000:]}\n{result.stderr[-2000:]}")
    return mjcf


def collision_geometry() -> dict[str, tuple[str, str, str]]:
    """{link: (类型, 尺寸/网格, origin xyz)}，取自 dist URDF 的 <collision>。"""
    root = ET.parse(URDF).getroot()
    shapes = {}
    for link in root.findall("link"):
        name = link.get("name")
        collision = link.find("collision")
        if collision is None or name in NO_COLLISION:
            continue
        origin = collision.find("origin").get("xyz")
        geometry = collision.find("geometry")
        box = geometry.find("box")
        if box is not None:
            shapes[name] = ("box", box.get("size"), origin)
            continue
        mesh = geometry.find("mesh").get("filename")
        mesh = mesh.replace(f"package://robot_sim/models/{MODEL_DIR.name}/", "")
        shapes[name] = ("mesh", mesh, origin)      # 形如 meshes/collision/<link>.STL
    return shapes


def postprocess(mjcf: Path) -> None:
    tree = ET.parse(mjcf)
    root = tree.getroot()
    shapes = collision_geometry()

    compiler = root.find("compiler")
    compiler.set("meshdir", "../meshes")
    if "texturedir" in compiler.attrib:
        del compiler.attrib["texturedir"]

    asset = root.find("asset")
    # visual mesh：指向仓库网格（base_link 用降采样版）
    for mesh in asset.findall("mesh"):
        name = mesh.get("name")
        filename = ("base_link_simplified.STL" if name == "base_link" else f"{name}.STL")
        mesh.set("file", filename)
    # collision mesh 的新 asset（凸包），file 保持 "collision/<link>.STL"
    for name, (kind, value, _) in shapes.items():
        if kind != "mesh":
            continue
        ET.SubElement(asset, "mesh", {
            "name": f"collision_{name}", "content_type": "model/stl",
            "file": value.replace("meshes/", "", 1),
        })

    def collision_geom(link: str) -> ET.Element | None:
        if link not in shapes:
            return None
        kind, value, origin = shapes[link]
        attributes = {
            "name": f"collision_{link}", "group": "3", "density": "0",
            "contype": "1", "conaffinity": "1", "friction": FRICTION,
            "pos": origin,
        }
        if kind == "box":
            half = [float(v) / 2 for v in value.split()]
            attributes.update({"type": "box",
                               "size": " ".join(f"{v:.6g}" for v in half)})
        else:
            attributes.update({"type": "mesh", "mesh": f"collision_{link}"})
        return ET.Element("geom", attributes)

    worldbody = root.find("worldbody")
    # base_link 的碰撞体（box）放在 worldbody 层级
    geom = collision_geom("base_link")
    if geom is not None:
        worldbody.insert(0, geom)
    bodies = {body.get("name"): body for body in worldbody.iter("body")}
    for name, body in bodies.items():
        geom = collision_geom(name)
        if geom is None:
            continue
        first_geom = body.find("geom")
        body.insert(list(body).index(first_geom) if first_geom is not None else len(body), geom)

    # 相邻连杆排除（joint 的 parent/child），MuJoCo 不会自动排除
    source = ET.parse(URDF).getroot()
    contact = ET.SubElement(root, "contact")
    for joint in source.findall("joint"):
        parent = joint.find("parent").get("link")
        child = joint.find("child").get("link")
        # base_link 的碰撞体在 worldbody 层级 → 对应 MuJoCo 的 world
        if parent == "base_link":
            parent = "world"
        ET.SubElement(contact, "exclude", {"body1": parent, "body2": child})

    # 力矩执行器：每个关节一个 <motor>（gear=1 → ctrl 就是关节力矩）。
    # 没有 actuator 时 mujoco_ros2_control 会把关节注册成 passive，effort 命令无处施加。
    # 力矩上限继承关节的 actuatorfrcrange（= URDF 的 <limit effort>，±10 N·m）。
    actuator = ET.SubElement(root, "actuator")
    for joint in root.find("worldbody").iter("joint"):
        ET.SubElement(actuator, "motor", {
            "name": f"actuator_{joint.get('name')}", "joint": joint.get("name"), "gear": "1",
        })

    ET.indent(tree, space="  ")
    tree.write(mjcf, encoding="utf-8", xml_declaration=False)


def self_check(mjcf: Path) -> int:
    script = f'''
import mujoco, numpy as np
model = mujoco.MjModel.from_xml_path("{mjcf}")
data = mujoco.MjData(model)
collisions = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
              for i in range(model.ngeom) if model.geom_contype[i] != 0]
print(f"nq={{model.nq}} nv={{model.nv}} nbody={{model.nbody}} "
      f"ngeom={{model.ngeom}} nmesh={{model.nmesh}} nexclude={{model.nexclude}}")
print("碰撞 geom:", collisions)
joints = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)]
print("关节数:", len(joints), "首个限位:", model.jnt_range[0])
model.opt.gravity[:] = 0.0
for _ in range(500):
    mujoco.mj_step(model, data)
print("零重力 0.5s 稳定:", bool(np.isfinite(data.qpos).all()))
model.opt.gravity[:] = (0, 0, -9.81)
index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "joint_link1")
actuator = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "actuator_joint_link1")
for _ in range(2000):
    data.ctrl[actuator] = 1.5
    mujoco.mj_step(model, data)
print(f"执行器数={{model.nu}}；1.5 N·m 2s 后 joint_link1 = "
      f"{{data.qpos[model.jnt_qposadr[index]]:+.4f}}"
      f"（限位 {{model.jnt_range[index][0]:+.2f}}..{{model.jnt_range[index][1]:+.2f}}）")
# 力矩上限：给 30 N·m，应被 joint 的 actuatorfrcrange(±10) 收住
for _ in range(50):
    data.ctrl[actuator] = 30.0
    mujoco.mj_step(model, data)
print(f"指令 30 N·m 时的实际执行器力 = {{data.actuator_force[actuator]:+.2f}} N·m"
      f"（actuatorfrcrange 上限 {{model.jnt_actfrcrange[index][1]:+.2f}}）")
'''
    result = subprocess.run([str(VENV_PYTHON), "-c", script],
                            capture_output=True, text=True, check=False)
    print(result.stdout.strip())
    if result.returncode != 0:
        print(result.stderr[-1500:], file=sys.stderr)
        return 1
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="只自检已生成件")
    args = parser.parse_args()

    if not args.check:
        OUTPUT.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix="mjcf_") as tmp:
            mjcf = convert(Path(tmp))
            postprocess(mjcf)
            shutil.copy2(mjcf, OUTPUT)
        print(f"写出 {OUTPUT.relative_to(PACKAGE_DIR)}")
    return self_check(OUTPUT)


if __name__ == "__main__":
    sys.exit(main())
