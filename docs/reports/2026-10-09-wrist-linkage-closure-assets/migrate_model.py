#!/usr/bin/env python3
"""把 SolidWorks 新导出件迁移为 robot_arm_gazebo 模型库格式（一次性迁移工具）。

改写内容（与模型库既有惯例一致）：
1. robot name -> zhoubuguanjie_youbi_001；
2. mesh 路径 -> package://robot_arm_gazebo/models/zhoubuguanjie_youbi_001/meshes/，
   网格文件名去点号（Joint_Link7.0.STL -> Joint_link7_0.STL）；
3. 连杆名 Joint_LinkN(.0) -> Joint_linkN(_0)（保留 CAD 名风格、去点号）；
4. 关节名 Joint_LinkN(.0) -> joint_linkN(_0)（ROS 命名规范，小写）；
5. revolute 关节的 0/0/0/0 占位限位（导出件会导致关节锁死）；
6. <material name=""> -> <连杆名>_material（导出件共用空名会被解析器去重）；
7. fixed 关节的 <axis xyz="0 0 0"> 删除（URDF 对 fixed 无意义）。
"""

import re
import shutil
import sys
from pathlib import Path

OLD_PKG = "zhoubuguanjie ROBOT-0"
NEW_NAME = "zhoubuguanjie_youbi_001"
NEW_MESH_PREFIX = f"package://robot_arm_gazebo/models/{NEW_NAME}/meshes/"

FREE_LIMIT = """<limit
      lower="-1.5708"
      upper="1.5708"
      effort="10.0"
      velocity="1.0" />"""

HEADER = f"""<!-- {NEW_NAME}（右臂：R58 肩座 + 6 个主链 revolute 关节 + 腕部远置驱动连杆机构）。
     原始文件由 SolidWorks SW2URDF 导出器生成（Commit Version: 1.6.0-4-g7f85cfe
     Build Version: 1.6.7995.38578，2026-10-09），移植进 robot_arm_gazebo 时做了改写：
       1. mesh 路径 -> package://robot_arm_gazebo/models/{NEW_NAME}/meshes/，
          网格文件名去点号；
       2. 连杆名 Joint_LinkN(.0) -> Joint_linkN(_0)，关节名 -> 小写 joint_linkN(_0)
          （"." 不是 ROS 名字字符，进话题/参数前必须去掉）；
       3. 导出的 limit 全为 0（会锁死关节），填入占位值；
       4. 材质名从空串改为 <连杆名>_material（空名会被解析器按名字去重导致颜色错乱）；
       5. fixed 关节的 <axis xyz="0 0 0"> 删除。
     ⚠ 几何/惯性来自 CAD 未做真机标定；limit 是占位值，接真机前必须按实际机械限位与
     执行器参数替换。⚠ 腕部（joint_link5 之后）是远置驱动连杆机构，物理上两台 X25S
     电机固定在小臂（Joint_link4）上形成闭环，URDF 树只能保留 SW2URDF 切开的一支
     （详见 README.md 的「腕部机构与闭环说明」）。原始导出件与导出日志见 ../cad/。 -->"""


def rename_ident(match: "re.Match[str]", joint: bool) -> str:
    tag, raw = match.group(1), match.group(2)
    cooked = raw.replace(".", "_")
    prefix = "joint_link" if joint else "Joint_link"
    return f"{tag}{prefix}{cooked}\""


def main(src_urdf: Path, src_meshes: Path, model_dir: Path) -> None:
    text = src_urdf.read_text(encoding="utf-8")

    # 1. robot name（注释块一并替换）
    text = re.sub(
        r"^<\?xml[^\n]*\?>\s*<!--.*?-->\s*",
        '<?xml version="1.0" encoding="utf-8"?>\n' + HEADER + "\n",
        text,
        count=1,
        flags=re.S,
    )
    text = re.sub(r'name="' + re.escape(OLD_PKG) + r'"', f'name="{NEW_NAME}"', text)

    # 2. mesh 路径与文件名
    def mesh_repl(match: "re.Match[str]") -> str:
        raw = match.group(1)
        cooked = raw.replace("Joint_Link", "Joint_link").replace(".", "_")
        return f'filename="{NEW_MESH_PREFIX}{cooked}.STL"'

    text = re.sub(
        r'filename="package://' + re.escape(OLD_PKG) + r'/meshes/([^"]+)\.STL"',
        mesh_repl,
        text,
    )

    # 3/4. 连杆与关节名
    text = re.sub(
        r'(<link\s+name=")Joint_Link(\d+(?:\.\d+)?)(")',
        lambda m: rename_ident(m, joint=False),
        text,
    )
    text = re.sub(
        r'(<joint\s+name=")Joint_Link(\d+(?:\.\d+)?)(")',
        lambda m: rename_ident(m, joint=True),
        text,
    )
    text = re.sub(
        r'(link=")Joint_Link(\d+(?:\.\d+)?)(")',
        lambda m: rename_ident(m, joint=False),
        text,
    )

    # 5. 占位限位
    text, count_limits = re.subn(
        r'<limit\s+lower="0"\s+upper="0"\s+effort="0"\s+velocity="0"\s*/>',
        FREE_LIMIT,
        text,
    )

    # 6. 材质名（每个 link 块内 name="" 替换为该连杆名）
    def link_block(match: "re.Match[str]") -> str:
        block, link = match.group(0), match.group(1)
        return block.replace('name=""', f'name="{link}_material"')

    text, count_material = re.subn(
        r'<link\s+name="([^"]+)">.*?</link>',
        link_block,
        text,
        flags=re.S,
    )

    # 7. fixed 关节的零轴
    text, count_axes = re.subn(r'\s*<axis\s+xyz="0 0 0"\s*/>', "", text)

    if not text.endswith("\n"):
        text += "\n"
    out_urdf = model_dir / "urdf" / f"{NEW_NAME}.urdf"
    out_urdf.write_text(text, encoding="utf-8")
    print(f"wrote {out_urdf}")
    print(f"  limits={count_limits} link_blocks={count_material} zero_axes={count_axes}")

    # mesh 拷贝（去点号）
    out_meshes = model_dir / "meshes"
    out_meshes.mkdir(parents=True, exist_ok=True)
    copied = []
    for src in sorted(src_meshes.iterdir()):
        if src.suffix.upper() != ".STL":
            continue
        dst = out_meshes / (
            src.stem.replace("Joint_Link", "Joint_link").replace(".", "_")
            + src.suffix
        )
        shutil.copyfile(src, dst)
        copied.append((src.name, dst.name))
    for a, b in copied:
        print(f"  mesh {a} -> {b}")


if __name__ == "__main__":
    main(Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3]))
