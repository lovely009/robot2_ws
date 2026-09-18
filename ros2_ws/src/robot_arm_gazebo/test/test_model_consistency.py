"""模型目录自洽性：URDF / xacro / controllers.yaml 的关节清单必须逐项一致。

不启动 ROS 节点与 Gazebo，只做静态渲染与解析。每个 models/<模型名>/ 都要满足：
- 三件套齐全：urdf/<模型名>.urdf、urdf/<模型名>.urdf.xacro、config/controllers.yaml；
- 关节顺序：URDF == xacro 渲染出的 <ros2_control> == controllers.yaml 的 joints
  （顺序即 Float64MultiArray 的命令顺序，错位会静默给错关节）；
- URDF 引用的 mesh 文件真实存在；
- 关节限位不退化：lower < upper、effort > 0、velocity > 0
  （SolidWorks 导出的 0/0/0/0 会让 RViz 滑杆拖不动、Gazebo 锁死关节）；
- 派生的 standalone 单文件与 dist 对外发布件都与 URDF 同源
  （防止改了一处忘了重生成另一处）。
"""

import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

HERE = Path(__file__).parent
PACKAGE_DIR = HERE.parent
MODELS_DIR = PACKAGE_DIR / "models"
DIST_DIR = PACKAGE_DIR / "dist"
MESH_PREFIX = "package://robot_arm_gazebo/"
LIMITED_TYPES = {"revolute", "prismatic"}


def _limit_attributes(joint):
    """关节 <limit> 的属性字典；没有 <limit> 时返回空（fixed/continuous 合法）。"""
    limit = joint.find("limit")
    return {} if limit is None else dict(limit.attrib)


def _joint_shape(root):
    return [
        (
            joint.get("name"),
            joint.get("type"),
            joint.find("parent").get("link"),
            joint.find("child").get("link"),
            tuple(sorted(_limit_attributes(joint).items())),
        )
        for joint in root.findall("joint")
    ]


def _link_masses(root):
    return {
        link.get("name"): link.find("inertial/mass").get("value")
        for link in root.findall("link")
    }


def _model_dirs():
    return sorted(path for path in MODELS_DIR.iterdir() if path.is_dir())


def _model_ids(model_dir):
    return model_dir.name


def _render_xacro(xacro_file, controllers_file):
    result = subprocess.run(
        ["xacro", str(xacro_file), f"controllers_file:={controllers_file}"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    return ET.fromstring(result.stdout)


def _physical_joints(root):
    return [joint.get("name") for joint in root.findall("joint")]


def _control_joints(root):
    return [joint.get("name") for joint in root.findall("ros2_control/joint")]


def _controller_joints(controllers_file):
    data = yaml.safe_load(controllers_file.read_text(encoding="utf-8"))
    return data["/**/arm_position_controller"]["ros__parameters"]["joints"]


@pytest.mark.parametrize("model_dir", _model_dirs(), ids=_model_ids)
def test_model_directory_is_complete(model_dir):
    name = model_dir.name
    for relative in (
        f"urdf/{name}.urdf",
        f"urdf/{name}.urdf.xacro",
        "config/controllers.yaml",
        "README.md",
    ):
        assert (model_dir / relative).is_file(), f"{name} 缺少 {relative}"


@pytest.mark.parametrize("model_dir", _model_dirs(), ids=_model_ids)
def test_joint_lists_agree_in_order(model_dir):
    name = model_dir.name
    controllers_file = model_dir / "config" / "controllers.yaml"
    urdf_root = ET.parse(model_dir / "urdf" / f"{name}.urdf").getroot()
    xacro_root = _render_xacro(
        model_dir / "urdf" / f"{name}.urdf.xacro", controllers_file
    )

    physical = _physical_joints(urdf_root)
    control = _control_joints(xacro_root)
    controllers = _controller_joints(controllers_file)

    assert control == physical, (
        f"{name}: ros2_control 关节清单 {control} 与 URDF 物理关节 {physical} 不一致"
    )
    assert controllers == physical, (
        f"{name}: controllers.yaml 的 joints {controllers} 与 URDF 物理关节 "
        f"{physical} 不一致（顺序即命令顺序）"
    )


@pytest.mark.parametrize("model_dir", _model_dirs(), ids=_model_ids)
def test_urdf_meshes_and_limits_are_usable(model_dir):
    name = model_dir.name
    root = ET.parse(model_dir / "urdf" / f"{name}.urdf").getroot()

    meshes = {mesh.get("filename") for mesh in root.iter("mesh")}
    assert meshes, f"{name}: URDF 没有任何 mesh 引用"
    for filename in sorted(meshes):
        assert filename.startswith(MESH_PREFIX), (
            f"{name}: mesh 路径必须以 {MESH_PREFIX} 开头（当前 {filename}）"
        )
        assert (PACKAGE_DIR / filename[len(MESH_PREFIX):]).is_file(), (
            f"{name}: mesh 文件不存在 {filename}"
        )

    for joint in root.findall("joint"):
        if joint.get("type") not in LIMITED_TYPES:
            continue  # fixed/continuous 没有 <limit> 是合法的
        attributes = _limit_attributes(joint)
        assert attributes, f"{name}: 关节 {joint.get('name')} 缺少 <limit>"
        lower = float(attributes["lower"])
        upper = float(attributes["upper"])
        assert lower < upper, (
            f"{name}: 关节 {joint.get('name')} 限位退化（{lower}..{upper}），关节会被锁死"
        )
        assert float(attributes["effort"]) > 0, f"{name}: {joint.get('name')} effort 为 0"
        assert float(attributes["velocity"]) > 0, f"{name}: {joint.get('name')} velocity 为 0"


@pytest.mark.parametrize("model_dir", _model_dirs(), ids=_model_ids)
def test_standalone_matches_urdf(model_dir):
    # standalone 是派生文件：改了 URDF 必须重生成，否则两份模型会悄悄分叉。
    name = model_dir.name
    urdf_root = ET.parse(model_dir / "urdf" / f"{name}.urdf").getroot()
    standalone_file = model_dir / "urdf" / f"{name}.standalone.urdf"
    assert standalone_file.is_file(), f"{name}: 缺少 {standalone_file.name}"
    standalone_root = ET.parse(standalone_file).getroot()

    assert _joint_shape(standalone_root) == _joint_shape(urdf_root), (
        f"{name}: standalone 与 URDF 的关节定义不一致，需重新生成 standalone"
    )
    assert len(standalone_root.findall("ros2_control")) == 0
    assert len(standalone_root.findall("gazebo")) == 0


@pytest.mark.parametrize("model_dir", _model_dirs(), ids=_model_ids)
def test_dist_artifact_matches_model(model_dir):
    # dist/<名>_urdf/ 是派生发布件：改了模型必须重生成，否则发出去的是旧模型。
    name = model_dir.name
    dist_urdf = DIST_DIR / f"{name}_urdf" / f"{name}.urdf"
    assert dist_urdf.is_file(), f"{name}: 缺少对外发布件 {dist_urdf}"

    model_root = ET.parse(model_dir / "urdf" / f"{name}.urdf").getroot()
    dist_root = ET.parse(dist_urdf).getroot()

    assert _joint_shape(dist_root) == _joint_shape(model_root), (
        f"{name}: dist 的关节定义与模型不一致，需重生成发布件"
    )
    assert _link_masses(dist_root) == _link_masses(model_root), (
        f"{name}: dist 的质量参数与模型不一致"
    )
    for mesh in dist_root.iter("mesh"):
        filename = mesh.get("filename")
        assert not filename.startswith("package://"), (
            f"{name}: 发布件的网格必须用相对路径（当前 {filename}），否则脱离 ROS 打不开"
        )
        assert (dist_urdf.parent / filename).is_file(), f"{name}: 发布件缺少网格 {filename}"
