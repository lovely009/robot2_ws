"""`web_host/models.py` 的单测（模型目录只读视图：臂视图候选与关节映射）。

臂视图的 motor_id → 关节名 映射原先硬编码在前端 store.js、写死成 half_body_robot
的 18 台，换模型就**静默失效**（关节名对不上 → 那些关节角恒为 0、臂上小球点不中，
表现得像"电机没注册"）。现在向模型配置取：① 模型自带 motor_driver.yaml；
② 回落「拓扑 × motor_hardware_interface 权威驱动配置」。这几条守的就是"换模型
不再静默失效"。

只读文件、不起进程；不碰 CAN 与串口。
"""

from __future__ import annotations

from pathlib import Path

import pytest

from web_host.models import (
    enumerate_arm_models,
    has_joint_mapping_source,
    motor_library,
    model_transmissions,
    simulation_model_joints,
)

PKG = Path(__file__).resolve().parent.parent


# --- 枚举：臂视图候选 --------------------------------------------------------

def test_enumerate_arm_models_uses_root(tmp_path: Path) -> None:
    renderable = tmp_path / 'demo'
    (renderable / 'urdf').mkdir(parents=True)
    (renderable / 'urdf' / 'demo.urdf').write_text('<robot/>', encoding='utf-8')
    (tmp_path / 'no_urdf').mkdir()
    assert enumerate_arm_models(tmp_path) == ['demo']


# --- 映射：解析顺序 ① 模型自带驱动配置 ---------------------------------------

def test_model_joints_pins_current_half_body_model() -> None:
    """钉住当前默认模型的内容，与 simulation_topology.yaml 逐条一致。"""
    joints = simulation_model_joints('half_body_robot')
    assert set(joints) == set(range(1, 19)), '必须是 1..18 连续——曾经漏掉 10..18'
    assert joints[1] == 'joint_link1'
    assert joints[9] == 'joint_link9'
    assert joints[10] == 'joint_link1.1'   # 第二链首台
    assert joints[18] == 'joint_link9.9'   # 第二链末台
    # 两条链的关节名不能混：第一链无点号，第二链有点号
    assert all('.' not in joints[i] for i in range(1, 10))
    assert all('.' in joints[i] for i in range(10, 19))


@pytest.mark.parametrize('bad', ['', '..', '../etc', 'a/b', 'nope_does_not_exist'])
def test_model_joints_rejects_bad_model_names(bad: str) -> None:
    """非法模型名必须显式报错，不能悄悄返回空表或读到目录外。"""
    with pytest.raises(ValueError):
        simulation_model_joints(bad)


def test_model_driver_yaml_wins_over_topology(tmp_path: Path,
                                              monkeypatch: pytest.MonkeyPatch) -> None:
    """① 存在时不回查权威配置（id 与 joint 同在一处，一步到位）。"""
    model = tmp_path / 'models' / 'demo'
    (model / 'config').mkdir(parents=True)
    (model / 'config' / 'motor_driver.yaml').write_text(
        'motor_hardware:\n  motors:\n    a: {id: 5, joint: j5}\n',
        encoding='utf-8')
    (model / 'config' / 'simulation_topology.yaml').write_text(
        'simulation:\n  motors:\n    - motor: a\n      joint: other\n',
        encoding='utf-8')

    def failing_share(name):
        raise AssertionError('driver yaml 存在时不该回查 %s' % name)

    monkeypatch.setattr(
        'web_host.models.get_package_share_directory', failing_share)

    assert simulation_model_joints('demo', root=tmp_path / 'models') == {
        5: 'j5'}


# --- 映射：解析顺序 ② 拓扑 × 权威驱动配置 ------------------------------------

def _fake_mhi_share(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    mhi = tmp_path / 'mhi' / 'config'
    mhi.mkdir(parents=True, exist_ok=True)
    (mhi / 'motor_driver.yaml').write_text(
        'motor_hardware:\n'
        '  motors:\n'
        '    right_shoulder_yaw: {id: 1}\n'
        '    right_wrist_pitch: {id: 8}\n',
        encoding='utf-8')

    def fake_share(name):
        assert name == 'motor_hardware_interface'
        return str(tmp_path / 'mhi')

    monkeypatch.setattr('web_host.models.get_package_share_directory', fake_share)


def test_model_joints_falls_back_to_topology_plus_authoritative_driver(
        tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """仅拓扑的模型（无 motor_driver.yaml）：joint 来自拓扑，id 回查权威配置。"""
    model = tmp_path / 'models' / 'demo_arm'
    (model / 'config').mkdir(parents=True)
    (model / 'config' / 'simulation_topology.yaml').write_text(
        'simulation:\n'
        '  motors:\n'
        '    - motor: right_shoulder_yaw\n      joint: joint_link1\n'
        '    - motor: right_wrist_pitch\n      joint: joint_link6\n',
        encoding='utf-8')
    _fake_mhi_share(tmp_path, monkeypatch)

    assert simulation_model_joints('demo_arm', root=tmp_path / 'models') == {
        1: 'joint_link1', 8: 'joint_link6'}


def test_model_joints_rejects_topology_motor_missing_from_driver(
        tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    model = tmp_path / 'models' / 'demo_arm'
    (model / 'config').mkdir(parents=True)
    (model / 'config' / 'simulation_topology.yaml').write_text(
        'simulation:\n'
        '  motors:\n'
        '    - motor: unknown_motor\n      joint: joint_link1\n',
        encoding='utf-8')
    _fake_mhi_share(tmp_path, monkeypatch)

    with pytest.raises(ValueError, match='unknown_motor'):
        simulation_model_joints('demo_arm', root=tmp_path / 'models')


def test_model_joints_rejects_model_without_any_config(tmp_path: Path) -> None:
    (tmp_path / 'models' / 'empty_model').mkdir(parents=True)
    with pytest.raises(ValueError, match='motor_driver.yaml'):
        simulation_model_joints('empty_model', root=tmp_path / 'models')


# --- 映射来源判据：区分「没配」与「配错了」----------------------------------

def test_has_joint_mapping_source_needs_config_not_just_urdf(tmp_path: Path) -> None:
    """只有主 URDF 的模型（预览件）算「没配」，不算契约缺项。

    臂视图候选的判据只有主 URDF，映射另要 config 里的文件——两者本就不一致，
    页面选到这类模型时退回内置兜底表。集成测试靠本判据跳过它们。
    """
    models_root = tmp_path / 'models'
    (models_root / 'preview' / 'urdf').mkdir(parents=True)
    (models_root / 'preview' / 'urdf' / 'preview.urdf').write_text(
        '<robot/>', encoding='utf-8')
    assert enumerate_arm_models(models_root) == ['preview'], '前提：它确实可枚举'
    assert not has_joint_mapping_source('preview', root=models_root)

    (models_root / 'preview' / 'config').mkdir()
    (models_root / 'preview' / 'config' / 'simulation_topology.yaml').write_text(
        'simulation:\n  motors: []\n', encoding='utf-8')
    assert has_joint_mapping_source('preview', root=models_root), '有拓扑就算配了'


@pytest.mark.parametrize('bad', ['', '..', '../etc', 'a/b', 'nope_does_not_exist'])
def test_has_joint_mapping_source_rejects_bad_names(bad: str) -> None:
    """非法/不存在的模型名一律 False——不能借它读到目录外。"""
    assert not has_joint_mapping_source(bad)


# --- 与真实源码树的集成（臂视图 model_dir 可指源码目录）----------------------

def test_model_joints_cover_source_tree_arm_models() -> None:
    """源码树里每个**配了映射**的可渲染模型都必须取得到映射。

    臂视图的模型候选由 model_dir 指向的目录枚举（可指源码树，改模型不必重构建）；
    新模型常常先有拓扑、后补 motor_driver.yaml——解析顺序的②就是为它们准备的。

    只跳过 `has_joint_mapping_source` 为假的模型（预览件：只有 URDF，页面上本来
    就退回内置兜底表）。配了文件却解析不出来的仍报红——那是契约缺项，不是预览。
    """
    from ament_index_python.packages import get_package_share_directory
    try:
        get_package_share_directory('motor_hardware_interface')
    except Exception:
        pytest.skip('motor_hardware_interface 未安装：② 回退路径的前提不成立')

    models_root = PKG.parent / 'robot_description' / 'models'
    if not models_root.is_dir():
        pytest.skip('源码树没有 robot_description/models')
    models = enumerate_arm_models(models_root)
    if not models:
        pytest.skip('源码树没有可渲染模型')
    checked = 0
    for model in models:
        if not has_joint_mapping_source(model, root=models_root):
            continue
        joints = simulation_model_joints(model, root=models_root)
        assert joints, f'{model} 取不出 id→关节映射'
        for motor_id, joint in joints.items():
            assert isinstance(motor_id, int) and motor_id > 0
            assert isinstance(joint, str) and joint, f'{model} 的 id {motor_id} 关节名为空'
        checked += 1
    assert checked, '一个配了映射的模型都没查到：本用例已空转，检查枚举判据'


def test_model_joints_covers_installed_arm_models() -> None:
    """安装树（默认 root）里每个配了映射的可渲染模型也必须能取到映射。"""
    models = enumerate_arm_models()
    if not models:
        pytest.skip('安装树里没有可渲染模型，测试前提不成立')
    checked = 0
    for model in models:
        if not has_joint_mapping_source(model):
            continue
        joints = simulation_model_joints(model)
        assert joints, f'{model} 取不出 id→关节映射'
        checked += 1
    assert checked, '一个配了映射的模型都没查到：本用例已空转，检查枚举判据'


# --- 型号库：设备页折算设备端原值 ---------------------------------------------

def test_motor_library_exposes_gear_ratio_keyed_by_topology_model() -> None:
    """设备页用减速比把关节侧 rad/s 折成**电机端 RPM**（设备对象 0x6080 存电机端量）。

    键必须与固件 Profile 的 .model_name / 拓扑的 model_name 同源——对不上就静默
    不显示设备端值，看起来像"页面没做"。
    """
    lib = motor_library()
    for name in ('cerb_r48_101', 'cerb_r58_101', 'vaxor_2550'):
        assert name in lib, f'型号库缺 {name}：设备页的设备端 RPM 折算会静默失效'
    assert lib['cerb_r48_101']['gear_ratio'] == 101
    assert lib['cerb_r58_101']['gear_ratio'] == 101
    assert lib['vaxor_2550']['gear_ratio'] == 50
    assert lib['cerb_r58_101']['protocol'] == 'cerb'


def test_motor_library_missing_root_is_empty(tmp_path: Path) -> None:
    """取不到型号库要退化成空表（页面不显示该列），不能抛。"""
    assert motor_library(tmp_path) == {}


def test_coupled_metadata_does_not_flatten_current_wrists() -> None:
    root = PKG.parent / "robot_description" / "models"
    metadata = model_transmissions("half_body_robot", root)
    groups = {g["name"]: g for g in metadata["groups"]}
    assert groups["wrist_a"]["motors"] == [
        {"name": "arm_joint8", "id": 8}, {"name": "arm_joint9", "id": 9}]
    assert groups["wrist_a"]["joints"] == ["joint_link8", "joint_link9"]
    assert groups["wrist_b"]["joints"] == ["joint_link8.8", "joint_link9.9"]
    direct = model_transmissions("half_body_robot", root)["direct_joints"]
    assert set(direct) == set(range(1, 19)) - {8, 9, 17, 18}
    assert direct[1] == "joint_link1"
    assert not groups["wrist_a"]["direct_mapping"]


def test_declared_invalid_transmission_metadata_cannot_use_legacy_alias(tmp_path: Path) -> None:
    config = tmp_path / "demo" / "config"
    config.mkdir(parents=True)
    (config / "motor_driver.yaml").write_text(
        "motor_hardware:\n  transmissions_file: missing.yaml\n  motors:\n    a: {id: 1, joint: yaw}\n")
    with pytest.raises(ValueError, match="传动组"):
        model_transmissions("demo", tmp_path)


def test_transmission_policy_defaults_come_from_model_yaml() -> None:
    root = PKG.parent / "robot_description" / "models"
    defaults = model_transmissions("half_body_robot", root)["policy_defaults"]
    assert defaults[8] == {"kp": 0.008, "kd": 1.0}
    assert defaults[9] == {"kp": 0.008, "kd": 1.0}
    assert defaults[1] == {"kp": 30.0, "kd": 1.0}
