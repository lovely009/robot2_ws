"""`scripts/mujoco_command_mapping.py` 的纯逻辑测试。

不启动 mujoco、不依赖 rclpy：被测模块按路径加载（`scripts/` 不是 python 包），
这样 pytest 在系统 python3 下也能跑。
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
from types import ModuleType

HERE = Path(__file__).parent
MODULE_PATH = HERE.parent / "scripts" / "mujoco_command_mapping.py"

# 第一链 1..9 后接第二链 1.1..9.9：这是 half_body_robot 的拓扑顺序，也是 MJCF 的声明顺序。
TOPOLOGY = [
    "joint_link1", "joint_link2", "joint_link3", "joint_link4", "joint_link5",
    "joint_link6", "joint_link7", "joint_link8", "joint_link9",
    "joint_link1.1", "joint_link2.2", "joint_link3.3", "joint_link4.4", "joint_link5.5",
    "joint_link6.6", "joint_link7.7", "joint_link8.8", "joint_link9.9",
]


def _module() -> ModuleType:
    spec = importlib.util.spec_from_file_location("mujoco_command_mapping", MODULE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_parse_joint_names_keeps_order_and_drops_blanks() -> None:
    module = _module()

    assert module.parse_joint_names(" joint_b ,joint_a,, ") == ["joint_b", "joint_a"]
    assert module.parse_joint_names("") == []


def test_empty_joint_names_falls_back_to_positional_order() -> None:
    module = _module()

    assert module.resolve_command_indices([], TOPOLOGY[:3]) == [(0, 0), (1, 1), (2, 2)]
    assert module.mapping_error([], TOPOLOGY[:3], 3) is None


def test_empty_joint_names_rejects_wrong_length() -> None:
    module = _module()

    error = module.mapping_error([], TOPOLOGY, 9)

    assert error is not None
    assert "9" in error and "18" in error


def test_named_mapping_follows_names_not_positions() -> None:
    """名字顺序与 MJCF 顺序不同时，必须按名字对齐——这是本次修复的核心。"""
    module = _module()
    # 桥看到的 actuator 顺序 = MJCF 顺序；命令数组按拓扑顺序给，但拓扑只含一个子集。
    actuator_joint_names = ["joint_link1", "joint_link2", "joint_link3"]
    joint_names = ["joint_link3", "joint_link1"]

    assert module.mapping_error(joint_names, actuator_joint_names, 2) is None
    assert module.resolve_command_indices(joint_names, actuator_joint_names) == [(0, 2), (1, 0)]


def test_dotted_names_are_not_sorted() -> None:
    """带点号的第二链：按名字映射时不得退化成字典序（sorted 会插入 link2 之前）。"""
    module = _module()
    assert module.resolve_command_indices(TOPOLOGY, TOPOLOGY) == [
        (index, index) for index in range(len(TOPOLOGY))
    ]

    # 反证：字典序与拓扑序在 half_body_robot 上确实不同，所以"按名字"不是空话。
    assert sorted(TOPOLOGY) != TOPOLOGY


def test_named_mapping_rejects_unknown_or_duplicate_names() -> None:
    module = _module()
    actuator_joint_names = ["joint_link1", "joint_link2"]

    unknown = module.mapping_error(["joint_link1", "joint_link9"], actuator_joint_names, 2)
    assert unknown is not None and "joint_link9" in unknown

    duplicated = module.mapping_error(["joint_link1", "joint_link1"], actuator_joint_names, 2)
    assert duplicated is not None and "joint_link1" in duplicated

    wrong_length = module.mapping_error(["joint_link1"], actuator_joint_names, 2)
    assert wrong_length is not None and "joint_names" in wrong_length
