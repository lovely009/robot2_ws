#!/usr/bin/env python3
"""命令数组下标 ↔ MuJoCo actuator 下标的纯映射。

为什么单独成模块：`/arm_effort_controller/commands` 是**无名**的 Float64MultiArray，
顺序语义完全由发布方决定。集成路径（`simulation.launch.py`）里发布方是 `motor_sim`，
它按 `simulation_topology.yaml` 的**电机顺序**发（`motor_sim_node.cpp` 用
`config().motors` 下标索引 `last_efforts_`）；而 MuJoCo 桥的 actuator 顺序是 **MJCF
声明顺序**（生成器按 URDF 顺序写 `<motor>`）。两者对 half_body_robot 这 18 个关节恰好一致，
但那是巧合——Gazebo 侧同类的顺序错位已经踩过一次。所以桥改为按**关节名**吃命令：
launch 把拓扑顺序的关节名交给它，本模块负责把「命令下标 → 关节名 → actuator 下标」
翻译出来。

本模块刻意**不 import mujoco / rclpy**：pytest 跑在系统 python3，那里 import 不到
`mujoco_ros2_control` 的 venv，桥脚本本身无法被测试导入，纯逻辑只能放在这里。
"""

from __future__ import annotations

from collections.abc import Sequence


def parse_joint_names(text: str) -> list[str]:
    """把 launch 传来的逗号分隔关节名解析成有序列表；空白项丢弃，顺序原样保留。"""
    return [name.strip() for name in text.split(",") if name.strip()]


def resolve_command_indices(
    joint_names: Sequence[str],
    actuator_joint_names: Sequence[str],
) -> list[tuple[int, int]]:
    """返回 (命令下标, actuator 下标) 对。

    `joint_names` 为空 = 旧语义：数组顺序即 MJCF 关节顺序、长度等于关节数，
    位置一一对应（手工 `ros2 topic pub` 那 18 个数的用法保留）。
    """
    if not joint_names:
        return [(index, index) for index in range(len(actuator_joint_names))]
    actuator_of_name = {name: index for index, name in enumerate(actuator_joint_names)}
    return [
        (command_index, actuator_of_name[name])
        for command_index, name in enumerate(joint_names)
        if name in actuator_of_name
    ]


def mapping_error(
    joint_names: Sequence[str],
    actuator_joint_names: Sequence[str],
    command_length: int,
) -> str | None:
    """校验这一次命令能否按当前映射下发；返回人话错误说明，None 表示可用。

    只校验本次命令的可用性：名字重复、名字不在模型里、长度不符都拒绝整帧，
    而不是悄悄少发几个关节（静默错位正是这条路要避免的失效模式）。
    """
    if not joint_names:
        if command_length != len(actuator_joint_names):
            return (
                f"命令长度 {command_length} 与 MuJoCo 关节数 "
                f"{len(actuator_joint_names)} 不符（未给 joint_names 时按 MJCF 顺序等长下发）"
            )
        return None

    if command_length != len(joint_names):
        return f"命令长度 {command_length} 与 joint_names 长度 {len(joint_names)} 不符"

    known = set(actuator_joint_names)
    duplicates: list[str] = []
    seen: set[str] = set()
    for name in joint_names:
        if name in seen and name not in duplicates:
            duplicates.append(name)
        seen.add(name)
    if duplicates:
        return "joint_names 有重复：" + ", ".join(duplicates)

    unknown = [name for name in joint_names if name not in known]
    if unknown:
        return "这些关节在 MuJoCo 模型里没有 actuator：" + ", ".join(unknown)

    return None
