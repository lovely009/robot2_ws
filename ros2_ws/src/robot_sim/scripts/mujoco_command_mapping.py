#!/usr/bin/env python3
"""命令数组下标到 MuJoCo actuator 的关节名映射。

命令数组无名，集成路径按拓扑电机顺序发布，MJCF 则按模型顺序声明；
必须通过 launch 传入的关节名映射，不能依赖两种顺序恰好一致。
本模块不依赖 mujoco/rclpy，便于在系统 Python 中测试。
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

    joint_names 为空时，命令须与 MJCF 关节等长且顺序一致。
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
