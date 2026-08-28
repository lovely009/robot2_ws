"""Gazebo launch 在创建进程前使用的纯校验函数。"""

import math
from typing import Sequence


def parse_initial_positions(
    raw: str, limits: Sequence[tuple[float, float]]
) -> list[float]:
    """解析六轴弧度向量，并以模型 hard limit 作为启动边界。"""

    try:
        values = [float(item.strip()) for item in raw.split(",")]
    except ValueError as error:
        raise ValueError(
            "initial_positions must contain six finite radians"
        ) from error
    if len(values) != 6 or len(limits) != 6 or not all(
        math.isfinite(value) for value in values
    ):
        raise ValueError("initial_positions must contain six finite radians")
    if any(
        not lower <= value <= upper
        for value, (lower, upper) in zip(values, limits)
    ):
        raise ValueError("initial_positions exceed UR5e hard limits")
    return values
