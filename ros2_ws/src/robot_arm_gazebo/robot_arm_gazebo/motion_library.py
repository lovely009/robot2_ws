"""只用于 Gazebo 六轴模型验收的有界关节空间动作。"""

from dataclasses import dataclass
import math
from typing import Dict, Mapping, Sequence, Tuple


JOINT_ORDER = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
JOINT_LIMITS = {
    joint: (-2.0 * math.pi, 2.0 * math.pi) for joint in JOINT_ORDER
}
JOINT_LIMITS["elbow_joint"] = (-math.pi, math.pi)
JOINT_EFFORT_LIMITS = dict(zip(JOINT_ORDER, (60.0, 60.0, 60.0, 10.0, 10.0, 10.0)))

JointVector = Tuple[float, ...]


@dataclass(frozen=True)
class Action:
    name: str
    purpose: str
    targets: Tuple[JointVector, ...]
    durations: Tuple[float, ...]
    positions: Tuple[str, ...] = JOINT_ORDER


def _pose_deg(values: Mapping[str, float] | None = None) -> JointVector:
    requested = dict(values or {})
    unknown = set(requested) - set(JOINT_ORDER)
    if unknown:
        raise ValueError(f"动作包含未知关节：{sorted(unknown)}")
    pose = tuple(math.radians(requested.get(joint, 0.0)) for joint in JOINT_ORDER)
    validate_pose(pose)
    return pose


def validate_pose(values: Sequence[float]) -> None:
    if len(values) != 6 or not all(math.isfinite(value) for value in values):
        raise ValueError("姿态必须包含六个有限弧度值")
    for joint, value in zip(JOINT_ORDER, values):
        lower, upper = JOINT_LIMITS[joint]
        if not lower <= value <= upper:
            raise ValueError(f"{joint}={value} rad 超出 [{lower}, {upper}]")


ZERO = _pose_deg()
HOME = _pose_deg({"shoulder_lift_joint": -90.0, "wrist_1_joint": -90.0})
COMBINED = _pose_deg(
    {
        "shoulder_pan_joint": 20.0,
        "shoulder_lift_joint": -70.0,
        "elbow_joint": 30.0,
        "wrist_1_joint": -65.0,
        "wrist_2_joint": 18.0,
        "wrist_3_joint": 20.0,
    }
)


def _joint_scan_targets() -> Tuple[JointVector, ...]:
    targets = [HOME]
    for joint in JOINT_ORDER:
        values = dict(zip(JOINT_ORDER, HOME))
        values[joint] += math.radians(15.0)
        target = tuple(values[name] for name in JOINT_ORDER)
        validate_pose(target)
        targets.append(target)
    targets.append(HOME)
    return tuple(targets)


ACTIONS: Dict[str, Action] = {
    "home": Action("仿真默认姿态", "平滑进入 UR5e 仿真默认姿态。", (HOME,), (1.8,)),
    "joint_scan": Action(
        "六轴逐轴扫描",
        "按固定 ABI 顺序确认关节映射和正方向。",
        _joint_scan_targets(),
        (1.0,) * 8,
    ),
    "combined": Action(
        "六轴组合动作", "以小幅组合姿态检查六轴协同。", (COMBINED, HOME), (1.8, 1.8)
    ),
    "zero": Action("六轴回零", "平滑进入六轴数学零位。", (ZERO,), (1.8,)),
}


def smoothstep_interpolate(
    start: Sequence[float], target: Sequence[float], progress: float
) -> JointVector:
    if len(start) != 6 or len(target) != 6:
        raise ValueError("smoothstep 输入必须是两个六轴向量")
    u = min(max(progress, 0.0), 1.0)
    blend = u * u * (3.0 - 2.0 * u)
    return tuple(q0 + (q1 - q0) * blend for q0, q1 in zip(start, target))


def menu_text() -> str:
    return "\n".join(
        ["Gazebo UR5e 六轴动作："]
        + [f"  {key:12s} {action.name} — {action.purpose}" for key, action in ACTIONS.items()]
    )
