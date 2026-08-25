"""用于 Gazebo 模型验收的 30 轴参考动作库。

动作只驱动 Gazebo effort controller，不经过 USB 电机仿真节点。角度输入在
构造动作时使用 degree，公开关键帧统一存储为 rad，并严格遵循 motors.yaml 的
30 轴顺序。
"""

from dataclasses import dataclass
import math
from typing import Dict, Mapping, Sequence, Tuple


JOINT_ORDER = (
    "left_scapula_protraction_joint",
    "left_scapula_elevation_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_pitch_joint",
    "left_forearm_roll_joint",
    "left_wrist_flexion_joint",
    "left_wrist_deviation_joint",
    "right_scapula_protraction_joint",
    "right_scapula_elevation_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_pitch_joint",
    "right_forearm_roll_joint",
    "right_wrist_flexion_joint",
    "right_wrist_deviation_joint",
    "left_hip_yaw_joint",
    "left_hip_roll_joint",
    "left_hip_pitch_joint",
    "left_knee_pitch_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_yaw_joint",
    "right_hip_roll_joint",
    "right_hip_pitch_joint",
    "right_knee_pitch_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
)

_PROFILE_LIMITS_DEG = {
    "scapula_protraction": (-25.0, 25.0),
    "scapula_elevation": (-15.0, 30.0),
    "shoulder_pitch": (-60.0, 150.0),
    "shoulder_roll": (-30.0, 150.0),
    "shoulder_yaw": (-90.0, 90.0),
    "elbow_pitch": (0.0, 145.0),
    "forearm_roll": (-90.0, 90.0),
    "wrist_flexion": (-70.0, 70.0),
    "wrist_deviation": (-25.0, 25.0),
    "hip_yaw": (-45.0, 45.0),
    "hip_roll": (-35.0, 35.0),
    "hip_pitch": (-30.0, 120.0),
    "knee_pitch": (0.0, 135.0),
    "ankle_pitch": (-40.0, 30.0),
    "ankle_roll": (-25.0, 25.0),
}
_PROFILE_EFFORT_NM = {
    "scapula_protraction": 30.0,
    "scapula_elevation": 30.0,
    "shoulder_pitch": 60.0,
    "shoulder_roll": 60.0,
    "shoulder_yaw": 60.0,
    "elbow_pitch": 60.0,
    "forearm_roll": 4.0,
    "wrist_flexion": 4.0,
    "wrist_deviation": 4.0,
    "hip_yaw": 60.0,
    "hip_roll": 60.0,
    "hip_pitch": 60.0,
    "knee_pitch": 60.0,
    "ankle_pitch": 10.0,
    "ankle_roll": 10.0,
}


def _profile(joint: str) -> str:
    for side in ("left_", "right_"):
        if joint.startswith(side):
            return joint[len(side) : -len("_joint")]
    raise ValueError(f"未知关节名：{joint}")


JOINT_LIMITS = {
    joint: tuple(math.radians(value) for value in _PROFILE_LIMITS_DEG[_profile(joint)])
    for joint in JOINT_ORDER
}
JOINT_EFFORT_LIMITS = {
    joint: _PROFILE_EFFORT_NM[_profile(joint)] for joint in JOINT_ORDER
}

JointVector = Tuple[float, ...]


@dataclass(frozen=True)
class Keyframe:
    label: str
    values: JointVector
    duration: float = 1.4
    hold: float = 0.25


@dataclass(frozen=True)
class Action:
    name: str
    purpose: str
    keyframes: Tuple[Keyframe, ...]
    positions: Tuple[str, ...] = JOINT_ORDER


def apply_shoulder_rhythm(
    raise_deg: float,
    *,
    enabled: bool = False,
    forward_raise_deg: float | None = None,
) -> Tuple[float, float]:
    """返回 (肩胛上提, 肩胛前伸) degree；默认不启用肩肱节律。"""

    if not enabled:
        return (0.0, 0.0)
    forward = raise_deg if forward_raise_deg is None else forward_raise_deg
    elevation = 0.25 * max(0.0, raise_deg - 30.0)
    protraction = 0.12 * max(0.0, forward - 30.0)
    elevation_limits = _PROFILE_LIMITS_DEG["scapula_elevation"]
    protraction_limits = _PROFILE_LIMITS_DEG["scapula_protraction"]
    return (
        min(max(elevation, elevation_limits[0]), elevation_limits[1]),
        min(max(protraction, protraction_limits[0]), protraction_limits[1]),
    )


def _pose_deg(values: Mapping[str, float] | None = None, **named: float) -> JointVector:
    requested = dict(values or {})
    requested.update(named)
    unknown = set(requested) - set(JOINT_ORDER)
    if unknown:
        raise ValueError(f"动作包含未知关节：{sorted(unknown)}")
    result = tuple(math.radians(requested.get(joint, 0.0)) for joint in JOINT_ORDER)
    validate_pose(result)
    return result


def validate_pose(values: Sequence[float]) -> None:
    if len(values) != len(JOINT_ORDER):
        raise ValueError(f"姿态必须包含 30 个值，收到 {len(values)}")
    for joint, value in zip(JOINT_ORDER, values):
        lower, upper = JOINT_LIMITS[joint]
        if not lower <= value <= upper:
            raise ValueError(f"{joint}={value} rad 超出 [{lower}, {upper}]")


NEUTRAL = _pose_deg()


def _joint_scan_frames() -> Tuple[Keyframe, ...]:
    frames = [Keyframe("30 轴零位", NEUTRAL, 1.2, 0.2)]
    for joint in JOINT_ORDER:
        lower_deg, upper_deg = _PROFILE_LIMITS_DEG[_profile(joint)]
        target_deg = min(20.0, upper_deg * 0.35) if upper_deg > 0.0 else max(-20.0, lower_deg * 0.35)
        frames.append(Keyframe(f"扫描 {joint}", _pose_deg({joint: target_deg}), 0.6, 0.12))
    frames.append(Keyframe("扫描完成回零", NEUTRAL, 1.2, 0.3))
    return tuple(frames)


_rhythm_elevation, _rhythm_protraction = apply_shoulder_rhythm(
    60.0, enabled=True
)
SHOULDER_RHYTHM_POSE = _pose_deg(
    {
        "left_scapula_protraction_joint": _rhythm_protraction,
        "left_scapula_elevation_joint": _rhythm_elevation,
        "left_shoulder_pitch_joint": 60.0,
        "right_scapula_protraction_joint": _rhythm_protraction,
        "right_scapula_elevation_joint": _rhythm_elevation,
        "right_shoulder_pitch_joint": 60.0,
    }
)
ARM_COMBO_POSE = _pose_deg(
    {
        "left_shoulder_pitch_joint": 45.0,
        "left_shoulder_roll_joint": 35.0,
        "left_shoulder_yaw_joint": 20.0,
        "left_elbow_pitch_joint": 80.0,
        "left_forearm_roll_joint": 35.0,
        "left_wrist_flexion_joint": 25.0,
        "left_wrist_deviation_joint": 12.0,
        "right_shoulder_pitch_joint": 45.0,
        "right_shoulder_roll_joint": 35.0,
        "right_shoulder_yaw_joint": 20.0,
        "right_elbow_pitch_joint": 80.0,
        "right_forearm_roll_joint": 35.0,
        "right_wrist_flexion_joint": 25.0,
        "right_wrist_deviation_joint": 12.0,
    }
)
SUSPENDED_LEGS_POSE = _pose_deg(
    {
        "left_hip_yaw_joint": 10.0,
        "left_hip_roll_joint": 12.0,
        "left_hip_pitch_joint": 32.0,
        "left_knee_pitch_joint": 62.0,
        "left_ankle_pitch_joint": -18.0,
        "left_ankle_roll_joint": 8.0,
        "right_hip_yaw_joint": 10.0,
        "right_hip_roll_joint": 12.0,
        "right_hip_pitch_joint": 32.0,
        "right_knee_pitch_joint": 62.0,
        "right_ankle_pitch_joint": -18.0,
        "right_ankle_roll_joint": 8.0,
    }
)
MIRROR_POSE = _pose_deg(
    {
        "left_scapula_protraction_joint": 8.0,
        "right_scapula_protraction_joint": 8.0,
        "left_shoulder_roll_joint": 20.0,
        "right_shoulder_roll_joint": 20.0,
        "left_shoulder_yaw_joint": 25.0,
        "right_shoulder_yaw_joint": 25.0,
        "left_elbow_pitch_joint": 55.0,
        "right_elbow_pitch_joint": 55.0,
        "left_hip_roll_joint": 15.0,
        "right_hip_roll_joint": 15.0,
        "left_ankle_roll_joint": 10.0,
        "right_ankle_roll_joint": 10.0,
    }
)


ACTIONS: Dict[str, Action] = {
    "joint_scan": Action(
        "30 轴逐轴扫描",
        "按固定 ABI 顺序逐个确认关节映射、正方向和限位。",
        _joint_scan_frames(),
    ),
    "shoulder_rhythm": Action(
        "肩肱节律",
        "显式联动新增的肩胛前伸、上提轴与肩关节抬举。",
        (
            Keyframe("肩部联动抬举", SHOULDER_RHYTHM_POSE, 2.0, 0.6),
            Keyframe("肩部回零", NEUTRAL, 2.0, 0.3),
        ),
    ),
    "arm_combo": Action(
        "双臂九轴组合",
        "同时检查肩部复合体、双轴肘部和双轴腕部的协调运动。",
        (
            Keyframe("双臂组合姿态", ARM_COMBO_POSE, 2.2, 0.7),
            Keyframe("双臂回零", NEUTRAL, 2.0, 0.3),
        ),
    ),
    "suspended_legs": Action(
        "悬空双腿",
        "在躯体固定条件下检查双腿各六轴及踝部 Vaxor。",
        (
            Keyframe("双腿屈曲", SUSPENDED_LEGS_POSE, 2.2, 0.8),
            Keyframe("双腿回零", NEUTRAL, 2.2, 0.3),
        ),
    ),
    "mirror_check": Action(
        "左右镜像检查",
        "以同号解剖命令检查左右镜像关节是否串轴或反向。",
        (
            Keyframe("左右镜像姿态", MIRROR_POSE, 2.0, 0.8),
            Keyframe("镜像检查回零", NEUTRAL, 2.0, 0.3),
        ),
    ),
}


def smoothstep_interpolate(
    start: Sequence[float],
    target: Sequence[float],
    u: float,
    duration: float,
) -> Tuple[JointVector, JointVector]:
    if len(start) != len(JOINT_ORDER) or len(target) != len(JOINT_ORDER):
        raise ValueError("smoothstep 输入必须是两个 30 轴向量")
    if duration <= 0.0:
        raise ValueError("duration 必须 > 0")
    progress = min(max(u, 0.0), 1.0)
    blend = progress * progress * (3.0 - 2.0 * progress)
    blend_rate = 6.0 * progress * (1.0 - progress) / duration
    position = tuple(
        q0 + (q1 - q0) * blend for q0, q1 in zip(start, target)
    )
    velocity = tuple((q1 - q0) * blend_rate for q0, q1 in zip(start, target))
    return position, velocity


def menu_text() -> str:
    lines = ["Gazebo 30 轴模型动作："]
    for key, action in ACTIONS.items():
        lines.append(f"  {key:18s} {action.name} — {action.purpose}")
    return "\n".join(lines)
