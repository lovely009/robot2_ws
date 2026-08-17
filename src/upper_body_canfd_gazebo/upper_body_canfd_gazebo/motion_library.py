"""辨识度优先的 14-DoF 半身双臂参考动作库。

动作并不是为了复现某个具体人的姿态，而是为了让仿真调试更有参考意义：

* 静态姿态用于确认左右镜像、关节正方向和保持能力；
* 单臂姿态用于区分 vcan0 / vcan1；
* 腕部自检用于依次确认 25/30 mm N=1..3；
* 肩肘自检用于依次确认 RP ID 1..4；
* 挥手和综合动作可观察轨迹平滑性、CAN-FD 连续通信和 MIT 跟踪。

所有角度单位均为 rad，且这里的零位/正方向严格对应本工程 URDF。
"""

from dataclasses import dataclass
from typing import Dict, List, Tuple


JOINT_NAMES = (
    "shoulder_yaw",
    "shoulder_roll",
    "shoulder_pitch",
    "elbow_pitch",
    "wrist_yaw",
    "wrist_pitch",
    "wrist_roll",
)

# 与 urdf/upper_body.urdf.xacro 保持一致，用于在动作库加载时提前拦截越限姿态。
JOINT_LIMITS = (
    (-1.57, 1.57),
    (-1.40, 1.40),
    (-2.20, 2.20),
    (0.00, 2.55),
    (-1.75, 1.75),
    (-1.25, 1.25),
    (-2.80, 2.80),
)

ArmVector = Tuple[float, float, float, float, float, float, float]


@dataclass(frozen=True)
class Pose:
    """左右各 7 个关节的绝对目标位置。"""

    left: ArmVector
    right: ArmVector


def pose_to_command(pose: Pose) -> List[float]:
    """按 position controller 的 joint 顺序展开左右臂目标。"""

    return [*pose.left, *pose.right]


@dataclass(frozen=True)
class Keyframe:
    """一个动作关键帧：先平滑过渡 duration 秒，再保持 hold 秒。"""

    label: str
    pose: Pose
    duration: float = 1.4
    hold: float = 0.25


@dataclass(frozen=True)
class Action:
    number: int
    name: str
    purpose: str
    frames: Tuple[Keyframe, ...]


def arm(
    yaw: float = 0.0,
    roll: float = 0.0,
    pitch: float = 0.0,
    elbow: float = 0.25,
    wrist_yaw: float = 0.0,
    wrist_pitch: float = 0.0,
    wrist_roll: float = 0.0,
) -> ArmVector:
    """按 RP ID 1..4 + 腕部 N=1..3 的顺序生成一条手臂姿态。"""

    return (yaw, roll, pitch, elbow, wrist_yaw, wrist_pitch, wrist_roll)


# 准备姿态故意不贴住肘关节 0 rad 的机械限位，并让两臂略向外、略向前。
READY = Pose(
    left=arm(roll=0.10, pitch=-0.10, elbow=0.25),
    right=arm(roll=-0.10, pitch=-0.10, elbow=0.25),
)

T_POSE = Pose(
    left=arm(roll=1.25, elbow=0.10),
    right=arm(roll=-1.25, elbow=0.10),
)

FORWARD = Pose(
    left=arm(roll=0.12, pitch=-1.25, elbow=0.18),
    right=arm(roll=-0.12, pitch=-1.25, elbow=0.18),
)

LEFT_RAISED = Pose(
    left=arm(yaw=0.30, roll=1.05, pitch=-0.55, elbow=0.90),
    right=READY.right,
)

RIGHT_RAISED = Pose(
    left=READY.left,
    right=arm(yaw=-0.30, roll=-1.05, pitch=-0.55, elbow=0.90),
)

BENT_ARMS = Pose(
    left=arm(roll=0.92, pitch=-0.32, elbow=1.65, wrist_pitch=0.18),
    right=arm(roll=-0.92, pitch=-0.32, elbow=1.65, wrist_pitch=0.18),
)

CHEER = Pose(
    left=arm(roll=0.95, pitch=-1.55, elbow=0.30),
    right=arm(roll=-0.95, pitch=-1.55, elbow=0.30),
)


def _left_wave(wrist_yaw: float, wrist_roll: float) -> Pose:
    return Pose(
        left=arm(
            yaw=0.30,
            roll=1.05,
            pitch=-0.55,
            elbow=0.90,
            wrist_yaw=wrist_yaw,
            wrist_pitch=0.30,
            wrist_roll=wrist_roll,
        ),
        right=READY.right,
    )


def _right_wave(wrist_yaw: float, wrist_roll: float) -> Pose:
    return Pose(
        left=READY.left,
        right=arm(
            yaw=-0.30,
            roll=-1.05,
            pitch=-0.55,
            elbow=0.90,
            wrist_yaw=wrist_yaw,
            wrist_pitch=0.30,
            wrist_roll=wrist_roll,
        ),
    )


def _wrist_pose(wrist_yaw: float = 0.0, wrist_pitch: float = 0.30, wrist_roll: float = 0.0) -> Pose:
    """双臂保持前伸，只改变同一类腕部电机，便于观察 N 编号。"""

    return Pose(
        left=arm(
            roll=0.18,
            pitch=-1.00,
            elbow=0.45,
            wrist_yaw=wrist_yaw,
            wrist_pitch=wrist_pitch,
            wrist_roll=wrist_roll,
        ),
        right=arm(
            roll=-0.18,
            pitch=-1.00,
            elbow=0.45,
            wrist_yaw=wrist_yaw,
            wrist_pitch=wrist_pitch,
            wrist_roll=wrist_roll,
        ),
    )


def _sway_pose(yaw: float) -> Pose:
    """双臂同向左右摆动（“不要不要”摆手）：只激励 shoulder_yaw。"""

    return Pose(
        left=arm(yaw=yaw, roll=0.30, pitch=-0.20, elbow=0.90),
        right=arm(yaw=yaw, roll=-0.30, pitch=-0.20, elbow=0.90),
    )


ACTIONS: Dict[int, Action] = {
    0: Action(
        0,
        "回到准备姿态",
        "建立统一基准，动作之间复位，也可检查零位附近的稳定保持。",
        (Keyframe("回准备姿态", READY, 1.5, 0.35),),
    ),
    1: Action(
        1,
        "T 字双臂展开",
        "重点检查 shoulder_roll 与左右镜像方向；两臂应明显向身体两侧展开。",
        (Keyframe("双臂向两侧展开", T_POSE, 2.0, 0.55),),
    ),
    2: Action(
        2,
        "双臂向前伸",
        "重点检查 shoulder_pitch 正负方向和左右同步；与动作 1 的运动轴明显不同。",
        (Keyframe("双臂向前", FORWARD, 2.0, 0.55),),
    ),
    3: Action(
        3,
        "只抬左臂",
        "只让 vcan0/左臂运动，用来确认左总线、左侧 RP/腕部映射没有串到右臂。",
        (Keyframe("左臂抬起，右臂保持准备位", LEFT_RAISED, 1.8, 0.55),),
    ),
    4: Action(
        4,
        "只抬右臂",
        "只让 vcan1/右臂运动，与动作 3 配对检查两条 CAN 总线是否独立。",
        (Keyframe("右臂抬起，左臂保持准备位", RIGHT_RAISED, 1.8, 0.55),),
    ),
    5: Action(
        5,
        "双臂对称屈肘",
        "重点检查 RP ID4 elbow_pitch、左右同步性以及有重力负载时的 MIT 保持能力。",
        (Keyframe("双侧肩部展开并屈肘", BENT_ARMS, 2.0, 0.65),),
    ),
    6: Action(
        6,
        "左手挥手",
        "动态检查 vcan0 连续 CAN-FD、腕部 N1/N3 协调和轨迹平滑性。",
        (
            Keyframe("左臂进入挥手准备位", LEFT_RAISED, 1.6, 0.25),
            Keyframe("挥手 1/6", _left_wave(0.55, 1.15), 0.45, 0.05),
            Keyframe("挥手 2/6", _left_wave(-0.55, -1.15), 0.55, 0.05),
            Keyframe("挥手 3/6", _left_wave(0.55, 1.15), 0.55, 0.05),
            Keyframe("挥手 4/6", _left_wave(-0.55, -1.15), 0.55, 0.05),
            Keyframe("挥手 5/6", _left_wave(0.55, 1.15), 0.55, 0.05),
            Keyframe("挥手 6/6", _left_wave(0.0, 0.0), 0.55, 0.25),
        ),
    ),
    7: Action(
        7,
        "右手挥手",
        "动态检查 vcan1 连续 CAN-FD；与动作 6 对照可看出左右总线/机构差异。",
        (
            Keyframe("右臂进入挥手准备位", RIGHT_RAISED, 1.6, 0.25),
            Keyframe("挥手 1/6", _right_wave(-0.55, -1.15), 0.45, 0.05),
            Keyframe("挥手 2/6", _right_wave(0.55, 1.15), 0.55, 0.05),
            Keyframe("挥手 3/6", _right_wave(-0.55, -1.15), 0.55, 0.05),
            Keyframe("挥手 4/6", _right_wave(0.55, 1.15), 0.55, 0.05),
            Keyframe("挥手 5/6", _right_wave(-0.55, -1.15), 0.55, 0.05),
            Keyframe("挥手 6/6", _right_wave(0.0, 0.0), 0.55, 0.25),
        ),
    ),
    8: Action(
        8,
        "腕部 N1/N2/N3 三轴自检",
        "按 yaw→pitch→roll 顺序单独激励 25/30 mm 腕部电机，可直接核对节点映射和正负方向。",
        (
            Keyframe("进入腕部观察姿态", _wrist_pose(), 1.7, 0.25),
            Keyframe("N1 wrist_yaw 正向", _wrist_pose(wrist_yaw=0.65), 0.7, 0.20),
            Keyframe("N1 wrist_yaw 反向", _wrist_pose(wrist_yaw=-0.65), 1.0, 0.20),
            Keyframe("N1 回零", _wrist_pose(), 0.7, 0.15),
            Keyframe("N2 wrist_pitch 正向", _wrist_pose(wrist_pitch=0.70), 0.7, 0.20),
            Keyframe("N2 wrist_pitch 反向", _wrist_pose(wrist_pitch=-0.35), 1.0, 0.20),
            Keyframe("N2 回观察基准", _wrist_pose(), 0.7, 0.15),
            Keyframe("N3 wrist_roll 正向", _wrist_pose(wrist_roll=1.10), 0.8, 0.20),
            Keyframe("N3 wrist_roll 反向", _wrist_pose(wrist_roll=-1.10), 1.2, 0.20),
            Keyframe("N3 回零", _wrist_pose(), 0.8, 0.20),
            Keyframe("返回准备姿态", READY, 1.5, 0.25),
        ),
    ),
    9: Action(
        9,
        "RP ID1~4 肩肘逐轴自检",
        "依次激励 shoulder_yaw、roll、pitch、elbow，检查 RP 广播槽 ID1~4 是否一一对应。",
        (
            Keyframe("基准位", READY, 1.2, 0.20),
            Keyframe("RP1 shoulder_yaw", Pose(arm(yaw=0.50, roll=0.10), arm(yaw=-0.50, roll=-0.10)), 0.8, 0.25),
            Keyframe("RP1 回基准", READY, 0.8, 0.15),
            Keyframe("RP2 shoulder_roll", Pose(arm(roll=0.70), arm(roll=-0.70)), 0.8, 0.25),
            Keyframe("RP2 回基准", READY, 0.8, 0.15),
            Keyframe("RP3 shoulder_pitch", Pose(arm(roll=0.10, pitch=-0.75), arm(roll=-0.10, pitch=-0.75)), 0.9, 0.25),
            Keyframe("RP3 回基准", READY, 0.9, 0.15),
            Keyframe("RP4 elbow_pitch", Pose(arm(roll=0.10, pitch=-0.10, elbow=1.25), arm(roll=-0.10, pitch=-0.10, elbow=1.25)), 1.0, 0.30),
            Keyframe("RP4 回基准", READY, 1.0, 0.25),
        ),
    ),
    10: Action(
        10,
        "综合辨识演示",
        "把展开、前伸、屈肘、左右单臂和腕部摆动串起来，作为整条 CAN-FD→MIT→Gazebo 链路回归测试。",
        (
            Keyframe("准备", READY, 1.1, 0.20),
            Keyframe("T 字展开", T_POSE, 1.6, 0.40),
            Keyframe("双臂前伸", FORWARD, 1.6, 0.40),
            Keyframe("双臂屈肘", BENT_ARMS, 1.6, 0.40),
            Keyframe("左臂辨识", LEFT_RAISED, 1.4, 0.30),
            Keyframe("左腕摆动", _left_wave(0.55, 1.10), 0.55, 0.12),
            Keyframe("左腕反向", _left_wave(-0.55, -1.10), 0.75, 0.12),
            Keyframe("右臂辨识", RIGHT_RAISED, 1.4, 0.30),
            Keyframe("右腕摆动", _right_wave(-0.55, -1.10), 0.55, 0.12),
            Keyframe("右腕反向", _right_wave(0.55, 1.10), 0.75, 0.12),
            Keyframe("结束并回准备位", READY, 1.6, 0.40),
        ),
    ),
    11: Action(
        11,
        "双手上举（欢呼）",
        "双臂对称上举过头，检查两条总线的 shoulder_roll+pitch 协调与大角度保持。",
        (
            Keyframe("双臂上举", CHEER, 2.0, 0.60),
            Keyframe("回准备姿态", READY, 2.0, 0.30),
        ),
    ),
    12: Action(
        12,
        "双臂左右齐摆",
        "双臂同向 yaw 摆动，动态检查左右两条总线的 shoulder_yaw 同步性。",
        (
            Keyframe("进入摆手姿态", _sway_pose(0.0), 1.6, 0.20),
            Keyframe("摆 1/6", _sway_pose(0.55), 0.45, 0.05),
            Keyframe("摆 2/6", _sway_pose(-0.55), 0.55, 0.05),
            Keyframe("摆 3/6", _sway_pose(0.55), 0.55, 0.05),
            Keyframe("摆 4/6", _sway_pose(-0.55), 0.55, 0.05),
            Keyframe("摆 5/6", _sway_pose(0.55), 0.55, 0.05),
            Keyframe("摆 6/6", _sway_pose(0.0), 0.55, 0.25),
        ),
    ),
    13: Action(
        13,
        "双手转腕",
        "双臂前伸同时转动腕部 N3（wrist_roll），核对左右腕部节点映射与幅度。",
        (
            Keyframe("进入腕部观察姿态", _wrist_pose(), 1.7, 0.25),
            Keyframe("N3 正向", _wrist_pose(wrist_roll=1.20), 0.8, 0.20),
            Keyframe("N3 反向", _wrist_pose(wrist_roll=-1.20), 1.2, 0.20),
            Keyframe("N3 正向", _wrist_pose(wrist_roll=1.20), 1.2, 0.20),
            Keyframe("N3 回零", _wrist_pose(), 0.8, 0.20),
            Keyframe("回准备姿态", READY, 1.5, 0.25),
        ),
    ),
}


def smoothstep_interpolate(start: Pose, target: Pose, u: float, duration: float) -> Tuple[Pose, Pose]:
    """三次 smoothstep 插值，同时给出位置和解析速度目标。

    s(u)=3u²-2u³，ds/dt=6u(1-u)/T。
    因此每段轨迹的起点和终点速度都为 0，避免角度目标阶跃。
    返回值为 (position_pose, velocity_pose)。
    """

    if duration <= 0.0:
        raise ValueError("duration must be positive")
    u = min(max(float(u), 0.0), 1.0)
    s = 3.0 * u * u - 2.0 * u * u * u
    ds_dt = 6.0 * u * (1.0 - u) / duration

    def interpolate_arm(a: ArmVector, b: ArmVector) -> Tuple[ArmVector, ArmVector]:
        q = tuple(a_i + (b_i - a_i) * s for a_i, b_i in zip(a, b))
        dq = tuple((b_i - a_i) * ds_dt for a_i, b_i in zip(a, b))
        return q, dq  # type: ignore[return-value]

    left_q, left_dq = interpolate_arm(start.left, target.left)
    right_q, right_dq = interpolate_arm(start.right, target.right)
    return Pose(left_q, right_q), Pose(left_dq, right_dq)


def validate_pose(pose: Pose) -> None:
    """确认动作库不会命令超过当前 URDF 的关节范围。"""

    for side_name, values in (("left", pose.left), ("right", pose.right)):
        if len(values) != len(JOINT_NAMES):
            raise ValueError(f"{side_name} arm must contain 7 joints")
        for joint_name, value, (low, high) in zip(JOINT_NAMES, values, JOINT_LIMITS):
            if not low <= value <= high:
                raise ValueError(
                    f"{side_name}_{joint_name}={value:.3f} rad is outside [{low}, {high}]"
                )


def validate_actions() -> None:
    expected_numbers = set(range(0, max(ACTIONS) + 1))
    if set(ACTIONS) != expected_numbers:
        raise ValueError("action numbers must be continuous from 0 to 10")
    for number, action in ACTIONS.items():
        if action.number != number or not action.frames:
            raise ValueError(f"invalid action definition for number {number}")
        for frame in action.frames:
            if frame.duration <= 0.0 or frame.hold < 0.0:
                raise ValueError(f"invalid timing in action {number}: {frame.label}")
            validate_pose(frame.pose)


def menu_text() -> str:
    lines = ["可播放动作："]
    for number in sorted(ACTIONS):
        action = ACTIONS[number]
        lines.append(f"  [{number:>2}] {action.name:<22} - {action.purpose}")
    lines.extend(
        [
            "  [ m] 重新显示菜单",
            "  [ q] 平滑回准备姿态并退出",
            "  [ x] 立即失能并退出（仿真急停）",
        ]
    )
    return "\n".join(lines)


# 在导入时就检查全部关键帧。改动作参数后若越过 URDF 限位，会立即报错而不是运行时才发现。
validate_actions()
