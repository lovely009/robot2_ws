"""只读反馈桥的关节转换与映射校验，无 ROS 依赖。"""

import math

MAX_TRAJECTORY_POINTS = 256
MAX_TRAJECTORY_MOTORS = 64


class BridgeError(ValueError):
    """本地校验失败。消息面向操作者：说清是哪一条、为什么。"""


# ---------------------------------------------------------------------------
# 纯函数（无 ROS 依赖，可单测）
# ---------------------------------------------------------------------------

def motion_states_to_joint_state(states, joint_of_motor):
    """`MotorMotionStateArray.states` → JointState 的四条并行数组。

    states: [{'motor_id', 'position', 'velocity', 'effort'}, ...]
    joint_of_motor: {motor_id: joint_name}

    约定与理由：
      · 未映射的电机**忽略**（桥只管映射表里声明过的关节）；
      · 任一量是 NaN/Inf 的关节**整条跳过**——不发 0。发 0 会让 rviz /
        robot_state_publisher 把"没有数据"当成"关节真的在 0 位"，那是在骗人。
    """
    names, pos, vel, eff = [], [], [], []
    for st in states:
        joint = joint_of_motor.get(st.get('motor_id'))
        if joint is None:
            continue
        p, v, e = st.get('position'), st.get('velocity'), st.get('effort')
        if not all(isinstance(x, (int, float)) and math.isfinite(x)
                   for x in (p, v, e)):
            continue
        names.append(joint)
        pos.append(float(p))
        vel.append(float(v))
        eff.append(float(e))
    return names, pos, vel, eff


def parse_joint_map(motor_ids, joint_names):
    """(motor_ids, joint_names) → {motor_id: joint_name}，并做一致性校验。"""
    if len(motor_ids) != len(joint_names):
        raise BridgeError('motor_ids(%d) 与 joint_names(%d) 长度不符'
                          % (len(motor_ids), len(joint_names)))
    if not motor_ids:
        raise BridgeError('映射为空：至少要给一台电机与它的关节名')
    if len(set(motor_ids)) != len(motor_ids):
        raise BridgeError('motor_ids 里有重复：%s' % list(motor_ids))
    return dict(zip(motor_ids, joint_names))


def motor_accepts_joint_target(online: bool, lifecycle: int, fault: int) -> bool:
    """就绪态允许首目标；逐电机策略授权由固件在执行前裁决。"""
    return bool(online) and lifecycle in (3, 4) and fault == 0
