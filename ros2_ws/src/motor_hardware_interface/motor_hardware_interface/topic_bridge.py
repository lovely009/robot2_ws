"""只读反馈桥的关节转换与映射校验，无 ROS 依赖。"""

import math

class BridgeError(ValueError):
    """反馈映射或关节参数无效。"""

def motion_states_to_joint_state(states, joint_of_motor):
    """生成 JointState 数组，跳过未映射或含无效测量值的电机。"""
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
