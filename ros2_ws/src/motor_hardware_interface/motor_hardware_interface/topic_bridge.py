"""标准关节话题的转换与校验，供节点和测试共用，无 ROS 依赖。

固件会静默丢弃非法轨迹，因此在桥上按相同规则提前校验并报告原因。
"""

import math

# 固件侧硬上限（与 motor_control.h 的 MOTOR_CONTROL_MAX_TRAJECTORY_* 一致）。
MAX_TRAJECTORY_POINTS = 256
MAX_TRAJECTORY_MOTORS = 64
# 单消息承载完整轨迹（最多 256 点），与 web_host 的 TRAJ_SEGMENT_POINTS 一致。
# XRCE 窗口为 MTU 512 × history 16 = 8 KB；256 点约 5.2 KB（11 片）。
# 拆成两条 128 点会增至约 12 片；窗口外丢片会导致整组静默组装失败，无事件或回执。
MAX_TRAJECTORY_SEGMENT_POINTS = MAX_TRAJECTORY_POINTS
# CERB 的 MIT 位置量程（关节侧 ±π）。
POSITION_LIMIT_RAD = math.pi


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


def transpose_trajectory(joint_names, samples, joint_of_motor):
    """标准 `JointTrajectory` 的 (joint_names, points) → 固件的 (target_ids, points)。

    samples: [{'t_ms': int, 'positions': [...], 'velocities': [...] 或 None,
               'effort': [...] 或 None}, ...]（按时间升序）

    返回 (target_ids, flat_points, warnings)：
      target_ids  —— 参与本轨迹的电机 id（升序）
      flat_points —— [(t_ms, motor_id, pos, vel, eff), ...] 按时间升序
      warnings    —— 不致命但值得说的（例如位置超出 ±π 量程）

    形状转换：标准消息是"每个采样点一串关节值"，固件要的是"每个 (时刻, 电机) 一个点"，
    故按 (时刻, 电机) 双层展开，并保持时间非递减。

    校验规则**照抄固件的**（`trajectory_validate`），只为把失败从"固件静默丢弃"
    变成"桥当场报错"：未知关节、向量长度不符、NaN/Inf、时间非严格递增、点数/台数超限。
    """
    if not joint_names:
        raise BridgeError('joint_names 为空：不知道要控制哪些关节')
    if not samples:
        raise BridgeError('轨迹没有任何点')

    unknown = [j for j in joint_names if j not in joint_of_motor]
    if unknown:
        raise BridgeError('未知关节 %s；本桥认识的关节：%s'
                          % (', '.join(unknown), ', '.join(sorted(joint_of_motor))))
    motor_ids = [joint_of_motor[j] for j in joint_names]
    if len(set(motor_ids)) != len(motor_ids):
        raise BridgeError('关节映射到重复的 motor_id：%s' % motor_ids)
    if len(motor_ids) > MAX_TRAJECTORY_MOTORS:
        raise BridgeError('受控电机 %d 台超过固件上限 %d'
                          % (len(motor_ids), MAX_TRAJECTORY_MOTORS))

    flat, warnings = [], []
    previous_t = None
    for i, s in enumerate(samples):
        t_ms = int(s['t_ms'])
        if previous_t is not None and t_ms <= previous_t:
            raise BridgeError('第 %d 个采样点的时间 %d ms 未严格递增（上一个 %d ms）'
                              % (i, t_ms, previous_t))
        previous_t = t_ms
        positions = s.get('positions')
        if positions is None or len(positions) != len(joint_names):
            raise BridgeError('第 %d 个采样点的 positions 长度 %s 与 joint_names(%d) 不符'
                              % (i, '缺失' if positions is None else len(positions),
                                 len(joint_names)))
        velocities = s.get('velocities')
        efforts = s.get('effort')
        for k, joint in enumerate(joint_names):
            p = positions[k]
            v = velocities[k] if velocities is not None else 0.0
            e = efforts[k] if efforts is not None else 0.0
            for label, val in (('position', p), ('velocity', v), ('effort', e)):
                if not isinstance(val, (int, float)) or not math.isfinite(val):
                    raise BridgeError('第 %d 个采样点、关节 %s 的 %s 不是有限数（%r）'
                                      % (i, joint, label, val))
            if abs(p) > POSITION_LIMIT_RAD + 1e-6:
                warnings.append('关节 %s 在 %d ms 的位置 %.3f rad 超出 ±π 量程'
                                % (joint, t_ms, p))
            flat.append((t_ms, motor_ids[k], float(p), float(v), float(e)))

    if len(flat) > MAX_TRAJECTORY_POINTS:
        raise BridgeError('展开后 %d 个点超过固件上限 %d（关节数 × 采样点数）'
                          % (len(flat), MAX_TRAJECTORY_POINTS))
    return sorted(set(motor_ids)), flat, warnings


def split_trajectory(joint_names, samples, joint_of_motor):
    """把一条合法轨迹切成可靠下发消息，但不改变整条时间线。

    返回 ``(target_ids, segments, warnings)``。``segments`` 内的每个元素都是
    ``(t_ms, motor_id, position, velocity, effort)`` 列表，时间仍是整条轨迹的
    absolute ``time_from_start_ms``，不会从 0 重置。总路点数仍不能超过固件的
    256 点上限；分段只是传输边界，不是容量扩展。

    分段容量等于总量上限（见 ``MAX_TRAJECTORY_SEGMENT_POINTS``），所以一条
    合法轨迹**就是一条消息**；切分逻辑保留只是为了固定「按绝对时间线连续切」
    这个语义，改了常量也不会变成多条独立轨迹。
    """
    target_ids, flat, warnings = transpose_trajectory(
        joint_names, samples, joint_of_motor)
    segments = [
        flat[start:start + MAX_TRAJECTORY_SEGMENT_POINTS]
        for start in range(0, len(flat), MAX_TRAJECTORY_SEGMENT_POINTS)
    ]
    return target_ids, segments, warnings


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
