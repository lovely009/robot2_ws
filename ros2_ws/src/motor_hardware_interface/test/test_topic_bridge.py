"""固件话题适配桥的转换与校验（纯函数，不启动任何 ROS 节点）。

这些用例的价值集中在一点：**固件对非法轨迹是静默丢弃的**（无事件、无计数），
所以"错误必须在桥上被讲清楚"这件事只能靠这里钉住。
"""

import math

import pytest

from motor_hardware_interface.topic_bridge import (BridgeError,
                                                  motion_states_to_joint_state,
                                                  parse_joint_map,
                                                  split_trajectory,
                                                  transpose_trajectory)


MAP = {1: 'joint_link1', 2: 'joint_link2', 3: 'joint_link3', 4: 'joint_link4'}
REV = {v: k for k, v in MAP.items()}


def test_uplink_maps_by_motor_id_and_skips_nan():
    states = [
        {'motor_id': 2, 'position': 0.5, 'velocity': 0.1, 'effort': 1.0},
        {'motor_id': 1, 'position': float('nan'), 'velocity': 0.0, 'effort': 0.0},
        {'motor_id': 9, 'position': 0.2, 'velocity': 0.0, 'effort': 0.0},  # 未映射
    ]
    names, pos, vel, eff = motion_states_to_joint_state(states, MAP)
    # 未映射的 9 被忽略；NaN 的 1 被整条跳过（不发 0 骗 rviz）。
    assert names == ['joint_link2']
    assert pos == [0.5] and vel == [0.1] and eff == [1.0]


def test_uplink_all_missing_yields_empty():
    names, *_ = motion_states_to_joint_state([], MAP)
    assert names == []


def test_downlink_transposes_time_major_to_motor_major():
    samples = [
        {'t_ms': 0, 'positions': [0.0, 0.0], 'velocities': None, 'effort': None},
        {'t_ms': 1000, 'positions': [0.6, -0.3], 'velocities': None, 'effort': None},
    ]
    ids, flat, warn = transpose_trajectory(['joint_link1', 'joint_link2'],
                                           samples, REV)
    assert ids == [1, 2]
    # 每个采样点展开成"每台一个点"，时间保持非递减。
    assert [p[:2] for p in flat] == [(0, 1), (0, 2), (1000, 1), (1000, 2)]
    assert flat[2][2] == 0.6 and flat[3][2] == -0.3
    assert flat[0][3] == 0.0 and flat[0][4] == 0.0   # 缺的通道补 0
    assert warn == []


@pytest.mark.parametrize('samples,needle', [
    ([{'t_ms': 0, 'positions': [0.0]},
      {'t_ms': 0, 'positions': [0.1]}], '未严格递增'),
    ([{'t_ms': 100, 'positions': [0.0]},
      {'t_ms': 50, 'positions': [0.1]}], '未严格递增'),
])
def test_downlink_rejects_non_increasing_time(samples, needle):
    with pytest.raises(BridgeError) as e:
        transpose_trajectory(['joint_link1'], samples, REV)
    assert needle in str(e.value)


def test_downlink_rejects_non_finite():
    with pytest.raises(BridgeError) as e:
        transpose_trajectory(['joint_link1'],
                             [{'t_ms': 0, 'positions': [math.inf]}], REV)
    assert '不是有限数' in str(e.value)


def test_downlink_rejects_wrong_vector_length():
    with pytest.raises(BridgeError) as e:
        transpose_trajectory(['joint_link1', 'joint_link2'],
                             [{'t_ms': 0, 'positions': [0.0]}], REV)
    assert '不符' in str(e.value)


def test_downlink_rejects_unknown_joint_and_names_it():
    with pytest.raises(BridgeError) as e:
        transpose_trajectory(['joint_link1', 'joint_bogus'],
                             [{'t_ms': 0, 'positions': [0.0, 0.0]}], REV)
    assert 'joint_bogus' in str(e.value)


def test_downlink_rejects_point_overflow():
    names = ['joint_link1', 'joint_link2', 'joint_link3', 'joint_link4']
    samples = [{'t_ms': t * 10, 'positions': [0.0] * 4} for t in range(65)]
    with pytest.raises(BridgeError) as e:
        transpose_trajectory(names, samples, REV)   # 4 × 65 = 260 > 256
    assert '超过固件上限' in str(e.value)


def test_downlink_full_trajectory_is_a_single_message():
    """满容量轨迹必须是**一条**消息（单条消息容量 = 总量上限）。

    拆成两条 128 点不会多送一个点，只会把同一份数据变成两条背靠背消息，
    而 MCU 的 XRCE 输入流只有 16 × 512 B 的分片窗口：一条 256 点（≈5.2 KB）
    放得下，两条 128 点更容易让分片落窗外 → 固件整组拼不齐（静默）→
    标准接口上表现为"发了没反应"。
    """
    names = ['joint_link1', 'joint_link2', 'joint_link3', 'joint_link4']
    samples = [
        {'t_ms': t * 10, 'positions': [0.0] * 4}
        for t in range(64)  # 256 total points
    ]
    ids, segments, warn = split_trajectory(names, samples, REV)
    assert ids == [1, 2, 3, 4]
    assert len(segments) == 1
    assert len(segments[0]) == 256
    assert segments[0][0][0] == 0
    assert segments[0][-1][0] == 630   # 时间仍是整条绝对时间线，未按段重置
    assert warn == []
    # 超出总量的轨迹在 transpose 阶段就被拒，不会退化成多条独立轨迹。
    overflow = [{'t_ms': t * 10, 'positions': [0.0] * 4} for t in range(65)]
    with pytest.raises(BridgeError):
        split_trajectory(names, overflow, REV)


def test_downlink_warns_out_of_range_position():
    _ids, _flat, warn = transpose_trajectory(
        ['joint_link1'], [{'t_ms': 0, 'positions': [4.0]}], REV)
    assert warn and '超出' in warn[0]


def test_parse_joint_map_validates():
    assert parse_joint_map([1, 2], ['a', 'b']) == {1: 'a', 2: 'b'}
    for ids, names in (([1], ['a', 'b']), ([], []), ([1, 1], ['a', 'b'])):
        with pytest.raises(BridgeError):
            parse_joint_map(ids, names)
