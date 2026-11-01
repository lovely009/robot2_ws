"""只读固件反馈桥的转换与映射校验，不启动 ROS 节点。"""

import os
from pathlib import Path

import pytest

from motor_hardware_interface.topic_bridge import (BridgeError,
                                                  motion_states_to_joint_state,
                                                  parse_joint_map)


MAP = {1: 'joint_link1', 2: 'joint_link2', 3: 'joint_link3', 4: 'joint_link4'}


def test_bridge_entrypoint_is_executable_for_symlink_install() -> None:
    script = Path(__file__).parent.parent / 'scripts' / 'firmware_topic_bridge.py'
    assert os.access(script, os.X_OK), 'symlink-install 保留源文件权限，桥脚本必须可执行'


def test_node_consumes_decoded_facade_topics_only() -> None:
    script = Path(__file__).parent.parent / 'scripts' / 'firmware_topic_bridge.py'
    source = script.read_text(encoding='utf-8')
    assert "'/motor/motion_states'" in source
    assert "'/motor/status_array'" in source
    assert "'/motor/control_states'" in source
    assert 'micro_ros_up/' not in source


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


@pytest.mark.parametrize('ids', [[1], [1, 4]])
def test_uplink_publishes_only_the_connected_subset(ids: list[int]) -> None:
    states = [{'motor_id': i, 'position': 0.25, 'velocity': 0.0, 'effort': 0.0}
              for i in ids]
    names, positions, _, _ = motion_states_to_joint_state(states, MAP)
    assert names == (['joint_link1'] if ids == [1] else ['joint_link1', 'joint_link4'])
    assert positions == [0.25] * len(ids)


def test_parse_joint_map_validates():
    assert parse_joint_map([1, 2], ['a', 'b']) == {1: 'a', 2: 'b'}
    for ids, names in (([1], ['a', 'b']), ([], []), ([1, 1], ['a', 'b'])):
        with pytest.raises(BridgeError):
            parse_joint_map(ids, names)


@pytest.mark.parametrize('lifecycle,online,fault,expected', [
    (3, True, 0, True), (4, True, 0, True),
    (3, False, 0, False), (3, True, 6, False),
    (2, True, 0, False), (5, True, 0, False), (6, True, 0, False),
])
def test_first_target_health_accepts_ready_before_active(
    lifecycle: int, online: bool, fault: int, expected: bool,
) -> None:
    from motor_hardware_interface.topic_bridge import motor_accepts_joint_target
    assert motor_accepts_joint_target(online, lifecycle, fault) is expected
