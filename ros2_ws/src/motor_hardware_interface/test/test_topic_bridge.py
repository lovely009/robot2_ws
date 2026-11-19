"""单电机反馈映射及无效测量过滤。"""

import os
from pathlib import Path

import pytest

from motor_hardware_interface.topic_bridge import (
    BridgeError, motion_states_to_joint_state, parse_joint_map,
)

MAP = {1: 'joint_link1', 2: 'joint_link2', 3: 'joint_link3', 4: 'joint_link4'}


def test_bridge_entrypoint_is_executable_for_symlink_install():
    script = Path(__file__).parent.parent / 'scripts' / 'firmware_topic_bridge.py'
    assert os.access(script, os.X_OK)


def test_feedback_follows_motor_ids_and_input_order():
    states = [
        {'motor_id': 2, 'position': .5, 'velocity': .1, 'effort': 1.},
        {'motor_id': 1, 'position': -.2, 'velocity': -.3, 'effort': 2.},
        {'motor_id': 9, 'position': .2, 'velocity': 0., 'effort': 0.},
    ]
    assert motion_states_to_joint_state(states, MAP) == (
        ['joint_link2', 'joint_link1'], [.5, -.2], [.1, -.3], [1., 2.],
    )


@pytest.mark.parametrize('field', ['position', 'velocity', 'effort'])
@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf'), None, '0'])
def test_invalid_measurement_skips_only_its_motor(field, value):
    valid = {'motor_id': 2, 'position': .5, 'velocity': .1, 'effort': 1.}
    invalid = {**valid, 'motor_id': 1, field: value}
    assert motion_states_to_joint_state([invalid, valid], MAP) == (
        ['joint_link2'], [.5], [.1], [1.],
    )


@pytest.mark.parametrize('ids', [[], [1], [1, 4]])
def test_feedback_contains_only_connected_motors(ids):
    states = [
        {'motor_id': motor_id, 'position': .25, 'velocity': 0., 'effort': 0.}
        for motor_id in ids
    ]
    assert motion_states_to_joint_state(states, MAP) == (
        [MAP[motor_id] for motor_id in ids], [.25] * len(ids),
        [0.] * len(ids), [0.] * len(ids),
    )


def test_parse_joint_map():
    assert parse_joint_map([1, 4], ['a', 'b']) == {1: 'a', 4: 'b'}


@pytest.mark.parametrize('ids,names', [
    ([1], ['a', 'b']), ([], []), ([1, 1], ['a', 'b']),
])
def test_parse_joint_map_rejects_invalid_inputs(ids, names):
    with pytest.raises(BridgeError):
        parse_joint_map(ids, names)
