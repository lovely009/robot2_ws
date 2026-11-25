from collections import Counter
from pathlib import Path
from types import SimpleNamespace

import pytest

from motor_hardware_interface.motor_param_sync import (
    FIELD_INDEX,
    build_param_set_batches,
    build_system_command_request,
    diff_device_info,
    load_motor_params,
)


def _real_yaml():
    return (
        Path(__file__).resolve().parents[2]
        / 'robot_description/models/full_body_robot/config/motor_params.yaml'
    )


def _entry_from_cfg(mid, cfg):
    return SimpleNamespace(
        motor_id=mid,
        dev_pos_min=cfg['pos_min'],
        dev_pos_max=cfg['pos_max'],
        dev_vel_limit=cfg['vel_limit'],
        dev_torque_limit=cfg['torque_limit'],
        dev_safety=0.0 if cfg['limit_protect'] else 1.0,
        dev_direction=float(cfg['direction']),
        dev_t_max=cfg['t_max'],
        dev_node_id=float(cfg['node_id']),
    )


def test_load_real_37_motor_params_yaml():
    import xml.etree.ElementTree as ET
    resolved = load_motor_params(_real_yaml())
    assert len(resolved) == 37
    counts = Counter(v['template'] for v in resolved.values())
    assert counts == {'vaxor_2550': 6, 'cerb_r58_101': 15, 'cerb_r48_101': 16}
    assert resolved[13]['template'] == 'vaxor_2550'
    assert resolved[1]['template'] == 'cerb_r58_101'
    assert resolved[6]['template'] == 'cerb_r48_101'
    urdf = _real_yaml().parents[1] / 'urdf/full_body_robot.urdf'
    urdf_joints = {
        j.get('name') for j in ET.parse(urdf).getroot().findall('joint')
        if j.get('type') != 'fixed'
    }
    assert {v['joint'] for v in resolved.values()} == urdf_joints


def test_overrides_and_zero_diff(tmp_path):
    yaml_file = tmp_path / 'params.yaml'
    lines = [
        'schema_version: 1',
        'templates:',
        '  t1: {pos_min: -3.14, pos_max: 3.14, vel_limit: 3.14, torque_limit: 12.0, limit_protect: true, direction: 0, t_max: 80.0}',
        'motors:',
    ]
    for mid in range(1, 38):
        ov = '{direction: 1}' if mid == 5 else '{}'
        lines.append(f'  - {{id: {mid}, joint: j{mid}, template: t1, overrides: {ov}}}')
    yaml_file.write_text('\n'.join(lines), encoding='utf-8')

    resolved = load_motor_params(yaml_file)
    assert resolved[5]['direction'] == 1
    info = SimpleNamespace(params=[_entry_from_cfg(mid, cfg) for mid, cfg in resolved.items()])
    assert diff_device_info(resolved, info) == []


def test_diff_and_batch_chunking_and_sync_w25():
    resolved = load_motor_params(_real_yaml())
    entries = [_entry_from_cfg(mid, cfg) for mid, cfg in resolved.items()]
    for idx in range(20):
        entries[idx].dev_vel_limit = 9.9
        if idx == 0:
            entries[idx].dev_direction = 1.0
    info = SimpleNamespace(params=entries)
    diffs = diff_device_info(resolved, info)
    assert len(diffs) == 21

    batches = build_param_set_batches(resolved, diffs, session_id=42, start_request_id=10, max_batch=16)
    assert len(batches) == 2
    assert batches[0].count == 16 and batches[0].request_id == 10
    assert batches[1].count == 5 and batches[1].request_id == 11
    assert batches[0].targets[0].motor_id == 1 and batches[0].targets[0].field == FIELD_INDEX['vel_limit']
    assert batches[0].targets[1].motor_id == 1 and batches[0].targets[1].field == FIELD_INDEX['direction']

    w25_req = build_system_command_request(6, session_id=42, request_id=99)
    assert w25_req.kind == w25_req.CONFIG_KIND_SYSTEM_COMMAND
    assert w25_req.command == 6
    assert w25_req.motor_id == 0xFFFF
