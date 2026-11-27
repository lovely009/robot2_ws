#!/usr/bin/env python3
"""Scan 2D reachable differential workspaces for full_body_robot and update limits YAML files."""

from collections import deque
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from web_host.motion_simulation import load_mujoco
from robot_transmission.coupled_limits import CoupledLimits

MODEL_DIR = Path(__file__).resolve().parents[1]
MJCF_PATH = MODEL_DIR / 'mujoco/full_body_robot.xml'
SIM_LIMITS_PATH = MODEL_DIR / 'config/simulation_limits.yaml'
PREVIEW_LIMITS_PATH = MODEL_DIR / 'config/motion_preview_limits.yaml'
COUPLED_LIMITS_PATH = MODEL_DIR / 'config/coupled_limits.yaml'

GROUPS = {
    'waist': ['waist_pitch', 'waist_roll'],
    'hips': ['pelvis_roll', 'pelvis_pitch'],
    'left_shoulder': ['left_scapula_yaw', 'left_scapula_roll'],
    'right_shoulder': ['right_scapula_yaw', 'right_scapula_roll'],
    'left_wrist': ['left_wrist_pitch', 'left_wrist_roll'],
    'right_wrist': ['right_wrist_pitch', 'right_wrist_roll'],
}


def _bound_pair(entry):
    if isinstance(entry, dict):
        return float(entry['lower']), float(entry['upper'])
    return float(entry[0]), float(entry[1])


def _connected_zero_component(valid: np.ndarray, r_zero: int, c_zero: int) -> np.ndarray:
    """Keep only the 4-connected component containing (r_zero, c_zero)."""
    rows, cols = valid.shape
    keep = np.zeros_like(valid, dtype=bool)
    if not valid[r_zero, c_zero]:
        raise RuntimeError(f'Zero pose ({r_zero}, {c_zero}) is not reachable')
    queue = deque([(r_zero, c_zero)])
    keep[r_zero, c_zero] = True
    while queue:
        r, c = queue.popleft()
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and valid[nr, nc] and not keep[nr, nc]:
                keep[nr, nc] = True
                queue.append((nr, nc))
    return keep


def main() -> int:
    mujoco = load_mujoco()
    from robot_transmission.mujoco_positions import JointPositionTargets

    root = ET.parse(MJCF_PATH).getroot()
    cfg = json.loads(root.find("custom/text[@name='robot_sim_joint_position']").get('data'))
    mj_model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))
    jpt = JointPositionTargets(mj_model, cfg)
    data = mujoco.MjData(mj_model)
    joint_list = list(jpt.joints)

    raw_limits = yaml.safe_load(SIM_LIMITS_PATH.read_text(encoding='utf-8'))['joint_limits']
    joint_limits = {
        name: {'lower': _bound_pair(bound)[0], 'upper': _bound_pair(bound)[1]}
        for name, bound in raw_limits.items()
    }

    step = math.radians(2.0)
    base_q = np.zeros(len(joint_list), dtype=float)
    coupled_limits = {}

    for gname, (j0, j1) in GROUPS.items():
        idx0, idx1 = joint_list.index(j0), joint_list.index(j1)
        lo0, hi0 = _bound_pair(joint_limits[j0])
        lo1, hi1 = _bound_pair(joint_limits[j1])
        lo0, hi0 = max(-math.pi / 2.0, lo0), min(math.pi / 2.0, hi0)
        lo1, hi1 = max(-math.pi / 2.0, lo1), min(math.pi / 2.0, hi1)

        k0_min, k0_max = math.ceil(lo0 / step), math.floor(hi0 / step)
        k1_min, k1_max = math.ceil(lo1 / step), math.floor(hi1 / step)
        v0 = np.array([k * step for k in range(k0_min, k0_max + 1)], dtype=float)
        v1 = np.array([k * step for k in range(k1_min, k1_max + 1)], dtype=float)
        r_zero, c_zero = -k0_min, -k1_min

        valid = np.zeros((len(v0), len(v1)), dtype=bool)
        for r, a in enumerate(v0):
            for c, b in enumerate(v1):
                q = base_q.copy()
                q[idx0], q[idx1] = a, b
                try:
                    jpt.set_configuration(data, q)
                    valid[r, c] = True
                except ValueError:
                    pass

        keep = _connected_zero_component(valid, r_zero, c_zero)
        rs, cs = np.where(keep)
        r0, r1 = int(rs.min()), int(rs.max())
        c0, c1 = int(cs.min()), int(cs.max())
        trimmed = keep[r0:r1 + 1, c0:c1 + 1]

        coupled_limits[gname] = {
            'joints': [j0, j1],
            'lower': [float(v0[r0]), float(v1[c0])],
            'step': float(step),
            'valid_rows': [''.join('1' if cell else '0' for cell in row) for row in trimmed],
        }
        joint_limits[j0] = {
            'lower': round(float(v0[r0]), 6),
            'upper': round(float(v0[r1]), 6),
        }
        joint_limits[j1] = {
            'lower': round(float(v1[c0]), 6),
            'upper': round(float(v1[c1]), 6),
        }
        print(
            f'{gname:15s} {j0}=[{math.degrees(v0[r0]):+.1f}°, {math.degrees(v0[r1]):+.1f}°] '
            f'{j1}=[{math.degrees(v1[c0]):+.1f}°, {math.degrees(v1[c1]):+.1f}°] '
            f'grid={trimmed.shape[0]}x{trimmed.shape[1]} valid={int(trimmed.sum())}'
        )

    CoupledLimits(coupled_limits, list(joint_limits)).validate(dict.fromkeys(joint_limits, 0.0))

    payload = {'joint_limits': joint_limits, 'coupled_limits': coupled_limits}
    dumped = yaml.safe_dump(payload, sort_keys=False)
    SIM_LIMITS_PATH.write_text(dumped, encoding='utf-8')
    PREVIEW_LIMITS_PATH.write_text(dumped, encoding='utf-8')
    COUPLED_LIMITS_PATH.write_text(
        '# 真机耦合工作空间限位（来自称重版三维碰撞最宽松包络）。\n' + dumped,
        encoding='utf-8',
    )
    print('Updated simulation_limits.yaml, motion_preview_limits.yaml, coupled_limits.yaml')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
