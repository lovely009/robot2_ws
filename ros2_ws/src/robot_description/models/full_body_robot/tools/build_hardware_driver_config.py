#!/usr/bin/env python3
"""Generate calibrated motor_driver.yaml, transmissions.yaml, and coupled_limits.yaml for full_body_robot."""
from __future__ import annotations

import json
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin
import yaml

TOOLS_DIR = Path(__file__).resolve().parent
MODEL = TOOLS_DIR.parent
sys.path.insert(0, str(TOOLS_DIR))
import build_hil_config as hil  # noqa: E402


def compute_direct_cw_signs(urdf_path: Path) -> dict[str, int]:
    """Determine CW actuator_reduction (-1 or +1) for each direct-drive URDF joint.

    URDF joints follow the right-hand rule (CCW when looking down +axis).
    For a motor whose output shaft faces outward along +world_ax, CW = -1.
    """
    model = pin.buildModelFromUrdf(str(urdf_path))
    data = model.createData()
    pin.forwardKinematics(model, data, pin.neutral(model))
    pin.updateFramePlacements(model, data)

    root = ET.parse(urdf_path).getroot()
    signs: dict[str, int] = {}
    for joint in root.findall('joint'):
        if joint.attrib.get('type') == 'fixed':
            continue
        name = joint.attrib['name']
        jid = model.getJointId(name)
        r_j = data.oMi[jid].rotation
        ax_local = np.array([float(x) for x in joint.find('axis').attrib['xyz'].split()], dtype=float)
        ax_world = r_j @ ax_local
        ax_world /= np.linalg.norm(ax_world)

        # Use child link CoM offset from joint origin projected onto world_ax to infer outward shaft direction
        child_name = joint.find('child').attrib['link']
        child_fid = model.getFrameId(child_name, pin.FrameType.BODY)
        com_local = model.inertias[jid].lever
        com_world = data.oMf[child_fid].rotation @ com_local
        proj = float(np.dot(com_world, ax_world))
        if abs(proj) > 5e-3:
            # Output shaft points toward child CoM when proj > 0; looking down outward shaft, CW is opposite +axis -> -1
            signs[name] = -1 if proj > 0 else 1
        else:
            # Concentric pitch/yaw joint: use local axis Z sign (positive Z -> -1, negative Z -> +1)
            signs[name] = -1 if ax_local[2] >= 0 else 1
    return signs


def compute_differential_cw_signs(mjcf_root: ET.Element, motor_specs: dict) -> dict[str, tuple[int, int]]:
    """Determine CW actuator_reduction (-1 or +1) for each motor in the 6 differential groups."""
    joints, sites = hil.zero_geometry(mjcf_root)
    config = json.loads(mjcf_root.find('custom/text[@name="robot_sim_joint_position"]').get('data'))
    topology = yaml.safe_load((MODEL / 'config/motor_topology.yaml').read_text())

    diff_signs: dict[str, tuple[int, int]] = {}
    for gname, group in topology['differential_groups'].items():
        outputs = group['joints']
        numbers = group['motor_numbers']
        g_signs: list[int] = []
        for idx, num in enumerate(numbers):
            m_name = f'motor_{num:02d}'
            m_joint = motor_specs[m_name]['joint']
            if idx == 0 and m_joint == outputs[0]:
                # Direct first motor (e.g. left/right scapula_yaw)
                _, ax, _ = joints[m_joint]
                g_signs.append(-1 if ax[2] >= 0 else 1)
                continue
            # Linkage motor: motor axis points outward from motor origin toward crank ball/flange
            origin, ax, _ = joints[m_joint]
            # Find flange tendon site
            act_el = next(a for a in mjcf_root.findall('actuator/position') if a.get('joint') == m_joint)
            tendon_name = config['flanges'][act_el.get('name')]
            tendon = mjcf_root.find(f'tendon/spatial[@name="{tendon_name}"]')
            ball, _ = sites[tendon[0].get('site')]
            # Vector from motor origin to ball along motor axis
            axial = float(np.dot(ball - origin, ax))
            if abs(axial) > 1e-4:
                g_signs.append(-1 if axial > 0 else 1)
            else:
                g_signs.append(-1 if ax[2] >= 0 else 1)
        diff_signs[gname] = (g_signs[0], g_signs[1])
    return diff_signs


def main() -> None:
    urdf_path = MODEL / 'urdf/full_body_robot.urdf'
    mjcf_path = MODEL / 'mujoco/full_body_robot.xml'
    specs_path = MODEL.parents[1] / 'config/motor.yaml'
    hil_driver = yaml.safe_load((MODEL / 'config/motor_driver.hil.yaml').read_text())['motor_hardware']
    topology = yaml.safe_load((MODEL / 'config/motor_topology.yaml').read_text())

    mjcf_root = ET.parse(mjcf_path).getroot()
    actuators = list(mjcf_root.find('actuator'))
    specs = yaml.safe_load(specs_path.read_text())['motor_types']

    # Build motor_limits & motors dict using CAD types from motor_driver.hil.yaml
    motor_limits = {}
    hil_motors = hil_driver['motors']
    for a in actuators:
        joint = a.get('joint')
        # Match node ID from hil_motors
        m_entry = next(v for v in hil_motors.values() if v['joint'] == joint)
        mtype = m_entry['type']
        params = specs[mtype]['parameters']
        lower, upper = map(float, a.get('ctrlrange').split())
        torque = float(params.get('peak_torque_nm', params['rated_torque_nm']))
        motor_limits[joint] = (lower, upper, float(params['rated_speed_rad_s']), torque)

    # Compute differential transmissions from MJCF
    transmissions, _ = hil.hil_transmissions(MODEL, mjcf_root, actuators, hil_motors, motor_limits)
    direct_signs = compute_direct_cw_signs(urdf_path)
    diff_signs = compute_differential_cw_signs(mjcf_root, hil_motors)

    # Assemble clean motor_driver.yaml (serial_slcan /dev/ttyACM0, MIT position mode effort=0)
    motors_out = {}
    diff_motor_nums = {n for g in topology['differential_groups'].values() for n in g['motor_numbers']}
    for num in range(1, 38):
        key = f'motor_{num:02d}'
        src = hil_motors[key]
        entry = {
            'id': int(src['id']),
            'type': src['type'],
            'bus': 'arm',
            'command': dict(src['command']),
        }
        if num not in diff_motor_nums:
            entry['joint'] = src['joint']
        motors_out[key] = entry

    driver_doc = {
        'motor_hardware': {
            'motor_specs_file': os.path.relpath(specs_path, MODEL / 'config'),
            'transmissions_file': 'transmissions.yaml',
            'coupled_limits_file': 'coupled_limits.yaml',
            'buses': {
                'arm': {
                    'transport': 'serial_slcan',
                    'tty': '/dev/ttyACM0',
                },
            },
            'motors': motors_out,
        }
    }

    # Assemble transmissions.yaml (25 SimpleTransmission + 6 ParallelLinkageTransmission)
    trans_out = {}
    for num in range(1, 38):
        if num in diff_motor_nums:
            continue
        key = f'motor_{num:02d}'
        jname = motors_out[key]['joint']
        sign = direct_signs[jname]
        trans_out[f'direct_{key}'] = {
            'plugin': 'transmission_interface/SimpleTransmission',
            'actuators': [key],
            'joints': [jname],
            'actuator_reduction': [sign],
            'joint_reduction': [1],
            'joint_offset': [0],
        }

    for gname, gdata in transmissions.items():
        s1, s2 = diff_signs[gname]
        trans_out[gname] = {
            'plugin': gdata['plugin'],
            'actuators': list(gdata['actuators']),
            'joints': list(gdata['joints']),
            'actuator_reduction': [s1, s2],
            'joint_reduction': [1, 1],
            'joint_offset': [0, 0],
            'parameters': dict(gdata['parameters']),
        }

    pos_cfg = json.loads(mjcf_root.find('custom/text[@name="robot_sim_joint_position"]').get('data'))
    coupled_limits_doc = {
        'joint_limits': pos_cfg['joint_limits'],
        'coupled_limits': pos_cfg.get('coupled_limits', {}),
    }

    (MODEL / 'config/motor_driver.yaml').write_text(
        '# 真机驱动配置：CW 转向，机械零位对齐 URDF，耦合连杆由模型读取并采用位置模式。\n'
        + yaml.safe_dump(driver_doc, allow_unicode=True, sort_keys=False),
        encoding='utf-8',
    )
    (MODEL / 'config/transmissions.yaml').write_text(
        '# 真机传动配置：25 组直驱 + 6 组空间平行连杆（由称重版模型生成，CW 转向）。\n'
        + yaml.safe_dump({'transmissions': trans_out}, allow_unicode=True, sort_keys=False),
        encoding='utf-8',
    )
    (MODEL / 'config/coupled_limits.yaml').write_text(
        '# 真机耦合工作空间限位（来自称重版三维碰撞最宽松包络）。\n'
        + yaml.safe_dump(coupled_limits_doc, allow_unicode=True, sort_keys=False),
        encoding='utf-8',
    )

    print('Generated motor_driver.yaml, transmissions.yaml, coupled_limits.yaml:')
    for gname, g in trans_out.items():
        print(f'  {gname:18s} | {g["plugin"].split("/")[-1]:27s} | acts={g["actuators"]} | joints={g["joints"]} | cw_red={g["actuator_reduction"]}')


if __name__ == '__main__':
    main()
