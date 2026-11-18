#!/usr/bin/env python3
"""从 CAD 电机记录与已装配 MJCF 生成 HIL 位置电机拓扑。"""
import argparse
import csv
import hashlib
import copy
import json
import math
import os
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import yaml
import numpy as np

MODEL = Path(__file__).resolve().parents[1]


def zero_geometry(root):
    """Read the generated MJCF zero assembly without requiring a MuJoCo runtime."""
    if root.find('compiler').get('angle') != 'radian' or root.findall('include'):
        raise ValueError('HIL geometry requires an expanded radian MJCF')
    joints, sites = {}, {}

    def vector(element, name, default):
        value = np.fromstring(element.get(name, default), sep=' ')
        if value.shape != (3,) or not np.isfinite(value).all():
            raise ValueError(f'invalid MJCF {name}')
        return value

    def visit(body, rotation, origin, parents):
        if any(key in body.attrib for key in ('euler', 'axisangle', 'xyaxes', 'zaxis')):
            raise ValueError('HIL geometry requires explicit quaternion body frames')
        quat = np.fromstring(body.get('quat', '1 0 0 0'), sep=' ')
        if quat.shape != (4,) or not np.isfinite(quat).all() or np.linalg.norm(quat) < 1e-12:
            raise ValueError('invalid MJCF quaternion')
        w, x, y, z = quat / np.linalg.norm(quat)
        local = np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                          [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                          [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])
        origin = origin + rotation @ vector(body, 'pos', '0 0 0')
        rotation = rotation @ local
        owned = []
        for joint in body.findall('joint'):
            if joint.get('type', 'hinge') != 'hinge' or float(joint.get('ref', '0')) != 0:
                raise ValueError('HIL geometry requires zero-reference hinges')
            name = joint.get('name')
            axis = rotation @ vector(joint, 'axis', '0 0 1')
            axis /= np.linalg.norm(axis)
            joints[name] = (origin + rotation @ vector(joint, 'pos', '0 0 0'), axis, tuple(parents))
            owned.append(name)
        for site in body.findall('site'):
            sites[site.get('name')] = (origin + rotation @ vector(site, 'pos', '0 0 0'), tuple(parents + owned))
        for child in body.findall('body'):
            visit(child, rotation, origin, parents + owned)

    for body in root.findall('worldbody/body'):
        visit(body, np.eye(3), np.zeros(3), [])
    return joints, sites


def hil_transmissions(model, root, actuators, motors, motor_limits):
    config = json.loads(root.find('custom/text[@name="robot_sim_joint_position"]').get('data'))
    topology = yaml.safe_load((model / 'config/model_motor_topology.yaml').read_text())
    joints, sites = zero_geometry(root)
    transmissions, output_limits = {}, {}
    fmt = lambda values: ' '.join(format(float(v), '.17g') for v in values)
    for name, group in topology['differential_groups'].items():
        outputs = group['joints']
        inputs = [f'motor_{number:02}' for number in group['motor_numbers']]
        direct = motors[inputs[0]]['joint'] == outputs[0]
        if outputs[0] not in joints[outputs[1]][2]:
            raise ValueError(f'{name}: output hinges must be serial in topology order')
        parameters = {'direct_first': '1' if direct else '0'}
        for i, output_name in enumerate(outputs, 1):
            origin, axis, _ = joints[output_name]
            parameters[f'joint{i}_origin'] = fmt(origin)
            parameters[f'joint{i}_axis'] = fmt(axis)
        for i, (input_name, number) in enumerate(zip(inputs, group['motor_numbers']), 1):
            actuator = actuators[number-1]
            input_joint = motors[input_name]['joint']
            parameters[f'motor{i}_bounds'] = actuator.get('ctrlrange')
            if direct and i == 1:
                continue
            tendon_name = config['flanges'][actuator.get('name')]
            tendon = root.find(f'tendon/spatial[@name="{tendon_name}"]')
            if tendon is None or len(tendon) != 2 or any(s.tag != 'site' for s in tendon):
                raise ValueError(f'{name}: linkage must have exactly two ball sites')
            ball, ball_parents = sites[tendon[0].get('site')]
            platform, platform_parents = sites[tendon[1].get('site')]
            origin, axis, input_parents = joints[input_joint]
            if (not ball_parents or ball_parents[-1] != input_joint
                    or outputs[1] not in platform_parents
                    or any(output in input_parents for output in outputs)):
                raise ValueError(f'{name}: unsupported motor/platform topology')
            for key, value in [('origin', origin), ('axis', axis), ('ball', ball), ('platform', platform)]:
                parameters[f'motor{i}_{key}'] = fmt(value)
            parameters[f'motor{i}_length'] = fmt([np.linalg.norm(ball-platform)])
        transmissions[name] = {'plugin': 'robot_transmission/ParallelLinkageTransmission',
                               'actuators': inputs, 'joints': outputs, 'parameters': parameters}
        # Nominal output P/V/E envelope; the transmission and motor interfaces
        # still enforce actual input geometry and each motor's own limits.
        speed = min(motor_limits[motors[a]['joint']][2] for a in inputs)
        torque = min(motor_limits[motors[a]['joint']][3] for a in inputs)
        for output_name in outputs:
            if output_name not in motor_limits:
                output_limits[output_name] = (*config['joint_limits'][output_name], speed, torque)
    return transmissions, output_limits


def cad_motor_types(model):
    source = next((model / 'cad/urdf').glob('*.csv'))
    with source.open(encoding='utf-8-sig', newline='') as stream:
        rows = {r['Link Name']: r['SW Components'] for r in csv.DictReader(stream)}
    # 电机固定部分所在 CAD link，不是输出轴的 child link。
    groups = {'base_link': [1, 4, 5], 'Joint_Link1': [2, 3],
              'Joint_Link2': [6, 7, 15, 16]}
    for suffix, start in [('', 8), ('r', 17)]:
        for offset in range(5):
            groups[f'Joint_link3.{offset + 2}{suffix}'] = [start + offset]
        groups[f'Joint_link3.7{suffix}'] = [start + 5, start + 6]
    for suffix, start in [('', 24), ('r', 31)]:
        groups[f'Joint_link5.1{suffix}'] = [start]
        groups[f'Joint_link5.2{suffix}'] = [start + 1, start + 2]
        for offset in range(3, 7):
            groups[f'Joint_link5.{offset}{suffix}'] = [start + offset]
    result = {}
    evidence = {}
    for link, ids in groups.items():
        parts = rows[link].split(';')
        types = set()
        for part in parts:
            leaf = part.rsplit('/', 1)[-1]
            if re.search(r'CE-RB-R58|巨蟹58电机', leaf):
                types.add('cerb_r58_101')
            if re.search(r'CE-RB-R48|巨蟹48电机', leaf):
                types.add('cerb_r48_101')
            if 'X25S-BZ153050' in part:
                types.add('vaxor_2550')
        if len(types) != 1:
            raise ValueError(f'{link}: CAD 电机型号不唯一或缺失：{types}')
        for node in ids:
            result[node] = next(iter(types))
            evidence[node] = {'cad_link': link, 'type': result[node]}
    return result, {'file': str(source.relative_to(model)),
                    'sha256': hashlib.sha256(source.read_bytes()).hexdigest(), 'motors': evidence}


def control_urdf(model, motor_limits):
    """HIL 输入轴：模型位置范围与电机输出轴规格，几何/零位保持同源。"""
    robot = ET.parse(model / 'urdf/full_body_robot.standalone.urdf').getroot()
    joints = robot.findall('joint')
    by_name = {joint.get('name'): joint for joint in joints}
    if len(by_name) != len(joints):
        raise ValueError('HIL URDF 关节名重复')
    for name, values in motor_limits.items():
        joint = by_name.get(name)
        if (joint is None or joint.get('type') not in ('revolute', 'continuous')
                or joint.find('mimic') is not None):
            raise ValueError(f'{name}: HIL 电机输入必须是独立转动关节')
        lower, upper, speed, torque = values
        limit = joint.find('limit')
        if joint.get('type') == 'revolute':
            try:
                original = float(limit.get('lower')), float(limit.get('upper'))
                if not all(math.isfinite(v) for v in original) or original[0] >= original[1]:
                    raise ValueError()
            except (AttributeError, TypeError, ValueError) as error:
                raise ValueError(f'{name}: 模型位置限位无效') from error
            lower, upper = max(lower, original[0]), min(upper, original[1])
        if (not all(math.isfinite(v) for v in (lower, upper, speed, torque))
                or lower >= upper or speed <= 0 or torque <= 0):
            raise ValueError(f'{name}: 模型范围或电机速度/力矩规格无效')
        joint.set('type', 'revolute')
        if limit is None:
            limit = ET.SubElement(joint, 'limit')
        limit.attrib.update(lower=str(lower), upper=str(upper), velocity=str(speed), effort=str(torque))
    robot.insert(0, ET.Comment(' HIL control: model position bounds; motor output-shaft speed and effort specs. '))
    ET.indent(robot, space='  ')
    return ET.tostring(robot, encoding='unicode') + '\n'


def build(model, output, motor_types=None):
    model, output = Path(model), Path(output)
    inferred, evidence = cad_motor_types(model)
    motor_types = inferred if motor_types is None else motor_types
    specs_path = model.parents[1] / 'config/motor.yaml'
    specs = yaml.safe_load(specs_path.read_text())['motor_types']
    if set(motor_types) != set(range(1, 38)) or any(t not in specs for t in motor_types.values()):
        raise ValueError('HIL 需要 CAN ID 1–37 的完整已知型号表')
    root = ET.parse(model / 'mujoco/full_body_robot.xml').getroot()
    actuators = root.find('actuator')
    if len(actuators) != 37 or any(a.tag != 'position' for a in actuators):
        raise ValueError('源模型必须有 37 个位置输入轴；重新核对电机编号')
    by_name = {a.get('name'): a for a in actuators}
    mechanism_inputs = {2: 'waist_flange_plus_x', 3: 'waist_flange_minus_x',
                        4: 'hips_flange_plus_x', 5: 'hips_flange_minus_x',
                        6: 'left_shoulder_axis_1_direct', 7: 'left_shoulder_flange_linkage',
                        13: 'left_wrist_flange_proximal', 14: 'left_wrist_flange_distal',
                        15: 'right_shoulder_axis_1_direct', 16: 'right_shoulder_flange_linkage',
                        22: 'right_wrist_flange_proximal', 23: 'right_wrist_flange_distal'}
    expected_names = [mechanism_inputs.get(node, f'motor_{node:02}') for node in range(1, 38)]
    if set(by_name) != set(expected_names):
        raise ValueError('源模型输入轴名称有变，需要重新核对 CAN ID 对应关系')
    actuators[:] = [by_name[name] for name in expected_names]
    motors, mappings, profiles, motor_limits = {}, [], {}, {}
    for node, actuator in enumerate(list(actuators), 1):
        motor_type = motor_types[node]
        parameters = specs[motor_type]['parameters']
        vaxor = specs[motor_type]['protocol'] == 'vaxor'
        # 保留型号库字段供协议配置使用；MuJoCo 位置执行器参数保持原样。
        torque = float(parameters.get('peak_torque_nm', parameters['rated_torque_nm']))
        lower, upper = map(float, actuator.get('ctrlrange').split())
        name, joint = f'motor_{node:02}', actuator.get('joint')
        motor_limits[joint] = (lower, upper, float(parameters['rated_speed_rad_s']), torque)
        command = {'kp': 0.008, 'kd': 1.0} if vaxor else {'kp': 30.0, 'kd': 1.0}
        motors[name] = {'id': node, 'joint': joint, 'type': motor_type, 'bus': 'arm', 'command': command}
        profile_name = f'{motor_type}_{node:02}'
        plant = {'kp': command['kp'], 'kd': command['kd']}
        if vaxor:
            scale = parameters['torque_constant_nm_per_amp'] * parameters['gear_ratio']**2 * parameters['gear_efficiency']
            plant = {'kp': command['kp'] * scale, 'kd': command['kd'] * scale,
                     'current_limit_a': torque / (parameters['torque_constant_nm_per_amp'] * parameters['gear_ratio'] * parameters['gear_efficiency'])}
        profiles[profile_name] = {'hard_lower_deg': math.degrees(lower), 'hard_upper_deg': math.degrees(upper),
                                 'command_lower_deg': math.degrees(lower), 'command_upper_deg': math.degrees(upper),
                                 'velocity_rad_s': parameters['rated_speed_rad_s'], 'effort_nm': torque, 'plant': plant}
        mappings.append({'motor': name, 'joint': joint, 'profile': profile_name})
    transmissions, output_limits = hil_transmissions(model, root, list(actuators), motors, motor_limits)
    control_description = control_urdf(model, motor_limits | output_limits)
    control_nodes = {joint.get('name'): joint for joint in ET.fromstring(control_description).findall('joint')}
    for mapping in mappings:
        limit = control_nodes[mapping['joint']].find('limit')
        profile = profiles[mapping['profile']]
        # hard_* 保留 MJCF 输入范围，command_* 与控制 URDF 的模型交集一致。
        profile['command_lower_deg'] = math.degrees(float(limit.get('lower')))
        profile['command_upper_deg'] = math.degrees(float(limit.get('upper')))
    (output / 'urdf').mkdir(parents=True, exist_ok=True)
    (output / 'urdf/full_body_robot.hil.urdf').write_text(control_description)
    (output / 'config').mkdir(parents=True, exist_ok=True)
    driver = {'motor_hardware': {'motor_specs_file': os.path.relpath(specs_path, output / 'config'),
                                'buses': {'arm': {'transport': 'socketcan', 'interface': 'can0'}}, 'motors': motors}}
    host_driver = copy.deepcopy(driver)
    host_driver['motor_hardware'].update(transmissions_file='transmissions.hil.yaml',
                                        coupled_limits_file='coupled_limits.hil.yaml', hil_input_fallback=True)
    position_config = json.loads(root.find('custom/text[@name="robot_sim_joint_position"]').get('data'))
    coupled_limits = {key: position_config[key] for key in ('joint_limits', 'coupled_limits')}
    topology = {'simulation': {'model': 'full_body_robot', 'mujoco_model': 'full_body_robot.xml', 'control_mode': 'position',
                              'cad_motor_evidence': evidence, 'timing': {'control_hz': 100, 'vaxor_feedback_hz': 50, 'command_timeout_ms': 150},
                              'driver_timing': {'feedback_timeout_s': 0.5}, 'profiles': profiles, 'motors': mappings}}
    for filename, document in [('motor_driver.simulation.yaml', driver), ('simulation_topology.yaml', topology),
                               ('motor_driver.hil.yaml', host_driver),
                               ('coupled_limits.hil.yaml', coupled_limits),
                               ('transmissions.hil.yaml', {'transmissions': transmissions})]:
        (output / 'config' / filename).write_text('# 仅供隔离总线 HIL；型号取自 CAD，输入轴取自 MJCF，不用于真机。\n' + yaml.safe_dump(document, allow_unicode=True, sort_keys=False))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=MODEL)
    args = parser.parse_args()
    build(MODEL, args.output)
