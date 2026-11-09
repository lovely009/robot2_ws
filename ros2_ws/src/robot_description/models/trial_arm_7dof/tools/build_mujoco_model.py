#!/usr/bin/env python3
"""生成仅供仿真的 MJCF/URDF 和可重现的几何范围记录。用 MuJoCo venv 执行。"""
import hashlib
import importlib.util
import itertools
import json
import math
import os
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import yaml
import trimesh
from robot_transmission import Axis, Model as TransmissionModel

MODEL = Path(__file__).resolve().parents[1]
SOURCE = MODEL / 'urdf/trial_arm_7dof.urdf'
OUTPUT = MODEL / 'mujoco/trial_arm_7dof.xml'
REPORT = MODEL / 'mujoco/geometry_limits.json'
STEP = math.radians(1)
MARGIN = math.radians(2)
PENETRATION = 0.0001
TANGENT_COS_MAX = .995  # 排除球面与电机圆近切的奇异姿态，仅为仿真几何余量。


def import_model():
    spec = importlib.util.spec_from_file_location('trial_collision_builder', MODEL / 'tools/build_collision_meshes.py')
    collision_builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(collision_builder)
    for mesh in (MODEL / 'meshes').glob('*.STL'):
        assert collision_builder.completed(mesh, MODEL / 'meshes/collision'), f'{mesh.name}: incomplete or stale decomposition'
    root = ET.parse(SOURCE).getroot()
    for joint in root.findall("joint[@type='revolute']"):
        joint.find('limit').attrib.update(lower=str(-math.pi), upper=str(math.pi))
    compiler = ET.SubElement(root, 'mujoco')
    ET.SubElement(compiler, 'compiler', meshdir=str(MODEL / 'meshes'),
                  discardvisual='false', fusestatic='false', balanceinertia='false')
    model = mujoco.MjModel.from_xml_string(ET.tostring(root).decode())
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / 'model.xml'
        mujoco.mj_saveLastXML(str(path), model)
        mjcf = ET.parse(path).getroot()
    # saveLastXML 的默认精度不足以保留来源的小惯量；写回编译后的双精度值。
    def numbers(values):
        return ' '.join(format(float(v), '.17g') for v in values)

    for body in mjcf.find('worldbody').iter('body'):
        index = model.body(body.get('name')).id
        body.set('pos', numbers(model.body_pos[index]))
        body.set('quat', numbers(model.body_quat[index]))
        inertial = body.find('inertial')
        if inertial is not None:
            inertial.attrib.update(mass=format(model.body_mass[index], '.17g'),
                                   pos=numbers(model.body_ipos[index]),
                                   quat=numbers(model.body_iquat[index]),
                                   diaginertia=numbers(model.body_inertia[index]))
    assets = mjcf.find('asset')
    for body in mjcf.find('worldbody').iter('body'):
        for geom in list(body.findall('geom')):
            if geom.get('contype') == '0':
                continue
            body.remove(geom)
            mesh = geom.get('mesh')
            parts = sorted((MODEL / 'meshes/collision').glob(f'{mesh}_*.obj'))
            assert parts, f'{mesh}: run tools/build_collision_meshes.py first'
            for index, part in enumerate(parts):
                if abs(trimesh.load(part, force='mesh').volume) < 1e-12:
                    continue  # 小于1e-12 m³的退化碎片不参与碰撞；清单写入报告。
                name = f'collision_{mesh}_{index}'
                ET.SubElement(assets, 'mesh', name=name, file=str(part))
                attributes = dict(geom.attrib, mesh=name, name=name)
                ET.SubElement(body, 'geom', attributes)
    mjcf.find('compiler').set('fusestatic', 'false')
    model = mujoco.MjModel.from_xml_string(ET.tostring(mjcf).decode())
    return model, mjcf


def collision_pairs(model, data):
    exclusions = {}
    bodies = sorted(set(int(model.geom_bodyid[g]) for g in range(model.ngeom)))
    for ba, bb in itertools.combinations(bodies, 2):
        wa, wb = int(model.body_weldid[ba]), int(model.body_weldid[bb])
        pa = int(model.body_weldid[model.body_parentid[wa]])
        pb = int(model.body_weldid[model.body_parentid[wb]])
        if wa == wb or wa == pb or wb == pa:
            exclusions[(ba, bb)] = 'rigid_or_adjacent'
    for contact in data.contact:
        ba, bb = sorted(int(model.geom_bodyid[g]) for g in contact.geom)
        if contact.dist < -PENETRATION:
            exclusions.setdefault((ba, bb), 'zero_pose_decomposition_overlap')
    return set(exclusions), [dict(body1=model.body(a).name, body2=model.body(b).name,
                                  reason=reason) for (a, b), reason in exclusions.items()]


def shoulder_reachable(model, data, linkage):
    """输入球沿电机圆运动：球面与圆有非切交点才视为可达。"""
    # STEP 球心转换至 Joint_Link2 本地，避免依赖近似的欧拉角反算。
    body = model.body('Joint_Link2').id
    ball = data.xpos[body] + data.xmat[body].reshape(3, 3) @ linkage['local']
    offset = ball - linkage['origin']
    radius, length = linkage['radius'], linkage['length']
    radial = offset - (offset @ linkage['axis']) * linkage['axis']
    denominator = 2 * radius * np.linalg.norm(radial)
    numerator = float(offset @ offset) + radius**2 - length**2
    return denominator > 0 and abs(numerator / denominator) < TANGENT_COS_MAX


def derive_limits(model):
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    pairs, exclusions = collision_pairs(model, data)
    transmission = TransmissionModel.load(str(MODEL / 'config/motor_driver.yaml'))
    body = model.body('Joint_Link2').id
    params = yaml.safe_load((MODEL / 'config/transmissions.yaml').read_text())[
        'transmissions']['shoulder_linkage']['parameters']
    def vector(key):
        return np.fromstring(params[key], sep=' ')
    axis = vector('motor_axis_direction')
    axis /= np.linalg.norm(axis)
    origin = vector('motor_axis_origin')
    radius_vector = vector('motor_ball_zero') - origin
    linkage = dict(local=data.xmat[body].reshape(3, 3).T @ (
        vector('joint_ball_zero') - data.xpos[body]), origin=origin, axis=axis,
        radius=np.linalg.norm(radius_vector - (radius_vector @ axis) * axis),
        length=float(params['rod_length']))

    def valid(q):
        data.qpos[:] = q
        mujoco.mj_forward(model, data)
        if not shoulder_reachable(model, data, linkage):
            return False, 'shoulder_circle_sphere_reachability'
        try:
            transmission.joint_to_actuator({f'Joint_Link{i+1}': Axis(float(v), 0, 0)
                                            for i, v in enumerate(q)})
        except ValueError:
            return False, 'shoulder_transmission_branch'
        for contact in data.contact:
            a, b = sorted(int(model.geom_bodyid[g]) for g in contact.geom)
            if (a, b) not in pairs and contact.dist < -PENETRATION:
                return False, f'{model.body(a).name}/{model.body(b).name}'
        return True, None

    assert valid(np.zeros(7))[0], 'zero pose is not reachable'
    limits, hits = [], []
    for axis in range(7):
        bounds, reasons = [], []
        for direction in (-1, 1):
            last, reason = math.pi, 'scan_cap_180_deg'
            for angle in np.arange(STEP, math.pi + STEP / 2, STEP):
                q = np.zeros(7)
                q[axis] = direction * angle
                ok, obstacle = valid(q)
                if not ok:
                    last, reason = angle - STEP, obstacle
                    break
            bound = max(0., last - MARGIN)

            bounds.append(direction * bound)
            reasons.append(reason)
        assert bounds[1] - bounds[0] >= STEP, f'Joint_Link{axis+1}: no useful range ({reasons})'
        limits.append(bounds)
        hits.append(reasons)
    # 单轴扫描不能保证两轴耦合可达。将肩部矩形同步收缩，验证二维网格。
    shoulder = np.array(limits[:2])
    scale = 1.0
    while True:
        good = True
        for a, b in itertools.product(np.linspace(*shoulder[0] * scale, 31),
                                      np.linspace(*shoulder[1] * scale, 31)):
            q = np.zeros(7)
            q[:2] = a, b
            if not valid(q)[0]:
                good = False
                break
        if good:
            break
        scale *= .95
        assert scale > .01, 'no useful shoulder rectangle'
    limits[:2] = (shoulder * scale).tolist()
    return limits, hits, exclusions, scale


def build():
    model, root = import_model()
    limits, hits, exclusions, scale = derive_limits(model)
    root.find('compiler').set('meshdir', '../meshes')
    for asset in root.find('asset').findall('mesh'):
        if asset.get('name', '').startswith('collision_'):
            asset.set('file', 'collision/' + Path(asset.get('file')).name)
    ET.SubElement(root, 'option', timestep='0.001', integrator='implicitfast', gravity='0 0 -9.81')
    contact = ET.SubElement(root, 'contact')
    for item in exclusions:
        ET.SubElement(contact, 'exclude', body1=item['body1'], body2=item['body2'])
    actuator = ET.SubElement(root, 'actuator')
    specs = yaml.safe_load((MODEL.parents[1] / 'config/motor.yaml').read_text())['motor_types']
    motors = yaml.safe_load((MODEL / 'config/motor_driver.yaml').read_text())['motor_hardware']['motors']
    parameters = {m['joint']: specs[m['type']]['parameters'] for m in motors.values()}
    for index, joint in enumerate(root.find('worldbody').iter('joint')):
        lower, upper = limits[index]
        # 仿真稳定化惯量，非实测转子惯量；不复制进来源或实机 URDF。
        joint.attrib.update(range=f'{lower:.12g} {upper:.12g}', limited='true',
                            armature='0.03', damping='0.1')
        torque = float(parameters[joint.get('name')]['peak_torque_nm'])
        joint.set('actuatorfrcrange', f'{-torque} {torque}')
        ET.SubElement(actuator, 'motor', name=f'actuator_{joint.get("name")}',
                      joint=joint.get('name'), gear='1', ctrllimited='true',
                      ctrlrange=f'{-torque} {torque}',
                      user=str(parameters[joint.get('name')]['rated_speed_rad_s']))
    for geom in root.find('worldbody').iter('geom'):
        if geom.get('contype') != '0':
            geom.attrib.update(group='3', density='0', friction='0.8 0.005 0.0001')
    ET.SubElement(root, 'statistic', center='0 0.08 0.35', extent='0.8')
    OUTPUT.parent.mkdir(exist_ok=True)
    ET.indent(root)
    ET.ElementTree(root).write(OUTPUT, encoding='unicode')
    source = ET.parse(SOURCE).getroot()
    for i, joint in enumerate(source.findall("joint[@type='revolute']")):
        lower, upper = limits[i]
        motor = parameters[joint.get('name')]
        joint.find('limit').attrib.update(lower=f'{lower:.12g}', upper=f'{upper:.12g}',
                                         velocity=str(motor['rated_speed_rad_s']),
                                         effort=str(motor['peak_torque_nm']))
    ET.indent(source)
    ET.ElementTree(source).write(MODEL / 'urdf/trial_arm_7dof.standalone.urdf', encoding='unicode')
    files = [SOURCE, MODEL / 'config/transmissions.yaml', MODEL / 'config/motor_driver.yaml',
             MODEL.parents[1] / 'config/motor.yaml', *sorted((MODEL / 'meshes').glob('*.STL')),
             *sorted((MODEL / 'meshes/collision').glob('*.obj')),
             *sorted((MODEL / 'meshes/collision').glob('*.complete.json'))]
    report = {
        'discarded_collision_fragments': [p.name for p in sorted((MODEL / 'meshes/collision').glob('*.obj'))
            if 'collision/' + p.name not in {a.get('file') for a in root.find('asset')}],
        'software': {'mujoco': mujoco.__version__, 'coacd': '1.0.7', 'vtk': '9.1.0'},
        'scope': 'SIMULATION ONLY; neutral-connected convex-decomposition screening, not mechanical stops',
        'tangent_cos_max': TANGENT_COS_MAX,
        'step_deg': 1, 'inward_margin_deg': 2, 'penetration_threshold_m': PENETRATION,
        'shoulder_grid': 31, 'shoulder_rectangle_scale': scale,
        'limitations': ['CoACD approximate convex parts can still alter contact boundaries.',
                       'Rigid/adjacent pairs and initial convex overlaps are excluded, see exclusions.',
                       'Single-axis sweeps hold other axes at zero; arbitrary multi-axis clearance is not guaranteed.',
                       'The 31x31 shoulder grid is a sampled reachability check, not a continuous proof.',
                       'Unmodeled linkage bodies, cables and actual mechanical stops are not covered.',
                       'A scan cap is not a measured stop; inertia includes earlier documented approximations.'],
        'joints': {f'Joint_Link{i+1}': {'range_rad': bound,
                    'range_deg': np.rad2deg(bound).tolist(),
                    'velocity_rad_s': parameters[f'Joint_Link{i+1}']['rated_speed_rad_s'],
                    'effort_nm': parameters[f'Joint_Link{i+1}']['peak_torque_nm'],
                    'boundary_reasons': hits[i]}
                   for i, bound in enumerate(limits)},
        'exclusions': exclusions,
        'sha256': {os.path.relpath(p, MODEL): hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
    }
    REPORT.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps(report['joints'], indent=2))


if __name__ == '__main__':
    build()
