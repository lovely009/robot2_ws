#!/usr/bin/env python3
"""Build weighed 27.2 kg MuJoCo MJCF with 6 closed-loop ball-joint linkages and standing display URDF."""
from __future__ import annotations

from collections import defaultdict
from copy import deepcopy
import json
import math
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
import vtk
import yaml

MODEL = Path(__file__).resolve().parents[1]
SOURCE_URDF = MODEL / 'urdf/full_body_robot.urdf'
TOPOLOGY_YAML = MODEL / 'config/motor_topology.yaml'
PARAMS_YAML = MODEL / 'config/motor_params.yaml'
LIMITS_YAML = MODEL / 'config/simulation_limits.yaml'
OUT_MJCF = MODEL / 'mujoco/full_body_robot.xml'
OUT_PHYSICS = MODEL / 'mujoco/full_body_robot.physics.xml'
OUT_STANDALONE = MODEL / 'urdf/full_body_robot.standalone.urdf'
OUT_STANDING = MODEL / 'urdf/full_body_robot.standing.urdf'

LOCKED_SOLVER_RANGE = 1e-9

# Map differential XML joint names -> new URDF joint names
DIFF_TO_URDF_JOINTS = {
    'waist': {'BN_Spine_pitch': 'waist_pitch', 'BN_Spine_roll': 'waist_roll'},
    'hips': {'BN_hips_roll': 'pelvis_roll', 'BN_hips_pitch': 'pelvis_pitch'},
    'left_shoulder': {'BN_LeftShoulder_1': 'left_scapula_yaw', 'BN_LeftShoulder_2': 'left_scapula_roll'},
    'right_shoulder': {'BN_RightShoulder_1': 'right_scapula_yaw', 'BN_RightShoulder_2': 'right_scapula_roll'},
    'left_wrist': {'BN_LeftHand_roll': 'left_wrist_roll', 'BN_LeftHand_pitch': 'left_wrist_pitch'},
    'right_wrist': {'BN_RightHand_roll': 'right_wrist_roll', 'BN_RightHand_pitch': 'right_wrist_pitch'},
}

CAD_MESH_FILES = {
    'waist_plus': ('waist_plus_flange.STL', 'waist_plus_rod.STL', 'waist_plus_platform.STL'),
    'waist_minus': ('waist_minus_flange.STL', 'waist_minus_rod.STL', 'waist_minus_platform.STL'),
    'hips_plus': ('hips_plus_flange.STL', 'hips_plus_rod.STL', 'hips_plus_platform.STL'),
    'hips_minus': ('hips_minus_flange.STL', 'hips_minus_rod.STL', 'hips_minus_platform.STL'),
    'left_shoulder_flange_linkage': (
        'left_shoulder_flange_linkage_flange.STL',
        'left_shoulder_flange_linkage_rod.STL',
        'left_shoulder_flange_linkage_platform.STL',
    ),
    'right_shoulder_flange_linkage': (
        'right_shoulder_flange_linkage_flange.STL',
        'right_shoulder_flange_linkage_rod.STL',
        'right_shoulder_flange_linkage_platform.STL',
    ),
    'left_wrist_flange_proximal': (
        'left_wrist_flange_proximal_flange.STL',
        'left_wrist_flange_proximal_rod.STL',
        'left_wrist_flange_proximal_platform.STL',
    ),
    'left_wrist_flange_distal': (
        'left_wrist_flange_distal_flange.STL',
        'left_wrist_flange_distal_rod.STL',
        'left_wrist_flange_distal_platform.STL',
    ),
    'right_wrist_flange_proximal': (
        'right_wrist_flange_proximal_flange.STL',
        'right_wrist_flange_proximal_rod.STL',
        'right_wrist_flange_proximal_platform.STL',
    ),
    'right_wrist_flange_distal': (
        'right_wrist_flange_distal_flange.STL',
        'right_wrist_flange_distal_rod.STL',
        'right_wrist_flange_distal_platform.STL',
    ),
}

HIPS_CAD_BALLS = {
    'plus': (
        np.array([0.075922944, 0.023653986, 0.028]),
        np.array([0.10532156, 0.081621874, 0.027332679]),
        np.array([-0.01128057, 0.05793355, 0.99825671]),
    ),
    'minus': (
        np.array([-0.063494767, 0.022800067, 0.028]),
        np.array([-0.073238621, 0.08660591, 0.020327214]),
        np.array([0.16205043, 0.13625783, 0.97732976]),
    ),
}
HIPS_CROSSBEAM_NORMAL = np.array([2.19096428e-7, 0.0361379527, 0.999346811])


def numbers(values) -> str:
    return ' '.join(f'{float(v):.9g}' for v in values)


def urdf_rpy(rotation: Rotation) -> np.ndarray:
    return rotation.as_euler('xyz', degrees=False)


def pose_attributes(position, rotation: Rotation) -> dict[str, str]:
    q = rotation.as_quat()  # x, y, z, w
    return {'pos': numbers(position), 'quat': numbers([q[3], q[0], q[1], q[2]])}


def parse_origin(el) -> tuple[np.ndarray, Rotation]:
    if el is None:
        return np.zeros(3), Rotation.identity()
    xyz = np.fromstring(el.get('xyz', '0 0 0'), sep=' ')
    rpy = np.fromstring(el.get('rpy', '0 0 0'), sep=' ')
    return xyz, Rotation.from_euler('xyz', rpy, degrees=False)


def rod_frame(direction: np.ndarray, hint: np.ndarray) -> Rotation:
    z = direction / np.linalg.norm(direction)
    x = hint - np.dot(hint, z) * z
    if np.linalg.norm(x) < 1e-6:
        x = np.array([0.0, 1.0, 0.0]) - z[1] * z
    x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    return Rotation.from_matrix(np.column_stack([x, y, z]))


def decimate_for_mujoco(src_stl: Path, dst_stl: Path, target_faces: int = 120_000) -> None:
    dst_stl.parent.mkdir(parents=True, exist_ok=True)
    reader = vtk.vtkSTLReader()
    reader.SetFileName(str(src_stl))
    reader.Update()
    poly = reader.GetOutput()
    n_cells = poly.GetNumberOfCells()
    if n_cells <= 180_000:
        shutil.copy2(src_stl, dst_stl)
        return
    reduction = max(0.0, min(0.95, 1.0 - (target_faces / float(n_cells))))
    dec = vtk.vtkQuadricDecimation()
    dec.SetInputData(poly)
    dec.SetTargetReduction(reduction)
    dec.Update()
    writer = vtk.vtkSTLWriter()
    writer.SetFileName(str(dst_stl))
    writer.SetInputConnection(dec.GetOutputPort())
    writer.SetFileTypeToBinary()
    writer.Write()


def add_visual_and_inertia(body_el, link_el, assets, added_meshes: set[str]):
    inertial = link_el.find('inertial')
    if inertial is not None:
        pos, rot = parse_origin(inertial.find('origin'))
        mass = inertial.find('mass').get('value')
        inert = inertial.find('inertia').attrib
        # Express inertia in body frame if inertial origin is rotated, or use fullinertia
        I_mat = np.array([
            [float(inert['ixx']), float(inert['ixy']), float(inert['ixz'])],
            [float(inert['ixy']), float(inert['iyy']), float(inert['iyz'])],
            [float(inert['ixz']), float(inert['iyz']), float(inert['izz'])],
        ])
        R = rot.as_matrix()
        I_body = R @ I_mat @ R.T
        # fullinertia order in MuJoCo: ixx, iyy, izz, ixy, ixz, iyz
        full = [I_body[0, 0], I_body[1, 1], I_body[2, 2], I_body[0, 1], I_body[0, 2], I_body[1, 2]]
        ET.SubElement(body_el, 'inertial', pos=numbers(pos), mass=mass, fullinertia=numbers(full))

    for idx, vis in enumerate(link_el.findall('visual')):
        mesh_el = vis.find('geometry/mesh')
        if mesh_el is None:
            continue
        fn = mesh_el.get('filename')
        rel = fn.split('models/full_body_robot/')[-1]
        stl_name = Path(rel).name
        mj_rel = f'mujoco/visual/native/{stl_name}'
        mname = f'{link_el.get("name")}_vis_{idx}'
        if mname not in added_meshes:
            decimate_for_mujoco(MODEL / rel, MODEL / mj_rel)
            ET.SubElement(assets, 'mesh', name=mname, file=mj_rel)
            added_meshes.add(mname)
        pos, rot = parse_origin(vis.find('origin'))
        ET.SubElement(body_el, 'geom', name=mname, type='mesh', mesh=mname,
                      rgba='0.82 0.85 0.88 1', **pose_attributes(pos, rot))


def align_module_to_urdf(name: str, group: dict, urdf_joints: dict, frames: dict):
    path = MODEL / f'mujoco/{name}_differential.xml'
    module_xml = ET.parse(path).getroot()
    mod_model = mujoco.MjModel.from_xml_path(str(path))
    mod_data = mujoco.MjData(mod_model)
    mujoco.mj_forward(mod_model, mod_data)

    ref_link = urdf_joints[group['joints'][0]].find('parent').get('link')
    out_link = urdf_joints[group['joints'][-1]].find('child').get('link')
    ref_pos, ref_rot = frames[ref_link]
    out_pos, out_rot = frames[out_link]

    # Collect old vs new joint positions & axes in ref_link frame
    jmap = DIFF_TO_URDF_JOINTS[name]
    old_axes, new_axes = [], []
    old_anchor = np.zeros(3)
    new_anchor = np.zeros(3)
    for old_jn, new_jn in jmap.items():
        mj = mod_model.joint(old_jn)
        old_pos = mod_data.xanchor[mj.id].copy()
        old_ax = mod_data.xaxis[mj.id].copy()
        # New joint in world -> ref_link frame
        child_link = urdf_joints[new_jn].find('child').get('link')
        c_pos, c_rot = frames[child_link]
        ax_local = np.fromstring(urdf_joints[new_jn].find('axis').get('xyz'), sep=' ')
        ax_local = ax_local / np.linalg.norm(ax_local)
        new_pos_ref = ref_rot.inv().apply(c_pos - ref_pos)
        new_ax_ref = ref_rot.inv().apply(c_rot.apply(ax_local))
        old_axes.append(old_ax)
        new_axes.append(new_ax_ref)
        old_anchor += old_pos
        new_anchor += new_pos_ref
    old_anchor /= len(jmap)
    new_anchor /= len(jmap)

    # Break 180-deg flip ambiguity using motor-to-gimbal direction in ref_link frame
    flange_positions = [
        np.fromstring(b.get('pos', '0 0 0'), sep=' ')
        for b in module_xml.findall('worldbody/body')
        if b.find('joint').get('name') not in jmap
    ]
    v_old = old_anchor - np.mean(flange_positions, axis=0)
    v_old_n = v_old / (np.linalg.norm(v_old) + 1e-12)
    v_new_n = new_anchor / (np.linalg.norm(new_anchor) + 1e-12)

    best_rot = Rotation.identity()
    best_score = -1e9
    best_err = float('inf')
    best_dot = 0.0
    for s1 in (1.0, -1.0):
        for s2 in (1.0, -1.0):
            rot, rssd = Rotation.align_vectors(
                [new_axes[0], new_axes[1]],
                [s1 * old_axes[0], s2 * old_axes[1]],
            )
            dot = float(np.dot(rot.apply(v_old_n), v_new_n))
            score = -100.0 * rssd + dot
            if score > best_score:
                best_score = score
                best_err = rssd
                best_rot = rot
                best_dot = dot
    t_align = new_anchor - best_rot.apply(old_anchor)
    print(f'  [{name:14s}] axis_align_rms={best_err:.4f}, motor_side_dot={best_dot:+.3f}, t_align={np.round(t_align, 4).tolist()}')
    return path, module_xml, mod_model, mod_data, ref_link, out_link, ref_pos, ref_rot, out_pos, out_rot, best_rot, t_align


def build():
    source = ET.parse(SOURCE_URDF).getroot()
    joints = {j.get('name'): j for j in source.findall('joint')}
    links = {l.get('name'): l for l in source.findall('link')}
    children = defaultdict(list)
    for j in joints.values():
        children[j.find('parent').get('link')].append(j)

    topology = yaml.safe_load(TOPOLOGY_YAML.read_text(encoding='utf-8'))
    limits_raw = yaml.safe_load(LIMITS_YAML.read_text(encoding='utf-8'))['joint_limits']
    motors, groups = topology['motors'], topology['differential_groups']

    root = ET.Element('mujoco', model='Full body weighed differential')
    root.append(ET.Comment(' Generated by tools/build_weighed_mujoco.py from weighed 27.2 kg full_body_robot. '))
    ET.SubElement(root, 'compiler', angle='radian', meshdir='..', fusestatic='false')
    ET.SubElement(root, 'option', timestep='0.002', gravity='0 0 -9.81', integrator='implicitfast')
    custom = ET.SubElement(root, 'custom')
    ET.SubElement(custom, 'text', name='robot_sim_control_mode', data='viewer_position')
    ET.SubElement(root, 'statistic', center='0 0 -0.25', extent='1.42')
    ET.SubElement(ET.SubElement(root, 'visual'), 'global', azimuth='120', elevation='-14')
    defaults = ET.SubElement(root, 'default')
    ET.SubElement(defaults, 'joint', type='hinge', damping='0.1', armature='0.01')
    ET.SubElement(defaults, 'geom', contype='0', conaffinity='0', density='850')
    ET.SubElement(defaults, 'site', size='0.003', group='2')
    ET.SubElement(defaults, 'equality', solref='0.006 1', solimp='0.999 0.9999 0.001')
    ET.SubElement(defaults, 'position', kp='350', kv='4.0', ctrlrange=numbers((-np.pi, np.pi)))

    assets = ET.SubElement(root, 'asset')
    world = ET.SubElement(root, 'worldbody')
    ET.SubElement(world, 'light', pos='0 1 1.8', dir='0 -0.45 -1')
    ET.SubElement(world, 'light', pos='0 -1 0.8', dir='0 0.55 -1')

    added_meshes: set[str] = set()
    bodies = {'Base_link': ET.SubElement(world, 'body', name='Base_link')}
    frames = {'Base_link': (np.zeros(3), Rotation.identity())}

    def add_children(parent: str):
        add_visual_and_inertia(bodies[parent], links[parent], assets, added_meshes)
        p_pos, p_rot = frames[parent]
        for j in children[parent]:
            jn = j.get('name')
            child = j.find('child').get('link')
            pos, rot = parse_origin(j.find('origin'))
            frames[child] = (p_pos + p_rot.apply(pos), p_rot * rot)
            body = ET.SubElement(bodies[parent], 'body', name=child, **pose_attributes(pos, rot))
            lo, hi = float(limits_raw[jn][0]), float(limits_raw[jn][1])
            if lo == hi:
                lo, hi = -LOCKED_SOLVER_RANGE, LOCKED_SOLVER_RANGE
            ET.SubElement(body, 'joint', name=jn, axis=j.find('axis').get('xyz'),
                          limited='true', range=numbers((lo, hi)))
            bodies[child] = body
            add_children(child)

    add_children('Base_link')

    tendons = ET.SubElement(root, 'tendon')
    equalities = ET.SubElement(root, 'equality')

    controls = {
        num: [{'name': f'motor_{num:02d}', 'joint': m['joint']}]
        for num, m in motors.items() if 'joint' in m
    }
    flanges: dict[str, str] = {}

    # Register CAD linkage meshes
    for prefix, (f_stl, r_stl, p_stl) in CAD_MESH_FILES.items():
        for kind, fname in (('flange', f_stl), ('rod', r_stl), ('platform', p_stl)):
            mname = f'{prefix}_{kind}'
            ET.SubElement(assets, 'mesh', name=mname, file=f'mujoco/collision/native/{fname}')

    print('=== ALIGNING 6 DIFFERENTIAL LINKAGE MODULES ===')
    for name, group in groups.items():
        (_path, module_xml, _mm, mod_data, ref_link, out_link,
         ref_pos, ref_rot, out_pos, out_rot, R_align, t_align) = align_module_to_urdf(
            name, group, joints, frames,
        )
        # Transform from ref_link frame to out_link frame
        R_out_ref = out_rot.inv() * ref_rot
        t_out_ref = out_rot.inv().apply(ref_pos - out_pos)

        site_names: dict[str, str] = {}
        flange_bodies: dict[str, ET.Element] = {}
        flange_old_pos: dict[str, np.ndarray] = {}

        for src_body in module_xml.findall('worldbody/body'):
            j_el = src_body.find('joint')
            if j_el.get('name') in DIFF_TO_URDF_JOINTS[name]:
                continue
            old_fpos = np.fromstring(src_body.get('pos', '0 0 0'), sep=' ')
            new_fpos = t_align + R_align.apply(old_fpos)
            flange = ET.SubElement(
                bodies[ref_link], 'body',
                name=f'{name}_{src_body.get("name")}',
                **pose_attributes(new_fpos, R_align),
            )
            ET.SubElement(flange, 'inertial', pos='0 0 0', mass='0.01', diaginertia='1e-6 1e-6 1e-6')
            ET.SubElement(flange, 'joint', name=f'{name}_{j_el.get("name")}',
                          axis=j_el.get('axis'), limited='true', range='-1.5708 1.5708')
            for site in src_body.findall('site'):
                orig = site.get('name')
                sname = f'{name}_{orig}'
                site_names[orig] = sname
                flange_bodies[orig] = flange
                flange_old_pos[orig] = old_fpos
                ET.SubElement(flange, 'site', name=sname, pos=site.get('pos'), rgba='0 0 0 0')

        for spatial in module_xml.findall('tendon/spatial'):
            lower, upper = (s.get('site') for s in spatial.findall('site'))
            side = lower.removesuffix('_motor_ball')
            prefix = f'{name}_{side}'
            start_old = mod_data.site(lower).xpos.copy()
            end_old = mod_data.site(upper).xpos.copy()
            length = float(np.linalg.norm(end_old - start_old))

            # Add platform ball site in out_link frame
            end_ref = t_align + R_align.apply(end_old)
            end_out = t_out_ref + R_out_ref.apply(end_ref)
            upper_name = f'{name}_{upper}'
            site_names[upper] = upper_name
            ET.SubElement(bodies[out_link], 'site', name=upper_name, pos=numbers(end_out), rgba='0 0 0 0')

            # Attach visual CAD meshes for flange & platform
            flange = flange_bodies[lower]
            fpos_old = flange_old_pos[lower]
            if name == 'hips':
                old_start, old_end, old_normal = HIPS_CAD_BALLS[side]
                flange_turn, _ = Rotation.align_vectors([start_old - fpos_old], [old_start - fpos_old])
                platform_turn, _ = Rotation.align_vectors([HIPS_CROSSBEAM_NORMAL], [old_normal])
                vertical = np.array([0.0, 0.0, 1.0])
                rod_turn = rod_frame(end_old - start_old, vertical) * rod_frame(old_end - old_start, vertical).inv()
            elif name == 'waist':
                shift = np.array([-np.sign(start_old[0]) * 0.001, 0.0, 0.0])
                old_start, old_end = start_old - shift, end_old - shift
                flange_turn = platform_turn = rod_turn = Rotation.identity()
            else:
                old_start, old_end = start_old, end_old
                flange_turn = platform_turn = rod_turn = Rotation.identity()

            ET.SubElement(
                flange, 'geom', name=f'{prefix}_flange', type='mesh', mesh=f'{prefix}_flange',
                group='2', rgba='0 0 0 0',
                **pose_attributes(start_old - fpos_old - flange_turn.apply(old_start), flange_turn),
            )
            ET.SubElement(
                bodies[out_link], 'geom', name=f'{prefix}_platform', type='mesh', mesh=f'{prefix}_platform',
                group='2', rgba='0 0 0 0',
                **pose_attributes(
                    t_out_ref + R_out_ref.apply(t_align + R_align.apply(end_old - platform_turn.apply(old_end))),
                    R_out_ref * R_align * platform_turn,
                ),
            )

            # Add rod body with universal joint at motor ball + tip site + equality connect
            rod_name = f'{prefix}_rod'
            rod_orient = rod_frame(end_old - start_old, np.array([1.0, 0.0, 0.0]))
            rod = ET.SubElement(
                flange, 'body', name=rod_name,
                **pose_attributes(start_old - fpos_old, rod_orient),
            )
            for ax_suffix, ax_vec in (('x', '1 0 0'), ('y', '0 1 0')):
                ET.SubElement(rod, 'joint', name=f'{rod_name}_{ax_suffix}', axis=ax_vec,
                              damping='0.001', armature='0.00005')
            trans_I = 0.01 * length**2 / 12.0
            ET.SubElement(rod, 'inertial', pos=numbers((0, 0, length / 2)), mass='0.01',
                          diaginertia=numbers((trans_I, trans_I, 1e-7)))
            mesh_rot = rod_orient.inv() * rod_turn
            ET.SubElement(
                rod, 'geom', name=rod_name, type='mesh', mesh=rod_name,
                group='2', rgba='0 0 0 0',
                **pose_attributes(-mesh_rot.apply(old_start), mesh_rot),
            )
            tip_name = f'{rod_name}_tip'
            ET.SubElement(rod, 'site', name=tip_name, pos=numbers((0, 0, length)), rgba='0 0 0 0')
            ET.SubElement(equalities, 'connect', name=f'{rod_name}_end',
                          site1=tip_name, site2=upper_name,
                          solref='0.006 1', solimp='0.999 0.9999 0.001')

            ten = ET.SubElement(tendons, 'spatial', name=rod_name, width='0.002', rgba='0 0 0 0')
            ET.SubElement(ten, 'site', site=site_names[lower])
            ET.SubElement(ten, 'site', site=upper_name)

        g_controls = []
        for src_act in module_xml.findall('actuator/position'):
            jn = src_act.get('joint')
            mapped = DIFF_TO_URDF_JOINTS[name].get(jn)
            ctrl_name = f'{name}_{src_act.get("name")}'
            if mapped is not None:
                g_controls.append({'name': ctrl_name, 'joint': mapped})
            else:
                g_controls.append({'name': ctrl_name, 'joint': f'{name}_{jn}'})
                side = jn.removeprefix(f'{name}_flange_').removeprefix('flange_').removesuffix('_x')
                flanges[ctrl_name] = f'{name}_{side}_rod' if name in ('waist', 'hips') else f'{name}_flange_{side}_rod'
        controls[group['motor_numbers'][0]] = g_controls

    ET.SubElement(custom, 'text', name='robot_sim_joint_position', data=json.dumps({
        'model': MODEL.name,
        'joints': list(joints),
        'flanges': flanges,
        'joint_limits': {name: [float(limits_raw[name][0]), float(limits_raw[name][1])] for name in joints},
    }, separators=(',', ':')))

    actuators = ET.SubElement(root, 'actuator')
    for num in sorted(controls):
        for ctrl in controls[num]:
            jn = ctrl['joint']
            if jn in limits_raw:
                lo, hi = float(limits_raw[jn][0]), float(limits_raw[jn][1])
                if lo == hi:
                    lo, hi = -LOCKED_SOLVER_RANGE, LOCKED_SOLVER_RANGE
            else:
                lo, hi = -1.5708, 1.5708
            ET.SubElement(actuators, 'position', **ctrl, ctrlrange=numbers((lo, hi)))

    ET.SubElement(ET.SubElement(root, 'keyframe'), 'key', name='Neutral')
    ET.indent(root, space='  ')

    # Compile & verify closed-loop closure + stepping stability
    compiled = deepcopy(root)
    compiled.find('compiler').set('meshdir', str(MODEL))
    mj_model = mujoco.MjModel.from_xml_string(ET.tostring(compiled, encoding='unicode'))
    mj_data = mujoco.MjData(mj_model)
    mujoco.mj_forward(mj_model, mj_data)

    max_err0 = 0.0
    for eq_id in range(mj_model.neq):
        s1 = mj_model.eq_obj1id[eq_id]
        s2 = mj_model.eq_obj2id[eq_id]
        err = float(np.linalg.norm(mj_data.site_xpos[s1] - mj_data.site_xpos[s2]))
        max_err0 = max(max_err0, err)
    print(f'MuJoCo compiled: nu={mj_model.nu}, neq={mj_model.neq}, qpos0_max_closure_err={max_err0:.2e} m')
    assert mj_model.nu == 37, f'Expected 37 actuators, got {mj_model.nu}'
    assert mj_model.neq == 10, f'Expected 10 connect constraints, got {mj_model.neq}'
    assert max_err0 < 1e-4, f'Closure error at q=0 too large: {max_err0}'

    # Test stepping with small coupled motions
    for idx in range(mj_model.nu):
        if mj_model.actuator_trnid[idx, 0] in [mj_model.joint(f).id for f in range(mj_model.njnt)]:
            pass
    for _ in range(50):
        mujoco.mj_step(mj_model, mj_data)
    max_err50 = max(
        float(np.linalg.norm(mj_data.site_xpos[mj_model.eq_obj1id[i]] - mj_data.site_xpos[mj_model.eq_obj2id[i]]))
        for i in range(mj_model.neq)
    )
    print(f'After 50 gravity steps: max_closure_err={max_err50:.2e} m')

    # Test JointPositionTargets integration
    from robot_transmission.mujoco_positions import JointPositionTargets
    cfg = json.loads(custom.find("text[@name='robot_sim_joint_position']").get('data'))
    jpt = JointPositionTargets(mj_model, cfg)
    test_targets = np.zeros(len(jpt.joints))
    for jn in ('waist_pitch', 'waist_roll', 'pelvis_pitch', 'left_scapula_roll', 'left_wrist_pitch'):
        test_targets[list(jpt.joints).index(jn)] = 0.05
    jpt.set_configuration(mj_data, test_targets)
    max_err_cfg = max(
        float(np.linalg.norm(mj_data.site_xpos[mj_model.eq_obj1id[i]] - mj_data.site_xpos[mj_model.eq_obj2id[i]]))
        for i in range(mj_model.neq)
    )
    print(f'JointPositionTargets test OK: max_closure_err={max_err_cfg:.2e} m')

    xml_text = ET.tostring(root, encoding='unicode') + '\n'
    OUT_MJCF.write_text(xml_text, encoding='utf-8')
    OUT_PHYSICS.write_text(xml_text, encoding='utf-8')

    # Write standalone URDF; keep full_body_robot.standing.urdf clean (38 weighed links, no duplicated CAD linkages)
    import build_mujoco_model as shared
    limits_dict = {
        'joint_limits': {jn: {'lower': float(limits_raw[jn][0]), 'upper': float(limits_raw[jn][1])} for jn in joints}
    }
    shared.STANDALONE = OUT_STANDALONE
    shared.write_standalone_urdf(root, joints, limits_dict)
    shutil.copy2(SOURCE_URDF, OUT_STANDING)
    print(f'Wrote {OUT_MJCF}, {OUT_STANDALONE}, {OUT_STANDING}')
    return 0


if __name__ == '__main__':
    raise SystemExit(build())
