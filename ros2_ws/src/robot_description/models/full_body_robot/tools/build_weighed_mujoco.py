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
MOTOR_SPECS_YAML = MODEL.parents[1] / 'config/motor.yaml'
OUT_MJCF = MODEL / 'mujoco/full_body_robot.xml'
OUT_PHYSICS = MODEL / 'mujoco/full_body_robot.physics.xml'
OUT_TORQUE = MODEL / 'mujoco/full_body_robot.torque.xml'
OUT_STANDALONE = MODEL / 'urdf/full_body_robot.standalone.urdf'
OUT_STANDING = MODEL / 'urdf/full_body_robot.standing.urdf'

LOCKED_SOLVER_RANGE = 1e-9


def _joint_bounds(entry):
    """Return (lower, upper) floats from either {lower, upper} dict or [lo, hi] list."""
    if isinstance(entry, dict):
        return float(entry['lower']), float(entry['upper'])
    return float(entry[0]), float(entry[1])

# Map differential XML joint names -> new URDF joint names
DIFF_TO_URDF_JOINTS = {
    'waist': {'BN_Spine_pitch': 'waist_pitch', 'BN_Spine_roll': 'waist_roll'},
    'hips': {'BN_hips_roll': 'pelvis_roll', 'BN_hips_pitch': 'pelvis_pitch'},
    'left_shoulder': {'BN_LeftShoulder_1': 'left_scapula_yaw', 'BN_LeftShoulder_2': 'left_scapula_roll'},
    'right_shoulder': {'BN_RightShoulder_1': 'right_scapula_yaw', 'BN_RightShoulder_2': 'right_scapula_roll'},
    'left_wrist': {'BN_LeftHand_roll': 'left_wrist_roll', 'BN_LeftHand_pitch': 'left_wrist_pitch'},
    'right_wrist': {'BN_RightHand_roll': 'right_wrist_pitch', 'BN_RightHand_pitch': 'right_wrist_roll'},
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

# Exact weighed-CAD (flange_ref, lower_ball_ref, upper_ball_ref) in ref_link frame
WEIGHED_CAD_OVERRIDES = {
    'waist': {
        'plus': (
            np.array([0.051, -0.008308, -0.049369]),
            np.array([0.051, -0.053237, -0.071309]),
            np.array([0.051, -0.046210, -0.112813]),
        ),
        'minus': (
            np.array([-0.051, -0.008308, -0.049369]),
            np.array([-0.051, -0.053237, -0.071309]),
            np.array([-0.051, -0.046210, -0.112813]),
        ),
    },
    'hips': {
        'plus': (
            np.array([0.000117, -0.07205, 0.029625]),
            np.array([0.00835, -0.10475, 0.02675]),
            np.array([0.062, -0.0715, 0.0114]),
        ),
        'minus': (
            np.array([0.000117, 0.07205, 0.029625]),
            np.array([0.00835, 0.10475, 0.02675]),
            np.array([0.062, 0.0715, 0.0114]),
        ),
    },
}


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
        if src_stl.resolve() != dst_stl.resolve():
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

    # Score all 4 (s1, s2) sign combinations against actual weighed ref_link & out_link STLs
    from scipy.spatial import cKDTree
    from vtk.util.numpy_support import vtk_to_numpy

    def _stl_tree(stl_path: Path) -> cKDTree:
        r = vtk.vtkSTLReader()
        r.SetFileName(str(stl_path))
        r.Update()
        pts = vtk_to_numpy(r.GetOutput().GetPoints().GetData()).astype(float)
        return cKDTree(pts)

    ref_tree = _stl_tree(MODEL / f'meshes/visual/native/{ref_link}.STL')
    out_tree = _stl_tree(MODEL / f'meshes/visual/native/{out_link}.STL')
    R_out_ref = out_rot.inv() * ref_rot
    t_out_ref = out_rot.inv().apply(ref_pos - out_pos)

    lowers, uppers = [], []
    for sp in module_xml.findall('tendon/spatial'):
        lo_s, hi_s = [s.get('site') for s in sp.findall('site')]
        lowers.append(mod_data.site(lo_s).xpos.copy())
        uppers.append(mod_data.site(hi_s).xpos.copy())
    flanges = [
        np.fromstring(b.get('pos', '0 0 0'), sep=' ')
        for b in module_xml.findall('worldbody/body')
        if b.find('joint').get('name') not in jmap
    ]
    lowers, uppers, flanges = np.array(lowers), np.array(uppers), np.array(flanges)

    best_rot = Rotation.identity()
    best_score = float('inf')
    best_err = float('inf')
    best_lo = best_hi = best_fl = 0.0
    for s1 in (1.0, -1.0):
        for s2 in (1.0, -1.0):
            rot, rssd = Rotation.align_vectors(
                [new_axes[0], new_axes[1]],
                [s1 * old_axes[0], s2 * old_axes[1]],
            )
            t_cand = new_anchor - rot.apply(old_anchor)
            lo_ref = rot.apply(lowers) + t_cand
            hi_out = R_out_ref.apply(rot.apply(uppers) + t_cand) + t_out_ref
            fl_ref = rot.apply(flanges) + t_cand
            d_lo = float(ref_tree.query(lo_ref)[0].mean()) * 1000.0
            d_hi = float(out_tree.query(hi_out)[0].mean()) * 1000.0
            d_fl = float(ref_tree.query(fl_ref)[0].mean()) * 1000.0
            score = 100000.0 * rssd + d_lo + d_hi + d_fl
            if score < best_score:
                best_score = score
                best_err = rssd
                best_rot = rot
                best_lo, best_hi, best_fl = d_lo, d_hi, d_fl
    t_align = new_anchor - best_rot.apply(old_anchor)
    print(f'  [{name:14s}] axis_align_rms={best_err:.4f}, lo={best_lo:.1f}mm hi={best_hi:.1f}mm fl={best_fl:.1f}mm, t_align={np.round(t_align, 4).tolist()}')
    return path, module_xml, mod_model, mod_data, ref_link, out_link, ref_pos, ref_rot, out_pos, out_rot, best_rot, t_align


def _write_binary_stl(path: Path, pts: np.ndarray, tris: np.ndarray) -> None:
    import struct
    path.parent.mkdir(parents=True, exist_ok=True)
    tri_pts = pts[tris].astype(np.float32)
    v1 = tri_pts[:, 1] - tri_pts[:, 0]
    v2 = tri_pts[:, 2] - tri_pts[:, 0]
    norms = np.cross(v1, v2)
    nlen = np.linalg.norm(norms, axis=1, keepdims=True)
    norms = np.divide(norms, nlen, out=np.zeros_like(norms), where=nlen > 1e-12)
    with path.open('wb') as f:
        f.write(b'split weighed linkage'.ljust(80, b'\0'))
        f.write(struct.pack('<I', len(tris)))
        rec = np.zeros(len(tris), dtype=[('n', '<f4', 3), ('v', '<f4', (3, 3)), ('attr', '<u2')])
        rec['n'] = norms
        rec['v'] = tri_pts
        f.write(rec.tobytes())


def _split_weighed_visual_linkages(root: ET.Element, mj_model) -> None:
    """Split real SolidWorks cranks & pushrods from weighed STLs into moving MuJoCo bodies."""
    from vtk.util.numpy_support import vtk_to_numpy

    zero_data = mujoco.MjData(mj_model)
    mujoco.mj_forward(mj_model, zero_data)

    assets = root.find('asset')
    mesh_els = {el.get('name'): el for el in assets.findall('mesh')}
    body_els = {el.get('name'): el for el in root.iter('body') if el.get('name')}
    parent_map = {child: parent for parent in root.iter('body') for child in parent.findall('body')}

    split_dir = MODEL / 'mujoco/visual_split'
    split_dir.mkdir(parents=True, exist_ok=True)

    specs = [
        {
            'link': 'waist_yaw',
            'out_link': 'waist_roll',
            'src_stl': MODEL / 'meshes/visual/native/waist_yaw.STL',
            'mesh_name': 'waist_yaw_vis_0',
            'decimate_faces': 0,
            'group': 'waist',
            'rod_sets': [{58}, {68}],
            'flange_sets': [
                {57, 2, 63, 69, 77, 78, 79, 82, 83, 84, 87, 91},
                {64, 65, 66, 71, 93, 94, 99, 104, 105, 106, 107, 108},
            ],
            'platform_rids': {1, 62, 74, 60, 61, 67},
        },
        {
            'link': 'Base_link',
            'out_link': 'pelvis_pitch',
            'src_stl': MODEL / 'meshes/collision/native/Base_link.STL',
            'mesh_name': 'Base_link_vis_0',
            'decimate_faces': 30000,
            'group': 'hips',
            'rod_sets': [{153}, {163}],
            'flange_sets': [
                {155, 152, 158, 171, 181, 182, 185, 186, 187, 192, 193, 204},
                {159, 162, 157, 166, 0, 180, 183, 184, 188, 189, 190, 191},
            ],
            'platform_rids': {154, 156, 165, 160, 161, 167},
        },
    ]

    for spec in specs:
        link_name = spec['link']
        gname = spec['group']
        r = vtk.vtkSTLReader()
        r.SetFileName(str(spec['src_stl']))
        r.Update()
        conn = vtk.vtkPolyDataConnectivityFilter()
        conn.SetInputConnection(r.GetOutputPort())
        conn.SetExtractionModeToAllRegions()
        conn.ColorRegionsOn()
        conn.Update()
        poly = conn.GetOutput()
        pts = vtk_to_numpy(poly.GetPoints().GetData()).astype(float)
        tris = vtk_to_numpy(poly.GetPolys().GetData()).reshape(-1, 4)[:, 1:]
        rids = vtk_to_numpy(poly.GetPointData().GetArray('RegionId'))
        tri_rids = rids[tris[:, 0]]

        link_bid = mj_model.body(link_name).id
        link_pos = zero_data.xpos[link_bid]
        link_rot = zero_data.xmat[link_bid].reshape(3, 3)

        # Candidate MuJoCo rod bodies & their parent flange bodies, keyed by motor_ball site world pos
        cand_rods = {}
        for side in ('plus', 'minus'):
            rod_name = f'{gname}_{side}_rod'
            site_pos = zero_data.site(f'{gname}_{side}_motor_ball').xpos.copy()
            flange_el = parent_map[body_els[rod_name]]
            cand_rods[rod_name] = (site_pos, flange_el.get('name'))

        assigned_rids: set[int] = set()
        for kind, region_sets in (('rod', spec['rod_sets']), ('flange', spec['flange_sets'])):
            remaining_cands = dict(cand_rods)
            for rset in region_sets:
                mask = np.isin(tri_rids, list(rset))
                sub_tris = tris[mask]
                assert len(sub_tris) > 0, f'No triangles for {link_name} regions {rset}'
                used_idx, inv_tris = np.unique(sub_tris, return_inverse=True)
                sub_pts_w = pts[used_idx] @ link_rot.T + link_pos

                # Match to closest motor_ball site among remaining candidates
                best_rod = min(
                    remaining_cands,
                    key=lambda rn: float(np.linalg.norm(sub_pts_w - remaining_cands[rn][0], axis=1).min()),
                )
                site_pos, flange_body_name = remaining_cands.pop(best_rod)
                target_body_name = best_rod if kind == 'rod' else flange_body_name

                t_bid = mj_model.body(target_body_name).id
                t_pos = zero_data.xpos[t_bid]
                t_rot = zero_data.xmat[t_bid].reshape(3, 3)
                local_pts = (sub_pts_w - t_pos) @ t_rot

                vis_name = f'{target_body_name}_vis'
                out_stl = split_dir / f'{vis_name}.STL'
                _write_binary_stl(out_stl, local_pts, inv_tris.reshape(-1, 3))
                ET.SubElement(assets, 'mesh', name=vis_name, file=f'mujoco/visual_split/{vis_name}.STL')
                ET.SubElement(
                    body_els[target_body_name],
                    'geom',
                    name=vis_name,
                    type='mesh',
                    mesh=vis_name,
                    group='0',
                    rgba='0.72 0.76 0.80 1',
                )
                assigned_rids.update(rset)

        platform_rids = spec.get('platform_rids', set())
        if platform_rids:
            out_link = spec['out_link']
            mask = np.isin(tri_rids, list(platform_rids))
            sub_tris = tris[mask]
            assert len(sub_tris) > 0, f'No platform triangles for {link_name} regions {platform_rids}'
            used_idx, inv_tris = np.unique(sub_tris, return_inverse=True)
            sub_pts_w = pts[used_idx] @ link_rot.T + link_pos
            o_bid = mj_model.body(out_link).id
            o_pos = zero_data.xpos[o_bid]
            o_rot = zero_data.xmat[o_bid].reshape(3, 3)
            local_pts = (sub_pts_w - o_pos) @ o_rot
            vis_name = f'{out_link}_platform_pins_vis'
            out_stl = split_dir / f'{vis_name}.STL'
            _write_binary_stl(out_stl, local_pts, inv_tris.reshape(-1, 3))
            ET.SubElement(assets, 'mesh', name=vis_name, file=f'mujoco/visual_split/{vis_name}.STL')
            ET.SubElement(
                body_els[out_link],
                'geom',
                name=vis_name,
                type='mesh',
                mesh=vis_name,
                group='0',
                rgba='0.72 0.76 0.80 1',
            )
            assigned_rids.update(platform_rids)

        # Write remaining housing triangles
        keep_mask = ~np.isin(tri_rids, list(assigned_rids))
        keep_tris = tris[keep_mask]
        assert len(keep_tris) > 100, f'Too few housing triangles left for {link_name}'
        housing_stl = split_dir / f'{link_name}_housing.STL'
        used_idx, inv_tris = np.unique(keep_tris, return_inverse=True)
        _write_binary_stl(housing_stl, pts[used_idx], inv_tris.reshape(-1, 3))
        if spec['decimate_faces'] and len(keep_tris) > spec['decimate_faces']:
            decimate_for_mujoco(housing_stl, housing_stl, target_faces=spec['decimate_faces'])
        mesh_els[spec['mesh_name']].set('file', f'mujoco/visual_split/{link_name}_housing.STL')
        print(f'Split {link_name}: extracted {len(assigned_rids)} linkage regions -> {housing_stl.name}')

    recompiled = deepcopy(root)
    recompiled.find('compiler').set('meshdir', str(MODEL))
    mujoco.MjModel.from_xml_string(ET.tostring(recompiled, encoding='unicode'))


def build():
    source = ET.parse(SOURCE_URDF).getroot()
    joints = {j.get('name'): j for j in source.findall('joint')}
    links = {l.get('name'): l for l in source.findall('link')}
    children = defaultdict(list)
    for j in joints.values():
        children[j.find('parent').get('link')].append(j)

    topology = yaml.safe_load(TOPOLOGY_YAML.read_text(encoding='utf-8'))
    limits_doc = yaml.safe_load(LIMITS_YAML.read_text(encoding='utf-8'))
    limits_raw = limits_doc['joint_limits']
    flange_limits = limits_doc.get('flange_limits', {})
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
            lo, hi = _joint_bounds(limits_raw[jn])
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

        overrides = WEIGHED_CAD_OVERRIDES.get(name, {})
        flange_new_pos: dict[str, np.ndarray] = {}

        for src_body in module_xml.findall('worldbody/body'):
            j_el = src_body.find('joint')
            if j_el.get('name') in DIFF_TO_URDF_JOINTS[name]:
                continue
            old_fpos = np.fromstring(src_body.get('pos', '0 0 0'), sep=' ')
            sites = src_body.findall('site')
            side = sites[0].get('name').removesuffix('_motor_ball') if sites else (
                'plus' if 'plus' in src_body.get('name', '') else 'minus'
            )
            if side in overrides:
                flange_ref, lower_ref, _upper_ref = overrides[side]
                new_fpos = flange_ref
            else:
                new_fpos = t_align + R_align.apply(old_fpos)
            flange = ET.SubElement(
                bodies[ref_link], 'body',
                name=f'{name}_{src_body.get("name")}',
                **pose_attributes(new_fpos, R_align),
            )
            ET.SubElement(flange, 'inertial', pos='0 0 0', mass='0.01', diaginertia='1e-6 1e-6 1e-6')
            flange_jn = f'{name}_{j_el.get("name")}'
            flange_range = (
                numbers(_joint_bounds(flange_limits[flange_jn]))
                if flange_jn in flange_limits else '-1.5708 1.5708'
            )
            ET.SubElement(flange, 'joint', name=flange_jn,
                          axis=j_el.get('axis'), limited='true', range=flange_range)
            for site in sites:
                orig = site.get('name')
                sname = f'{name}_{orig}'
                site_names[orig] = sname
                flange_bodies[orig] = flange
                flange_old_pos[orig] = old_fpos
                flange_new_pos[orig] = new_fpos
                site_pos_str = (
                    numbers(R_align.inv().apply(overrides[side][1] - new_fpos))
                    if side in overrides else site.get('pos')
                )
                ET.SubElement(flange, 'site', name=sname, pos=site_pos_str, rgba='0 0 0 0')

        for spatial in module_xml.findall('tendon/spatial'):
            lower, upper = (s.get('site') for s in spatial.findall('site'))
            side = lower.removesuffix('_motor_ball')
            prefix = f'{name}_{side}'
            start_old = mod_data.site(lower).xpos.copy()
            end_old = mod_data.site(upper).xpos.copy()
            flange = flange_bodies[lower]
            fpos_old = flange_old_pos[lower]

            if side in overrides:
                flange_ref, lower_ref, upper_ref = overrides[side]
                end_ref = upper_ref
                rod_start_local = R_align.inv().apply(lower_ref - flange_ref)
                rod_end_local = R_align.inv().apply(upper_ref - flange_ref)
                rod_vec_local = rod_end_local - rod_start_local
                length = float(np.linalg.norm(rod_vec_local))
                rod_orient = rod_frame(rod_vec_local, np.array([1.0, 0.0, 0.0]))
            else:
                end_ref = t_align + R_align.apply(end_old)
                rod_start_local = start_old - fpos_old
                length = float(np.linalg.norm(end_old - start_old))
                rod_orient = rod_frame(end_old - start_old, np.array([1.0, 0.0, 0.0]))

            # Add platform ball site in out_link frame
            end_out = t_out_ref + R_out_ref.apply(end_ref)
            upper_name = f'{name}_{upper}'
            site_names[upper] = upper_name
            ET.SubElement(bodies[out_link], 'site', name=upper_name, pos=numbers(end_out), rgba='0 0 0 0')

            # Attach visual CAD meshes for flange & platform
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
            rod = ET.SubElement(
                flange, 'body', name=rod_name,
                **pose_attributes(rod_start_local, rod_orient),
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
        'joint_limits': {name: list(_joint_bounds(limits_raw[name])) for name in joints},
    }, separators=(',', ':')))

    actuators = ET.SubElement(root, 'actuator')
    for num in sorted(controls):
        for ctrl in controls[num]:
            jn = ctrl['joint']
            if jn in limits_raw:
                lo, hi = _joint_bounds(limits_raw[jn])
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

    # Split real SolidWorks cranks & pushrods from weighed STLs into MuJoCo moving bodies
    _split_weighed_visual_linkages(root, mj_model)

    xml_text = ET.tostring(root, encoding='unicode') + '\n'
    OUT_MJCF.write_text(xml_text, encoding='utf-8')
    OUT_PHYSICS.write_text(xml_text, encoding='utf-8')

    # Build effort/torque MJCF variant with measured R58/R48 viscous damping and Coulomb friction
    motor_specs = yaml.safe_load(MOTOR_SPECS_YAML.read_text(encoding='utf-8'))['motor_types']
    motor_params = yaml.safe_load(PARAMS_YAML.read_text(encoding='utf-8'))['motors']
    template_by_id = {int(entry['id']): entry['template'] for entry in motor_params}
    group_motor_ids = {g['motor_numbers'][0]: list(g['motor_numbers']) for g in groups.values()}

    torque_root = deepcopy(root)
    torque_custom = torque_root.find('custom')
    if torque_custom is not None:
        cm = torque_custom.find("text[@name='robot_sim_control_mode']")
        if cm is not None:
            torque_custom.remove(cm)
    torque_defaults = torque_root.find('default')
    if torque_defaults is not None:
        pos_def = torque_defaults.find('position')
        if pos_def is not None:
            torque_defaults.remove(pos_def)

    passive_diff_joints = {
        jn for mapping in DIFF_TO_URDF_JOINTS.values() for jn in mapping.values()
    }
    joint_elements = {j.get('name'): j for j in torque_root.iter('joint') if j.get('name')}
    for jn in passive_diff_joints:
        je = joint_elements.get(jn)
        if je is not None:
            je.set('damping', '0.02')
            je.set('frictionloss', '0')

    torque_actuators = torque_root.find('actuator')
    torque_actuators.clear()
    motor_driven_specs = {}
    for num in sorted(controls):
        mids = group_motor_ids.get(num, [num])
        for mid, ctrl in zip(mids, controls[num]):
            tmpl_name = template_by_id[mid]
            spec = motor_specs[tmpl_name]['parameters']
            damping = float(spec.get('viscous_damping_nm_s_per_rad', 0.1))
            frictionloss = float(spec.get('coulomb_friction_nm', 0.0))
            peak_torque = float(spec.get('peak_torque_nm', spec.get('rated_torque_nm', 1.15)))
            jn = ctrl['joint']
            je = joint_elements[jn]
            je.set('damping', f'{damping:g}')
            je.set('frictionloss', f'{frictionloss:g}')
            je.set('actuatorfrcrange', f'{-peak_torque:g} {peak_torque:g}')
            ET.SubElement(
                torque_actuators,
                'motor',
                name=ctrl['name'],
                joint=jn,
                ctrllimited='true',
                ctrlrange=f'{-peak_torque:g} {peak_torque:g}',
            )
            motor_driven_specs[jn] = (tmpl_name, damping, frictionloss, peak_torque)

    ET.indent(torque_root, space='  ')
    compiled_torque = deepcopy(torque_root)
    compiled_torque.find('compiler').set('meshdir', str(MODEL))
    t_model = mujoco.MjModel.from_xml_string(ET.tostring(compiled_torque, encoding='unicode'))
    assert t_model.nu == 37, f'Expected 37 torque actuators, got {t_model.nu}'
    assert t_model.neq == 10, f'Expected 10 connect constraints, got {t_model.neq}'
    for jn, (tmpl_name, damping, frictionloss, peak_torque) in motor_driven_specs.items():
        dof = int(t_model.jnt_dofadr[t_model.joint(jn).id])
        assert np.isclose(t_model.dof_damping[dof], damping), f'{jn} damping mismatch'
        assert np.isclose(t_model.dof_frictionloss[dof], frictionloss), f'{jn} frictionloss mismatch'

    t_data = mujoco.MjData(t_model)
    act_dofs = t_model.jnt_dofadr[t_model.actuator_trnid[:, 0]]
    act_qpos = t_model.jnt_qposadr[t_model.actuator_trnid[:, 0]]
    act_ranges = t_model.actuator_ctrlrange
    for _ in range(200):
        tau = 30.0 * (0.0 - t_data.qpos[act_qpos]) - 1.0 * t_data.qvel[act_dofs]
        t_data.ctrl[:] = np.clip(tau, act_ranges[:, 0], act_ranges[:, 1])
        mujoco.mj_step(t_model, t_data)
    assert np.all(np.isfinite(t_data.qpos)) and np.all(np.isfinite(t_data.qvel)), 'Torque sim produced NaN/Inf'
    t_err = max(
        float(np.linalg.norm(t_data.site_xpos[t_model.eq_obj1id[i]] - t_data.site_xpos[t_model.eq_obj2id[i]]))
        for i in range(t_model.neq)
    )
    assert t_err < 1e-3, f'Torque sim closure error too large: {t_err}'
    print(f'Torque MJCF compiled & 200-step PD hold OK: nu={t_model.nu}, neq={t_model.neq}, max_closure_err={t_err:.2e} m')
    OUT_TORQUE.write_text(ET.tostring(torque_root, encoding='unicode') + '\n', encoding='utf-8')

    # Write standalone URDF; keep full_body_robot.standing.urdf clean (38 weighed links, no duplicated CAD linkages)
    import build_mujoco_model as shared
    limits_dict = {
        'joint_limits': {jn: dict(zip(('lower', 'upper'), _joint_bounds(limits_raw[jn]))) for jn in joints}
    }
    shared.STANDALONE = OUT_STANDALONE
    shared.write_standalone_urdf(root, joints, limits_dict)
    shutil.copy2(SOURCE_URDF, OUT_STANDING)
    print(f'Wrote {OUT_MJCF}, {OUT_TORQUE}, {OUT_STANDALONE}, {OUT_STANDING}')
    return 0


if __name__ == '__main__':
    raise SystemExit(build())
