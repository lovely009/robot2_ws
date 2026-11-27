#!/usr/bin/env python3
"""Shift pelvis_pitch zero pose by +14.46 deg in URDFs, restore stator screws, and remount hips pushrods."""
from __future__ import annotations

import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import coal
import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation
import vtk
from vtk.util.numpy_support import numpy_to_vtk, vtk_to_numpy

MODEL = Path(__file__).resolve().parents[1]
REPO_SRC = MODEL.parents[2]
URDF_MAIN = MODEL / 'urdf/full_body_robot.urdf'
URDF_FILES = [
    MODEL / 'urdf/full_body_robot.urdf',
    MODEL / 'urdf/full_body_robot.standing.urdf',
    MODEL / 'urdf/full_body_robot.standalone.urdf',
    MODEL / 'urdf/full_body_robot.mapping.urdf',
]
GENERIC_PROFILES = REPO_SRC / 'bvh_broadcaster/config/generic_profiles.yaml'

COLLISION_BASE = MODEL / 'meshes/collision/native/Base_link.STL'
VISUAL_BASE = MODEL / 'meshes/visual/native/Base_link.STL'
MUJOCO_BASE = MODEL / 'mujoco/visual/native/Base_link.STL'
PELVIS_PITCH_STL = MODEL / 'meshes/collision/native/pelvis_pitch.STL'
WAIST_YAW_STL = MODEL / 'meshes/visual/native/waist_yaw.STL'

DELTA = float(np.radians(14.46))

ROLES = {
    'plus': {
        'sign': 1,
        'motor': 3,
        'crank': 159,
        'crank_bolts': [0, 180, 183, 184, 188, 189, 190, 191],
        'stator_screws': [40, 42],
        'stator_undo_rad': float(-np.radians(52.38)),
        'flange_ball': 157,
        'flange_outer': 162,
        'flange_bolt': 166,
        'rod': 163,
        'plat_ball': 160,
        'plat_outer': 161,
        'plat_bolt': 167,
    },
    'minus': {
        'sign': -1,
        'motor': 78,
        'crank': 155,
        'crank_bolts': [181, 182, 185, 186, 187, 192, 193, 204],
        'stator_screws': [115, 116, 117, 118],
        'stator_undo_rad': float(np.radians(38.60)),
        'flange_ball': 158,
        'flange_outer': 152,
        'flange_bolt': 171,
        'rod': 153,
        'plat_ball': 156,
        'plat_outer': 154,
        'plat_bolt': 165,
    },
}


def joint_transform(urdf_root: ET.Element, joint_name: str) -> np.ndarray:
    for joint in urdf_root.findall('joint'):
        if joint.attrib['name'] == joint_name:
            origin = joint.find('origin')
            xyz = [float(x) for x in origin.attrib.get('xyz', '0 0 0').split()]
            rpy = [float(x) for x in origin.attrib.get('rpy', '0 0 0').split()]
            mat = np.eye(4)
            mat[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
            mat[:3, 3] = xyz
            return mat
    raise ValueError(f'Missing joint {joint_name}')


def direction_frame(direction: np.ndarray, normal: np.ndarray) -> np.ndarray:
    z = direction / np.linalg.norm(direction)
    x = normal - np.dot(normal, z) * z
    x /= np.linalg.norm(x)
    return np.column_stack((x, np.cross(z, x), z))


def rigid(rotation: np.ndarray, old_center: np.ndarray, new_center: np.ndarray) -> np.ndarray:
    mat = np.eye(4)
    mat[:3, :3] = rotation
    mat[:3, 3] = new_center - rotation @ old_center
    return mat


def apply_rigid(points: np.ndarray, mat: np.ndarray) -> np.ndarray:
    return points @ mat[:3, :3].T + mat[:3, 3]


def decimate_stl(poly: vtk.vtkPolyData, target_faces: int = 12000) -> vtk.vtkPolyData:
    n_cells = poly.GetNumberOfCells()
    if n_cells <= target_faces:
        return poly
    reduction = max(0.0, min(0.99, 1.0 - (target_faces / float(n_cells))))
    decimator = vtk.vtkQuadricDecimation()
    decimator.SetInputData(poly)
    decimator.SetTargetReduction(reduction)
    decimator.VolumePreservationOn()
    decimator.Update()
    return decimator.GetOutput()


def shift_urdf_zero_poses() -> float:
    old_sha = hashlib.sha256(URDF_MAIN.read_bytes()).hexdigest()
    # Shifts in joint angle space: +delta on pelvis_pitch, -delta on hip pitches
    shifts = {
        'pelvis_pitch': +DELTA,
        'left_hip_pitch': -DELTA,
        'right_hip_pitch': -DELTA,
    }
    for path in URDF_FILES:
        tree = ET.parse(path)
        root = tree.getroot()
        for joint in root.findall('joint'):
            name = joint.attrib['name']
            if name not in shifts:
                continue
            q_shift = shifts[name]
            origin = joint.find('origin')
            axis_el = joint.find('axis')
            axis = np.array([float(x) for x in axis_el.attrib['xyz'].split()], dtype=float)
            axis /= np.linalg.norm(axis)
            rpy_old = [float(x) for x in origin.attrib.get('rpy', '0 0 0').split()]
            r_old = Rotation.from_euler('xyz', rpy_old).as_matrix()
            r_new = r_old @ Rotation.from_rotvec(axis * q_shift).as_matrix()
            rpy_new = Rotation.from_matrix(r_new).as_euler('xyz')
            origin.attrib['rpy'] = ' '.join(f'{v:.12g}' for v in rpy_new)
            limit = joint.find('limit')
            if limit is not None:
                lo = float(limit.attrib['lower']) - q_shift
                hi = float(limit.attrib['upper']) - q_shift
                limit.attrib['lower'] = f'{lo:.6f}'
                limit.attrib['upper'] = f'{hi:.6f}'
        tree.write(path, encoding='utf-8', xml_declaration=False)

    # Compute exact foot_zmin at q=0 for full_body_robot.urdf
    model = pin.buildModelFromUrdf(str(URDF_MAIN))
    data = model.createData()
    pin.forwardKinematics(model, data, pin.neutral(model))
    pin.updateFramePlacements(model, data)
    foot_zmin = float('inf')
    for fname in ('left_ankle_roll', 'right_ankle_roll'):
        fid = model.getFrameId(fname, pin.FrameType.BODY)
        t_f = data.oMf[fid]
        rf = vtk.vtkSTLReader()
        rf.SetFileName(str(MODEL / f'meshes/visual/native/{fname}.STL'))
        rf.Update()
        fpts = vtk_to_numpy(rf.GetOutput().GetPoints().GetData())
        wpts = fpts @ t_f.rotation.T + t_f.translation
        foot_zmin = min(foot_zmin, float(wpts[:, 2].min()))

    # Update world_to_base in standing URDF
    standing_path = MODEL / 'urdf/full_body_robot.standing.urdf'
    st_tree = ET.parse(standing_path)
    st_root = st_tree.getroot()
    for joint in st_root.findall('joint'):
        if joint.attrib['name'] == 'world_to_base':
            joint.find('origin').attrib['xyz'] = f'0 0 {-foot_zmin:.6f}'
            joint.find('origin').attrib['rpy'] = '0 0 0'
    st_tree.write(standing_path, encoding='utf-8', xml_declaration=False)

    new_sha = hashlib.sha256(URDF_MAIN.read_bytes()).hexdigest()
    gp_text = GENERIC_PROFILES.read_text()
    if old_sha in gp_text:
        GENERIC_PROFILES.write_text(gp_text.replace(old_sha, new_sha))
        print(f'Updated generic_profiles.yaml sha256: {old_sha[:12]} -> {new_sha[:12]}')
    else:
        print(f'WARNING: old sha256 {old_sha[:12]} not found in generic_profiles.yaml; new={new_sha[:12]}')
    print(f'Shifted URDFs by delta={np.degrees(DELTA):.2f} deg; foot_zmin={foot_zmin:.6f} m')
    return foot_zmin


def remount_hips_and_verify() -> None:
    urdf_root = ET.parse(URDF_MAIN).getroot()
    t_pr = joint_transform(urdf_root, 'pelvis_roll')
    t_pp = t_pr @ joint_transform(urdf_root, 'pelvis_pitch')

    # Lug normal after +14.46 deg pelvis_pitch rotation in Base_link frame
    # Note: pelvis_pitch axis in Base_link frame is t_pp[:3,:3] @ [0,0,1] = [0, 1, 0] (or [0,-1,0])
    # Wait: t_pp[:3,:3] rotated from old by +DELTA around pp_axis, so old world +Z rotated by R_delta_base:
    pp_axis_base = t_pp[:3, :3] @ np.array([0.0, 0.0, 1.0])
    lug_normal = Rotation.from_rotvec(pp_axis_base * (+DELTA)).as_matrix() @ np.array([0.0, 0.0, 1.0])
    lug_normal /= np.linalg.norm(lug_normal)
    print(f'pp_axis_base={np.round(pp_axis_base, 5)}, lug_normal={np.round(lug_normal, 5)}')

    p_reader = vtk.vtkSTLReader()
    p_reader.SetFileName(str(PELVIS_PITCH_STL))
    p_reader.Update()
    p_pts = vtk_to_numpy(p_reader.GetOutput().GetPoints().GetData()).astype(float)
    p_base = apply_rigid(p_pts, t_pp)

    b_reader = vtk.vtkSTLReader()
    b_reader.SetFileName(str(COLLISION_BASE))
    b_reader.Update()
    cf = vtk.vtkPolyDataConnectivityFilter()
    cf.SetInputConnection(b_reader.GetOutputPort())
    cf.SetExtractionModeToAllRegions()
    cf.ColorRegionsOn()
    cf.Update()

    poly = cf.GetOutput()
    n_regions = cf.GetNumberOfExtractedRegions()
    if n_regions != 207:
        raise RuntimeError(f'Expected 207 regions in Base_link.STL, got {n_regions}')

    pts = vtk_to_numpy(poly.GetPoints().GetData()).astype(float).copy()
    ids = vtk_to_numpy(poly.GetPointData().GetScalars()).astype(int)

    # First restore the 6 stationary stator screws
    motor_axes = {}
    motor_origins = {}
    for side, spec in ROLES.items():
        mpts = pts[ids == spec['motor']]
        c_bb = 0.5 * (mpts.min(0) + mpts.max(0))
        _, _, vt = np.linalg.svd(mpts - mpts.mean(0), full_matrices=False)
        ax = vt[2]
        if ax[2] < 0:
            ax = -ax
        fl = pts[ids == spec['flange_ball']].mean(0)
        orig = c_bb + np.dot(fl - c_bb, ax) * ax
        motor_axes[side] = ax
        motor_origins[side] = orig
        t_undo = rigid(Rotation.from_rotvec(ax * spec['stator_undo_rad']).as_matrix(), orig, orig)
        for rid in spec['stator_screws']:
            mask = ids == rid
            pts[mask] = apply_rigid(pts[mask], t_undo)

    new_fl_map = {}
    for side, spec in ROLES.items():
        sign = spec['sign']
        motor_axis = motor_axes[side]
        origin = motor_origins[side]

        # Find the lug top face in Base_link frame
        lug = p_base[(p_base[:, 0] > 0.010) & (sign * p_base[:, 1] > 0.065) & (p_base[:, 2] > -0.020)]
        proj = lug @ lug_normal
        top = lug[proj > proj.max() - 0.0005]
        seat = top.mean(0)
        target = seat + 0.004 * lug_normal

        fl = pts[ids == spec['flange_ball']].mean(0)
        pl = pts[ids == spec['plat_ball']].mean(0)
        rod_len = float(np.linalg.norm(pl - fl))
        radius = fl - origin
        delta_vec = target - origin
        u = float(np.dot(delta_vec, radius))
        w = float(np.dot(delta_vec, np.cross(motor_axis, radius)))
        cos_val = (np.dot(delta_vec, delta_vec) + np.dot(radius, radius) - rod_len**2) / (2.0 * np.hypot(u, w))
        if abs(cos_val) > 1.0 + 1e-8:
            raise RuntimeError(f'{side}: no rigid rod solution (cos={cos_val})')
        alpha = np.arctan2(w, u)
        beta = np.arccos(np.clip(cos_val, -1.0, 1.0))

        chosen = None
        for ang in (alpha + beta, alpha - beta):
            rot = Rotation.from_rotvec(motor_axis * ang).as_matrix()
            cand_fl = origin + rot @ radius
            if (cand_fl[1] - origin[1]) * sign > 0:
                chosen = (ang, rot, cand_fl)
                break
        if chosen is None:
            raise RuntimeError(f'{side}: no outward branch')
        ang, flange_rot, new_fl = chosen
        new_fl_map[side] = new_fl

        crank_group = [spec['crank'], spec['flange_ball'], spec['flange_bolt'], *spec['crank_bolts']]
        t_crank = rigid(flange_rot, origin, origin)

        rod_group = [spec['rod'], spec['flange_outer'], spec['plat_outer']]
        # Current rod normal: estimate from flange_outer or plat_outer PCA, or use previous normal
        # Note: previous rod_rot aligned [0,0,1] to motor_axis + [0,0,1], so current plat_outer normal is ~[0,0,1]
        cur_pouter = pts[ids == spec['plat_outer']]
        _, _, p_vt = np.linalg.svd(cur_pouter - cur_pouter.mean(0), full_matrices=False)
        cur_normal = p_vt[2]
        if cur_normal[2] < 0:
            cur_normal = -cur_normal
        rod_rot = direction_frame(target - new_fl, motor_axis + lug_normal) @ direction_frame(pl - fl, cur_normal).T
        t_rod = rigid(rod_rot, fl, new_fl)

        bolt_pts = pts[ids == spec['plat_bolt']]
        _, _, b_vt = np.linalg.svd(bolt_pts - bolt_pts.mean(0), full_matrices=False)
        bolt_axis = b_vt[0]
        if bolt_axis[2] < 0:
            bolt_axis = -bolt_axis
        plat_rot = Rotation.align_vectors([lug_normal], [bolt_axis])[0].as_matrix()
        t_plat = rigid(plat_rot, pl, target)
        plat_group = [spec['plat_ball'], spec['plat_bolt']]

        for rid in crank_group:
            mask = ids == rid
            pts[mask] = apply_rigid(pts[mask], t_crank)
        for rid in rod_group:
            mask = ids == rid
            pts[mask] = apply_rigid(pts[mask], t_rod)
        for rid in plat_group:
            mask = ids == rid
            pts[mask] = apply_rigid(pts[mask], t_plat)

        gap = float(np.linalg.norm(pts[ids == spec['plat_ball']].mean(0) - target))
        print(f'{side}: ang={np.degrees(ang):+.2f} deg, seat={np.round(seat, 5)}, target={np.round(target, 5)}, new_fl={np.round(new_fl, 5)}, gap={gap:.2e}')
        assert gap < 1e-6

    fl_p, fl_m = new_fl_map['plus'], new_fl_map['minus']
    assert abs(fl_p[0] - fl_m[0]) < 1e-3
    assert abs(fl_p[1] + fl_m[1]) < 1e-3
    assert abs(fl_p[2] - fl_m[2]) < 1e-3

    # Verify 0 collisions with waist_yaw using coal
    model = pin.buildModelFromUrdf(str(MODEL / 'urdf/full_body_robot.standing.urdf'))
    data = model.createData()
    pin.forwardKinematics(model, data, pin.neutral(model))
    pin.updateFramePlacements(model, data)
    t_b = data.oMf[model.getFrameId('Base_link', pin.FrameType.BODY)]
    t_wy = t_b.inverse() * data.oMf[model.getFrameId('waist_yaw', pin.FrameType.BODY)]

    r_wy = vtk.vtkSTLReader()
    r_wy.SetFileName(str(WAIST_YAW_STL))
    r_wy.Update()
    wy_poly = r_wy.GetOutput()
    wy_pts = vtk_to_numpy(wy_poly.GetPoints().GetData()).astype(float) @ t_wy.rotation.T + t_wy.translation
    wy_cells = vtk_to_numpy(wy_poly.GetPolys().GetData()).reshape(-1, 4)[:, 1:]
    wy_bvh = coal.BVHModelOBBRSS()
    wy_bvh.beginModel(len(wy_cells), len(wy_pts))
    for tri in wy_cells:
        wy_bvh.addTriangle(wy_pts[tri[0]], wy_pts[tri[1]], wy_pts[tri[2]])
    wy_bvh.endModel()

    cells = vtk_to_numpy(poly.GetPolys().GetData()).reshape(-1, 4)[:, 1:]
    ident = coal.Transform3s()
    req = coal.CollisionRequest()
    for side, spec in ROLES.items():
        for key in ('rod', 'plat_ball', 'plat_bolt', 'plat_outer', 'crank'):
            rid = spec[key]
            sub_cells = cells[ids[cells[:, 0]] == rid]
            bvh = coal.BVHModelOBBRSS()
            bvh.beginModel(len(sub_cells), len(pts))
            for tri in sub_cells:
                bvh.addTriangle(pts[tri[0]], pts[tri[1]], pts[tri[2]])
            bvh.endModel()
            res = coal.CollisionResult()
            coal.collide(bvh, ident, wy_bvh, ident, req, res)
            assert not res.isCollision(), f'{side}_{key} still collides with waist_yaw!'
    print('Coal check: 0 collisions between hips linkages and waist_yaw!')

    vtk_pts = vtk.vtkPoints()
    vtk_pts.SetData(numpy_to_vtk(pts.astype(np.float32), deep=1))
    poly.SetPoints(vtk_pts)
    poly.Modified()

    w_col = vtk.vtkSTLWriter()
    w_col.SetFileTypeToBinary()
    w_col.SetFileName(str(COLLISION_BASE))
    w_col.SetInputData(poly)
    w_col.Write()

    vis_poly = decimate_stl(poly, target_faces=12000)
    for out_path in (VISUAL_BASE, MUJOCO_BASE):
        if out_path.parent.exists():
            w_vis = vtk.vtkSTLWriter()
            w_vis.SetFileTypeToBinary()
            w_vis.SetFileName(str(out_path))
            w_vis.SetInputData(vis_poly)
            w_vis.Write()
    print(f'Saved Base_link.STL: collision={poly.GetNumberOfCells()} cells, visual={vis_poly.GetNumberOfCells()} cells')


if __name__ == '__main__':
    shift_urdf_zero_poses()
    remount_hips_and_verify()
