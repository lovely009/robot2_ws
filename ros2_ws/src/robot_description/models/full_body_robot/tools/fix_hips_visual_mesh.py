#!/usr/bin/env python3
"""Re-mount the splayed hips pushrods in Base_link.STL onto pelvis_pitch lugs."""
from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import vtk
from vtk.util.numpy_support import numpy_to_vtk, vtk_to_numpy

MODEL = Path(__file__).resolve().parents[1]
URDF_PATH = MODEL / 'urdf/full_body_robot.urdf'
COLLISION_BASE = MODEL / 'meshes/collision/native/Base_link.STL'
VISUAL_BASE = MODEL / 'meshes/visual/native/Base_link.STL'
MUJOCO_BASE = MODEL / 'mujoco/visual/native/Base_link.STL'
PELVIS_PITCH_STL = MODEL / 'meshes/collision/native/pelvis_pitch.STL'

ROLES = {
    'plus': {
        'sign': 1,
        'motor': 3,
        'crank': 159,
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


def main() -> None:
    urdf_root = ET.parse(URDF_PATH).getroot()
    t_pp = joint_transform(urdf_root, 'pelvis_roll') @ joint_transform(urdf_root, 'pelvis_pitch')

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

    region_pts = {rid: pts[ids == rid] for rid in range(n_regions)}
    new_fl_map: dict[str, np.ndarray] = {}

    for side, spec in ROLES.items():
        sign = spec['sign']
        lug = p_base[(p_base[:, 0] > 0.025) & (sign * p_base[:, 1] > 0.065) & (p_base[:, 2] > 0.010)]
        top = lug[lug[:, 2] > lug[:, 2].max() - 0.0005]
        seat = top.mean(0)
        target = seat + np.array([0.0, 0.0, 0.004])

        motor_pts = region_pts[spec['motor']]
        c_bb = 0.5 * (motor_pts.min(0) + motor_pts.max(0))
        _, _, vt = np.linalg.svd(motor_pts - motor_pts.mean(0), full_matrices=False)
        motor_axis = vt[2]
        if motor_axis[2] < 0:
            motor_axis = -motor_axis

        fl = region_pts[spec['flange_ball']].mean(0)
        pl = region_pts[spec['plat_ball']].mean(0)
        rod_len = float(np.linalg.norm(pl - fl))
        origin = c_bb + np.dot(fl - c_bb, motor_axis) * motor_axis
        radius = fl - origin
        delta = target - origin
        u = float(np.dot(delta, radius))
        w = float(np.dot(delta, np.cross(motor_axis, radius)))
        cos_val = (np.dot(delta, delta) + np.dot(radius, radius) - rod_len**2) / (2.0 * np.hypot(u, w))
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
            raise RuntimeError(f'{side}: no outward branch found')
        ang, flange_rot, new_fl = chosen
        new_fl_map[side] = new_fl

        # Auto-detect crank mounting screws (n_pts < 1000, dist < 1mm to crank, excluding main/motor/rod/balls)
        excluded = {1, spec['motor'], spec['crank'], spec['flange_ball'], spec['flange_outer'],
                    spec['flange_bolt'], spec['rod'], spec['plat_ball'], spec['plat_outer'], spec['plat_bolt']}
        crank_tree = cKDTree(region_pts[spec['crank']])
        crank_bolts = []
        for rid, rpts in region_pts.items():
            if rid in excluded or len(rpts) >= 1000:
                continue
            d, _ = crank_tree.query(rpts)
            if d.min() < 0.001:
                crank_bolts.append(rid)

        crank_group = [spec['crank'], spec['flange_ball'], spec['flange_bolt'], *crank_bolts]
        t_crank = rigid(flange_rot, origin, origin)

        rod_group = [spec['rod'], spec['flange_outer'], spec['plat_outer']]
        rod_rot = direction_frame(target - new_fl, motor_axis + np.array([0.0, 0.0, 1.0])) @ direction_frame(pl - fl, np.array([0.0, 0.0, 1.0])).T
        t_rod = rigid(rod_rot, fl, new_fl)

        # Align platform bolt axis (longest principal axis) to +Z (or -Z)
        bolt_pts = region_pts[spec['plat_bolt']]
        _, _, b_vt = np.linalg.svd(bolt_pts - bolt_pts.mean(0), full_matrices=False)
        bolt_axis = b_vt[0]
        if bolt_axis[2] < 0:
            bolt_axis = -bolt_axis
        plat_rot = Rotation.align_vectors([np.array([0.0, 0.0, 1.0])], [bolt_axis])[0].as_matrix()
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

        new_pl = pts[ids == spec['plat_ball']].mean(0)
        gap = float(np.linalg.norm(new_pl - target))
        print(
            f'{side}: ang={np.degrees(ang):+.2f} deg, crank_bolts={crank_bolts}, '
            f'seat={np.round(seat, 5)}, target={np.round(target, 5)}, '
            f'new_fl={np.round(new_fl, 5)}, gap={gap:.2e} m'
        )
        assert gap < 1e-6, f'{side} gap too large: {gap}'

    fl_p, fl_m = new_fl_map['plus'], new_fl_map['minus']
    assert abs(fl_p[0] - fl_m[0]) < 1e-3
    assert abs(fl_p[1] + fl_m[1]) < 1e-3
    assert abs(fl_p[2] - fl_m[2]) < 1e-3

    # Write back to polydata
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
    print(f'Saved collision Base_link.STL ({poly.GetNumberOfCells()} cells) and visual ({vis_poly.GetNumberOfCells()} cells)')


if __name__ == '__main__':
    main()
