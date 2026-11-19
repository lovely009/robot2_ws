#!/usr/bin/env python3
"""Build an inspection URDF without changing the control model.

Fits bolt-circle bores in the original STL; motor datums are stator-fixed
installation references at model q=0, not live rotor transforms. Wrist ends
remain unassigned candidates until the assembly is identified.
Run this file to regenerate cad/frames/ outputs (numpy/scipy/PyYAML).
"""
from pathlib import Path
from collections import defaultdict
import copy
import csv
import hashlib
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parent))
from build_simplified_collision_meshes import read_binary_stl

MODEL = Path(__file__).resolve().parent.parent
OUTPUT = MODEL / "cad/frames"

def circles(path):
 t=read_binary_stl(path); v,ids=np.unique(np.round(t.reshape(-1,3),7),axis=0,return_inverse=True);ids=ids.reshape(-1,3)
 n=np.cross(t[:,1]-t[:,0],t[:,2]-t[:,0]);length=np.linalg.norm(n,axis=1);valid=length>1e-14;n[valid]/=length[valid,None]
 # Plane normals have a canonical sign, so both sides group consistently.
 s=np.where(n[np.arange(len(n)),np.abs(n).argmax(axis=1)]<0,-1,1);n*=s[:,None]
 d=np.einsum('ij,ij->i',n,t[:,0]);key=np.column_stack([np.round(n,4),np.round(d,6)])
 _,groups=np.unique(key,axis=0,return_inverse=True)
 counts=np.bincount(groups);out=[]
 for g in np.where(counts>=6)[0]:
  tri=ids[groups==g];edges=np.sort(np.concatenate([tri[:,[0,1]],tri[:,[1,2]],tri[:,[2,0]]]),axis=1)
  edges,c=np.unique(edges,axis=0,return_counts=True);edges=edges[c==1];adj=defaultdict(set)
  for a,b in edges:adj[a].add(b);adj[b].add(a)
  while adj:
   start=next(iter(adj));stack=[start];comp=set()
   while stack:
    a=stack.pop()
    if a in comp:continue
    comp.add(a);stack.extend(adj[a]-comp)
   closed=all(len(adj[a])==2 for a in comp)
   for a in comp:del adj[a]
   if not closed or len(comp)<8:continue
   p=v[list(comp)];origin=p.mean(axis=0);_,_,vt=np.linalg.svd(p-origin,full_matrices=False);xy=(p-origin)@vt[:2].T
   co=np.linalg.lstsq(np.column_stack([2*xy,np.ones(len(xy))]),(xy**2).sum(axis=1),rcond=None)[0]
   rad=np.sqrt(max(0,co[2]+co[:2]@co[:2]));err=np.max(np.abs(np.linalg.norm(xy-co[:2],axis=1)-rad));plane=np.max(np.abs((p-origin)@vt[2]))
   if err<2e-6 and plane<2e-6 and .0004<rad<.04:
    center=origin+co[:2]@vt[:2];normal=vt[2];normal*=np.sign(normal[np.abs(normal).argmax()]);out.append(dict(center=center.tolist(),normal=normal.tolist(),radius=float(rad),error=float(err),vertices=len(comp)))
 return out


def basis(normal, reference=None):
    """Z follows the axis; project the locating ray (or a parent axis) for X."""
    z = np.asarray(normal, dtype=float)
    z = z / np.linalg.norm(z)
    reference = np.array([1., 0., 0.]) if reference is None else np.asarray(reference, dtype=float)
    if abs(reference @ z) > .95:
        reference = np.array([0., 1., 0.])
    x = reference - (reference @ z) * z
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def fit_circle(points, normal):
    points = np.asarray(points, dtype=float)
    if len(points) < 3:
        raise ValueError('A circle needs three non-collinear points')
    rotation = basis(normal)
    origin = points.mean(axis=0)
    local = (points - origin) @ rotation
    matrix = np.column_stack([2 * local[:, :2], np.ones(len(points))])
    coefficient, _, rank, _ = np.linalg.lstsq(matrix, (local[:, :2] ** 2).sum(axis=1), rcond=None)
    if rank != 3:
        raise ValueError('Collinear bolt centers')
    center = origin + rotation[:, :2] @ coefficient[:2]
    radius = float(np.sqrt(coefficient[2] + coefficient[:2] @ coefficient[:2]))
    radial_error = np.abs(np.linalg.norm(local[:, :2] - coefficient[:2], axis=1) - radius)
    residual = float(max(radial_error.max(), np.abs(local[:, 2]).max()))
    return center, radius, residual


def transform(element):
    origin = element.find('origin')
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy'), sep=' ')).as_matrix()
    result[:3, 3] = np.fromstring(origin.get('xyz'), sep=' ')
    return result


def add_frame(root, name, parent, xyz, rotation):
    root.append(ET.Element('link', name=name))
    joint = ET.SubElement(root, 'joint', name=f'{name}_fixed', type='fixed')
    ET.SubElement(joint, 'parent', link=parent)
    ET.SubElement(joint, 'child', link=name)
    import warnings
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', UserWarning)  # Euler gimbal lock has an equivalent valid RPY.
        rpy = Rotation.from_matrix(rotation).as_euler('xyz')
    ET.SubElement(joint, 'origin', xyz=' '.join(f'{v:.12g}' for v in xyz),
                  rpy=' '.join(f'{v:.12g}' for v in rpy))
    ET.indent(joint, space='  ', level=1)
    joint.tail = '\n'


def generate():
    source = MODEL / 'urdf/half_body_robot.urdf'
    original = ET.parse(source).getroot()
    root = copy.deepcopy(original)
    joints = original.findall('joint')
    poses = {'base_link': np.eye(4)}
    joint_rows = []
    for joint in joints:
        name = joint.get('name')
        parent = joint.find('parent').get('link')
        child = joint.find('child').get('link')
        poses[child] = poses[parent] @ transform(joint)
        axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
        axis /= np.linalg.norm(axis)
        frame = f'{name}_axis'
        add_frame(root, frame, child, np.zeros(3), basis(axis))
        joint_rows.append(dict(joint=name, frame=frame, parent=child,
            base_xyz=poses[child][:3, 3].tolist(), base_positive_axis=(poses[child][:3, :3] @ axis).tolist(),
            zero='existing_CAD_assembly_pose_not_encoder_zero'))

    seeds_path = OUTPUT / 'datum_seeds.yaml'
    seeds = yaml.safe_load(seeds_path.read_text())['datums']
    mesh_circles = {}
    rows = []
    sources = {source, seeds_path, Path(__file__).resolve(),
               MODEL / 'tools/build_simplified_collision_meshes.py',
               MODEL / 'config/motor_driver.yaml', MODEL / 'config/transmissions.yaml'}
    for spec in seeds:
        parent = spec['parent']
        mesh = MODEL / 'meshes' / f'{parent}.STL'
        sources.add(mesh)
        if parent not in mesh_circles:
            mesh_circles[parent] = circles(mesh)
        seed = np.array(spec['seed_xyz'])
        z = np.array(spec['outward_z'])
        z /= np.linalg.norm(z)
        holes = []
        for circle in mesh_circles[parent]:
            p = np.array(circle['center'])
            delta = p - seed
            if (abs(circle['radius'] - spec['bore_radius']) < 5e-6
                and abs(np.dot(delta, z)) < 2e-5
                and abs(np.linalg.norm(np.cross(delta, z)) - spec['pcd'] / 2) < 5e-5
                and abs(np.dot(circle['normal'], z)) > .99999
                and not any(np.linalg.norm(p - q) < 1e-6 for q in holes)):
                holes.append(p)
        verified = spec['status'] == 'bolt_circle_verified'
        if verified and len(holes) < 4:
            raise ValueError(f"Motor {spec['motor_id']}: insufficient bolt-circle evidence ({len(holes)})")
        center, radius, residual = seed, None, None
        if len(holes) >= 3:
            points = np.array(holes)
            _, _, vectors = np.linalg.svd(points - points.mean(axis=0), full_matrices=False)
            fitted_normal = vectors[-1]
            z = fitted_normal if fitted_normal @ z >= 0 else -fitted_normal
            center, radius, residual = fit_circle(holes, z)
            if abs(2 * radius - spec['pcd']) > 5e-5 or residual > 5e-5:
                raise ValueError(f"{parent}: bolt-circle fit exceeds 0.05 mm")
        frame = spec.get('frame', f"motor_{spec['motor_id']:02d}_mount_reference" if verified else '')
        locating_holes = []
        for circle in mesh_circles[parent]:
            p = np.array(circle['center'])
            delta = p - center
            if (abs(circle['radius'] - .0015) < 5e-6
                and abs(delta @ z) < 2e-5
                and abs(np.linalg.norm(np.cross(delta, z)) - spec['pcd'] / 2) < 5e-5
                and abs(np.dot(circle['normal'], z)) > .99999
                and not any(np.linalg.norm(p - q) < 1e-6 for q in locating_holes)):
                locating_holes.append(p)
        locator = locating_holes[0] if verified and len(locating_holes) == 1 else None
        rotation = basis(z, None if locator is None else locator - center)
        add_frame(root, frame, parent, center, rotation)
        base_pose = poses[parent]
        base_xyz = base_pose[:3, :3] @ center + base_pose[:3, 3]
        base_z = base_pose[:3, :3] @ z
        row = dict(frame=frame, motor_id=spec['motor_id'], parent=parent, model=spec['model'],
            status=spec['status'], parent_xyz=center.tolist(), parent_rpy=Rotation.from_matrix(rotation).as_euler('xyz').tolist(),
            parent_outward_z=z.tolist(), base_xyz=base_xyz.tolist(), base_outward_z=base_z.tolist(),
            pcd_mm=spec['pcd'] * 1000, observed_holes=len(holes),
            hole_centers_parent=[p.tolist() for p in holes],
            measured_pcd_mm=None if radius is None else 2 * radius * 1000,
            pcd_error_mm=None if radius is None else abs(2 * radius - spec['pcd']) * 1000,
            fit_residual_mm=None if residual is None else residual * 1000,
            clocking_reference='X_toward_diameter_3_locating_hole' if locator is not None else 'parent_axis_projection_only',
            clocking_status='locator_verified' if locator is not None else 'unverified',
            locating_hole_parent=None if locator is None else locator.tolist(),
            encoder_zero='unverified', command_direction='unverified')
        if verified:
            joint = joints[spec['joint_index'] - 1]
            child = joint.find('child').get('link')
            axis = poses[child][:3, :3] @ np.fromstring(joint.find('axis').get('xyz'), sep=' ')
            axis /= np.linalg.norm(axis)
            delta = base_xyz - poses[child][:3, 3]
            row.update(joint=joint.get('name'),
                flange_center_to_joint_axis_mm=float(np.linalg.norm(np.cross(delta, axis)) * 1000),
                axis_angle_deg=float(np.degrees(np.arccos(np.clip(abs(base_z @ axis), 0, 1)))),
                axial_offset_mm=float(delta @ axis * 1000),
                outward_vs_joint_sign=1 if base_z @ axis >= 0 else -1)
        rows.append(row)
    root.set('name', 'half_body_robot_frame_inspection')
    (OUTPUT / 'half_body_robot.frames.urdf').write_text(
        '<?xml version="1.0"?>\n<!-- Inspection only; fixed mount references are not live rotor frames. -->\n'
        + ET.tostring(root, encoding='unicode') + '\n')
    hashes = {str(p.relative_to(MODEL)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(sources)}
    report = dict(length_unit='m', angle_unit='rad', zero='unchanged_CAD_assembly_pose',
        source_sha256=hashes, joint_frames=joint_rows, motor_datums=rows)
    (OUTPUT / 'measurements.yaml').write_text(yaml.safe_dump(report, sort_keys=False))
    fields = ['frame', 'motor_id', 'parent', 'model', 'status', 'parent_xyz', 'parent_rpy',
              'parent_outward_z', 'base_xyz', 'base_outward_z', 'observed_holes', 'measured_pcd_mm',
              'pcd_error_mm', 'fit_residual_mm', 'joint', 'flange_center_to_joint_axis_mm', 'axis_angle_deg',
              'axial_offset_mm', 'outward_vs_joint_sign', 'clocking_status', 'encoder_zero', 'command_direction']
    with (OUTPUT / 'motor_frames.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fields, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)
    with (OUTPUT / 'joint_frames.csv').open('w') as stream:
        writer = csv.DictWriter(stream, list(joint_rows[0]))
        writer.writeheader()
        writer.writerows(joint_rows)
    driver = yaml.safe_load((MODEL / 'config/motor_driver.yaml').read_text())['motor_hardware']
    transmissions = yaml.safe_load((MODEL / 'config/transmissions.yaml').read_text())['transmissions']
    joint_frames = {row['joint']: row['frame'] for row in joint_rows}
    mounts = {row['motor_id']: row for row in rows if row['motor_id'] is not None}
    relations = {}
    for actuator, motor in driver['motors'].items():
        groups = [name for name, group in transmissions.items() if actuator in group['actuators']]
        if len(groups) > 1:
            raise ValueError(f'{actuator}: multiple transmission groups')
        group_name = groups[0] if groups else None
        related_joints = transmissions[group_name]['joints'] if groups else [motor['joint']]
        mount = mounts.get(motor['id'])
        relation = ('differential_group' if groups else
                    'linkage' if mount['flange_center_to_joint_axis_mm'] > .05 else 'coaxial_axes')
        relations[motor['id']] = dict(actuator=actuator, motor_type=motor['type'],
            relation=relation, transmission_group=group_name,
            joint_axis_tfs=[joint_frames[joint] for joint in related_joints],
            mount_reference_tf=None if mount is None else mount['frame'])
    (OUTPUT / 'motor_tf.yaml').write_text(
        '# TF 与配置电机编号的关系；不是电机角度到关节角度的换算或实物标定。\n'
        '# mount_reference_tf 固定在安装连杆；腕部仅关联差动组，不绑定端面候选。\n'
        + yaml.safe_dump(dict(base_frame='base_link', urdf='half_body_robot.frames.urdf',
                             motors=relations), sort_keys=False))
    print(f'Wrote {len(joint_rows)} joint axes, {sum(r["status"] == "bolt_circle_verified" for r in rows)} '
          f'motor mount references and {sum(r["motor_id"] is None for r in rows)} unassigned wrist candidates')
    return report


if __name__ == '__main__':
    generate()
