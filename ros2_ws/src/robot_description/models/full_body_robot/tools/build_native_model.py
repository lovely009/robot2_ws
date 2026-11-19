#!/usr/bin/env python3
"""从已核验的 CAD 装配记录生成机械骨架和显示／碰撞网格，不运行仿真。"""
from collections import defaultdict
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import vtk
from vtk.util.numpy_support import vtk_to_numpy
import arm_mounting
import joint_convention

MODEL = Path(__file__).resolve().parents[1]
DIRECTORY = MODEL / 'meshes/visual/native'
ARM_INERTIA_SOURCE = MODEL / 'cad/arm_inertia/urdf/全身实验版_仅双臂质量.urdf'
ARM_LINK_NAMES = {
    f'BN_{side}{part}_link'
    for side in ('Left', 'Right')
    for part in ('Shoulder_1', 'Shoulder_2', 'UpperArm_1', 'UpperArm_2',
                 'UpperArm_3', 'LowerArm_1', 'LowerArm_2', 'Hand_roll', 'Hand_pitch')
}
STL_FACE_LIMIT = 200_000
DISPLAY_FACE_TARGET = 120_000
WRIST_ROTOR_RADIUS = 0.00855

def numbers(values):
    return " ".join(format(float(value), ".14g") for value in values)


def pose(origin):
    return (np.fromstring(origin.get("xyz"), sep=" "),
            Rotation.from_euler("xyz", np.fromstring(origin.get("rpy"), sep=" ")))


def urdf_rpy(rotation):
    """Keep near-gimbal rotations exact when expressing MJCF frames in URDF."""
    matrix = rotation.as_matrix()
    cosine = np.hypot(matrix[0, 0], matrix[1, 0])
    pitch = np.arctan2(-matrix[2, 0], cosine)
    yaw = np.arctan2(matrix[1, 0], matrix[0, 0]) if cosine > 1e-12 else 0.
    sine_yaw, cosine_yaw = np.sin(yaw), np.cos(yaw)
    roll = np.arctan2(sine_yaw * matrix[0, 2] - cosine_yaw * matrix[1, 2],
                      -sine_yaw * matrix[0, 1] + cosine_yaw * matrix[1, 1])
    return np.array([roll, pitch, yaw])


def write_display_mesh(polydata, output):
    output.parent.mkdir(parents=True, exist_ok=True)
    faces = polydata.GetNumberOfPolys()
    if faces >= STL_FACE_LIMIT:
        decimator = vtk.vtkQuadricDecimation()
        decimator.SetInputData(polydata)
        decimator.SetTargetReduction(1 - DISPLAY_FACE_TARGET / faces)
        decimator.Update()
        polydata = decimator.GetOutput()
        faces = polydata.GetNumberOfPolys()
    if not 0 < faces < STL_FACE_LIMIT:
        raise ValueError(f"{output.name}: invalid display mesh ({faces} faces)")
    writer = vtk.vtkSTLWriter()
    writer.SetInputData(polydata)
    writer.SetFileTypeToBinary()
    writer.SetFileName(str(output))
    if not writer.Write():
        raise OSError(f"Could not write {output}")

def circle_support(points, origin, axis, radius):
    relative = points - origin
    radial = relative - np.outer(relative @ axis, axis)
    selected = np.abs(np.linalg.norm(radial, axis=1) - radius) < 0.000025
    if selected.sum() < 8:
        return None
    u = np.cross(axis, [1., 0., 0.] if abs(axis[0]) < .8 else [0., 1., 0.])
    u /= np.linalg.norm(u)
    v = np.cross(axis, u)
    angles = np.sort(np.mod(np.arctan2(radial[selected] @ v, radial[selected] @ u), 2 * np.pi))
    gap = float(np.max(np.diff(np.r_[angles, angles[0] + 2 * np.pi])))
    if gap > np.deg2rad(45):
        return None
    return selected, float(np.rad2deg(gap))


def transform(position=(0.,0.,0.), rotation=None):
    result = np.eye(4)
    result[:3,3] = position
    if rotation is not None:
        result[:3,:3] = rotation
    return result


def urdf_origin(element, matrix):
    element.set('xyz', numbers(matrix[:3,3]))
    element.set('rpy', numbers(urdf_rpy(Rotation.from_matrix(matrix[:3,:3]))))


def read_regions(source):
    reader = vtk.vtkSTLReader();reader.SetFileName(str(source));reader.Update()
    regions = vtk.vtkPolyDataConnectivityFilter();regions.SetInputConnection(reader.GetOutputPort())
    regions.SetExtractionModeToAllRegions();regions.ColorRegionsOn();regions.Update()
    vertices = vtk_to_numpy(regions.GetOutput().GetPoints().GetData()).astype(float)
    ids = vtk_to_numpy(regions.GetOutput().GetPointData().GetScalars()).astype(int)
    return regions, vertices, ids


def extract(regions, indices):
    regions.SetExtractionModeToSpecifiedRegions();regions.InitializeSpecifiedRegionList()
    for index in indices: regions.AddSpecifiedRegion(index)
    clean = vtk.vtkCleanPolyData();clean.SetInputConnection(regions.GetOutputPort());clean.Update()
    result=vtk.vtkPolyData();result.DeepCopy(clean.GetOutput());return result


def append(pieces):
    result=vtk.vtkAppendPolyData()
    for piece in pieces: result.AddInputData(piece)
    result.Update();return result.GetOutput()


def source_tree(alignment, hand_mounting=None):
    source=MODEL/alignment['source_urdf']
    if hashlib.sha256(source.read_bytes()).hexdigest()!=alignment['urdf_sha256']:
        raise ValueError('Repeat STEP calibration after changing the source URDF')
    root=ET.parse(source).getroot()
    root.set('name',MODEL.name)
    # Both exports use the same local arm link frames. Import before the
    # calibrated axis offsets and mounting transforms re-express the inertias.
    arm_links = {
        link.get('name').strip(): link
        for link in ET.parse(ARM_INERTIA_SOURCE).getroot().findall('link')
    }
    for link in root.findall('link'):
        name = link.get('name').strip()
        if name in ARM_LINK_NAMES:
            link.remove(link.find('inertial'))
            link.insert(0, deepcopy(arm_links[name].find('inertial')))
    frames={name:np.asarray(entry['world_transform']) for name,entry in alignment['links'].items()}
    if hand_mounting is not None:
        frames[hand_mounting['hand_link']]=np.asarray(hand_mounting['hand_world_transform'])
    physical={name:value.copy() for name,value in frames.items()}
    axis_file=MODEL/'config/kinematic_axes.json'
    axis_calibration=json.loads(axis_file.read_text()) if axis_file.exists() else {}
    if axis_calibration and axis_calibration['step_sha256']!=alignment['step_sha256']:
        raise ValueError('Joint-axis calibration belongs to another STEP assembly')
    overrides=axis_calibration.get('joints',{})
    for link in root.findall('link'):
        link.set('name',link.get('name').strip())
    for joint in root.findall('joint'):
        name=joint.get('name').strip().removesuffix('_link');joint.set('name',name)
        for tag in ('parent','child'):
            joint.find(tag).set('link',joint.find(tag).get('link').strip())
        child=joint.find('child').get('link')
        if name in overrides:
            item=overrides[name];axis=np.asarray(item['axis']);point=np.asarray(item['point_on_axis_m'])
            axis=axis/np.linalg.norm(axis)
            source_axis=frames[child][:3,:3]@np.fromstring(joint.find('axis').get('xyz'),sep=' ')
            if np.dot(axis,source_axis) < 0:axis=-axis
            physical[child][:3,3]=point+axis*np.dot(frames[child][:3,3]-point,axis)
            joint.find('axis').set('xyz',numbers(physical[child][:3,:3].T@axis))
    arm_mount=arm_mounting.solve(root,physical)
    arm_delta=np.asarray(arm_mount['world_transform'])
    for name in arm_mount['affected_links']:
        frames[name]=arm_delta@frames[name]
        physical[name]=arm_delta@physical[name]
    for joint in root.findall('joint'):
        parent,child=(joint.find(tag).get('link') for tag in ('parent','child'))
        urdf_origin(joint.find('origin'),np.linalg.inv(physical[parent])@physical[child])
    for link in root.findall('link'):
        name=link.get('name');offset=np.linalg.inv(physical[name])@frames[name]
        for item in link:
            if item.tag not in ('visual','collision','inertial'):continue
            origin=item.find('origin')
            if origin is None:origin=ET.SubElement(item,'origin',xyz='0 0 0',rpy='0 0 0')
            position,rotation=pose(origin)
            urdf_origin(origin,offset@transform(position,rotation.as_matrix()))
        for mesh in link.findall('.//mesh'):
            filename=Path(mesh.get('filename')).name.lstrip()
            if not (source.parents[1]/'meshes'/filename).is_file():raise ValueError(filename)
            mesh.set('filename','package://robot_description/models/full_body_robot/'+alignment['source']+'/meshes/'+filename)
    return root,frames,physical,arm_mount


def split_geometry(alignment, metadata, rods, mounting):
    files={};ownership={}
    for old_name,entry in metadata['links'].items():
        name=Path(entry['source']).stem.strip();source=MODEL/metadata['source']/'meshes'/entry['source']
        if hashlib.sha256(source.read_bytes()).hexdigest()!=entry['source_sha256']:raise ValueError('Source mesh changed')
        if alignment['links'][name]['source_sha256']!=entry['source_sha256']:
            raise ValueError(f'{name}: mesh ownership and STEP calibration use different sources')
        regions,vertices,ids=read_regions(source)
        if entry.get('source_region_count',regions.GetNumberOfExtractedRegions())!=regions.GetNumberOfExtractedRegions():raise ValueError('Region count changed')
        parts=deepcopy(entry.get('components',{}));rotors=entry.get('rotor_regions',{})
        native=np.asarray(alignment['links'][name]['world_transform'])
        world=vertices@native[:3,:3].T+native[:3,3]
        # An outer race fitting the rod eye belongs to the rod, not the bearing stud.
        for rod_name,record in rods.items():
            if record['source_body']!=name:continue
            prefix=rod_name.removesuffix('_rod')
            for side,end in record['endpoints'].items():
                center=np.asarray(end['center_m']);axis=np.asarray(end['rod_end_cylinder']['axis'])
                radius=end['rod_end_cylinder']['radius_m']
                mate=prefix+'_'+side
                for region in parts[mate][:]:
                    points=world[ids==region]
                    if np.max(np.linalg.norm(points-center,axis=1))>max(.015,2*radius):continue
                    circle=circle_support(points,center,axis,radius)
                    sphere=np.count_nonzero(abs(np.linalg.norm(points-center,axis=1)-end['ball_radius_m'])<0.000025)
                    stations=(points-center)@axis
                    if circle is not None and sphere>=30 and abs((stations.min()+stations.max())/2)<0.000025:
                        # A convex ball stud can also have this outer cylinder.
                        # Only a concave bearing seat is an outer race.
                        piece=extract(regions,[region])
                        xyz=vtk_to_numpy(piece.GetPoints().GetData()).astype(float)
                        xyz=xyz@native[:3,:3].T+native[:3,3]
                        triangles=xyz[vtk_to_numpy(piece.GetPolys().GetData()).reshape(-1,4)[:,1:]]
                        surface=np.all(abs(np.linalg.norm(triangles-center,axis=2)-end['ball_radius_m'])<0.000025,axis=1)
                        normals=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
                        signed=np.sum(normals*(triangles.mean(1)-center),axis=1)[surface]
                        if np.count_nonzero(signed < -1e-14)<8 or np.any(signed>1e-14):
                            continue
                        parts[mate].remove(region);parts[rod_name].append(region)
        if name=='Base_link':
            parts={f'hips_{side}_{part}':indices for side,item in mounting['sides'].items() for part,indices in item['regions'].items()}
        moved=[i for indices in parts.values() for i in indices]+[i for indices in rotors.values() for i in indices]
        if len(moved)!=len(set(moved)):raise ValueError('Duplicate source component ownership')
        extras=defaultdict(list)
        for side,indices in rotors.items():
            group='left_wrist' if 'Left' in name else 'right_wrist';prefix=group+'_flange_'+side
            candidate=rods[prefix+'_rod']['motor_axis_candidates']
            if len(candidate)!=1:raise ValueError('Ambiguous rotor axis')
            axis=np.asarray(candidate[0]['axis']);origin=np.asarray(candidate[0]['point_on_axis_m'])
            cylinder=vtk.vtkCylinder();cylinder.SetCenter(*(native[:3,:3].T@(origin-native[:3,3])))
            cylinder.SetAxis(*(native[:3,:3].T@axis));cylinder.SetRadius(WRIST_ROTOR_RADIUS)
            clip=vtk.vtkClipPolyData();clip.SetInputData(extract(regions,indices));clip.SetClipFunction(cylinder)
            clip.InsideOutOn();clip.GenerateClippedOutputOn();clip.Update()
            if not clip.GetOutput().GetNumberOfCells() or not clip.GetClippedOutput().GetNumberOfCells():raise ValueError('Rotor split failed')
            rotor=vtk.vtkPolyData();rotor.DeepCopy(clip.GetOutput());extras[prefix+'_flange'].append(rotor)
            cover=vtk.vtkPolyData();cover.DeepCopy(clip.GetClippedOutput());extras[name].append(cover)
        fixed=sorted(set(range(regions.GetNumberOfExtractedRegions()))-set(moved))
        for part,indices in [(name,fixed),*parts.items()]:
            if not indices and not extras[part]:
                continue
            output=DIRECTORY/(part+'.STL')
            piece=append([extract(regions,indices),*extras[part]])
            write_display_mesh(piece,output)
            files[part]=output.relative_to(MODEL).as_posix()
        ownership[name]={'source':entry['source'],'source_sha256':entry['source_sha256'],'parts':parts,'rotor_regions':rotors,'fixed_regions':fixed}
    return files,ownership



def build():
    alignment = json.loads((MODEL / 'config/step_alignment.json').read_text())
    report = json.loads((MODEL / 'cad/assembly_audit/2026-11-13-step-linkages.json').read_text())
    mounting = json.loads((MODEL / 'config/hips_mount_alignment.json').read_text())
    hand = json.loads((MODEL / 'config/left_hand_mount_alignment.json').read_text())
    step = hashlib.sha256((MODEL / 'cad/全身不带头.STEP').read_bytes()).hexdigest()
    if not step == alignment['step_sha256'] == report['source_step_sha256'] == mounting['step_sha256'] == hand['source_step_sha256']:
        raise ValueError('装配记录与当前 STEP 不一致，须重新核验机械装配')
    source, _, _, _ = source_tree(alignment, hand)
    metadata = json.loads((MODEL / 'config/display_meshes.json').read_text())
    files, _ = split_geometry(alignment, metadata, report['rods'], mounting)
    rotation, coordinates = joint_convention.load(MODEL)
    current = ET.parse(MODEL / 'urdf/full_body_robot.urdf').getroot()
    for joint in source.findall('joint'):
        name, sign = coordinates[joint.get('name')]
        limit = current.find(f"joint[@name='{name}']/limit")
        lower, upper = joint_convention.signed_interval(float(limit.get('lower')), float(limit.get('upper')), sign)
        joint.find('limit').attrib.update(limit.attrib, lower=str(lower), upper=str(upper))
    for link in source.findall('link'):
        for mesh in link.findall('.//mesh'):
            mesh.set('filename', 'package://robot_description/models/full_body_robot/' + files[link.get('name')])
    joint_convention.convert_urdf(source, rotation, coordinates)
    ET.indent(source, space='  ')
    content = ET.tostring(source, encoding='unicode') + '\n'
    for suffix in ('', '.mapping'):
        (MODEL / f'urdf/full_body_robot{suffix}.urdf').write_text(content)
    print('已生成 37 轴机械骨架与 CAD 显示／碰撞网格；保留当前限位，未进行实机标定')


if __name__ == '__main__':
    build()
