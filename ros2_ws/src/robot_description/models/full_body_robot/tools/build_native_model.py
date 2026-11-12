#!/usr/bin/env python3
"""Build the articulated full model in the verified STEP assembly frames."""
from collections import defaultdict
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.ndimage import binary_erosion, label
from scipy.spatial.transform import Rotation
import vtk
from vtk.util.numpy_support import vtk_to_numpy
import yaml

import audit_step_linkages as audit
import hips_mounting as hips
import native_names
import build_mujoco_model as shared
from assembly_fingerprint import geometry_fingerprint

MODEL = Path(__file__).resolve().parents[1]
DIRECTORY = MODEL / 'mujoco/visual/native'


def transform(position=(0.,0.,0.), rotation=None):
    result = np.eye(4)
    result[:3,3] = position
    if rotation is not None:
        result[:3,:3] = rotation
    return result


def attrs(matrix):
    return shared.pose_attributes(matrix[:3,3], Rotation.from_matrix(matrix[:3,:3]))


def urdf_origin(element, matrix):
    element.set('xyz', shared.numbers(matrix[:3,3]))
    element.set('rpy', shared.numbers(shared.urdf_rpy(Rotation.from_matrix(matrix[:3,:3]))))


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


def source_tree(alignment):
    source=MODEL.parent/'Robot_body1/urdf/Robot_body1.urdf'
    if hashlib.sha256(source.read_bytes()).hexdigest()!=alignment['urdf_sha256']:
        raise ValueError('Repeat STEP calibration after changing the source URDF')
    root=ET.parse(source).getroot()
    root.set('name',MODEL.name)
    frames={name:np.asarray(entry['world_transform']) for name,entry in alignment['links'].items()}
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
            joint.find('axis').set('xyz',shared.numbers(physical[child][:3,:3].T@axis))
    for joint in root.findall('joint'):
        parent,child=(joint.find(tag).get('link') for tag in ('parent','child'))
        urdf_origin(joint.find('origin'),np.linalg.inv(physical[parent])@physical[child])
    for link in root.findall('link'):
        name=link.get('name');offset=np.linalg.inv(physical[name])@frames[name]
        for item in link:
            if item.tag not in ('visual','collision','inertial'):continue
            origin=item.find('origin')
            if origin is None:origin=ET.SubElement(item,'origin',xyz='0 0 0',rpy='0 0 0')
            position,rotation=shared.pose(origin)
            urdf_origin(origin,offset@transform(position,rotation.as_matrix()))
        for mesh in link.findall('.//mesh'):
            filename=Path(mesh.get('filename')).name.lstrip()
            if not (MODEL.parent/'Robot_body1/meshes'/filename).is_file():raise ValueError(filename)
            mesh.set('filename','package://robot_description/models/Robot_body1/meshes/'+filename)
    return root,frames,physical


def split_geometry(alignment, metadata, rods, mounting):
    files={};ownership={}
    for old_name,entry in metadata['links'].items():
        name=Path(entry['source']).stem.strip();source=MODEL.parent/metadata['source']/'meshes'/entry['source']
        if hashlib.sha256(source.read_bytes()).hexdigest()!=entry['source_sha256']:raise ValueError('Source mesh changed')
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
                    circle=audit.circle_support(points,center,axis,radius)
                    sphere=np.count_nonzero(abs(np.linalg.norm(points-center,axis=1)-end['ball_radius_m'])<audit.SURFACE_TOL)
                    stations=(points-center)@axis
                    if circle is not None and sphere>=30 and abs((stations.min()+stations.max())/2)<audit.CENTER_TOL:
                        # A convex ball stud can also have this outer cylinder.
                        # Only a concave bearing seat is an outer race.
                        piece=extract(regions,[region])
                        xyz=vtk_to_numpy(piece.GetPoints().GetData()).astype(float)
                        xyz=xyz@native[:3,:3].T+native[:3,3]
                        triangles=xyz[vtk_to_numpy(piece.GetPolys().GetData()).reshape(-1,4)[:,1:]]
                        surface=np.all(abs(np.linalg.norm(triangles-center,axis=2)-end['ball_radius_m'])<audit.SURFACE_TOL,axis=1)
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
            cylinder.SetAxis(*(native[:3,:3].T@axis));cylinder.SetRadius(shared.WRIST_ROTOR_RADIUS)
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
            shared.write_display_mesh(append([extract(regions,indices),*extras[part]]),output)
            files[part]=output.relative_to(MODEL).as_posix()
        ownership[name]={'source':entry['source'],'source_sha256':entry['source_sha256'],'parts':parts,'rotor_regions':rotors,'fixed_regions':fixed}
    return files,ownership


def build():
    mounting=hips.solve()
    if not hips.validate(MODEL/'mujoco/full_body_robot.hips-mounted.xml')['all_passed']:raise ValueError('Fixture baseline is invalid')
    alignment=json.loads((MODEL/'config/step_alignment.json').read_text())
    metadata=json.loads((MODEL/'config/display_meshes.json').read_text())
    report=audit.audit();rods=report['rods']
    source,native,frames=source_tree(alignment)
    aliases,link_aliases=native_names.names()
    topology=native_names.rename_tree(yaml.safe_load((MODEL/'config/motor_topology.yaml').read_text()),aliases)
    limits=native_names.rename_tree(yaml.safe_load((MODEL/'config/simulation_limits.yaml').read_text()),aliases)
    limits.setdefault('legacy_scalar_caps',deepcopy(limits['joint_limits']))
    # Old asymmetric intervals use different joint frames and zero poses.
    # Their magnitude remains a provisional search cap, not collision evidence.
    for number,motor in topology['motors'].items():
        if 'joint' not in motor:continue
        bound=limits['legacy_scalar_caps'][motor['joint']]
        magnitude=max(abs(bound['lower']),abs(bound['upper']))
        limits['joint_limits'][motor['joint']].update(lower=-magnitude,upper=magnitude)
        limits['actuator_limits'][f'motor_{number:02d}'].update(lower=-magnitude,upper=magnitude)
    limits['preview_limit_policy']='Series-joint search caps use the legacy maximum magnitude symmetrically in native coordinates; coupled output and auxiliary actuator caps retained. Collision/hardware travel remains uncalibrated.'
    joints={j.get('name'):j for j in source.findall('joint')}
    for name,joint in joints.items():
        bound=limits['joint_limits'][name]
        joint.find('limit').set('lower',str(bound['lower']))
        joint.find('limit').set('upper',str(bound['upper']))
    files,ownership=split_geometry(alignment,metadata,rods,mounting)
    root=ET.Element('mujoco',model='Full body STEP-calibrated articulated preview')
    root.append(ET.Comment(' Native CAD assembly; simulation geometry preview. Collision travel and hardware calibration are not implied. '))
    ET.SubElement(root,'compiler',angle='radian',meshdir='..',fusestatic='false')
    ET.SubElement(root,'option',timestep='.002',gravity='0 0 0',integrator='implicitfast')
    ET.SubElement(root,'statistic',center='0 .08 -.28',extent='1.6')
    ET.SubElement(ET.SubElement(root,'visual'),'global',azimuth='90',elevation='-8',offwidth='1280',offheight='960')
    default=ET.SubElement(root,'default')
    ET.SubElement(default,'joint',type='hinge',damping='.1',armature='.01')
    ET.SubElement(default,'geom',contype='0',conaffinity='0')
    ET.SubElement(default,'site',size='.003',group='2',rgba='0 0 0 0')
    ET.SubElement(default,'position',kp='12',kv='1.2')
    assets=ET.SubElement(root,'asset');world=ET.SubElement(root,'worldbody')
    ET.SubElement(world,'light',pos='0 1 1.8',dir='0 -.45 -1');ET.SubElement(world,'light',pos='0 -1 .8',dir='0 .55 -1')
    bodies={'Base_link':ET.SubElement(world,'body',name='Base_link')}
    for joint in source.findall('joint'):
        parent,child=(joint.find(t).get('link') for t in ('parent','child'))
        body=ET.SubElement(bodies[parent],'body',name=child,**attrs(np.linalg.inv(frames[parent])@frames[child]))
        lower,upper=(limits['joint_limits'][joint.get('name')][k] for k in ('lower','upper'))
        if lower==upper:lower,upper=-shared.LOCKED_SOLVER_RANGE,shared.LOCKED_SOLVER_RANGE
        ET.SubElement(body,'joint',name=joint.get('name'),axis=joint.find('axis').get('xyz'),limited='true',range=shared.numbers((lower,upper)))
        bodies[child]=body
    def visual(body,name,file,pose):
        ET.SubElement(assets,'mesh',name=name,file=file)
        ET.SubElement(body,'geom',name=name,type='mesh',mesh=name,mass='0',contype='0',conaffinity='0',rgba='.84 .85 .86 1',**attrs(pose))
    for link in source.findall('link'):
        name=link.get('name');inertial=link.find('inertial');position,rotation=shared.pose(inertial.find('origin'))
        inertia=inertial.find('inertia');components=[float(inertia.get(k)) for k in ('ixx','iyy','izz','ixy','ixz','iyz')]
        xx,yy,zz,xy,xz,yz=components
        tensor=rotation.as_matrix()@np.array([[xx,xy,xz],[xy,yy,yz],[xz,yz,zz]])@rotation.as_matrix().T
        values=tensor[0,0],tensor[1,1],tensor[2,2],tensor[0,1],tensor[0,2],tensor[1,2]
        ET.SubElement(bodies[name],'inertial',mass=inertial.find('mass').get('value'),fullinertia=shared.numbers(values),pos=shared.numbers(position))
        visual(bodies[name],name+'_robot_body',files[name],np.linalg.inv(frames[name])@native[name])
    tendons=ET.SubElement(root,'tendon');equalities=ET.SubElement(root,'equality')
    flanges={};actuator_by_motor={};geometry_poses={}
    for number,motor in topology['motors'].items():
        if 'joint' in motor:actuator_by_motor[number]=[{'name':f'motor_{number:02d}','joint':motor['joint']}]
    for group_name,group in topology['differential_groups'].items():
        reference=joints[group['joints'][0]].find('parent').get('link')
        output=joints[group['joints'][-1]].find('child').get('link')
        local=ET.parse(MODEL/'mujoco'/f'{group_name}_differential.xml').getroot()
        auxiliary={}
        for rod_name,record in rods.items():
            if not rod_name.startswith(group_name+'_'):continue
            prefix=rod_name.removesuffix('_rod')
            if len(record['motor_axis_candidates'])!=1:raise ValueError('Ambiguous motor hinge')
            candidate=record['motor_axis_candidates'][0];axis=np.asarray(candidate['axis']);origin=np.asarray(candidate['point_on_axis_m'])
            delta={part:np.eye(4) for part in ('flange','platform','rod')}
            if group_name=='hips':
                side=prefix.removeprefix('hips_');delta={part:np.asarray(mounting['sides'][side][part+'_transform']) for part in delta}
            centers={part:hips.point(delta[part],record['endpoints'][tag]['center_m']) for part,tag in (('flange','flange'),('platform','platform'))}
            start,end=centers['flange'],centers['platform'];origin=origin+axis*np.dot(start-origin,axis)
            flange_world=transform(origin,frames[reference][:3,:3]);flange_body=ET.SubElement(bodies[reference],'body',name=prefix+'_flange',**attrs(np.linalg.inv(frames[reference])@flange_world))
            matching=[]
            suffix=prefix.removeprefix(group_name+'_')
            for actuator in local.findall('actuator/position'):
                native_joint=aliases.get(actuator.get('joint'),actuator.get('joint'))
                if native_joint in joints:continue
                token=actuator.get('joint').removesuffix('_x')
                if token==suffix or token.removeprefix('flange_')==suffix:matching.append(actuator)
            if len(matching)!=1:raise ValueError(f'{prefix}: ambiguous actuator binding')
            joint_name=group_name+'_'+matching[0].get('joint')
            ET.SubElement(flange_body,'joint',name=joint_name,axis=shared.numbers(flange_world[:3,:3].T@axis))
            ET.SubElement(flange_body,'inertial',pos='0 0 0',mass='.03',diaginertia='.00001 .00001 .00001')
            source_frame=native[record['source_body']]
            for part,body,body_frame in (('flange',flange_body,flange_world),('platform',bodies[output],frames[output])):
                target=delta[part]@source_frame
                if prefix+'_'+part not in files:
                    if part!='platform':raise ValueError('Missing crank geometry')
                    # Some export groups contain only the outer race: the stud
                    # is already part of the physical output mesh.
                    output_entry=alignment['links'][output]
                    _,output_points,_=read_regions(MODEL.parent/'Robot_body1/meshes'/output_entry['source'])
                    output_points=output_points@native[output][:3,:3].T+native[output][:3,3]
                    sphere=record['endpoints']['platform']
                    if np.count_nonzero(abs(np.linalg.norm(output_points-end,axis=1)-sphere['ball_radius_m'])<audit.SURFACE_TOL)<30:
                        raise ValueError(f'{prefix}: no platform bearing stud in output mesh')
                    continue
                visual(body,prefix+'_'+part+'_visual',files[prefix+'_'+part],np.linalg.inv(body_frame)@target)
                geometry_poses[prefix+'_'+part+'_visual']=target.tolist()
            ET.SubElement(flange_body,'site',name=prefix+'_motor_ball',pos=shared.numbers(flange_world[:3,:3].T@(start-origin)))
            ET.SubElement(bodies[output],'site',name=prefix+'_platform_ball',pos=shared.numbers(hips.point(np.linalg.inv(frames[output]),end)))
            normal=delta['rod'][:3,:3]@record['endpoints']['flange']['rod_end_cylinder']['axis']
            rod_rotation=hips.direction_frame(end-start,normal)
            rod_world=transform(start,rod_rotation);rod_body=ET.SubElement(flange_body,'body',name=rod_name,**attrs(np.linalg.inv(flange_world)@rod_world))
            for tag,vector in (('x','1 0 0'),('y','0 1 0')):ET.SubElement(rod_body,'joint',name=rod_name+'_'+tag,axis=vector,damping='.001',armature='.00005')
            length=np.linalg.norm(end-start)
            ET.SubElement(rod_body,'inertial',pos=shared.numbers((0,0,length/2)),mass='.01',diaginertia=shared.numbers((.01*length**2/12,.01*length**2/12,1e-7)))
            target=delta['rod']@source_frame
            visual(rod_body,rod_name+'_visual',files[rod_name],np.linalg.inv(rod_world)@target)
            geometry_poses[rod_name+'_visual']=target.tolist()
            tip=rod_name+'_tip';ET.SubElement(rod_body,'site',name=tip,pos=shared.numbers((0,0,length)))
            ET.SubElement(equalities,'connect',name=rod_name+'_end',site1=tip,site2=prefix+'_platform_ball',solref='.006 1',solimp='.999 .9999 .001')
            tendon=ET.SubElement(tendons,'spatial',name=rod_name,rgba='0 0 0 0')
            ET.SubElement(tendon,'site',site=prefix+'_motor_ball');ET.SubElement(tendon,'site',site=prefix+'_platform_ball')
            auxiliary[matching[0].get('name')]=(joint_name,rod_name)
        controls=[]
        for item in local.findall('actuator/position'):
            name=group_name+'_'+item.get('name');canonical=aliases.get(item.get('joint'),item.get('joint'))
            if canonical in joints:joint_name=canonical
            else:
                joint_name,rod_name=auxiliary[item.get('name')];flanges[name]=rod_name
            controls.append({'name':name,'joint':joint_name})
        if len(controls)!=len(group['motor_numbers']):raise ValueError('Invalid motor group')
        actuator_by_motor[group['motor_numbers'][0]]=controls
    actuators=ET.SubElement(root,'actuator')
    for number in sorted(actuator_by_motor):
        for control in actuator_by_motor[number]:
            bound=limits['actuator_limits'][control['name']];lower,upper=bound['lower'],bound['upper']
            if lower==upper:lower,upper=-shared.LOCKED_SOLVER_RANGE,shared.LOCKED_SOLVER_RANGE
            ET.SubElement(actuators,'position',**control,ctrlrange=shared.numbers((lower,upper)))
    config={'model':MODEL.name,'joints':list(joints),'joint_aliases':aliases,'flanges':flanges,
            'joint_limits':{j:[limits['joint_limits'][j][k] for k in ('lower','upper')] for j in joints},'coupled_limits':{}}
    custom=ET.SubElement(root,'custom');ET.SubElement(custom,'text',name='robot_sim_control_mode',data='viewer_position')
    control_text=ET.SubElement(custom,'text',name='robot_sim_joint_position',data=json.dumps(config,separators=(',',':')))
    compiled=deepcopy(root);compiled.find('compiler').set('meshdir',str(MODEL))
    model=mujoco.MjModel.from_xml_string(ET.tostring(compiled,encoding='unicode'))
    data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    if (model.nv,model.nu,model.neq)!=(67,37,10) or np.max(abs(data.efc_pos))>.000025:raise ValueError('Invalid native linkage closure')
    # Scalar caps remain a provisional search domain; old collision grids are discarded.
    from robot_transmission.mujoco_positions import JointPositionTargets
    controller=JointPositionTargets(model,config)
    workspaces={};step=np.deg2rad(1.)
    for name,group in topology['differential_groups'].items():
        pair=group['joints'];bounds=np.asarray([config['joint_limits'][j] for j in pair])
        first=np.ceil(bounds[:,0]/step).astype(int);last=np.floor(bounds[:,1]/step).astype(int)
        mask=np.zeros(tuple(last-first+1),dtype=bool)
        for index in np.ndindex(mask.shape):
            targets=dict.fromkeys(joints,0.);targets.update({j:float((first[k]+index[k])*step) for k,j in enumerate(pair)})
            try:controller.apply(data,list(targets),list(targets.values()))
            except ValueError:continue
            mask[index]=True
        mask=binary_erosion(mask,structure=np.ones((3,3)),border_value=1)
        labels,_=label(mask);zero=tuple(-first)
        component=labels[zero]
        if not component:raise ValueError(f'{name}: neutral is outside the interior of the reachable preview domain')
        mask=labels==component
        workspaces[name]={'joints':pair,'lower':(first*step).tolist(),'step':float(step),'valid_rows':[''.join('1' if v else '0' for v in row) for row in mask]}
    limits['coupled_limits']=workspaces
    limits['source']='Native STEP assembly: recomputed kinematic reachability; provisional serial search magnitudes from legacy scan, symmetric in native coordinates'
    limits['collision_validation']='pending: legacy collision results do not certify this assembly'
    config['coupled_limits']=workspaces;control_text.set('data',json.dumps(config,separators=(',',':')))
    compiled=deepcopy(root);compiled.find('compiler').set('meshdir',str(MODEL))
    model=mujoco.MjModel.from_xml_string(ET.tostring(compiled,encoding='unicode'))
    data=mujoco.MjData(model)
    controller=JointPositionTargets(model,config)
    keys=ET.SubElement(root,'keyframe');ET.SubElement(keys,'key',name='Neutral')
    for name,signs in (('Six groups same direction',(1,1)),('Six groups opposite direction',(1,-1))):
        targets=dict.fromkeys(joints,0.)
        for group in topology['differential_groups'].values():
            trial=targets|{j:float(np.deg2rad(.5)*sign) for j,sign in zip(group['joints'],signs)}
            controller.workspace.validate(trial);targets=trial
        controller.set_configuration(data,list(targets.values()))
        ET.SubElement(keys,'key',name=name,qpos=shared.numbers(data.qpos),ctrl=shared.numbers(data.ctrl))
    # Main/mapping URDF is the 37-output skeleton; standalone additionally exposes auxiliary bodies.
    for link in source.findall('link'):
        name=link.get('name')
        for mesh in link.findall('visual/geometry/mesh')+link.findall('collision/geometry/mesh'):
            mesh.set('filename','package://robot_description/models/full_body_robot/'+files[name])
    ET.indent(source,space='  ');urdf_text=ET.tostring(source,encoding='unicode')+'\n'
    for suffix in ('','.mapping'):(MODEL/f'urdf/full_body_robot{suffix}.urdf').write_text(urdf_text)
    limits['source_sha256']=hashlib.sha256(urdf_text.encode()).hexdigest()
    ET.indent(root,space='  ')
    limits['geometry_sha256']=geometry_fingerprint(MODEL,root)
    (MODEL/'config/simulation_limits.yaml').write_text(yaml.safe_dump(limits,allow_unicode=True,sort_keys=False))
    (MODEL/'config/motor_topology.yaml').write_text(yaml.safe_dump(topology,allow_unicode=True,sort_keys=False))
    (MODEL/'config/joint_names.json').write_text(json.dumps({'source':'Robot_body1/meshes','rule':'strip whitespace and remove _link for joints','joints':aliases,'links':link_aliases},indent=2)+'\n')
    (MODEL/'config/native_assembly.json').write_text(json.dumps({'source_step_sha256':alignment['step_sha256'],'basis':'native STEP frames plus independently checked hips installation; no ICP matrices','ownership':ownership,'coupled_visual_world_transforms':geometry_poses,'collision_validation':'pending','auxiliary_inertia':'numerical preview surrogates; source physical-link inertias retained from Robot_body1'},indent=2)+'\n')
    for filename in ('full_body_robot.xml','full_body_robot.physics.xml'):(MODEL/'mujoco'/filename).write_text(ET.tostring(root,encoding='unicode')+'\n')
    shared.write_standalone_urdf(root,joints,limits)
    print('Native full model: 37 outputs, 30 auxiliary hinges, 10 closed rods; kinematic workspaces regenerated')


if __name__=='__main__':build()
