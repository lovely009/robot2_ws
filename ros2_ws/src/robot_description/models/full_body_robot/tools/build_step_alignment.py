#!/usr/bin/env python3
"""Build a static assembly reference, calibrated against the supplied STEP export."""
from pathlib import Path
import hashlib
import importlib.util
import json
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import vtk

MODEL = Path(__file__).resolve().parents[1]
SOURCE = MODEL.parent / 'Robot_body1'
URDF = SOURCE / 'urdf/Robot_body1.urdf'
STEP = MODEL / 'cad/全身装配体（不含头部）.STEP'
if not STEP.exists():
    STEP = MODEL.parent / '全身装配体（不含头部）.STEP'
OUTPUT = MODEL / 'mujoco/full_body_robot.step-aligned.xml'


def build():
    audit = json.loads((MODEL / 'cad/assembly_audit/2026-11-12.json').read_text())
    for path, expected in ((STEP, audit['step']['sha256']),
                           (URDF, audit['new_export']['urdf_sha256'])):
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError(f'{path.name}: repeat STEP assembly calibration after source changes')
    source = ET.parse(URDF).getroot()
    # This single source-export correction was checked against the STEP assembly.
    source.find("joint[@name='BN_Spine_pitch_link ']/origin").set('rpy', '-1.5708 1.4421 1.5708')
    frames = {'Base_link': np.eye(4)}
    pending = list(source.findall('joint'))
    while pending:
        joint = next(j for j in pending if j.find('parent').get('link') in frames)
        pending.remove(joint)
        origin = joint.find('origin')
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy'), sep=' ')).as_matrix()
        transform[:3, 3] = np.fromstring(origin.get('xyz'), sep=' ')
        frames[joint.find('child').get('link')] = frames[joint.find('parent').get('link')] @ transform

    spec = importlib.util.spec_from_file_location('full_body_builder', MODEL / 'tools/build_mujoco_model.py')
    builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(builder)
    root = ET.Element('mujoco', model='Full body STEP assembly reference (static)')
    root.append(ET.Comment(' Static STEP alignment reference. No joint, actuator or closure calibration implied. '))
    ET.SubElement(root, 'compiler', meshdir='..', angle='radian', fusestatic='false')
    ET.SubElement(root, 'option', gravity='0 0 0')
    visual = ET.SubElement(root, 'visual')
    ET.SubElement(visual, 'global', azimuth='90', elevation='-8', offwidth='1280', offheight='960')
    ET.SubElement(root, 'statistic', center='0 .08 -.28', extent='1.6')
    assets, world = ET.SubElement(root, 'asset'), ET.SubElement(root, 'worldbody')
    ET.SubElement(world, 'light', pos='0 1 1.8', dir='0 -.45 -1')
    ET.SubElement(world, 'light', pos='0 -1 .8', dir='0 .55 -1')
    correspondence = {}
    for index, link in enumerate(source.findall('link')):
        raw = link.find('visual/geometry/mesh').get('filename')
        # Five raw URIs contain a leading space absent from the exported filename.
        path = SOURCE / 'meshes' / Path(raw).name.lstrip()
        reader = vtk.vtkSTLReader()
        reader.SetFileName(str(path))
        reader.Update()
        output = MODEL / 'mujoco/visual/step_aligned' / path.name
        builder.write_display_mesh(reader.GetOutput(), output)
        name = link.get('name').strip()
        transform = frames[link.get('name')]
        quaternion = Rotation.from_matrix(transform[:3, :3]).as_quat()[[3, 0, 1, 2]]
        body = ET.SubElement(world, 'body', name=name, pos=builder.numbers(transform[:3, 3]),
                             quat=builder.numbers(quaternion))
        asset = f'assembly_{index:02d}'
        ET.SubElement(assets, 'mesh', name=asset, file=output.relative_to(MODEL).as_posix())
        ET.SubElement(body, 'geom', name=f'{name}_assembly', type='mesh', mesh=asset,
                      rgba='.84 .85 .86 1', contype='0', conaffinity='0', mass='0')
        correspondence[name] = {'source': path.name, 'world_transform': transform.tolist(),
                                'source_sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
    ET.indent(root, space='  ')
    OUTPUT.write_text(ET.tostring(root, encoding='unicode') + '\n')
    manifest = {'mode': 'static STEP zero-pose reference', 'step_sha256': audit['step']['sha256'],
                'urdf_sha256': audit['new_export']['urdf_sha256'], 'links': correspondence,
                'spine_pitch_rpy_correction': '-1.5708 1.4421 1.5708'}
    (MODEL / 'config/step_alignment.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    print(OUTPUT)


if __name__ == '__main__':
    build()
