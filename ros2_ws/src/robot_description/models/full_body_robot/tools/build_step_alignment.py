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
STEP = MODEL.parent / '全身不带头.STEP'
OUTPUT = MODEL / 'mujoco/full_body_robot.step-aligned.xml'


def build():
    calibration = json.loads((MODEL / 'config/step_alignment.json').read_text())
    urdf = MODEL.parent / calibration['source_urdf']
    source_directory = urdf.parents[1]
    for path, expected in ((STEP, calibration['step_sha256']),
                           (urdf, calibration['urdf_sha256'])):
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError(f'{path.name}: repeat STEP assembly calibration after source changes')
    source = ET.parse(urdf).getroot()

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
    for index, link in enumerate(source.findall('link')):
        raw = link.find('visual/geometry/mesh').get('filename')
        # Five raw URIs contain a leading space absent from the exported filename.
        path = source_directory / 'meshes' / Path(raw).name.lstrip()
        name = link.get('name').strip()
        entry = calibration['links'][name]
        if hashlib.sha256(path.read_bytes()).hexdigest() != entry['source_sha256']:
            raise ValueError(f'{path.name}: repeat STEP mesh calibration after source changes')
        reader = vtk.vtkSTLReader()
        reader.SetFileName(str(path))
        reader.Update()
        output = MODEL / 'mujoco/visual/step_aligned' / path.name
        builder.write_display_mesh(reader.GetOutput(), output)
        transform = np.asarray(entry['world_transform'])
        quaternion = Rotation.from_matrix(transform[:3, :3]).as_quat()[[3, 0, 1, 2]]
        body = ET.SubElement(world, 'body', name=name, pos=builder.numbers(transform[:3, 3]),
                             quat=builder.numbers(quaternion))
        asset = f'assembly_{index:02d}'
        ET.SubElement(assets, 'mesh', name=asset, file=output.relative_to(MODEL).as_posix())
        ET.SubElement(body, 'geom', name=f'{name}_assembly', type='mesh', mesh=asset,
                      rgba='.84 .85 .86 1', contype='0', conaffinity='0', mass='0')
    ET.indent(root, space='  ')
    OUTPUT.write_text(ET.tostring(root, encoding='unicode') + '\n')
    print(OUTPUT)


if __name__ == '__main__':
    build()
