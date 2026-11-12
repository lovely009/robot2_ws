#!/usr/bin/env python3
"""Generate separate +/-180 degree previews without changing the main model."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

MODEL = Path(__file__).resolve().parents[1]
LOWER, UPPER = str(-math.pi), str(math.pi)


def write(root, path):
    ET.indent(root, space='  ')
    path.write_text(ET.tostring(root, encoding='unicode') + '\n')
    print(path)


def build():
    root = ET.parse(MODEL / 'mujoco/full_body_robot.xml').getroot()
    root.set('model', 'Full body +/-180 degree preview')
    for joint in root.findall('worldbody//joint'):
        joint.set('limited', 'true')
        joint.set('range', f'{LOWER} {UPPER}')
    for actuator in root.findall('actuator/position'):
        actuator.set('ctrllimited', 'true')
        actuator.set('ctrlrange', f'{LOWER} {UPPER}')
    control = root.find("custom/text[@name='robot_sim_joint_position']")
    config = json.loads(control.get('data'))
    config['model'] = 'full_body_robot_180deg'
    config['joint_limits'] = {name: [-math.pi, math.pi] for name in config['joints']}
    # The old sampled workspace encodes the narrower preview domain. Keep the
    # physical loops and geometric IK; arbitrary angle combinations may fail IK.
    config['coupled_limits'] = {}
    control.set('data', json.dumps(config, separators=(',', ':')))
    ET.SubElement(root.find('custom'), 'text', name='preview_limit_policy',
                  data='All hinge and servo bounds are +/-pi. Linkage closure remains enforced; collision and hardware travel are uncalibrated.')
    write(root, MODEL / 'mujoco/full_body_robot.180deg.xml')
    for source, output in (
        ('full_body_robot.urdf', 'full_body_robot.180deg.urdf'),
        ('full_body_robot.standalone.urdf', 'full_body_robot.180deg.standalone.urdf'),
    ):
        robot = ET.parse(MODEL / 'urdf' / source).getroot()
        robot.set('name', 'full_body_robot_180deg')
        for joint in robot.findall('joint'):
            if joint.get('type') not in ('revolute', 'continuous'):
                continue
            joint.set('type', 'revolute')
            limit = joint.find('limit')
            if limit is None:
                limit = ET.SubElement(joint, 'limit', effort='0', velocity='0')
            limit.set('lower', LOWER)
            limit.set('upper', UPPER)
        write(robot, MODEL / 'urdf' / output)


if __name__ == '__main__':
    build()
