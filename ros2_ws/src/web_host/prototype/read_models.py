"""原型只读素材：从源码 YAML/URDF 提供模型信息，不访问 ROS 或设备。"""
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

src = Path(__file__).resolve().parents[2]
root = src / 'robot_sim' / 'models'
global_config = yaml.safe_load((src / 'motor_hardware_interface/config/motor_driver.yaml').read_text())
models = []
for directory in sorted(root.iterdir()):
    urdf = directory / 'urdf' / f'{directory.name}.urdf'
    if not urdf.is_file():
        continue
    config = directory / 'config/motor_driver.yaml'
    motors = (yaml.safe_load(config.read_text()) if config.exists() else global_config)['motor_hardware']['motors']
    mapping = {}
    if config.exists():
        mapping = {spec['joint']: {**spec, 'name': name} for name, spec in motors.items()}
    else:
        topology = directory / 'config/simulation_topology.yaml'
        if topology.exists():
            entries = yaml.safe_load(topology.read_text())['simulation']['motors']
            mapping = {entry['joint']: {**motors[entry['motor']], 'name': entry['motor']} for entry in entries}
    joints = []
    for joint in ET.parse(urdf).getroot().findall('joint'):
        if joint.get('type') == 'fixed':
            continue
        name = joint.attrib['name']
        limit = joint.find('limit')
        motor = mapping.get(name)
        joints.append({'name': name, 'parent': joint.find('parent').get('link'),
                       'child': joint.find('child').get('link'),
                       'type': joint.get('type'),
                       'lower': float(limit.get('lower', '-3.14')) if limit is not None else -3.14,
                       'upper': float(limit.get('upper', '3.14')) if limit is not None else 3.14,
                       'motor': motor})
    models.append({'name': directory.name, 'joints': joints,
                   'gazebo': (directory / 'config/simulation_topology.yaml').exists(),
                   'mujoco': (directory / 'mujoco' / f'{directory.name}.xml').exists()})
print(json.dumps(models, ensure_ascii=False))
