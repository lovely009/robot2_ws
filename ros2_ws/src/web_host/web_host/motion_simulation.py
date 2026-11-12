"""Registered simulation constraints and CAD poses for offline motion jobs."""

import hashlib
import json
import math
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from robot_transmission.coupled_limits import CoupledLimits


def registered_simulation(request, models_root):
    name = request.get('model')
    if name is None:
        return None
    if not isinstance(name, str) or not name or name != Path(name).name or name in ('.', '..') or '\\' in name:
        raise ValueError('模型名称无效')
    if models_root is None:
        from .models import _models_root
        models_root = _models_root()
    if models_root is None:
        return None
    directory = Path(models_root) / name
    config_path = directory / 'config/simulation_limits.yaml'
    if not config_path.is_file():
        return None
    source = (directory / 'urdf' / f'{name}.urdf').read_text()
    if request['urdf_text'].replace('\r\n', '\n') != source:
        raise ValueError('模型已更新或与模型库不一致，请重新导入')
    config_bytes = config_path.read_bytes()
    limits = yaml.safe_load(config_bytes)
    robot = ET.fromstring(source)
    joints = [joint for joint in robot.findall('joint') if joint.get('type') != 'fixed']
    if set(limits['joint_limits']) != {joint.get('name') for joint in joints}:
        raise ValueError('仿真位置限位须覆盖模型的全部关节')
    for joint in joints:
        bound = limits['joint_limits'][joint.get('name')]
        lower, upper = float(bound['lower']), float(bound['upper'])
        if (joint.get('type') != 'revolute' or joint.find('mimic') is not None
                or not np.isfinite([lower, upper]).all() or not lower <= 0 <= upper or lower >= upper):
            raise ValueError(f"{joint.get('name')} 的仿真范围无效")
        element = joint.find('limit')
        if element is None:
            element = ET.SubElement(joint, 'limit')
        # This rate is an offline preview cap, not a calibrated motor specification.
        element.attrib.update(lower=str(lower), upper=str(upper), velocity=str(math.pi), effort='0')
    return {'directory': directory, 'limits': limits,
            'config_sha256': hashlib.sha256(config_bytes).hexdigest(),
            'urdf_text': ET.tostring(robot, encoding='unicode'), 'velocity_limit_rad_s': math.pi}


def position_filter(simulation, names):
    limits = {name: (bound['lower'], bound['upper'])
              for name, bound in simulation['limits']['joint_limits'].items()}
    workspace = CoupledLimits(simulation['limits'].get('coupled_limits', {}), names)
    workspace.validate(dict.fromkeys(names, 0.))

    def project(previous, requested, max_step):
        result = workspace.project_step(limits, dict(zip(names, previous)),
                                        dict(zip(names, requested)), dict(zip(names, max_step)))
        return np.array([result[name] for name in names])
    return project


def validate_positions(simulation, names, positions):
    workspace = CoupledLimits(simulation['limits'].get('coupled_limits', {}), names)
    for row in positions:
        workspace.validate(dict(zip(names, row)))


def load_mujoco():
    os.environ.setdefault('MUJOCO_GL', 'disable')
    try:
        import mujoco
    except ModuleNotFoundError:
        # The installed MuJoCo launcher uses this environment; keep the ROS NumPy already loaded.
        sys.path.append(str(Path.home() / '.ros/ros2_control/.venv/lib' /
                            f'python{sys.version_info.major}.{sys.version_info.minor}/site-packages'))
        import mujoco
    return mujoco


def cad_preview(simulation, names, positions):
    directory = simulation['directory']
    current = (directory / 'config/simulation_limits.yaml').read_bytes()
    if hashlib.sha256(current).hexdigest() != simulation['config_sha256']:
        raise ValueError('仿真限位已更新，请重新导入动作')
    mujoco = load_mujoco()
    from robot_transmission.mujoco_positions import JointPositionTargets
    model_path = directory / 'mujoco' / f'{directory.name}.xml'
    root = ET.parse(model_path).getroot()
    config = json.loads(root.find("custom/text[@name='robot_sim_joint_position']").get('data'))
    model = mujoco.MjModel.from_xml_path(str(model_path))
    controller = JointPositionTargets(model, config)
    if set(names) != set(controller.joints):
        raise ValueError('仿真显示模型与动作关节不一致')
    indices = [names.index(name) for name in controller.joints]
    data = mujoco.MjData(model)
    frames = []
    for row in positions:
        controller.set_configuration(data, row[indices])
        frames.append(data.qpos[model.jnt_qposadr].tolist())
    return {'urdf': (directory / 'urdf' / f'{directory.name}.standalone.urdf').read_text(),
            'joint_names': [model.joint(index).name for index in range(model.njnt)],
            'joint_positions': frames, 'ignore_limits': False}
