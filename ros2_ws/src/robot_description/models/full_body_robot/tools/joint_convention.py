"""Publish native CAD kinematics in the functional whole-body coordinate system."""

from copy import deepcopy
import hashlib
import json
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import yaml

from native_names import rename_tree


def load(model):
    document = yaml.safe_load((model / 'config/joint_convention.yaml').read_text())
    coordinates = document['joint_coordinates']
    if len(coordinates) != 37 or len({name for name, _ in coordinates.values()}) != 37:
        raise ValueError('Expected 37 unique functional joints')
    if any(sign not in (-1, 1) for _, sign in coordinates.values()):
        raise ValueError('Joint direction must be +1 or -1')
    return Rotation.from_matrix(document['cad_to_model']), coordinates


def inverse(coordinates):
    return {name: (native, sign) for native, (name, sign) in coordinates.items()}


def signed_interval(lower, upper, sign):
    return (-upper, -lower) if sign == -1 else (lower, upper)


def convert_limits(native, coordinates, actuator_signs):
    limits = deepcopy(native)
    for key in ('joint_limits', 'legacy_scalar_caps', 'actuator_limits'):
        if key not in limits:
            continue
        mapping = coordinates if key != 'actuator_limits' else {
            name: (name, sign) for name, sign in actuator_signs.items()}
        converted = {}
        for name, bound in limits[key].items():
            target, sign = mapping.get(name, (name, 1))
            bound['lower'], bound['upper'] = signed_interval(bound['lower'], bound['upper'], sign)
            converted[target] = bound
        limits[key] = converted
    for grid in limits.get('coupled_limits', {}).values():
        rows = grid['valid_rows']
        lengths = len(rows), len(rows[0])
        for axis, native_name in enumerate(grid['joints']):
            name, sign = coordinates[native_name]
            grid['joints'][axis] = name
            if sign == -1:
                start = round(grid['lower'][axis] / grid['step'])
                grid['lower'][axis] = (-start - lengths[axis] + 1) * grid['step']
                rows = rows[::-1] if axis == 0 else [row[::-1] for row in rows]
        grid['valid_rows'] = rows
    return limits


def vector(element, key, default='0 0 0'):
    return np.fromstring(element.get(key, default), sep=' ')


def numbers(values):
    return ' '.join(format(float(value) if value else 0., '.17g') for value in values)


def convert_urdf(robot, rotation, coordinates):
    base = robot.find('link[@name="Base_link"]')
    origins = [element.find('origin') for element in base]
    origins.extend(joint.find('origin') for joint in robot.findall('joint')
                   if joint.find('parent').get('link') == 'Base_link')
    for origin in origins:
        origin.set('xyz', numbers(rotation.apply(vector(origin, 'xyz'))))
        orientation = rotation * Rotation.from_euler('xyz', vector(origin, 'rpy'))
        origin.set('rpy', numbers(orientation.as_euler('xyz')))
    for joint in robot.findall('joint'):
        name, sign = coordinates[joint.get('name')]
        joint.set('name', name)
        axis, limit = joint.find('axis'), joint.find('limit')
        axis.set('xyz', numbers(sign * vector(axis, 'xyz')))
        lower, upper = signed_interval(float(limit.get('lower')), float(limit.get('upper')), sign)
        limit.attrib.update(lower=str(lower), upper=str(upper))


def actuator_directions(root, coordinates):
    return {actuator.get('name'): coordinates.get(actuator.get('joint'), (None, 1))[1]
            for actuator in root.findall('actuator/position')}


def convert_mjcf(root, rotation, coordinates, limits):
    world = root.find('worldbody')
    base = world.find('body[@name="Base_link"]')
    for element in (*base.findall('body'), *base.findall('geom'), *world.findall('light')):
        element.set('pos', numbers(rotation.apply(vector(element, 'pos'))))
        if element.tag == 'light':
            element.set('dir', numbers(rotation.apply(vector(element, 'dir'))))
            continue
        quat = vector(element, 'quat', '1 0 0 0')
        orientation = rotation * Rotation.from_quat(quat[[1, 2, 3, 0]])
        element.set('quat', numbers(orientation.as_quat()[[3, 0, 1, 2]]))
    center = root.find('statistic')
    center.set('center', numbers(rotation.apply(vector(center, 'center'))))
    # Look from +X at the robot's front in the functional frame.
    root.find('visual/global').attrib.update(azimuth='180', elevation='0')
    inertial = base.find('inertial')
    inertial.set('pos', numbers(rotation.apply(vector(inertial, 'pos'))))
    xx, yy, zz, xy, xz, yz = vector(inertial, 'fullinertia')
    basis = rotation.as_matrix()
    tensor = basis @ np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]]) @ basis.T
    inertial.set('fullinertia', numbers((tensor[0, 0], tensor[1, 1], tensor[2, 2],
                                       tensor[0, 1], tensor[0, 2], tensor[1, 2])))
    joints = world.findall('.//joint')
    actuators = root.findall('actuator/position')
    for joint in joints:
        old = joint.get('name')
        if old not in coordinates:
            continue
        name, sign = coordinates[old]
        joint.set('name', name)
        joint.set('axis', numbers(sign * vector(joint, 'axis')))
        joint.set('range', numbers(signed_interval(*vector(joint, 'range'), sign)))
    for actuator in actuators:
        old = actuator.get('joint')
        if old in coordinates:
            name, sign = coordinates[old]
            actuator.set('joint', name)
            actuator.set('ctrlrange', numbers(signed_interval(*vector(actuator, 'ctrlrange'), sign)))
    text = root.find("custom/text[@name='robot_sim_joint_position']")
    config = json.loads(text.get('data'))
    legacy_names = {**{name: name for name in coordinates}, **config['joint_aliases']}
    config.update(
        joints=[coordinates[name][0] for name in config['joints']],
        joint_aliases={old: coordinates[name][0] for old, name in legacy_names.items()},
        joint_alias_signs={old: -1 for old, name in legacy_names.items() if coordinates[name][1] == -1},
        joint_limits={name: [bound['lower'], bound['upper']]
                      for name, bound in limits['joint_limits'].items()},
        coupled_limits=limits['coupled_limits'],
    )
    text.set('data', json.dumps(config, separators=(',', ':')))
    return config


def place_on_ground(root, model):
    """Make a standing keyframe without changing the CAD joint zeroes."""
    import mujoco

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    sides = ('Left', 'Right')
    shoulders = np.mean([data.body(f'BN_{side}Shoulder_1_link').xpos for side in sides], axis=0)
    torso = shoulders - data.body('BN_hips_pitch_link').xpos
    axis = np.cross(torso, [0., 0., 1.])
    sine = np.linalg.norm(axis)
    orientation = Rotation.from_rotvec(axis * (np.arctan2(sine, torso[2]) / sine if sine else 0.))
    ankles = [data.body(f'BN_{side}Foot_roll_link') for side in sides]
    soles = []
    for side in sides:
        geom = data.geom(f'BN_{side}Foot_roll_link_robot_body')
        mesh = model.geom_dataid[geom.id]
        start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
        vertices = model.mesh_vert[start:start + count]
        start, count = model.mesh_faceadr[mesh], model.mesh_facenum[mesh]
        triangles = vertices[model.mesh_face[start:start + count]].astype(float)
        normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
        # The broad underside of the sole has the largest downward area.
        world_normals = orientation.apply(normals @ geom.xmat.reshape(3, 3).T)
        normal = normals[world_normals[:, 2].argmin()]
        normal /= np.linalg.norm(normal)
        soles.append((geom, vertices, normal))

    joints = [model.joint(f'{side.lower()}_{joint}')
              for side in sides for joint in ('hip_pitch', 'hip_roll', 'knee_pitch',
                                               'ankle_pitch', 'ankle_roll')]
    addresses = [int(joint.qposadr[0]) for joint in joints]
    ranges = np.asarray([joint.range for joint in joints])
    actuators = [int(np.flatnonzero(model.actuator_trnid[:, 0] == joint.id).item())
                 for joint in joints]
    lower = np.maximum(ranges[:, 0], model.actuator_ctrlrange[actuators, 0])
    upper = np.minimum(ranges[:, 1], model.actuator_ctrlrange[actuators, 1])

    def foot_geometry():
        normals, heights = [], []
        for geom, vertices, normal in soles:
            rotation = geom.xmat.reshape(3, 3)
            normals.append(orientation.apply(rotation @ normal))
            heights.append(orientation.apply(vertices @ rotation.T + geom.xpos)[:, 2].min())
        return normals, heights

    def stance_error(positions):
        data.qpos[addresses] = positions
        mujoco.mj_kinematics(model, data)
        normals, heights = foot_geometry()
        ankle_center = np.mean([ankle.xpos for ankle in ankles], axis=0)
        return np.r_[normals[0][:2], normals[1][:2], heights[0] - heights[1],
                     orientation.apply(ankle_center - shoulders)[:2]]

    fit = least_squares(stance_error, np.zeros(len(joints)), bounds=(lower, upper),
                        gtol=1e-12, xtol=1e-12, ftol=1e-12)
    error = stance_error(fit.x)
    normals, heights = foot_geometry()
    if not fit.success or np.max(np.abs(error)) > 1e-7 or any(normal[2] >= 0 for normal in normals):
        raise ValueError(f'Could not fit the standing pose within joint limits: {error}')
    data.ctrl[actuators] = fit.x
    ET.SubElement(root.find('keyframe'), 'key', name='Standing',
                  qpos=numbers(data.qpos), ctrl=numbers(data.ctrl))
    offset = np.array([0., 0., -min(heights)])
    world = root.find('worldbody')
    base = world.find('body[@name="Base_link"]')
    base.attrib.update(pos=numbers(offset), quat=numbers(orientation.as_quat()[[3, 0, 1, 2]]))
    center = root.find('statistic')
    center.set('center', numbers(orientation.apply(vector(center, 'center')) + offset))
    for light in world.findall('light'):
        light.set('pos', numbers(vector(light, 'pos') + offset))
    ET.SubElement(world, 'geom', name='ground', type='plane', size='0 0 .1',
                  rgba='.23 .25 .28 1', contype='0', conaffinity='0')
    return {model.joint(index).name: float(data.qpos[model.jnt_qposadr[index]])
            for index in range(model.njnt)}


def preview_keys(directory, root, config, groups):
    import mujoco
    from robot_transmission.mujoco_positions import JointPositionTargets

    previous = root.find('keyframe')
    if previous is not None:
        root.remove(previous)
    compiled = deepcopy(root)
    compiled.find('compiler').set('meshdir', str(directory))
    model = mujoco.MjModel.from_xml_string(ET.tostring(compiled, encoding='unicode'))
    data = mujoco.MjData(model)
    controller = JointPositionTargets(model, config)
    keys = ET.SubElement(root, 'keyframe')
    ET.SubElement(keys, 'key', name='Neutral')
    for name, signs in (('Six groups same direction', (1, 1)),
                        ('Six groups opposite direction', (1, -1))):
        targets = dict.fromkeys(controller.joints, 0.)
        for group in groups.values():
            trial = targets | {joint: float(np.deg2rad(.5) * sign)
                               for joint, sign in zip(group['joints'], signs)}
            try:
                controller.workspace.validate(trial)
            except ValueError:
                continue
            targets = trial
        controller.set_configuration(data, list(targets.values()))
        ET.SubElement(keys, 'key', name=name, qpos=numbers(data.qpos), ctrl=numbers(data.ctrl))
    return place_on_ground(root, model)


def write_standing_urdf(model, mjcf, positions):
    urdf_dir = model / 'urdf'
    robot = ET.parse(urdf_dir / 'full_body_robot.standalone.urdf').getroot()
    robot.insert(0, ET.Element('link', name='world'))
    joint = ET.Element('joint', name='world_to_base', type='fixed')
    robot.insert(1, joint)
    ET.SubElement(joint, 'parent', link='world')
    ET.SubElement(joint, 'child', link='Base_link')
    base = mjcf.find('worldbody/body[@name="Base_link"]')
    quat = vector(base, 'quat')
    orientation = Rotation.from_quat(quat[[1, 2, 3, 0]])
    ET.SubElement(joint, 'origin', xyz=base.get('pos'), rpy=numbers(orientation.as_euler('xyz')))
    ET.indent(robot, space='  ')
    (urdf_dir / 'full_body_robot.standing.urdf').write_text(
        ET.tostring(robot, encoding='unicode') + '\n')
    (model / 'config/standing.json').write_text(json.dumps(positions, indent=2) + '\n')


def native_limits(model, limits):
    if limits.pop('joint_convention', None) is None:
        return limits
    _, coordinates = load(model)
    root = ET.parse(model / 'mujoco/full_body_robot.physics.xml').getroot()
    return convert_limits(limits, inverse(coordinates), actuator_directions(root, coordinates))


def publish(model, robot, root, limits, aliases, link_aliases):
    from build_mujoco_model import write_standalone_urdf

    rotation, coordinates = load(model)
    limits = convert_limits(limits, coordinates, actuator_directions(root, coordinates))
    convert_urdf(robot, rotation, coordinates)
    config = convert_mjcf(root, rotation, coordinates, limits)
    topology = yaml.safe_load((model / 'config/motor_topology.yaml').read_text())
    topology = rename_tree(topology, {native: name for native, (name, _) in coordinates.items()})
    standing = preview_keys(model, root, config, topology['differential_groups'])
    ET.indent(robot, space='  ')
    urdf = ET.tostring(robot, encoding='unicode') + '\n'
    ET.indent(root, space='  ')
    mjcf = ET.tostring(root, encoding='unicode') + '\n'
    limits['source_sha256'] = hashlib.sha256(urdf.encode()).hexdigest()
    limits['joint_convention'] = 'functional'
    mapping = {**{native: name for native, (name, _) in coordinates.items()},
               **{old: coordinates[name][0] for old, name in aliases.items()}}
    (model / 'config/model_motor_topology.yaml').write_text(yaml.safe_dump(topology, sort_keys=False))
    (model / 'config/simulation_limits.yaml').write_text(yaml.safe_dump(limits, sort_keys=False))
    (model / 'config/joint_names.json').write_text(json.dumps({
        'source': 'config/joint_convention.yaml',
        'joints': mapping,
        'native_joint_coordinates': coordinates,
        'links': link_aliases,
    }, indent=2) + '\n')
    for suffix in ('', '.mapping'):
        (model / f'urdf/full_body_robot{suffix}.urdf').write_text(urdf)
    (model / 'mujoco/full_body_robot.xml').write_text(mjcf)
    write_standalone_urdf(root, limits['joint_limits'], limits)
    write_standing_urdf(model, root, standing)
