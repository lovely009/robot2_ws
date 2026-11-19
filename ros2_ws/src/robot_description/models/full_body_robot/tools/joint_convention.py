"""机械关节坐标转换。"""
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

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
