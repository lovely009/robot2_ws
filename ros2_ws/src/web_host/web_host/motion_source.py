"""Recognize humanoid BVH exports for offline workbench imports only."""

import numpy as np


def anatomical_basis(names, rest, up_axis):
    """Map source rest-frame forward/left/up into the canonical ROS axes."""
    basis = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]]) if up_axis == 'y_up' else np.eye(3)
    points = {name.lower(): basis @ point for name, point in zip(names, rest)}
    for left_name, right_name in [('leftupperleg', 'rightupperleg'), ('leftshoulder', 'rightshoulder')]:
        if left_name not in points or right_name not in points:
            continue
        left = points[left_name] - points[right_name]
        left[2] = 0
        if np.linalg.norm(left) < 1e-6:
            continue
        left /= np.linalg.norm(left)
        up = np.array([0., 0., 1.])
        forward = np.cross(left, up)
        return np.array([forward, left, up])
    return np.eye(3)


def prepare_source(clip, up_axis):
    """Return the resolved up axis and position convention; normalize duplicate offsets."""
    rest = []
    for joint in clip.joints:
        rest.append(joint.offset + (rest[joint.parent] if joint.parent >= 0 else 0))
    if up_axis == 'auto':
        heads = [rest[i] for i, joint in enumerate(clip.joints) if joint.name.lower() == 'head']
        feet = [rest[i] for i, joint in enumerate(clip.joints)
                if joint.name.lower() in ('leftfoot', 'rightfoot')]
        up_axis = 'y_up'
        if heads and len(feet) == 2:
            vertical = np.abs(np.mean(heads, axis=0) - np.mean(feet, axis=0))
            if vertical[2] > 2 * max(vertical[0], vertical[1], 1e-8):
                up_axis = 'z_up'

    # Some exporters repeat the full local rest translation in every position
    # channel. Require the whole non-root skeleton to match, throughout the clip;
    # ordinary additive position channels and rotation-only BVH remain untouched.
    joints = [joint for joint in clip.joints if joint.parent >= 0 and not joint.is_end]
    repeated = len(joints) >= 3 and any(np.linalg.norm(j.offset) > 1e-6 for j in joints)
    tolerance = max([1e-6] + [np.linalg.norm(j.offset) * 1e-5 for j in joints])
    for joint in joints:
        channels = [axis + 'position' for axis in 'XYZ']
        if not all(channel in joint.channels for channel in channels):
            repeated = False
            break
        columns = [joint.start + joint.channels.index(channel) for channel in channels]
        if not np.allclose(clip.frames[:, columns], joint.offset, rtol=0, atol=tolerance):
            repeated = False
            break
    canonical = anatomical_basis([joint.name for joint in clip.joints], rest, up_axis)
    if repeated:
        for joint in joints:
            joint.offset = np.zeros(3)
    return up_axis, 'local_translation' if repeated else 'offset_delta', canonical
