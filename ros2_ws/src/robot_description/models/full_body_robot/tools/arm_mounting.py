"""Face the screen-left arm outward by remounting its upper-arm roll zero."""
import numpy as np
from scipy.spatial.transform import Rotation

JOINT = 'BN_LeftUpperArm_3'


def solve(root, frames):
    joint = root.find(f"joint[@name='{JOINT}']")
    child = joint.find('child').get('link')
    frame = frames[child]
    axis = frame[:3, :3] @ np.fromstring(joint.find('axis').get('xyz'), sep=' ')
    axis /= np.linalg.norm(axis)
    rotation = Rotation.from_rotvec(np.pi * axis).as_matrix()
    delta = np.eye(4)
    delta[:3, :3] = rotation
    delta[:3, 3] = frame[:3, 3] - rotation @ frame[:3, 3]
    affected = [child]
    for name in affected:
        affected.extend(j.find('child').get('link') for j in root.findall('joint')
                        if j.find('parent').get('link') == name)
    return {'joint': JOINT, 'rotation_rad': float(np.pi), 'affected_links': affected,
            'world_transform': delta.tolist(), 'axis_world': axis.tolist(),
            'point_on_axis_m': frame[:3, 3].tolist(),
            'reason': 'User requested the complete screen-left arm to face outward, symmetric to the other arm.'}
