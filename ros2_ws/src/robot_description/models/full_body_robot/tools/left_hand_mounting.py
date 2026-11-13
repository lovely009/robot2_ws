"""Retain outward exports; remount an inward hand and its rods only when needed."""
import numpy as np
from scipy.spatial.transform import Rotation

import hips_mounting as hips

HAND = 'BN_LeftHand_pitch_link'


def solve(report, alignment, axes):
    if not report['source_step_sha256'] == alignment['step_sha256'] == axes['step_sha256']:
        raise ValueError('Left hand mounting inputs belong to different STEP assemblies')
    hinge = axes['joints']['BN_LeftHand_pitch']
    point = np.asarray(hinge['point_on_axis_m'])
    axis = np.asarray(hinge['axis']); axis = axis / np.linalg.norm(axis)
    native = np.asarray(alignment['links'][HAND]['world_transform'])
    forearm = np.asarray(alignment['links']['BN_LeftLowerArm_2_link']['world_transform'])
    outward = native[:3, 3] - forearm[:3, 3]
    outward /= np.linalg.norm(outward)
    projection = np.dot(-native[:3, 1], outward)
    if abs(projection) < .9:
        raise ValueError('Left hand export has an ambiguous plate direction')
    angle = 0. if projection > 0 else np.pi
    platform = hips.rigid(Rotation.from_rotvec(angle * axis).as_matrix(), point, point)
    hand_angle = angle
    rods = {}
    for name, record in report['rods'].items():
        if not name.startswith('left_wrist_'):
            continue
        start, end = (np.asarray(record['endpoints'][part]['center_m'])
                      for part in ('flange', 'platform'))
        target = hips.point(platform, end)
        if len(record['motor_axis_candidates']) != 1:
            raise ValueError(f'{name}: ambiguous motor axis')
        motor = record['motor_axis_candidates'][0]
        motor_axis = np.asarray(motor['axis']); motor_axis /= np.linalg.norm(motor_axis)
        origin = np.asarray(motor['point_on_axis_m'])
        origin += motor_axis * np.dot(start - origin, motor_axis)
        radius, delta = start - origin, target - origin
        u, v = np.dot(delta, radius), np.dot(delta, np.cross(motor_axis, radius))
        denominator = 2 * np.hypot(u, v)
        if denominator < 1e-12:
            raise ValueError(f'{name}: degenerate crank circle')
        cosine = (np.dot(delta, delta) + np.dot(radius, radius)
                  - record['center_distance_m'] ** 2) / denominator
        if abs(cosine) > 1 + 1e-10:
            raise ValueError(f'{name}: outward hand has no rigid-rod assembly solution')
        alpha, beta = np.arctan2(v, u), np.arccos(np.clip(cosine, -1., 1.))
        choices = sorted(((angle + np.pi) % (2 * np.pi) - np.pi
                          for angle in (alpha + beta, alpha - beta)), key=abs)
        if abs(abs(choices[0]) - abs(choices[1])) < 1e-8:
            raise ValueError(f'{name}: ambiguous nearest crank branch')
        angle = choices[0]
        flange_rotation = Rotation.from_rotvec(motor_axis * angle).as_matrix()
        flange = hips.rigid(flange_rotation, origin, origin)
        new_start = hips.point(flange, start)
        normal = np.asarray(record['endpoints']['flange']['rod_end_cylinder']['axis'])
        flange_normal = flange_rotation @ normal
        platform_normal = platform[:3, :3] @ np.asarray(
            record['endpoints']['platform']['rod_end_cylinder']['axis'])
        if np.dot(platform_normal, flange_normal) < 0:
            platform_normal = -platform_normal
        rod_rotation = (hips.direction_frame(target - new_start, flange_normal + platform_normal)
                        @ hips.direction_frame(end - start, normal).T)
        rod = hips.rigid(rod_rotation, start, new_start)
        if max(np.linalg.norm(hips.point(rod, end) - target),
               abs(np.linalg.norm(target - new_start) - record['center_distance_m'])) > 1e-9:
            raise ValueError(f'{name}: remount changed the rigid rod length')
        rods[name] = {'flange_transform': flange.tolist(), 'platform_transform': platform.tolist(),
                      'rod_transform': rod.tolist(), 'motor_zero_shift_rad': float(angle),
                      'branch_candidates_rad': choices, 'branch_policy': 'nearest source crank angle',
                      'rod_length_m': record['center_distance_m'], 'new_flange_ball_m': new_start.tolist(),
                      'new_platform_ball_m': target.tolist()}
    if len(rods) != 2:
        raise ValueError('Expected two left wrist rods')
    return {'source_step_sha256': alignment['step_sha256'],
            'source_urdf_sha256': alignment['urdf_sha256'],
            'reason': ('Source export already extends outward; preserve its mounting side, wrist fixtures and motor zeros.'
                       if hand_angle == 0 else
                       'User confirmed the inward source hand must be remounted outward like the right.'),
            'hand_link': HAND, 'hand_transform': platform.tolist(),
            'hand_world_transform': (platform @ native).tolist(),
            'hinge_axis': axis.tolist(), 'point_on_hinge_m': point.tolist(), 'rotation_rad': float(hand_angle),
            'scope': 'Simulation assembly correction; hardware zero, direction and limits remain uncalibrated.',
            'rods': rods}
