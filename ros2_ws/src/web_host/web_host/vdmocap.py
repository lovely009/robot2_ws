"""Validate VDMocap WS_Geo body frames and encode the workbench's BVH input."""

import warnings
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


# SDK body order and the parent whose world rotation moves each bone.
BODY = (
    ('Hips', -1),
    ('RightUpperLeg', 0), ('RightLowerLeg', 1), ('RightFoot', 2), ('RightToe', 3),
    ('LeftUpperLeg', 0), ('LeftLowerLeg', 5), ('LeftFoot', 6), ('LeftToe', 7),
    ('Spine', 0), ('Spine1', 9), ('Spine2', 10), ('Spine3', 11), ('Neck', 12), ('Head', 13),
    ('RightShoulder', 12), ('RightUpperArm', 15), ('RightLowerArm', 16), ('RightHand', 17),
    ('LeftShoulder', 12), ('LeftUpperArm', 19), ('LeftLowerArm', 20), ('LeftHand', 21),
)
MAX_FRAMES = 7200


def _array(value, shape, label):
    message = f'{label} 必须是形状为 {shape} 的有限数值数组'
    try:
        # JSON strings and booleans must not silently become coordinates.
        data = np.asarray(value, dtype=object)
        if data.shape != shape or any(type(x) not in (int, float) for x in data.flat):
            raise ValueError(message)
        data = data.astype(float)
    except (TypeError, ValueError, OverflowError):
        raise ValueError(message) from None
    if not np.isfinite(data).all():
        raise ValueError(message)
    return data


def _timeline(frames):
    ticks = [0]
    frequency = None
    for i, frame in enumerate(frames):
        if not isinstance(frame, dict):
            raise ValueError(f'第 {i + 1} 帧必须是对象')
        index, rate = frame.get('index'), frame.get('frequency')
        if type(index) is not int or not 0 <= index < 2**32:
            raise ValueError(f'第 {i + 1} 帧的 index 必须为 uint32')
        if type(rate) is not int or not 0 < rate < 2**31 or (i and rate != frequency):
            raise ValueError('frequency 必须为全段一致的正整数 Hz')
        if i:
            step = (index - previous) % 2**32
            if step == 0 or step >= 2**31:
                raise ValueError(f'帧号 {index} 重复或倒退，请分段提交')
            if step - 1 > rate / 4:
                raise ValueError(f'帧号 {index} 前丢帧超过 0.25 秒，请分段提交')
            ticks.append(ticks[-1] + step)
            if ticks[-1] >= MAX_FRAMES:
                raise ValueError(f'补帧后最多支持 {MAX_FRAMES} 帧')
        frequency, previous = rate, index
    return np.asarray(ticks), frequency


def _bvh_text(offsets, root, rotations, frequency):
    lines = ['HIERARCHY']
    order = []

    def hierarchy(index, depth):
        name, parent = BODY[index]
        pad = '  ' * depth
        order.append(index)
        xyz = ' '.join(f'{v:.8f}' for v in offsets[index] * 100)
        lines.extend([pad + ('ROOT ' if parent < 0 else 'JOINT ') + name, pad + '{',
                      f'{pad}  OFFSET {xyz}', pad + '  CHANNELS ' +
                      ('6 Xposition Yposition Zposition ' if parent < 0 else '3 ') +
                      'Zrotation Xrotation Yrotation'])
        children = [i for i, (_, p) in enumerate(BODY) if p == index]
        for child in children:
            hierarchy(child, depth + 1)
        if not children:
            lines.extend([pad + '  End Site', pad + '  {', pad + '    OFFSET 0 0 0', pad + '  }'])
        lines.append(pad + '}')

    hierarchy(0, 0)
    with warnings.catch_warnings():
        # At gimbal lock SciPy still returns an equivalent Euler decomposition.
        warnings.filterwarnings('ignore', message='Gimbal lock detected.*', category=UserWarning)
        angles = Rotation.from_matrix(rotations[:, order].reshape(-1, 3, 3)).as_euler('ZXY', degrees=True)
    rows = np.column_stack((root * 100, angles.reshape(len(root), -1)))
    lines.extend(['MOTION', f'Frames: {len(rows)}', f'Frame Time: {1 / frequency:.12g}'])
    lines.extend(' '.join(f'{x:.7f}' for x in row) for row in rows)
    return '\n'.join(lines) + '\n'


@dataclass
class BodySkeleton:
    order: list
    parents: np.ndarray
    offsets: np.ndarray
    lengths: np.ndarray

    @classmethod
    def from_tpose(cls, names, rest):
        if (not isinstance(names, list) or len(names) != len(BODY) or
                not all(isinstance(n, str) for n in names) or set(names) != {n for n, _ in BODY}):
            raise ValueError('身体骨架须包含 SDK 全部 23 个关节，每个名称只出现一次')
        order = [names.index(name) for name, _ in BODY]
        parents = np.array([parent for _, parent in BODY[1:]])
        rest = _array(rest, (len(BODY), 3), 'T 姿')[order]
        offsets = np.zeros_like(rest)
        offsets[1:] = rest[1:] - rest[parents]
        lengths = np.linalg.norm(offsets[1:], axis=1)
        head_to_feet = rest[14] - rest[[3, 7]].mean(axis=0)
        if (not .2 <= head_to_feet[2] <= 3 or np.ptp(rest, axis=0).max() > 4 or
                head_to_feet[2] <= 2 * max(abs(head_to_feet[:2]))):
            raise ValueError('T 姿的头足高度或方向异常，请使用米和 Z 向上的 SDK 地理坐标')
        if (np.any(lengths < 1e-6) or rest[14, 2] <= rest[0, 2] or
                rest[1, 0] <= rest[5, 0] or rest[15, 0] <= rest[19, 0]):
            raise ValueError('T 姿骨架不符合 SDK 地理坐标的骨长、左右或竖直方向')

        return cls(order, parents, offsets, lengths)

    def world_poses(self, frames):
        n = len(frames)
        positions = _array([f.get('positions') for f in frames], (n, len(BODY), 3), 'positions')[:, self.order]
        raw = _array([f.get('quaternions') for f in frames], (n, len(BODY), 4), 'quaternions')[:, self.order]
        if np.any(np.abs(np.linalg.norm(raw, axis=2) - 1) > .01):
            raise ValueError('身体姿态必须为近单位四元数（wxyz）')
        world = Rotation.from_quat(raw[:, :, [1, 2, 3, 0]].reshape(-1, 4)).as_matrix().reshape(n, -1, 3, 3)
        expected = np.einsum('fnij,nj->fni', world[:, self.parents], self.offsets[1:])
        errors = np.linalg.norm(positions[:, 1:] - positions[:, self.parents] - expected, axis=2)
        # SDK quaternion quantization causes submillimeter differences in the stock T pose.
        bad = np.argwhere(errors > np.maximum(.002, .01 * self.lengths))
        if len(bad):
            i, child = bad[0]
            raise ValueError(f'帧 {frames[i]["index"]} 的 {BODY[child + 1][0]} 与 T 姿骨长或父关节世界姿态不符')
        return positions, world


def body_bvh(capture):
    if not isinstance(capture, dict) or capture.get('world_space') != 'WS_Geo':
        raise ValueError('需要 SDK 地理坐标 WS_Geo 下的身体世界位置（米）及世界四元数（wxyz）')
    skeleton = BodySkeleton.from_tpose(capture.get('joints'), capture.get('rest'))

    frames = capture.get('frames')
    if not isinstance(frames, list) or not 2 <= len(frames) <= MAX_FRAMES:
        raise ValueError(f'每段需要 2 到 {MAX_FRAMES} 个不同的动捕帧')
    ticks, frequency = _timeline(frames)
    n = len(frames)
    positions, world = skeleton.world_poses(frames)
    parents = skeleton.parents
    local = world.copy()
    local[:, 1:] = world[:, parents].swapaxes(-1, -2) @ world[:, 1:]
    root = positions[:, 0]
    count = int(ticks[-1]) + 1
    if count != n:
        timeline = np.arange(count)
        root = np.column_stack([np.interp(timeline, ticks, axis) for axis in root.T])
        local = np.stack([Slerp(ticks, Rotation.from_matrix(joint))(timeline).as_matrix()
                          for joint in local.swapaxes(0, 1)], axis=1)
    info = {'received_frames': n, 'frames': count, 'frequency': frequency,
            'first_index': frames[0]['index'], 'last_index': frames[-1]['index'],
            'interpolated_frames': count - n}
    return _bvh_text(skeleton.offsets, root, local, frequency), info
