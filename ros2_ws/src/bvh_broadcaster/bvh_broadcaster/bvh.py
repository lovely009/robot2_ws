"""BVH parsing and local transforms, independent of ROS."""

from dataclasses import dataclass
from io import StringIO
from pathlib import Path
import math
import re

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass
class Joint:
    name: str
    parent: int
    offset: np.ndarray
    channels: tuple
    start: int


@dataclass
class BvhClip:
    joints: list
    frames: np.ndarray
    frame_time: float

    @classmethod
    def load(cls, path):
        text = Path(path).read_text(encoding='utf-8-sig')
        parts = re.split(r'^\s*MOTION\s*$', text, maxsplit=1, flags=re.MULTILINE)
        if len(parts) != 2:
            raise ValueError('BVH must contain a MOTION section')
        tokens = iter(re.findall(r'[{}]|[^{}\s]+', parts[0]))
        joints, names = [], set()
        channel_count = 0

        def expect(value):
            actual = next(tokens)
            if actual != value:
                raise ValueError(f'Expected {value!r}, got {actual!r}')

        def read_joint(parent, end=False):
            nonlocal channel_count
            if end:
                expect('Site')
                name = joints[parent].name + '__end'
            else:
                name = re.sub(r'[^A-Za-z0-9_]', '_', next(tokens))
            if not name or name in names:
                raise ValueError(f'Duplicate or empty TF joint name: {name!r}')
            names.add(name)
            expect('{')
            index = len(joints)
            joint = Joint(name, parent, None, (), channel_count)
            joints.append(joint)
            has_channels = False
            while True:
                token = next(tokens)
                if token == 'OFFSET' and joint.offset is None:
                    joint.offset = np.array([float(next(tokens)) for _ in range(3)])
                    if not np.isfinite(joint.offset).all():
                        raise ValueError('Non-finite joint OFFSET')
                elif token == 'CHANNELS' and not end and not has_channels:
                    count = int(next(tokens))
                    if not 0 <= count <= 6:
                        raise ValueError('Invalid channel count')
                    joint.channels = tuple(next(tokens) for _ in range(count))
                    allowed = {axis + kind for axis in 'XYZ' for kind in ('position', 'rotation')}
                    if len(set(joint.channels)) != count or not set(joint.channels) <= allowed:
                        raise ValueError('Duplicate or unsupported BVH channels')
                    rotation_seen = False
                    for channel in joint.channels:
                        if channel.endswith('rotation'):
                            rotation_seen = True
                        elif rotation_seen:
                            raise ValueError('Position channels must precede rotations')
                    channel_count += count
                    has_channels = True
                elif token in ('JOINT', 'End') and not end:
                    if not has_channels:
                        raise ValueError('CHANNELS must precede child joints')
                    read_joint(index, end=token == 'End')
                elif token == '}':
                    if joint.offset is None or (not end and not has_channels):
                        raise ValueError('Joint is missing OFFSET or CHANNELS')
                    return
                else:
                    raise ValueError(f'Unexpected hierarchy token: {token!r}')

        try:
            expect('HIERARCHY')
            expect('ROOT')
            read_joint(-1)
        except StopIteration as exc:
            raise ValueError('Truncated BVH hierarchy') from exc
        if next(tokens, None) is not None:
            raise ValueError('Unexpected data after hierarchy')
        motion = [line.strip() for line in parts[1].splitlines() if line.strip()]
        if len(motion) < 3:
            raise ValueError('Missing BVH motion data')
        frame_match = re.fullmatch(r'Frames:\s*(\d+)', motion[0])
        time_match = re.fullmatch(r'Frame\s+Time:\s*(\S+)', motion[1])
        if not frame_match or not time_match:
            raise ValueError('Invalid BVH motion header')
        count, dt = int(frame_match[1]), float(time_match[1])
        if count < 1 or not math.isfinite(dt) or dt <= 0 or channel_count == 0:
            raise ValueError('Frame count, frame time and channels must be positive')
        frames = np.loadtxt(StringIO('\n'.join(motion[2:])), ndmin=2)
        if frames.shape != (count, channel_count) or not np.isfinite(frames).all():
            raise ValueError('Frame/channel count mismatch or non-finite motion data')
        return cls(joints, frames, dt)

    def local_transforms(self, frame, scale=0.01, up_axis='y_up', apply_root_offset=True):
        if not math.isfinite(scale) or scale <= 0 or up_axis not in ('y_up', 'z_up'):
            raise ValueError('scale must be positive; up_axis must be y_up or z_up')
        # Y-up BVH: +Z forward, +X left -> ROS +X forward, +Y left, +Z up.
        basis = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]]) if up_axis == 'y_up' else np.eye(3)
        values = self.frames[frame]
        transforms = []
        for joint in self.joints:
            position = joint.offset.copy()
            if joint.parent == -1 and not apply_root_offset and any(
                    c.endswith('position') for c in joint.channels):
                position[:] = 0
            order, angles = '', []
            for channel, value in zip(joint.channels, values[joint.start:]):
                if channel.endswith('position'):
                    position['XYZ'.index(channel[0])] += value
                else:
                    order += channel[0]
                    angles.append(value)
            rotation = (Rotation.from_euler(order, angles[0] if len(angles) == 1 else angles,
                                           degrees=True).as_matrix() if angles else np.eye(3))
            quaternion = Rotation.from_matrix(basis @ rotation @ basis.T).as_quat()
            transforms.append((basis @ position * scale, quaternion))
        return transforms


def playback_frame(elapsed, frame_time, count, speed, loop):
    index = max(0, int(elapsed * speed / frame_time))
    return index % count if loop else min(index, count - 1)
