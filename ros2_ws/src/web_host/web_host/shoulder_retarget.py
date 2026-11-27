"""The confirmed two-joint shoulder mapping, for offline preview only."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pinocchio as pin
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


class ShoulderRetargeter:
    joints = ['joint_link1', 'joint_link2']
    source = 'shoulder_r'
    frame = 'Joint_link2'

    @staticmethod
    def supports(model, clip):
        try:
            urdf = (Path(get_package_share_directory('robot_description')) /
                    'models/half_body_robot/urdf/half_body_robot.urdf')
            reference = pin.buildModelFromUrdf(str(urdf))
        except (OSError, RuntimeError, LookupError):
            return False
        if model != reference or not model.existFrame(ShoulderRetargeter.frame, pin.FrameType.BODY):
            return False
        ids = [model.getJointId(name) for name in ShoulderRetargeter.joints]
        frame = model.frames[model.getFrameId(ShoulderRetargeter.frame, pin.FrameType.BODY)]
        return (list(model.supports[frame.parentJoint]) == [0, *ids]
                and any(j.name == ShoulderRetargeter.source and j.parent == 0 for j in clip.joints))

    def __init__(self, urdf, mapping, clip, *, up_axis='y_up'):
        self.model = pin.buildModelFromUrdf(str(urdf))
        if not self.supports(self.model, clip):
            raise ValueError('肩关节映射仅支持已确认的 half_body_robot 与 shoulder_r 骨架')
        if (not isinstance(mapping, dict) or mapping.get('mode') != 'shoulders12'
                or mapping.get('source') != self.source or mapping.get('joints') != self.joints):
            raise ValueError('肩关节映射必须使用 shoulder_r 与 joint_link1、joint_link2')
        self.basis = np.asarray(mapping.get('source_to_robot'), dtype=float)
        if (self.basis.shape != (3, 3) or not np.isfinite(self.basis).all()
                or not np.allclose(self.basis.T @ self.basis, np.eye(3))
                or not np.isclose(np.linalg.det(self.basis), 1)):
            raise ValueError('source_to_robot must be a proper 3x3 rotation')
        if up_axis not in ('y_up', 'z_up'):
            raise ValueError('不支持的 BVH 坐标')
        self.data = self.model.createData()
        self.names = list(self.model.names)[1:]
        self.indices = [self.model.joints[self.model.getJointId(n)].idx_q for n in self.joints]
        self.q = np.clip(pin.neutral(self.model), self.model.lowerPositionLimit,
                         self.model.upperPositionLimit)
        self.lower = self.model.lowerPositionLimit[self.indices]
        self.upper = self.model.upperPositionLimit[self.indices]
        if (not np.isfinite([self.lower, self.upper]).all() or np.any(self.lower >= self.upper)
                or not np.isfinite(clip.frames).all() or not len(clip.frames)
                or not np.isfinite(clip.frame_time) or clip.frame_time <= 0):
            raise ValueError('肩关节映射需要有限限位和有效的 BVH 帧、时间')
        self.clip, self.up_axis = clip, up_axis
        self.source_index = next(i for i, joint in enumerate(clip.joints) if joint.name == self.source)
        self.frame_id = self.model.getFrameId(self.frame, pin.FrameType.BODY)
        pin.framesForwardKinematics(self.model, self.data, self.q)
        self.robot_reference = self.data.oMf[self.frame_id].rotation.copy()
        self.source_reference = self._rotation(0)
        self.nonconverged = 0

    def _rotation(self, frame):
        rotations = []
        for joint, (_, quat) in zip(self.clip.joints, self.clip.local_transforms(
                frame, up_axis=self.up_axis, apply_root_offset=False)):
            rotation = Rotation.from_quat(quat).as_matrix()
            if joint.parent >= 0:
                rotation = rotations[joint.parent] @ rotation
            rotations.append(rotation)
        return rotations[0].T @ rotations[self.source_index]

    def _residual(self, values, target):
        self.q[self.indices] = values
        pin.framesForwardKinematics(self.model, self.data, self.q)
        return pin.log3(target.T @ self.data.oMf[self.frame_id].rotation)

    def convert(self, progress=None):
        values = self.q[self.indices].copy()
        positions, errors = [], []
        for frame in range(len(self.clip.frames)):
            target = (self.basis @ self._rotation(frame) @ self.source_reference.T
                      @ self.basis.T @ self.robot_reference)
            fit = least_squares(self._residual, values, args=(target,),
                                bounds=(self.lower, self.upper), max_nfev=40,
                                ftol=1e-8, xtol=1e-8, gtol=1e-8)
            if not fit.success or not np.isfinite(fit.x).all():
                raise ValueError(f'第 {frame} 帧肩关节姿态映射失败')
            values = fit.x
            error = np.degrees(np.linalg.norm(self._residual(values, target)))
            positions.append(self.q.copy())
            errors.append([error])
            if progress:
                progress(frame, len(self.clip.frames))
        return np.asarray(positions), np.empty((len(positions), 0)), np.asarray(errors)
