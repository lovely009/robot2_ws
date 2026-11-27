"""Warm-started, bounded-time differential IK for a single WS_Geo display frame."""

import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin
from scipy.linalg import solve
from scipy.spatial.transform import Rotation

from bvh_broadcaster.generic_retarget import GenericRetargeter, _swing_rotation
from .motion_simulation import load_mujoco, position_filter
from .vdmocap import BODY, BodySkeleton, _timeline, body_bvh
from .vdmocap_import import _MappingCheck, _check_targets


class _CadJoints:
    """Keep the display model and its linkage geometry for the entire stream."""

    def __init__(self, directory, names):
        mj = load_mujoco()
        from robot_transmission.mujoco_positions import JointPositionTargets
        path = directory / 'mujoco' / f'{directory.name}.xml'
        config = json.loads(ET.parse(path).find("custom/text[@name='robot_sim_joint_position']").get('data'))
        self.model = mj.MjModel.from_xml_path(str(path))
        self.data = mj.MjData(self.model)
        self.projector = JointPositionTargets(self.model, config)
        if set(self.projector.joints) != set(names):
            raise ValueError('实时显示模型与解算关节不一致')
        self.indices = [names.index(name) for name in self.projector.joints]
        self.urdf = (directory / 'urdf' / f'{directory.name}.standalone.urdf').read_text()
        self.names = [self.model.joint(i).name for i in range(self.model.njnt)]

    def positions(self, q):
        self.projector.set_configuration(self.data, q[self.indices])
        return self.data.qpos[self.model.jnt_qposadr]


class LivePose:
    def __init__(self, model, models_root, profiles_dir, names, rest):
        from .models import _models_root, enumerate_arm_models
        root = models_root or _models_root()
        if root is None or model not in enumerate_arm_models(root):
            raise ValueError('请选择模型库中支持动捕映射的机器人模型')
        self.skeleton = BodySkeleton.from_tpose(names, rest)
        rest_frame = {'frequency': 60, 'positions': rest, 'quaternions': [[1, 0, 0, 0]] * len(BODY)}
        # Stable calibration even when the operator joins in the middle of a movement.
        text, _ = body_bvh({'world_space': 'WS_Geo', 'joints': names, 'rest': rest,
                            'frames': [dict(rest_frame, index=i) for i in range(2)]})
        urdf = (Path(root) / model / 'urdf' / f'{model}.urdf').read_text()
        checker = _MappingCheck(models_root=root, profiles_dir=profiles_dir)
        try:
            status = checker.create({'model': model, 'urdf_text': urdf, 'bvh_text': text,
                                     'retargeting': 'generic', 'scale': .01, 'up_axis': 'auto',
                                     'review_mapping': True})
            matched = _check_targets(status, model, checker.builtin)
            job = checker.jobs[status['id']]
            path = job['directory'] / 'robot.urdf'
            simulation = job['simulation']
            solver = GenericRetargeter(path, status['mapping'], job['clip'], up_axis=job['up_axis'],
                                       match_directions=bool(simulation))
        finally:
            checker.close()

        source_names = [n for n, _ in BODY]
        self.solver = solver
        self.root = source_names.index(solver.clip.joints[solver.source_root].name)
        self.source_ids = {i: source_names.index(j.name) for i, j in enumerate(solver.clip.joints) if not j.is_end}
        self.project = position_filter(simulation, solver.robot.independent) if simulation else None
        self.cad = _CadJoints(simulation['directory'], solver.names) if simulation else None
        links = status['robot_links']
        parents = {j.find('child').get('link'): j.find('parent').get('link') for j in ET.fromstring(urdf).findall('joint')}
        self.links = [solver.model.getFrameId(n, pin.FrameType.BODY) for n in links]
        self.scene = {'preview_only': True, 'model': model, 'matched_targets': matched,
                      'source_to_robot': solver.basis.tolist(),
                      'source': {'names': source_names, 'parents': [p for _, p in BODY]},
                      'robot': {'names': links, 'parents': [links.index(parents[n]) if n in parents else -1 for n in links],
                                'urdf': self.cad.urdf if self.cad else urdf,
                                'joint_names': self.cad.names if self.cad else solver.names,
                                'root_rotation_source': source_names[self.root],
                                'ignore_limits': not simulation and not solver.mechanical_limits_enforced}}
        self.q = solver.reference.copy()
        self.previous_index = None
        self.previous_rate = None
        self.derivative = solver.robot.derivative[:, solver.active]

    def _goals(self, points, rotations):
        s = self.solver
        basis = s.basis @ rotations[self.root].T
        relative = (points - points[self.root]) @ basis.T
        goals, translations = {}, {}
        for (source, target, prismatic), (p0, r0) in zip(s.targets, s.robot_reference):
            i = self.source_ids[source]
            goals[target] = basis @ rotations[i] @ s.source_reference[1][source].T @ s.basis.T @ r0
            if prismatic:
                translations[target] = p0 + relative[i] - s.basis @ s.source_reference[0][source]

        # Align body segments using the same anatomical axes as the offline fit.
        for left, right, target in s.trunk_targets:
            direction = (relative[self.source_ids[left[0]]] + relative[self.source_ids[right[0]]]) / 2
            direction -= relative[self.root]
            swing = _swing_rotation(goals[target] @ s.bone_axes[target], direction / np.linalg.norm(direction))
            joint = s.model.frames[target].parentJoint
            for f in goals:
                if joint in s.model.supports[s.model.frames[f].parentJoint]:
                    goals[f] = swing @ goals[f]
        for a, b, start, end in s.direction_targets:
            direction = relative[self.source_ids[b]] - relative[self.source_ids[a]]
            swing = _swing_rotation(goals[start] @ s.bone_axes[start], direction / np.linalg.norm(direction))
            goals[start] = swing @ goals[start]
            if end not in s.bone_axes:
                goals[end] = swing @ goals[end]
        return goals, translations

    def frame(self, frame, *, restart=False):
        _, rate = _timeline([frame])
        points, rotations = self.skeleton.world_poses([frame])
        points, rotations = points[0], rotations[0]
        s = self.solver
        index = frame['index']
        gap = (index - self.previous_index) % 2**32 if self.previous_index is not None else 0
        reset = restart or self.previous_rate != rate or gap > 1 + rate / 4
        q = s.reference.copy() if reset else self.q.copy()
        max_step = s.upper - s.lower if reset else s.velocity * (gap / rate)
        lower = np.maximum(s.lower, q - max_step)
        upper = np.minimum(s.upper, q + max_step)
        previous = q.copy()
        goals, translations = self._goals(points, rotations)
        active = s.active
        # A bounded number of damped Jacobian updates allows skipped packets to be
        # superseded immediately. Restrict the whole update to this source interval.
        for _ in range(10 if reset else 3):
            full = s.robot.affine @ q + s.robot.offsets
            pin.computeJointJacobians(s.model, s.data, s.configuration(full))
            pin.updateFramePlacements(s.model, s.data)
            errors, rows = [], []
            for target, desired in goals.items():
                pose = s.data.oMf[target]
                jac = pin.getFrameJacobian(s.model, s.data, target, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED) @ self.derivative
                errors.append(pin.log3(desired @ pose.rotation.T))
                rows.append(jac[3:])
                if target in translations:
                    errors.append(translations[target] - pose.translation)
                    rows.append(jac[:3])
            jac = np.vstack(rows)
            normal = jac.T @ jac + .02 * np.eye(len(active))
            step = solve(normal, jac.T @ np.concatenate(errors), assume_a='pos')
            candidate = np.clip(q[active] + np.clip(step, -.35, .35), lower[active], upper[active])
            change = np.max(np.abs(candidate - q[active]))
            q[active] = candidate
            if change < 1e-5:
                break
        if self.project:
            q = self.project(previous, q, max_step)
        full = s.robot.affine @ q + s.robot.offsets
        pin.framesForwardKinematics(s.model, s.data, s.configuration(full))
        display = self.cad.positions(full) if self.cad else full
        if not np.isfinite(display).all():
            raise ValueError('实时解算产生了无效关节位置')
        self.q, self.previous_index, self.previous_rate = q, index, rate
        root_rotation = s.basis @ rotations[self.root] @ s.basis.T
        return {'index': index, 'frequency': rate, 'source': points.round(6).tolist(),
                'robot': np.round([s.data.oMf[f].translation for f in self.links], 6).tolist(),
                'root_quaternion': Rotation.from_matrix(root_rotation).as_quat().tolist(),
                'joint_positions': display.round(8).tolist()}
