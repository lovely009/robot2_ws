"""Confirmed BVH targets on an arbitrary single-axis URDF tree, preview only."""

import copy
import re
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def _normalized(name):
    text = re.sub(r'([a-z])([A-Z])', r'\1_\2', name).lower()
    words = [w for w in re.split(r'[\W_]+', text) if w not in ('joint', 'link')]
    return ''.join(words)


def _role(name):
    tokens = re.split(r'[^a-z]+', re.sub(r'([a-z])([A-Z])', r'\1_\2', name).lower())
    name = _normalized(name)
    side = 'left' if 'left' in name or 'l' in tokens else 'right' if 'right' in name or 'r' in tokens else ''
    if not side:
        return None
    for role, aliases in (
        ('forearm', ('forearm', 'lowerarm', 'elbow')),
        ('upperarm', ('upperarm', 'arm')),
        ('shoulder', ('shoulder', 'clavicle', 'collar')),
        ('hand', ('hand', 'wrist')),
        ('lowerleg', ('lowerleg', 'shin', 'knee', 'calf')),
        ('upperleg', ('upperleg', 'thigh', 'hip')),
        ('toe', ('toe',)), ('foot', ('foot', 'ankle')),
    ):
        if any(alias in name for alias in aliases):
            return side, role
    return None


class _Robot:
    def __init__(self, urdf):
        text = urdf.read_text()
        if '<!DOCTYPE' in text.upper() or '<!ENTITY' in text.upper():
            raise ValueError('URDF 不支持 DTD 或实体')
        root = ET.fromstring(text)
        self.links = [link.get('name') for link in root.findall('link')]
        joints = root.findall('joint')
        names = [j.get('name') for j in joints]
        if len(set(names)) != len(names) or len(set(self.links)) != len(self.links):
            raise ValueError('URDF 关节和连杆名称不能重复')
        for joint in joints:
            if joint.get('type') not in ('fixed', 'revolute', 'continuous', 'prismatic'):
                raise ValueError(f"暂不支持关节 {joint.get('name')} 的 {joint.get('type')} 类型")
        self.spec = {j.get('name'): j for j in joints if j.get('type') != 'fixed'}
        if not self.spec:
            raise ValueError('URDF 没有可转换的单轴关节')
        # Parser placeholders stay in memory; constraints and exported URDF use the original values.
        parsed = copy.deepcopy(root)
        for joint in parsed.findall('joint'):
            if joint.get('type') == 'fixed':
                continue
            limit = joint.find('limit')
            if limit is None:
                limit = ET.SubElement(joint, 'limit')
            for field in ('lower', 'upper', 'velocity', 'effort'):
                value = float(limit.get(field, 'nan'))
                if not np.isfinite(value):
                    limit.set(field, '0')
        self.model = pin.buildModelFromXML(ET.tostring(parsed, encoding='unicode'))
        self.data = self.model.createData()
        self.names = list(self.model.names)[1:]
        if set(self.names) != set(self.spec) or any(j.nv != 1 for j in list(self.model.joints)[1:]):
            raise ValueError('URDF 必须是完整的单轴关节树')
        self.independent = [n for n in self.names if self.spec[n].find('mimic') is None]
        if not self.independent:
            raise ValueError('mimic 关系缺少独立关节')
        self.affine = np.zeros((len(self.names), len(self.independent)))
        self.offsets = np.zeros(len(self.names))
        self.lower, self.upper, self.velocity, self.missing = [], [], [], []
        self.warnings = []

        def relation(name, visiting=()):
            if name not in self.spec or name in visiting:
                raise ValueError(f'mimic 引用缺失或循环：{name}')
            mimic = self.spec[name].find('mimic')
            if mimic is None:
                return name, 1., 0.
            leader = mimic.get('joint')
            if leader not in self.spec or ((self.spec[name].get('type') == 'prismatic') !=
                                          (self.spec[leader].get('type') == 'prismatic')):
                raise ValueError(f'mimic 关节类型或引用不相容：{name}')
            base, scale, offset = relation(leader, (*visiting, name))
            a, b = float(mimic.get('multiplier', 1)), float(mimic.get('offset', 0))
            if not np.isfinite([a, b]).all():
                raise ValueError(f'mimic 参数无效：{name}')
            return base, a * scale, a * offset + b

        for i, name in enumerate(self.names):
            joint = self.spec[name]
            leader, scale, offset = relation(name)
            self.affine[i, self.independent.index(leader)] = scale
            self.offsets[i] = offset
            limit = joint.find('limit')
            continuous = joint.get('type') == 'continuous'
            lo = float(limit.get('lower', 'nan')) if limit is not None and not continuous else -np.inf
            hi = float(limit.get('upper', 'nan')) if limit is not None and not continuous else np.inf
            velocity = float(limit.get('velocity', 'nan')) if limit is not None else np.nan
            position_missing = not continuous and (not np.isfinite([lo, hi]).all() or lo == hi)
            velocity_missing = not np.isfinite(velocity) or velocity <= 0
            if np.isfinite([lo, hi]).all() and lo > hi:
                raise ValueError(f'关节 {name} 的上下限颠倒')
            self.lower.append(-np.inf if not np.isfinite(lo) or lo == hi else lo)
            self.upper.append(np.inf if not np.isfinite(hi) or lo == hi else hi)
            self.velocity.append(np.inf if velocity_missing else velocity)
            self.missing.append(position_missing or velocity_missing)
            if position_missing or velocity_missing:
                self.warnings.append(f'{name}：缺少有效' +
                                     ('位置' if position_missing else '') +
                                     ('/速度' if velocity_missing else '') + '限位')
        self.lower, self.upper, self.velocity = map(np.asarray, (self.lower, self.upper, self.velocity))
        self.derivative = np.zeros((self.model.nv, len(self.independent)))
        for i, name in enumerate(self.names):
            self.derivative[self.model.joints[self.model.getJointId(name)].idx_v] = self.affine[i]

    def bounds(self):
        lower, upper = np.full(len(self.independent), -np.inf), np.full(len(self.independent), np.inf)
        velocity = np.full(len(self.independent), np.inf)
        for i, row in enumerate(self.affine):
            indices = np.flatnonzero(row)
            if not len(indices):
                if not self.lower[i] <= self.offsets[i] <= self.upper[i]:
                    raise ValueError(f'mimic 常量违反限位：{self.names[i]}')
                continue
            j = indices[0]
            ends = sorted(((self.lower[i] - self.offsets[i]) / row[j],
                           (self.upper[i] - self.offsets[i]) / row[j]))
            lower[j], upper[j] = max(lower[j], ends[0]), min(upper[j], ends[1])
            velocity[j] = min(velocity[j], self.velocity[i] / abs(row[j]))
        if np.any(lower > upper):
            raise ValueError('mimic 与主关节限位不相容')
        return lower, upper, velocity

    def configuration(self, positions):
        q = pin.neutral(self.model)
        for name, value in zip(self.names, positions):
            joint = self.model.joints[self.model.getJointId(name)]
            q[joint.idx_q:joint.idx_q + joint.nq] = (
                [np.cos(value), np.sin(value)] if joint.nq == 2 else value)
        return q


def analyze(urdf, clip, *, up_axis='y_up', scale=.01):
    robot = _Robot(urdf)
    sources = [j.name for j in clip.joints if not j.is_end]
    targets = []
    for name in robot.names:
        if robot.spec[name].find('mimic') is not None:
            continue
        target = robot.spec[name].find('child').get('link')
        exact = [s for s in sources if _normalized(s) in (_normalized(target), _normalized(name))]
        semantic = [s for s in sources if _role(s) is not None and _role(s) in (_role(target), _role(name))]
        matches = exact or semantic
        targets.append({'target': target, 'source': matches[0] if len(matches) == 1 else '',
                        'confidence': 'name' if len(exact) == 1 else 'semantic' if len(matches) == 1 else 'unknown',
                        'reason': '名称对应，需核对参考姿势' if len(exact) == 1 else
                                  '人体语义候选，需确认' if len(matches) == 1 else
                                  '存在多个候选' if matches else '无法从名称确定人体部位'})
    return {'confirmation_required': True, 'warnings': robot.warnings,
            'robot_joints': robot.names,
            'mapping': {'mode': 'generic', 'source_root': sources[0], 'reference_frame': 0,
                        'source_to_robot': np.eye(3).tolist(), 'limit_policy': 'urdf', 'targets': targets}}


class GenericRetargeter:
    def __init__(self, urdf, mapping, clip, *, up_axis='y_up', scale=.01):
        self.robot = _Robot(urdf)
        self.model, self.data, self.names = self.robot.model, self.robot.data, self.robot.names
        self.clip, self.up_axis, self.scale = clip, up_axis, scale
        if (up_axis not in ('y_up', 'z_up') or not np.isfinite(scale) or scale <= 0
                or not np.isfinite(clip.frames).all() or not len(clip.frames)
                or not np.isfinite(clip.frame_time) or clip.frame_time <= 0):
            raise ValueError('BVH 坐标、单位或帧时间无效')
        if not isinstance(mapping, dict) or mapping.get('mode') != 'generic':
            raise ValueError('需要通用映射')
        self.basis = np.asarray(mapping.get('source_to_robot'), dtype=float)
        if (self.basis.shape != (3, 3) or not np.isfinite(self.basis).all()
                or not np.allclose(self.basis.T @ self.basis, np.eye(3))
                or not np.isclose(np.linalg.det(self.basis), 1)):
            raise ValueError('坐标变换必须是有效旋转矩阵')
        reference = mapping.get('reference_frame', 0)
        if type(reference) is not int or not 0 <= reference < len(clip.frames):
            raise ValueError('参考帧超出动作范围')
        self.reference_frame = reference
        source_names = [j.name for j in clip.joints]
        root = mapping.get('source_root')
        if root not in source_names:
            raise ValueError('人体根参考不存在')
        self.source_root = source_names.index(root)
        if mapping.get('limit_policy') not in ('urdf', 'preview'):
            raise ValueError('限位策略无效')
        self.mechanical_limits_enforced = not any(self.robot.missing)
        self.lower, self.upper, self.velocity = self.robot.bounds()
        self.reference = np.clip(np.zeros(len(self.robot.independent)), self.lower, self.upper)
        self.targets, covered, used = [], set(), set()
        entries = mapping.get('targets')
        if not isinstance(entries, list) or len(entries) > len(self.robot.links):
            raise ValueError('目标对应表无效')
        for entry in entries:
            if not isinstance(entry, dict):
                raise ValueError('目标对应条目无效')
            target, source = entry.get('target'), entry.get('source')
            if target not in self.robot.links:
                raise ValueError(f'机器人连杆不存在：{target}')
            if target in used:
                raise ValueError(f'机器人目标重复：{target}')
            used.add(target)
            if not source:
                continue
            if source not in source_names or clip.joints[source_names.index(source)].is_end:
                raise ValueError(f'人体关节不存在：{source}')
            frame = self.model.getFrameId(target, pin.FrameType.BODY)
            chain = list(self.model.supports[self.model.frames[frame].parentJoint])[1:]
            indices = {k for j in chain for k in np.flatnonzero(self.robot.derivative[self.model.joints[j].idx_v])}
            if not indices:
                raise ValueError(f'机器人目标 {target} 没有可动关节')
            covered.update(indices)
            translation = any(self.robot.spec[self.model.names[j]].get('type') == 'prismatic' for j in chain)
            self.targets.append((source_names.index(source), frame, translation))
        if not self.targets:
            raise ValueError('请至少指定一个人体与机器人目标对应')
        affected = [bool(set(np.flatnonzero(row)) & covered) for row in self.robot.affine]
        if mapping['limit_policy'] == 'urdf' and any(m and a for m, a in zip(self.robot.missing, affected)):
            raise ValueError('目标链缺少有效限位；确认 CAD 仅几何预览后才能转换')
        self.active = np.array([i for i in sorted(covered) if self.lower[i] < self.upper[i]], dtype=int)
        if not len(self.active):
            raise ValueError('目标链没有可求解的关节范围')
        self.report = {'preview_only': True, 'mechanical_limits_enforced': self.mechanical_limits_enforced,
                       'limit_policy': mapping['limit_policy'], 'warnings': self.robot.warnings,
                       'covered_joints': [n for n, a in zip(self.names, affected) if a],
                       'unmapped_joints': [n for n, a in zip(self.names, affected) if not a],
                       'reference_frame': reference, 'root_translation': 'fixed'}
        self.source_reference = self._source_pose(reference)
        neutral = self.robot.affine @ self.reference + self.robot.offsets
        pin.framesForwardKinematics(self.model, self.data, self.configuration(neutral))
        self.robot_reference = [(self.data.oMf[f].translation.copy(), self.data.oMf[f].rotation.copy())
                                for _, f, _ in self.targets]
        self.nonconverged = 0

    def configuration(self, positions):
        return self.robot.configuration(positions)

    def _source_pose(self, frame):
        positions, rotations = [], []
        for joint, (position, quaternion) in zip(self.clip.joints, self.clip.local_transforms(
                frame, scale=self.scale, up_axis=self.up_axis, apply_root_offset=False)):
            rotation = Rotation.from_quat(quaternion).as_matrix()
            if joint.parent >= 0:
                position = positions[joint.parent] + rotations[joint.parent] @ position
                rotation = rotations[joint.parent] @ rotation
            positions.append(position)
            rotations.append(rotation)
        base = rotations[self.source_root].T
        return (np.asarray([base @ (p - positions[self.source_root]) for p in positions]),
                np.asarray([base @ r for r in rotations]))

    def convert(self, progress=None):
        current = self.reference.copy()
        positions, errors, translation_errors = [], [], []
        active = self.active
        derivative = self.robot.derivative[:, active]
        for frame in range(len(self.clip.frames)):
            if progress:
                progress(frame, len(self.clip.frames))
            ps, rs = self._source_pose(frame)
            desired = [(p0 + self.basis @ (ps[s] - self.source_reference[0][s]),
                        self.basis @ rs[s] @ self.source_reference[1][s].T @ self.basis.T @ r0)
                       for (s, _, _), (p0, r0) in zip(self.targets, self.robot_reference)]
            previous = current.copy()
            cached_x, cached = None, None

            def evaluate(values):
                nonlocal cached_x, cached
                if cached_x is not None and np.array_equal(cached_x, values):
                    return cached
                q = previous.copy()
                q[active] = values
                full = self.robot.affine @ q + self.robot.offsets
                pin.computeJointJacobians(self.model, self.data, self.configuration(full))
                pin.updateFramePlacements(self.model, self.data)
                residuals, jacobians = [], []
                for (_, target, translation), (position, rotation) in zip(self.targets, desired):
                    pose = self.data.oMf[target]
                    jac = pin.getFrameJacobian(self.model, self.data, target,
                                              pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).reshape(6, self.model.nv) @ derivative
                    error = rotation.T @ pose.rotation
                    residuals.append(pin.log3(error))
                    jacobians.append(pin.Jlog3(error) @ pose.rotation.T @ jac[3:])
                    if translation:
                        residuals.append(pose.translation - position)
                        jacobians.append(jac[:3])
                residuals.append(.01 * (values - previous[active]))
                jacobians.append(.01 * np.eye(len(active)))
                cached_x, cached = values.copy(), (np.concatenate(residuals), np.vstack(jacobians))
                return cached

            fit = least_squares(lambda q: evaluate(q)[0], current[active],
                                jac=lambda q: evaluate(q)[1],
                                bounds=(self.lower[active], self.upper[active]),
                                max_nfev=60, ftol=1e-7, xtol=1e-7, gtol=1e-7)
            if not np.isfinite(fit.x).all():
                raise ValueError(f'第 {frame} 帧求解产生非有限值')
            self.nonconverged += int(not fit.success)
            step = self.velocity[active] * self.clip.frame_time
            current[active] += np.clip(fit.x - current[active], -step, step)
            full = self.robot.affine @ current + self.robot.offsets
            positions.append(full)
            pin.framesForwardKinematics(self.model, self.data, self.configuration(full))
            errors.append([float(np.degrees(np.linalg.norm(pin.log3(r.T @ self.data.oMf[f].rotation))))
                           for (_, f, _), (_, r) in zip(self.targets, desired)])
            translation_errors.append([float(np.linalg.norm(p - self.data.oMf[f].translation))
                                       for (_, f, translation), (p, _) in zip(self.targets, desired) if translation])
        positions = np.asarray(positions)
        self.validate_result(positions)
        self.report.update(
            orientation_targets=[self.model.frames[f].name for _, f, _ in self.targets],
            orientation_mean_deg=np.mean(errors, axis=0).tolist(),
            orientation_max_deg=float(np.max(errors)),
            translation_targets=[self.model.frames[f].name for _, f, translation in self.targets if translation],
            translation_mean_m=np.mean(translation_errors, axis=0).tolist())
        return positions, np.empty((len(positions), 0)), np.asarray(errors)

    def validate_result(self, positions):
        if positions.shape != (len(self.clip.frames), len(self.names)) or not np.isfinite(positions).all():
            raise ValueError('转换关节结果的形状或数值无效')
        if (np.any(positions < self.robot.lower - 1e-8) or np.any(positions > self.robot.upper + 1e-8)
                or np.any(np.abs(np.diff(positions, axis=0)) / self.clip.frame_time > self.robot.velocity + 1e-6)):
            raise ValueError('转换结果违反有效 URDF 限位')
