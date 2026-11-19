"""Fixed-base arm retargeting for offline preview; no robot control publishers."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import pinocchio as pin
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import yaml

from .bvh import BvhClip


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class ArmRetargeter:
    def __init__(self, urdf, mapping, clip, *, up_axis='y_up'):
        self.model = pin.buildModelFromUrdf(str(urdf))
        self.data = self.model.createData()
        self.names = list(self.model.names)[1:]
        if any(j.nq != 1 or j.nv != 1 for j in list(self.model.joints)[1:]):
            raise ValueError('Only bounded one-DOF joints are supported')
        self.lower = self.model.lowerPositionLimit.copy()
        self.upper = self.model.upperPositionLimit.copy()
        self.velocity = self.model.velocityLimit.copy()
        if (not np.isfinite([self.lower, self.upper, self.velocity]).all()
                or np.any(self.lower >= self.upper) or np.any(self.velocity <= 0)):
            raise ValueError('URDF requires finite position bounds and positive velocity limits')
        self.clip = clip
        self.up_axis = up_axis
        self.basis = np.asarray(mapping['source_to_robot'], dtype=float)
        if (self.basis.shape != (3, 3) or not np.allclose(self.basis.T @ self.basis, np.eye(3))
                or not np.isclose(np.linalg.det(self.basis), 1)):
            raise ValueError('source_to_robot must be a proper 3x3 rotation')
        source_names = [joint.name for joint in clip.joints]

        def source(name):
            if name not in source_names:
                raise ValueError(f'BVH joint missing: {name}')
            return source_names.index(name)

        self.torso = source(mapping['torso'])
        self.source_frames, self.robot_frames, self.arm_indices, covered = [], [], [], set()
        self.arm_names = list(mapping['arms'])
        if not self.arm_names:
            raise ValueError('At least one arm mapping is required')
        for arm in mapping['arms'].values():
            if len(arm['source']) != 3 or len(arm['robot']) != 3:
                raise ValueError('Each arm requires shoulder, elbow and hand')
            self.source_frames.append([source(name) for name in arm['source']])
            frames = []
            for name in arm['robot']:
                if not self.model.existFrame(name, pin.FrameType.BODY):
                    raise ValueError(f'URDF link missing: {name}')
                frames.append(self.model.getFrameId(name, pin.FrameType.BODY))
            parents = [self.model.frames[f].parentJoint for f in frames]
            if not all(a in self.model.supports[b] and a != b
                       for a, b in zip(parents, parents[1:])):
                raise ValueError('Robot shoulder/elbow/hand must follow one joint chain')
            chain = set(self.model.supports[parents[-1]]) - {0}
            if chain & covered:
                raise ValueError('This fixed-base mapper requires independent arm chains')
            covered.update(chain)
            self.arm_indices.append([self.model.joints[j].idx_q for j in sorted(chain)])
            self.robot_frames.append(frames)
        if covered - {0} != set(range(1, self.model.njoints)):
            raise ValueError('URDF has unmapped joints; update the semantic mapping')
        self.reference = np.clip(pin.neutral(self.model), self.lower, self.upper)
        self.nonconverged = 0
        directions, rotations = self.targets(0)
        self.source_reference = rotations.copy()
        # An all-zero start on one-sided bounds can stall SciPy's initial trust region.
        interior = np.clip(self.reference, self.lower + 0.05 * (self.upper - self.lower),
                           self.upper - 0.05 * (self.upper - self.lower))
        self.reference = self.solve(directions, None, interior, self.lower, self.upper)
        self.robot_reference = self.poses(self.reference)[1].copy()

    def targets(self, frame):
        positions, rotations = [], []
        for joint, (p, quat) in zip(self.clip.joints, self.clip.local_transforms(
                frame, up_axis=self.up_axis, apply_root_offset=False)):
            r = Rotation.from_quat(quat).as_matrix()
            if joint.parent >= 0:
                p = positions[joint.parent] + rotations[joint.parent] @ p
                r = rotations[joint.parent] @ r
            positions.append(p)
            rotations.append(r)
        torso_rotation = rotations[self.torso]
        directions, hands = [], []
        for shoulder, elbow, hand in self.source_frames:
            points = np.array([positions[i] for i in (shoulder, elbow, hand)])
            segments = np.diff(points, axis=0)
            lengths = np.linalg.norm(segments, axis=1)
            if np.any(lengths < 1e-8):
                raise ValueError('BVH arm contains a zero-length segment')
            directions.append((self.basis @ torso_rotation.T @ (segments / lengths[:, None]).T).T)
            hands.append(torso_rotation.T @ rotations[hand])
        return np.array(directions), np.array(hands)

    def poses(self, q):
        pin.computeJointJacobians(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        points = np.array([[self.data.oMf[f].translation.copy() for f in arm]
                           for arm in self.robot_frames])
        hands = np.array([self.data.oMf[arm[-1]].rotation.copy() for arm in self.robot_frames])
        return points, hands

    def residual(self, q, directions, orientations):
        points, hands = self.poses(q)
        residuals, jacobians = [], []
        for arm, frames in enumerate(self.robot_frames):
            js = [pin.getFrameJacobian(self.model, self.data, f,
                                      pin.ReferenceFrame.LOCAL_WORLD_ALIGNED) for f in frames]
            for i in range(2):
                d = points[arm, i + 1] - points[arm, i]
                length = np.linalg.norm(d)
                if length < 1e-8:
                    raise ValueError('URDF arm contains a zero-length segment')
                u = d / length
                residuals.append(u - directions[arm, i])
                jacobians.append((np.eye(3) - np.outer(u, u)) @ (js[i + 1][:3] - js[i][:3]) / length)
            if orientations is not None:
                error = orientations[arm].T @ hands[arm]
                residuals.append(0.35 * pin.log3(error))
                jacobians.append(0.35 * pin.Jlog3(error) @ hands[arm].T @ js[-1][3:])
        residuals.append(0.025 * (q - self.reference))
        jacobians.append(0.025 * np.eye(self.model.nv))
        return np.concatenate(residuals), np.vstack(jacobians)

    def solve(self, directions, orientations, initial, lower, upper):
        result = least_squares(
            lambda q: self.residual(q, directions, orientations)[0],
            np.clip(initial, lower, upper),
            jac=lambda q: self.residual(q, directions, orientations)[1],
            bounds=(lower, upper), max_nfev=60, ftol=1e-6, xtol=1e-6, gtol=1e-6)
        if not np.isfinite(result.x).all():
            raise ValueError('IK returned non-finite joint angles')
        self.nonconverged += int(not result.success)
        return result.x

    def convert(self, progress=None):
        q = self.reference.copy()
        fit = q.copy()
        values, direction_errors, orientation_errors = [], [], []
        for frame in range(len(self.clip.frames)):
            directions, rotations = self.targets(frame)
            targets = np.array([self.basis @ (r @ ref.T) @ self.basis.T @ robot_ref
                                for r, ref, robot_ref in zip(
                                    rotations, self.source_reference, self.robot_reference)])
            step = self.velocity * self.clip.frame_time
            # Narrow velocity boxes can trap IK in a bad elbow branch indefinitely.
            candidates = [self.solve(directions, targets, seed, self.lower, self.upper)
                          for seed in (fit, self.reference, (self.lower + self.upper) / 2)]
            residuals = [self.residual(c, directions, targets)[0] for c in candidates]
            for arm, indices in enumerate(self.arm_indices):
                scores = [np.linalg.norm(r[arm * 9:(arm + 1) * 9]) ** 2
                          + 0.025 ** 2 * np.linalg.norm(c[indices] - self.reference[indices]) ** 2
                          + 0.003 * np.linalg.norm(c[indices] - fit[indices]) ** 2
                          for c, r in zip(candidates, residuals)]
                fit[indices] = candidates[int(np.argmin(scores))][indices]
            q = q + np.clip(fit - q, -step, step)
            points, hands = self.poses(q)
            segments = np.diff(points, axis=1)
            segments /= np.linalg.norm(segments, axis=2)[:, :, None]
            direction_errors.append(np.degrees(np.arccos(np.clip(
                np.sum(segments * directions, axis=2), -1, 1))))
            orientation_errors.append([np.degrees(np.linalg.norm(pin.log3(t.T @ h)))
                                       for t, h in zip(targets, hands)])
            values.append(q.copy())
            if progress and frame % 300 == 0:
                progress(frame, len(self.clip.frames))
        return np.array(values), np.array(direction_errors), np.array(orientation_errors)


def load_motion(path, urdf):
    """Reject stale exports when the model changes; only numeric NPZ arrays are loaded."""
    with np.load(path, allow_pickle=False) as data:
        if str(data['urdf_sha256']) != digest(urdf):
            raise ValueError('URDF changed; regenerate the retargeted motion')
        names, positions, dt = data['joint_names'].tolist(), data['positions'].copy(), float(data['frame_time'])
    model = pin.buildModelFromUrdf(str(urdf))
    if (names != list(model.names)[1:] or positions.ndim != 2 or not len(positions)
            or positions.shape[1] != model.nq or model.nq != model.nv
            or not np.isfinite(positions).all() or not np.isfinite(dt) or dt <= 0):
        raise ValueError('Invalid retargeted motion shape, joint names or timing')
    if (np.any(positions < model.lowerPositionLimit - 1e-8)
            or np.any(positions > model.upperPositionLimit + 1e-8)
            or np.any(np.abs(np.diff(positions, axis=0)) / dt > model.velocityLimit + 1e-6)):
        raise ValueError('Motion exceeds URDF position or velocity limits')
    return names, positions, dt


def main():
    from ament_index_python.packages import get_package_share_directory
    parser = argparse.ArgumentParser(description='BVH arms → fixed-base robot IK (preview only)')
    parser.add_argument('bvh_file')
    parser.add_argument('--urdf', help='Target URDF; defaults to the project half_body_robot model')
    parser.add_argument('--mapping', default=str(Path(get_package_share_directory('bvh_broadcaster')) /
                                               'config/half_body_robot.yaml'))
    parser.add_argument('--output', required=True, help='New .npz output path; existing files are refused')
    args = parser.parse_args()
    if args.urdf is None:
        args.urdf = str(Path(get_package_share_directory('robot_description')) /
                        'models/half_body_robot/urdf/half_body_robot.urdf')
    output = Path(args.output)
    report_path = output.with_suffix('.json')
    if output.suffix != '.npz' or output.exists() or report_path.exists():
        parser.error('Use a new .npz output path (and matching .json report)')
    urdf_hash, mapping_hash = digest(args.urdf), digest(args.mapping)
    mapping = yaml.safe_load(Path(args.mapping).read_text())
    clip = BvhClip.load(args.bvh_file)
    solver = ArmRetargeter(args.urdf, mapping, clip)
    positions, direction_errors, orientation_errors = solver.convert(
        lambda i, n: print(f'IK {i}/{n}', flush=True))
    if urdf_hash != digest(args.urdf) or mapping_hash != digest(args.mapping):
        raise ValueError('URDF/mapping changed during conversion; retry with a stable model')
    with output.open('xb') as stream:
        np.savez_compressed(stream, positions=positions, joint_names=np.array(solver.names),
                            frame_time=clip.frame_time, urdf_sha256=urdf_hash,
                            mapping_sha256=mapping_hash, direction_error_deg=direction_errors,
                            orientation_error_deg=orientation_errors)
    load_motion(output, args.urdf)
    report = {'preview_only': True, 'bvh_file': str(Path(args.bvh_file).resolve()),
              'urdf': str(Path(args.urdf).resolve()), 'urdf_sha256': urdf_hash,
              'mapping': mapping, 'frames': len(positions), 'frame_time': clip.frame_time,
              'joint_names': solver.names, 'arms': {}, 'nonconverged_solves': solver.nonconverged,
              'max_velocity_rad_s': np.max(np.abs(np.diff(positions, axis=0)) /
                                           clip.frame_time, axis=0).tolist() if len(positions) > 1 else []}
    for i, name in enumerate(solver.arm_names):
        report['arms'][name] = {
            'direction_mean_deg': np.mean(direction_errors[:, i], axis=0).tolist(),
            'direction_p95_deg': np.percentile(direction_errors[:, i], 95, axis=0).tolist(),
            'direction_max_deg': np.max(direction_errors[:, i], axis=0).tolist(),
            'hand_orientation_mean_deg': float(np.mean(orientation_errors[:, i])),
            'hand_orientation_p95_deg': float(np.percentile(orientation_errors[:, i], 95))}
    with report_path.open('x') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(f'Saved {output} and {report_path}', flush=True)


if __name__ == '__main__':
    main()
