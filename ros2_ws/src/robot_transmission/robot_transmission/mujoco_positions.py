"""Convert output-joint targets to the position servos of a closed-loop MJCF."""

import math

import mujoco
import numpy as np
from robot_transmission.coupled_limits import CoupledLimits


class JointPositionTargets:
    def __init__(self, model, config):
        self.model = model
        self.pose = mujoco.MjData(model)
        self.joints = {name: model.joint(name).qposadr[0] for name in config["joints"]}
        self.aliases = dict(config.get("joint_aliases", {}))
        if any(target not in self.joints or (name in self.joints and name != target)
               for name, target in self.aliases.items()):
            raise ValueError("仿真关节别名必须指向实际关节且不能覆盖其他关节")
        self.targets = {name: float(model.qpos0[address]) for name, address in self.joints.items()}
        if "joint_limits" in config:
            self.limits = {name: tuple(bounds) for name, bounds in config["joint_limits"].items()}
        else:
            self.limits = {name: (-limit, limit) for name, limit in config["differential_limits"].items()}
        self.direct = {}
        self.rods = {}
        mujoco.mj_fwdPosition(model, self.pose)
        for actuator in range(model.nu):
            joint = model.joint(model.actuator_trnid[actuator, 0])
            if joint.name in self.joints:
                self.direct[actuator] = joint.name
                if joint.name not in self.limits:
                    self.limits[joint.name] = tuple(model.actuator_ctrlrange[actuator])
                continue
            tendon = model.tendon(config["flanges"][model.actuator(actuator).name])
            start = model.tendon_adr[tendon.id]
            if model.tendon_num[tendon.id] != 2 or any(
                kind != mujoco.mjtWrap.mjWRAP_SITE for kind in model.wrap_type[start:start + 2]
            ):
                raise ValueError(f"{tendon.name} 必须由两个球头定义")
            ball, platform = model.wrap_objid[start:start + 2]
            if model.site_bodyid[ball] != model.jnt_bodyid[joint.id]:
                raise ValueError(f"{tendon.name} 的首个球头必须随 {joint.name} 转动")
            self.rods[actuator] = (joint.id, ball, platform, float(self.pose.ten_length[tendon.id]))
        if self.limits.keys() != self.joints.keys():
            raise ValueError("仿真位置限位必须覆盖全部可控关节")
        if any(not np.isfinite([lower, upper]).all() or lower > 0 or upper < 0
               for lower, upper in self.limits.values()):
            raise ValueError("仿真关节限位须为包含零位的有限范围")
        self.workspace = CoupledLimits(config.get("coupled_limits", {}), self.joints)
        self.coupled = self.workspace.groups
        self._validate_coupled(self.targets)
        self.rod_frames = []
        for name in config["flanges"].values():
            body = model.body(name)
            tendon = model.tendon(name)
            start = model.tendon_adr[tendon.id]
            ball, platform = model.wrap_objid[start:start + 2]
            self.rod_frames.append((body.id, ball, platform, model.joint(name + "_x").qposadr[0],
                                    model.joint(name + "_y").qposadr[0]))

    def set_configuration(self, data, positions):
        """Set a closed kinematic configuration for CAD checks and model keyframes."""
        mujoco.mj_resetData(self.model, data)
        self.apply(data, list(self.joints), positions)
        for name, position in zip(self.joints, positions):
            data.qpos[self.joints[name]] = position
        for actuator in range(self.model.nu):
            joint = self.model.actuator_trnid[actuator, 0]
            data.qpos[self.model.jnt_qposadr[joint]] = data.ctrl[actuator]
        mujoco.mj_kinematics(self.model, data)
        for body, start, end, x, y in self.rod_frames:
            direction = data.xmat[body].reshape(3, 3).T @ (
                data.site_xpos[end] - data.site_xpos[start])
            direction /= np.linalg.norm(direction)
            data.qpos[x] = math.atan2(-direction[1], direction[2])
            data.qpos[y] = math.asin(np.clip(direction[0], -1, 1))
        mujoco.mj_kinematics(self.model, data)

    _grid_indices = staticmethod(CoupledLimits.grid_indices)

    def _validate_coupled(self, targets):
        self.workspace.validate(targets)

    def active_limits(self):
        return self.workspace.active_limits(self.limits, self.targets)

    def apply(self, data, names, positions):
        if not names:
            raise ValueError("关节名称和位置须成对，且名称不可重复")
        names = [self.aliases.get(name, name) for name in names]
        if not names or len(names) != len(positions) or len(set(names)) != len(names):
            raise ValueError("关节名称和位置须成对，且名称不可重复")
        bounded = {}
        for name, position in zip(names, positions):
            if name not in self.limits:
                raise ValueError(f"未知的仿真关节 {name}")
            lower, upper = self.limits[name]
            if not lower - 1e-12 <= position <= upper + 1e-12:
                raise ValueError(f"{name} 目标超出仿真范围 [{lower:.3f}, {upper:.3f}] rad")
            # Integer-degree grid reconstruction can differ by a few floating-point bits.
            bounded[name] = min(upper, max(lower, position))
        targets = self.targets | bounded
        self._validate_coupled(targets)
        for name, address in self.joints.items():
            self.pose.qpos[address] = targets[name]
        mujoco.mj_kinematics(self.model, self.pose)
        controls = np.empty(self.model.nu)
        for actuator, name in self.direct.items():
            controls[actuator] = targets[name]
        for actuator, (joint, ball, platform, length) in self.rods.items():
            # The motor ball moves on a circle about the hinge. Intersect it
            # with the sphere of radius length centred on the platform ball.
            axis = self.pose.xaxis[joint]
            radius = self.pose.site_xpos[ball] - self.pose.xanchor[joint]
            radial = radius - axis * np.dot(axis, radius)
            centre = self.pose.site_xpos[platform] - self.pose.xanchor[joint] - (radius - radial)
            cosine = float(np.dot(centre, radial))
            sine = float(np.dot(centre, np.cross(axis, radial)))
            offset = (np.dot(centre, centre) + np.dot(radial, radial) - length**2) / 2
            amplitude = math.hypot(cosine, sine)
            lower, upper = self.model.actuator_ctrlrange[actuator]
            angles = []
            if amplitude > 1e-12 and abs(offset) <= amplitude:
                phase = math.atan2(sine, cosine)
                delta = math.acos(offset / amplitude)
                angles = [math.remainder(phase + sign * delta, math.tau) for sign in (-1, 1)]
            valid = [angle for angle in angles if lower <= angle <= upper]
            if not valid:
                raise ValueError(f"{self.model.actuator(actuator).name} 无法到达该差动姿态")
            controls[actuator] = min(valid, key=lambda angle: abs(angle - data.ctrl[actuator]))
        data.ctrl[:] = controls
        self.targets = targets
