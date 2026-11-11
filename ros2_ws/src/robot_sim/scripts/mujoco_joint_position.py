"""Convert output-joint targets to the position servos of a closed-loop MJCF."""

import math

import mujoco
import numpy as np


class JointPositionTargets:
    def __init__(self, model, config):
        self.model = model
        self.pose = mujoco.MjData(model)
        self.joints = {name: model.joint(name).qposadr[0] for name in config["joints"]}
        self.targets = {name: float(model.qpos0[address]) for name, address in self.joints.items()}
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

    def apply(self, data, names, positions):
        if not names or len(names) != len(positions) or len(set(names)) != len(names):
            raise ValueError("关节名称和位置须成对，且名称不可重复")
        for name, position in zip(names, positions):
            if name not in self.limits:
                raise ValueError(f"未知的仿真关节 {name}")
            lower, upper = self.limits[name]
            if not lower <= position <= upper:
                raise ValueError(f"{name} 目标超出仿真范围 [{lower:.3f}, {upper:.3f}] rad")
        targets = self.targets | dict(zip(names, positions))
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
