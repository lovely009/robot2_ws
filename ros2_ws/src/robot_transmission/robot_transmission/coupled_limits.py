"""Sampled two-joint workspaces used to validate joint pairs for control and offline motion preview."""

import math

import numpy as np


class CoupledLimits:
    def __init__(self, config, joint_names):
        self.groups = {}
        for name, workspace in config.items():
            joints, rows = workspace["joints"], workspace["valid_rows"]
            lower, step = np.asarray(workspace["lower"], dtype=float), float(workspace["step"])
            if (len(joints) != 2 or len(set(joints)) != 2 or not set(joints) <= set(joint_names)
                    or lower.shape != (2,) or not np.isfinite(lower).all()
                    or not math.isfinite(step) or step <= 0 or not rows or not rows[0]
                    or any(len(row) != len(rows[0]) or set(row) - {"0", "1"} for row in rows)):
                raise ValueError(f"{name} 的差动可动区域无效")
            mask = np.array([[value == "1" for value in row] for row in rows])
            self.groups[name] = (joints, lower, step, mask)

    @staticmethod
    def grid_indices(joints, lower, step, targets):
        coordinates = (np.array([targets[joint] for joint in joints]) - lower) / step
        return np.column_stack((np.floor(coordinates + 1e-8), np.ceil(coordinates - 1e-8))).astype(int)

    def validate(self, targets, *, groups=None):
        for name in self.groups if groups is None else groups:
            joints, lower, step, mask = self.groups[name]
            indices = self.grid_indices(joints, lower, step, targets)
            if (np.any(indices < 0) or np.any(indices >= np.array(mask.shape)[:, None])
                    or not mask[np.ix_(indices[0], indices[1])].all()):
                raise ValueError(f"{name} 两轴组合超出 CAD 可动区域，请先将另一轴移回零位附近")

    @classmethod
    def _group_bounds(cls, group, limits, targets, *, sweep_from=None):
        joints, lower, step, mask = group
        indices = cls.grid_indices(joints, lower, step, targets)
        swept = indices if sweep_from is None else cls.grid_indices(joints, lower, step, sweep_from)
        bounds = {}
        for axis, joint in enumerate(joints):
            rows = mask if axis == 0 else mask.T
            other_low = min(indices[1 - axis][0], swept[1 - axis][0])
            other_high = max(indices[1 - axis][1], swept[1 - axis][1])
            valid = rows[:, other_low:other_high + 1].all(axis=1)
            left, right = indices[axis]
            blocked_left = np.flatnonzero(~valid[:left])
            blocked_right = np.flatnonzero(~valid[right + 1:])
            start = int(blocked_left[-1] + 1) if len(blocked_left) else 0
            end = int(right + blocked_right[0]) if len(blocked_right) else len(valid) - 1
            bounds[joint] = (max(limits[joint][0], float(lower[axis] + start * step)),
                             min(limits[joint][1], float(lower[axis] + end * step)))
        return bounds

    def active_limits(self, limits, targets, *, groups=None):
        bounds = dict(limits)
        for name in self.groups if groups is None else groups:
            bounds.update(self._group_bounds(self.groups[name], limits, targets))
        return bounds

    def project_step(self, limits, previous, requested, max_step):
        """Keep the full frame-to-frame rectangle inside the coupled workspace."""
        result = dict(requested)
        for group in self.groups.values():
            joints = group[0]
            candidates = []
            for order in (joints, joints[::-1]):
                trial = dict(previous)
                for joint in order:
                    lower, upper = self._group_bounds(
                        group, limits, trial, sweep_from=previous)[joint]
                    lower = max(lower, previous[joint] - max_step[joint])
                    upper = min(upper, previous[joint] + max_step[joint])
                    trial[joint] = min(upper, max(lower, requested[joint]))
                candidates.append(trial)
            selected = min(candidates, key=lambda candidate: sum(
                (candidate[joint] - requested[joint])**2 for joint in joints))
            result.update({joint: selected[joint] for joint in joints})
        self.validate(result)
        return result
