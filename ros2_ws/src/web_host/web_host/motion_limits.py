"""CAD workspace constraints for offline BVH conversion of a registered model."""

import hashlib
from pathlib import Path

import numpy as np
import yaml

from robot_transmission.coupled_limits import CoupledLimits


class MotionPreviewLimits:
    @classmethod
    def for_model(cls, request, models_root, robot):
        name = request.get('model')
        if name is None:
            return None
        if not isinstance(name, str) or not name or name != Path(name).name or name in ('.', '..') or '\\' in name:
            raise ValueError('模型名称无效')
        if models_root is None:
            from .models import _models_root
            models_root = _models_root()
        if models_root is None:
            return None
        directory = Path(models_root) / name
        path = directory / 'config/motion_preview_limits.yaml'
        if not path.is_file():
            return None
        if (directory / 'urdf' / f'{name}.urdf').read_text() != request['urdf_text'].replace('\r\n', '\n'):
            raise ValueError('模型已更新或与模型库不一致，请重新导入')
        return cls(path.read_bytes(), robot)

    def __init__(self, content, robot):
        config = yaml.safe_load(content)
        self.digest = hashlib.sha256(content).hexdigest()
        self.velocity = float(config['velocity_limit_rad_s'])
        if not np.isfinite(self.velocity) or self.velocity <= 0:
            raise ValueError('动作预览速度限位无效')
        if (any(j.get('type') != 'revolute' or j.find('mimic') is not None for j in robot.spec.values())
                or not np.isfinite([robot.lower, robot.upper]).all()
                or np.any(robot.lower > 0) or np.any(robot.upper < 0)):
            raise ValueError('CAD 动作预览要求全部独立转轴具有包含零位的位置限位')
        self.bounds = dict(zip(robot.names, zip(robot.lower, robot.upper)))
        self.workspace = CoupledLimits(config['coupled_limits'], robot.names)
        self.workspace.validate(dict.fromkeys(robot.names, 0.))

    def apply_to(self, solver):
        # Keep any tighter URDF speed. The CAD cap applies only to the offline solver.
        solver.velocity = np.minimum(solver.velocity, self.velocity)
        solver.robot.velocity = np.minimum(solver.robot.velocity, self.velocity)
        solver.report.update(cad_limits_enforced=True, velocity_limit_rad_s=self.velocity,
                             coupled_clamped_frames=0)
        names = solver.robot.independent

        def project(previous, requested, max_step):
            result = self.workspace.project_step(self.bounds, dict(zip(names, previous)),
                                                 dict(zip(names, requested)), dict(zip(names, max_step)))
            projected = np.array([result[name] for name in names])
            if np.any(np.abs(projected - requested) > 1e-9):
                solver.report['coupled_clamped_frames'] += 1
            return projected

        solver.position_filter = project

    def validate(self, names, positions):
        for row in positions:
            self.workspace.validate(dict(zip(names, row)))
