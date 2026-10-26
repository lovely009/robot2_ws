"""只读固件关节反馈的模型归属与进程管理；不修改命令通道。"""
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from robot_transmission import Model

from .systemd_units import STATE_NAMES, UNIT_FEEDBACK, SystemdUnits, UnitState

MODEL_ENV = 'BENCH_FEEDBACK_MODEL'
_MODEL = re.compile(r'[A-Za-z0-9][A-Za-z0-9_.-]*\Z')


class FirmwareFeedbackManager:
    def __init__(self, models_root: Path | None = None,
                 systemd: SystemdUnits | None = None):
        if models_root is None:
            from .simulation_manager import _models_root
            models_root = _models_root()
        self.models_root = Path(models_root) if models_root else None
        self._systemd = systemd or SystemdUnits()
        self._desired = ''
        self._source = 'waiting'
        self._reason = '等待选择模型'
        self._error = ''
        self._attempt = None

    @staticmethod
    def validate_name(model: str) -> None:
        if not isinstance(model, str) or (model and not _MODEL.fullmatch(model)):
            raise ValueError('模型必须是单层目录名')

    def _validate(self, model: str) -> None:
        self.validate_name(model)
        if not self.models_root:
            raise ValueError('找不到模型根目录')
        base = self.models_root / model
        driver = base / 'config' / 'motor_driver.yaml'
        urdf = base / 'urdf' / f'{model}.urdf'
        if not driver.is_file() or not urdf.is_file():
            raise ValueError(f'模型 {model} 缺少电机配置或 URDF')
        transmission = Model.load(str(driver))
        ids = list(transmission.actuator_ids.values())
        if len(set(ids)) != len(ids):
            raise ValueError('固件映射 motor_id 不能重复')
        joints = {j.get('name'): j for j in ET.parse(urdf).getroot().findall('joint')}
        for name in transmission.joint_names:
            joint = joints.get(name)
            if joint is None or joint.get('type') == 'fixed' or joint.find('mimic') is not None:
                raise ValueError(f'传动输出 {name} 必须是 URDF 独立活动关节')

    def status(self) -> dict:
        unit = self._systemd.probe(UNIT_FEEDBACK)
        env = (self._systemd.environment(UNIT_FEEDBACK)
               if unit.state is not UnitState.ABSENT else {})
        return {'state': 'failed' if self._error else STATE_NAMES[unit.state],
                'model': env.get(MODEL_ENV, '') if unit.state is UnitState.RUNNING else '',
                'desired_model': self._desired, 'source': self._source,
                'reason': self._reason, 'error': self._error,
                'returncode': unit.returncode}

    def stop(self) -> None:
        self._systemd.stop(UNIT_FEEDBACK)
        self._attempt = None

    def reconcile(self, model: str, agent_running: bool = False,
                  simulation_running: bool = False,
                  external_publishers: list[str] | None = None) -> None:
        self._desired = model
        previous_error = self._error
        self._error = ''
        source, reason = 'firmware', '固件关节反馈'
        if simulation_running:
            source, reason = 'simulation', '由仿真会话提供关节反馈'
        elif external_publishers:
            source, reason = 'external', '由外部关节源提供反馈'
        elif not model:
            source, reason = 'waiting', '等待选择模型'
        elif not agent_running:
            source, reason = 'waiting', '等待 Agent'
        self._source, self._reason = source, reason
        if source != 'firmware':
            if self.status()['state'] != 'stopped':
                self.stop()
            self._attempt = None
            return
        status = self.status()
        if status['state'] == 'running' and status['model'] == model:
            return
        # 启动或运行失败后保留诊断；模型/会话归属变化后才重新尝试。
        if self._attempt == model:
            if status['state'] != 'running':
                self._error = previous_error or f'模型 {model} 的关节反馈启动失败；请检查反馈日志'
            return
        self._attempt = model
        try:
            self._validate(model)
            if status['state'] != 'stopped':
                self._systemd.stop(UNIT_FEEDBACK)
            self._systemd.start(UNIT_FEEDBACK, [
                'ros2', 'launch', 'robot_bringup', 'firmware_tf.launch.py',
                f'model:={model}', f'model_dir:={self.models_root.parent}',
                'enable_trajectory:=false',
            ], auto=True, session_env={MODEL_ENV: model})
        except (ValueError, OSError, RuntimeError, ET.ParseError) as exc:
            self._systemd.stop(UNIT_FEEDBACK)
            self._error = str(exc)
