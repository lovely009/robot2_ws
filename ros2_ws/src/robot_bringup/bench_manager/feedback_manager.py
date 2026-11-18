"""管理选定模型的 ros2_control/TF 会话；启动不使能电机、不恢复旧目标。"""
from pathlib import Path
import re
import time
import xml.etree.ElementTree as ET

from robot_transmission import Model

from .systemd_units import STATE_NAMES, UNIT_FEEDBACK, SystemdUnits, UnitState

MODEL_ENV = 'BENCH_FEEDBACK_MODEL'
CONTROL_ENV = 'BENCH_CONTROL_EXECUTION'
MOTORS_ENV = 'BENCH_CONTROL_MOTORS'
SESSION_ENV = 'BENCH_MCU_SESSION'
PUBLISH_ENV = 'BENCH_FEEDBACK_PUBLISH'
INVENTORY_SETTLE_S = 1.5
_MODEL = re.compile(r'[A-Za-z0-9][A-Za-z0-9_.-]*\Z')


class FeedbackInventory:
    """只按连接和测量新鲜度选设备，使能或故障变化不改变会话集合。"""
    def __init__(self):
        self.message = None
        self.received = 0.

    def observe(self, message, now_ns=None, now=None):
        now_ns = time.time_ns() if now_ns is None else now_ns
        now = time.monotonic() if now is None else now
        previous = self.message
        if (message.schema_version != 2 or not message.session_id or not message.sequence
                or not -20_000_000 <= now_ns - message.sync_time_ns <= 300_000_000
                or len(message.states) > 48):
            return False
        if previous and (message.session_id < previous.session_id or
                (message.session_id == previous.session_id and
                 not 0 < (message.sequence - previous.sequence) % 2**32 < 2**31)):
            return False
        ids = [state.motor_id for state in message.states]
        if len(set(ids)) != len(ids) or any(not 1 <= mid <= 64 for mid in ids):
            return False
        self.message, self.received = message, now
        return True

    def current(self, now_ns=None, now=None):
        now_ns = time.time_ns() if now_ns is None else now_ns
        now = time.monotonic() if now is None else now
        message = self.message
        if not message:
            return [], 0
        age = now_ns - message.sync_time_ns
        if not 0 <= now-self.received <= .3 or not -20_000_000 <= age <= 300_000_000:
            return [], message.session_id
        return sorted(state.motor_id for state in message.states
                      if state.online and state.state_age_ms + max(0, age)/1_000_000 < 300), message.session_id


class FirmwareFeedbackManager:
    def __init__(self, models_root: Path | None = None,
                 systemd: SystemdUnits | None = None, stop_control=lambda: None):
        if models_root is None:
            from .model_catalog import default_models_root
            models_root = default_models_root()
        self.models_root = Path(models_root) if models_root else None
        self._systemd = systemd or SystemdUnits()
        self._stop_control = stop_control
        self._desired = ''
        self._source = 'waiting'
        self._reason = '等待选择模型'
        self._error = ''
        self._attempt = None
        self._attempt_count = 0
        self._attempt_at = 0.
        self._healthy_since = None
        self._unhealthy_since = None
        self._health_at = 0.
        self._pending_inventory = None
        self._inventory_since = 0.

    @staticmethod
    def validate_name(model: str) -> None:
        if not isinstance(model, str) or (model and not _MODEL.fullmatch(model)):
            raise ValueError('模型必须是单层目录名')

    def _model_files(self, model: str, simulation: bool = False):
        self.validate_name(model)
        if not self.models_root:
            raise ValueError('找不到模型根目录')
        base = self.models_root / model
        driver = base / 'config' / 'motor_driver.yaml'
        urdf = base / 'urdf' / f'{model}.urdf'
        simulation_driver = base / 'config' / 'motor_driver.simulation.yaml'
        host_hil_driver = base / 'config' / 'motor_driver.hil.yaml'
        if host_hil_driver.is_file():
            simulation_driver = host_hil_driver
        if simulation and simulation_driver.is_file():
            driver = simulation_driver
            control_urdf = base / 'urdf' / f'{model}.hil.urdf'
            if model == 'full_body_robot' and not control_urdf.is_file():
                raise ValueError(f'模型 {model} 缺少 HIL 控制 URDF，请运行 build_hil_config.py')
            urdf = control_urdf if control_urdf.is_file() else base / 'urdf' / f'{model}.standalone.urdf'
            if not urdf.is_file():
                raise ValueError(f'模型 {model} 的仿真 mapping 缺少 standalone URDF：{urdf}')
        return driver, urdf

    def _validate(self, model: str) -> None:
        driver, urdf = self._model_files(model)
        return self._validate_files(model, driver, urdf)

    def _validate_files(self, model: str, driver: Path, urdf: Path) -> None:
        if not driver.is_file() or not urdf.is_file():
            raise ValueError(f'模型 {model} 缺少电机配置或 URDF')
        transmission = Model.load(str(driver))
        ids = list(transmission.actuator_ids.values())
        if (not 0 < len(ids) <= 48 or len(set(ids)) != len(ids)
                or any(not 1 <= mid <= 64 for mid in ids)):
            raise ValueError('固件映射 motor_id 必须唯一、在 1..64 内且最多 48 台')
        joints = {j.get('name'): j for j in ET.parse(urdf).getroot().findall('joint')}
        for name in transmission.joint_names:
            joint = joints.get(name)
            if joint is None or joint.get('type') == 'fixed' or joint.find('mimic') is not None:
                raise ValueError(f'传动输出 {name} 必须是 URDF 独立活动关节')
        return transmission

    def status(self) -> dict:
        unit = self._systemd.probe(UNIT_FEEDBACK)
        env = (self._systemd.environment(UNIT_FEEDBACK)
               if unit.state is not UnitState.ABSENT else {})
        return {'state': 'failed' if self._error else STATE_NAMES[unit.state],
                'model': env.get(MODEL_ENV, '') if unit.state is UnitState.RUNNING else '',
                'desired_model': self._desired, 'source': self._source,
                'execution': env.get(CONTROL_ENV, ''),
                'motor_ids': env.get(MOTORS_ENV, ''),
                'session_id': env.get(SESSION_ENV, '0'),
                'publishes_joint_states': env.get(PUBLISH_ENV, 'true') == 'true',
                'dds_profile': env.get('FASTRTPS_DEFAULT_PROFILES_FILE', ''),
                'reason': self._reason, 'error': self._error,
                'returncode': unit.returncode}

    def stop(self, *, reset_retry=True) -> None:
        self._stop_control()
        self._systemd.stop(UNIT_FEEDBACK)
        self._healthy_since = None
        self._unhealthy_since = None
        if reset_retry:
            self._attempt = None
            self._attempt_count = 0
            self._pending_inventory = None

    def observe_control_status(self, status):
        now = time.monotonic()
        if not 0 <= now-self._health_at < 2.5:
            self._healthy_since = self._unhealthy_since = None
        self._health_at = now
        ready = (isinstance(status, dict) and status.get('model') == self._desired
                 and status.get('feedback_fresh') is True
                 and status.get('controllers_ready') is True)
        if not ready:
            self._healthy_since = None
            if self._unhealthy_since is None:
                self._unhealthy_since = now
        elif self._healthy_since is None:
            self._healthy_since = now
        if ready:
            self._unhealthy_since = None

    def reconcile(self, model: str, agent_running: bool = False,
                  simulation_running: bool = False,
                  firmware_simulation: bool = False,
                  external_publishers: list[str] | None = None,
                  motor_ids: list[int] | None = None, session_id: int = 0) -> None:
        self._desired = model
        self._error = ''
        hil = simulation_running and firmware_simulation
        publish_states = not hil
        source, reason = 'firmware', 'MCU 反馈与 ros2_control 控制会话'
        if simulation_running and not hil:
            source, reason = 'simulation', '由仿真会话提供关节反馈'
        elif external_publishers:
            source, reason = 'external', '由外部关节源提供反馈'
        elif not model:
            source, reason = 'waiting', '等待选择模型'
        elif not agent_running:
            source, reason = 'waiting', '等待 Agent'
        self._source = 'simulation' if hil else source
        self._reason = f'仿真关节反馈；{reason}' if hil else reason
        if source != 'firmware':
            if self.status()['state'] != 'stopped':
                self.stop()
            self._attempt = None
            self._attempt_count = 0
            self._pending_inventory = None
            return
        status = self.status()
        try:
            driver, urdf = self._model_files(model, simulation=hil)
            # 保留既有 _validate(model) 扩展点；仅实际仿真 mapping 需要新路径。
            transmission = (self._validate_files(model, driver, urdf)
                            if driver.name in ('motor_driver.simulation.yaml', 'motor_driver.hil.yaml')
                            else self._validate(model))
            available = set(transmission.actuator_ids.values() if motor_ids is None else motor_ids)
            selected = sorted({transmission.actuator_ids[name]
                for group in transmission.groups
                if driver.name == 'motor_driver.hil.yaml' or all(
                    transmission.actuator_ids[name] in available for name in group.actuators)
                for name in group.actuators if transmission.actuator_ids[name] in available})
            ids = ','.join(map(str, selected)) if motor_ids is not None else ''
            session = str(session_id)
            profile = self.models_root / model / 'config' / 'fastdds_udp.xml'
            dds_profile = str(profile) if hil and profile.is_file() else ''
            key = (model, ids, session, publish_states, dds_profile)
            now = time.monotonic()
            same_context = (
                status['model'] == model and status['execution'] == 'ros2_control'
                and (status['session_id'] == session or (session_id == 0 and motor_ids == []))
                and status['publishes_joint_states'] == publish_states
                and status['dds_profile'] == dds_profile)
            matching = (status['state'] == 'running' and same_context
                        and status['motor_ids'] == ids and bool(selected))
            # MCU 重启或控制来源切换立即释放旧控制权；在线集合抖动只延迟重建。
            if status['state'] == 'running' and not same_context:
                self.stop(reset_retry=False)
                status = self.status()
            # 管理器刚重启时 session=0 表示尚未收到首帧，不能据此撤销已运行的会话。
            if status['state'] == 'running' and session_id == 0 and motor_ids == []:
                self._pending_inventory = None
                self._reason = '等待首次电机反馈以确认现有会话'
                return
            if matching or motor_ids is None:
                self._pending_inventory = None
            else:
                if self._pending_inventory != key:
                    self._pending_inventory = key
                    self._inventory_since = now
                if now - self._inventory_since < INVENTORY_SETTLE_S:
                    if status['state'] != 'running':
                        self._source = 'simulation' if hil else 'waiting'
                    self._reason = '等待电机在线列表稳定'
                    return
            if not selected:
                if status['state'] != 'stopped':
                    self.stop(reset_retry=False)
                elif self._attempt is not None:
                    self._stop_control()
                self._attempt = None
                self._attempt_count = 0
                self._source = 'simulation' if hil else 'waiting'
                self._reason = ('等待新鲜电机反馈' if not motor_ids
                                else '等待完整传动组的电机反馈')
                return
            if self._attempt != key:
                self._attempt_count = 0
            if matching:
                if self._attempt != key:
                    self._attempt, self._attempt_at = key, now
                    return
                health_fresh = 0 <= now-self._health_at < 2.5
                if self._healthy_since is not None and health_fresh:
                    if now-self._healthy_since >= 5.:
                        self._attempt_count = 0
                    return
                # 进程存活不代表接口正常；给发现/控制器加载留时间，再重建失效会话。
                if now-self._attempt_at < 8. or (health_fresh and
                        (self._unhealthy_since is None or now-self._unhealthy_since < 3.)):
                    return
            if self._attempt == key:
                if self._attempt_count >= 3:
                    self._error = f'模型 {model} 的关节反馈连续启动失败，已停止自动重试；请检查反馈日志'
                    return
                if now-self._attempt_at < 3.:
                    self._error = f'模型 {model} 的关节反馈中断，正在等待重试'
                    return
            if status['state'] != 'stopped':
                self.stop(reset_retry=False)
            else:
                self._stop_control()
            self._attempt = key
            self._attempt_count += 1
            self._attempt_at = now
            self._healthy_since = None
            self._unhealthy_since = None
            self._systemd.start(UNIT_FEEDBACK, [
                'ros2', 'launch', 'motor_hardware_interface', 'host_trajectory.launch.py',
                f'model:={model}', f'motor_ids:={ids}',
                f'motor_config_file:={driver}', f'urdf_file:={urdf}',
                'start_controllers:=false',
                f'publish_joint_states:={str(publish_states).lower()}',
                f'publish_robot_state:={str(publish_states).lower()}',
            ], auto=True, session_env={MODEL_ENV: model, CONTROL_ENV: 'ros2_control',
                                      MOTORS_ENV: ids, SESSION_ENV: session,
                                      PUBLISH_ENV: str(publish_states).lower(),
                                      **({'FASTRTPS_DEFAULT_PROFILES_FILE': dds_profile}
                                         if dds_profile else {})})
        except (ValueError, OSError, RuntimeError, ET.ParseError) as exc:
            self.stop(reset_retry=False)
            self._error = str(exc)
