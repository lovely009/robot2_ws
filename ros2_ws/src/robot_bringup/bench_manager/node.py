"""bench_manager 节点：台架会话在 ROS 侧的**唯一编排者**。

会话进程（仿真 / micro-ROS Agent / 电机后端）由 systemd user
transient unit 承载，本节点不持有任何进程句柄——状态权威是 unit 状态，会话描述存在 unit
环境里，本节点重启后可认领已有会话。重力前馈直接适配 ros2_control 参数，
状态由 /control/status 的新鲜回读恢复。

- 服务全在 `/bench/` 绝对命名空间下（web 前端按固定名调用，不做 remap）：
  `/bench/options`、`/bench/agent_status`、`/bench/{start,stop}_simulation`、
  `/bench/{start,stop}_agent`、`/bench/{start,stop}_motor_backend`、
  `/bench/motor_backend_command`、`/bench/get_unit_log`、
  `/bench/{start,stop}_gravity_feedforward`、`/bench/set_gravity_feedforward_gain`；
- 状态 `/bench/status`（std_msgs/String，JSON）1 Hz 周期发布 + 每条命令后立即发布，
  无订阅者时不构造；命令响应里也带同一份快照（status_json），失败时供页面回显；
- 日志不进周期快照，按需调用 `/bench/get_unit_log`。

lifecycle 语义（`robot_bringup` 的 `robot.launch.py` 负责 configure → activate）：

- `on_configure` 只做校验；systemd user manager 不可达时立即失败；
- `on_activate` 认领已在跑的 unit，并按参数启动尚未运行的 Agent；
- `on_deactivate` 停掉整个 `bench.slice`——这是「安全停止」的唯一显式语义；
- `on_cleanup` 只释放句柄。

干净退出（Ctrl-C、SIGTERM）停掉 slice，避免页面失去停止入口；
异常死亡（崩溃、kill -9）保留 unit，供下一个 `bench_manager` 认领。
"""

from functools import partial
import json
import os

import rclpy
from rcl_interfaces.srv import SetParametersAtomically
from rcl_interfaces.msg import SetParametersResult
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.parameter import Parameter
from std_msgs.msg import String, Empty
from can_bridge_msgs.msg import MotorControlStateArray

from bench_msgs.srv import (
    GetAgentStatus,
    GetBenchOptions,
    GetUnitLog,
    MotorBackendCommand,
    SetGravityFeedforwardGain,
    StartAgent,
    StartGravityFeedforward,
    StartMotorBackend,
    StartSimulation,
    StopAgent,
    StopGravityFeedforward,
    StopMotorBackend,
    StopSimulation,
)

from .agent_manager import AgentProcessManager, enumerate_serial_ports
from .feedback_manager import FirmwareFeedbackManager, FeedbackInventory
from .gravity_feedforward_manager import GravityFeedforwardManager
from .motor_sim_manager import MotorSimProcessManager, hil_motor_backend_request
from .model_catalog import enumerate_mujoco_models, enumerate_simulation_models
from .simulation_manager import SimulationProcessManager
from .systemd_units import (
    UNIT_AGENT, UNIT_MOTOR_BACKEND, UNIT_SIMULATION, UNIT_FEEDBACK,
    SystemdUnits,
)

_STATUS_PERIOD_S = 1.0
# 管理器契约内的可预期失败；其余异常照常冒泡（rclpy 记日志，那是 bug）
_EXPECTED_ERRORS = (ValueError, PermissionError, RuntimeError, OSError)

# 日志服务的白名单：`unit_name` 直接进 `journalctl -u`，只认契约 unit 名，
# 免得页面（或任何服务调用方）拿它去翻别的 unit。
_UNITS = (UNIT_SIMULATION, UNIT_MOTOR_BACKEND, UNIT_AGENT, UNIT_FEEDBACK)
_LOG_LINES_DEFAULT = 80
_LOG_LINES_MAX = 2000

_GRAVITY_SERVICE = '/host_control/gravity_controller/set_parameters_atomically'
_OWN_FEEDBACK_NODES = {'firmware_topic_bridge', 'workbench_control'}


def _simulation_request(message):
    """把服务的扁平字段还原成 SimulationProcessManager 认识的请求 dict。

    空串表示未给参数，不写入命令行。
    """
    request = {
        'backend': message.backend,
        'transport': message.transport,
        'controller_mode': message.controller_mode,
        'model': message.model,
        # 物理总线隔离确认必须传给 simulation_manager 校验。
        'physical_bus_isolated': bool(message.physical_bus_isolated),
    }
    for key in ('vcan_interface', 'sim_interface', 'driver_interface',
                'sim_tty', 'driver_tty'):
        value = getattr(message, key)
        if value:
            request[key] = value
    if message.spawn_z:
        try:
            request['spawn_z'] = float(message.spawn_z)
        except ValueError:
            raise ValueError('spawn_z must be a number (metres)') from None
    for key in ('gravity', 'fixed_base'):
        value = getattr(message, key)
        if value:
            if value not in ('true', 'false'):
                raise ValueError(f'{key} must be "true" or "false"')
            request[key] = value == 'true'
    return request


class BenchManager(LifecycleNode):

    def __init__(self):
        super().__init__('bench_manager')
        self._change_state_callback = self._service_change_state.callback
        self._service_change_state.callback = self._on_change_state
        self.declare_parameter('model_dir', '')
        self.declare_parameter('feedback_model', '')
        self.declare_parameter('auto_start_agent', False)
        self.declare_parameter('agent_device', '')
        model_dir = self.get_parameter('model_dir').value
        models_dir = ''
        if model_dir and os.path.isdir(os.path.join(model_dir, 'models')):
            models_dir = os.path.join(model_dir, 'models')
        elif model_dir:
            self.get_logger().warn(
                'model_dir 下没有 models/，模型枚举回落到 robot_description 安装树：%s'
                % model_dir)
        models_root = models_dir or None
        model_provider = partial(enumerate_simulation_models, models_root)

        # 所有会话共用一个可注入的 systemd 访问层。
        self._systemd = SystemdUnits()
        self.feedback = FirmwareFeedbackManager(models_root, self._systemd,
            stop_control=lambda: self._feedback_stop_pub.publish(Empty()))
        self._feedback_inventory = FeedbackInventory()
        self._feedback_inventory_sub = None
        self._feedback_stop_pub = None
        self._feedback_error = ''
        self.add_on_set_parameters_callback(self._validate_feedback_parameters)
        self.agent = AgentProcessManager(systemd=self._systemd)
        self.simulation = SimulationProcessManager(
            model_provider=model_provider,
            mujoco_model_provider=partial(enumerate_mujoco_models, models_root),
            systemd=self._systemd)
        self.motor_backend = MotorSimProcessManager(
            model_provider=model_provider, systemd=self._systemd)
        self.feedforward = GravityFeedforwardManager(
            models_root, parameter_writer=self._write_gravity_parameters)

        self._gravity_client = None
        self._control_status_sub = None
        self._status_pub = None
        self._timer = None

    # ---------- lifecycle ----------

    def _on_change_state(self, request, response):
        # Humble 会让非法 transition ID 的异常逃出服务回调，导致整个管理器退出。
        if not request.transition.label and request.transition.id not in (
                transition[0] for transition in self._state_machine.available_transitions):
            response.success = False
            return response
        return self._change_state_callback(request, response)

    def on_configure(self, _state):
        if not self._systemd.available():
            self.get_logger().error(
                '拿不到 systemd user manager（session bus 不通？）——台架会话没有承载者，'
                '拒绝进入配置态。检查 `systemctl --user show-environment` 是否可用。')
            return TransitionCallbackReturn.FAILURE

        self._status_pub = self.create_publisher(String, '/bench/status', 10)
        self._feedback_stop_pub = self.create_publisher(Empty, '/control/stop', 10)
        self._feedback_inventory_sub = self.create_subscription(
            MotorControlStateArray, '/motor/control_states', self._feedback_inventory.observe, 10)
        self._control_status_sub = self.create_subscription(
            String, '/control/status', self._on_control_status, 10)
        for srv_type, name, callback in (
            (GetBenchOptions, '/bench/options', self._on_options),
            (GetAgentStatus, '/bench/agent_status', self._on_agent_status),
            (StartSimulation, '/bench/start_simulation', self._on_start_simulation),
            (StopSimulation, '/bench/stop_simulation', self._on_stop_simulation),
            (StartAgent, '/bench/start_agent', self._on_start_agent),
            (StopAgent, '/bench/stop_agent', self._on_stop_agent),
            (StartMotorBackend, '/bench/start_motor_backend',
             self._on_start_motor_backend),
            (StopMotorBackend, '/bench/stop_motor_backend',
             self._on_stop_motor_backend),
            (MotorBackendCommand, '/bench/motor_backend_command',
             self._on_motor_backend_command),
            (GetUnitLog, '/bench/get_unit_log', self._on_get_unit_log),
            (StartGravityFeedforward, '/bench/start_gravity_feedforward',
             self._on_start_gravity_feedforward),
            (StopGravityFeedforward, '/bench/stop_gravity_feedforward',
             self._on_stop_gravity_feedforward),
            (SetGravityFeedforwardGain, '/bench/set_gravity_feedforward_gain',
             self._on_set_gravity_feedforward_gain),
        ):
            self.create_service(srv_type, name, callback)
        self._timer = self.create_timer(_STATUS_PERIOD_S, self._tick)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        adopted_model = self.feedback.status()['model']
        if adopted_model and not self.get_parameter('feedback_model').value:
            self.set_parameters([Parameter('feedback_model', value=adopted_model)])
        # 认领：unit 里还活着的会话归本节点编排（只报告，不动它们——重启一个正在跑的
        # 仿真只会把它打断，而页面要的正是「看得见」）。
        for label, manager in self._sessions():
            status = manager.status()
            if status['state'] != 'stopped':
                self.get_logger().info(
                    '认领已有会话 %s：%s %s'
                    % (label, status['state'], status.get('config') or ''))
        self._start_default_agent()
        self._publish_status()
        return super().on_activate(state)

    def _start_default_agent(self):
        if not self.get_parameter('auto_start_agent').value:
            return
        try:
            if self.agent.status()['state'] == 'running':
                return
            device = self.get_parameter('agent_device').value.strip()
            if not device:
                ports = enumerate_serial_ports()
                if len(ports) != 1:
                    raise ValueError(
                        '检测到 %d 个串口，请在页面选择串口启动，或指定 agent_tty'
                        % len(ports))
                device = ports[0]['path']
            self.agent.start(device, auto=True)
            self.get_logger().info('自动启动 Agent：%s' % device)
        except _EXPECTED_ERRORS as exc:
            self.get_logger().warn('自动启动 Agent 失败：%s' % exc)

    def on_deactivate(self, state):
        self.get_logger().info('停掉 bench.slice 下的全部会话')
        self.stop_sessions()
        return super().on_deactivate(state)

    def stop_sessions(self):
        """停掉 `bench.slice` 下的全部会话。

        `deactivate` 与进程干净退出共用这一条：本节点是页面唯一的会话出口，它一没，
        页面就再也停不掉还在跑的 Gazebo 和占着串口的 Agent。异常死亡（崩溃、kill -9）
        走不到这里，那些 unit 由下一个 `bench_manager` 在 `on_activate` 里认领。
        """
        self._systemd.stop_slice()

    def on_cleanup(self, _state):
        # 只释放句柄：会话不回收（见模块 docstring）。
        self.destroy_timer(self._timer)
        self.destroy_publisher(self._status_pub)
        self.feedforward.close()
        if self._feedback_inventory_sub is not None:
            self.destroy_subscription(self._feedback_inventory_sub)
            self._feedback_inventory_sub = None
        if self._feedback_stop_pub is not None:
            self.destroy_publisher(self._feedback_stop_pub)
            self._feedback_stop_pub = None
        if self._gravity_client is not None:
            self.destroy_client(self._gravity_client)
            self._gravity_client = None
        if self._control_status_sub is not None:
            self.destroy_subscription(self._control_status_sub)
            self._control_status_sub = None
        self._timer = None
        self._status_pub = None
        return TransitionCallbackReturn.SUCCESS

    def _sessions(self):
        return (('simulation', self.simulation),
                ('motor backend', self.motor_backend),
                ('micro-ROS Agent', self.agent),
                ('gravity feedforward', self.feedforward),
                ('joint feedback', self.feedback))

    def _validate_feedback_parameters(self, parameters):
        try:
            for parameter in parameters:
                if parameter.name == 'feedback_model':
                    if parameter.type_ is not Parameter.Type.STRING:
                        raise ValueError('feedback_model 必须是字符串')
                    self.feedback.validate_name(parameter.value)
        except ValueError as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    def _tick(self):
        self.feedforward.tick()
        try:
            simulation = self.simulation.status()
            simulation_running = simulation['state'] == 'running'
            config = simulation.get('config') or {}
            hil = simulation_running and config.get('controller_mode') == 'firmware'
            feedback = self.feedback.status()
            publishes_feedback = feedback['state'] == 'running' and feedback['publishes_joint_states']
            publishers = self.get_publishers_info_by_topic('/joint_states')
            expected = {'mujoco': 'mujoco_effort_bridge',
                        'gazebo': 'joint_state_broadcaster'}.get(config.get('backend'))
            sim_publishers = [p for p in publishers if hil and expected
                              and p.node_name == expected and p.node_namespace == '/']
            if len(sim_publishers) == 1:
                publishers = [p for p in publishers if p is not sim_publishers[0]]
            own_count = sum(p.node_name in _OWN_FEEDBACK_NODES for p in publishers)
            external = [p.node_name for p in publishers
                        if p.node_name not in _OWN_FEEDBACK_NODES or not publishes_feedback or own_count > 1]
            motor_ids, session_id = self._feedback_inventory.current()
            self.feedback.reconcile(
                config.get('model', '') if hil else self.get_parameter('feedback_model').value,
                agent_running=self.agent.status()['state'] == 'running',
                simulation_running=simulation_running, firmware_simulation=hil,
                external_publishers=external, motor_ids=motor_ids, session_id=session_id)
            self._feedback_error = ''
        except _EXPECTED_ERRORS as exc:
            self._feedback_error = str(exc)
            self.get_logger().error('关节反馈自动管理失败：%s' % exc)
        self._publish_status()

    # ---------- 状态快照与响应收尾 ----------

    def _snapshot_json(self):
        feedback = self.feedback.status()
        if self._feedback_error:
            feedback.update(error=self._feedback_error, state='failed')
        return json.dumps({
            'simulation': self.simulation.status(),
            'agent': self.agent.status(),
            'motor_backend': self.motor_backend.status(),
            'gravity_feedforward': self.feedforward.status(),
            'feedback': feedback,
        }, ensure_ascii=False)

    def _publish_status(self):
        if self._status_pub is None:
            return
        if self._status_pub.get_subscription_count() == 0:
            return
        self._status_pub.publish(String(data=self._snapshot_json()))

    def _finish(self, response, error=None):
        response.ok = error is None
        response.error = '' if error is None else str(error)
        response.status_json = self._snapshot_json()
        self._publish_status()
        return response

    # ---------- 服务回调 ----------

    def _on_options(self, _request, response):
        options = self.simulation.options()
        response.can_interfaces_json = json.dumps(
            options['can_interfaces'], ensure_ascii=False)
        response.serial_ports_json = json.dumps(
            options['serial_ports'], ensure_ascii=False)
        response.simulation_models = options['models']
        response.mujoco_models = options['mujoco_models']
        return response

    def _on_agent_status(self, _request, response):
        response.ok = True
        response.status_json = json.dumps(
            self.agent.status(), ensure_ascii=False)
        return response

    def _on_start_simulation(self, request, response):
        try:
            sim_request = _simulation_request(request)
            # HIL + 物理总线：先把电机后端拉起来（配置冲突/CANable 占用在这里
            # 就地拒绝，不对仿真产生副作用）；仿真启动失败则回滚。
            bus_request = hil_motor_backend_request(sim_request)
            backend_started = False
            if bus_request is not None:
                was_running = (
                    self.motor_backend.status().get('state') == 'running')
                self.motor_backend.start(bus_request, auto=True)
                backend_started = (
                    not was_running
                    and self.motor_backend.status().get('state') == 'running')
            try:
                self.feedback.stop()
                self.simulation.start(sim_request)
            except Exception:
                if backend_started:
                    try:
                        self.motor_backend.stop()
                    except _EXPECTED_ERRORS as exc:
                        self.get_logger().error(
                            'Could not roll back motor backend: %s' % exc)
                raise
        except _EXPECTED_ERRORS as exc:
            return self._finish(response, exc)
        return self._finish(response)

    def _on_stop_simulation(self, _request, response):
        try:
            if self.simulation.status().get('config', {}).get('controller_mode') == 'firmware':
                self.feedback.stop()
            self.simulation.stop()
        except _EXPECTED_ERRORS as exc:
            return self._finish(response, exc)
        # 随仿真自动拉起的电机后端联动停；手动起的保持运行（归属存在 unit 环境里，
        # 所以这个判断在本节点重启后依然成立）
        if self.motor_backend.status().get('auto_started'):
            try:
                self.motor_backend.stop()
            except _EXPECTED_ERRORS as exc:
                self.get_logger().error(
                    'Could not stop motor backend: %s' % exc)
        return self._finish(response)

    def _on_start_agent(self, request, response):
        try:
            device = (request.device or '').strip()
            if not device:
                raise ValueError('device is required')
            self.agent.start(device)
        except _EXPECTED_ERRORS as exc:
            return self._finish(response, exc)
        return self._finish(response)

    def _on_stop_agent(self, _request, response):
        try:
            self.agent.stop()
            self.feedback.stop()
        except _EXPECTED_ERRORS as exc:
            return self._finish(response, exc)
        return self._finish(response)

    def _on_start_motor_backend(self, request, response):
        try:
            motor_request = {
                'model': request.model,
                'transport': request.transport,
                'physical_bus_isolated': bool(request.physical_bus_isolated),
            }
            if request.transport == 'socketcan':
                motor_request['sim_interface'] = request.sim_interface
            else:
                motor_request['sim_tty'] = request.sim_tty
            self.motor_backend.start(motor_request, auto=False)
        except _EXPECTED_ERRORS as exc:
            return self._finish(response, exc)
        return self._finish(response)

    def _on_stop_motor_backend(self, _request, response):
        try:
            self.motor_backend.stop()
        except _EXPECTED_ERRORS as exc:
            return self._finish(response, exc)
        return self._finish(response)

    def _on_motor_backend_command(self, request, response):
        try:
            try:
                payload = json.loads(request.request_json)
            except json.JSONDecodeError as exc:
                raise ValueError('invalid JSON request: %s' % exc) from None
            if not isinstance(payload, dict):
                raise ValueError('request must be a JSON object')
            result = self.motor_backend.motor_request(payload)
        except _EXPECTED_ERRORS as exc:
            response.ok = False
            response.error = str(exc)
            return response
        response.ok = True
        response.error = ''
        response.response_json = json.dumps(result, ensure_ascii=False)
        return response

    def _on_get_unit_log(self, request, response):
        try:
            unit = (request.unit_name or '').strip()
            if unit not in _UNITS:
                raise ValueError(
                    'unknown unit %r; expected one of %s'
                    % (unit, ', '.join(_UNITS)))
            lines = request.lines if request.lines > 0 else _LOG_LINES_DEFAULT
            response.text = self._systemd.log(unit, min(lines, _LOG_LINES_MAX))
        except _EXPECTED_ERRORS as exc:
            response.ok = False
            response.error = str(exc)
            response.text = ''
            return response
        response.ok = True
        response.error = ''
        return response

    # ---------- ros2_control 重力前馈 ----------

    def _on_control_status(self, message):
        try:
            status = json.loads(message.data)
            self.feedforward.observe(status)
            self.feedback.observe_control_status(status)
        except (ValueError, TypeError) as exc:
            self.feedforward.invalidate_status(str(exc))
            self.feedback.observe_control_status(None)
        self._publish_status()

    def _finish_gravity(self, response, action, callback):
        try:
            callback()
        except _EXPECTED_ERRORS as exc:
            self.feedforward.note_failed(str(exc), action)
            return self._finish(response, exc)
        status = self.feedforward.status()
        # ok 只表示受理；pending 明确区分排队、回执成功和真实回读确认。
        return self._finish(response, status['error'] or None)

    def _on_start_gravity_feedforward(self, request, response):
        return self._finish_gravity(response, 'start', lambda: self.feedforward.start({
            'model': request.model,
            'joint_map': request.joint_map,
            'effort_gain': request.effort_gain,
        }))

    def _on_stop_gravity_feedforward(self, _request, response):
        return self._finish_gravity(response, 'stop', self.feedforward.stop)

    def _on_set_gravity_feedforward_gain(self, request, response):
        return self._finish_gravity(response, 'gain',
                                    lambda: self.feedforward.set_gain(request.effort_gain))

    def _write_gravity_parameters(self, values):
        if self._gravity_client is None:
            self._gravity_client = self.create_client(SetParametersAtomically, _GRAVITY_SERVICE)
        if not self._gravity_client.service_is_ready():
            raise RuntimeError(f'{_GRAVITY_SERVICE} is unavailable')
        request = SetParametersAtomically.Request()
        request.parameters = [Parameter(name, value=value).to_parameter_msg()
                              for name, value in values.items()]
        return self._gravity_client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = BenchManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 干净退出＝会话结束：停掉整个 slice。异常死亡走不到这里，那些 unit 留给下一个
        # bench_manager 认领（见 stop_sessions 的 docstring）。
        try:
            node.stop_sessions()
        except Exception as exc:
            node.get_logger().error('停会话失败：%s' % exc)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
