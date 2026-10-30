"""bench_manager 节点：台架会话在 ROS 侧的**唯一编排者**。

会话进程（仿真 / micro-ROS Agent / 电机后端 / 重力前馈叠加节点）由 systemd user
transient unit 承载，本节点不持有任何进程句柄——状态权威是 unit 状态，会话描述存在 unit
环境里，本节点重启后可认领已有会话。

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
- `on_activate` 认领已在跑的 unit（只报告，不动它们）；
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
from std_msgs.msg import String

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
from .feedback_manager import FirmwareFeedbackManager
from .gravity_feedforward_manager import (
    DIRECT_COMMAND_TOPIC, OVERLAY_COMMAND_TOPIC,
    GravityFeedforwardProcessManager,
)
from .motor_sim_manager import MotorSimProcessManager, hil_motor_backend_request
from .simulation_manager import (
    SimulationProcessManager,
    enumerate_can_interfaces,
    enumerate_mujoco_models,
    enumerate_simulation_models,
)
from .systemd_units import (
    UNIT_AGENT, UNIT_GRAVITY_FEEDFORWARD, UNIT_MOTOR_BACKEND, UNIT_SIMULATION, UNIT_FEEDBACK,
    SystemdUnits,
)

_STATUS_PERIOD_S = 1.0
# 管理器契约内的可预期失败；其余异常照常冒泡（rclpy 记日志，那是 bug）
_EXPECTED_ERRORS = (ValueError, PermissionError, RuntimeError, OSError)

# 日志服务的白名单：`unit_name` 直接进 `journalctl -u`，只认契约 unit 名，
# 免得页面（或任何服务调用方）拿它去翻别的 unit。
_UNITS = (UNIT_SIMULATION, UNIT_MOTOR_BACKEND, UNIT_AGENT,
          UNIT_GRAVITY_FEEDFORWARD, UNIT_FEEDBACK)
_LOG_LINES_DEFAULT = 80
_LOG_LINES_MAX = 2000

# 叠加节点的参数服务。Humble 的 rclpy 没有 parameters client 封装（那是 Iron+ 的
# rclpy.parameter_client），所以直接调节点的 set_parameters_atomically。
_GAIN_NODE = 'gravity_feedforward_overlay'
_GAIN_SERVICE = f'/{_GAIN_NODE}/set_parameters_atomically'
_GAIN_DISCOVERY_TIMEOUT_S = 0.5

# 命令节拍持有者的参数服务（ADR 0014）。命令流改道 = 改它的 output_topic：页面只下
# 设定值，命令从哪条路出去由这里决定。同 _GAIN_SERVICE，Humble 没有 parameters client。
_ROUTE_NODE = 'command_streamer'
_ROUTE_SERVICE = f'/{_ROUTE_NODE}/set_parameters_atomically'


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
        self.declare_parameter('model_dir', '')
        self.declare_parameter('feedback_model', '')
        model_dir = self.get_parameter('model_dir').value
        models_dir = ''
        if model_dir and os.path.isdir(os.path.join(model_dir, 'models')):
            models_dir = os.path.join(model_dir, 'models')
        elif model_dir:
            self.get_logger().warn(
                'model_dir 下没有 models/，模型枚举回落到 robot_description 安装树：%s'
                % model_dir)
        models_root = models_dir or None
        self._model_provider = partial(enumerate_simulation_models, models_root)
        self._mujoco_model_provider = partial(enumerate_mujoco_models, models_root)

        # 所有会话共用一个可注入的 systemd 访问层。
        self._systemd = SystemdUnits()
        self.feedback = FirmwareFeedbackManager(models_root, self._systemd)
        self._feedback_error = ''
        self.add_on_set_parameters_callback(self._validate_feedback_parameters)
        self.agent = AgentProcessManager(systemd=self._systemd)
        self.simulation = SimulationProcessManager(
            model_provider=self._model_provider,
            mujoco_model_provider=self._mujoco_model_provider,
            systemd=self._systemd)
        self.motor_backend = MotorSimProcessManager(
            model_provider=self._model_provider, systemd=self._systemd)
        self.feedforward = GravityFeedforwardProcessManager(
            systemd=self._systemd, gain_writer=self._write_effort_gain)

        self._gain_client = None
        self._route_client = None
        self._status_pub = None
        self._timer = None

    # ---------- lifecycle ----------

    def on_configure(self, _state):
        if not self._systemd.available():
            self.get_logger().error(
                '拿不到 systemd user manager（session bus 不通？）——台架会话没有承载者，'
                '拒绝进入配置态。检查 `systemctl --user show-environment` 是否可用。')
            return TransitionCallbackReturn.FAILURE

        self._status_pub = self.create_publisher(String, '/bench/status', 10)
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
        self._publish_status()
        return super().on_activate(state)

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
        if self._gain_client is not None:
            self.destroy_client(self._gain_client)
            self._gain_client = None
        if self._route_client is not None:
            self.destroy_client(self._route_client)
            self._route_client = None
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
        try:
            running = self.feedback.status()['state'] == 'running'
            publishers = self.get_publishers_info_by_topic('/joint_states')
            own_count = sum(p.node_name == 'firmware_topic_bridge' for p in publishers)
            external = [p.node_name for p in publishers
                        if p.node_name != 'firmware_topic_bridge' or not running or own_count > 1]
            self.feedback.reconcile(
                self.get_parameter('feedback_model').value,
                agent_running=self.agent.status()['state'] == 'running',
                simulation_running=self.simulation.status()['state'] == 'running',
                external_publishers=external)
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

    def _serial_ports_payload(self):
        ports = []
        for port in enumerate_serial_ports():
            ports.append({
                **port,
                'accessible': os.access(port['path'], os.R_OK | os.W_OK),
            })
        return ports

    # ---------- 服务回调 ----------

    def _on_options(self, _request, response):
        response.can_interfaces_json = json.dumps(
            enumerate_can_interfaces(), ensure_ascii=False)
        response.serial_ports_json = json.dumps(
            self._serial_ports_payload(), ensure_ascii=False)
        response.simulation_models = self._model_provider()
        response.mujoco_models = self._mujoco_model_provider()
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

    # ---------- 重力前馈（ADR 0013）----------

    def _on_start_gravity_feedforward(self, request, response):
        try:
            self.feedforward.start({
                'model': request.model,
                'joint_map': request.joint_map,
                'effort_gain': request.effort_gain,
            })
        except _EXPECTED_ERRORS as exc:
            return self._finish(response, exc)
        # 顺序（ADR 0013 决定 7）：unit 确认 running **之后**才切接线。切不过去不回滚
        # unit——命令继续走直连是有命令、只是没补偿（安全方向），回滚反而多一次断流。
        self._request_command_route(OVERLAY_COMMAND_TOPIC)
        return self._finish(response)

    def _on_stop_gravity_feedforward(self, _request, response):
        # 顺序与开启镜像：**先**切回直连、**确认成功**再停 unit。反过来，命令会在
        # 「接线还指着叠加节点、节点已经没了」的窗口里全部落空 → 固件命令超时软停，
        # 而机构没有抱闸。所以停 unit 挂在改道的成功回执上（见 _request_command_route）。
        self._request_command_route(
            DIRECT_COMMAND_TOPIC, on_success=self._stop_feedforward_unit)
        return self._finish(response)

    def _stop_feedforward_unit(self):
        try:
            self.feedforward.stop()
        except _EXPECTED_ERRORS as exc:
            self.get_logger().error(f'停叠加节点失败：{exc}')
        self._publish_status()

    def _on_set_gravity_feedforward_gain(self, request, response):
        try:
            self.feedforward.set_gain(request.effort_gain)
        except _EXPECTED_ERRORS as exc:
            return self._finish(response, exc)
        return self._finish(response)

    def _write_effort_gain(self, gain: float) -> None:
        """把增益写进运行中的叠加节点（异步，结果由回调回报）。

        服务未就绪时**不能**静默发出去：没有对端时请求会被中间件丢掉，future 永不
        完成，于是「界面显示已改、节点其实没改」。宁可在这里短暂等一下，就绪不了就报错。
        """
        if self._gain_client is None:
            self._gain_client = self.create_client(
                SetParametersAtomically, _GAIN_SERVICE)
        if not self._gain_client.wait_for_service(
                timeout_sec=_GAIN_DISCOVERY_TIMEOUT_S):
            raise RuntimeError(
                f'{_GAIN_NODE} 的参数服务不可达——叠加节点可能没在跑')
        request = SetParametersAtomically.Request()
        request.parameters = [
            Parameter('effort_gain', value=float(gain)).to_parameter_msg()]
        self._gain_client.call_async(request).add_done_callback(
            partial(self._on_gain_written, gain))

    def _request_command_route(self, topic: str, on_success=None) -> None:
        """把命令流改道到 topic（异步，结果由回调回报）。

        失败方向按调用点分岔，见两个服务处理的注释：开启时失败就留在直连，停止时
        失败则**不执行** on_success（即不停 unit）。
        """
        if self._route_client is None:
            self._route_client = self.create_client(
                SetParametersAtomically, _ROUTE_SERVICE)
        if not self._route_client.wait_for_service(
                timeout_sec=_GAIN_DISCOVERY_TIMEOUT_S):
            self.feedforward.note_route_failed(
                f'{_ROUTE_NODE} 的参数服务不可达——命令节拍节点没在跑，'
                '命令流改道未执行')
            self._publish_status()
            return
        request = SetParametersAtomically.Request()
        request.parameters = [
            Parameter('output_topic', value=str(topic)).to_parameter_msg()]
        self._route_client.call_async(request).add_done_callback(
            partial(self._on_route_written, topic, on_success))

    def _on_route_written(self, topic: str, on_success, future) -> None:
        try:
            result = future.result().result
        except Exception as exc:  # noqa: BLE001 - 回执里的任何失败都要说出来
            self.feedforward.note_route_failed(f'命令流改道失败：{exc}')
            self._publish_status()
            return
        if not result.successful:
            self.feedforward.note_route_failed(
                result.reason or '命令流改道被节点拒绝')
            self._publish_status()
            return
        self.feedforward.note_route_applied(topic)
        if on_success is not None:
            on_success()
        else:
            self._publish_status()

    def _on_gain_written(self, gain: float, future) -> None:
        try:
            result = future.result().result
        except Exception as exc:  # noqa: BLE001 - 回执里的任何失败都要说出来
            self.feedforward.note_gain_failed(f'增益写入失败：{exc}')
            return
        if not result.successful:
            self.feedforward.note_gain_failed(
                result.reason or '增益写入被节点拒绝')
            return
        self.feedforward.note_gain_applied(gain)


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
