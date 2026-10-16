"""bench_manager 节点：台架会话在 ROS 侧的**唯一编排者**。

会话进程（仿真 / micro-ROS Agent / 电机后端）由 systemd user transient unit 承载，
本节点不持有任何进程句柄——状态权威是 unit 状态，会话描述存在 unit 环境里，所以本
节点重启后能原样认领上一轮的会话，而不是把它们变成孤儿。

- 服务全在 `/bench/` 绝对命名空间下（web 前端按固定名调用，不做 remap）：
  `/bench/options`、`/bench/agent_status`、`/bench/{start,stop}_simulation`、
  `/bench/{start,stop}_agent`、`/bench/{start,stop}_motor_backend`、
  `/bench/motor_backend_command`、`/bench/get_unit_log`；
- 状态 `/bench/status`（std_msgs/String，JSON）1 Hz 周期发布 + 每条命令后立即发布，
  无订阅者时不构造；命令响应里也带同一份快照（status_json），失败时供页面回显；
- 日志**不进快照**：三个 unit 各留 80 行会把这个每秒都发的 JSON 撑成几十 KB，而页面
  只在抽屉展开时才看它。改成按需拉 `/bench/get_unit_log`。

lifecycle 语义（`robot_bringup` 的 `robot.launch.py` 负责 configure → activate）：

- `on_configure` **只做校验**，其中 systemd 可达性是 fail-fast——没有 user manager
  就承载不了任何会话，早点说清楚好过等到页面点「启动」才报错；
- `on_activate` 认领已在跑的 unit（只报告，不动它们）；
- `on_deactivate` 停掉整个 `bench.slice`——这是「安全停止」的唯一显式语义；
- `on_cleanup` 只释放句柄。

会话在两种情况下归宿不同：**干净退出**（Ctrl-C、SIGTERM）停掉 slice，因为本节点是
页面唯一的会话出口，它没了页面就停不掉还在跑的 Gazebo；**异常死亡**（崩溃、kill -9）
则把 unit 留给下一个 `bench_manager` 认领——进程归属在 systemd 而不是本节点，这正是
这次改造要消掉的「父进程一死子进程失联」。
"""

from functools import partial
import json
import os

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from std_msgs.msg import String

from bench_msgs.srv import (
    GetAgentStatus,
    GetBenchOptions,
    GetUnitLog,
    MotorBackendCommand,
    StartAgent,
    StartMotorBackend,
    StartSimulation,
    StopAgent,
    StopMotorBackend,
    StopSimulation,
)

from .agent_manager import AgentProcessManager, enumerate_serial_ports
from .motor_sim_manager import MotorSimProcessManager, hil_motor_backend_request
from .simulation_manager import (
    SimulationProcessManager,
    enumerate_can_interfaces,
    enumerate_mujoco_models,
    enumerate_simulation_models,
)
from .systemd_units import (
    UNIT_AGENT, UNIT_MOTOR_BACKEND, UNIT_SIMULATION, SystemdUnits,
)

_STATUS_PERIOD_S = 1.0
# 管理器契约内的可预期失败；其余异常照常冒泡（rclpy 记日志，那是 bug）
_EXPECTED_ERRORS = (ValueError, PermissionError, RuntimeError, OSError)

# 日志服务的白名单：`unit_name` 直接进 `journalctl -u`，只认这三个契约名，
# 免得页面（或任何服务调用方）拿它去翻别的 unit。
_UNITS = (UNIT_SIMULATION, UNIT_MOTOR_BACKEND, UNIT_AGENT)
_LOG_LINES_DEFAULT = 80
_LOG_LINES_MAX = 2000


def _simulation_request(message):
    """把服务的扁平字段还原成 SimulationProcessManager 认识的请求 dict。

    空串 = 未给该参数（缺省不进命令行），与旧 HTTP 请求「键缺省」的语义一致。
    """
    request = {
        'backend': message.backend,
        'transport': message.transport,
        'controller_mode': message.controller_mode,
        'model': message.model,
        # 非 vcan 的放行条件，simulation_manager 里校验；页面自己拦一道不等于
        # 服务端收到过，漏传会让物理总线上的仿真永远起不来。
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
        model_dir = self.get_parameter('model_dir').value
        models_dir = ''
        if model_dir and os.path.isdir(os.path.join(model_dir, 'models')):
            models_dir = os.path.join(model_dir, 'models')
        elif model_dir:
            self.get_logger().warn(
                'model_dir 下没有 models/，模型枚举回落到 robot_sim 安装树：%s'
                % model_dir)
        models_root = models_dir or None
        self._model_provider = partial(enumerate_simulation_models, models_root)
        self._mujoco_model_provider = partial(enumerate_mujoco_models, models_root)

        # 一个 SystemdUnits 实例贯穿三个 manager：唯一缝只开一次，测试注入也只需一处。
        self._systemd = SystemdUnits()
        self.agent = AgentProcessManager(systemd=self._systemd)
        self.simulation = SimulationProcessManager(
            model_provider=self._model_provider,
            mujoco_model_provider=self._mujoco_model_provider,
            systemd=self._systemd)
        self.motor_backend = MotorSimProcessManager(
            model_provider=self._model_provider, systemd=self._systemd)

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
        ):
            self.create_service(srv_type, name, callback)
        self._timer = self.create_timer(_STATUS_PERIOD_S, self._publish_status)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
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
        self._timer = None
        self._status_pub = None
        return TransitionCallbackReturn.SUCCESS

    def _sessions(self):
        return (('simulation', self.simulation),
                ('motor backend', self.motor_backend),
                ('micro-ROS Agent', self.agent))

    # ---------- 状态快照与响应收尾 ----------

    def _snapshot_json(self):
        return json.dumps({
            'simulation': self.simulation.status(),
            'agent': self.agent.status(),
            'motor_backend': self.motor_backend.status(),
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
