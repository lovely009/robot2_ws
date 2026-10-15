"""bench_manager 节点：把台架主机侧能力（仿真 / micro-ROS Agent / 电机后端）
暴露为一组 ROS 服务 + 一个状态话题，取代旧 web_host 进程内的私有 HTTP API。

- 服务全在 `/bench/` 绝对命名空间下（web 前端按固定名调用，不做 remap）：
  `/bench/options`、`/bench/agent_status`、`/bench/{start,stop}_simulation`、
  `/bench/{start,stop}_agent`、`/bench/{start,stop}_motor_backend`、
  `/bench/motor_backend_command`；
- 状态 `/bench/status`（std_msgs/String，JSON）1 Hz 周期发布 + 每条命令后立即发布，
  无订阅者时不构造；命令响应里也带同一份快照（status_json），失败时供页面回显；
- 子进程所有权全在本节点：退出（launch 的 SIGINT 会走到 finally）时停掉三个 manager
  持有的子进程。
"""

from functools import partial
import json
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from bench_msgs.srv import (
    GetAgentStatus,
    GetBenchOptions,
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

_STATUS_PERIOD_S = 1.0
# 管理器契约内的可预期失败；其余异常照常冒泡（rclpy 记日志，那是 bug）
_EXPECTED_ERRORS = (ValueError, PermissionError, RuntimeError, OSError)


def _simulation_request(message):
    """把服务的扁平字段还原成 SimulationProcessManager 认识的请求 dict。

    空串 = 未给该参数（缺省不进命令行），与旧 HTTP 请求「键缺省」的语义一致。
    """
    request = {
        'backend': message.backend,
        'transport': message.transport,
        'controller_mode': message.controller_mode,
        'model': message.model,
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


class BenchManager(Node):

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

        self.agent = AgentProcessManager()
        self.simulation = SimulationProcessManager(
            model_provider=self._model_provider,
            mujoco_model_provider=self._mujoco_model_provider)
        self.motor_backend = MotorSimProcessManager(
            model_provider=self._model_provider)

        self._status_pub = self.create_publisher(String, '/bench/status', 10)
        self.create_service(GetBenchOptions, '/bench/options', self._on_options)
        self.create_service(
            GetAgentStatus, '/bench/agent_status', self._on_agent_status)
        self.create_service(
            StartSimulation, '/bench/start_simulation', self._on_start_simulation)
        self.create_service(
            StopSimulation, '/bench/stop_simulation', self._on_stop_simulation)
        self.create_service(StartAgent, '/bench/start_agent', self._on_start_agent)
        self.create_service(StopAgent, '/bench/stop_agent', self._on_stop_agent)
        self.create_service(
            StartMotorBackend, '/bench/start_motor_backend',
            self._on_start_motor_backend)
        self.create_service(
            StopMotorBackend, '/bench/stop_motor_backend',
            self._on_stop_motor_backend)
        self.create_service(
            MotorBackendCommand, '/bench/motor_backend_command',
            self._on_motor_backend_command)
        self.create_timer(_STATUS_PERIOD_S, self._publish_status)

    # ---------- 状态快照与响应收尾 ----------

    def _snapshot_json(self):
        return json.dumps({
            'simulation': self.simulation.status(),
            'agent': self.agent.status(),
            'motor_backend': self.motor_backend.status(),
        }, ensure_ascii=False)

    def _publish_status(self):
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
                    except OSError as exc:
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
        # 随仿真自动拉起的电机后端联动停；手动起的保持运行
        if self.motor_backend.status().get('auto_started'):
            try:
                self.motor_backend.stop()
            except OSError as exc:
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


def main(args=None):
    rclpy.init(args=args)
    node = BenchManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 子进程只由本节点持有：退出即回收（launch 升级 SIGTERM 前必须自己停干净）
        for label, manager in (('simulation', node.simulation),
                               ('motor backend', node.motor_backend),
                               ('micro-ROS Agent', node.agent)):
            try:
                manager.stop()
            except Exception as exc:
                node.get_logger().error(
                    'Could not stop %s: %s' % (label, exc))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
