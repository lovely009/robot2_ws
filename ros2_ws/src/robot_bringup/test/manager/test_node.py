"""`BenchManager` 节点契约的单测：状态快照形状、lifecycle 门槛、日志服务白名单。

不起 rclpy 上下文——被测的都是纯函数式的回调与派生，用 `__new__` 造出的空壳节点
（跳过 `__init__`）加假 systemd 就够。真起一个节点会拉起 DDS 参与者，而这里要验的
东西跟 DDS 无关。
"""

from __future__ import annotations

from concurrent.futures import Future
import json
import logging
from pathlib import Path

import pytest
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import SetParametersAtomically

from conftest import FakeSystemd

from bench_manager.agent_manager import AgentProcessManager
from bench_manager.feedback_manager import FirmwareFeedbackManager, FeedbackInventory
from bench_manager.gravity_feedforward_manager import (
    GravityFeedforwardManager, PARAMETER_TIMEOUT_S,
)
from bench_manager.motor_sim_manager import MotorSimProcessManager
from bench_manager.node import BenchManager, _GRAVITY_SERVICE, _simulation_request
from bench_manager.simulation_manager import SimulationProcessManager
from bench_manager.systemd_units import (
    SLICE, UNIT_AGENT, UNIT_MOTOR_BACKEND,
    UNIT_SIMULATION, UNIT_FEEDBACK, SystemdUnits,
)

_UNITS = (UNIT_SIMULATION, UNIT_MOTOR_BACKEND, UNIT_AGENT, UNIT_FEEDBACK)
_MODEL = 'half_body_robot'


class _Response:
    ok = False
    error = ''
    text = ''


class _ParameterClient:
    def __init__(self, ready=True, successful=True, reason='', defer=True):
        self.ready = ready
        self.successful = successful
        self.reason = reason
        self.defer = defer
        self.requests = []
        self.pending = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        future = Future()
        self.pending.append(future)
        if not self.defer:
            self.fire()
        return future

    def fire(self):
        pending, self.pending = self.pending, []
        for future in pending:
            response = SetParametersAtomically.Response()
            response.result = SetParametersResult(
                successful=self.successful, reason=self.reason)
            future.set_result(response)


@pytest.fixture
def models(tmp_path):
    config = tmp_path / _MODEL / 'config'
    config.mkdir(parents=True)
    (config / 'motor_driver.yaml').write_text(
        'motor_hardware:\n  motors:\n    motor: {id: 1, joint: joint_link1}\n')
    return tmp_path


def _shell(systemd: FakeSystemd, models=None,
           client: _ParameterClient | None = None) -> BenchManager:
    """不建 rclpy 上下文；所有外部 IO 均由假体承接。"""
    node = BenchManager.__new__(BenchManager)
    node._systemd = SystemdUnits(runner=systemd)
    node.feedback = FirmwareFeedbackManager(Path('/missing-models'), node._systemd)
    node._feedback_error = ''
    node._feedback_inventory = FeedbackInventory()
    node._feedback_inventory_sub = None
    node._feedback_stop_pub = None
    node.get_parameter = lambda _name: type('Parameter', (), {'value': ''})()
    node.get_publishers_info_by_topic = lambda _topic: []
    node.agent = AgentProcessManager(
        port_provider=lambda: [], systemd=node._systemd)
    node.simulation = SimulationProcessManager(
        can_provider=lambda: [], serial_provider=lambda: [],
        model_provider=lambda: [], mujoco_model_provider=lambda: [],
        systemd=node._systemd)
    node.motor_backend = MotorSimProcessManager(
        can_provider=lambda: [], serial_provider=lambda: [],
        model_provider=lambda: [], systemd=node._systemd)
    node._now = [100.0]
    node.feedforward = GravityFeedforwardManager(
        models or Path('/missing-models'), node._write_gravity_parameters,
        clock=lambda: node._now[0])
    node._gravity_client = client if client is not None else _ParameterClient()
    node._control_status_sub = None
    node._status_pub = None
    node._timer = None
    node.get_logger = lambda: logging.getLogger('bench_manager-test')
    node._managed_entities = []
    return node


def _all_absent() -> FakeSystemd:
    return FakeSystemd({unit: {'LoadState': 'not-found'} for unit in _UNITS})


# --- 快照形状（前端与 srv 的共同契约）---------------------------------------

def test_snapshot_has_five_sections_and_no_log_key() -> None:
    """日志**不进**快照。

    四个 unit 各留 80 行会把这个每秒都发的 JSON 撑成几十 KB，而页面只在抽屉展开时
    才看它——改成按需拉 `/bench/get_unit_log`。
    """
    node = _shell(_all_absent())

    snapshot = json.loads(node._snapshot_json())

    assert set(snapshot) == {'simulation', 'agent', 'motor_backend', 'feedback',
                             'gravity_feedforward'}
    for section in snapshot.values():
        assert 'log' not in section
    assert snapshot['simulation'] == {
        'state': 'stopped', 'config': None, 'returncode': None}
    assert snapshot['agent'] == {
        'state': 'stopped', 'device': '', 'returncode': None,
        'auto_started': False}
    assert snapshot['motor_backend'] == {
        'state': 'stopped', 'config': None, 'returncode': None,
        'auto_started': False}
    # 前馈没起时 config 是 None：页面据此判断按钮该显示「开」还是「关」。
    assert snapshot['gravity_feedforward'] == {
        'state': 'stopped', 'config': None, 'returncode': None,
        'gain_error': '', 'error': '/control/status unavailable or stale',
        'pending': None, 'fresh': False}


def test_snapshot_state_is_derived_from_the_unit() -> None:
    """`state` 由后端派生，页面不自己拼四态。"""
    systemd = _all_absent()
    systemd.properties[UNIT_SIMULATION] = {
        'LoadState': 'loaded', 'ActiveState': 'failed', 'SubState': 'failed',
        'ExecMainStatus': '2', 'Environment': 'BENCH_SIM_MODEL=half_body_robot'}
    node = _shell(systemd)

    snapshot = json.loads(node._snapshot_json())

    assert snapshot['simulation']['state'] == 'failed'
    assert snapshot['simulation']['returncode'] == 2
    assert snapshot['simulation']['config'] == {'model': 'half_body_robot'}


def test_feedback_tick_runs_even_without_status_subscribers() -> None:
    node = _shell(_all_absent())
    reconciled = []
    node.feedback.reconcile = lambda *args, **kwargs: reconciled.append((args, kwargs))
    node._tick()
    assert reconciled == [(('',), {'agent_running': False, 'simulation_running': False,
                                   'external_publishers': [], 'motor_ids': [], 'session_id': 0})]


def test_feedback_model_parameter_rejects_paths_without_changing_units() -> None:
    from rclpy.parameter import Parameter
    systemd = _all_absent()
    node = _shell(systemd)
    result = node._validate_feedback_parameters([Parameter('feedback_model', value='../other')])
    assert not result.successful
    assert not systemd.commands()


def test_external_joint_publisher_prevents_duplicate_feedback() -> None:
    node = _shell(_all_absent())
    node.get_parameter = lambda _name: type('Parameter', (), {'value': 'arm'})()
    node.get_publishers_info_by_topic = lambda _topic: [
        type('Endpoint', (), {'node_name': 'firmware_topic_bridge'})()]
    node._tick()
    assert node.feedback.status()['source'] == 'external'


def test_activate_restores_model_from_feedback_unit() -> None:
    systemd = _all_absent()
    systemd.properties[UNIT_FEEDBACK] = {
        'LoadState': 'loaded', 'ActiveState': 'active',
        'Environment': 'BENCH_FEEDBACK_MODEL=arm_a'}
    node = _shell(systemd)
    restored = []
    node.set_parameters = lambda parameters: restored.extend(p.value for p in parameters)
    node.on_activate(None)
    assert restored == ['arm_a']
    assert not [c for c in systemd.commands() if c[0] == 'systemd-run']


# --- lifecycle 门槛 ---------------------------------------------------------

@pytest.mark.parametrize('ports,device,enabled,running,expected', [
    (['/dev/ttyACM0'], '', True, False, '/dev/ttyACM0'),
    ([], '', True, False, None),
    (['/dev/ttyACM0', '/dev/ttyUSB0'], '', True, False, None),
    (['/dev/ttyACM0', '/dev/ttyUSB0'], '/dev/ttyUSB0', True, False, '/dev/ttyUSB0'),
    (['/dev/ttyACM0'], '', False, False, None),
    (['/dev/ttyACM0'], '', True, True, None),
])
def test_activate_starts_default_agent_only_when_unambiguous(
        monkeypatch, ports, device, enabled, running, expected):
    from types import SimpleNamespace
    node = _shell(_all_absent())
    values = {'auto_start_agent': enabled, 'agent_device': device, 'feedback_model': ''}
    node.get_parameter = lambda name: SimpleNamespace(value=values[name])
    monkeypatch.setattr('bench_manager.node.enumerate_serial_ports',
                        lambda: [{'path': port} for port in ports])
    monkeypatch.setattr(node.agent, 'status',
                        lambda: {'state': 'running' if running else 'stopped'})
    started = []
    monkeypatch.setattr(node.agent, 'start',
                        lambda path, auto=False: started.append((path, auto)))
    node.on_activate(None)
    assert started == ([] if expected is None else [(expected, True)])


def test_agent_autostart_failure_keeps_manager_active(monkeypatch):
    from types import SimpleNamespace
    from unittest.mock import Mock
    from rclpy.lifecycle import TransitionCallbackReturn
    node = _shell(_all_absent())
    values = {'auto_start_agent': True, 'agent_device': '/dev/ttyACM0',
              'feedback_model': ''}
    node.get_parameter = lambda name: SimpleNamespace(value=values[name])
    logger = Mock()
    node.get_logger = lambda: logger

    def occupied(*args, **kwargs):
        raise RuntimeError('serial port occupied')

    monkeypatch.setattr(node.agent, 'start', occupied)
    assert node.on_activate(None) is TransitionCallbackReturn.SUCCESS
    assert 'serial port occupied' in logger.warn.call_args.args[0]


def test_configure_fails_fast_without_a_user_manager() -> None:
    """没有 systemd user manager 就没法承载会话——在 configure 阶段就说清楚。

    留到运行期的话，页面点「启动」才报错，而那时已经过了好几层（服务、管理器、
    命令行构造），错误信息离根因很远。
    """
    from rclpy.lifecycle import TransitionCallbackReturn

    systemd = FakeSystemd(fail={('systemctl', '--user', 'show-environment'):
                                (1, 'Failed to connect to bus')})
    node = _shell(systemd)

    assert BenchManager.on_configure(node, None) is TransitionCallbackReturn.FAILURE


def test_deactivate_stops_the_whole_slice() -> None:
    """`deactivate` 是「安全停止」的唯一显式语义：停整个 slice，不是逐个 unit。"""
    systemd = _all_absent()
    node = _shell(systemd)

    BenchManager.on_deactivate(node, None)

    assert ['systemctl', '--user', 'stop', SLICE] in systemd.commands()


def test_activate_adopts_running_units_without_rebuilding_them() -> None:
    """认领已在跑的 unit：只报告，不动它们。"""
    systemd = _all_absent()
    systemd.properties[UNIT_AGENT] = {
        'LoadState': 'loaded', 'ActiveState': 'active', 'SubState': 'running',
        'Environment': 'BENCH_DEVICE=/dev/serial/by-id/usb-controller'}
    node = _shell(systemd)

    BenchManager.on_activate(node, None)

    assert not [call for call in systemd.commands() if call[0] == 'systemd-run']


# --- 日志服务白名单 ---------------------------------------------------------

@pytest.mark.parametrize('unit', _UNITS)
def test_unit_log_accepts_contract_units(unit: str) -> None:
    systemd = _all_absent()
    node = _shell(systemd)

    class _Request:
        unit_name = unit
        lines = 40

    response = BenchManager._on_get_unit_log(node, _Request(), _Response())

    assert response.ok
    assert ['journalctl', '--user', '-u', unit, '-n', '40',
            '--output=cat', '--no-pager'] in systemd.commands()


@pytest.mark.parametrize('unit', ['', 'bench.slice', 'sshd.service', '../etc'])
def test_unit_log_rejects_anything_else(unit: str) -> None:
    """`unit_name` 直接进 `journalctl -u`，只认四个契约名。"""
    systemd = _all_absent()
    node = _shell(systemd)

    class _Request:
        unit_name = unit
        lines = 0

    response = BenchManager._on_get_unit_log(node, _Request(), _Response())

    assert not response.ok
    assert not [call for call in systemd.commands() if call[0] == 'journalctl']


# --- 服务消息 → 请求 dict ---------------------------------------------------

def test_simulation_request_keeps_the_confirmation_bit() -> None:
    from bench_msgs.srv import StartSimulation

    message = StartSimulation.Request()
    message.backend = 'gazebo'
    message.transport = 'socketcan'
    message.controller_mode = 'firmware'
    message.model = 'half_body_robot'
    message.sim_interface = 'can0'
    message.physical_bus_isolated = True

    request = _simulation_request(message)

    assert request['physical_bus_isolated'] is True
    assert request['sim_interface'] == 'can0'
    assert 'driver_interface' not in request, '空串＝未给该参数，不该进命令行'


# --- ros2_control 重力参数服务 ----------------------------------------------

def _start_request(model: str = _MODEL, joint_map: str = '1:joint_link1',
                   effort_gain: float = 1.0):
    from bench_msgs.srv import StartGravityFeedforward
    message = StartGravityFeedforward.Request()
    message.model = model
    message.joint_map = joint_map
    message.effort_gain = effort_gain
    return message


def _observe(node, **overrides):
    from std_msgs.msg import String
    value = dict(model=_MODEL, joint_names=['joint_link1'], mode='trajectory',
                 gravity_enabled=False, effort_gain=1.0) | overrides
    node._on_control_status(String(data=json.dumps(value)))


def _values(client):
    from rclpy.parameter import Parameter
    return [{p.name: Parameter.from_parameter_msg(p).value for p in r.parameters}
            for r in client.requests]


def _ff(response):
    return json.loads(response.status_json)['gravity_feedforward']


def test_start_writes_atomic_controller_parameters_and_returns_explicit_pending(models):
    systemd = _all_absent()
    node = _shell(systemd, models)
    _observe(node)
    response = node._on_start_gravity_feedforward(
        _start_request(effort_gain=0.6), _Response())
    assert response.ok  # 受理成功；pending 不代表已经运行。
    assert _values(node._gravity_client) == [{'enabled': True, 'effort_gain': 0.6}]
    values = node._gravity_client.requests[0].parameters
    assert [p.value.type for p in values] == [1, 3]  # BOOL / DOUBLE
    assert _ff(response)['pending'] == 'start'
    assert _ff(response)['state'] == 'stopped'
    assert _ff(response)['config'] == {'model': _MODEL, 'effort_gain': 1.0}
    assert not [c for c in systemd.commands() if c[0] == 'systemd-run']
    node._gravity_client.fire()
    assert node.feedforward.status()['pending'] == 'start'
    _observe(node, gravity_enabled=True, effort_gain=0.6)
    assert node.feedforward.status()['state'] == 'running'
    assert node.feedforward.status()['pending'] is None


def test_configure_subscribes_to_real_control_status_without_creating_a_ros_graph():
    from rclpy.lifecycle import TransitionCallbackReturn
    from std_msgs.msg import String
    node = _shell(_all_absent())
    subscriptions, services = [], []
    node.create_publisher = lambda *args: object()
    node.create_subscription = lambda *args: subscriptions.append(args) or object()
    node.create_service = lambda kind, name, callback: services.append(name)
    node.create_timer = lambda *args: object()
    assert node.on_configure(None) is TransitionCallbackReturn.SUCCESS
    from can_bridge_msgs.msg import MotorControlStateArray
    assert subscriptions == [(MotorControlStateArray, '/motor/control_states', node._feedback_inventory.observe, 10),
                             (String, '/control/status', node._on_control_status, 10)]
    assert '/bench/start_gravity_feedforward' in services
    assert '/bench/stop_gravity_feedforward' in services
    assert '/bench/set_gravity_feedforward_gain' in services


def test_parameter_client_uses_controller_service_without_waiting_or_spinning():
    node = _shell(_all_absent())
    client = node._gravity_client
    node._gravity_client = None
    created = []
    node.create_client = lambda kind, name: created.append((kind, name)) or client
    node._write_gravity_parameters({'enabled': False, 'effort_gain': 0.0})
    assert created == [(SetParametersAtomically, _GRAVITY_SERVICE)]
    assert _GRAVITY_SERVICE == '/host_control/gravity_controller/set_parameters_atomically'
    assert _values(client) == [{'enabled': False, 'effort_gain': 0.0}]


def test_stop_only_disables_controller_and_zeros_effort(models):
    systemd = _all_absent()
    node = _shell(systemd, models)
    _observe(node, gravity_enabled=True)
    response = node._on_stop_gravity_feedforward(None, _Response())
    assert response.ok
    assert _values(node._gravity_client) == [{'enabled': False, 'effort_gain': 0.0}]
    assert _ff(response)['pending'] == 'stop'
    assert _ff(response)['state'] == 'running'
    node._gravity_client.fire()
    assert node.feedforward.status()['state'] == 'running'
    _observe(node, gravity_enabled=False, effort_gain=0.0)
    assert node.feedforward.status()['state'] == 'stopped'
    assert node.feedforward.status()['pending'] is None
    assert not [c for c in systemd.commands() if c[1:3] == ['--user', 'stop']]


@pytest.mark.parametrize('model,joint_map', [('other', ''), (_MODEL, '1:wrong'),
                                            (_MODEL, '2:joint_link1')])
def test_validation_error_is_in_response_and_status(models, model, joint_map):
    node = _shell(_all_absent(), models)
    _observe(node)
    response = node._on_start_gravity_feedforward(
        _start_request(model=model, joint_map=joint_map), _Response())
    assert not response.ok
    assert response.error == _ff(response)['error']
    assert not node._gravity_client.requests


def test_missing_real_status_rejects_start_without_any_parameter_request(models):
    node = _shell(_all_absent(), models)
    response = node._on_start_gravity_feedforward(_start_request(), _Response())
    assert not response.ok
    assert 'stale' in _ff(response)['error']
    assert not node._gravity_client.requests


@pytest.mark.parametrize('ready,successful,error', [
    (False, True, 'unavailable'), (True, False, 'refused'),
])
def test_immediate_parameter_failure_cannot_return_success(models, ready, successful, error):
    client = _ParameterClient(ready=ready, successful=successful, reason='refused', defer=False)
    node = _shell(_all_absent(), models, client)
    _observe(node)
    response = node._on_start_gravity_feedforward(_start_request(), _Response())
    assert not response.ok
    assert error in response.error
    assert response.error == _ff(response)['error']
    assert _ff(response)['pending'] is None
    assert _ff(response)['state'] == 'stopped'


def test_deferred_rejection_is_published_as_error_without_fake_running(models):
    client = _ParameterClient(successful=False, reason='refused')
    node = _shell(_all_absent(), models, client)
    _observe(node)
    response = node._on_start_gravity_feedforward(_start_request(), _Response())
    assert response.ok and _ff(response)['pending'] == 'start'
    client.fire()
    node._tick()
    assert node.feedforward.status()['error'] == 'refused'
    assert node.feedforward.status()['state'] == 'stopped'


def test_tick_expires_pending_request_even_without_status_subscribers(models):
    node = _shell(_all_absent(), models)
    _observe(node)
    node._on_start_gravity_feedforward(_start_request(), _Response())
    node._now[0] += PARAMETER_TIMEOUT_S
    node._tick()
    assert node.feedforward.status()['pending'] is None
    assert 'timed out' in node.feedforward.status()['error']
    assert node._gravity_client.pending[0].cancelled()


def test_set_gain_requires_active_enabled_controller(models):
    node = _shell(_all_absent(), models)
    _observe(node, mode='manual', gravity_enabled=True)
    response = node._on_set_gravity_feedforward_gain(
        type('Request', (), {'effort_gain': 0.5})(), _Response())
    assert not response.ok
    assert 'not running' in _ff(response)['error']
    assert not node._gravity_client.requests


def test_gain_changes_only_on_real_readback(models):
    node = _shell(_all_absent(), models)
    _observe(node, gravity_enabled=True)
    response = node._on_set_gravity_feedforward_gain(
        type('Request', (), {'effort_gain': 0.25})(), _Response())
    assert response.ok and _ff(response)['pending'] == 'gain'
    assert _values(node._gravity_client) == [{'effort_gain': 0.25}]
    node._gravity_client.fire()
    assert node.feedforward.status()['config']['effort_gain'] == 1.0
    _observe(node, gravity_enabled=True, effort_gain=0.25)
    assert node.feedforward.status()['config']['effort_gain'] == 0.25
    assert node.feedforward.status()['pending'] is None


@pytest.mark.parametrize('payload', ['{', '[]', 'null', '{}',
    '{"model":"arm","joint_names":["a"],"mode":"trajectory",'
    '"gravity_enabled":"true","effort_gain":1.0}'])
def test_invalid_control_status_invalidates_previously_running_state(payload):
    from std_msgs.msg import String
    node = _shell(_all_absent())
    _observe(node, gravity_enabled=True)
    node._on_control_status(String(data=payload))
    assert node.feedforward.status()['state'] == 'stopped'
    assert not node.feedforward.status()['fresh']
    assert node.feedforward.status()['error']


def test_cleanup_releases_controller_handles_and_cancels_pending_without_stopping_units(models):
    systemd = _all_absent()
    node = _shell(systemd, models)
    _observe(node, gravity_enabled=True)
    node._on_stop_gravity_feedforward(None, _Response())
    client, subscription = node._gravity_client, object()
    node._control_status_sub = subscription
    destroyed = []
    node.destroy_client = lambda value: destroyed.append(value)
    node.destroy_subscription = lambda value: destroyed.append(value)
    node.destroy_publisher = lambda value: None
    node.destroy_timer = lambda value: None
    node.on_cleanup(None)
    assert destroyed == [client, subscription]
    assert node._gravity_client is None and node._control_status_sub is None
    assert client.pending[0].cancelled()
    assert node.feedforward.status()['state'] == 'stopped'
    assert not [c for c in systemd.commands() if c[1:3] == ['--user', 'stop']]


# --- 反馈发布者归属 ---------------------------------------------------------

@pytest.mark.parametrize('publishers,managed,expected_external', [
    (['workbench_control'], True, False),
    (['firmware_topic_bridge'], True, False),
    (['workbench_control'], False, True),
    (['firmware_topic_bridge'], False, True),
    (['workbench_control', 'workbench_control'], True, True),
    (['firmware_topic_bridge', 'workbench_control'], True, True),
    (['workbench_control', 'external_joint_source'], True, True),
    (['external_joint_source'], True, True),
])
def test_feedback_ownership_excludes_only_one_managed_publisher(
        publishers, managed, expected_external):
    from types import SimpleNamespace
    systemd = _all_absent()
    if managed:
        systemd.properties[UNIT_FEEDBACK] = {
            'LoadState': 'loaded', 'ActiveState': 'active',
            'Environment': 'BENCH_FEEDBACK_MODEL=arm BENCH_CONTROL_EXECUTION=ros2_control BENCH_CONTROL_MOTORS=1 BENCH_MCU_SESSION=100'}
    node = _shell(systemd)
    node._feedback_inventory.current = lambda: ([1], 100)
    node.feedback._validate = lambda _: SimpleNamespace(actuator_ids={'m1':1}, groups=[SimpleNamespace(actuators=['m1'])])
    node.agent.status = lambda: {'state': 'running'}
    node.get_parameter = lambda _name: SimpleNamespace(value='arm')
    node.get_publishers_info_by_topic = lambda _topic: [
        SimpleNamespace(node_name=name) for name in publishers]
    node._tick()
    assert (node.feedback.status()['source'] == 'external') == expected_external
    stopped = ['systemctl', '--user', 'stop', UNIT_FEEDBACK] in systemd.commands()
    assert stopped == (managed and expected_external)
    assert not [c for c in systemd.commands() if c[0] == 'systemd-run']


def test_immediate_empty_future_exception_cannot_return_success(models):
    node = _shell(_all_absent(), models)
    _observe(node)
    future = Future()
    future.set_exception(TimeoutError())
    node._gravity_client.call_async = lambda request: future
    response = node._on_start_gravity_feedforward(_start_request(), _Response())
    assert not response.ok
    assert response.error == _ff(response)['error']
    assert response.error
    assert _ff(response)['pending'] is None
