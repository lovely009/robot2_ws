"""`BenchManager` 节点契约的单测：状态快照形状、lifecycle 门槛、日志服务白名单。

不起 rclpy 上下文——被测的都是纯函数式的回调与派生，用 `__new__` 造出的空壳节点
（跳过 `__init__`）加假 systemd 就够。真起一个节点会拉起 DDS 参与者，而这里要验的
东西跟 DDS 无关。
"""

from __future__ import annotations

import json
import logging
from pathlib import Path

import pytest
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import SetParametersAtomically

from conftest import FakeSystemd

from bench_manager.agent_manager import AgentProcessManager
from bench_manager.feedback_manager import FirmwareFeedbackManager
from bench_manager.gravity_feedforward_manager import (
    DIRECT_COMMAND_TOPIC, OVERLAY_COMMAND_TOPIC,
    GravityFeedforwardProcessManager,
)
from bench_manager.motor_sim_manager import MotorSimProcessManager
from bench_manager.node import BenchManager, _simulation_request
from bench_manager.simulation_manager import SimulationProcessManager
from bench_manager.systemd_units import (
    SLICE, UNIT_AGENT, UNIT_GRAVITY_FEEDFORWARD, UNIT_MOTOR_BACKEND,
    UNIT_SIMULATION, UNIT_FEEDBACK, SystemdUnits,
)

_UNITS = (UNIT_SIMULATION, UNIT_MOTOR_BACKEND, UNIT_AGENT,
          UNIT_GRAVITY_FEEDFORWARD, UNIT_FEEDBACK)
_MODEL = 'half_body_robot'


class _Response:
    ok = False
    error = ''
    text = ''


class _RouteFuture:
    def __init__(self, client):
        self._client = client

    def add_done_callback(self, callback):
        if self._client.defer:
            self._client.pending.append((callback, self))
        else:
            callback(self)

    def result(self):
        response = SetParametersAtomically.Response()
        response.result = SetParametersResult(
            successful=self._client.successful, reason=self._client.reason)
        return response


class _RouteClient:
    """假改道通道（command_streamer 的参数服务）。

    默认**同步回执**：`call_async` 当场把 future 交给回调，于是「切接线 → 停 unit」
    这条链在一个调用栈里走完，断言好写。`defer=True` 把回执扣住、由测试显式
    `fire()`——验「切成功之前不许停 unit」要的就是这个窗口。
    """

    def __init__(self, ready=True, successful=True, reason='', defer=False):
        self.ready = ready
        self.successful = successful
        self.reason = reason
        self.defer = defer
        self.topics: list[str] = []
        self.pending: list = []
        # 请求改道那一刻 unit 的状态，由测试在拿到 node 后挂上。
        self.probe = None
        self.seen_states: list[str] = []

    def wait_for_service(self, timeout_sec=None):
        return self.ready

    def call_async(self, request):
        self.topics.append(request.parameters[0].value.string_value)
        if self.probe is not None:
            self.seen_states.append(self.probe())
        return _RouteFuture(self)

    def fire(self):
        pending, self.pending = self.pending, []
        for callback, future in pending:
            callback(future)


@pytest.fixture
def model_urdf(tmp_path: Path) -> str:
    """前馈管理器按 `is_file()` 判 URDF 存不存在，所以这里得给真文件。"""
    path = tmp_path / 'models' / _MODEL / 'urdf' / f'{_MODEL}.urdf'
    path.parent.mkdir(parents=True)
    path.write_text('<robot name="x"/>', encoding='utf-8')
    return str(path)


def _shell(systemd: FakeSystemd, urdf_path: str = '',
           written: list[float] | None = None,
           route: _RouteClient | None = None) -> BenchManager:
    """跳过 `__init__`（那要 rclpy 上下文）造一个节点外壳。

    `urdf_path` 缺省为空串＝解析不出任何模型，前馈相关之外的用例不必关心它。
    """
    node = BenchManager.__new__(BenchManager)
    node._systemd = SystemdUnits(runner=systemd)
    node.feedback = FirmwareFeedbackManager(Path('/missing-models'), node._systemd)
    node._feedback_error = ''
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
    node.feedforward = GravityFeedforwardProcessManager(
        urdf_provider=lambda _model: urdf_path,
        gain_writer=(written.append if written is not None else None),
        systemd=node._systemd)
    node._gain_client = None
    node._route_client = route if route is not None else _RouteClient()
    node._status_pub = None
    node._timer = None
    node.get_logger = lambda: logging.getLogger('bench_manager-test')
    # `LifecycleNode.__init__` 建的受管实体表；空壳得自己补上，
    # 否则 `super().on_activate()` 会在遍历它时炸（回调体本身没问题）。
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
        'gain_error': '', 'command_topic': None, 'route_error': ''}


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
                                   'external_publishers': []})]


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


# --- 重力前馈服务（ADR 0013）------------------------------------------------

def _start_request(model: str = _MODEL, joint_map: str = '1:joint_link1',
                   effort_gain: float = 1.0):
    from bench_msgs.srv import StartGravityFeedforward

    message = StartGravityFeedforward.Request()
    message.model = model
    message.joint_map = joint_map
    message.effort_gain = effort_gain
    return message


def _creations(systemd: FakeSystemd) -> list[list[str]]:
    return [call for call in systemd.commands() if call[0] == 'systemd-run']


def test_start_gravity_feedforward_passes_the_three_fields_through(
        model_urdf: str) -> None:
    """服务的三个字段都要真进 unit：漏一个页面就会拿到「起来了但不是你要的」。"""
    systemd = _all_absent()
    node = _shell(systemd, urdf_path=model_urdf)

    response = BenchManager._on_start_gravity_feedforward(
        node, _start_request(effort_gain=0.6), _Response())

    assert response.ok
    assert json.loads(response.status_json)['gravity_feedforward']['config'] == {
        'model': _MODEL, 'urdf_path': model_urdf,
        'joint_map': '1:joint_link1', 'effort_gain': 0.6}


# --- 命令流改道（ADR 0014 决定 3）------------------------------------------
# 页面只下设定值，命令从哪条路出去由 bench_manager 改 command_streamer 的 output_topic
# 决定。两个方向的顺序都是安全边界，失败方向还不同，所以逐条钉住。

def test_start_switches_the_command_route_only_after_the_unit_is_up(
        model_urdf: str) -> None:
    """开：unit 确认 running **之后**才切接线。

    切早了，命令会在「接线已指叠加节点、节点还没起」的窗口里落进没人订阅的话题。
    """
    systemd = _all_absent()
    route = _RouteClient()
    node = _shell(systemd, urdf_path=model_urdf, route=route)
    route.probe = lambda: node.feedforward.status()['state']

    response = BenchManager._on_start_gravity_feedforward(
        node, _start_request(), _Response())

    assert response.ok
    assert route.topics == [OVERLAY_COMMAND_TOPIC]
    assert route.seen_states == ['running'], '改道请求发出时 unit 还不是 running'
    assert json.loads(response.status_json)['gravity_feedforward'][
        'command_topic'] == OVERLAY_COMMAND_TOPIC


def test_stop_switches_the_route_back_before_stopping_the_unit(
        model_urdf: str) -> None:
    """关：**先**切回直连、**确认成功**再停 unit。

    反过来，命令会在「接线还指着叠加节点、节点已经没了」的窗口里全部落空 → 固件
    命令超时软停，而机构没有抱闸。
    """
    systemd = _all_absent()
    route = _RouteClient(defer=True)
    node = _shell(systemd, urdf_path=model_urdf, route=route)
    BenchManager._on_start_gravity_feedforward(
        node, _start_request(), _Response())
    route.fire()  # 收尾开启那一步的改道

    BenchManager._on_stop_gravity_feedforward(node, None, _Response())

    assert route.topics[-1] == DIRECT_COMMAND_TOPIC
    # 回执还没到：unit 必须还活着
    assert ['systemctl', '--user', 'stop', UNIT_GRAVITY_FEEDFORWARD]         not in systemd.commands(), '改道没确认就把 unit 停了'

    route.fire()
    assert ['systemctl', '--user', 'stop', UNIT_GRAVITY_FEEDFORWARD]         in systemd.commands()


def test_failed_route_switch_keeps_the_unit_running_on_start(
        model_urdf: str) -> None:
    """切不过去**不回滚** unit：命令继续走直连是有命令、只是没补偿（安全方向），
    回滚反而多一次断流。失败要留在状态里，页面才看得见。"""
    systemd = _all_absent()
    node = _shell(systemd, urdf_path=model_urdf,
                  route=_RouteClient(successful=False, reason='参数被拒'))

    response = BenchManager._on_start_gravity_feedforward(
        node, _start_request(), _Response())

    assert response.ok
    ff = json.loads(response.status_json)['gravity_feedforward']
    assert ff['state'] == 'running'
    assert ff['command_topic'] is None
    assert '参数被拒' in ff['route_error']


def test_failed_route_switch_blocks_stopping_the_unit(model_urdf: str) -> None:
    """切不回来就**不许**停 unit——那正是「命令落进没人订阅的话题」那条路。"""
    systemd = _all_absent()
    route = _RouteClient()
    node = _shell(systemd, urdf_path=model_urdf, route=route)
    BenchManager._on_start_gravity_feedforward(
        node, _start_request(), _Response())

    route.successful = False
    route.reason = '参数被拒'
    response = BenchManager._on_stop_gravity_feedforward(node, None, _Response())

    assert ['systemctl', '--user', 'stop', UNIT_GRAVITY_FEEDFORWARD]         not in systemd.commands()
    ff = json.loads(response.status_json)['gravity_feedforward']
    assert ff['state'] == 'running'
    assert '参数被拒' in ff['route_error']


def test_unreachable_streamer_is_reported_not_silently_skipped(
        model_urdf: str) -> None:
    """命令节拍节点没在跑（页面那条路整个下不了命令）时要说出来，不能静默跳过改道。"""
    node = _shell(_all_absent(), urdf_path=model_urdf,
                  route=_RouteClient(ready=False))

    response = BenchManager._on_start_gravity_feedforward(
        node, _start_request(), _Response())

    ff = json.loads(response.status_json)['gravity_feedforward']
    assert 'command_streamer' in ff['route_error']


def test_start_gravity_feedforward_rejects_before_touching_systemd() -> None:
    """解析不出 URDF 就当场拒：unit 起来再退出，页面只看到一条没头没尾的 failed。"""
    systemd = _all_absent()
    node = _shell(systemd)  # 没有模型可解析

    response = BenchManager._on_start_gravity_feedforward(
        node, _start_request(), _Response())

    assert not response.ok
    assert 'urdf' in response.error
    assert not _creations(systemd)


def test_stop_gravity_feedforward_stops_the_unit(model_urdf: str) -> None:
    systemd = _all_absent()
    node = _shell(systemd, urdf_path=model_urdf)
    BenchManager._on_start_gravity_feedforward(
        node, _start_request(), _Response())

    response = BenchManager._on_stop_gravity_feedforward(node, None, _Response())

    assert response.ok
    assert ['systemctl', '--user', 'stop', UNIT_GRAVITY_FEEDFORWARD] \
        in systemd.commands()
    assert json.loads(response.status_json)[
        'gravity_feedforward']['state'] == 'stopped'


def test_set_gain_requires_a_running_overlay() -> None:
    """没在跑就别写：参数服务根本不存在，静默发出去会让界面显示「已改」。"""
    systemd = _all_absent()
    written: list[float] = []
    node = _shell(systemd, written=written)

    class _Request:
        effort_gain = 0.5

    response = BenchManager._on_set_gravity_feedforward_gain(
        node, _Request(), _Response())

    assert not response.ok
    assert 'not running' in response.error
    assert written == []


def test_set_gain_defers_to_the_confirmation_callback(model_urdf: str) -> None:
    """服务返回 ok 只代表「写请求发出去了」，状态里的增益等回执确认才变。"""
    systemd = _all_absent()
    written: list[float] = []
    node = _shell(systemd, urdf_path=model_urdf, written=written)
    BenchManager._on_start_gravity_feedforward(
        node, _start_request(), _Response())

    class _Request:
        effort_gain = 0.25

    response = BenchManager._on_set_gravity_feedforward_gain(
        node, _Request(), _Response())

    assert response.ok
    assert written == [0.25]
    assert json.loads(response.status_json)['gravity_feedforward'][
        'config']['effort_gain'] == 1.0

    node.feedforward.note_gain_applied(0.25)
    assert json.loads(node._snapshot_json())['gravity_feedforward'][
        'config']['effort_gain'] == 0.25
