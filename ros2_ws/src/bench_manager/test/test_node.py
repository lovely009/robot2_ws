"""`BenchManager` 节点契约的单测：状态快照形状、lifecycle 门槛、日志服务白名单。

不起 rclpy 上下文——被测的都是纯函数式的回调与派生，用 `__new__` 造出的空壳节点
（跳过 `__init__`）加假 systemd 就够。真起一个节点会拉起 DDS 参与者，而这里要验的
东西跟 DDS 无关。
"""

from __future__ import annotations

import json
import logging

import pytest

from conftest import FakeSystemd

from bench_manager.agent_manager import AgentProcessManager
from bench_manager.motor_sim_manager import MotorSimProcessManager
from bench_manager.node import BenchManager, _simulation_request
from bench_manager.simulation_manager import SimulationProcessManager
from bench_manager.systemd_units import (
    SLICE, UNIT_AGENT, UNIT_MOTOR_BACKEND, UNIT_SIMULATION, SystemdUnits,
)

_UNITS = (UNIT_SIMULATION, UNIT_MOTOR_BACKEND, UNIT_AGENT)


class _Response:
    ok = False
    error = ''
    text = ''


def _shell(systemd: FakeSystemd) -> BenchManager:
    """跳过 `__init__`（那要 rclpy 上下文）造一个节点外壳。"""
    node = BenchManager.__new__(BenchManager)
    node._systemd = SystemdUnits(runner=systemd)
    node.agent = AgentProcessManager(
        port_provider=lambda: [], systemd=node._systemd)
    node.simulation = SimulationProcessManager(
        can_provider=lambda: [], serial_provider=lambda: [],
        model_provider=lambda: [], mujoco_model_provider=lambda: [],
        systemd=node._systemd)
    node.motor_backend = MotorSimProcessManager(
        can_provider=lambda: [], serial_provider=lambda: [],
        model_provider=lambda: [], systemd=node._systemd)
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

def test_snapshot_has_three_sections_and_no_log_key() -> None:
    """日志**不进**快照。

    三个 unit 各留 80 行会把这个每秒都发的 JSON 撑成几十 KB，而页面只在抽屉展开时
    才看它——改成按需拉 `/bench/get_unit_log`。
    """
    node = _shell(_all_absent())

    snapshot = json.loads(node._snapshot_json())

    assert set(snapshot) == {'simulation', 'agent', 'motor_backend'}
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


# --- lifecycle 门槛 ---------------------------------------------------------

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
def test_unit_log_accepts_the_three_contract_units(unit: str) -> None:
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
    """`unit_name` 直接进 `journalctl -u`，只认三个契约名。"""
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
