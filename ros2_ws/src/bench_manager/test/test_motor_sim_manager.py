"""`MotorSimProcessManager` 与联动规则的测试（电机后端：`bench-motor-backend.service`）。

同 test_simulation_manager：不产生真实进程、不启任何 unit（systemd 走
`conftest.FakeSystemd`），不碰 CAN 接口与串口；socket 通信用临时目录里的真实
Unix socket 端到端。
"""

from __future__ import annotations

import json
import socket
import threading
from pathlib import Path
from typing import Any

import pytest

from conftest import FakeSystemd

from bench_manager.motor_sim_manager import (
    MotorSimProcessManager,
    hil_motor_backend_request,
    query_motor_sim,
)
from bench_manager.systemd_units import (
    AUTO_STARTED_ENV, UNIT_MOTOR_BACKEND, SystemdUnits,
)


def _systemd() -> FakeSystemd:
    return FakeSystemd({UNIT_MOTOR_BACKEND: {'LoadState': 'not-found'}})


def _creations(systemd: FakeSystemd) -> list[list[str]]:
    return [call for call in systemd.commands() if call[0] == 'systemd-run']


def _exec_start(systemd: FakeSystemd) -> list[str]:
    create = _creations(systemd)[0]
    return create[create.index('ros2'):]


def _manager(
    systemd: FakeSystemd,
    can: tuple[dict[str, Any], ...] = (),
    serial: tuple[dict[str, Any], ...] = (),
    models: tuple[str, ...] = ('half_body_robot',),
) -> MotorSimProcessManager:
    return MotorSimProcessManager(
        can_provider=lambda: list(can),
        serial_provider=lambda: list(serial),
        model_provider=lambda: list(models),
        systemd=SystemdUnits(runner=systemd),
    )


def _request(**overrides: Any) -> dict[str, Any]:
    request: dict[str, Any] = {
        'model': 'half_body_robot',
        'transport': 'slcan',
        'sim_tty': '',
        'physical_bus_isolated': True,
    }
    request.update(overrides)
    return request


@pytest.fixture
def canable(tmp_path: Path) -> str:
    path = tmp_path / 'canable-tty'
    path.write_text('')
    return str(path)


def _serial(path: str) -> tuple[dict[str, Any], ...]:
    return ({'path': path, 'label': path},)


# --- hil_motor_backend_request：默认拉起规则 --------------------------------
#
# 仅 STM32 固件 HIL + 物理总线默认拉起电机
# 后端；vcan 与其他控制端模式的总线电机是软件 motor_sim 节点，不涉及。

def test_hil_backend_request_rules() -> None:
    assert hil_motor_backend_request(None) is None
    assert hil_motor_backend_request(
        {'controller_mode': 'host', 'transport': 'slcan', 'sim_tty': 'x'}) is None
    assert hil_motor_backend_request(
        {'controller_mode': 'firmware', 'transport': 'vcan'}) is None
    slcan = hil_motor_backend_request({
        'controller_mode': 'firmware', 'transport': 'slcan',
        'sim_tty': 'x', 'model': 'm', 'physical_bus_isolated': True})
    assert slcan == {
        'model': 'm', 'transport': 'slcan',
        'physical_bus_isolated': True, 'sim_tty': 'x'}
    socketcan = hil_motor_backend_request({
        'controller_mode': 'firmware', 'transport': 'socketcan',
        'sim_interface': 'can0', 'model': 'm', 'physical_bus_isolated': True})
    assert socketcan == {
        'model': 'm', 'transport': 'socketcan',
        'physical_bus_isolated': True, 'sim_interface': 'can0'}


def test_hil_backend_request_carries_the_confirmation_verbatim() -> None:
    """确认位与仿真请求同源，本函数不再写死 True。

    写死会让未确认时后端先起、再被仿真的校验拒掉后回滚——白起停一次进程，而回滚
    只兜 OSError。透传则后端就地拒绝，报的还是同一条原因（见 motor_sim_manager
    的 _validate）。
    """
    request = {'controller_mode': 'firmware', 'transport': 'socketcan',
               'sim_interface': 'can0', 'model': 'm'}

    assert hil_motor_backend_request(request)['physical_bus_isolated'] is False

    request['physical_bus_isolated'] = True
    assert hil_motor_backend_request(request)['physical_bus_isolated'] is True


# --- MotorSimProcessManager：校验与启动命令 ---------------------------------

def test_start_builds_simulator_only_launch(canable: str) -> None:
    systemd = _systemd()
    manager = _manager(systemd, serial=_serial(canable))

    status = manager.start(_request(sim_tty=canable))

    command = _exec_start(systemd)
    assert command[:3] == ['ros2', 'launch', 'robot_sim']
    assert 'simulation.launch.py' in command
    assert 'simulator_only:=true' in command
    assert 'controller_mode:=firmware' in command
    assert 'transport:=slcan' in command
    assert f'sim_tty:={canable}' in command
    assert 'start_web_host:=false' in command
    assert 'start_micro_ros_agent:=false' in command
    # 固定路径：unit 化之后不能再带 bench_manager 的 pid（重启就变）
    assert 'sim_control_socket:=/tmp/robot_sim_motor_backend.sock' in command
    assert status['state'] == 'running'
    assert status['config']['sim_tty'] == canable
    assert status['auto_started'] is False


def test_socketcan_backend_uses_interface(canable: str) -> None:
    systemd = _systemd()
    manager = _manager(systemd, can=({'name': 'can0', 'up': True},))

    manager.start(_request(transport='socketcan', sim_interface='can0',
                           sim_tty=None))

    command = _exec_start(systemd)
    assert 'sim_interface:=can0' in command
    assert not any(a.startswith('sim_tty:=') for a in command)


@pytest.mark.parametrize('overrides, message', [
    ({'transport': 'vcan'}, 'vcan'),
    ({'model': 'nope'}, 'topology'),
    ({'physical_bus_isolated': False}, 'physical'),
    ({'sim_tty': '/dev/does-not-exist'}, 'serial'),
])
def test_validation_rejects_bad_requests(canable: str, overrides: Any,
                                         message: str) -> None:
    systemd = _systemd()
    manager = _manager(systemd, serial=_serial(canable))
    request = _request(sim_tty=canable)
    request.update(overrides)

    with pytest.raises(ValueError, match=message):
        manager.start(request)

    assert _creations(systemd) == []


def test_validation_rejects_down_socketcan_interface() -> None:
    systemd = _systemd()
    manager = _manager(systemd, can=({'name': 'can0', 'up': False},))

    with pytest.raises(ValueError, match='down'):
        manager.start(_request(transport='socketcan', sim_interface='can0',
                               sim_tty=None))

    assert _creations(systemd) == []


def test_same_config_is_reused_and_conflict_rejected(
        canable: str, tmp_path: Path) -> None:
    other = tmp_path / 'other-tty'
    other.write_text('')
    systemd = _systemd()
    manager = _manager(systemd, serial=_serial(canable) + _serial(str(other)))

    first = manager.start(_request(sim_tty=canable))
    again = manager.start(_request(sim_tty=canable))

    assert len(_creations(systemd)) == 1
    assert again == first

    with pytest.raises(RuntimeError, match='different configuration'):
        manager.start(_request(sim_tty=str(other)))
    assert len(_creations(systemd)) == 1


def test_auto_started_flag_is_not_stolen_on_reuse(canable: str) -> None:
    systemd = _systemd()
    manager = _manager(systemd, serial=_serial(canable))
    request = _request(sim_tty=canable)

    # 用户手动起（auto=False）；仿真启动时以 auto=True 复用 → 归属仍属用户
    assert manager.start(request)['auto_started'] is False
    assert manager.start(request, auto=True)['auto_started'] is False

    # 反过来：随仿真起（auto=True）→ 状态里带标记，供停止联动判断
    systemd2 = _systemd()
    manager2 = _manager(systemd2, serial=_serial(canable))
    assert manager2.start(request, auto=True)['auto_started'] is True


def test_auto_started_survives_a_bench_manager_restart() -> None:
    """归属标记存在 unit 里，不在内存里——重启后停仿真仍敢停它。"""
    systemd = FakeSystemd({UNIT_MOTOR_BACKEND: {
        'LoadState': 'loaded', 'ActiveState': 'active', 'SubState': 'running',
        'Environment': f'{AUTO_STARTED_ENV}=1'}})

    assert _manager(systemd).status()['auto_started'] is True


def test_stop_stops_the_backend_unit(canable: str) -> None:
    systemd = _systemd()
    manager = _manager(systemd, serial=_serial(canable))
    manager.start(_request(sim_tty=canable), auto=True)

    status = manager.stop()

    assert ['systemctl', '--user', 'stop', UNIT_MOTOR_BACKEND] in systemd.commands()
    assert status['state'] == 'stopped'
    assert status['auto_started'] is False
    assert status['config'] is None


# --- 控制通道转发（/api/motor-sim/status|command 的底座） -------------------

def _serve_control_once(
    socket_path: str,
    response: dict[str, Any],
    seen: list[dict[str, Any]],
    close_immediately: bool = False,
) -> tuple[socket.socket, threading.Thread]:
    """起一个只应答一次的假模拟器控制端，返回 (server, thread)。"""
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(socket_path)
    server.listen(1)

    def serve() -> None:
        conn, _ = server.accept()
        if close_immediately:
            # 读走请求后静默关闭（不回响应）：query 侧必须报错而不是挂死
            data = b''
            while b'\n' not in data:
                chunk = conn.recv(4096)
                if not chunk:
                    break
                data += chunk
            conn.close()
            return
        with conn:
            data = b''
            while b'\n' not in data:
                chunk = conn.recv(4096)
                if not chunk:
                    break
                data += chunk
            seen.append(json.loads(data.split(b'\n', 1)[0]))
            conn.sendall(json.dumps(response).encode() + b'\n')

    thread = threading.Thread(target=serve, daemon=True)
    thread.start()
    return server, thread


def test_motor_request_requires_running_backend() -> None:
    manager = _manager(_systemd())
    with pytest.raises(RuntimeError, match='未运行'):
        manager.motor_request({'cmd': 'status'})


def test_motor_request_forwards_through_manager(
        canable: str, tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    # 控制通道路径是模块常量、指向 /tmp 上的固定名——台架上真跑着 HIL 时它会被人占着，
    # 所以这里改指临时目录，测试不碰台架状态。
    socket_path = str(tmp_path / 'motor-backend.sock')
    monkeypatch.setattr(
        'bench_manager.motor_sim_manager._CONTROL_SOCKET', socket_path)
    systemd = _systemd()
    manager = _manager(systemd, serial=_serial(canable))
    manager.start(_request(sim_tty=canable))
    seen: list[dict[str, Any]] = []
    response = {'ok': True, 'motors': [{'id': 1, 'linked': True}]}
    server, thread = _serve_control_once(socket_path, response, seen)
    try:
        assert manager.motor_request({'cmd': 'status'}) == response
    finally:
        thread.join(timeout=2)
        server.close()
    assert seen == [{'cmd': 'status'}]


def test_query_motor_sim_roundtrip(tmp_path: Path) -> None:
    socket_path = str(tmp_path / 'sim.sock')
    seen: list[dict[str, Any]] = []
    response = {'ok': True, 'motors': [{'id': 1, 'linked': True}]}
    server, thread = _serve_control_once(socket_path, response, seen)
    try:
        assert query_motor_sim(socket_path, {'cmd': 'add', 'id': 9}) == response
    finally:
        thread.join(timeout=2)
        server.close()
    assert seen == [{'cmd': 'add', 'id': 9}]


def test_query_motor_sim_unreachable_and_silent(tmp_path: Path) -> None:
    # 路径不存在（模拟器没开控制通道）→ 明确报"不可用"
    with pytest.raises(RuntimeError, match='不可用'):
        query_motor_sim(str(tmp_path / 'missing.sock'), {'cmd': 'status'})
    # 连接被静默关闭（模拟器崩溃）→ 报"无响应"，不能挂死
    silent_path = str(tmp_path / 'silent.sock')
    server, thread = _serve_control_once(silent_path, {}, [],
                                         close_immediately=True)
    try:
        with pytest.raises(RuntimeError, match='无响应'):
            query_motor_sim(silent_path, {'cmd': 'status'})
    finally:
        thread.join(timeout=2)
        server.close()
