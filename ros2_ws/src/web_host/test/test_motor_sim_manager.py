"""`MotorSimProcessManager` 与联动规则的测试（电机后端：web 上位机独立进程）。

同 test_simulation_manager：不产生真实子进程（fake popen 注入），不碰
CAN 接口与串口；socket 通信用临时目录里的真实 Unix socket 端到端。
"""

from __future__ import annotations

import json
import socket
import threading
from pathlib import Path
from typing import Any

import pytest

from web_host.motor_sim_manager import (
    MotorSimProcessManager,
    hil_motor_backend_request,
    query_motor_sim,
)


class FakeProcess:
    pid = 4321

    def __init__(self) -> None:
        self.returncode: int | None = None
        self.stdout = None

    def poll(self) -> int | None:
        return self.returncode

    def wait(self, timeout: float | None = None) -> int:
        self.returncode = -2  # SIGINT
        return self.returncode


def _manager(
    spawned: list[tuple[list[str], dict[str, Any]]],
    can: tuple[dict[str, Any], ...] = (),
    serial: tuple[dict[str, Any], ...] = (),
    models: tuple[str, ...] = ('half_body_robot',),
) -> MotorSimProcessManager:
    def fake_popen(command: list[str], **kwargs: Any) -> FakeProcess:
        spawned.append((command, kwargs))
        return FakeProcess()

    return MotorSimProcessManager(
        can_provider=lambda: list(can),
        serial_provider=lambda: list(serial),
        model_provider=lambda: list(models),
        popen_factory=fake_popen,
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
# 规则（2026-10-14 与用户对齐）：仅 STM32 固件 HIL + 物理总线默认拉起电机
# 后端；vcan 与其他控制端模式的总线电机是软件 motor_sim 节点，不涉及。

def test_hil_backend_request_rules() -> None:
    assert hil_motor_backend_request(None) is None
    assert hil_motor_backend_request(
        {'controller_mode': 'host', 'transport': 'slcan', 'sim_tty': 'x'}) is None
    assert hil_motor_backend_request(
        {'controller_mode': 'firmware', 'transport': 'vcan'}) is None
    slcan = hil_motor_backend_request({
        'controller_mode': 'firmware', 'transport': 'slcan',
        'sim_tty': 'x', 'model': 'm'})
    assert slcan == {
        'model': 'm', 'transport': 'slcan',
        'physical_bus_isolated': True, 'sim_tty': 'x'}
    socketcan = hil_motor_backend_request({
        'controller_mode': 'firmware', 'transport': 'socketcan',
        'sim_interface': 'can0', 'model': 'm'})
    assert socketcan == {
        'model': 'm', 'transport': 'socketcan',
        'physical_bus_isolated': True, 'sim_interface': 'can0'}


# --- MotorSimProcessManager：校验与启动命令 ---------------------------------

def test_start_builds_simulator_only_launch(canable: str,
                                            monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr('web_host.motor_sim_manager.os.getpid', lambda: 65001)
    spawned: list[tuple[list[str], dict[str, Any]]] = []
    manager = _manager(spawned, serial=_serial(canable))

    status = manager.start(_request(sim_tty=canable))

    command, kwargs = spawned[0]
    assert command[:3] == ['ros2', 'launch', 'robot_sim']
    assert 'simulation.launch.py' in command
    assert 'simulator_only:=true' in command
    assert 'controller_mode:=firmware' in command
    assert 'transport:=slcan' in command
    assert f'sim_tty:={canable}' in command
    assert 'start_web_host:=false' in command
    assert 'start_micro_ros_agent:=false' in command
    assert 'sim_control_socket:=/tmp/web_host_motor_backend_65001.sock' in command
    assert kwargs['start_new_session'] is True
    assert status['state'] == 'running'
    assert status['config']['sim_tty'] == canable
    assert status['auto_started'] is False


def test_socketcan_backend_uses_interface(canable: str) -> None:
    spawned: list[tuple[list[str], dict[str, Any]]] = []
    manager = _manager(spawned, can=({'name': 'can0', 'up': True},))

    manager.start(_request(transport='socketcan', sim_interface='can0',
                           sim_tty=None))

    command, _ = spawned[0]
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
    spawned: list[tuple[list[str], dict[str, Any]]] = []
    manager = _manager(spawned, serial=_serial(canable))
    request = _request(sim_tty=canable)
    request.update(overrides)

    with pytest.raises(ValueError, match=message):
        manager.start(request)

    assert spawned == []


def test_validation_rejects_down_socketcan_interface() -> None:
    spawned: list[tuple[list[str], dict[str, Any]]] = []
    manager = _manager(spawned, can=({'name': 'can0', 'up': False},))

    with pytest.raises(ValueError, match='down'):
        manager.start(_request(transport='socketcan', sim_interface='can0',
                               sim_tty=None))

    assert spawned == []


def test_same_config_is_reused_and_conflict_rejected(
        canable: str, tmp_path: Path) -> None:
    other = tmp_path / 'other-tty'
    other.write_text('')
    spawned: list[tuple[list[str], dict[str, Any]]] = []
    manager = _manager(spawned, serial=_serial(canable) + _serial(str(other)))

    first = manager.start(_request(sim_tty=canable))
    again = manager.start(_request(sim_tty=canable))

    assert len(spawned) == 1
    assert again == first

    with pytest.raises(RuntimeError, match='different configuration'):
        manager.start(_request(sim_tty=str(other)))
    assert len(spawned) == 1


def test_auto_started_flag_is_not_stolen_on_reuse(
        canable: str, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr('web_host.motor_sim_manager.os.getpid', lambda: 65002)
    spawned: list[tuple[list[str], dict[str, Any]]] = []
    manager = _manager(spawned, serial=_serial(canable))
    request = _request(sim_tty=canable)

    # 用户手动起（auto=False）；仿真启动时以 auto=True 复用 → 归属仍属用户
    assert manager.start(request)['auto_started'] is False
    assert manager.start(request, auto=True)['auto_started'] is False

    # 反过来：随仿真起（auto=True）→ 状态里带标记，供停止联动判断
    spawned2: list[tuple[list[str], dict[str, Any]]] = []
    manager2 = _manager(spawned2, serial=_serial(canable))
    assert manager2.start(request, auto=True)['auto_started'] is True


def test_stop_removes_control_socket_file(
        canable: str, monkeypatch: pytest.MonkeyPatch) -> None:
    # FakeProcess 的 pid 是假的；别让 stop 真的 killpg 到无关进程组
    monkeypatch.setattr('web_host.motor_sim_manager.os.killpg',
                        lambda *args, **kwargs: None)
    monkeypatch.setattr('web_host.motor_sim_manager.os.getpid', lambda: 65003)
    spawned: list[tuple[list[str], dict[str, Any]]] = []
    manager = _manager(spawned, serial=_serial(canable))
    status = manager.start(_request(sim_tty=canable), auto=True)
    path = Path(status['config']['motor_sim_control_socket'])
    path.write_text('')  # 模拟器异常退出留下的死 socket 文件
    try:
        stopped = manager.stop()
        assert not path.exists()
        assert stopped['auto_started'] is False
    finally:
        path.unlink(missing_ok=True)


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
    manager = _manager([])
    with pytest.raises(RuntimeError, match='未运行'):
        manager.motor_request({'cmd': 'status'})


def test_motor_request_forwards_through_manager(
        canable: str, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr('web_host.motor_sim_manager.os.getpid', lambda: 65004)
    spawned: list[tuple[list[str], dict[str, Any]]] = []
    manager = _manager(spawned, serial=_serial(canable))
    manager.start(_request(sim_tty=canable))
    socket_path = '/tmp/web_host_motor_backend_65004.sock'
    seen: list[dict[str, Any]] = []
    response = {'ok': True, 'motors': [{'id': 1, 'linked': True}]}
    server, thread = _serve_control_once(socket_path, response, seen)
    try:
        assert manager.motor_request({'cmd': 'status'}) == response
    finally:
        thread.join(timeout=2)
        server.close()
        Path(socket_path).unlink(missing_ok=True)
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
