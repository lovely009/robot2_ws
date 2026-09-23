import os
import signal
import subprocess

import pytest

from web_host.agent_manager import AgentProcessManager, enumerate_serial_ports


def test_enumerate_serial_ports_prefers_stable_by_id_names(tmp_path):
    device_root = tmp_path / 'dev'
    by_id = device_root / 'serial' / 'by-id'
    by_id.mkdir(parents=True)
    acm = device_root / 'ttyACM0'
    usb = device_root / 'ttyUSB0'
    acm.touch()
    usb.touch()
    (by_id / 'usb-controller').symlink_to('../../ttyACM0')

    ports = enumerate_serial_ports(str(device_root))

    assert ports == [
        {'path': str(by_id / 'usb-controller'),
         'label': f'usb-controller ({acm})'},
        {'path': str(usb), 'label': f'ttyUSB0 ({usb})'},
    ]


class FakeProcess:
    pid = 1234

    def __init__(self):
        self.returncode = None

    def poll(self):
        return self.returncode

    def wait(self, timeout=None):
        self.returncode = -signal.SIGTERM
        return self.returncode


def test_start_launches_fixed_agent_for_enumerated_port(monkeypatch):
    port = '/dev/serial/by-id/usb-controller'
    popen_calls = []

    def fake_popen(command, **kwargs):
        popen_calls.append((command, kwargs))
        return FakeProcess()

    monkeypatch.setattr(os, 'access', lambda _path, _mode: True)
    manager = AgentProcessManager(
        port_provider=lambda: [{'path': port, 'label': 'controller'}],
        popen_factory=fake_popen,
    )

    status = manager.start(port)

    assert status == {'state': 'running', 'device': port, 'returncode': None}
    assert popen_calls == [(
        ['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
         'serial', '--dev', port],
        {'stdin': subprocess.DEVNULL, 'start_new_session': True},
    )]


def test_start_rejects_unlisted_device_without_spawning(monkeypatch):
    launched = False

    def fake_popen(*_args, **_kwargs):
        nonlocal launched
        launched = True
        return FakeProcess()

    manager = AgentProcessManager(
        port_provider=lambda: [
            {'path': '/dev/ttyACM0', 'label': 'controller'}],
        popen_factory=fake_popen,
    )

    with pytest.raises(ValueError, match='available serial port'):
        manager.start('/dev/ttyACM1')

    assert not launched


def test_start_reuses_same_agent_and_rejects_a_second_port(monkeypatch):
    first = '/dev/ttyACM0'
    second = '/dev/ttyACM1'
    popen_count = 0

    def fake_popen(*_args, **_kwargs):
        nonlocal popen_count
        popen_count += 1
        return FakeProcess()

    monkeypatch.setattr(os, 'access', lambda _path, _mode: True)
    manager = AgentProcessManager(
        port_provider=lambda: [
            {'path': first, 'label': 'controller 0'},
            {'path': second, 'label': 'controller 1'},
        ],
        popen_factory=fake_popen,
    )

    assert manager.start(first)['device'] == first
    assert manager.start(first)['device'] == first
    with pytest.raises(RuntimeError, match='already running'):
        manager.start(second)
    assert popen_count == 1


def test_stop_terminates_managed_agent_process_group(monkeypatch):
    port = '/dev/ttyACM0'
    process = FakeProcess()
    kill_calls = []

    monkeypatch.setattr(os, 'access', lambda _path, _mode: True)
    monkeypatch.setattr(
        os, 'killpg', lambda pid, sig: kill_calls.append((pid, sig)))
    manager = AgentProcessManager(
        port_provider=lambda: [{'path': port, 'label': 'controller'}],
        popen_factory=lambda *_args, **_kwargs: process,
    )
    manager.start(port)

    status = manager.stop()

    assert kill_calls == [(process.pid, signal.SIGTERM)]
    assert status['state'] == 'stopped'
    assert status['device'] == port
