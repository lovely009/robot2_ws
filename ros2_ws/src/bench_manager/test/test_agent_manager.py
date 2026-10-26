"""`AgentProcessManager` 的单测。

不启任何真 unit、不碰真串口：systemd 走 `conftest.FakeSystemd`，`/proc` 走注入的假根。
"""

from __future__ import annotations

import os

import pytest

from conftest import FakeSystemd

from bench_manager.agent_manager import (
    DEVICE_ENV,
    AgentProcessManager,
    enumerate_serial_ports,
)
from bench_manager.systemd_units import (
    AUTO_STARTED_ENV, UNIT_AGENT, SystemdUnits,
)

PORT = '/dev/serial/by-id/usb-controller'
OTHER = '/dev/serial/by-id/usb-second'


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


def manager(systemd: FakeSystemd, ports=(PORT,), proc_root='/proc',
            monkeypatch: pytest.MonkeyPatch | None = None) -> AgentProcessManager:
    if monkeypatch is not None:
        monkeypatch.setattr(os, 'access', lambda _path, _mode: True)
    return AgentProcessManager(
        port_provider=lambda: [{'path': p, 'label': os.path.basename(p)} for p in ports],
        systemd=SystemdUnits(runner=systemd), proc_root=proc_root)


def test_start_creates_the_agent_unit_on_the_enumerated_port(monkeypatch):
    systemd = FakeSystemd({UNIT_AGENT: {'LoadState': 'not-found'}})

    status = manager(systemd, monkeypatch=monkeypatch).start(PORT)

    create = next(c for c in systemd.commands() if c[0] == 'systemd-run')
    assert create[:3] == ['systemd-run', '--user', f'--unit={UNIT_AGENT}']
    assert create[-7:] == ['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
                           'serial', '--dev', PORT]
    assert status['state'] == 'running'
    assert status['device'] == PORT


def test_start_rejects_unlisted_device_without_creating_a_unit(monkeypatch):
    systemd = FakeSystemd()

    with pytest.raises(ValueError, match='available serial port'):
        manager(systemd, monkeypatch=monkeypatch).start('/dev/ttyACM1')

    assert not any(c[0] == 'systemd-run' for c in systemd.commands())


def test_start_adopts_the_same_device_and_rejects_a_second_port(monkeypatch):
    """同设备＝认领（不重启）；换设备＝拒绝。

    认领是 systemd 免费给的单写者所有权：unit 在跑就意味着有人管着，重启它只会
    打断正在跑的会话。
    """
    systemd = FakeSystemd({UNIT_AGENT: {
        'LoadState': 'loaded', 'ActiveState': 'active', 'SubState': 'running',
        'Environment': f'{DEVICE_ENV}={PORT}'}})
    agent = manager(systemd, ports=(PORT, OTHER), monkeypatch=monkeypatch)

    assert agent.start(PORT)['device'] == PORT
    assert not any(c[0] == 'systemd-run' for c in systemd.commands())

    with pytest.raises(RuntimeError, match='already running'):
        agent.start(OTHER)


def test_stop_stops_the_agent_unit(monkeypatch):
    systemd = FakeSystemd()
    agent = manager(systemd, monkeypatch=monkeypatch)

    agent.stop()

    assert ['systemctl', '--user', 'stop', UNIT_AGENT] in systemd.commands()


def test_start_refuses_a_device_another_process_already_holds(tmp_path, monkeypatch):
    """孤儿进程还占着 tty 时必须拒绝，而不是再起一个。

    串口没有 O_EXCL：两个进程能同时打开同一个 tty 互相抢字节。上一次会话留下的
    agent（父进程已死、它成了孤儿）占着设备时，新起的 bench_manager 一无所知、
    照起第二个——这就是实测踩到的那个坑。
    """
    device = tmp_path / 'dev' / 'ttyACM1'
    device.parent.mkdir(parents=True)
    device.touch()
    holder = tmp_path / '4242' / 'fd'
    holder.mkdir(parents=True)
    (holder / '3').symlink_to(device)

    systemd = FakeSystemd({UNIT_AGENT: {'LoadState': 'not-found'}})
    agent = manager(systemd, ports=(str(device),), proc_root=str(tmp_path),
                    monkeypatch=monkeypatch)

    with pytest.raises(RuntimeError, match='4242'):
        agent.start(str(device))
    assert not any(c[0] == 'systemd-run' for c in systemd.commands())


def test_status_recovers_the_device_from_the_unit_after_a_restart(monkeypatch):
    """设备从 unit 的环境里读回来——bench_manager 重启后页面仍说得清在跑什么。

    以前 device 活在 manager 的内存里，进程一换就只剩「在跑」两个字。
    """
    systemd = FakeSystemd({UNIT_AGENT: {
        'LoadState': 'loaded', 'ActiveState': 'active', 'SubState': 'running',
        'Environment': f'{DEVICE_ENV}={PORT} {AUTO_STARTED_ENV}=1'}})

    status = manager(systemd, monkeypatch=monkeypatch).status()

    assert status['device'] == PORT
    assert status['auto_started'] is True


def test_failed_unit_reports_the_exit_code(monkeypatch):
    systemd = FakeSystemd({UNIT_AGENT: {
        'LoadState': 'loaded', 'ActiveState': 'failed',
        'SubState': 'failed', 'ExecMainStatus': '3'}})

    status = manager(systemd, monkeypatch=monkeypatch).status()

    assert status['state'] == 'failed'
    assert status['returncode'] == 3


def test_absent_unit_reports_stopped_without_a_device(monkeypatch):
    systemd = FakeSystemd({UNIT_AGENT: {'LoadState': 'not-found'}})

    status = manager(systemd, monkeypatch=monkeypatch).status()

    assert status == {'state': 'stopped', 'device': '', 'returncode': None,
                      'auto_started': False}
