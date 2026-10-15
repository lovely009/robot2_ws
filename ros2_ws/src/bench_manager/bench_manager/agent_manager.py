"""Enumerate controller serial devices and supervise the micro-ROS Agent unit."""

import glob
import os
import re

from .systemd_units import AUTO_STARTED_ENV, UNIT_AGENT, SystemdUnits, UnitState

_SUPPORTED_TTY = re.compile(r'tty(?:ACM|USB)\d+$')

# 会话描述随 unit 持久化：以前它活在 manager 的内存里，bench_manager 一重启就丢，
# 页面只能显示「在跑」却说不清跑在哪台设备上。
DEVICE_ENV = 'BENCH_DEVICE'

_STATE_NAMES = {
    UnitState.ABSENT: 'stopped',
    UnitState.STOPPED: 'stopped',
    UnitState.RUNNING: 'running',
    UnitState.FAILED: 'failed',
}


def enumerate_serial_ports(dev_root='/dev'):
    """Return stable by-id ports first, then unaliased ACM/USB tty devices."""
    stable_ports = []
    known_devices = set()
    by_id_pattern = os.path.join(dev_root, 'serial', 'by-id', '*')

    for path in sorted(glob.glob(by_id_pattern)):
        if not os.path.exists(path):
            continue
        target = os.path.realpath(path)
        target_name = os.path.basename(target)
        if not _SUPPORTED_TTY.fullmatch(target_name):
            continue
        if target in known_devices:
            continue
        known_devices.add(target)
        stable_ports.append({
            'path': path,
            'label': '%s (%s)' % (os.path.basename(path), target),
        })

    fallback_ports = []
    for pattern in ('ttyACM*', 'ttyUSB*'):
        for path in sorted(glob.glob(os.path.join(dev_root, pattern))):
            if not os.path.exists(path):
                continue
            target = os.path.realpath(path)
            if target in known_devices or not _SUPPORTED_TTY.fullmatch(
                    os.path.basename(target)):
                continue
            known_devices.add(target)
            fallback_ports.append({
                'path': path,
                'label': '%s (%s)' % (os.path.basename(path), target),
            })

    return stable_ports + fallback_ports


class AgentProcessManager:
    """Own the Agent unit, constrained to enumerated ports."""

    def __init__(self, port_provider=enumerate_serial_ports, systemd=None,
                 proc_root='/proc'):
        self._port_provider = port_provider
        self._systemd = systemd or SystemdUnits()
        self._proc_root = proc_root

    def _holders(self, path):
        """谁正开着这个设备。

        串口没有 O_EXCL 语义——两个进程可以同时打开同一个 tty 互相抢字节，所以
        「已经有人占着」只能自己查：扫 `/proc/<pid>/fd` 里指向该设备的符号链接。
        这正是「孤儿 agent 还占着 ttyACM1，新起的 bench_manager 照起第二个」的堵口。
        """
        target = os.path.realpath(path)
        holders = []
        for entry in glob.glob(os.path.join(self._proc_root, '[0-9]*', 'fd', '*')):
            try:
                if os.path.realpath(entry) != target:
                    continue
            except OSError:  # 进程在扫描途中退出：fd 目录消失
                continue
            pid = entry[len(self._proc_root):].split('/')[1]
            holders.append(int(pid))
        return holders

    def _status(self):
        unit = self._systemd.probe(UNIT_AGENT)
        env = ({} if unit.state is UnitState.ABSENT
               else self._systemd.environment(UNIT_AGENT))
        return {
            'state': _STATE_NAMES[unit.state],
            'device': env.get(DEVICE_ENV, ''),
            'returncode': unit.returncode,
            'auto_started': env.get(AUTO_STARTED_ENV) == '1',
        }

    def status(self):
        return self._status()

    def start(self, device, auto=False):
        """Start Agent on an enumerated device, or adopt the running one."""
        status = self._status()
        if status['state'] == 'running':
            if status['device'] == device:
                return status
            raise RuntimeError(
                'Agent is already running on %s; stop it before '
                'switching ports' % status['device'])

        available = {item['path'] for item in self._port_provider()}
        if device not in available:
            raise ValueError('device is not an available serial port')
        if not os.access(device, os.R_OK | os.W_OK):
            raise PermissionError(
                'bench_manager cannot read and write %s; grant serial access '
                '(for example, add its user to dialout)' % device)

        holders = self._holders(device)
        if holders:
            raise RuntimeError(
                '%s 已被进程 %s 占用——多半是上一次会话留下的 agent。先停掉它再起；'
                '同一个 tty 上两个 agent 会互相抢字节。'
                % (device, ', '.join(str(pid) for pid in holders)))

        self._systemd.start(
            UNIT_AGENT,
            ['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
             'serial', '--dev', device],
            auto=auto, session_env={DEVICE_ENV: device})
        return self._status()

    def stop(self):
        """Stop the Agent unit (no-op when it is not running)."""
        self._systemd.stop(UNIT_AGENT)
        return self._status()
