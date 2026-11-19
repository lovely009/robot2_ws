"""枚举控制器串口并管理 micro-ROS Agent unit。"""

import glob
import os
import re

from .systemd_units import (
    AUTO_STARTED_ENV, STATE_NAMES, UNIT_AGENT, SystemdUnits, UnitState,
)

_SUPPORTED_TTY = re.compile(r'tty(?:ACM|USB)\d+$')

# 设备路径随 unit 持久化，管理器重启后仍可恢复。
DEVICE_ENV = 'BENCH_DEVICE'


def enumerate_serial_ports(dev_root='/dev'):
    """优先返回 by-id 路径，再返回未被别名覆盖的 ACM/USB 串口。"""
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
    """管理 Agent unit，仅允许使用枚举到的串口。"""

    def __init__(self, port_provider=enumerate_serial_ports, systemd=None,
                 proc_root='/proc'):
        self._port_provider = port_provider
        self._systemd = systemd or SystemdUnits()
        self._proc_root = proc_root

    def _holders(self, path):
        """扫描 `/proc/<pid>/fd`，返回占用设备的进程。

        串口无 O_EXCL 互斥，必须检查占用，避免多个 Agent 抢读同一 tty。
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
            'state': STATE_NAMES[unit.state],
            'device': env.get(DEVICE_ENV, ''),
            'returncode': unit.returncode,
            'auto_started': env.get(AUTO_STARTED_ENV) == '1',
        }

    def status(self):
        return self._status()

    def start(self, device, auto=False):
        """在枚举到的设备上启动 Agent；同设备已运行时复用。"""
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
        """停止 Agent unit；未运行时无操作。"""
        self._systemd.stop(UNIT_AGENT)
        return self._status()
