"""Enumerate controller serial devices and supervise one micro-ROS Agent."""

import glob
import os
import re
import signal
import subprocess
import threading


_SUPPORTED_TTY = re.compile(r'tty(?:ACM|USB)\d+$')


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
    """Own at most one Agent child process and constrain it to listed ports."""

    def __init__(self, port_provider=enumerate_serial_ports,
                 popen_factory=None, stop_timeout_s=5):
        self._port_provider = port_provider
        self._popen_factory = popen_factory or subprocess.Popen
        self._stop_timeout_s = stop_timeout_s
        self._lock = threading.RLock()
        self._process = None
        self._device = None
        self._last_status = {
            'state': 'stopped', 'device': '', 'returncode': None,
        }

    def _status_locked(self):
        if self._process is None:
            return dict(self._last_status)

        returncode = self._process.poll()
        if returncode is not None:
            self._last_status = {
                'state': 'failed' if returncode else 'stopped',
                'device': self._device or '',
                'returncode': returncode,
            }
            self._process = None
            self._device = None
            return dict(self._last_status)

        return {
            'state': 'running', 'device': self._device or '',
            'returncode': None,
        }

    def status(self):
        with self._lock:
            return self._status_locked()

    def start(self, device):
        """Start Agent on an enumerated device, or reuse this process."""
        with self._lock:
            status = self._status_locked()
            if status['state'] == 'running':
                if self._device == device:
                    return status
                raise RuntimeError(
                    'Agent is already running on %s; stop it before '
                    'switching ports' % self._device)

            available = {item['path'] for item in self._port_provider()}
            if device not in available:
                raise ValueError('device is not an available serial port')
            if not os.access(device, os.R_OK | os.W_OK):
                raise PermissionError(
                    'web_host cannot read and write %s; grant serial access '
                    '(for example, add its user to dialout)' % device)

            command = [
                'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
                'serial', '--dev', device,
            ]
            self._process = self._popen_factory(
                command, stdin=subprocess.DEVNULL, start_new_session=True)
            self._device = device
            self._last_status = {
                'state': 'running', 'device': device, 'returncode': None,
            }
            return dict(self._last_status)

    def stop(self):
        """Stop only the Agent process started by this manager."""
        with self._lock:
            status = self._status_locked()
            process = self._process
            device = self._device or status['device']
            if process is None:
                return status

            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass

            try:
                returncode = process.wait(timeout=self._stop_timeout_s)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                returncode = process.wait()

            self._process = None
            self._device = None
            self._last_status = {
                'state': 'stopped', 'device': device or '',
                'returncode': returncode,
            }
            return dict(self._last_status)
