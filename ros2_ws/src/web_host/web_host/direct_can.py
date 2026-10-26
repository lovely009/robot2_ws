"""主机直连参数会话：复用 python-can 传输与 CANopen SDO，只发送 SYNC 与 SDO 上传。"""

from __future__ import annotations

import fcntl
import hashlib
import math
import struct
import secrets
import threading
import time
from pathlib import Path

import can
import canopen
from serial.tools import list_ports


# 对象地址、单位来自 docs/research/cerb-device-reference.md；不开放任意对象写入。
CERB_OBJECTS = (
    ('identity', '设备型号', 0x1008, 0, 'text', ''),
    ('hardware', '硬件版本', 0x1009, 0, 'text', ''),
    ('firmware', '固件版本', 0x100A, 0, 'text', ''),
    ('status', '状态字', 0x6041, 0, 'hex', ''),
    ('position_min', '位置下限', 0x607D, 1, 'signed', 'count'),
    ('position_max', '位置上限', 0x607D, 2, 'signed', 'count'),
    ('speed_max', '设备最大转速', 0x6080, 0, 'unsigned', 'rpm'),
    ('current_max', '设备最大电流', 0x2538, 0, 'unsigned', 'mA'),
    ('protection', '看门狗/限位开关（0 开、1 关）', 0x2650, 0, 'unsigned', ''),
    ('direction', '方向（0 CW、1 CCW）', 0x2541, 0, 'unsigned', ''),
    ('p_scale', 'MIT 位置半量程', 0x2542, 0, 'centi', 'rad'),
    ('v_scale', 'MIT 速度半量程', 0x2543, 0, 'centi', 'rad/s'),
    ('kp_scale', 'MIT Kp 满量程', 0x2544, 0, 'unsigned', 'Nm/rad'),
    ('kd_scale', 'MIT Kd 满量程', 0x2545, 0, 'unsigned', 'Nm·s/rad'),
    ('t_scale', 'MIT 力矩半量程', 0x2546, 0, 'unsigned', 'Nm'),
)


def device_options() -> dict:
    """只枚举本机真实 CAN 接口与串口，不修改接口位率。"""
    interfaces = []
    for path in sorted(Path('/sys/class/net').iterdir()):
        try:
            if path.joinpath('type').read_text().strip() == '280':
                interfaces.append(path.name)
        except OSError:
            continue
    ports = [p.device for p in list_ports.comports()
             if p.vid is not None or p.device.startswith(('/dev/ttyACM', '/dev/ttyUSB'))]
    return {'socketcan': interfaces, 'slcan': sorted(set(ports)),
            'protocols': ['cerb', 'vaxor'], 'read_only': True}


class StandardMessageListener(canopen.network.MessageListener):
    def on_message_received(self, message):
        if not message.is_extended_id:
            super().on_message_received(message)


class FdSdoNetwork(canopen.Network):
    """CERB SDO 使用标准 CAN FD+BRS，SDO 分段事务由 canopen 管理。"""

    def __init__(self, bus=None):
        super().__init__(bus)
        self.listeners = [StandardMessageListener(self)]

    def send_message(self, can_id, data, remote=False):
        # 主动帧仅允许发现 SYNC、SDO 上传/中止，禁止 NMT、下载与运动控制。
        if remote or not ((can_id == 0x080 and len(data) == 0) or
                          (0x601 <= can_id <= 0x67F and len(data) == 8
                           and data[0] in (0x40, 0x60, 0x70, 0x80))):
            raise ValueError('直连通道只允许 SYNC 与 SDO 读取')
        with self.send_lock:
            self.bus.send(can.Message(arbitration_id=can_id, data=data,
                                      is_extended_id=False, is_fd=True,
                                      bitrate_switch=True), timeout=0.2)
        self.check()


class DirectCanSession:
    """单主站、有期限的参数读取会话；所有事务串行执行。"""

    def __init__(self, network_factory=None, options_factory=device_options):
        self._factory = network_factory or self._open_network
        self._options = options_factory
        self._lock = threading.RLock()
        self._network = None
        self._file = None
        self._token = ''
        self._endpoint = ''
        self._transport = ''
        self._last_used = 0.0
        self._foreign_at = 0.0
        self._rx_lock = threading.Lock()
        self._seen = {}
        self._scan_stop = threading.Event()
        self._scan_thread = None
        self._scan_error = ''
        self._closed = threading.Event()
        self._reaper = threading.Thread(target=self._reap, daemon=True)
        self._reaper.start()

    def _reap(self):
        while not self._closed.wait(5):
            with self._lock:
                if self._network is not None and time.monotonic() - self._last_used > 90:
                    self._disconnect()

    @staticmethod
    def _open_network(transport, endpoint):
        network = FdSdoNetwork()
        try:
            if transport == 'socketcan':
                network.connect(interface='socketcan', channel=endpoint, fd=True,
                                receive_own_messages=False)
            else:
                timing = can.BitTimingFd.from_sample_point(
                    f_clock=80000000, nom_bitrate=1000000, nom_sample_point=80,
                    data_bitrate=5000000, data_sample_point=80)
                network.connect(interface='slcan', channel=endpoint, timing=timing,
                                sleep_after_open=0.2)
            return network
        except Exception:
            if network.bus:
                network.disconnect()
            raise

    def _foreign(self, can_id, data, timestamp):
        self._foreign_at = time.monotonic()

    def _feedback(self, protocol, can_id, data, timestamp):
        now = time.monotonic()
        if protocol == 'cerb':
            if len(data) != 12 or not 0x301 <= can_id <= 0x37F:
                return
            node = can_id - 0x300
            values = [('position_raw', '位置原值', int.from_bytes(data[:2], 'big', signed=True), 'count'),
                      ('speed_raw', '速度原值', int.from_bytes(data[2:4], 'big', signed=True), 'rpm'),
                      ('temperature', '温度', int.from_bytes(data[8:10], 'big', signed=True) / 10, '°C'),
                      ('fault', '故障位', bool(data[11] & 0x20), '')]
        else:
            base = 0x702 if len(data) == 20 else 0x701 if len(data) == 12 else 0
            if not base or can_id <= base or (can_id - base) % 4:
                return
            node = (can_id - base) // 4
            if not 1 <= node <= 60:
                return
            position, speed, current = struct.unpack_from('<IHH', data, 2)
            values = [('position', '电机轴位置', (position - 0x80000000) * 2 * math.pi / 2097152, 'rad'),
                      ('speed', '电机轴速度', (speed - 0x8000) * math.pi / 300, 'rad/s'),
                      ('current', '电流', (current - 0x8000) * 11 / 4096, 'A')]
            if len(data) == 20:
                values += [('voltage', '电压', struct.unpack_from('<h', data, 10)[0] / 10, 'V'),
                           ('fault', '故障码', int.from_bytes(data[12:16], 'little'), ''),
                           ('motor_state', '电机状态', data[16], ''),
                           ('calibration', '标定状态', data[17], ''),
                           ('mode', '控制模式', data[18], ''),
                           ('motor_type', '电机类型码', data[19], '')]
        fields = [{'key': key, 'label': label, 'value': value, 'unit': unit,
                   'object': f'0x{can_id:03X} 主动反馈'} for key, label, value, unit in values]
        key = f'{protocol}:{node}'
        with self._rx_lock:
            previous = self._seen.get(key, {})
            # 12B 回显不抹掉先前 20B 状态帧中的设备信息。
            merged = {f['key']: f for f in previous.get('fields', [])}
            merged.update({f['key']: f for f in fields})
            self._seen[key] = {'id': node, 'key': key, 'protocol': protocol,
                               'fields': list(merged.values()), 'last_seen': now,
                               'read_at': time.time()}

    def _scan(self, network, stop):
        while not stop.wait(0.02):
            try:
                network.send_message(0x080, b'')
            except Exception as exc:
                self._scan_error = f'SYNC 探测失败：{exc}'
                return

    def discover(self, token):
        with self._lock:
            self._require(token)
            if self._scan_error:
                raise ValueError(self._scan_error)
            now = time.monotonic()
            with self._rx_lock:
                motors = [dict(m, age_ms=round((now - m['last_seen']) * 1000),
                               online=now - m['last_seen'] <= 0.1)
                          for m in self._seen.values()]
            return {'motors': sorted(motors, key=lambda m: (m['protocol'], m['id'])),
                    'source': 'direct_can', 'read_only': True}

    def connect(self, transport, endpoint):
        with self._lock:
            if self._network is not None:
                raise ValueError('已有直连会话，请先断开原会话')
            if transport not in ('socketcan', 'slcan'):
                raise ValueError('只支持 SocketCAN 或 SLCAN')
            if endpoint not in self._options()[transport]:
                raise ValueError('请选择本机枚举到的 CAN 接口或串口')
            canonical = str(Path(endpoint).resolve()) if transport == 'slcan' else endpoint
            key = hashlib.sha256(f'{transport}:{canonical}'.encode()).hexdigest()
            handle = open(f'/tmp/robot-device-can-{key}.lock', 'a')
            try:
                try:
                    fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError as exc:
                    raise ValueError('该 CAN 端点已被另一个直连会话占用，请先断开原会话') from exc
                network = self._factory(transport, endpoint)
            except Exception:
                handle.close()
                raise
            self._network, self._file = network, handle
            self._transport, self._endpoint = transport, endpoint
            self._token = secrets.token_urlsafe(24)
            self._last_used = time.monotonic()
            self._foreign_at = 0.0
            for node in range(1, 128):
                network.subscribe(0x600 + node, self._foreign)
            with self._rx_lock:
                self._seen = {}
            for node in range(1, 128):
                network.subscribe(0x300 + node, lambda cid, data, stamp: self._feedback('cerb', cid, data, stamp))
            for node in range(1, 61):
                for base in (0x701, 0x702):
                    network.subscribe(base + 4 * node, lambda cid, data, stamp: self._feedback('vaxor', cid, data, stamp))
            self._scan_error = ''
            self._scan_stop = threading.Event()
            self._scan_thread = threading.Thread(target=self._scan, args=(network, self._scan_stop), daemon=True)
            self._scan_thread.start()
            return {'session': self._token, 'transport': transport,
                    'endpoint': endpoint, 'read_only': True}

    def _require(self, token):
        if self._network is None or not secrets.compare_digest(str(token), self._token):
            raise ValueError('直连会话已失效，请重新连接')
        self._last_used = time.monotonic()

    def heartbeat(self, token):
        with self._lock:
            self._require(token)
            return {'connected': True, 'transport': self._transport,
                    'endpoint': self._endpoint}

    def read(self, token, node_ids=None, device_key=None):
        discovered = None
        if node_ids is None:
            discovered = self.discover(token)['motors']
            if device_key is not None:
                discovered = [m for m in discovered if m['key'] == device_key]
                if not discovered:
                    raise ValueError('该电机尚未被发现，请等待总线反馈')
            node_ids = [m['id'] for m in discovered if m['protocol'] == 'cerb' and m['online']]
            if not node_ids:
                return {'motors': discovered, 'source': 'direct_can', 'read_only': True}
        if (not isinstance(node_ids, list) or not node_ids or len(node_ids) > 16
                or any(type(n) is not int or not 1 <= n <= 127 for n in node_ids)):
            raise ValueError('节点 ID 必须是 1～127 的整数列表')
        with self._lock:
            self._require(token)
            # 无第二个 SDO 主站；这里不消费或覆盖固件 facade 的状态。
            time.sleep(0.25)
            if time.monotonic() - self._foreign_at < 2:
                raise ValueError('检测到其他 SDO 主站，请停止其配置事务后重试')
            motors = []
            for node_id in dict.fromkeys(node_ids):
                if node_id not in self._network:
                    node = self._network.add_node(node_id)
                    node.sdo.RESPONSE_TIMEOUT = 0.2
                    node.sdo.MAX_RETRIES = 1
                node = self._network[node_id]
                fields = []
                for key, label, index, sub, kind, unit in CERB_OBJECTS:
                    if time.monotonic() - self._foreign_at < 2:
                        raise ValueError('检测到其他 SDO 主站，已停止读取')
                    field = {'key': key, 'label': label,
                             'object': f'0x{index:04X}:{sub:02X}', 'unit': unit}
                    try:
                        raw = node.sdo.upload(index, sub)
                        value = int.from_bytes(raw, 'little', signed=kind == 'signed')
                        field['raw'] = raw.hex()
                        field['value'] = (raw.rstrip(b'\x00').decode('utf-8', 'replace')
                                          if kind == 'text' else f'0x{value:04X}'
                                          if kind == 'hex' else value / 100
                                          if kind == 'centi' else value)
                    except (canopen.SdoAbortedError, canopen.SdoCommunicationError) as exc:
                        field['error'] = str(exc)
                    fields.append(field)
                    if key == 'identity' and 'error' in field:
                        break
                motors.append({'id': node_id, 'key': f'cerb:{node_id}', 'protocol': 'cerb', 'online': 'error' not in fields[0],
                               'fields': fields, 'read_at': time.time()})
            self._last_used = time.monotonic()
            if discovered is not None:
                motors += [m for m in discovered if m['protocol'] == 'vaxor' or not m['online']]
            return {'motors': motors, 'source': 'direct_can', 'read_only': True}

    def disconnect(self, token):
        with self._lock:
            self._require(token)
            self._disconnect()
            return {'connected': False}

    def _disconnect(self):
        self._scan_stop.set()
        if self._scan_thread:
            self._scan_thread.join(timeout=0.5)
            self._scan_thread = None
        try:
            if self._network is not None:
                self._network.disconnect()
        finally:
            self._network = None
            self._token = ''
            if self._file:
                self._file.close()
                self._file = None

    def close(self):
        self._closed.set()
        with self._lock:
            self._disconnect()
