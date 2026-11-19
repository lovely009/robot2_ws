"""Chuangxin Linux USB-CAN FD transport; SDK calls are serialized per device."""
from collections import deque
from contextlib import ExitStack
import ctypes as C
import fcntl
import os
from pathlib import Path
import struct
import tempfile
import threading


class Packed(C.Structure):
    _pack_ = 1


class FDConfig(Packed):
    _fields_ = [(x, C.c_uint32) for x in ('code', 'mask', 'abit', 'dbit', 'brp')] + [
        ('filter', C.c_uint8), ('mode', C.c_uint8), ('pad', C.c_uint16), ('reserved', C.c_uint32)]


class Config(Packed):
    _fields_ = [('can_type', C.c_uint32), ('fd', FDConfig)]


class Frame(Packed):
    _fields_ = [('can_id', C.c_uint32), ('length', C.c_uint8), ('flags', C.c_uint8),
                ('reserved', C.c_uint16), ('data', C.c_uint8 * 64)]


class ClassicFrame(Packed):
    _fields_ = [('can_id', C.c_uint32), ('length', C.c_uint8),
                ('reserved', C.c_uint8 * 3), ('data', C.c_uint8 * 8)]


class RxFD(Packed):
    _fields_ = [('frame', Frame), ('timestamp', C.c_uint64)]


class RxCAN(Packed):
    _fields_ = [('frame', ClassicFrame), ('timestamp', C.c_uint64)]


class TxFD(Packed):
    _fields_ = [('frame', Frame), ('transmit_type', C.c_uint32)]


class TxCAN(Packed):
    _fields_ = [('frame', ClassicFrame), ('transmit_type', C.c_uint32)]


def enumerate_usbcanfd_devices(root='/sys/bus/usb/devices'):
    from pathlib import Path
    devices = []
    for path in Path(root).glob('*'):
        try:
            if ((path / 'idVendor').read_text().strip(),
                    (path / 'idProduct').read_text().strip()) != ('04d8', '0053'):
                continue
            bus = int((path / 'busnum').read_text())
            address = int((path / 'devnum').read_text())
        except (OSError, ValueError):
            continue
        node = f'/dev/bus/usb/{bus:03d}/{address:03d}'
        devices.append({'bus': bus, 'address': address, 'path': node,
                        'accessible': os.access(node, os.R_OK | os.W_OK)})
    devices.sort(key=lambda d: (d['bus'], d['address']))
    for index, device in enumerate(devices):
        device['index'] = index
        device['label'] = f"创芯 USB-CAN FD #{index + 1} ({device['bus']:03d}/{device['address']:03d})"
    return devices

usb_devices = enumerate_usbcanfd_devices


class CtypesSDK:
    def __init__(self, library=None):
        library = library or os.environ.get('ROBOT2_CONTROLCANFD_LIB')
        candidates = [Path(library)] if library else [
            directory / name
            for directory in (Path.home() / '.local/lib/robot2', Path('/usr/local/lib'))
            for name in ('libcontrolcanfd_robot2.so', 'libcontrolcanfd.so')]
        path = next((p for p in candidates if p.is_file()), None)
        if path is None:
            raise RuntimeError('找不到 libcontrolcanfd.so；设置 ROBOT2_CONTROLCANFD_LIB 或 --usbcanfd-library')
        self.view = tempfile.TemporaryDirectory(prefix='robot2-usbcanfd-')
        try:
            for device in usb_devices():
                bus = Path(self.view.name) / f"{device['bus']:03d}"
                bus.mkdir(exist_ok=True)
                (bus / f"{device['address']:03d}").symlink_to(device['path'])
            # The vendor's bundled libusb otherwise opens unrelated USB devices.
            os.environ['USB_DEVFS_PATH'] = self.view.name
            self.lib = C.CDLL(str(path), mode=C.RTLD_LOCAL)
            u = C.c_uint32; p = C.c_void_p
            signatures = {
                'ZCAN_OpenDeviceEx': ([u, u, u, u, u], p),
                'ZCAN_CloseDevice': ([p], u),
                'ZCAN_InitCAN': ([p, u, C.POINTER(Config)], p),
                'ZCAN_StartCAN': ([p], u), 'ZCAN_ResetCAN': ([p], u),
                'ZCAN_ClearFilter': ([p], u), 'ZCAN_AckFilter': ([p], u),
                'ZCAN_ReceiveFD': ([p, C.POINTER(RxFD), u, C.c_int], u),
                'ZCAN_Receive': ([p, C.POINTER(RxCAN), u, C.c_int], u),
                'ZCAN_TransmitFD': ([p, C.POINTER(TxFD), u], u),
                'ZCAN_Transmit': ([p, C.POINTER(TxCAN), u], u),
            }
            for name in ('ZCAN_SetAbitBaud', 'ZCAN_SetDbitBaud',
                         'ZCAN_SetCANFDStandard', 'ZCAN_SetResistanceEnable'):
                signatures[name] = ([p, u, u], u)
            for name, (args, result) in signatures.items():
                func = getattr(self.lib, name)
                func.argtypes = args; func.restype = result
                setattr(self, name, func)
        except BaseException:
            self.view.cleanup()
            raise

    def close(self):
        self.view.cleanup()


def _ok(value, operation):
    if value != 1:
        raise RuntimeError(f'{operation} failed (SDK status {value})')


class CxDevice:
    def __init__(self, api, bus, address, lock_fd=None, device_index=0, lock=None):
        self.api = api; self.lock = lock or threading.RLock(); self.channels = {}
        self.lock_fd = lock_fd; self.handle = None
        try:
            self.handle = api.ZCAN_OpenDeviceEx(41, device_index, bus, address, 0)
            if not self.handle:
                raise RuntimeError(f'cannot open USB-CAN FD {bus:03d}/{address:03d}; check USB permissions, device response and connection')
        except BaseException:
            self.close()
            raise

    def open_channel(self, index):
        with self.lock:
            if index in self.channels:
                raise ValueError('duplicate USB-CAN FD channel')
            if not self.handle:
                raise RuntimeError('USB-CAN FD device is closed')
            for name, value in (('ZCAN_SetAbitBaud', 1000000), ('ZCAN_SetDbitBaud', 5000000),
                                ('ZCAN_SetCANFDStandard', 0), ('ZCAN_SetResistanceEnable', 1)):
                _ok(getattr(self.api, name)(self.handle, index, value), name)
            config = Config(); config.can_type = 1; config.fd.mask = 0xffffffff; config.fd.filter = 1
            handle = self.api.ZCAN_InitCAN(self.handle, index, C.byref(config))
            if not handle:
                raise RuntimeError(f'ZCAN_InitCAN failed for channel {index}')
            channel = CxChannel(self, handle, index)
            self.channels[index] = channel
            try:
                _ok(self.api.ZCAN_ClearFilter(handle), 'ZCAN_ClearFilter')
                _ok(self.api.ZCAN_AckFilter(handle), 'ZCAN_AckFilter')
                _ok(self.api.ZCAN_StartCAN(handle), 'ZCAN_StartCAN')
            except BaseException:
                channel.close()
                raise
            return channel

    def close(self):
        with self.lock:
            try:
                with ExitStack() as cleanup:
                    if self.handle:
                        cleanup.callback(self.api.ZCAN_CloseDevice, self.handle)
                    for channel in list(self.channels.values()):
                        cleanup.callback(channel.close)
            finally:
                self.handle = None
                if self.lock_fd is not None:
                    os.close(self.lock_fd); self.lock_fd = None

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


class CxChannel:
    polling = True

    def __init__(self, device, handle, index):
        self.device = device; self.handle = handle; self.index = index
        self.pending = deque()
        self.label = f'USB-CAN FD channel {index + 1}'

    def send(self, data):
        with self.device.lock:
            if not self.handle:
                raise OSError('USB-CAN FD channel is closed')
            can_id, size, flags = struct.unpack_from('=IBB', data)
            fd = len(data) == 72
            packet = TxFD() if fd else TxCAN()
            packet.frame.can_id = can_id; packet.frame.length = size
            if fd:
                packet.frame.flags = flags & 1
            packet.frame.data[:size] = data[8:8 + size]
            func = self.device.api.ZCAN_TransmitFD if fd else self.device.api.ZCAN_Transmit
            sent = func(self.handle, C.byref(packet), 1)
            if sent == 0:
                raise BlockingIOError('USB-CAN FD transmit queue is full')
            if sent != 1:
                raise OSError(f'USB-CAN FD transmit failed ({sent})')
            return len(data)

    def recv(self, _size):
        with self.device.lock:
            if not self.handle:
                raise OSError('USB-CAN FD channel is closed')
            if self.pending:
                return self.pending.popleft()
            for fd, packet_type, receive in (
                    (True, RxFD, self.device.api.ZCAN_ReceiveFD),
                    (False, RxCAN, self.device.api.ZCAN_Receive)):
                packets = (packet_type * 128)()
                count = receive(self.handle, packets, 128, 0)
                if not 0 <= count <= 128:
                    raise OSError(f'USB-CAN FD receive failed ({count})')
                for packet in packets[:count]:
                    frame = packet.frame
                    # Vendor FD RX uses bit 29 for error-passive ESI, not a Linux error frame.
                    if frame.can_id & 0xc0000000 or frame.length > (64 if fd else 8):
                        continue
                    can_id = frame.can_id & 0x1fffffff
                    if can_id > 0x7ff:
                        continue
                    payload = bytes(frame.data[:frame.length])
                    self.pending.append(struct.pack('=IBBBB64s' if fd else '=IBBBB8s',
                        can_id, frame.length, frame.flags & 1 if fd else 0, 0, 0, payload))
            if not self.pending:
                raise BlockingIOError()
            return self.pending.popleft()

    def close(self):
        with self.device.lock:
            if self.handle:
                try:
                    self.device.api.ZCAN_ResetCAN(self.handle)
                finally:
                    self.handle = None
                    self.device.channels.pop(self.index, None)


class CxPool:
    def __init__(self, library=None):
        self.library = library; self.api = None; self.devices = {}
        self.lock = threading.RLock()

    def channel(self, device_index, channel):
        if device_index not in self.devices:
            devices = usb_devices()
            if device_index >= len(devices):
                raise ValueError(f'USB-CAN FD device {device_index} is not connected')
            info = devices[device_index]
            lock_path = f"/tmp/robot2-usbcanfd-{info['bus']}-{info['address']}.lock"
            fd = os.open(lock_path, os.O_CREAT | os.O_RDWR | os.O_CLOEXEC, 0o600)
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                if self.api is None:
                    self.api = CtypesSDK(self.library)
                device = CxDevice(self.api, info['bus'], info['address'],
                                  device_index=device_index, lock=self.lock)
                device.lock_fd = fd
                self.devices[device_index] = device
            except BaseException:
                os.close(fd)
                raise
        return self.devices[device_index].open_channel(channel)

    def close(self):
        try:
            with ExitStack() as cleanup:
                if self.api is not None:
                    cleanup.callback(self.api.close)
                for device in self.devices.values():
                    cleanup.callback(device.close)
        finally:
            self.devices.clear()
            self.api = None
