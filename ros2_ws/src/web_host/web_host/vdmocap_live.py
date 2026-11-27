"""Request-scoped SDK receiver and latest-frame stream for the motion workbench."""

import threading
import time

from .vdmocap_record import body_connection, sdk_frame


LIVE_PATH = '/api/motion/live/vdmocap'


class _FrameClock:
    """Confirm a seek with the sender before letting an out-of-order packet rewind IK."""

    def __init__(self):
        self.index = self.rate = self.pending = None
        self.received = 0.
        self.segment = 0

    def accept(self, index, rate, now):
        restart = self.index is None or rate != self.rate or now - self.received > 2
        self.received = now
        if not restart:
            step = (index - self.index) % 2**32
            if step == 0:
                self.pending = None
                return False
            if step >= 2**31:
                if self.pending is not None:
                    candidate, since = self.pending
                    progress = (index - candidate) % 2**32
                    restart = 0 < progress <= 1 + rate / 4 or (progress == 0 and now - since >= .5)
                if not restart:
                    if self.pending is None or self.pending[0] != index:
                        self.pending = index, now
                    return False
        self.index, self.rate, self.pending = index, rate, None
        self.segment += int(restart)
        return True


class LiveStream:
    def __init__(self, sdk_dir, ip, port):
        self.sdk_dir, self.ip, self.port = sdk_dir, ip, port
        self.access = threading.Condition()
        self.owner = self.stop = None
        self.shutdown = threading.Event()

    def acquire(self, viewer):
        with self.access:
            if self.stop is not None:
                if not viewer or viewer != self.owner:
                    return None
                # Same tab restarted before TCP reported the previous abort.
                self.stop.set()
                self.access.wait_for(lambda: self.stop is None or self.shutdown.is_set(), timeout=5)
            if self.stop is not None or self.shutdown.is_set():
                return None
            self.owner, self.stop = viewer, threading.Event()
            return self.stop

    def release(self):
        with self.access:
            self.owner = self.stop = None
            self.access.notify_all()

    def run(self, model, jobs, emit, stop):
        from .vdmocap_live_pose import LivePose

        changed = threading.Condition()
        rig, packet, error = None, None, None
        received = seq = 0

        def receive():
            nonlocal rig, received, seq, packet, error
            try:
                with body_connection(self.sdk_dir, self.ip, self.port) as (sdk, peer, names, rest):
                    with changed:
                        rig = (names, rest)
                        changed.notify()
                    data = sdk.MocapData()
                    clock = _FrameClock()
                    reconnect_at = time.monotonic() + 3
                    while not stop.is_set() and not self.shutdown.is_set():
                        if sdk.udp_recv_mocap_data(*peer, data) and data.isUpdate:
                            now = time.monotonic()
                            reconnect_at = now + 3
                            with changed:
                                received = now
                                if clock.accept(data.frameIndex, data.frequency, now):
                                    packet = clock.segment, sdk_frame(data)
                                    seq += 1
                                    changed.notify()
                        elif time.monotonic() >= reconnect_at:
                            sdk.udp_send_request_connect(*peer)
                            reconnect_at = time.monotonic() + 3
                        stop.wait(.002)
            except Exception as exc:
                with changed:
                    error = str(exc) or 'SDK 接收失败'
                    changed.notify()

        emit({'type': 'state', 'state': 'waiting', 'endpoint': f'{self.ip}:{self.port}'})
        reader = threading.Thread(target=receive, name='vdmocap-udp', daemon=True)
        reader.start()
        try:
            pose, sent, last_pose, segment = None, 0, 0, None
            while not stop.is_set() and not self.shutdown.is_set():
                with changed:
                    changed.wait_for(lambda: error or seq != sent or (pose is None and rig is not None)
                                     or stop.is_set() or self.shutdown.is_set(), timeout=.5)
                    if stop.is_set() or self.shutdown.is_set():
                        return
                    if error:
                        raise OSError(error)
                    skeleton, source, version, last_packet = rig, packet, seq, received
                if pose is None and skeleton is not None:
                    pose = LivePose(model, jobs.models_root, jobs.profiles_dir, *skeleton)
                    emit({'type': 'scene', **pose.scene})
                    continue  # Re-read the latest frame after constructing the model.
                if version != sent:
                    sent = version
                    try:
                        new_segment, sample = source
                        frame = pose.frame(sample, restart=new_segment != segment)
                        segment = new_segment
                    except ValueError as exc:
                        emit({'type': 'state', 'state': 'invalid', 'message': str(exc)})
                        continue
                    last_pose = time.monotonic()
                    emit({'type': 'frame', **frame})
                else:
                    now = time.monotonic()
                    if not last_pose:
                        state = 'waiting'
                    elif now - last_packet > 2:
                        state = 'disconnected'
                    elif now - last_pose >= .5:
                        state = 'paused'
                    else:
                        state = 'streaming'
                    # A heartbeat also detects a closed tab when the broadcaster is silent.
                    emit({'type': 'state', 'state': state})
        finally:
            stop.set()
            reader.join()

    def close(self):
        self.shutdown.set()
        # Wait for the request handler to close its native UDP socket.
        with self.access:
            if self.stop:
                self.stop.set()
            self.access.notify_all()
            self.access.wait_for(lambda: self.stop is None)


def serve_live(handler, live, jobs, request):
    import json
    from .models import enumerate_arm_models

    model = request.get('model') if isinstance(request, dict) else None
    if not isinstance(model, str) or model not in enumerate_arm_models(jobs.models_root):
        handler._send_json(400, {'error': '请选择模型库中的目标模型'})
        return
    viewer = request.get('viewer')
    if viewer is not None and (not isinstance(viewer, str) or len(viewer) != 32
                               or any(c not in '0123456789abcdef' for c in viewer)):
        handler._send_json(400, {'error': '实时预览会话标识无效'})
        return
    stop = live.acquire(viewer)
    if stop is None:
        handler._send_json(409, {'error': '实时接收端正在使用，请先停止原预览再重试'})
        return

    def emit(message):
        data = json.dumps(message, ensure_ascii=False, allow_nan=False, separators=(',', ':'))
        handler.wfile.write((data + '\n').encode('utf-8'))
        handler.wfile.flush()

    try:
        handler.close_connection = True
        handler.connection.settimeout(3)
        handler.send_response(200)
        handler.send_header('Content-Type', 'application/x-ndjson; charset=utf-8')
        handler.send_header('Cache-Control', 'no-store')
        handler.send_header('X-Content-Type-Options', 'nosniff')
        handler.send_header('X-Accel-Buffering', 'no')
        handler.send_header('Connection', 'close')
        handler.end_headers()
        try:
            live.run(model, jobs, emit, stop)
        except (ConnectionError, TimeoutError):
            return
        except Exception as exc:
            emit({'type': 'error', 'message': str(exc) or '实时预览已中断'})
    except (ConnectionError, TimeoutError):
        pass  # Browser stopped the stream.
    finally:
        live.release()
