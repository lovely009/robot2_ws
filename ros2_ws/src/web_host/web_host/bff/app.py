"""Single-port FastAPI + rclpy BFF application."""
from __future__ import annotations

import asyncio
import collections
from contextlib import asynccontextmanager
import json
import os
from pathlib import Path
import re
import threading
import time
from typing import Any, Literal
import uuid

from fastapi import FastAPI, Header, HTTPException, Query, Request, WebSocket, WebSocketDisconnect
from fastapi.middleware.gzip import GZipMiddleware
from fastapi.responses import FileResponse, RedirectResponse, Response, StreamingResponse

from ..actions import bvh_directory, list_actions, read_action, save_action
from ..models import enumerate_arm_models, model_motor_params, model_transmissions, motor_library
from ..motion import MotionJobs
from ..motion_catalog import source_statuses
from ..motion_playback import _load_trajectory
from ..vdmocap_import import ImportConflict, import_capture
from ..vdmocap_live import LiveStream
from ..vdmocap_record import DEFAULT_SDK_DIR
from .lease import ControlLeaseManager, LeaseAuthError, LeaseConflictError
from .recorder import RosbagRecorder
from .ros_gateway import CHANNEL_TOPICS, MockRosGateway, RosGatewayProtocol, RosGatewayThread
from .schemas import (
    ActionSaveRequest,
    ActionSaveResponse,
    BenchActionRequest,
    BenchGravityGainRequest,
    BenchUnitLogRequest,
    DiagnosticsSnapshot,
    ExecuteConfigRequest,
    JointTargetRequest,
    LeaseAcquireRequest,
    LeaseAcquireResponse,
    LeaseReleaseRequest,
    LeaseState,
    NodeParametersRequest,
    PublishAck,
    RecordingInfo,
    RecordingListResponse,
    RecordingStartRequest,
    RecordingStatus,
    RuntimeConfig,
    ServiceCallResponse,
    StopRequest,
    StopResponse,
    SubmitPolicyRequest,
    TrajectoryRequest,
)
from .security import enforce_same_origin, same_origin

_HASHED_ASSET_RE = re.compile(r'^/workbench/assets/.+-[0-9a-zA-Z_-]{8,}\.(?:js|mjs|css)$')
_NO_STORE_SUFFIXES = ('.html', '.js', '.mjs', '.css', '.json')
_MAX_ACTION_BYTES = 8 * 1024 * 1024
_MAX_IMPORT_BYTES = 24 * 1024 * 1024

_BENCH_LIFECYCLE_SERVICES: dict[str, tuple[str, str]] = {
    'simulation_start': ('/bench/start_simulation', 'bench_msgs/srv/StartSimulation'),
    'simulation_stop': ('/bench/stop_simulation', 'bench_msgs/srv/StopSimulation'),
    'agent_start': ('/bench/start_agent', 'bench_msgs/srv/StartAgent'),
    'agent_stop': ('/bench/stop_agent', 'bench_msgs/srv/StopAgent'),
    'motor_backend_start': ('/bench/start_motor_backend', 'bench_msgs/srv/StartMotorBackend'),
    'motor_backend_stop': ('/bench/stop_motor_backend', 'bench_msgs/srv/StopMotorBackend'),
    'motor_backend_command': ('/bench/motor_backend_command', 'bench_msgs/srv/MotorBackendCommand'),
    'gravity_start': ('/bench/start_gravity_feedforward', 'bench_msgs/srv/StartGravityFeedforward'),
    'gravity_stop': ('/bench/stop_gravity_feedforward', 'bench_msgs/srv/StopGravityFeedforward'),
}

_STATE_CHANNELS = (
    'telemetry:control',
    'telemetry:transmission',
    'telemetry:bench',
    'telemetry:sim_position',
    'telemetry:joints',
    'telemetry:imu',
    'telemetry:imu_temp',
    'telemetry:motor_motion',
    'telemetry:motor_state',
)


class ConnectionHub:
    """Manages active WebSocket clients and channel fan-out."""

    def __init__(self) -> None:
        self._clients: dict[str, WebSocket] = {}
        self._channels: dict[str, set[str]] = {}
        self._loop: asyncio.AbstractEventLoop | None = None

    def bind_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        self._loop = loop

    @property
    def client_count(self) -> int:
        return len(self._clients)

    def has_subscribers(self, channel: str) -> bool:
        return any(channel in ch for ch in self._channels.values())

    def add(self, client_id: str, ws: WebSocket) -> None:
        self._clients[client_id] = ws
        self._channels[client_id] = {'lease:state'}

    def remove(self, client_id: str) -> None:
        self._clients.pop(client_id, None)
        self._channels.pop(client_id, None)

    def subscribe(self, client_id: str, channels: list[str]) -> list[str]:
        subs = self._channels.setdefault(client_id, {'lease:state'})
        for ch in channels:
            if isinstance(ch, str) and ch:
                subs.add(ch)
        return sorted(subs)

    def unsubscribe(self, client_id: str, channels: list[str]) -> list[str]:
        subs = self._channels.setdefault(client_id, {'lease:state'})
        for ch in channels:
            subs.discard(ch)
        return sorted(subs)

    async def broadcast_text(self, channel: str, payload: str) -> int:
        targets = [
            (cid, ws)
            for cid, ws in list(self._clients.items())
            if channel in self._channels.get(cid, ())
        ]
        if not targets:
            return 0

        async def _send(cid: str, ws: WebSocket) -> bool:
            try:
                await asyncio.wait_for(ws.send_text(payload), timeout=0.5)
                return True
            except Exception:
                self.remove(cid)
                return False

        results = await asyncio.gather(*(_send(cid, ws) for cid, ws in targets))
        return sum(1 for ok in results if ok)

    def schedule_broadcast(self, channel: str, payload: str) -> None:
        loop = self._loop
        if loop is None or loop.is_closed():
            return
        try:
            running = asyncio.get_running_loop()
        except RuntimeError:
            running = None
        if running is loop:
            loop.create_task(self.broadcast_text(channel, payload))
        else:
            loop.call_soon_threadsafe(lambda: loop.create_task(self.broadcast_text(channel, payload)))

    async def close_all(self) -> None:
        for cid, ws in list(self._clients.items()):
            self.remove(cid)
            try:
                await ws.close()
            except Exception:
                pass


def _resolve_under(root: Path, rel_path: str) -> Path | None:
    if '\x00' in rel_path or '\\' in rel_path:
        return None
    parts = [p for p in rel_path.split('/') if p and p != '.']
    if any(p == '..' for p in parts):
        return None
    root_abs = os.path.abspath(str(root))
    candidate = os.path.abspath(os.path.join(root_abs, *parts))
    if candidate != root_abs and not candidate.startswith(root_abs + os.sep):
        return None
    return Path(candidate)


def create_app(
    *,
    ros_gateway: RosGatewayProtocol | None = None,
    web_root: str | Path | None = None,
    model_dir: str | Path | None = None,
    model_url_prefix: str = '/robot_sim',
    motion_data_dir: str | Path | None = None,
    recordings_dir: str | Path | None = None,
    vdmocap_sdk_dir: str | Path | None = None,
    vdmocap_ip: str = '192.168.1.9',
    vdmocap_port: int = 7000,
    control_mode: Literal['bench', 'readonly'] = 'bench',
    robot_model: str = '',
    lease_ttl_s: float = 3.0,
    pin: str | None = None,
    broadcast_hz: float = 30.0,
) -> FastAPI:
    """Create the production single-port FastAPI + rclpy BFF application."""
    if control_mode not in ('bench', 'readonly'):
        raise ValueError('control_mode must be bench or readonly')

    pkg_root = Path(os.path.abspath(str(Path(__file__)))).parents[2]
    resolved_web_root = Path(os.path.abspath(str(web_root))) if web_root else (pkg_root / 'web')
    resolved_web_root.mkdir(parents=True, exist_ok=True)

    resolved_model_dir: Path | None = None
    models_root: str | None = None
    if model_dir and Path(model_dir).is_dir():
        resolved_model_dir = Path(os.path.abspath(str(model_dir)))
        models_subdir = resolved_model_dir / 'models'
        if models_subdir.is_dir():
            models_root = str(models_subdir)
    prefix = model_url_prefix.rstrip('/') if resolved_model_dir else ''

    data_root = (
        Path(motion_data_dir)
        if motion_data_dir
        else Path(os.environ.get('XDG_DATA_HOME', Path.home() / '.local/share')) / 'robot2'
    )
    motion_jobs = MotionJobs(
        actions_dir=bvh_directory(str(resolved_web_root)),
        profiles_dir=data_root / 'motion_mappings',
        models_root=models_root,
        cache_dir=data_root / 'motion_results',
    )
    live_stream = LiveStream(
        Path(vdmocap_sdk_dir) if vdmocap_sdk_dir else Path(DEFAULT_SDK_DIR),
        vdmocap_ip,
        int(vdmocap_port),
    )

    gateway: RosGatewayProtocol = ros_gateway if ros_gateway is not None else RosGatewayThread()
    recorder = RosbagRecorder(
        Path(recordings_dir) if recordings_dir else (data_root / 'recordings'),
        gateway,
    )
    hub = ConnectionHub()
    started_mono = time.monotonic()
    metrics = {'event_loop_lag_ms': 0.0}
    tx_times: collections.deque[float] = collections.deque()

    def _on_lease_state_changed(state: LeaseState) -> None:
        payload = json.dumps({'op': 'lease:state', 'lease': state.model_dump()})
        hub.schedule_broadcast('lease:state', payload)

    lease = ControlLeaseManager(
        ttl_s=lease_ttl_s,
        pin=pin,
        on_active_count_changed=lambda count: gateway.publish_client_count(count),
        on_stop_requested=lambda _reason: gateway.publish_stop(),
        on_state_changed=_on_lease_state_changed,
    )

    @asynccontextmanager
    async def lifespan(_app: FastAPI):
        hub.bind_loop(asyncio.get_running_loop())
        gateway.start()

        async def _watchdog() -> None:
            interval = 0.1
            next_tick = time.monotonic() + interval
            while True:
                await asyncio.sleep(max(0.0, next_tick - time.monotonic()))
                now = time.monotonic()
                lag_ms = max(0.0, (now - next_tick) * 1000.0)
                metrics['event_loop_lag_ms'] = round(
                    0.8 * metrics['event_loop_lag_ms'] + 0.2 * lag_ms, 3,
                )
                next_tick = now + interval
                lease.expire_if_needed()

        async def _telemetry_loop() -> None:
            interval = 1.0 / max(1.0, float(broadcast_hz))
            last_seq: dict[str, int] = {}
            last_event_seq = 0
            while True:
                await asyncio.sleep(interval)
                now_mono = time.monotonic()
                sent_any = False

                for ch in _STATE_CHANNELS:
                    if not hub.has_subscribers(ch):
                        continue
                    snap, _rx_count, rx_hz = gateway.latest_channel(ch)
                    if snap is None or snap.seq == last_seq.get(ch):
                        continue
                    frame = json.dumps({
                        'channel': ch,
                        'seq': snap.seq,
                        'server_ts_ms': int(time.time() * 1000),
                        'ros_stamp_ns': snap.ros_stamp_ns,
                        'source_hz': round(rx_hz, 2),
                        'age_ms': round(max(0.0, (now_mono - snap.received_mono) * 1000.0), 3),
                        'data': snap.payload,
                    }, ensure_ascii=False, allow_nan=False, separators=(',', ':'))
                    if await hub.broadcast_text(ch, frame) > 0:
                        sent_any = True
                    last_seq[ch] = snap.seq

                if hub.has_subscribers('telemetry:events'):
                    events = gateway.events_since(last_event_seq)
                    for item in events:
                        pkt = json.dumps({
                            'channel': 'telemetry:events',
                            'seq': item.seq,
                            'server_ts_ms': int(time.time() * 1000),
                            'data': item.payload,
                        }, ensure_ascii=False, allow_nan=False, separators=(',', ':'))
                        if await hub.broadcast_text('telemetry:events', pkt) > 0:
                            sent_any = True
                        last_event_seq = item.seq

                if sent_any:
                    tx_times.append(now_mono)
                    cutoff = now_mono - 1.0
                    while tx_times and tx_times[0] < cutoff:
                        tx_times.popleft()

        watchdog_task = asyncio.create_task(_watchdog())
        telemetry_task = asyncio.create_task(_telemetry_loop())
        try:
            yield
        finally:
            watchdog_task.cancel()
            telemetry_task.cancel()
            await asyncio.gather(watchdog_task, telemetry_task, return_exceptions=True)
            await hub.close_all()
            if recorder.status().active:
                try:
                    recorder.stop()
                except Exception:
                    pass
            live_stream.close()
            motion_jobs.close()
            gateway.stop()

    app = FastAPI(title='Robot Workbench BFF', version='1.0.0', lifespan=lifespan)
    app.add_middleware(GZipMiddleware, minimum_size=512)
    app.state.lease = lease
    app.state.gateway = gateway
    app.state.hub = hub
    app.state.recorder = recorder

    @app.middleware('http')
    async def cache_control_middleware(request: Request, call_next):
        response = await call_next(request)
        if 'cache-control' not in response.headers:
            path_lower = request.url.path.lower()
            if _HASHED_ASSET_RE.match(path_lower):
                response.headers['Cache-Control'] = 'public, max-age=31536000, immutable'
            elif (
                path_lower == '/'
                or path_lower.endswith('/')
                or path_lower.startswith('/api/')
                or path_lower.endswith(_NO_STORE_SUFFIXES)
            ):
                response.headers['Cache-Control'] = 'no-store'
            else:
                response.headers['Cache-Control'] = 'no-cache'
        return response

    def _require_bench_owner(
        client_id: str | None,
        token: str | None,
    ) -> None:
        if control_mode != 'bench':
            raise HTTPException(
                status_code=403,
                detail={'error': '只读页面不开放控制或维护服务', 'code': 'readonly_mode'},
            )
        if not lease.is_owner(client_id, token):
            raise HTTPException(
                status_code=403,
                detail={'error': '未持有控制权或租约已过期', 'code': 'lease_required'},
            )

    async def _call_ros_service(
        service_name: str,
        service_type: str,
        payload: dict[str, Any],
        timeout_s: float = 8.0,
    ) -> ServiceCallResponse:
        try:
            res = await asyncio.to_thread(
                gateway.call_service, service_name, service_type, payload, timeout_s,
            )
            return ServiceCallResponse(ok=True, service=service_name, result=res)
        except TimeoutError as exc:
            raise HTTPException(status_code=504, detail={'error': str(exc)}) from exc
        except Exception as exc:
            raise HTTPException(status_code=502, detail={'error': str(exc)}) from exc

    # --- Health / Runtime / Diagnostics ---

    @app.get('/api/health')
    async def health() -> dict[str, str]:
        return {'status': 'ok'}

    @app.get('/api/runtime', response_model=RuntimeConfig)
    async def get_runtime() -> RuntimeConfig:
        return RuntimeConfig(
            control_mode=control_mode,
            robot_model=robot_model,
            transport='bff',
            pin_required=bool(pin),
        )

    @app.get('/api/diagnostics', response_model=DiagnosticsSnapshot)
    async def diagnostics() -> DiagnosticsSnapshot:
        now = time.monotonic()
        cutoff = now - 1.0
        while tx_times and tx_times[0] < cutoff:
            tx_times.popleft()
        state = lease.snapshot()
        return DiagnosticsSnapshot(
            uptime_s=round(now - started_mono, 3),
            ros_connected=gateway.is_ready(),
            event_loop_lag_ms=metrics['event_loop_lag_ms'],
            ws_clients=hub.client_count,
            active_controllers=state.active_controllers,
            ws_broadcast_hz=float(len(tx_times)),
            topics=gateway.topic_health(),
            recording_active=recorder.status().active,
        )

    # --- Control Lease & Universal Soft-Stop ---

    @app.get('/api/control/lease', response_model=LeaseState)
    async def get_lease() -> LeaseState:
        lease.expire_if_needed()
        return lease.snapshot()

    @app.post('/api/control/lease/acquire', response_model=LeaseAcquireResponse)
    async def acquire_lease(request: Request, body: LeaseAcquireRequest) -> LeaseAcquireResponse:
        enforce_same_origin(request)
        try:
            state, token, preempted = lease.acquire(
                body.client_id, body.label, force=body.force, pin=body.pin,
            )
        except LeaseAuthError as exc:
            raise HTTPException(status_code=403, detail={'error': str(exc)}) from exc
        except LeaseConflictError as exc:
            raise HTTPException(
                status_code=409,
                detail={'error': str(exc), 'lease': exc.current.model_dump()},
            ) from exc
        return LeaseAcquireResponse(lease=state, token=token, preempted_previous=preempted)

    @app.post('/api/control/lease/release', response_model=LeaseState)
    async def release_lease(request: Request, body: LeaseReleaseRequest) -> LeaseState:
        enforce_same_origin(request)
        return lease.release(body.client_id, body.token)

    @app.post('/api/control/stop', response_model=StopResponse)
    async def trigger_stop(request: Request, body: StopRequest) -> StopResponse:
        enforce_same_origin(request)
        published = gateway.publish_stop()
        return StopResponse(
            accepted=True,
            published=published,
            reason=body.reason,
            timestamp_ms=int(time.time() * 1000),
        )

    # --- Lease-Gated Control & Sim Publish Endpoints ---

    @app.post('/api/control/joint-target', response_model=PublishAck)
    async def publish_control_joint_target(
        request: Request,
        body: JointTargetRequest,
        x_client_id: str | None = Header(default=None),
        x_lease_token: str | None = Header(default=None),
    ) -> PublishAck:
        enforce_same_origin(request)
        _require_bench_owner(x_client_id, x_lease_token)
        ok = gateway.publish_joint_target(
            '/control/joint_target',
            body.names,
            body.positions,
            body.velocities,
            body.efforts,
            body.stamp_ns,
        )
        return PublishAck(
            accepted=True,
            published=ok,
            topic='/control/joint_target',
            timestamp_ms=int(time.time() * 1000),
        )

    @app.post('/api/sim/joint-target', response_model=PublishAck)
    async def publish_sim_joint_target(
        request: Request,
        body: JointTargetRequest,
        x_client_id: str | None = Header(default=None),
        x_lease_token: str | None = Header(default=None),
    ) -> PublishAck:
        enforce_same_origin(request)
        _require_bench_owner(x_client_id, x_lease_token)
        ok = gateway.publish_joint_target(
            '/robot_sim/joint_position_target',
            body.names,
            body.positions,
            body.velocities,
            body.efforts,
            body.stamp_ns,
        )
        return PublishAck(
            accepted=True,
            published=ok,
            topic='/robot_sim/joint_position_target',
            timestamp_ms=int(time.time() * 1000),
        )

    @app.post('/api/control/trajectory', response_model=PublishAck)
    async def publish_control_trajectory(
        request: Request,
        body: TrajectoryRequest,
        x_client_id: str | None = Header(default=None),
        x_lease_token: str | None = Header(default=None),
    ) -> PublishAck:
        enforce_same_origin(request)
        _require_bench_owner(x_client_id, x_lease_token)
        ok = gateway.publish_trajectory(
            body.joint_names,
            [pt.model_dump() for pt in body.points],
            body.stamp_ns,
        )
        return PublishAck(
            accepted=True,
            published=ok,
            topic='/control/trajectory',
            timestamp_ms=int(time.time() * 1000),
        )

    # --- Motor & Bench Domain Services ---

    @app.get('/api/motor/device-info', response_model=ServiceCallResponse)
    async def get_motor_device_info(motor_id: int = Query(default=0, ge=0)) -> ServiceCallResponse:
        return await _call_ros_service(
            '/motor/get_device_info',
            'can_bridge_msgs/srv/GetDeviceInfo',
            {'motor_id': motor_id},
            timeout_s=6.0,
        )

    @app.post('/api/motor/policy', response_model=ServiceCallResponse)
    async def submit_motor_policy(
        request: Request,
        body: SubmitPolicyRequest,
        x_client_id: str | None = Header(default=None),
        x_lease_token: str | None = Header(default=None),
    ) -> ServiceCallResponse:
        enforce_same_origin(request)
        _require_bench_owner(x_client_id, x_lease_token)
        ns = body.source_time_ns
        header = {'stamp': {'sec': ns // 1_000_000_000, 'nanosec': ns % 1_000_000_000}, 'frame_id': ''}
        return await _call_ros_service(
            '/motor/submit_policy',
            'can_bridge_msgs/srv/SubmitPolicy',
            {'header': header, 'session_id': body.session_id, 'targets': body.targets},
            timeout_s=8.0,
        )

    @app.post('/api/motor/config', response_model=ServiceCallResponse)
    async def execute_motor_config(
        request: Request,
        body: ExecuteConfigRequest,
        x_client_id: str | None = Header(default=None),
        x_lease_token: str | None = Header(default=None),
    ) -> ServiceCallResponse:
        enforce_same_origin(request)
        _require_bench_owner(x_client_id, x_lease_token)
        return await _call_ros_service(
            '/motor/execute_config',
            'can_bridge_msgs/srv/ExecuteConfig',
            {'request': body.request, 'timeout_ms': body.timeout_ms},
            timeout_s=(body.timeout_ms / 1000.0) + 2.0,
        )

    @app.get('/api/bench/options', response_model=ServiceCallResponse)
    async def get_bench_options() -> ServiceCallResponse:
        return await _call_ros_service('/bench/options', 'bench_msgs/srv/GetBenchOptions', {})

    @app.get('/api/bench/agent-status', response_model=ServiceCallResponse)
    async def get_bench_agent_status() -> ServiceCallResponse:
        return await _call_ros_service('/bench/agent_status', 'bench_msgs/srv/GetAgentStatus', {})

    @app.post('/api/bench/unit-log', response_model=ServiceCallResponse)
    async def get_bench_unit_log(request: Request, body: BenchUnitLogRequest) -> ServiceCallResponse:
        enforce_same_origin(request)
        return await _call_ros_service(
            '/bench/get_unit_log',
            'bench_msgs/srv/GetUnitLog',
            {'unit': body.unit, 'lines': body.lines},
        )

    async def _bench_write(
        key: str,
        request: Request,
        payload: dict[str, Any],
        x_client_id: str | None,
        x_lease_token: str | None,
    ) -> ServiceCallResponse:
        enforce_same_origin(request)
        _require_bench_owner(x_client_id, x_lease_token)
        srv_name, srv_type = _BENCH_LIFECYCLE_SERVICES[key]
        return await _call_ros_service(srv_name, srv_type, payload)

    @app.post('/api/bench/simulation/start', response_model=ServiceCallResponse)
    async def bench_sim_start(
        request: Request,
        body: BenchActionRequest,
        x_client_id: str | None = Header(default=None),
        x_lease_token: str | None = Header(default=None),
    ) -> ServiceCallResponse:
        return await _bench_write('simulation_start', request, body.payload, x_client_id, x_lease_token)

    @app.post('/api/bench/simulation/stop', response_model=ServiceCallResponse)
    async def bench_sim_stop(
        request: Request,
        x_client_id: str | None = Header(default=None),
        x_lease_token: str | None = Header(default=None),
    ) -> ServiceCallResponse:
        return await _bench_write('simulation_stop', request, {}, x_client_id, x_lease_token)

    @app.post('/api/bench/agent/start', response_model=ServiceCallResponse)
    async def bench_agent_start(
        request: Request,
        body: BenchActionRequest,
        x_client_id: str | None = Header(default=None),
        x_lease_token: str | None = Header(default=None),
    ) -> ServiceCallResponse:
        return await _bench_write('agent_start', request, body.payload, x_client_id, x_lease_token)

    @app.post('/api/bench/agent/stop', response_model=ServiceCallResponse)
    async def bench_agent_stop(
        request: Request,
        x_client_id: str | None = Header(default=None),
        x_lease_token: str | None = Header(default=None),
    ) -> ServiceCallResponse:
        return await _bench_write('agent_stop', request, {}, x_client_id, x_lease_token)

    @app.post('/api/bench/motor-backend/start', response_model=ServiceCallResponse)
    async def bench_motor_backend_start(
        request: Request,
        body: BenchActionRequest,
        x_client_id: str | None = Header(default=None),
        x_lease_token: str | None = Header(default=None),
    ) -> ServiceCallResponse:
        return await _bench_write(
            'motor_backend_start', request, body.payload, x_client_id, x_lease_token,
        )

    @app.post('/api/bench/motor-backend/stop', response_model=ServiceCallResponse)
    async def bench_motor_backend_stop(
        request: Request,
        x_client_id: str | None = Header(default=None),
        x_lease_token: str | None = Header(default=None),
    ) -> ServiceCallResponse:
        return await _bench_write('motor_backend_stop', request, {}, x_client_id, x_lease_token)

    @app.post('/api/bench/motor-backend/command', response_model=ServiceCallResponse)
    async def bench_motor_backend_command(
        request: Request,
        body: BenchActionRequest,
        x_client_id: str | None = Header(default=None),
        x_lease_token: str | None = Header(default=None),
    ) -> ServiceCallResponse:
        return await _bench_write(
            'motor_backend_command', request, body.payload, x_client_id, x_lease_token,
        )

    @app.post('/api/bench/gravity/start', response_model=ServiceCallResponse)
    async def bench_gravity_start(
        request: Request,
        body: BenchActionRequest,
        x_client_id: str | None = Header(default=None),
        x_lease_token: str | None = Header(default=None),
    ) -> ServiceCallResponse:
        return await _bench_write('gravity_start', request, body.payload, x_client_id, x_lease_token)

    @app.post('/api/bench/gravity/stop', response_model=ServiceCallResponse)
    async def bench_gravity_stop(
        request: Request,
        x_client_id: str | None = Header(default=None),
        x_lease_token: str | None = Header(default=None),
    ) -> ServiceCallResponse:
        return await _bench_write('gravity_stop', request, {}, x_client_id, x_lease_token)

    @app.post('/api/bench/gravity/gain', response_model=ServiceCallResponse)
    async def bench_gravity_gain(
        request: Request,
        body: BenchGravityGainRequest,
        x_client_id: str | None = Header(default=None),
        x_lease_token: str | None = Header(default=None),
    ) -> ServiceCallResponse:
        enforce_same_origin(request)
        _require_bench_owner(x_client_id, x_lease_token)
        return await _call_ros_service(
            '/bench/set_gravity_feedforward_gain',
            'bench_msgs/srv/SetGravityFeedforwardGain',
            {'gain': body.gain},
        )

    @app.post('/api/control/parameters', response_model=ServiceCallResponse)
    async def set_node_parameters(
        request: Request,
        body: NodeParametersRequest,
        x_client_id: str | None = Header(default=None),
        x_lease_token: str | None = Header(default=None),
    ) -> ServiceCallResponse:
        enforce_same_origin(request)
        _require_bench_owner(x_client_id, x_lease_token)
        return await _call_ros_service(
            f'{body.node}/set_parameters_atomically',
            'rcl_interfaces/srv/SetParametersAtomically',
            {'parameters': body.parameters},
        )

    # --- rosbag2 (MCAP / sqlite3) Recording Domain ---

    @app.get('/api/recordings', response_model=RecordingListResponse)
    async def list_recordings() -> RecordingListResponse:
        return RecordingListResponse(
            recordings=recorder.list_recordings(),
            status=recorder.status(),
        )

    @app.get('/api/recordings/status', response_model=RecordingStatus)
    async def recording_status() -> RecordingStatus:
        return recorder.status()

    @app.post('/api/recordings/start', response_model=RecordingStatus)
    async def start_recording(request: Request, body: RecordingStartRequest) -> RecordingStatus:
        enforce_same_origin(request)
        try:
            return recorder.start(body.name, body.topics)
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail={'error': str(exc)}) from exc
        except FileExistsError as exc:
            raise HTTPException(status_code=409, detail={'error': str(exc)}) from exc
        except ValueError as exc:
            raise HTTPException(status_code=400, detail={'error': str(exc)}) from exc

    @app.post('/api/recordings/stop', response_model=RecordingInfo)
    async def stop_recording(request: Request) -> RecordingInfo:
        enforce_same_origin(request, require_json=False)
        try:
            return recorder.stop()
        except FileNotFoundError as exc:
            raise HTTPException(status_code=404, detail={'error': str(exc)}) from exc

    @app.get('/api/recordings/{name}/download')
    async def download_recording(name: str):
        try:
            path, filename, media_type = recorder.resolve_download(name)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail={'error': str(exc)}) from exc
        except FileNotFoundError as exc:
            raise HTTPException(status_code=404, detail={'error': str(exc)}) from exc
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail={'error': str(exc)}) from exc
        return FileResponse(path, media_type=media_type, filename=filename)

    @app.delete('/api/recordings/{name}')
    async def delete_recording(request: Request, name: str) -> dict[str, bool]:
        enforce_same_origin(request, require_json=False)
        try:
            recorder.delete(name)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail={'error': str(exc)}) from exc
        except FileNotFoundError as exc:
            raise HTTPException(status_code=404, detail={'error': str(exc)}) from exc
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail={'error': str(exc)}) from exc
        return {'ok': True}

    # --- Model & Action File Domain ---

    @app.get('/api/models')
    async def get_models() -> dict[str, list[str]]:
        return {'models': enumerate_arm_models(models_root)}

    @app.get('/api/motor-specs')
    async def get_motor_specs() -> dict[str, Any]:
        return {'motor_types': motor_library()}

    @app.get('/api/motor-params')
    async def get_motor_params(model: str = Query(default='')) -> dict[str, Any]:
        try:
            return model_motor_params(model, models_root)
        except ValueError as exc:
            raise HTTPException(status_code=404, detail={'error': str(exc)}) from exc

    @app.get('/api/model-transmissions')
    async def get_model_transmissions(model: str = Query(default='')) -> dict[str, Any]:
        try:
            return model_transmissions(model, models_root)
        except ValueError as exc:
            raise HTTPException(status_code=404, detail={'error': str(exc)}) from exc

    @app.get('/api/actions')
    async def get_actions() -> dict[str, Any]:
        return {'actions': list_actions(str(resolved_web_root))}

    @app.get('/api/actions/{name:path}')
    async def get_action(name: str) -> Response:
        try:
            text = read_action(str(resolved_web_root), name)
        except FileNotFoundError as exc:
            raise HTTPException(status_code=404, detail={'error': '动作不存在'}) from exc
        except ValueError as exc:
            raise HTTPException(status_code=400, detail={'error': str(exc)}) from exc
        return Response(content=text, media_type='application/json; charset=utf-8')

    @app.post('/api/actions', response_model=ActionSaveResponse)
    async def post_action(request: Request, body: ActionSaveRequest) -> ActionSaveResponse:
        enforce_same_origin(request, max_bytes=_MAX_ACTION_BYTES)
        try:
            entry = save_action(
                str(resolved_web_root), body.name, body.text, overwrite=body.overwrite,
            )
        except FileExistsError as exc:
            raise HTTPException(
                status_code=409,
                detail={'error': '同名动作已存在；确认覆盖后重试', 'exists': True},
            ) from exc
        except ValueError as exc:
            raise HTTPException(status_code=400, detail={'error': str(exc)}) from exc
        except OSError as exc:
            raise HTTPException(status_code=500, detail={'error': f'动作库写入失败：{exc}'}) from exc
        return ActionSaveResponse(ok=True, action=entry)

    # --- Motion Retargeting & VDMocap Domain ---

    @app.get('/api/motion/sources')
    async def get_motion_sources() -> dict[str, list[str]]:
        return {'sources': motion_jobs.sources()}

    @app.post('/api/motion/sources/status')
    async def post_motion_sources_status(request: Request) -> dict[str, Any]:
        enforce_same_origin(request, max_bytes=_MAX_IMPORT_BYTES)
        try:
            payload = await request.json()
        except Exception as exc:
            raise HTTPException(status_code=400, detail={'error': 'invalid JSON request'}) from exc
        if not isinstance(payload, dict):
            raise HTTPException(status_code=400, detail={'error': '动作请求必须是 JSON 对象'})
        payload.pop('bvh_text', None)
        try:
            return {'sources': source_statuses(motion_jobs, payload)}
        except ImportError as exc:
            raise HTTPException(
                status_code=503,
                detail={'error': '转换组件未就绪，请构建 bvh_broadcaster 并加载 ROS 工作区环境'},
            ) from exc
        except Exception as exc:
            raise HTTPException(status_code=400, detail={'error': str(exc)}) from exc

    @app.post('/api/motion/jobs', status_code=202)
    async def create_motion_job(request: Request) -> dict[str, Any]:
        enforce_same_origin(request, max_bytes=_MAX_IMPORT_BYTES)
        try:
            payload = await request.json()
        except Exception as exc:
            raise HTTPException(status_code=400, detail={'error': 'invalid JSON request'}) from exc
        if not isinstance(payload, dict):
            raise HTTPException(status_code=400, detail={'error': '动作请求必须是 JSON 对象'})
        try:
            return motion_jobs.create(payload)
        except ImportError as exc:
            raise HTTPException(
                status_code=503,
                detail={'error': '转换组件未就绪，请构建 bvh_broadcaster 并加载 ROS 工作区环境'},
            ) from exc
        except Exception as exc:
            raise HTTPException(status_code=400, detail={'error': str(exc)}) from exc

    @app.post('/api/motion/jobs/{job_id}/convert', status_code=202)
    async def convert_motion_job(request: Request, job_id: str) -> dict[str, Any]:
        enforce_same_origin(request, max_bytes=_MAX_IMPORT_BYTES)
        try:
            payload = await request.json()
        except Exception as exc:
            raise HTTPException(status_code=400, detail={'error': 'invalid JSON request'}) from exc
        if not isinstance(payload, dict):
            raise HTTPException(status_code=400, detail={'error': '动作请求必须是 JSON 对象'})
        if type(payload.get('remember_mapping', False)) is not bool:
            raise HTTPException(status_code=400, detail={'error': '记住映射选项必须为布尔值'})
        try:
            return motion_jobs.start(
                job_id,
                payload.get('mapping'),
                remember=payload.get('remember_mapping', False),
            )
        except FileNotFoundError as exc:
            raise HTTPException(status_code=404, detail={'error': str(exc)}) from exc
        except Exception as exc:
            raise HTTPException(status_code=400, detail={'error': str(exc)}) from exc

    @app.get('/api/motion/jobs/{job_id}')
    async def get_motion_job(job_id: str) -> dict[str, Any]:
        try:
            return motion_jobs.status(job_id)
        except FileNotFoundError as exc:
            raise HTTPException(status_code=404, detail={'error': str(exc)}) from exc
        except Exception as exc:
            raise HTTPException(status_code=400, detail={'error': str(exc)}) from exc

    @app.get('/api/motion/jobs/{job_id}/trajectory')
    async def get_motion_trajectory(job_id: str) -> dict[str, Any]:
        try:
            raw = motion_jobs.result(job_id, 'motion.npz')
            return _load_trajectory(raw)
        except FileNotFoundError as exc:
            raise HTTPException(status_code=404, detail={'error': str(exc)}) from exc
        except Exception as exc:
            raise HTTPException(status_code=400, detail={'error': str(exc)}) from exc

    @app.get('/api/motion/jobs/{job_id}/{artifact}')
    async def get_motion_artifact(job_id: str, artifact: str) -> Response:
        try:
            body = motion_jobs.result(job_id, artifact)
        except FileNotFoundError as exc:
            raise HTTPException(status_code=404, detail={'error': str(exc)}) from exc
        except Exception as exc:
            raise HTTPException(status_code=400, detail={'error': str(exc)}) from exc
        content_types = {
            'motion.npz': 'application/octet-stream',
            'robot.urdf': 'application/xml; charset=utf-8',
        }
        headers: dict[str, str] = {}
        if artifact != 'preview.json':
            headers['Content-Disposition'] = f'attachment; filename="{artifact}"'
        return Response(
            content=body,
            media_type=content_types.get(artifact, 'application/json; charset=utf-8'),
            headers=headers,
        )

    @app.post('/api/motion/imports/vdmocap')
    async def import_vdmocap(request: Request) -> dict[str, Any]:
        enforce_same_origin(request, max_bytes=_MAX_IMPORT_BYTES)
        try:
            payload = await request.json()
        except Exception as exc:
            raise HTTPException(status_code=400, detail={'error': 'invalid JSON request'}) from exc
        try:
            return import_capture(motion_jobs, payload)
        except (FileExistsError, ImportConflict) as exc:
            raise HTTPException(status_code=409, detail={'error': str(exc)}) from exc
        except (ValueError, TypeError) as exc:
            raise HTTPException(status_code=400, detail={'error': str(exc)}) from exc
        except ImportError as exc:
            raise HTTPException(
                status_code=503,
                detail={'error': '动捕解算组件未就绪，请加载 ROS 工作区环境'},
            ) from exc
        except OSError as exc:
            raise HTTPException(status_code=500, detail={'error': str(exc)}) from exc

    @app.post('/api/motion/live/vdmocap')
    async def live_vdmocap(request: Request):
        enforce_same_origin(request, max_bytes=4096)
        try:
            payload = await request.json()
        except Exception as exc:
            raise HTTPException(status_code=400, detail={'error': 'invalid JSON request'}) from exc
        model = payload.get('model') if isinstance(payload, dict) else None
        if not isinstance(model, str) or model not in enumerate_arm_models(motion_jobs.models_root):
            raise HTTPException(status_code=400, detail={'error': '请选择模型库中的目标模型'})
        viewer = payload.get('viewer')
        if viewer is not None and (
            not isinstance(viewer, str)
            or len(viewer) != 32
            or any(c not in '0123456789abcdef' for c in viewer)
        ):
            raise HTTPException(status_code=400, detail={'error': '实时预览会话标识无效'})
        stop_ev = live_stream.acquire(viewer)
        if stop_ev is None:
            raise HTTPException(status_code=409, detail={'error': '实时接收端正在使用，请先停止原预览再重试'})

        loop = asyncio.get_running_loop()
        queue: asyncio.Queue[bytes | None] = asyncio.Queue(maxsize=64)

        def _emit(message: dict[str, Any]) -> None:
            line = (
                json.dumps(message, ensure_ascii=False, allow_nan=False, separators=(',', ':'))
                + '\n'
            ).encode('utf-8')
            loop.call_soon_threadsafe(queue.put_nowait, line)

        def _worker() -> None:
            try:
                live_stream.run(model, motion_jobs, _emit, stop_ev)
            except Exception as exc:
                _emit({'type': 'error', 'message': str(exc) or '实时预览已中断'})
            finally:
                live_stream.release()
                loop.call_soon_threadsafe(queue.put_nowait, None)

        threading.Thread(target=_worker, name='vdmocap_live_bff', daemon=True).start()

        async def _gen():
            try:
                while True:
                    chunk = await queue.get()
                    if chunk is None:
                        break
                    yield chunk
            finally:
                stop_ev.set()

        return StreamingResponse(
            _gen(),
            media_type='application/x-ndjson; charset=utf-8',
            headers={
                'Cache-Control': 'no-store',
                'X-Content-Type-Options': 'nosniff',
                'X-Accel-Buffering': 'no',
            },
        )

    # --- Unified WebSocket Hub (/ws) ---

    @app.websocket('/ws')
    async def ws_endpoint(ws: WebSocket, client_id: str | None = None) -> None:
        origin = ws.headers.get('origin')
        if origin is not None and not same_origin(origin, ws.headers.get('host', '')):
            await ws.close(code=1008)
            return

        cid = (client_id or '').strip() or uuid.uuid4().hex[:12]
        await ws.accept()
        hub.add(cid, ws)
        state = lease.set_connected_clients(hub.client_count)
        await ws.send_text(json.dumps({
            'op': 'hello',
            'client_id': cid,
            'control_mode': control_mode,
            'lease': state.model_dump(),
        }))
        try:
            while True:
                raw = await ws.receive_text()
                try:
                    msg = json.loads(raw)
                except json.JSONDecodeError:
                    await ws.send_text(json.dumps({'op': 'error', 'detail': 'invalid_json'}))
                    continue
                op = msg.get('op')
                if op == 'subscribe':
                    requested = [c for c in (msg.get('channels') or []) if isinstance(c, str)]
                    active = hub.subscribe(cid, requested)
                    await ws.send_text(json.dumps({'op': 'subscribed', 'channels': active}))
                    now_mono = time.monotonic()
                    for ch in requested:
                        if ch in CHANNEL_TOPICS and ch != 'telemetry:events':
                            snap, _cnt, rx_hz = gateway.latest_channel(ch)
                            if snap is not None:
                                await ws.send_text(json.dumps({
                                    'channel': ch,
                                    'seq': snap.seq,
                                    'server_ts_ms': int(time.time() * 1000),
                                    'ros_stamp_ns': snap.ros_stamp_ns,
                                    'source_hz': round(rx_hz, 2),
                                    'age_ms': round(
                                        max(0.0, (now_mono - snap.received_mono) * 1000.0), 3,
                                    ),
                                    'data': snap.payload,
                                }, ensure_ascii=False, allow_nan=False, separators=(',', ':')))
                elif op == 'unsubscribe':
                    active = hub.unsubscribe(cid, list(msg.get('channels') or []))
                    await ws.send_text(json.dumps({'op': 'subscribed', 'channels': active}))
                elif op == 'heartbeat':
                    ok = lease.heartbeat(cid, msg.get('token'))
                    await ws.send_text(json.dumps({
                        'op': 'pong',
                        'client_ts_ms': msg.get('client_ts_ms'),
                        'server_ts_ms': int(time.time() * 1000),
                        'lease_ok': ok,
                    }))
                elif op == 'ping':
                    await ws.send_text(json.dumps({
                        'op': 'pong',
                        'client_ts_ms': msg.get('client_ts_ms'),
                        'server_ts_ms': int(time.time() * 1000),
                    }))
                elif op in ('control:joint_target', 'sim:joint_target'):
                    if control_mode != 'bench' or not lease.is_owner(cid, msg.get('token')):
                        await ws.send_text(json.dumps({
                            'op': 'error',
                            'code': 'lease_required',
                            'detail': '未持有控制权或页面只读',
                        }))
                        continue
                    topic = (
                        '/control/joint_target'
                        if op == 'control:joint_target'
                        else '/robot_sim/joint_position_target'
                    )
                    gateway.publish_joint_target(
                        topic,
                        list(msg.get('names') or []),
                        [float(v) for v in (msg.get('positions') or [])],
                        [float(v) for v in (msg.get('velocities') or [])],
                        [float(v) for v in (msg.get('efforts') or [])],
                        msg.get('stamp_ns'),
                    )
        except WebSocketDisconnect:
            pass
        finally:
            hub.remove(cid)
            lease.on_client_disconnected(cid)
            lease.set_connected_clients(hub.client_count)

    # --- Static Files & Model Assets (keep last) ---

    if prefix and resolved_model_dir is not None:
        @app.get(prefix + '/{file_path:path}', include_in_schema=False)
        async def serve_model_asset(file_path: str):
            target = _resolve_under(resolved_model_dir, file_path)
            if target is None or not target.is_file():
                raise HTTPException(status_code=404, detail='Not Found')
            if target.suffix.lower() in ('.stl', '.dae'):
                return FileResponse(
                    target,
                    headers={
                        'Cache-Control': 'public, max-age=86400, immutable',
                        'Content-Encoding': 'identity',
                    },
                )
            return FileResponse(target)

    @app.get('/', include_in_schema=False)
    @app.get('/index.html', include_in_schema=False)
    async def root_redirect(request: Request):
        if (resolved_web_root / 'workbench' / 'index.html').is_file():
            query = request.url.query
            dest = '/workbench/' + ('?' + query if query else '')
            return RedirectResponse(url=dest, status_code=302)
        index_file = resolved_web_root / 'index.html'
        if index_file.is_file():
            return FileResponse(index_file)
        raise HTTPException(status_code=404, detail='Not Found')

    @app.get('/{file_path:path}', include_in_schema=False)
    async def serve_static(file_path: str):
        if file_path == 'api' or file_path.startswith('api/'):
            raise HTTPException(status_code=404, detail={'error': 'unknown API endpoint'})
        target = _resolve_under(resolved_web_root, file_path)
        if target is None:
            raise HTTPException(status_code=404, detail='Not Found')
        if target.is_dir():
            target = target / 'index.html'
        if not target.is_file():
            raise HTTPException(status_code=404, detail='Not Found')
        return FileResponse(target)

    return app
