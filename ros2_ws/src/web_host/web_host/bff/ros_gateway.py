"""Dedicated MultiThreadedExecutor rclpy gateway + MockRosGateway for the BFF."""
from __future__ import annotations

import array
import collections
from dataclasses import dataclass, field
import json
import math
import threading
import time
from typing import Any, Callable, Protocol

from .schemas import TopicHealth


def _finite_float(value: Any) -> float | None:
    """Convert numeric value to finite float, mapping NaN/Inf to None for valid JSON."""
    f = float(value)
    return f if math.isfinite(f) else None


def _to_plain(value: Any) -> Any:
    """Recursively convert rosidl / ordereddict / array values into JSON primitives."""
    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, (bytes, bytearray)):
        return list(value)
    if isinstance(value, array.array):
        return [_to_plain(v) for v in value]
    if isinstance(value, dict):
        return {str(k): _to_plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_to_plain(v) for v in value]
    if hasattr(value, 'tolist'):
        return _to_plain(value.tolist())
    return str(value)


@dataclass(slots=True)
class TopicSnapshot:
    """Latest sample received on a telemetry channel."""

    seq: int
    received_mono: float
    ros_stamp_ns: int
    payload: dict[str, Any] | None = None
    _raw_msg: Any = None
    _converter: Callable[[Any], dict[str, Any]] | None = None


def _header_to_dict(header: Any) -> dict[str, Any]:
    stamp = header.stamp
    return {
        'stamp': {'sec': int(stamp.sec), 'nanosec': int(stamp.nanosec)},
        'frame_id': str(header.frame_id),
    }


def _motor_motion_to_dict(msg: Any) -> dict[str, Any]:
    return {
        'header': _header_to_dict(msg.header),
        'states': [
            {
                'motor_id': int(s.motor_id),
                'position': _finite_float(s.position),
                'velocity': _finite_float(s.velocity),
                'effort': _finite_float(s.effort),
                'state_age_ms': int(s.state_age_ms),
                'target_source': int(s.target_source),
            }
            for s in msg.states
        ],
        'session_id': int(msg.session_id),
        'tick_ms': int(msg.tick_ms),
        'seq': int(msg.seq),
    }


def _motor_state_to_dict(msg: Any) -> dict[str, Any]:
    diag = msg.diagnostics
    sys_st = msg.system_status
    return {
        'header': _header_to_dict(msg.header),
        'schema_version': int(msg.schema_version),
        'session_id': int(msg.session_id),
        'snapshot_tick_ms': int(msg.snapshot_tick_ms),
        'sequence': int(msg.sequence),
        'info_revision': int(msg.info_revision),
        'command_sequence': int(msg.command_sequence),
        'stop_generation': int(msg.stop_generation),
        'required_policy_generation': int(msg.required_policy_generation),
        'lease_valid': bool(msg.lease_valid),
        'lease_id': int(msg.lease_id),
        'lease_sequence': int(msg.lease_sequence),
        'lease_valid_until_tick_ms': int(msg.lease_valid_until_tick_ms),
        'lease_active': bool(msg.lease_active),
        'clock_valid': bool(msg.clock_valid),
        'clock_sync_age_ms': int(msg.clock_sync_age_ms),
        'clock_sync_failures': int(msg.clock_sync_failures),
        'clock_offset_ns': int(msg.clock_offset_ns),
        'control_states': [
            {
                'motor_id': int(c.motor_id),
                'online': bool(c.online),
                'control_lifecycle': int(c.control_lifecycle),
                'execution_state': int(c.execution_state),
                'policy_enabled': bool(c.policy_enabled),
                'motion_enabled': bool(c.motion_enabled),
                'state_age_ms': int(c.state_age_ms),
                'unified_fault_code': int(c.unified_fault_code),
                'vendor_fault_code': int(c.vendor_fault_code),
                'last_failure_reason': int(c.last_failure_reason),
                'policy_generation': int(c.policy_generation),
            }
            for c in msg.control_states
        ],
        'statuses': [
            {
                'motor_id': int(s.motor_id),
                'temperature': _finite_float(s.temperature),
                'current': _finite_float(s.current),
                'voltage': _finite_float(s.voltage),
                'drive_mode': int(s.drive_mode),
                'unified_fault_code': int(s.unified_fault_code),
                'vendor_fault_code': int(s.vendor_fault_code),
                'protection_flags': int(s.protection_flags),
                'limit_flags': int(s.limit_flags),
                'device_protect_known': bool(s.device_protect_known),
                'device_protect_enabled': bool(s.device_protect_enabled),
                'device_direction_known': bool(s.device_direction_known),
                'device_direction': int(s.device_direction),
                'online': bool(s.online),
                'lifecycle': int(s.lifecycle),
                'bus_index': int(s.bus_index),
            }
            for s in msg.statuses
        ],
        'diagnostics': {
            'rx_frame_count': int(diag.rx_frame_count),
            'unknown_frame_count': int(diag.unknown_frame_count),
            'decode_error_count': int(diag.decode_error_count),
            'snapshot_accepted': int(diag.snapshot_accepted),
            'snapshot_rejected': int(diag.snapshot_rejected),
            'config_rejected': int(diag.config_rejected),
            'event_dropped': int(diag.event_dropped),
            'tx_dropped': int(diag.tx_dropped),
            'p0_retry': int(diag.p0_retry),
            'trajectory_accepted': int(diag.trajectory_accepted),
            'trajectory_rejected': int(diag.trajectory_rejected),
            'policy_accepted': int(diag.policy_accepted),
            'policy_rejected': int(diag.policy_rejected),
            'ring_drop': [int(v) for v in diag.ring_drop],
            'heap_free': int(diag.heap_free),
            'mailbox_rejected_command': int(diag.mailbox_rejected_command),
            'mailbox_rejected_trajectory': int(diag.mailbox_rejected_trajectory),
            'mailbox_rejected_policy': int(diag.mailbox_rejected_policy),
            'publish_fail': int(diag.publish_fail),
            'rx_hw_lost': [int(v) for v in diag.rx_hw_lost],
            'tx_frame_count': [int(v) for v in diag.tx_frame_count],
        },
        'system_status': {
            'sync_time_ns': int(sys_st.sync_time_ns),
            'last_error': int(sys_st.last_error),
            'last_error_motor': int(sys_st.last_error_motor),
            'refresh_seq': int(sys_st.refresh_seq),
            'set_zero_seq': int(sys_st.set_zero_seq),
            'clear_fault_seq': int(sys_st.clear_fault_seq),
            'save_seq': int(sys_st.save_seq),
            'save_in_progress': bool(sys_st.save_in_progress),
            'firmware_values_valid': bool(sys_st.firmware_values_valid),
            'firmware_values': [int(v) for v in sys_st.firmware_values],
            'firmware_unsaved': bool(sys_st.firmware_unsaved),
        },
    }


@dataclass(slots=True)
class EventItem:
    """Lossless event entry from /motor/events."""

    seq: int
    received_mono: float
    payload: dict[str, Any]


CHANNEL_TOPICS: dict[str, tuple[str, str]] = {
    'telemetry:control': ('/control/status', 'std_msgs/msg/String'),
    'telemetry:transmission': ('/robot_transmission/status', 'std_msgs/msg/String'),
    'telemetry:bench': ('/bench/status', 'std_msgs/msg/String'),
    'telemetry:sim_position': ('/robot_sim/position_status', 'std_msgs/msg/String'),
    'telemetry:joints': ('/joint_states', 'sensor_msgs/msg/JointState'),
    'telemetry:imu': ('/imu/data', 'sensor_msgs/msg/Imu'),
    'telemetry:imu_temp': ('/imu/temp_raw', 'sensor_msgs/msg/Temperature'),
    'telemetry:motor_motion': ('/motor/motion', 'can_bridge_msgs/msg/MotorMotionStateArray'),
    'telemetry:motor_state': ('/motor/state', 'can_bridge_msgs/msg/MotorSystemState'),
    'telemetry:events': ('/motor/events', 'can_bridge_msgs/msg/MotorEvent'),
}


def _has_robot_sim() -> bool:
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory('robot_sim')
        return True
    except Exception:
        return False


class RosGatewayProtocol(Protocol):
    """Protocol implemented by both RosGatewayThread and MockRosGateway."""

    def start(self) -> None: ...
    def stop(self) -> None: ...
    def is_ready(self) -> bool: ...
    def publish_client_count(self, count: int) -> bool: ...
    def publish_stop(self) -> bool: ...
    def publish_joint_target(
        self,
        topic: str,
        names: list[str],
        positions: list[float],
        velocities: list[float],
        efforts: list[float],
        stamp_ns: int | None = None,
        frame_id: str = '',
    ) -> bool: ...
    def publish_trajectory(
        self,
        joint_names: list[str],
        points: list[dict[str, Any]],
        stamp_ns: int | None = None,
        frame_id: str = '',
    ) -> bool: ...
    def call_service(
        self,
        service_name: str,
        service_type: str,
        request_dict: dict[str, Any],
        timeout_s: float = 8.0,
    ) -> dict[str, Any]: ...
    def latest_channel(self, channel: str) -> tuple[TopicSnapshot | None, int, float]: ...
    def events_since(self, last_seq: int) -> list[EventItem]: ...
    def topic_health(self) -> dict[str, TopicHealth]: ...
    def set_raw_sink(self, sink: Callable[[str, str, bytes, int], None] | None) -> None: ...


class _ChannelStore:
    """Thread-safe snapshot + rolling Hz store shared by RosGatewayThread and MockRosGateway."""

    def __init__(self, *, enable_sim: bool = True) -> None:
        self._lock = threading.Lock()
        self._channels: dict[str, tuple[str, str]] = {
            ch: spec
            for ch, spec in CHANNEL_TOPICS.items()
            if enable_sim or ch != 'telemetry:sim_position'
        }
        self._snapshots: dict[str, TopicSnapshot] = {}
        self._rx_counts: dict[str, int] = {ch: 0 for ch in self._channels}
        self._rx_times: dict[str, collections.deque[float]] = {
            ch: collections.deque() for ch in self._channels
        }
        self._events: collections.deque[EventItem] = collections.deque(maxlen=512)
        self._event_seq = 0

    def record_snapshot(
        self,
        channel: str,
        payload: dict[str, Any],
        *,
        ros_stamp_ns: int | None = None,
    ) -> TopicSnapshot:
        now = time.monotonic()
        stamp_ns = ros_stamp_ns if ros_stamp_ns is not None else time.time_ns()
        with self._lock:
            count = self._rx_counts.get(channel, 0) + 1
            self._rx_counts[channel] = count
            dq = self._rx_times.setdefault(channel, collections.deque())
            dq.append(now)
            cutoff = now - 1.0
            while dq and dq[0] < cutoff:
                dq.popleft()
            snap = TopicSnapshot(
                seq=count,
                received_mono=now,
                ros_stamp_ns=stamp_ns,
                payload=payload,
            )
            self._snapshots[channel] = snap
            return snap

    def record_lazy_snapshot(
        self,
        channel: str,
        raw_msg: Any,
        converter: Callable[[Any], dict[str, Any]],
        *,
        ros_stamp_ns: int | None = None,
    ) -> TopicSnapshot:
        now = time.monotonic()
        stamp_ns = ros_stamp_ns if ros_stamp_ns is not None else time.time_ns()
        with self._lock:
            count = self._rx_counts.get(channel, 0) + 1
            self._rx_counts[channel] = count
            dq = self._rx_times.setdefault(channel, collections.deque())
            dq.append(now)
            cutoff = now - 1.0
            while dq and dq[0] < cutoff:
                dq.popleft()
            snap = TopicSnapshot(
                seq=count,
                received_mono=now,
                ros_stamp_ns=stamp_ns,
                payload=None,
                _raw_msg=raw_msg,
                _converter=converter,
            )
            self._snapshots[channel] = snap
            return snap

    def record_event(self, payload: dict[str, Any]) -> EventItem:
        now = time.monotonic()
        with self._lock:
            self._rx_counts['telemetry:events'] = self._rx_counts.get('telemetry:events', 0) + 1
            dq = self._rx_times.setdefault('telemetry:events', collections.deque())
            dq.append(now)
            cutoff = now - 1.0
            while dq and dq[0] < cutoff:
                dq.popleft()
            self._event_seq += 1
            item = EventItem(seq=self._event_seq, received_mono=now, payload=payload)
            self._events.append(item)
            return item

    def latest_channel(self, channel: str) -> tuple[TopicSnapshot | None, int, float]:
        now = time.monotonic()
        with self._lock:
            dq = self._rx_times.get(channel)
            if dq is not None:
                cutoff = now - 1.0
                while dq and dq[0] < cutoff:
                    dq.popleft()
                hz = float(len(dq))
            else:
                hz = 0.0
            snap = self._snapshots.get(channel)
            if snap is not None and snap.payload is None and snap._raw_msg is not None and snap._converter is not None:
                snap.payload = snap._converter(snap._raw_msg)
                snap._raw_msg = None
                snap._converter = None
            return snap, self._rx_counts.get(channel, 0), hz

    def events_since(self, last_seq: int) -> list[EventItem]:
        with self._lock:
            return [item for item in self._events if item.seq > last_seq]

    def topic_health(self) -> dict[str, TopicHealth]:
        now = time.monotonic()
        out: dict[str, TopicHealth] = {}
        with self._lock:
            for ch, (topic, _msg_type) in self._channels.items():
                dq = self._rx_times[ch]
                cutoff = now - 1.0
                while dq and dq[0] < cutoff:
                    dq.popleft()
                snap = self._snapshots.get(ch)
                age_ms: float | None = None
                if ch == 'telemetry:events' and self._events:
                    age_ms = round(max(0.0, (now - self._events[-1].received_mono) * 1000.0), 2)
                elif snap is not None:
                    age_ms = round(max(0.0, (now - snap.received_mono) * 1000.0), 2)
                out[ch] = TopicHealth(
                    topic=topic,
                    channel=ch,
                    rx_count=self._rx_counts[ch],
                    rx_hz=float(len(dq)),
                    age_ms=age_ms,
                )
        return out


@dataclass
class RecordedPublish:
    topic: str
    payload: dict[str, Any]


@dataclass
class RecordedServiceCall:
    service: str
    service_type: str
    request: dict[str, Any]


class MockRosGateway:
    """In-memory RosGatewayProtocol double for deterministic unit and API tests."""

    def __init__(self, *, enable_sim: bool = True) -> None:
        self.started = False
        self.stopped = False
        self.client_counts: list[int] = []
        self.stop_count = 0
        self.publishes: list[RecordedPublish] = []
        self.service_calls: list[RecordedServiceCall] = []
        self.events_log: list[str] = []
        self._store = _ChannelStore(enable_sim=enable_sim)
        self._service_handlers: dict[str, Callable[[dict[str, Any]], dict[str, Any]]] = {}
        self._sink_lock = threading.Lock()
        self._raw_sink: Callable[[str, str, bytes, int], None] | None = None

    def start(self) -> None:
        self.started = True
        self.stopped = False
        self.publish_client_count(0)

    def stop(self) -> None:
        self.publish_client_count(0)
        self.stopped = True

    def is_ready(self) -> bool:
        return self.started and not self.stopped

    def publish_client_count(self, count: int) -> bool:
        self.client_counts.append(int(count))
        self.events_log.append(f'count:{count}')
        return True

    def publish_stop(self) -> bool:
        self.stop_count += 1
        self.events_log.append('stop')
        return True

    def publish_joint_target(
        self,
        topic: str,
        names: list[str],
        positions: list[float],
        velocities: list[float],
        efforts: list[float],
        stamp_ns: int | None = None,
        frame_id: str = '',
    ) -> bool:
        self.publishes.append(RecordedPublish(
            topic=topic,
            payload={
                'names': list(names),
                'positions': list(positions),
                'velocities': list(velocities),
                'efforts': list(efforts),
                'stamp_ns': stamp_ns,
                'frame_id': str(frame_id or ''),
            },
        ))
        return True

    def publish_trajectory(
        self,
        joint_names: list[str],
        points: list[dict[str, Any]],
        stamp_ns: int | None = None,
        frame_id: str = '',
    ) -> bool:
        self.publishes.append(RecordedPublish(
            topic='/control/trajectory',
            payload={
                'joint_names': list(joint_names),
                'points': list(points),
                'stamp_ns': stamp_ns,
                'frame_id': str(frame_id or ''),
            },
        ))
        return True

    def register_service_handler(
        self,
        service_name: str,
        handler: Callable[[dict[str, Any]], dict[str, Any]],
    ) -> None:
        self._service_handlers[service_name] = handler

    def call_service(
        self,
        service_name: str,
        service_type: str,
        request_dict: dict[str, Any],
        timeout_s: float = 8.0,
    ) -> dict[str, Any]:
        self.service_calls.append(RecordedServiceCall(
            service=service_name,
            service_type=service_type,
            request=dict(request_dict),
        ))
        handler = self._service_handlers.get(service_name)
        if handler is not None:
            return handler(request_dict)
        return {'ok': True}

    def inject_channel(
        self,
        channel: str,
        payload: dict[str, Any],
        *,
        ros_stamp_ns: int | None = None,
        raw_bytes: bytes | None = None,
    ) -> TopicSnapshot:
        snap = self._store.record_snapshot(channel, payload, ros_stamp_ns=ros_stamp_ns)
        if raw_bytes is not None and channel in CHANNEL_TOPICS:
            topic, msg_type = CHANNEL_TOPICS[channel]
            with self._sink_lock:
                sink = self._raw_sink
            if sink is not None:
                sink(topic, msg_type, raw_bytes, snap.ros_stamp_ns)
        return snap

    def inject_event(self, payload: dict[str, Any]) -> EventItem:
        return self._store.record_event(payload)

    def latest_channel(self, channel: str) -> tuple[TopicSnapshot | None, int, float]:
        return self._store.latest_channel(channel)

    def events_since(self, last_seq: int) -> list[EventItem]:
        return self._store.events_since(last_seq)

    def topic_health(self) -> dict[str, TopicHealth]:
        return self._store.topic_health()

    def set_raw_sink(self, sink: Callable[[str, str, bytes, int], None] | None) -> None:
        with self._sink_lock:
            self._raw_sink = sink


class RosGatewayThread:
    """Runs an isolated rclpy Context + MultiThreadedExecutor on a daemon thread."""

    def __init__(self, node_name: str = 'web_host_bff', *, enable_sim: bool | None = None) -> None:
        self._node_name = node_name
        self._enable_sim = _has_robot_sim() if enable_sim is None else bool(enable_sim)
        self._stop_event = threading.Event()
        self._ready_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._start_error: BaseException | None = None

        self._pub_lock = threading.Lock()
        self._client_lock = threading.Lock()
        self._sink_lock = threading.Lock()
        self._closed = False

        self._ctx = None
        self._node = None
        self._executor = None
        self._cb_group = None
        self._clients: dict[tuple[str, str], Any] = {}
        self._raw_sink: Callable[[str, str, bytes, int], None] | None = None

        self._client_count_pub = None
        self._stop_pub = None
        self._joint_target_pubs: dict[str, Any] = {}
        self._trajectory_pub = None

        self._Int32 = None
        self._Empty = None
        self._JointState = None
        self._JointTrajectory = None
        self._JointTrajectoryPoint = None
        self._serialize_message = None

        self._store = _ChannelStore(enable_sim=self._enable_sim)

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._ready_event.clear()
        self._start_error = None
        self._closed = False
        self._thread = threading.Thread(
            target=self._run,
            name=f'{self._node_name}_rclpy',
            daemon=True,
        )
        self._thread.start()
        if not self._ready_event.wait(timeout=5.0):
            raise RuntimeError('rclpy gateway thread startup timed out')
        if self._start_error is not None:
            raise RuntimeError('rclpy gateway thread failed to start') from self._start_error
        self.publish_client_count(0)

    def stop(self) -> None:
        if self._thread is None:
            return
        self.publish_client_count(0)
        self._stop_event.set()
        self._thread.join(timeout=2.5)
        self._thread = None

    def is_ready(self) -> bool:
        return (
            self._ready_event.is_set()
            and not self._closed
            and self._thread is not None
            and self._thread.is_alive()
        )

    def set_raw_sink(self, sink: Callable[[str, str, bytes, int], None] | None) -> None:
        with self._sink_lock:
            self._raw_sink = sink

    def latest_channel(self, channel: str) -> tuple[TopicSnapshot | None, int, float]:
        return self._store.latest_channel(channel)

    def events_since(self, last_seq: int) -> list[EventItem]:
        return self._store.events_since(last_seq)

    def topic_health(self) -> dict[str, TopicHealth]:
        return self._store.topic_health()

    def publish_client_count(self, count: int) -> bool:
        with self._pub_lock:
            if self._closed or self._client_count_pub is None or self._Int32 is None:
                return False
            msg = self._Int32()
            msg.data = int(count)
            self._client_count_pub.publish(msg)
            return True

    def publish_stop(self) -> bool:
        with self._pub_lock:
            if self._closed or self._stop_pub is None or self._Empty is None:
                return False
            self._stop_pub.publish(self._Empty())
            return True

    def _fill_stamp(self, msg: Any, stamp_ns: int | None) -> None:
        if stamp_ns is not None and stamp_ns > 0:
            msg.header.stamp.sec = int(stamp_ns // 1_000_000_000)
            msg.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
        elif self._node is not None:
            msg.header.stamp = self._node.get_clock().now().to_msg()

    def publish_joint_target(
        self,
        topic: str,
        names: list[str],
        positions: list[float],
        velocities: list[float],
        efforts: list[float],
        stamp_ns: int | None = None,
        frame_id: str = '',
    ) -> bool:
        with self._pub_lock:
            pub = self._joint_target_pubs.get(topic)
            if self._closed or pub is None or self._JointState is None:
                return False
            msg = self._JointState()
            self._fill_stamp(msg, stamp_ns)
            msg.header.frame_id = str(frame_id or '')
            msg.name = list(names)
            msg.position = [float(v) for v in positions]
            msg.velocity = [float(v) for v in velocities]
            msg.effort = [float(v) for v in efforts]
            pub.publish(msg)
            return True

    def publish_trajectory(
        self,
        joint_names: list[str],
        points: list[dict[str, Any]],
        stamp_ns: int | None = None,
        frame_id: str = '',
    ) -> bool:
        with self._pub_lock:
            if (
                self._closed
                or self._trajectory_pub is None
                or self._JointTrajectory is None
                or self._JointTrajectoryPoint is None
            ):
                return False
            msg = self._JointTrajectory()
            self._fill_stamp(msg, stamp_ns)
            msg.header.frame_id = str(frame_id or '')
            msg.joint_names = list(joint_names)
            built = []
            for pt in points:
                p = self._JointTrajectoryPoint()
                p.positions = [float(v) for v in pt.get('positions', [])]
                p.velocities = [float(v) for v in pt.get('velocities', [])]
                p.accelerations = [float(v) for v in pt.get('accelerations', [])]
                p.effort = [float(v) for v in pt.get('effort', [])]
                ms = max(0, int(pt.get('time_from_start_ms', 0)))
                p.time_from_start.sec = ms // 1000
                p.time_from_start.nanosec = (ms % 1000) * 1_000_000
                built.append(p)
            msg.points = built
            self._trajectory_pub.publish(msg)
            return True

    def call_service(
        self,
        service_name: str,
        service_type: str,
        request_dict: dict[str, Any],
        timeout_s: float = 8.0,
    ) -> dict[str, Any]:
        from rosidl_runtime_py.convert import message_to_ordereddict
        from rosidl_runtime_py.set_message import set_message_fields
        from rosidl_runtime_py.utilities import get_service

        if self._closed or self._node is None:
            raise RuntimeError('ROS 网关尚未就绪')
        srv_cls = get_service(service_type)
        key = (service_name, service_type)
        with self._client_lock:
            client = self._clients.get(key)
            if client is None:
                client = self._node.create_client(
                    srv_cls, service_name, callback_group=self._cb_group,
                )
                self._clients[key] = client
        wait_s = min(2.0, max(0.2, float(timeout_s)))
        if not client.wait_for_service(timeout_sec=wait_s):
            raise TimeoutError(f'服务不可用: {service_name}')

        req = srv_cls.Request()
        if request_dict:
            set_message_fields(req, request_dict)
        done = threading.Event()
        future = client.call_async(req)
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=max(0.2, float(timeout_s))):
            raise TimeoutError(f'服务无响应: {service_name}')
        exc = future.exception()
        if exc is not None:
            raise RuntimeError(f'服务调用失败: {exc}') from exc
        return _to_plain(message_to_ordereddict(future.result()))

    def _emit_raw(self, topic: str, msg_type: str, msg: Any) -> None:
        with self._sink_lock:
            sink = self._raw_sink
        if sink is None or self._serialize_message is None:
            return
        try:
            raw = self._serialize_message(msg)
            sink(topic, msg_type, raw, time.time_ns())
        except Exception:
            pass

    def _make_json_cb(self, channel: str, topic: str, msg_type: str):
        def _cb(msg: Any) -> None:
            try:
                parsed = json.loads(str(msg.data))
                payload = parsed if isinstance(parsed, dict) else {'data': parsed}
            except Exception:
                payload = {'raw': str(getattr(msg, 'data', ''))}
            self._store.record_snapshot(channel, payload)
            self._emit_raw(topic, msg_type, msg)
        return _cb

    def _on_joint_states(self, msg: Any) -> None:
        stamp = msg.header.stamp
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        payload = {
            'names': list(msg.name),
            'positions': [float(v) for v in msg.position],
            'velocities': [float(v) for v in msg.velocity],
            'efforts': [float(v) for v in msg.effort],
            'ros_stamp_ns': stamp_ns,
        }
        self._store.record_snapshot('telemetry:joints', payload, ros_stamp_ns=stamp_ns)
        self._emit_raw('/joint_states', 'sensor_msgs/msg/JointState', msg)

    def _on_imu(self, msg: Any) -> None:
        stamp = msg.header.stamp
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        q = msg.orientation
        av = msg.angular_velocity
        la = msg.linear_acceleration
        payload = {
            'orientation': {'x': float(q.x), 'y': float(q.y), 'z': float(q.z), 'w': float(q.w)},
            'angular_velocity': {'x': float(av.x), 'y': float(av.y), 'z': float(av.z)},
            'linear_acceleration': {'x': float(la.x), 'y': float(la.y), 'z': float(la.z)},
            'ros_stamp_ns': stamp_ns,
        }
        self._store.record_snapshot('telemetry:imu', payload, ros_stamp_ns=stamp_ns)
        self._emit_raw('/imu/data', 'sensor_msgs/msg/Imu', msg)

    def _on_imu_temp(self, msg: Any) -> None:
        payload = {'temperature': float(msg.temperature)}
        self._store.record_snapshot('telemetry:imu_temp', payload)
        self._emit_raw('/imu/temp_raw', 'sensor_msgs/msg/Temperature', msg)

    def _make_lazy_cb(
        self,
        channel: str,
        topic: str,
        msg_type: str,
        converter: Callable[[Any], dict[str, Any]],
    ):
        def _cb(msg: Any) -> None:
            self._store.record_lazy_snapshot(channel, msg, converter)
            self._emit_raw(topic, msg_type, msg)
        return _cb

    def _make_rosidl_cb(self, channel: str, topic: str, msg_type: str, *, is_event: bool = False):
        from rosidl_runtime_py.convert import message_to_ordereddict

        def _cb(msg: Any) -> None:
            payload = _to_plain(message_to_ordereddict(msg))
            if is_event:
                self._store.record_event(payload)
            else:
                self._store.record_snapshot(channel, payload)
            self._emit_raw(topic, msg_type, msg)
        return _cb

    def _run(self) -> None:
        try:
            import rclpy
            from rclpy.callback_groups import ReentrantCallbackGroup
            from rclpy.executors import MultiThreadedExecutor
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
            from rclpy.serialization import serialize_message
            from sensor_msgs.msg import Imu, JointState, Temperature
            from std_msgs.msg import Empty, Int32, String
            from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

            ctx = rclpy.Context()
            rclpy.init(context=ctx)
            node = rclpy.create_node(self._node_name, context=ctx)
            executor = MultiThreadedExecutor(num_threads=4, context=ctx)
            executor.add_node(node)
            cb_group = ReentrantCallbackGroup()

            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            sensor_qos = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            with self._pub_lock:
                self._ctx = ctx
                self._node = node
                self._executor = executor
                self._cb_group = cb_group
                self._Int32 = Int32
                self._Empty = Empty
                self._JointState = JointState
                self._JointTrajectory = JointTrajectory
                self._JointTrajectoryPoint = JointTrajectoryPoint
                self._serialize_message = serialize_message

                self._client_count_pub = node.create_publisher(Int32, '/client_count', latched)
                self._stop_pub = node.create_publisher(Empty, '/control/stop', 10)
                self._joint_target_pubs = {
                    '/control/joint_target': node.create_publisher(JointState, '/control/joint_target', 1),
                }
                if self._enable_sim:
                    self._joint_target_pubs['/robot_sim/joint_position_target'] = node.create_publisher(
                        JointState, '/robot_sim/joint_position_target', 1,
                    )
                self._trajectory_pub = node.create_publisher(JointTrajectory, '/control/trajectory', 10)

            node.create_subscription(
                String, '/control/status',
                self._make_json_cb('telemetry:control', '/control/status', 'std_msgs/msg/String'),
                latched, callback_group=cb_group,
            )
            node.create_subscription(
                String, '/robot_transmission/status',
                self._make_json_cb(
                    'telemetry:transmission', '/robot_transmission/status', 'std_msgs/msg/String',
                ),
                latched, callback_group=cb_group,
            )
            node.create_subscription(
                String, '/bench/status',
                self._make_json_cb('telemetry:bench', '/bench/status', 'std_msgs/msg/String'),
                10, callback_group=cb_group,
            )
            if self._enable_sim:
                node.create_subscription(
                    String, '/robot_sim/position_status',
                    self._make_json_cb(
                        'telemetry:sim_position', '/robot_sim/position_status', 'std_msgs/msg/String',
                    ),
                    10, callback_group=cb_group,
                )
            node.create_subscription(
                JointState, '/joint_states', self._on_joint_states, sensor_qos, callback_group=cb_group,
            )
            node.create_subscription(
                Imu, '/imu/data', self._on_imu, sensor_qos, callback_group=cb_group,
            )
            node.create_subscription(
                Temperature, '/imu/temp_raw', self._on_imu_temp, sensor_qos, callback_group=cb_group,
            )

            try:
                from can_bridge_msgs.msg import MotorEvent, MotorMotionStateArray, MotorSystemState
                node.create_subscription(
                    MotorMotionStateArray, '/motor/motion',
                    self._make_lazy_cb(
                        'telemetry:motor_motion',
                        '/motor/motion',
                        'can_bridge_msgs/msg/MotorMotionStateArray',
                        _motor_motion_to_dict,
                    ),
                    sensor_qos, callback_group=cb_group,
                )
                node.create_subscription(
                    MotorSystemState, '/motor/state',
                    self._make_lazy_cb(
                        'telemetry:motor_state',
                        '/motor/state',
                        'can_bridge_msgs/msg/MotorSystemState',
                        _motor_state_to_dict,
                    ),
                    10, callback_group=cb_group,
                )
                node.create_subscription(
                    MotorEvent, '/motor/events',
                    self._make_rosidl_cb(
                        'telemetry:events',
                        '/motor/events',
                        'can_bridge_msgs/msg/MotorEvent',
                        is_event=True,
                    ),
                    64, callback_group=cb_group,
                )
            except ImportError:
                pass

            self._ready_event.set()
            while not self._stop_event.is_set() and ctx.ok():
                executor.spin_once(timeout_sec=0.05)
        except BaseException as exc:
            self._start_error = exc
            self._ready_event.set()
        finally:
            with self._pub_lock:
                self._closed = True
                executor = self._executor
                node = self._node
                ctx = self._ctx
                self._client_count_pub = None
                self._stop_pub = None
                self._joint_target_pubs = {}
                self._trajectory_pub = None
                self._executor = None
                self._node = None
                self._ctx = None
            with self._client_lock:
                self._clients.clear()
            if executor is not None:
                try:
                    executor.shutdown()
                except Exception:
                    pass
            if node is not None:
                try:
                    node.destroy_node()
                except Exception:
                    pass
            if ctx is not None:
                try:
                    ctx.try_shutdown()
                except Exception:
                    pass
