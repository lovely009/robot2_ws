"""Pydantic v2 domain schemas for the single-port FastAPI + rclpy BFF."""
from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, Field


class RuntimeConfig(BaseModel):
    """Workbench runtime mode and initial model configuration."""

    control_mode: Literal['bench', 'readonly']
    robot_model: str = ''
    transport: Literal['bff'] = 'bff'
    pin_required: bool = False


class LeaseState(BaseModel):
    """Current single-controller lease state."""

    owner_id: str | None = None
    owner_label: str | None = None
    acquired_at_ms: int | None = None
    expires_at_ms: int | None = None
    active_controllers: int = Field(ge=0, le=1)
    connected_clients: int = Field(ge=0)
    pin_required: bool = False
    epoch: int = Field(ge=0)


class LeaseAcquireRequest(BaseModel):
    """Request body to acquire or force-takeover the single control lease."""

    client_id: str = Field(min_length=1, max_length=64)
    label: str = Field(default='', max_length=64)
    force: bool = False
    pin: str | None = None


class LeaseAcquireResponse(BaseModel):
    """Response returned when a client acquires the control lease."""

    lease: LeaseState
    token: str
    preempted_previous: bool = False


class LeaseReleaseRequest(BaseModel):
    """Request body to release an active control lease."""

    client_id: str = Field(min_length=1, max_length=64)
    token: str | None = None


class StopRequest(BaseModel):
    """Request body for operator soft-stop (open to all connected clients)."""

    reason: str = Field(default='operator_stop', max_length=128)
    client_id: str | None = None


class StopResponse(BaseModel):
    """Result of publishing a soft-stop intent to ROS 2."""

    accepted: bool
    published: bool
    reason: str
    timestamp_ms: int


class JointTargetRequest(BaseModel):
    """Joint follow target published to /control/joint_target or /robot_sim/joint_position_target."""

    names: list[str]
    positions: list[float]
    velocities: list[float] = Field(default_factory=list)
    efforts: list[float] = Field(default_factory=list)
    stamp_ns: int | None = None


class TrajectoryPoint(BaseModel):
    """Single trajectory waypoint."""

    positions: list[float]
    velocities: list[float] = Field(default_factory=list)
    accelerations: list[float] = Field(default_factory=list)
    effort: list[float] = Field(default_factory=list)
    time_from_start_ms: int = Field(ge=0)


class TrajectoryRequest(BaseModel):
    """Joint trajectory published to /control/trajectory."""

    joint_names: list[str]
    points: list[TrajectoryPoint]
    stamp_ns: int | None = None


class PublishAck(BaseModel):
    """Acknowledgement for a published domain command."""

    accepted: bool
    published: bool
    topic: str
    timestamp_ms: int


class SubmitPolicyRequest(BaseModel):
    """Domain payload for /motor/submit_policy."""

    session_id: int = Field(ge=0)
    source_time_ns: int = Field(default=0, ge=0)
    targets: list[dict[str, Any]]


class ExecuteConfigRequest(BaseModel):
    """Domain payload for /motor/execute_config."""

    request: dict[str, Any]
    timeout_ms: int = Field(default=10000, ge=500, le=60000)


class ServiceCallResponse(BaseModel):
    """Response wrapper for domainized ROS service calls."""

    ok: bool
    service: str
    result: dict[str, Any]


class BenchActionRequest(BaseModel):
    """Generic payload for bench lifecycle actions."""

    payload: dict[str, Any] = Field(default_factory=dict)


class BenchUnitLogRequest(BaseModel):
    """Request body for reading systemd unit logs via bench_manager."""

    unit: str = Field(min_length=1, max_length=128)
    lines: int = Field(default=80, ge=1, le=2000)


class BenchGravityGainRequest(BaseModel):
    """Request body to set gravity feedforward gain."""

    gain: float


class NodeParametersRequest(BaseModel):
    """Atomic parameter update against an allowlisted ROS 2 node."""

    node: Literal['/bench_manager', '/workbench_control', '/firmware_topic_bridge']
    parameters: list[dict[str, Any]]


class ActionSaveRequest(BaseModel):
    """Request body to save an action JSON file."""

    name: str = Field(min_length=1, max_length=128)
    text: str
    overwrite: bool = False


class ActionSaveResponse(BaseModel):
    """Saved action library entry."""

    ok: bool
    action: dict[str, Any]


class RecordingStartRequest(BaseModel):
    """Request body to start rosbag2 recording."""

    name: str = Field(default='', max_length=96)
    topics: list[str] = Field(default_factory=list)


class RecordingInfo(BaseModel):
    """Metadata for a completed rosbag2 recording."""

    name: str
    storage_format: Literal['mcap', 'sqlite3']
    size_bytes: int = Field(ge=0)
    duration_s: float = Field(ge=0.0)
    message_count: int = Field(ge=0)
    topics: list[str] = Field(default_factory=list)
    created_at_ms: int = Field(ge=0)


class RecordingStatus(BaseModel):
    """Current rosbag2 recorder status."""

    active: bool
    name: str | None = None
    storage_format: Literal['mcap', 'sqlite3'] = 'mcap'
    started_at_ms: int | None = None
    duration_s: float = 0.0
    message_count: int = 0
    topics: list[str] = Field(default_factory=list)


class RecordingListResponse(BaseModel):
    """List of saved recordings plus current active status."""

    recordings: list[RecordingInfo]
    status: RecordingStatus


class TopicHealth(BaseModel):
    """Live per-topic health metrics."""

    topic: str
    channel: str
    rx_count: int = Field(ge=0)
    rx_hz: float = Field(ge=0.0)
    age_ms: float | None = None


class DiagnosticsSnapshot(BaseModel):
    """Runtime health and throughput diagnostics for the BFF gateway."""

    uptime_s: float
    ros_connected: bool
    event_loop_lag_ms: float
    ws_clients: int
    active_controllers: int
    ws_broadcast_hz: float
    topics: dict[str, TopicHealth]
    recording_active: bool = False
