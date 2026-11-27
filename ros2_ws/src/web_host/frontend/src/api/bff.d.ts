/* eslint-disable */
// Auto-generated from FastAPI OpenAPI schema by scripts/generate_bff_types.py.

export interface ActionSaveRequest {
  name: string;
  text: string;
  overwrite?: boolean;
}

export interface ActionSaveResponse {
  ok: boolean;
  action: Record<string, unknown>;
}

export interface BenchActionRequest {
  payload?: Record<string, unknown>;
}

export interface BenchGravityGainRequest {
  gain: number;
}

export interface BenchUnitLogRequest {
  unit: string;
  lines?: number;
}

export interface DiagnosticsSnapshot {
  uptime_s: number;
  ros_connected: boolean;
  event_loop_lag_ms: number;
  ws_clients: number;
  active_controllers: number;
  ws_broadcast_hz: number;
  topics: Record<string, TopicHealth>;
  recording_active?: boolean;
}

export interface ExecuteConfigRequest {
  request: Record<string, unknown>;
  timeout_ms?: number;
}

export interface HTTPValidationError {
  detail?: ValidationError[];
}

export interface JointTargetRequest {
  names: string[];
  positions: number[];
  velocities?: number[];
  efforts?: number[];
  stamp_ns?: number | null;
}

export interface LeaseAcquireRequest {
  client_id: string;
  label?: string;
  force?: boolean;
  pin?: string | null;
}

export interface LeaseAcquireResponse {
  lease: LeaseState;
  token: string;
  preempted_previous?: boolean;
}

export interface LeaseReleaseRequest {
  client_id: string;
  token?: string | null;
}

export interface LeaseState {
  owner_id?: string | null;
  owner_label?: string | null;
  acquired_at_ms?: number | null;
  expires_at_ms?: number | null;
  active_controllers: number;
  connected_clients: number;
  pin_required?: boolean;
  epoch: number;
}

export interface NodeParametersRequest {
  node: "/bench_manager" | "/workbench_control" | "/firmware_topic_bridge";
  parameters: Record<string, unknown>[];
}

export interface PublishAck {
  accepted: boolean;
  published: boolean;
  topic: string;
  timestamp_ms: number;
}

export interface RecordingInfo {
  name: string;
  storage_format: "mcap" | "sqlite3";
  size_bytes: number;
  duration_s: number;
  message_count: number;
  topics?: string[];
  created_at_ms: number;
}

export interface RecordingListResponse {
  recordings: RecordingInfo[];
  status: RecordingStatus;
}

export interface RecordingStartRequest {
  name?: string;
  topics?: string[];
}

export interface RecordingStatus {
  active: boolean;
  name?: string | null;
  storage_format?: "mcap" | "sqlite3";
  started_at_ms?: number | null;
  duration_s?: number;
  message_count?: number;
  topics?: string[];
}

export interface RuntimeConfig {
  control_mode: "bench" | "readonly";
  robot_model?: string;
  transport?: "bff";
  pin_required?: boolean;
}

export interface ServiceCallResponse {
  ok: boolean;
  service: string;
  result: Record<string, unknown>;
}

export interface StopRequest {
  reason?: string;
  client_id?: string | null;
}

export interface StopResponse {
  accepted: boolean;
  published: boolean;
  reason: string;
  timestamp_ms: number;
}

export interface SubmitPolicyRequest {
  session_id: number;
  source_time_ns?: number;
  targets: Record<string, unknown>[];
}

export interface TopicHealth {
  topic: string;
  channel: string;
  rx_count: number;
  rx_hz: number;
  age_ms?: number | null;
}

export interface TrajectoryPoint {
  positions: number[];
  velocities?: number[];
  accelerations?: number[];
  effort?: number[];
  time_from_start_ms: number;
}

export interface TrajectoryRequest {
  joint_names: string[];
  points: TrajectoryPoint[];
  stamp_ns?: number | null;
}

export interface ValidationError {
  loc: (string | number)[];
  msg: string;
  type: string;
  input?: unknown;
  ctx?: Record<string, unknown>;
}

export interface BffEndpoints {
  "GET /api/actions": { request: never; response: Record<string, unknown> };
  "POST /api/actions": { request: ActionSaveRequest; response: ActionSaveResponse };
  "GET /api/actions/{name}": { request: never; response: unknown };
  "GET /api/bench/agent-status": { request: never; response: ServiceCallResponse };
  "POST /api/bench/agent/start": { request: BenchActionRequest; response: ServiceCallResponse };
  "POST /api/bench/agent/stop": { request: never; response: ServiceCallResponse };
  "POST /api/bench/gravity/gain": { request: BenchGravityGainRequest; response: ServiceCallResponse };
  "POST /api/bench/gravity/start": { request: BenchActionRequest; response: ServiceCallResponse };
  "POST /api/bench/gravity/stop": { request: never; response: ServiceCallResponse };
  "POST /api/bench/motor-backend/command": { request: BenchActionRequest; response: ServiceCallResponse };
  "POST /api/bench/motor-backend/start": { request: BenchActionRequest; response: ServiceCallResponse };
  "POST /api/bench/motor-backend/stop": { request: never; response: ServiceCallResponse };
  "GET /api/bench/options": { request: never; response: ServiceCallResponse };
  "POST /api/bench/simulation/start": { request: BenchActionRequest; response: ServiceCallResponse };
  "POST /api/bench/simulation/stop": { request: never; response: ServiceCallResponse };
  "POST /api/bench/unit-log": { request: BenchUnitLogRequest; response: ServiceCallResponse };
  "POST /api/control/joint-target": { request: JointTargetRequest; response: PublishAck };
  "GET /api/control/lease": { request: never; response: LeaseState };
  "POST /api/control/lease/acquire": { request: LeaseAcquireRequest; response: LeaseAcquireResponse };
  "POST /api/control/lease/release": { request: LeaseReleaseRequest; response: LeaseState };
  "POST /api/control/parameters": { request: NodeParametersRequest; response: ServiceCallResponse };
  "POST /api/control/stop": { request: StopRequest; response: StopResponse };
  "POST /api/control/trajectory": { request: TrajectoryRequest; response: PublishAck };
  "GET /api/diagnostics": { request: never; response: DiagnosticsSnapshot };
  "GET /api/health": { request: never; response: Record<string, string> };
  "GET /api/model-transmissions": { request: never; response: Record<string, unknown> };
  "GET /api/models": { request: never; response: Record<string, string[]> };
  "POST /api/motion/imports/vdmocap": { request: never; response: Record<string, unknown> };
  "POST /api/motion/jobs": { request: never; response: Record<string, unknown> };
  "GET /api/motion/jobs/{job_id}": { request: never; response: Record<string, unknown> };
  "POST /api/motion/jobs/{job_id}/convert": { request: never; response: Record<string, unknown> };
  "GET /api/motion/jobs/{job_id}/trajectory": { request: never; response: Record<string, unknown> };
  "GET /api/motion/jobs/{job_id}/{artifact}": { request: never; response: unknown };
  "POST /api/motion/live/vdmocap": { request: never; response: unknown };
  "GET /api/motion/sources": { request: never; response: Record<string, string[]> };
  "POST /api/motion/sources/status": { request: never; response: Record<string, unknown> };
  "GET /api/motor-params": { request: never; response: Record<string, unknown> };
  "GET /api/motor-specs": { request: never; response: Record<string, unknown> };
  "POST /api/motor/config": { request: ExecuteConfigRequest; response: ServiceCallResponse };
  "GET /api/motor/device-info": { request: never; response: ServiceCallResponse };
  "POST /api/motor/policy": { request: SubmitPolicyRequest; response: ServiceCallResponse };
  "GET /api/recordings": { request: never; response: RecordingListResponse };
  "POST /api/recordings/start": { request: RecordingStartRequest; response: RecordingStatus };
  "GET /api/recordings/status": { request: never; response: RecordingStatus };
  "POST /api/recordings/stop": { request: never; response: RecordingInfo };
  "DELETE /api/recordings/{name}": { request: never; response: Record<string, boolean> };
  "GET /api/recordings/{name}/download": { request: never; response: unknown };
  "GET /api/runtime": { request: never; response: RuntimeConfig };
  "POST /api/sim/joint-target": { request: JointTargetRequest; response: PublishAck };
}
