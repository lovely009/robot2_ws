import type {
  DiagnosticsSnapshot,
  LeaseAcquireResponse,
  LeaseState,
  PublishAck,
  RecordingInfo,
  RecordingListResponse,
  RecordingStatus,
  ServiceCallResponse,
  StopResponse,
} from "./bff";

const CLIENT_ID_KEY = "robot2.bffClientId";

function resolveClientId(): string {
  try {
    const existing = globalThis.sessionStorage?.getItem(CLIENT_ID_KEY);
    if (existing) return existing;
    const created = `web-${Math.random().toString(16).slice(2, 10)}`;
    globalThis.sessionStorage?.setItem(CLIENT_ID_KEY, created);
    return created;
  } catch {
    return `web-${Math.random().toString(16).slice(2, 10)}`;
  }
}

export interface BffSocketCallbacks {
  onConnect?: (hello: { client_id: string; lease: LeaseState }) => void;
  onDisconnect?: (reason: string) => void;
  onLease?: (lease: LeaseState) => void;
  onRtt?: (rttMs: number) => void;
  onTelemetry?: (channel: string, data: unknown, meta: Record<string, unknown>) => void;
  onError?: (detail: { code?: string; message: string }) => void;
}

function extractErrorMessage(payload: any, status: number): string {
  const detail = payload?.detail;
  if (typeof detail === "string" && detail) return detail;
  if (detail && typeof detail === "object") {
    if (typeof detail.error === "string" && detail.error) return detail.error;
    if (typeof detail.message === "string" && detail.message) return detail.message;
  }
  if (typeof payload?.error === "string" && payload.error) return payload.error;
  return `HTTP ${status}`;
}

export class BffClient {
  readonly clientId: string;
  private token: string | null = null;
  private ws: WebSocket | null = null;
  private manualClose = false;
  private backoffMs = 1000;
  private reconnectTimer: ReturnType<typeof setTimeout> | null = null;
  private heartbeatTimer: ReturnType<typeof setInterval> | null = null;
  private desiredChannels = new Set<string>(["lease:state"]);
  private activeChannels = new Set<string>();
  private callbacks: BffSocketCallbacks = {};

  constructor(clientId?: string) {
    this.clientId = clientId || resolveClientId();
  }

  getToken(): string | null {
    return this.token;
  }

  setToken(token: string | null): void {
    this.token = token;
  }

  isConnected(): boolean {
    return this.ws !== null && this.ws.readyState === WebSocket.OPEN;
  }

  private headers(includeJson = true): Record<string, string> {
    const out: Record<string, string> = { "X-Client-Id": this.clientId };
    if (includeJson) out["Content-Type"] = "application/json";
    if (this.token) out["X-Lease-Token"] = this.token;
    return out;
  }

  private async requestJson<T>(
    url: string,
    init: RequestInit = {},
    timeoutMs = 10000,
  ): Promise<T> {
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), timeoutMs);
    try {
      const hasBody = init.body !== undefined;
      const response = await fetch(url, {
        ...init,
        signal: controller.signal,
        headers: {
          ...this.headers(hasBody),
          ...(init.headers as Record<string, string> | undefined),
        },
      });
      const text = await response.text();
      let payload: any = null;
      if (text) {
        try {
          payload = JSON.parse(text);
        } catch {
          payload = { error: text };
        }
      }
      if (!response.ok) {
        throw new Error(extractErrorMessage(payload, response.status));
      }
      return payload as T;
    } finally {
      clearTimeout(timer);
    }
  }

  getLease(): Promise<LeaseState> {
    return this.requestJson<LeaseState>("/api/control/lease");
  }

  async acquireLease(opts: { force?: boolean; pin?: string | null; label?: string } = {}): Promise<LeaseAcquireResponse> {
    const res = await this.requestJson<LeaseAcquireResponse>("/api/control/lease/acquire", {
      method: "POST",
      body: JSON.stringify({
        client_id: this.clientId,
        label: opts.label || this.clientId,
        force: Boolean(opts.force),
        pin: opts.pin ?? null,
      }),
    });
    this.token = res.token;
    return res;
  }

  async releaseLease(): Promise<LeaseState> {
    const res = await this.requestJson<LeaseState>("/api/control/lease/release", {
      method: "POST",
      body: JSON.stringify({
        client_id: this.clientId,
        token: this.token,
      }),
    });
    this.token = null;
    return res;
  }

  triggerStop(reason = "operator_stop"): Promise<StopResponse> {
    return this.requestJson<StopResponse>("/api/control/stop", {
      method: "POST",
      body: JSON.stringify({
        client_id: this.clientId,
        reason,
      }),
    });
  }

  publishJointTarget(
    topic: "/control/joint_target" | "/robot_sim/joint_position_target",
    payload: {
      names: string[];
      positions: number[];
      velocities?: number[];
      efforts?: number[];
      stamp_ns?: number;
    },
  ): Promise<PublishAck> {
    const path = topic === "/robot_sim/joint_position_target"
      ? "/api/sim/joint-target"
      : "/api/control/joint-target";
    return this.requestJson<PublishAck>(path, {
      method: "POST",
      body: JSON.stringify(payload),
    });
  }

  publishTrajectory(payload: {
    joint_names: string[];
    points: {
      positions: number[];
      velocities?: number[];
      accelerations?: number[];
      effort?: number[];
      time_from_start?: { sec?: number; nanosec?: number };
      time_from_start_ms?: number;
    }[];
    stamp_ns?: number;
  }): Promise<PublishAck> {
    const points = (payload.points || []).map((pt) => {
      const ms = typeof pt.time_from_start_ms === "number"
        ? pt.time_from_start_ms
        : Math.max(0, Math.round(
          (Number(pt.time_from_start?.sec || 0) * 1000)
          + (Number(pt.time_from_start?.nanosec || 0) / 1e6),
        ));
      return {
        positions: pt.positions || [],
        velocities: pt.velocities || [],
        accelerations: pt.accelerations || [],
        effort: pt.effort || [],
        time_from_start_ms: ms,
      };
    });
    return this.requestJson<PublishAck>("/api/control/trajectory", {
      method: "POST",
      body: JSON.stringify({
        joint_names: payload.joint_names || [],
        points,
        stamp_ns: payload.stamp_ns ?? null,
      }),
    });
  }

  async callDomainService(
    serviceName: string,
    request: Record<string, any>,
    timeoutMs = 8000,
  ): Promise<Record<string, any>> {
    const post = (url: string, body: unknown) =>
      this.requestJson<ServiceCallResponse>(
        url,
        { method: "POST", body: JSON.stringify(body) },
        timeoutMs,
      ).then((r) => r.result);

    switch (serviceName) {
      case "/motor/get_device_info": {
        const motorId = Number(request?.motor_id ?? 0);
        const res = await this.requestJson<ServiceCallResponse>(
          `/api/motor/device-info?motor_id=${encodeURIComponent(String(motorId))}`,
          {},
          timeoutMs,
        );
        return res.result;
      }
      case "/motor/submit_policy": {
        const sec = Number(request?.header?.stamp?.sec || 0);
        const nsec = Number(request?.header?.stamp?.nanosec || 0);
        const sourceTimeNs = (sec * 1_000_000_000) + nsec;
        return post("/api/motor/policy", {
          session_id: Number(request?.session_id || 0),
          source_time_ns: sourceTimeNs,
          targets: request?.targets || [],
        });
      }
      case "/motor/execute_config":
        return post("/api/motor/config", {
          request: request?.request || {},
          timeout_ms: Number(request?.timeout_ms ?? timeoutMs),
        });
      case "/bench/options":
        return (await this.requestJson<ServiceCallResponse>("/api/bench/options", {}, timeoutMs)).result;
      case "/bench/agent_status":
        return (await this.requestJson<ServiceCallResponse>("/api/bench/agent-status", {}, timeoutMs)).result;
      case "/bench/get_unit_log":
        return post("/api/bench/unit-log", {
          unit: String(request?.unit || ""),
          lines: Number(request?.lines ?? 80),
        });
      case "/bench/start_simulation":
        return post("/api/bench/simulation/start", { payload: request || {} });
      case "/bench/stop_simulation":
        return post("/api/bench/simulation/stop", {});
      case "/bench/start_agent":
        return post("/api/bench/agent/start", { payload: request || {} });
      case "/bench/stop_agent":
        return post("/api/bench/agent/stop", {});
      case "/bench/start_motor_backend":
        return post("/api/bench/motor-backend/start", { payload: request || {} });
      case "/bench/stop_motor_backend":
        return post("/api/bench/motor-backend/stop", {});
      case "/bench/motor_backend_command":
        return post("/api/bench/motor-backend/command", { payload: request || {} });
      case "/bench/start_gravity_feedforward":
        return post("/api/bench/gravity/start", { payload: request || {} });
      case "/bench/stop_gravity_feedforward":
        return post("/api/bench/gravity/stop", {});
      case "/bench/set_gravity_feedforward_gain":
        return post("/api/bench/gravity/gain", { gain: Number(request?.gain ?? 1) });
      case "/bench_manager/set_parameters_atomically":
      case "/workbench_control/set_parameters_atomically":
      case "/firmware_topic_bridge/set_parameters_atomically": {
        const node = `/${serviceName.split("/")[1]}` as
          | "/bench_manager"
          | "/workbench_control"
          | "/firmware_topic_bridge";
        return post("/api/control/parameters", {
          node,
          parameters: request?.parameters || [],
        });
      }
      default:
        throw new Error(`未知领域服务：${serviceName}`);
    }
  }

  getDiagnostics(): Promise<DiagnosticsSnapshot> {
    return this.requestJson<DiagnosticsSnapshot>("/api/diagnostics");
  }

  listRecordings(): Promise<RecordingListResponse> {
    return this.requestJson<RecordingListResponse>("/api/recordings");
  }

  startRecording(name = "", topics: string[] = []): Promise<RecordingStatus> {
    return this.requestJson<RecordingStatus>("/api/recordings/start", {
      method: "POST",
      body: JSON.stringify({ name, topics }),
    });
  }

  stopRecording(): Promise<RecordingInfo> {
    return this.requestJson<RecordingInfo>("/api/recordings/stop", {
      method: "POST",
    });
  }

  deleteRecording(name: string): Promise<{ ok: boolean }> {
    return this.requestJson<{ ok: boolean }>(`/api/recordings/${encodeURIComponent(name)}`, {
      method: "DELETE",
    });
  }

  recordingDownloadUrl(name: string): string {
    return `/api/recordings/${encodeURIComponent(name)}/download`;
  }

  connect(callbacks: BffSocketCallbacks): void {
    this.callbacks = callbacks;
    this.manualClose = false;
    this.backoffMs = 1000;
    this.openSocket();
  }

  disconnect(): void {
    this.manualClose = true;
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }
    this.stopHeartbeat();
    this.activeChannels.clear();
    if (this.ws) {
      const sock = this.ws;
      this.ws = null;
      try {
        sock.close();
      } catch {
        /* ignore */
      }
    }
  }

  setChannels(channels: string[]): void {
    const next = new Set<string>(["lease:state", ...channels.filter(Boolean)]);
    const toAdd = [...next].filter((c) => !this.desiredChannels.has(c));
    const toRemove = [...this.desiredChannels].filter((c) => !next.has(c));
    this.desiredChannels = next;
    if (!this.isConnected()) return;
    if (toAdd.length) this.sendRaw({ op: "subscribe", channels: toAdd });
    if (toRemove.length) this.sendRaw({ op: "unsubscribe", channels: toRemove });
  }

  sendJointTarget(
    op: "control:joint_target" | "sim:joint_target",
    payload: {
      names: string[];
      positions: number[];
      velocities?: number[];
      efforts?: number[];
      stamp_ns?: number;
    },
  ): boolean {
    if (this.isConnected()) {
      this.sendRaw({
        op,
        token: this.token,
        names: payload.names,
        positions: payload.positions,
        velocities: payload.velocities || [],
        efforts: payload.efforts || [],
        stamp_ns: payload.stamp_ns ?? null,
      });
      return true;
    }
    const topic = op === "sim:joint_target"
      ? "/robot_sim/joint_position_target"
      : "/control/joint_target";
    void this.publishJointTarget(topic, payload).catch(() => {});
    return true;
  }

  private sendRaw(data: unknown): void {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(data));
    }
  }

  private openSocket(): void {
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }
    this.stopHeartbeat();
    const proto = location.protocol === "https:" ? "wss:" : "ws:";
    const url = `${proto}//${location.host}/ws?client_id=${encodeURIComponent(this.clientId)}`;
    const sock = new WebSocket(url);
    this.ws = sock;

    sock.addEventListener("open", () => {
      if (this.ws !== sock) return;
      this.backoffMs = 1000;
      this.startHeartbeat();
      this.sendRaw({ op: "subscribe", channels: [...this.desiredChannels] });
    });

    sock.addEventListener("message", (event) => {
      if (this.ws !== sock) return;
      let msg: any;
      try {
        msg = JSON.parse(String(event.data));
      } catch {
        return;
      }
      if (msg.op === "hello") {
        if (msg.lease) this.callbacks.onLease?.(msg.lease as LeaseState);
        this.callbacks.onConnect?.(msg);
      } else if (msg.op === "lease:state" && msg.lease) {
        this.callbacks.onLease?.(msg.lease as LeaseState);
      } else if (msg.op === "subscribed" && Array.isArray(msg.channels)) {
        this.activeChannels = new Set(msg.channels);
      } else if (msg.op === "pong") {
        if (typeof msg.client_ts_ms === "number") {
          this.callbacks.onRtt?.(Math.max(0, Date.now() - msg.client_ts_ms));
        }
        if (msg.lease_ok === false && this.token) {
          this.token = null;
        }
      } else if (msg.op === "error") {
        this.callbacks.onError?.({
          code: msg.code,
          message: String(msg.detail || msg.error || "WebSocket 错误"),
        });
      } else if (typeof msg.channel === "string") {
        this.callbacks.onTelemetry?.(msg.channel, msg.data, msg);
      }
    });

    sock.addEventListener("close", () => {
      if (this.ws !== sock) return;
      this.ws = null;
      this.stopHeartbeat();
      this.activeChannels.clear();
      if (this.manualClose) return;
      const delay = this.backoffMs;
      this.backoffMs = Math.min(8000, this.backoffMs * 2);
      this.callbacks.onDisconnect?.(`已断开，${Math.round(delay / 1000)}s 后重连…`);
      this.reconnectTimer = setTimeout(() => {
        this.reconnectTimer = null;
        if (!this.manualClose) this.openSocket();
      }, delay);
    });
  }

  private startHeartbeat(): void {
    this.stopHeartbeat();
    this.heartbeatTimer = setInterval(() => {
      this.sendRaw({
        op: "heartbeat",
        client_ts_ms: Date.now(),
        token: this.token,
      });
    }, 1000);
  }

  private stopHeartbeat(): void {
    if (this.heartbeatTimer) {
      clearInterval(this.heartbeatTimer);
      this.heartbeatTimer = null;
    }
  }
}

export const bffClient = new BffClient();
