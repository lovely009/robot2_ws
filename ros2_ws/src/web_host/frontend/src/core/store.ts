// 共享状态：所有跨屏数据都住在这里。面板只读它、只通过 ui 动作改它。
// 订阅由 Vue 工作台全局建立并写这里，与屏是否可见无关——所以切屏不丢曲线缓冲与事件列表。
import { createControlStateCache, describeMotorState, type ControlStateCache } from './motor_state';

export function applyAgentWireStatus(state: any, message: any, now = Date.now()): boolean {
  const uint32 = (value: any) => Number.isInteger(value) && value >= 0 && value <= 0xffffffff;
  if (!Number.isSafeInteger(message.session_id) || message.session_id <= 0 ||
      !Number.isInteger(message.schema_version) ||
      ![message.sequence, message.snapshot_tick_ms, message.clock_sync_age_ms,
        message.clock_sync_failures, message.lease_valid_until_tick_ms].every(uint32) ||
      typeof message.clock_valid !== 'boolean' || typeof message.lease_active !== 'boolean') return false;
  const previous = state.agentWireStatus?.data;
  if (previous && (message.session_id < previous.session_id ||
      (message.session_id === previous.session_id && ((message.sequence - previous.sequence) | 0) <= 0))) return false;
  state.agentWireStatus = { at: now, data: message };
  return true;
}

// 曲线数据环形缓冲：O(1) 写入，避免 1500 点数组频繁 Array.shift() 造成 GC。
// 同时通过懒加载只读数组 getter 保持对测试和旧读取方的兼容。
export const CHART_CAP = 1500;

export class ChartChannelBuffer {
  readonly capacity: number;
  private ts: Float64Array;
  private vals: Float64Array;
  private head = 0;
  private _count = 0;
  private version = 0;
  private cachedTVersion = -1;
  private cachedT: number[] = [];
  private cachedValVersion = -1;
  private cachedValues: number[] = [];

  constructor(capacity = CHART_CAP) {
    this.capacity = capacity;
    this.ts = new Float64Array(capacity);
    this.vals = new Float64Array(capacity);
  }

  get length(): number {
    return this._count;
  }

  push(t: number, value: number): void {
    const idx = (this.head + this._count) % this.capacity;
    this.ts[idx] = t;
    this.vals[idx] = Number.isFinite(value) ? value : NaN;
    if (this._count < this.capacity) {
      this._count++;
    } else {
      this.head = (this.head + 1) % this.capacity;
    }
    this.version++;
  }

  tAt(i: number): number {
    return this.ts[(this.head + i) % this.capacity]!;
  }

  valueAt(i: number): number {
    return this.vals[(this.head + i) % this.capacity]!;
  }

  get t(): number[] {
    if (this.cachedTVersion !== this.version) {
      const out = new Array<number>(this._count);
      for (let i = 0; i < this._count; i++) out[i] = this.tAt(i);
      this.cachedT = out;
      this.cachedTVersion = this.version;
    }
    return this.cachedT;
  }

  get values(): number[] {
    if (this.cachedValVersion !== this.version) {
      const out = new Array<number>(this._count);
      for (let i = 0; i < this._count; i++) out[i] = this.valueAt(i);
      this.cachedValues = out;
      this.cachedValVersion = this.version;
    }
    return this.cachedValues;
  }
}

export interface WorkbenchStore {
  runtimeMode?: string;
  conn: {
    connected: boolean;
    text: string;
    url: string;
    agent: string;
    agentDevice: string;
    agentError: string;
    agentReturncode: number | null;
    canStop: boolean;
    lastMsgAt: number;
    autoLoaded: boolean;
    benchLoaded: boolean;
  };
  motors: Map<number, any>;
  controlStates: ControlStateCache;
  agentWireStatus: { at: number; data: any } | null;
  lastError: number;
  lastErrorMotor: number;
  sysStatus: {
    refresh_seq: number;
    set_zero_seq: number;
    clear_fault_seq: number;
    save_seq: number;
    save_in_progress: boolean;
    seen: boolean;
  };
  policy: Map<number, any>;
  events: any[];
  imu: { count: number; last: number; roll: number; pitch: number; yaw: number; q: any };
  temp: number | undefined;
  tempLast: number;
  motionCount: number;
  tf: Map<string, any>;
  modelJoints: any[];
  selectedId: number;
  chartBuf: Map<number, Record<string, ChartChannelBuffer | { t: number[]; values: number[] }>>;
  chart: { running: boolean; windowMs: number };
  history: any[];
  paramCache: Map<number, any>;
  firmwareReport: { seq: number; at: number; values: number[] | null; unsaved: boolean };
  configResults: Map<number, any>;
  configSessionId: string | null;
  results: Map<string, { ok: boolean; text: string; t: number }>;
  traj: {
    activeTrajectoryId: number;
    waitingState: boolean;
    segmentIndex: number;
    segmentCount: number;
    state: any;
  };
  bench: Record<string, any>;
  jointStates: { count: number; last: number; byName: Record<string, any>; jointNames: string[] };
  transmissionStatus: { last: number; data: any };
  modelTransmissions: any;
  selectedGroup: string;
  selectedJoint: string;
  commandStatus: { last: number; data: any };
  simPositionStatus: { last: number; data: any };
}

export const store: WorkbenchStore = {
  conn: {
    connected: false, text: '未连接', url: '', agent: 'stopped', agentDevice: '',
    agentError: '', agentReturncode: null, canStop: false, lastMsgAt: 0,
    autoLoaded: false, benchLoaded: false,
  },
  motors: new Map(),
  controlStates: createControlStateCache(),
  agentWireStatus: null,
  lastError: 0,
  lastErrorMotor: -1,
  sysStatus: {
    refresh_seq: 0, set_zero_seq: 0, clear_fault_seq: 0,
    save_seq: 0, save_in_progress: false, seen: false,
  },
  policy: new Map(),
  events: [],
  imu: { count: 0, last: 0, roll: 0, pitch: 0, yaw: 0, q: null },
  temp: undefined,
  tempLast: 0,
  motionCount: 0,
  tf: new Map(),
  modelJoints: [],
  selectedId: 0,
  chartBuf: new Map(),
  chart: { running: true, windowMs: 10000 },
  history: [],
  paramCache: new Map(),
  firmwareReport: { seq: 0, at: 0, values: null, unsaved: false },
  configResults: new Map(),
  configSessionId: null,
  results: new Map(),
  traj: {
    activeTrajectoryId: 0, waitingState: false, segmentIndex: 0, segmentCount: 0,
    state: null,
  },
  bench: {
    simulation: null, agent: null, motor_backend: null,
    gravity_feedforward: null, gravity_feedforward_last: 0, feedback: null,
  },
  jointStates: { count: 0, last: 0, byName: {}, jointNames: [] },
  transmissionStatus: { last: 0, data: null },
  modelTransmissions: {
    model: '', driver_file: '', groups: [], error: '',
    driverFile: '', directJoints: {}, direct_mapping: {},
  },
  selectedGroup: '',
  selectedJoint: '',
  commandStatus: { last: 0, data: null },
  simPositionStatus: { last: 0, data: null },
};

export function orderedMotorIds(): number[] {
  return [...store.motors.keys()].sort((a, b) => a - b);
}

let dirty = new Set<string>();
let dirtyListener: ((batch: Set<string>) => void) | null = null;
let lastAgeCheck = 0;

export function markDirty(...keys: string[]): void {
  for (const k of keys) dirty.add(k);
}
export function onDirty(fn: ((batch: Set<string>) => void) | null): void { dirtyListener = fn; }

// 单一驱动入口：由 runtime.ts 的唯一 rAF 调度调用，不再自建第二个 rAF 循环。
export function consumeDirty(now = Date.now()): Set<string> | null {
  if (now - lastAgeCheck >= 500) {
    lastAgeCheck = now;
    dirty.add(DIRTY.STATUS);
  }
  if (!dirty.size) return null;
  const batch = dirty;
  dirty = new Set();
  return batch;
}

export function runRenderLoop(): () => void {
  let active = true;
  let frameId = 0;
  const frame = () => {
    if (!active) return;
    const batch = consumeDirty(Date.now());
    if (batch && dirtyListener) dirtyListener(batch);
    frameId = requestAnimationFrame(frame);
  };
  frameId = requestAnimationFrame(frame);
  return () => { active = false; cancelAnimationFrame(frameId); };
}

export const DIRTY = {
  CONN: 'conn', TOPOLOGY: 'topology', MOTION: 'motion', STATUS: 'status',
  EVENTS: 'events', HISTORY: 'history', PARAMS: 'params', TRAJ: 'traj',
  IMU: 'imu', POLICY: 'policy', SELECTED: 'selected', TF: 'tf', MODEL: 'model',
  BENCH: 'bench', COMMAND: 'command',
} as const;

export function writesAllowed(): boolean { return store.conn.connected; }

export function syncWriteButtons(ids: string[], allow: boolean): void {
  for (const id of ids) {
    const el = document.getElementById(id) as HTMLButtonElement | null;
    if (el) el.disabled = !allow;
  }
}

const object = (value: any): value is Record<string, any> =>
  value !== null && typeof value === 'object' && !Array.isArray(value);
const names = (value: any): value is string[] =>
  Array.isArray(value) && value.every(name => typeof name === 'string' && name.trim())
  && new Set(value).size === value.length;
const joints = (value: any): value is string[] => names(value) && value.length > 0;
const optional = (snapshot: any, key: string, valid: (val: any) => boolean) =>
  snapshot[key] === undefined || valid(snapshot[key]);

export function applyCommandStatus(snapshot: any): void {
  const valid = object(snapshot) && typeof snapshot.model === 'string' && snapshot.model.trim()
    && joints(snapshot.joint_names) && typeof snapshot.mode === 'string'
    && snapshot.output_topic === '/motor/command'
    && snapshot.trajectory_output_topic === '/host_control/trajectory_controller/follow_joint_trajectory'
    && typeof snapshot.feedback_fresh === 'boolean' && typeof snapshot.controllers_ready === 'boolean'
    && Array.isArray(snapshot.active_controllers) && snapshot.active_controllers.every((name: any) => typeof name === 'string')
    && (snapshot.gravity_enabled === null || typeof snapshot.gravity_enabled === 'boolean')
    && (snapshot.effort_gain === null || (Number.isFinite(snapshot.effort_gain) && snapshot.effort_gain >= 0))
    && optional(snapshot, 'joint_trajectory_ids', value => typeof value === 'boolean')
    && ['reason', 'reject'].every(key => optional(snapshot, key, value => typeof value === 'string'));
  store.commandStatus = valid ? { data: snapshot, last: Date.now() } : { data: null, last: 0 };
  markDirty(DIRTY.COMMAND);
}

export function applyTransmissionStatus(snapshot: any): void {
  const valid = object(snapshot) && typeof snapshot.model === 'string' && snapshot.model.trim()
    && joints(snapshot.joint_names) && typeof snapshot.trajectory_enabled === 'boolean'
    && ['feedback_fresh', 'controllers_ready', 'joint_trajectory_ids'].every(key => optional(snapshot, key, value => typeof value === 'boolean'))
    && optional(snapshot, 'active_controllers', value => Array.isArray(value) && value.every((name: any) => typeof name === 'string'))
    && optional(snapshot, 'gravity_enabled', value => value === null || typeof value === 'boolean')
    && optional(snapshot, 'effort_gain', value => value === null || (Number.isFinite(value) && value >= 0))
    && optional(snapshot, 'control_groups', groups => {
      if (!Array.isArray(groups)) return false;
      const motorIds: number[] = [], jointNames: string[] = [], groupNames: string[] = [];
      for (const group of groups) {
        if (!object(group) || typeof group.name !== 'string' || !group.name.trim()
            || !joints(group.joints) || !Array.isArray(group.motors) || !group.motors.length
            || group.motors.some((motor: any) => !object(motor) || !Number.isInteger(motor.id) || motor.id < 1 || motor.id > 65535)) return false;
        groupNames.push(group.name); jointNames.push(...group.joints);
        motorIds.push(...group.motors.map((motor: any) => motor.id));
      }
      return names(groupNames) && names(jointNames) && new Set(motorIds).size === motorIds.length
        && snapshot.joint_names.every((name: string) => jointNames.includes(name));
    })
    && optional(snapshot, 'dynamic_joint_names', value => names(value) && value.every((name: string) => snapshot.joint_names.includes(name)))
    && optional(snapshot, 'joint_limits', value => object(value) && Object.entries(value).every(([name, bounds]: [string, any]) =>
      name.trim() && object(bounds) && [bounds.lower, bounds.upper].every(bound => bound === null || Number.isFinite(bound))
      && (bounds.lower === null || bounds.upper === null || bounds.lower <= bounds.upper)))
    && optional(snapshot, 'joint_limit_errors', value => object(value) && Object.entries(value).every(([name, error]) => name.trim() && typeof error === 'string'))
    && ['joint_limit_error', 'last_error', 'control_service', 'trajectory_topic', 'control_state_source', 'driver_file']
      .every(key => optional(snapshot, key, value => typeof value === 'string'));
  store.transmissionStatus = valid ? { data: snapshot, last: Date.now() } : { data: null, last: 0 };
  markDirty(DIRTY.STATUS);
}

export function observeTrajectory(message: any): void {
  store.traj.activeTrajectoryId = message.trajectory_id;
  store.traj.state = {
    trajectory_id: message.trajectory_id, active: !!message.active,
    completed: !!message.completed, elapsed_ms: message.elapsed_ms, total_ms: message.total_ms,
    target_ids: message.target_ids || [],
  };
  markDirty(DIRTY.TRAJ);
}

export function beginTrajectory(id: number, clearState = false): void {
  store.traj.activeTrajectoryId = id;
  if (clearState) store.traj.state = null;
}

export function resetTrajectory(): void {
  store.traj.state = null;
  store.traj.activeTrajectoryId = 0;
}

export function applyBenchStatus(snapshot: any): void {
  if (!snapshot || typeof snapshot !== 'object') return;
  const bench = store.bench;
  if (snapshot.simulation) bench.simulation = snapshot.simulation;
  if (snapshot.motor_backend) bench.motor_backend = snapshot.motor_backend;
  if (snapshot.feedback) bench.feedback = snapshot.feedback;
  if ('gravity_feedforward' in snapshot) {
    const gravity = snapshot.gravity_feedforward;
    const valid = object(gravity)
      && optional(gravity, 'pending', value => value === null || value === 'start' || value === 'stop' || value === 'gain')
      && optional(gravity, 'fresh', value => typeof value === 'boolean')
      && optional(gravity, 'error', value => typeof value === 'string')
      && optional(gravity, 'config', value => value === null || (object(value)
        && optional(value, 'model', model => typeof model === 'string')));
    bench.gravity_feedforward = valid ? gravity : null;
    bench.gravity_feedforward_last = valid ? Date.now() : 0;
  }
  if (snapshot.agent) {
    bench.agent = snapshot.agent;
    const c = store.conn;
    const state = snapshot.agent.state || 'stopped';
    c.agentDevice = snapshot.agent.device || '';
    if (c.agent !== 'error' || state !== 'stopped') {
      c.agent = state;
      c.agentError = snapshot.agent.error || '';
    }
    c.agentReturncode = snapshot.agent.returncode === undefined
      ? null : snapshot.agent.returncode;
    c.canStop = c.agent === 'running' || !!snapshot.agent.canStop;
    markDirty(DIRTY.CONN);
  }
  if (!jointFeedbackMatchesModel()) {
    store.jointStates.byName = {};
    store.jointStates.last = 0;
    markDirty(DIRTY.MOTION);
  }
  markDirty(DIRTY.BENCH);
}

export function countMotors() {
  let online = 0, fault = 0, active = 0, conflict = 0;
  for (const [id, r] of store.motors) {
    const view = describeMotorState(store, id);
    const st = r.status || {}, t = r.topo || {};
    if (view.online) online++;
    if (view.fault) fault++;
    if (view.active) active++;
    if (t.conflict || st.lifecycle === 7) conflict++;
  }
  return { online, total: store.motors.size, fault, active, conflict };
}

export function chartPush(id: number, ch: string, val: number, t: number): void {
  let channels = store.chartBuf.get(id);
  if (!channels) {
    channels = {};
    store.chartBuf.set(id, channels);
  }
  let b = channels[ch];
  if (!b) {
    b = new ChartChannelBuffer(CHART_CAP);
    channels[ch] = b;
  }
  if (b instanceof ChartChannelBuffer) {
    b.push(t, val);
  } else {
    b.values.push(Number.isFinite(val) ? val : NaN);
    b.t.push(t);
    if (b.t.length > CHART_CAP) {
      b.t.shift();
      b.values.shift();
    }
  }
}

export const URLQ = new URLSearchParams(location.search);
const JOINT_OVERRIDE = (() => {
  const q = URLQ.get('joints');
  if (!q) return null;
  const m: Record<number, string> = {};
  for (const pair of q.split(',')) {
    const [id, j] = pair.split(':');
    if (id && j) m[parseInt(id, 10)] = j;
  }
  return m;
})();

export const JOINT_OF_MOTOR: Record<number, string> = JOINT_OVERRIDE || {
  1: 'joint_link1', 2: 'joint_link2', 3: 'joint_link3', 4: 'joint_link4',
  5: 'joint_link5', 6: 'joint_link6', 7: 'joint_link7', 8: 'joint_link8',
  9: 'joint_link9',
  10: 'joint_link1.1', 11: 'joint_link2.2', 12: 'joint_link3.3',
  13: 'joint_link4.4', 14: 'joint_link5.5', 15: 'joint_link6.6',
  16: 'joint_link7.7', 17: 'joint_link8.8', 18: 'joint_link9.9',
};
export const jointMapSource = { kind: JOINT_OVERRIDE ? 'url' : 'builtin', error: '' };
let modelLoadGeneration = 0;

function publicGroups(groups: any[]) {
  return (Array.isArray(groups) ? groups : []).map((g) => ({
    ...g,
    motors: Array.isArray(g.motors) ? g.motors.map((m: any) => ({ ...m })) : [],
    joints: Array.isArray(g.joints) ? [...g.joints] : [],
  }));
}

function selectedForModel(groups: any[], current = ''): string {
  if (groups.some((g) => g.name === current)) return current;
  return groups[0]?.name || '';
}

export async function loadModelTransmissions(name = MODEL) {
  const generation = ++modelLoadGeneration;
  store.modelTransmissions = {
    model: name, driver_file: '', groups: [], error: '',
    driverFile: '', directJoints: {}, direct_mapping: {},
  };
  store.selectedGroup = '';
  store.selectedJoint = '';
  jointMapSource.kind = 'loading';
  jointMapSource.error = '';
  markDirty(DIRTY.MODEL, DIRTY.TOPOLOGY);
  try {
    const resp = await fetch(
      `/api/model-transmissions?model=${encodeURIComponent(name)}`, { cache: 'no-store' });
    if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
    const data = await resp.json();
    if (generation !== modelLoadGeneration || name !== MODEL) return store.modelTransmissions;
    if (data.model !== name || !Array.isArray(data.groups)) {
      throw new Error('接口返回的传动组无效');
    }
    const groups = publicGroups(data.groups);
    const driverFile = typeof data.driver_file === 'string' ? data.driver_file : '';
    const direct = data.direct_joints && typeof data.direct_joints === 'object'
      ? { ...data.direct_joints } : {};
    store.modelTransmissions = {
      model: data.model, driver_file: driverFile, groups,
      error: '', driverFile, directJoints: direct, direct_mapping: { ...direct },
      mapping_kind: data.mapping_kind, hardware_ready: data.hardware_ready,
      hardware_block_reason: data.hardware_block_reason,
      policy_defaults: data.policy_defaults || {},
    };
    store.selectedGroup = selectedForModel(groups);
    jointMapSource.kind = data.hardware_ready === false ? 'topology' : 'api';
    markDirty(DIRTY.MODEL, DIRTY.TOPOLOGY);
  } catch (e: any) {
    if (generation !== modelLoadGeneration || name !== MODEL) return store.modelTransmissions;
    jointMapSource.kind = 'unavailable';
    jointMapSource.error = String((e && e.message) || e);
    store.modelTransmissions.error = jointMapSource.error;
    console.warn(`[web_host] ${name} 传动组读取失败：${jointMapSource.error}`);
    markDirty(DIRTY.MODEL, DIRTY.TOPOLOGY);
  }
  return store.modelTransmissions;
}

export async function loadModelJoints() {
  const model = await loadModelTransmissions(MODEL);
  if (model.model !== MODEL) return jointMapSource;
  for (const k of Object.keys(JOINT_OF_MOTOR)) delete JOINT_OF_MOTOR[Number(k)];
  for (const [id, joint] of Object.entries(JOINT_OVERRIDE || model.directJoints)) {
    JOINT_OF_MOTOR[parseInt(id, 10)] = String(joint);
  }
  return jointMapSource;
}

const DEFAULT_MODEL = 'full_body_robot';
const MODEL_STORAGE_KEY = 'web_host.model';
function readSavedModel(): string {
  try { return localStorage.getItem(MODEL_STORAGE_KEY) || ''; } catch (_e) { return ''; }
}
export let MODEL = URLQ.get('model') || readSavedModel() || DEFAULT_MODEL;

export function jointFeedbackMatchesModel(): boolean {
  const feedback = store.bench.feedback;
  if (!feedback) return true;
  if (feedback.source === 'firmware') {
    return feedback.state === 'running' && feedback.model === MODEL
      && feedback.desired_model === MODEL;
  }
  if (feedback.source === 'simulation') return store.bench.simulation?.config?.model === MODEL;
  if (feedback.source === 'external') {
    const { data, last } = store.transmissionStatus, age = Date.now() - last;
    return data?.model === MODEL && last > 0 && age >= 0 && age <= 2500;
  }
  return false;
}

export function setModel(name: string): void {
  if (!name || name === MODEL) return;
  MODEL = name;
  store.modelJoints = [];
  store.jointStates.byName = {};
  store.jointStates.jointNames = [];
  store.jointStates.last = 0;
  store.tf.clear();
  try { localStorage.setItem(MODEL_STORAGE_KEY, name); } catch (_e) { /* ignore */ }
  markDirty(DIRTY.MODEL);
  void loadModelJoints();
}

export function selectGroups(namesList: string[]): void {
  const chosen = [...new Set(namesList)].filter(
    (n) => store.modelTransmissions.groups.some((g: any) => g.name === n));
  if (!chosen.length) return;
  const value = chosen.join(',');
  if (store.selectedGroup === value) return;
  store.selectedGroup = value;
  if (!store.modelTransmissions.groups.some(
    (g: any) => g.joints.includes(store.selectedJoint))) store.selectedJoint = '';
  markDirty(DIRTY.SELECTED, DIRTY.STATUS);
}

export function selectJoint(name: string): void {
  const exists = store.modelTransmissions.groups.some((g: any) => g.joints.includes(name));
  store.selectedJoint = exists ? name : '';
  markDirty(DIRTY.SELECTED, DIRTY.STATUS);
}

export async function availableModels(): Promise<string[]> {
  const resp = await fetch('/api/models', { cache: 'no-store' });
  if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
  const data = await resp.json();
  return Array.isArray(data && data.models) ? data.models : [];
}

export function targetOf(id: number): number {
  const h = store.history[0];
  if (!h) return NaN;
  const t = h.targets.find((x: any) => x.id === id);
  return t ? t.pos : NaN;
}

export function paramRowIds(showAll: boolean): number[] {
  const all = [...store.paramCache.keys()].sort((a, b) => a - b);
  return (showAll || !store.selectedId) ? all : all.filter((i) => i === store.selectedId);
}

export function paramReadState(id: number) {
  const entry = store.paramCache.get(id);
  const fields = ['pos_min', 'pos_max', 'vel_limit', 'torque_limit', 'limit_protect', 'direction', 't_max', 'node_id'];
  const count = (source: string) => fields.filter(field => Number.isFinite(entry?.[source]?.[field])).length;
  const device = count('dev'), record = count('rec'), usable = count('pick');
  return { device, record, usable, verified: entry?.state === 2 && device === fields.length };
}
