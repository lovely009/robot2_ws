// rosbridge 层：连接、订阅、发布与「等一帧/等序号」工具。
// 数据写入 store 后 markDirty，由面板渲染；连接状态同步到 body.offline 以禁用写操作。
import {
  store, markDirty, DIRTY, chartPush, applyBenchStatus, applyCommandStatus, applyTransmissionStatus, observeTrajectory, resetTrajectory,
  jointFeedbackMatchesModel, applyAgentWireStatus,
} from './store';
import {
  $, TOPICS, EVENT_NAMES, MOTOR_ID_NONE, quatToEuler, nsToHeader, headerStampNs,
} from './format';

import {
  applyControlState, createControlStateCache, beginPolicyIntent, acceptPolicyTicket, failPolicyIntent, applyPolicyReceipt,
} from './motor_state';

import { bffClient } from '../api/bff_client';

export interface BffTransportObserver {
  onConfigure?(cfg: any): void;
  onConnected?(hello: { client_id: string; lease: any; control_mode?: string }): void;
  onLease?(lease: any): void;
  onRtt?(rttMs: number): void;
}
let bffObserver: BffTransportObserver | null = null;
export function setBffObserver(observer: BffTransportObserver | null): void {
  bffObserver = observer;
}

let controlMode = 'bench';
let transportMode: 'bff' | 'rosbridge' = 'rosbridge';
const READ_ONLY_SERVICES = new Set([
  '/motor/get_device_info',
  '/bench/options',
  '/bench/agent_status',
  '/bench/get_unit_log',
]);

export function configureControl(configuration: any) {
  if (!['readonly', 'bench'].includes(configuration?.control_mode)) throw new Error('工作台运行模式无效');
  controlMode = configuration.control_mode;
  transportMode = configuration?.transport === 'bff' ? 'bff' : 'rosbridge';
  store.runtimeMode = controlMode;
  bffObserver?.onConfigure?.(configuration);
}

export function setActiveChannels(channels: string[]): void {
  bffClient.setChannels(channels);
}

let ros: ROSLIB.Ros | null = null;
let connectionGeneration = 0;
export function getConnectionGeneration(): number { return connectionGeneration; }
let subs: ROSLIB.Topic[] = [];
const publishers: Record<string, ROSLIB.Topic> = {};
const CONFIG_RESULT_LIMIT = 64;
let configSessionId: string | null = null;

function clearConfigResults() {
  store.configResults.clear();
  store.configSessionId = null;
  configSessionId = null;
}

function ingestConfigResult(message: any) {
  const sessionId = String(message.session_id);
  if (configSessionId !== null && configSessionId !== sessionId) {
    clearConfigResults();
  }
  configSessionId = sessionId;
  store.configSessionId = sessionId;
  const result = { ...message };
  const requestId = Number(message.request_id);
  if (!Number.isInteger(requestId) || requestId <= 0) return;
  const previous = store.configResults.get(requestId);
  if (previous && String(previous.session_id) === sessionId
      && (previous.intent_id || '') === (message.intent_id || '')
      && Number(previous.status) !== 0 && Number(message.status) === 0) return;
  store.configResults.delete(requestId);
  store.configResults.set(requestId, result);
  while (store.configResults.size > CONFIG_RESULT_LIMIT) {
    store.configResults.delete(store.configResults.keys().next().value!);
  }
}

export function setConn(ok: boolean, text?: string): void {
  store.conn.connected = ok;
  store.conn.text = text || (ok ? '已连接' : '未连接');
  globalThis.document?.body?.classList?.toggle('offline', !ok);
  markDirty(DIRTY.CONN);
}

export function isConnected(): boolean { return store.conn.connected; }

// 浏览器仅转交人工意图；会话时钟、请求号与策略代数由主机分配。
export async function submitPolicy(targets: any[]) {
  if (!store.conn.connected || controlMode !== 'bench') throw new Error('未连接或当前页面只读，未提交使能');
  if (!Array.isArray(targets) || !targets.length || targets.length > 37
      || targets.some(target => !target || typeof target.enabled !== 'boolean')) throw new Error('使能目标无效');
  const enabled = targets.some(target => target.enabled), cache = store.controlStates;
  const age = Date.now() - cache.at;
  if (enabled && (!cache.valid || !cache.seen || !cache.session || !cache.sourceTimeNs
      || age < 0 || age > 250)) throw new Error('MCU 控制快照已过期，等待新反馈后重新使能');
  const connection = connectionGeneration, session = cache.session || 0;
  const intent = beginPolicyIntent(store, targets);
  markDirty(DIRTY.POLICY, DIRTY.STATUS);
  try {
    const ticket = await callService('/motor/submit_policy', 'can_bridge_msgs/srv/SubmitPolicy', {
      header: nsToHeader(cache.sourceTimeNs || 0),
      session_id: session,
      targets: targets.map(target => ({ ...target })),
    });
    if (!store.conn.connected || connection !== connectionGeneration
        || (enabled && session !== store.controlStates.session)) throw new Error('连接或 MCU 会话已变化，使能请求失效');
    if (intent.error) throw new Error(intent.error);
    if (ticket?.accepted !== true) throw new Error(ticket?.reason || '主机拒绝使能请求');
    const uint32 = (value: any) => Number.isInteger(value) && value >= 0 && value <= 0xffffffff;
    if (typeof ticket.intent_id !== 'string' || !ticket.intent_id
        || !Number.isSafeInteger(ticket.session_id) || ticket.session_id < 0
        || !uint32(ticket.request_id) || !uint32(ticket.policy_generation)
        || (enabled && (String(ticket.session_id) !== String(session)
          || !ticket.request_id || !ticket.policy_generation))) throw new Error('主机使能票据无效');
    acceptPolicyTicket(store, intent, ticket);
    return ticket;
  } catch (error) {
    failPolicyIntent(store, intent, error);
    throw error;
  } finally { markDirty(DIRTY.POLICY, DIRTY.STATUS); }
}

export function pub(key: string, message: Record<string, any>): boolean {
  if (!TOPICS[key]) return false;
  if (!store.conn.connected) return false;
  if (transportMode === 'bff') {
    if (key === 'stop') {
      void bffClient.triggerStop('operator_stop').catch(() => {});
      return true;
    }
    if (controlMode === 'readonly') return false;
    if (key === 'jointTarget' || key === 'simJointTarget') {
      const stampNs = headerStampNs(message);
      return bffClient.sendJointTarget(
        key === 'simJointTarget' ? 'sim:joint_target' : 'control:joint_target',
        {
          names: message.name || message.names || [],
          positions: message.position || message.positions || [],
          velocities: message.velocity || message.velocities || [],
          efforts: message.effort || message.efforts || [],
          stamp_ns: stampNs > 0 ? stampNs : null,
          frame_id: String(message.header?.frame_id || ''),
        },
      );
    }
    if (key === 'trajectory') {
      void bffClient.publishTrajectory(message as any).catch(() => {});
      return true;
    }
    return false;
  }
  if (controlMode === 'readonly') return false;
  ensurePublisher(key)?.publish(new ROSLIB.Message(message));
  return true;
}

export async function publishTrajectory(message: Record<string, any>): Promise<boolean> {
  if (!TOPICS.trajectory || !store.conn.connected || controlMode === 'readonly') return false;
  if (transportMode === 'bff') {
    await bffClient.publishTrajectory(message as any);
    return true;
  }
  return pub('trajectory', message);
}

export async function executeConfig(request: Record<string, any>, timeoutMs = 10000) {
  const response = await callService(
    '/motor/execute_config',
    'can_bridge_msgs/srv/ExecuteConfig',
    { request, timeout_ms: timeoutMs },
    timeoutMs + 2000,
  );
  return response?.result || response;
}

export function ensurePublisher(key: string): ROSLIB.Topic | null {
  if (transportMode === 'bff' || !ros || controlMode === 'readonly' || key === 'policy') return null;
  if (!publishers[key]) {
    const t = TOPICS[key]!;
    publishers[key] = new ROSLIB.Topic({ ros: ros!, name: t.name, messageType: t.type });
    publishers[key].advertise();
  }
  return publishers[key]!;
}

let reconnectTimer: ReturnType<typeof setTimeout> | null = null;

function resetSessionState() {
  lastInfoRevision = null;
  clearConfigResults();
  store.motors.clear();
  store.controlStates = createControlStateCache();
  store.agentWireStatus = null;
  store.paramCache.clear();
  store.tf.clear();
  store.sysStatus.seen = false;
  store.firmwareReport = { seq: 0, at: 0, values: null, unsaved: false };
  store.conn.autoLoaded = false;
  store.conn.benchLoaded = false;
  for (const key of Object.keys(store.bench)) store.bench[key] = null;
  store.conn.agent = 'stopped';
  store.conn.agentDevice = '';
  store.conn.canStop = false;
  store.commandStatus = { last: 0, data: null };
  store.simPositionStatus = { last: 0, data: null };
  resetTrajectory();
  store.jointStates.byName = {};
  store.jointStates.jointNames = [];
  store.transmissionStatus = { last: 0, data: null };
  markDirty(DIRTY.TOPOLOGY, DIRTY.STATUS, DIRTY.PARAMS, DIRTY.TF, DIRTY.BENCH,
    DIRTY.COMMAND, DIRTY.MOTION);
}

export function disconnect(): void {
  if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null; }
  bffClient.disconnect();
  closeSocket();
  resetSessionState();
  setConn(false, '已断开');
}

export function connect(explicitUrl?: string): void {
  if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null; }
  bffClient.disconnect();
  closeSocket();
  resetSessionState();
  setConn(false, '正在连接…');
  if (transportMode === 'bff') {
    bffClient.connect({
      onConnect: (hello) => {
        setConn(true, '已连接 (BFF)');
        bffObserver?.onConnected?.(hello as any);
      },
      onDisconnect: (reason) => {
        connectionGeneration++;
        clearConfigResults();
        resetSessionState();
        setConn(false, reason);
      },
      onLease: (leaseState) => {
        bffObserver?.onLease?.(leaseState);
      },
      onRtt: (rttMs) => {
        bffObserver?.onRtt?.(rttMs);
      },
      onTelemetry: (channel, payload, meta) => {
        handleBffTelemetry(channel, payload, meta);
      },
    });
    return;
  }
  const url = (explicitUrl !== undefined ? explicitUrl : ($('rosUrl')?.value || '')).trim();
  try {
    ros = new ROSLIB.Ros({ url });
  } catch (_e) {
    setConn(false, 'URL 无效');
    return;
  }
  const connection = ros;
  ros.on('connection', () => {
    if (ros !== connection) return;
    setConn(true);
    subscribeAll();
    ['trajectory', 'jointTarget', 'stop'].forEach(ensurePublisher);
  });
  ros.on('error', () => {
    if (ros === connection) {
      connectionGeneration++;
      clearConfigResults();
      setConn(false, '连接错误');
    }
  });
  ros.on('close', () => {
    if (ros !== connection) return;
    closeSocket();
    resetSessionState();
    if (!reconnectTimer) {
      setConn(false, '已断开，2s 后重连…');
      reconnectTimer = setTimeout(() => { reconnectTimer = null; connect(); }, 2000);
    }
  });
}

export function closeSocket(): void {
  connectionGeneration++;
  clearConfigResults();
  if (ros) { const r = ros; ros = null; try { r.close(); } catch (_e) { /* ignore */ } }
  subs = [];
  Object.values(publishers).forEach((p) => { try { p.unadvertise(); } catch (_e) { /* ignore */ } });
  Object.keys(publishers).forEach((k) => delete publishers[k]);
}

function sub(key: string, cb: (message: any) => void) {
  const connection = ros;
  const t = TOPICS[key]!;
  const topic = new ROSLIB.Topic({
    ros: ros!, name: t.name, messageType: t.type,
    throttle_rate: key === 'events' ? 0 : key === 'state' ? 20 : key === 'imu' ? 100 : 50,
    queue_length: key === 'events' ? 64 : 1,
  });
  topic.subscribe((message) => { if (ros === connection && store.conn.connected) cb(message); });
  subs.push(topic);
}

let lastInfoRevision: number | null = null;

export function applyDeviceInfo(m: any): void {
  if (!m) return;
  const seen = new Set<number>();
  for (const e of (m.topology || [])) {
    seen.add(e.motor_id);
    const row = store.motors.get(e.motor_id) || {};
    row.topo = e;
    store.motors.set(e.motor_id, row);
  }
  for (const id of [...store.motors.keys()]) {
    if (!seen.has(id)) store.motors.delete(id);
  }
  markDirty(DIRTY.TOPOLOGY);

  const safety = (v: any) => (Number.isFinite(v) ? (v === 0 ? 1 : 0) : NaN);
  for (const e of (m.params || [])) {
    const rec: Record<string, number> = {
      pos_min: e.rec_pos_min, pos_max: e.rec_pos_max,
      vel_limit: e.rec_vel_limit, torque_limit: e.rec_torque_limit,
      limit_protect: safety(e.rec_safety), direction: e.rec_direction, t_max: e.rec_t_max, node_id: e.rec_node_id,
    };
    const dev: Record<string, number> = {
      pos_min: e.dev_pos_min, pos_max: e.dev_pos_max,
      vel_limit: e.dev_vel_limit, torque_limit: e.dev_torque_limit,
      limit_protect: safety(e.dev_safety), direction: e.dev_direction, t_max: e.dev_t_max, node_id: e.dev_node_id,
    };
    const pick: Record<string, number> = {};
    for (const f of Object.keys(rec)) {
      pick[f] = Number.isFinite(dev[f]) ? dev[f]! : rec[f]!;
    }
    store.paramCache.set(e.motor_id, {
      rec, dev, pick,
      state: e.dev_init_state, dirty: e.record_dirty, mask: e.mismatch_mask,
    });
  }
  markDirty(DIRTY.PARAMS);
}

export async function fetchDeviceInfo(motorId = 0) {
  const response = await callService(
    '/motor/get_device_info',
    'can_bridge_msgs/srv/GetDeviceInfo',
    { motor_id: motorId },
    6000,
  );
  const info = response?.info || response;
  applyDeviceInfo(info);
  return info;
}

function ingestTransmission(raw: unknown) {
  try {
    const parsed = typeof raw === 'string' ? JSON.parse(raw) : raw;
    applyTransmissionStatus(parsed);
  } catch (_e) {
    applyTransmissionStatus(null);
  }
  markDirty(DIRTY.STATUS);
}

function ingestMotion(m: any) {
  if (!m) return;
  store.motionCount++;
  const t = Date.now();
  for (const s of (m.states || [])) {
    const row = store.motors.get(s.motor_id) || {};
    row.motion = s;
    store.motors.set(s.motor_id, row);
    chartPush(s.motor_id, 'position', s.position, t);
    chartPush(s.motor_id, 'velocity', s.velocity, t);
    chartPush(s.motor_id, 'effort', s.effort, t);
  }
  markDirty(DIRTY.MOTION);
}

function ingestState(m: any, meta?: Record<string, unknown>) {
  if (!m) return;
  const previousSession = store.controlStates.session;
  const accepted = applyControlState(store.controlStates, m);
  if (accepted) {
    const currentSession = String(store.controlStates.session);
    if (previousSession !== 0 && store.controlStates.session !== previousSession) {
      clearConfigResults();
      configSessionId = currentSession;
      store.configSessionId = currentSession;
    } else if (previousSession === 0 && configSessionId === null) {
      configSessionId = currentSession;
      store.configSessionId = currentSession;
    }
    if (m.info_revision !== undefined && m.info_revision !== lastInfoRevision) {
      lastInfoRevision = m.info_revision;
      fetchDeviceInfo(0).catch(() => {});
    }
  }
  const serverTsMs = typeof meta?.server_ts_ms === 'number' ? meta.server_ts_ms : undefined;
  if (applyAgentWireStatus(store, m, Date.now(), serverTsMs)) markDirty(DIRTY.CONN);
  const t = Date.now();
  for (const s of (m.statuses || [])) {
    const row = store.motors.get(s.motor_id) || {};
    row.status = s;
    row.statusAt = t;
    store.motors.set(s.motor_id, row);
    chartPush(s.motor_id, 'current', s.current, t);
  }
  const sys = m.system_status;
  if (sys) {
    if (sys.firmware_values_valid && Array.isArray(sys.firmware_values) && sys.firmware_values.length === 6 &&
        sys.firmware_values.every((v: any) => Number.isInteger(v) && v > 0 && v <= 1000)) {
      store.firmwareReport = {
        seq: store.firmwareReport.seq + 1, at: t,
        values: [...sys.firmware_values], unsaved: !!sys.firmware_unsaved,
      };
    }
    store.lastError = sys.last_error;
    store.lastErrorMotor = sys.last_error_motor === MOTOR_ID_NONE ? -1 : sys.last_error_motor;
    store.sysStatus = {
      refresh_seq: sys.refresh_seq, set_zero_seq: sys.set_zero_seq,
      clear_fault_seq: sys.clear_fault_seq, save_seq: sys.save_seq,
      save_in_progress: !!sys.save_in_progress, seen: true,
    };
  }
  markDirty(DIRTY.STATUS);
}

function ingestEvent(m: any) {
  if (!m) return;
  if (m.event_type === 13) {
    if (applyPolicyReceipt(store, m)) markDirty(DIRTY.POLICY, DIRTY.STATUS);
  }
  store.events.unshift({
    t: Date.now(),
    name: EVENT_NAMES[m.event_type] !== undefined
      ? EVENT_NAMES[m.event_type] : 'EVENT_' + m.event_type,
    id: m.motor_id,
    reason: m.reason,
    request_id: m.request_id || 0,
  });
  if (store.events.length > 2000) store.events.pop();
  markDirty(DIRTY.EVENTS, DIRTY.PARAMS);
}

function ingestImu(m: any) {
  const q = m?.orientation;
  if (!q) return;
  const e = quatToEuler(q.x, q.y, q.z, q.w);
  store.imu.roll = e.roll;
  store.imu.pitch = e.pitch;
  store.imu.yaw = e.yaw;
  store.imu.q = q;
  store.imu.count++;
  store.imu.last = Date.now();
  markDirty(DIRTY.IMU);
}

function ingestTemp(m: any) {
  store.temp = Number.isFinite(m?.temperature) ? m.temperature : m?.data;
  store.tempLast = Date.now();
  markDirty(DIRTY.IMU);
}

function ingestJointStates(m: any) {
  if (!jointFeedbackMatchesModel()) return;
  const now = Date.now();
  const names = Array.isArray(m?.names) ? m.names : Array.isArray(m?.name) ? m.name : [];
  const positions = Array.isArray(m?.positions) ? m.positions : Array.isArray(m?.position) ? m.position : [];
  const velocities = Array.isArray(m?.velocities) ? m.velocities : Array.isArray(m?.velocity) ? m.velocity : [];
  const efforts = Array.isArray(m?.efforts) ? m.efforts : Array.isArray(m?.effort) ? m.effort : [];
  for (let i = 0; i < names.length; i++) {
    const position = positions[i];
    if (typeof names[i] === 'string' && Number.isFinite(position)) {
      store.jointStates.byName[names[i]] = {
        position,
        velocity: Number.isFinite(velocities[i]) ? velocities[i] : NaN,
        effort: Number.isFinite(efforts[i]) ? efforts[i] : NaN,
        at: now,
      };
    } else if (typeof names[i] === 'string') delete store.jointStates.byName[names[i]];
  }
  if (names.length) store.jointStates.jointNames = [
    ...new Set<string>(names.filter((name: any): name is string => typeof name === 'string'))];
  store.jointStates.count++;
  store.jointStates.last = now;
  markDirty(DIRTY.MOTION);
}

function ingestBench(raw: unknown) {
  try {
    const data = typeof raw === 'string' ? JSON.parse(raw) : raw;
    applyBenchStatus(data);
  } catch (_e) { /* ignore */ }
}

function ingestCommandStatus(raw: unknown) {
  try {
    const data = typeof raw === 'string' ? JSON.parse(raw) : raw;
    applyCommandStatus(data);
    if (data && data.trajectory_state) observeTrajectory(data.trajectory_state);
  } catch (_e) {
    applyCommandStatus(null);
  }
}

function ingestSimPositionStatus(raw: unknown) {
  let data: any;
  try { data = typeof raw === 'string' ? JSON.parse(raw) : raw; } catch (_e) { data = null; }
  store.simPositionStatus = { last: Date.now(), data };
  markDirty(DIRTY.STATUS);
}

function handleBffTelemetry(channel: string, data: unknown, meta?: Record<string, unknown>) {
  switch (channel) {
    case 'telemetry:transmission': ingestTransmission(data); break;
    case 'telemetry:motor_motion': ingestMotion(data); break;
    case 'telemetry:motor_state': ingestState(data, meta); break;
    case 'telemetry:events': ingestEvent(data); break;
    case 'telemetry:imu': ingestImu(data); break;
    case 'telemetry:imu_temp': ingestTemp(data); break;
    case 'telemetry:joints': ingestJointStates(data); break;
    case 'telemetry:bench': ingestBench(data); break;
    case 'telemetry:control': ingestCommandStatus(data); break;
    case 'telemetry:sim_position': ingestSimPositionStatus(data); break;
  }
}

function subscribeAll() {
  subs.forEach((s) => s.unsubscribe());
  subs = [];
  const statusConnection = ros;
  const statusTopic = new ROSLIB.Topic({
    ros: ros!,
    name: TOPICS.robotTransmissionStatus!.name,
    messageType: TOPICS.robotTransmissionStatus!.type,
    throttle_rate: 100,
    queue_length: 1,
    qos: { reliability: 'reliable', durability: 'transient_local', history: 'keep_last', depth: 1 },
  });
  statusTopic.subscribe((message) => {
    if (ros !== statusConnection || !store.conn.connected) return;
    ingestTransmission(String(message && message.data));
  });
  subs.push(statusTopic);

  sub('motion', ingestMotion);
  sub('state', ingestState);
  sub('events', ingestEvent);
  sub('imu', ingestImu);
  sub('temp', ingestTemp);
  sub('jointStates', ingestJointStates);
  sub('bench', (m) => ingestBench(m?.data));
  sub('commandStatus', (m) => ingestCommandStatus(m?.data));
  sub('simPositionStatus', (m) => ingestSimPositionStatus(m?.data));
}

export function callService(name: string, type: string, request: Record<string, any>, timeoutMs = 8000): Promise<any> {
  if (transportMode === 'bff') {
    if (controlMode === 'readonly' && !READ_ONLY_SERVICES.has(name)) {
      return Promise.reject(new Error('只读页面不开放控制或维护服务'));
    }
    if (!store.conn.connected) {
      return Promise.reject(new Error('未连接 BFF 网关'));
    }
    return bffClient.callDomainService(name, request, timeoutMs);
  }
  return new Promise((resolve, reject) => {
    if (controlMode === 'readonly') {
      reject(new Error('只读页面不开放控制或维护服务')); return;
    }
    if (!ros || !store.conn.connected) {
      reject(new Error('未连接 rosbridge'));
      return;
    }
    const connection = ros;
    const service = new ROSLIB.Service({ ros, name, serviceType: type });
    let settled = false;
    const timer = setTimeout(() => {
      if (!settled) { settled = true; reject(new Error(`服务无响应：${name}`)); }
    }, timeoutMs);
    service.callService(new ROSLIB.Message(request || {}), (result) => {
      if (settled) return;
      settled = true;
      clearTimeout(timer);
      if (ros !== connection) { reject(new Error('连接已更换')); return; }
      resolve(result);
    }, (error) => {
      if (settled) return;
      settled = true;
      clearTimeout(timer);
      reject(new Error(typeof error === 'string'
        ? error : `服务调用失败：${name}`));
    });
  });
}

export function waitSysStatus(timeoutMs = 2000): Promise<boolean> {
  if (store.sysStatus.seen) return Promise.resolve(true);
  return new Promise((resolve) => {
    const deadline = Date.now() + timeoutMs;
    const timer = setInterval(() => {
      if (store.sysStatus.seen) { clearInterval(timer); resolve(true); }
      else if (Date.now() >= deadline) { clearInterval(timer); resolve(false); }
    }, 50);
  });
}

// 供测试检查内部入库逻辑（与原有行为一致）。
export { ingestConfigResult };
