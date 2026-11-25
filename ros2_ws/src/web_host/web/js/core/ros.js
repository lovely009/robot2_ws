// rosbridge 层：连接、订阅、发布与「等一帧/等序号」工具。
// 数据写入 store 后 markDirty，由面板渲染；连接状态同步到 body.offline 以禁用写操作。
import {
  store, markDirty, DIRTY, chartPush, applyBenchStatus, applyCommandStatus, applyTransmissionStatus, observeTrajectory, resetTrajectory,
  jointFeedbackMatchesModel, applyAgentWireStatus } from './store.js';
import {
  $, TOPICS, EVENT_NAMES, MOTOR_ID_NONE, quatToEuler, nsToHeader,
} from './format.js';

import { applyControlState, createControlStateCache, beginPolicyIntent, acceptPolicyTicket, failPolicyIntent, applyPolicyReceipt } from './motor_state.js';
let controlMode = 'bench';
export function configureControl(configuration) {
  if (!['readonly', 'bench'].includes(configuration?.control_mode)) throw new Error('工作台运行模式无效');
  controlMode = configuration.control_mode;
  store.runtimeMode = controlMode;
}

let ros = null;
let connectionGeneration = 0;
export function getConnectionGeneration() { return connectionGeneration; }
let subs = [];
const publishers = {};
const CONFIG_RESULT_LIMIT = 64;
const configWaiters = new Map();
let configSessionId = null;

function settleConfigWaiters(requestId, value) {
  const waiters = configWaiters.get(requestId);
  if (!waiters) return;
  configWaiters.delete(requestId);
  for (const waiter of waiters) waiter.finish(value);
}

function clearConfigResults(outcome = 'disconnected') {
  store.configResults.clear();
  store.configSessionId = null;
  configSessionId = null;
  for (const requestId of [...configWaiters.keys()]) {
    settleConfigWaiters(requestId, {outcome, request_id: requestId});
  }
}

function ingestConfigResult(message) {
  const sessionId = String(message.session_id);
  if (configSessionId !== null && configSessionId !== sessionId) {
    clearConfigResults('session_changed');
  }
  configSessionId = sessionId;
  store.configSessionId = sessionId;
  const result = {...message};
  const requestId = Number(message.request_id);
  if (!Number.isInteger(requestId) || requestId <= 0) return;
  const previous = store.configResults.get(requestId);
  if (previous && String(previous.session_id) === sessionId
      && (previous.intent_id || '') === (message.intent_id || '')
      && Number(previous.status) !== 0 && Number(message.status) === 0) return;
  store.configResults.delete(requestId);
  store.configResults.set(requestId, result);
  while (store.configResults.size > CONFIG_RESULT_LIMIT) {
    store.configResults.delete(store.configResults.keys().next().value);
  }
  if (Number(message.status) !== 0) {
    settleConfigWaiters(requestId, {outcome: 'terminal', request_id: requestId, result});
  }
}

export function waitConfigResult(requestId, timeoutMs) {
  const id = Number(requestId);
  if (!Number.isInteger(id) || id <= 0) {
    return Promise.resolve({outcome: 'invalid_request', request_id: requestId});
  }
  const cached = store.configResults.get(id);
  if (cached && Number(cached.status) !== 0) {
    return Promise.resolve({outcome: 'terminal', request_id: id, result: cached});
  }
  if (!store.conn.connected) return Promise.resolve({outcome: 'disconnected', request_id: id});
  const generation = connectionGeneration;
  return new Promise((resolve) => {
    let settled = false;
    const finish = (value) => {
      if (settled) return;
      settled = true;
      clearTimeout(timer);
      const waiters = configWaiters.get(id);
      if (waiters) {
        waiters.delete(waiter);
        if (!waiters.size) configWaiters.delete(id);
      }
      resolve(value);
    };
    const waiter = {finish};
    const timer = setTimeout(() => finish({outcome: 'timeout', request_id: id}), timeoutMs);
    if (!configWaiters.has(id)) configWaiters.set(id, new Set());
    configWaiters.get(id).add(waiter);
    if (generation !== connectionGeneration || !store.conn.connected) {
      finish({outcome: 'disconnected', request_id: id});
    }
  });
}

export function setConn(ok, text) {
  store.conn.connected = ok;
  store.conn.text = text || (ok ? '已连接' : '未连接');
  document.body.classList.toggle('offline', !ok);
  markDirty(DIRTY.CONN);
}

export function isConnected() { return store.conn.connected; }

// 浏览器仅转交人工意图；会话时钟、请求号与策略代数由主机分配。
export async function submitPolicy(targets) {
  if (!store.conn.connected || controlMode !== 'bench') throw new Error('未连接或当前页面只读，未提交使能');
  if (!Array.isArray(targets) || !targets.length || targets.length > 37
      || targets.some(target => !target || typeof target.enabled !== 'boolean')) throw new Error('使能目标无效');
  const enabled = targets.some(target => target.enabled), cache = store.controlStates;
  const age = Date.now() - cache.at;
  if (enabled && (!cache.valid || !cache.seen || !cache.session || !cache.sourceTimeNs
      || age < 0 || age > 250)) throw new Error('MCU 控制快照已过期，等待新反馈后重新使能');
  const connection = connectionGeneration, session = cache.session || 0;
  const intent=beginPolicyIntent(store,targets);
  markDirty(DIRTY.POLICY,DIRTY.STATUS);
  try {
    const ticket = await callService('/motor/submit_policy', 'can_bridge_msgs/srv/SubmitPolicy', {
      header: nsToHeader(cache.sourceTimeNs || 0),
      session_id: session,
      targets: targets.map(target => ({...target})),
    });
    if (!store.conn.connected || connection !== connectionGeneration
        || (enabled && session !== store.controlStates.session)) throw new Error('连接或 MCU 会话已变化，使能请求失效');
    if (intent.error) throw new Error(intent.error);
    if (ticket?.accepted !== true) throw new Error(ticket?.reason || '主机拒绝使能请求');
    const uint32 = value => Number.isInteger(value) && value >= 0 && value <= 0xffffffff;
    if (typeof ticket.intent_id !== 'string' || !ticket.intent_id
        || !Number.isSafeInteger(ticket.session_id) || ticket.session_id<0
        || !uint32(ticket.request_id) || !uint32(ticket.policy_generation)
        || (enabled && (String(ticket.session_id) !== String(session)
          || !ticket.request_id || !ticket.policy_generation))) throw new Error('主机使能票据无效');
    acceptPolicyTicket(store,intent,ticket);
    return ticket;
  } catch (error) {
    failPolicyIntent(store,intent,error);
    throw error;
  } finally {markDirty(DIRTY.POLICY,DIRTY.STATUS);}
}

export function pub(key, message) {
  if (!TOPICS[key]) return false;
  if (!store.conn.connected) return false;
  if (controlMode === 'readonly') return false;
  ensurePublisher(key).publish(new ROSLIB.Message(message));
  return true;
}

export async function executeConfig(request, timeoutMs = 10000) {
  const response = await callService(
    '/motor/execute_config',
    'can_bridge_msgs/srv/ExecuteConfig',
    {request, timeout_ms: timeoutMs},
    timeoutMs + 2000,
  );
  return response?.result || response;
}

// rosbridge 的 advertise 是异步 WebSocket 操作。若在第一次 advertise 后
// 立刻 publish，低频的一次性 command（尤其是动作前的授权快照）可能在
// micro-ROS 订阅端完成发现前丢掉；随后 trajectory_state 仍会回执，造成
// “轨迹已完成但没有运动帧”的假成功。连接建立时先把三个下行发布端注册好，
// 后续按钮/动作只发送已经存在的 publisher。
export function ensurePublisher(key) {
  if (controlMode === 'readonly' || key === 'policy') return null;
  if (!publishers[key]) {
    const t = TOPICS[key];
    publishers[key] = new ROSLIB.Topic({ros, name: t.name, messageType: t.type});
    publishers[key].advertise();
  }
  return publishers[key];
}

// roslibjs 不自动重连；连接中断后按 2 s 退避重试。
let reconnectTimer = null;

function resetSessionState() {
  lastInfoRevision = null;
  clearConfigResults();
  store.motors.clear();
  store.controlStates = createControlStateCache();
  store.agentWireStatus = null;
  store.mcuLifecycle = null;
  store.paramCache.clear();
  store.tf.clear();
  store.sysStatus.seen = false;
  store.firmwareReport = {seq: 0, at: 0, values: null, unsaved: false};
  store.conn.autoLoaded = false;
  store.conn.benchLoaded = false;
  for (const key of Object.keys(store.bench)) store.bench[key] = null;
  store.conn.agent = 'stopped';
  store.conn.agentDevice = '';
  store.conn.canStop = false;
  store.commandStatus = {last: 0, data: null};
  store.simPositionStatus = {last: 0, data: null};
  resetTrajectory();
  store.jointStates.byName = {};
  store.jointStates.jointNames = [];
  store.transmissionStatus = {last: 0, data: null};
  markDirty(DIRTY.TOPOLOGY, DIRTY.STATUS, DIRTY.PARAMS, DIRTY.TF, DIRTY.BENCH,
    DIRTY.COMMAND, DIRTY.MOTION);
}

export function disconnect() {
  if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null; }
  closeSocket();
  resetSessionState();
  setConn(false, '已断开');
}

export function connect() {
  if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null; }
  closeSocket();
  resetSessionState();
  setConn(false, '正在连接…');
  const url = $('rosUrl').value.trim();
  try {
    ros = new ROSLIB.Ros({url});
  } catch (e) {
    setConn(false, 'URL 无效');
    return;
  }
  const connection = ros;
  ros.on('connection', () => {
    if (ros !== connection) return;
    setConn(true);
    subscribeAll();
    ['trajectory', 'jointTarget', 'stop'].forEach(ensurePublisher);
    // 连接后自动读取参数表（REPORT → device_info）随设备屏接入——
    // loadParams 归 core/device.js，这里只负责建立连接与订阅。
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
    // !reconnectTimer：同一时刻只允许一个待触发的重连（否则会叠出重连风暴，
    // 表现为页面一直连不上、看着"卡"）。
    if (!reconnectTimer) {
      setConn(false, '已断开，2s 后重连…');
      reconnectTimer = setTimeout(() => { reconnectTimer = null; connect(); }, 2000);
    }
  });
}

// 只关 socket，不动连接意图（自动重连要复用）。
// 连发/曲线等定时器由各自面板在连接态脏帧里自行停止；本层不反向调用面板。
export function closeSocket() {
  connectionGeneration++;
  clearConfigResults();
  if (ros) { const r = ros; ros = null; try { r.close(); } catch (e) {} }
  subs = [];
  Object.values(publishers).forEach((p) => { try { p.unadvertise(); } catch (e) {} });
  Object.keys(publishers).forEach((k) => delete publishers[k]);
}

function sub(key, cb) {
  const connection = ros;
  const t = TOPICS[key];
  const topic = new ROSLIB.Topic({
    // 浏览器只保留最新观测；事件逐条传递，系统状态高优先级。
    ros, name: t.name, messageType: t.type,
    throttle_rate: key === 'events' ? 0 : key === 'state' ? 20 : key === 'imu' ? 100 : 50,
    queue_length: key === 'events' ? 64 : 1,
  });
  topic.subscribe((message) => { if (ros === connection && store.conn.connected) cb(message); });
  subs.push(topic);
}

let lastInfoRevision = null;

export function applyDeviceInfo(m) {
  if (!m) return;
  const seen = new Set();
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

  store.devParamSeq++;
  const safety = (v) => (Number.isFinite(v) ? (v === 0 ? 1 : 0) : NaN);
  for (const e of (m.params || [])) {
    const rec = {pos_min: e.rec_pos_min, pos_max: e.rec_pos_max,
      vel_limit: e.rec_vel_limit, torque_limit: e.rec_torque_limit,
      limit_protect: safety(e.rec_safety), direction: e.rec_direction, t_max: e.rec_t_max, node_id: e.rec_node_id};
    const dev = {pos_min: e.dev_pos_min, pos_max: e.dev_pos_max,
      vel_limit: e.dev_vel_limit, torque_limit: e.dev_torque_limit,
      limit_protect: safety(e.dev_safety), direction: e.dev_direction, t_max: e.dev_t_max, node_id: e.dev_node_id};
    const pick = {};
    for (const f of Object.keys(rec)) {
      pick[f] = Number.isFinite(dev[f]) ? dev[f] : rec[f];
    }
    store.paramCache.set(e.motor_id, {rec, dev, pick,
      state: e.dev_init_state, dirty: e.record_dirty, mask: e.mismatch_mask});
  }
  markDirty(DIRTY.PARAMS);
}

export async function fetchDeviceInfo(motorId = 0) {
  const response = await callService(
    '/motor/get_device_info',
    'can_bridge_msgs/srv/GetDeviceInfo',
    {motor_id: motorId},
    6000,
  );
  const info = response?.info || response;
  applyDeviceInfo(info);
  return info;
}

export async function getLifecycleState() {
  const response = await callService(
    '/micro_ros_node/get_state',
    'lifecycle_msgs/srv/GetState',
    {},
    3000,
  );
  const state = response?.current_state || null;
  store.mcuLifecycle = state ? {id: Number(state.id), label: String(state.label || '')} : null;
  markDirty(DIRTY.CONN);
  return store.mcuLifecycle;
}

function subscribeAll() {
  subs.forEach((s) => s.unsubscribe());
  subs = [];
  // 桥心跳与普通周期消息不同：late subscriber 必须拿到最后一帧，随后按 1 Hz 检查新鲜度。
  const statusConnection = ros;
  const statusTopic = new ROSLIB.Topic({
    ros,
    name: TOPICS.robotTransmissionStatus.name,
    messageType: TOPICS.robotTransmissionStatus.type,
    throttle_rate: 100,
    queue_length: 1,
    qos: {reliability: 'reliable', durability: 'transient_local', history: 'keep_last', depth: 1},
  });
  statusTopic.subscribe((message) => {
    if (ros !== statusConnection || !store.conn.connected) return;
    try {
      applyTransmissionStatus(JSON.parse(String(message && message.data)));
    } catch (_e) {
      applyTransmissionStatus(null);
    }
    markDirty(DIRTY.STATUS);
  });
  subs.push(statusTopic);

  sub('motion', (m) => {
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
    // 臂视图的关节角与曲线面板都从这里取数（经 store.motors）。
    markDirty(DIRTY.MOTION);
  });

  sub('state', (m) => {
    const previousSession = store.controlStates.session;
    const accepted = applyControlState(store.controlStates, m);
    if (accepted) {
      const currentSession = String(store.controlStates.session);
      if (previousSession !== 0 && store.controlStates.session !== previousSession) {
        clearConfigResults('session_changed');
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
    if (applyAgentWireStatus(store, m)) markDirty(DIRTY.CONN);
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
          sys.firmware_values.every(v => Number.isInteger(v) && v > 0 && v <= 1000)) {
        store.firmwareReport = {seq: store.firmwareReport.seq + 1, at: t,
          values: [...sys.firmware_values], unsaved: !!sys.firmware_unsaved};
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
  });

  sub('events', (m) => {
    if (m.event_type === 13) {
      if (applyPolicyReceipt(store,m)) markDirty(DIRTY.POLICY, DIRTY.STATUS);
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
    // DEV_INIT(type 15)：记下核对结论与"哪几项对不上"（request_id = 槽位位图）。
    if (m.event_type === 15) {
      store.devInit.set(m.motor_id, {
        state: m.reason,
        mask: m.request_id || 0,
      });
    }
    markDirty(DIRTY.EVENTS, DIRTY.PARAMS);
  });

  sub('imu', (m) => {
    const q = m.orientation;
    const e = quatToEuler(q.x, q.y, q.z, q.w);
    store.imu.roll = e.roll;
    store.imu.pitch = e.pitch;
    store.imu.yaw = e.yaw;
    store.imu.q = q;
    store.imu.count++;
    store.imu.last = Date.now();
    markDirty(DIRTY.IMU);
  });

  sub('temp', (m) => {
    store.temp = Number.isFinite(m?.temperature) ? m.temperature : m?.data;
    store.tempLast = Date.now();
    markDirty(DIRTY.IMU);
  });

  // 主机控制栈的实测关节反馈，按模型身份与新鲜度接收。
  sub('jointStates', (m) => {
    if (!jointFeedbackMatchesModel()) return;
    const now = Date.now();
    const names = Array.isArray(m && m.name) ? m.name : [];
    const positions = Array.isArray(m && m.position) ? m.position : [];
    for (let i = 0; i < names.length; i++) {
      const position = positions[i];
      if (typeof names[i] === 'string' && Number.isFinite(position)) {
        store.jointStates.byName[names[i]] = {
          position,
          velocity: Number.isFinite(m.velocity?.[i]) ? m.velocity[i] : NaN,
          effort: Number.isFinite(m.effort?.[i]) ? m.effort[i] : NaN,
          at: now,
        };
      } else if (typeof names[i] === 'string') delete store.jointStates.byName[names[i]];
    }
    if (names.length) store.jointStates.jointNames = [
      ...new Set(names.filter((name) => typeof name === 'string'))];
    store.jointStates.count++;
    store.jointStates.last = now;
    markDirty(DIRTY.MOTION);
  });

  // 台架状态（bench_manager，JSON 字符串）：仿真 / Agent / 电机后端三段的唯一
  // 数据入口（命令的服务响应也经 applyBenchStatus 写同一份）。
  sub('bench', (m) => {
    try { applyBenchStatus(JSON.parse(m.data)); }
    catch (_e) { /* 非法帧不入 store；下一帧会重来 */ }
  });

  // ros2_control 模型、模式、路由与错误（含 trajectory_state）。
  sub('commandStatus', (m) => {
    try {
      const data = JSON.parse(m.data);
      applyCommandStatus(data);
      if (data && data.trajectory_state) observeTrajectory(data.trajectory_state);
    } catch (_e) {
      applyCommandStatus(null);
    }
  });
  sub('simPositionStatus', (m) => {
    let data;
    try { data = JSON.parse(m.data); } catch (_e) { data = null; }
    store.simPositionStatus = {last: Date.now(), data};
    markDirty(DIRTY.STATUS);
  });
}

// 调 ROS 服务（bench_manager 的 /bench/*）。服务不存在/节点未起时，roslibjs 的
// 失败回调行为不统一，所以恒有超时兜底；resolve 出来的是响应对象（是否成功看
// 调用方——bench 服务的 ok/error 字段由 core/bench.js 统一裁决）。
export function callService(name, type, request, timeoutMs = 8000) {
  return new Promise((resolve, reject) => {
    if (controlMode === 'readonly') {
      reject(new Error('只读页面不开放控制或维护服务')); return;
    }
    if (!ros || !store.conn.connected) {
      reject(new Error('未连接 rosbridge'));
      return;
    }
    const connection = ros;
    const service = new ROSLIB.Service({ros, name, serviceType: type});
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

// 等首帧 system_status 到达（连接后参数自动读取的前置，Task 8 用）。
export function waitSysStatus(timeoutMs = 2000) {
  if (store.sysStatus.seen) return Promise.resolve(true);
  return new Promise((resolve) => {
    const deadline = Date.now() + timeoutMs;
    const timer = setInterval(() => {
      if (store.sysStatus.seen) { clearInterval(timer); resolve(true); }
      else if (Date.now() >= deadline) { clearInterval(timer); resolve(false); }
    }, 50);
  });
}

// 等下一帧 device_param_report（帧计数变大）。无完成序号可用，REPORT/写事务
// 都靠它确认帧到了。
export async function waitDevParamSeq(baseline, timeoutMs) {
  const deadline = Date.now() + timeoutMs;
  while (Date.now() < deadline) {
    if (store.devParamSeq > baseline) return 0;
    if (!store.conn.connected) return -2;
    await new Promise((r) => setTimeout(r, 100));
  }
  return -3;
}
