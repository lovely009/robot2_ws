// 共享状态：所有跨屏数据都住在这里。面板只读它、只通过 ui 动作改它。
// 订阅由 Vue 工作台全局建立并写这里，与屏是否可见无关——所以切屏不丢曲线缓冲与事件列表。
// 脏标记与 rAF 渲染循环也在本文件内实现，不另开模块。
import { createControlStateCache, describeMotorState } from './motor_state.js';

export function applyAgentWireStatus(state, message, now = Date.now()) {
  const uint32 = value => Number.isInteger(value) && value >= 0 && value <= 0xffffffff;
  if (!Number.isSafeInteger(message.session_id) || message.session_id <= 0 ||
      !Number.isInteger(message.schema_version) ||
      ![message.sequence, message.snapshot_tick_ms, message.clock_sync_age_ms,
        message.clock_sync_failures, message.lease_valid_until_tick_ms].every(uint32) ||
      typeof message.clock_valid !== 'boolean' || typeof message.lease_active !== 'boolean') return false;
  const previous = state.agentWireStatus?.data;
  if (previous && (message.session_id < previous.session_id ||
      (message.session_id === previous.session_id && ((message.sequence - previous.sequence) | 0) <= 0))) return false;
  state.agentWireStatus = {at: now, data: message};
  return true;
}

export const store = {
  // text：顶栏连接文案（setConn 写它，renderShell 画它）
  conn: {connected: false, text: '未连接', url: '', agent: 'stopped', agentDevice: '',
         agentError: '', agentReturncode: null, canStop: false, lastMsgAt: 0,
         autoLoaded: false, benchLoaded: false},
  // motors: id -> {topo, motion, status}（与重构前的 state.motors 同形状，搬迁期不改字段名）
  motors: new Map(),
  controlStates: createControlStateCache(),
  agentWireStatus: null,
  lastError: 0,
  lastErrorMotor: -1,
  sysStatus: {refresh_seq: 0, set_zero_seq: 0, clear_fault_seq: 0,
              save_seq: 0, save_in_progress: false, seen: false},
  devParamSeq: 0,
  devInit: new Map(),
  policy: new Map(),
  events: [],
  imu: {count: 0, last: 0, roll: 0, pitch: 0, yaw: 0, q: null},
  temp: undefined,
  tempLast: 0,
  motionCount: 0,
  // TF（关节侧）：child_frame → {parent, xyz, quat, dynamic, at, count}。关节列表
  // 按模型 URDF 的父子 link 配这里的条目，与电机流（motion_states）互不相干。
  tf: new Map(),
  // 模型关节表（collectModelJoints 装入）：关节列表除 TF 之外的另一半来源。
  modelJoints: [],
  selectedId: 0,
  // 曲线：motor_id -> channel -> {t:[], values:[]}；各通道按自身采样时刻对齐。
  chartBuf: new Map(),
  chart: {running: true, windowMs: 10000},
  history: [],
  paramCache: new Map(),
  firmwareReport: {seq: 0, at: 0, values: null, unsaved: false},
  configResults: new Map(),
  configSessionId: null,
  // 写事务的结果文本（key -> {ok, text, t}）：同一事务的按钮分布在设备屏与使能屏，
  // 结果不能只写给某一个 DOM。core/device.js 的 writeResult 写它。
  results: new Map(),
  // 轨迹：activeTrajectoryId = 页面最近下发的轨迹；state = 最近一帧
  // trajectory_state（动作屏的窗口交棒判定与进度显示都读它，见 core/ros.js）
  traj: {activeTrajectoryId: 0, waitingState: false, segmentIndex: 0, segmentCount: 0,
         state: null},
  // 台架（bench_manager 的 /bench/status 快照）：simulation / agent / motor_backend /
  // gravity_feedforward 四段；core/ros.js 订阅写入，台架服务（core/agent.js）的命令响应
  // 也经 applyBenchStatus 写同一份。
  bench: {simulation: null, agent: null, motor_backend: null,
          gravity_feedforward: null, gravity_feedforward_last: 0, feedback: null},
  // /joint_states：byName 保存每个关节的实际测量值；count/last 供其他面板判断流是否在。
  jointStates: {count: 0, last: 0, byName: {}, jointNames: []},
  // /robot_transmission/status：桥心跳；页面显示模型由 MODEL 决定，控制模型只能以它为准。
  transmissionStatus: {last: 0, data: null},
  // 当前显示模型的传动组（/api/model-transmissions）；selectedGroup 可含逗号分隔的多个组。
  // driver_file/direct_mapping 为跨面板固定名；driverFile/directJoints 保留已有调用。
  modelTransmissions: {model: '', driver_file: '', groups: [], error: '',
    driverFile: '', directJoints: {}, direct_mapping: {}},
  selectedGroup: '',
  selectedJoint: '',
  // /control/status：ros2_control 的模型、模式、输出路由与拒绝原因。
  commandStatus: {last: 0, data: null},
  simPositionStatus: {last: 0, data: null},
};

export function orderedMotorIds() {
  return [...store.motors.keys()].sort((a, b) => a - b);
}

let dirty = new Set();
let dirtyListener = null;

// 标脏：面板的 render(dirty) 会收到自上次渲染以来的键集合
export function markDirty(...keys) {
  for (const k of keys) dirty.add(k);
}
export function onDirty(fn) { dirtyListener = fn; }

// 单一 rAF 循环：没有任何面板标脏时不做任何事，因此未挂载的屏不空转。
export function runRenderLoop() {
  let active = true;
  let frameId = 0, lastAgeCheck = 0;
  const frame = () => {
    if (!active) return;
    const now = Date.now();
    if (now-lastAgeCheck >= 500) {
      lastAgeCheck = now;
      dirty.add(DIRTY.STATUS);
    }
    if (dirty.size && dirtyListener) {
      const batch = dirty;
      dirty = new Set();
      dirtyListener(batch);
    }
    frameId = requestAnimationFrame(frame);
  };
  frameId = requestAnimationFrame(frame);
  return () => { active = false; cancelAnimationFrame(frameId); };
}

// 脏标记的键（面板按这些键判断要重绘什么）
export const DIRTY = {
  CONN: 'conn', TOPOLOGY: 'topology', MOTION: 'motion', STATUS: 'status',
  EVENTS: 'events', HISTORY: 'history', PARAMS: 'params', TRAJ: 'traj',
  IMU: 'imu', POLICY: 'policy', SELECTED: 'selected', TF: 'tf', MODEL: 'model',
  BENCH: 'bench', COMMAND: 'command',
};

// 写操作是否可用。断连时页面不猜、不降级、不静默重试（与 motor_hardware_interface 的
// fail-fast 语义一致）：按钮直接禁用，而不是等请求超时。
export function writesAllowed() { return store.conn.connected; }

// 统一给一组按钮设禁用态；面板在自己的 render 里调用。
export function syncWriteButtons(ids, allow) {
  for (const id of ids) {
    const el = document.getElementById(id);
    if (el) el.disabled = !allow;
  }
}

// 台架状态入 store（/bench/status 话题帧与 bench 服务的响应共用这一条路）。
// agent 段顺带同步进 conn.agent*（顶栏文案由 Vue 工作台渲染）；
// 页面本地的「starting/stopping」瞬时态会被下一帧服务端真实状态覆盖，属预期。
// 无效状态立即撤销身份，不能用旧会话继续控制。
export function applyCommandStatus(snapshot) {
  const valid = snapshot && typeof snapshot.model === 'string' && snapshot.model
    && Array.isArray(snapshot.joint_names) && snapshot.joint_names.length
    && snapshot.joint_names.every(name => typeof name === 'string' && name)
    && new Set(snapshot.joint_names).size === snapshot.joint_names.length
    && typeof snapshot.mode === 'string'
    && snapshot.output_topic === '/motor/host_command_array'
    && snapshot.trajectory_output_topic === '/host_control/trajectory_controller/follow_joint_trajectory'
    && (snapshot.gravity_enabled === null || typeof snapshot.gravity_enabled === 'boolean')
    && (snapshot.effort_gain === null || (Number.isFinite(snapshot.effort_gain) && snapshot.effort_gain >= 0));
  store.commandStatus = valid ? {data: snapshot, last: Date.now()} : {data: null, last: 0};
  markDirty(DIRTY.COMMAND);
}

export function applyBenchStatus(snapshot) {
  if (!snapshot || typeof snapshot !== 'object') return;
  const bench = store.bench;
  if (snapshot.simulation) bench.simulation = snapshot.simulation;
  if (snapshot.motor_backend) bench.motor_backend = snapshot.motor_backend;
  if (snapshot.feedback) bench.feedback = snapshot.feedback;
  if (snapshot.gravity_feedforward) {
    bench.gravity_feedforward = snapshot.gravity_feedforward;
    bench.gravity_feedforward_last = Date.now();
  }
  if (snapshot.agent) {
    bench.agent = snapshot.agent;
    const c = store.conn;
    const state = snapshot.agent.state || 'stopped';
    c.agentDevice = snapshot.agent.device || '';
    // 1 Hz 快照只管**进程状态**，说不了「为什么没起来」（它压根没有 error 字段）。
    // 上一次命令失败后进程就是 stopped，若无条件覆盖，顶栏的失败原因活不过一秒。
    // 所以只要进程还没起来就整段留着，由下一次命令（startAgent/stopAgent 会重置）
    // 或真的起来清掉。
    if (c.agent !== 'error' || state !== 'stopped') {
      c.agent = state;
      c.agentError = snapshot.agent.error || '';
    }
    c.agentReturncode = snapshot.agent.returncode === undefined
      ? null : snapshot.agent.returncode;
    // 服务端状态没有 canStop 字段时按 state 推导；页面合成的错误态可显式带它
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

// 电机汇总计数：顶栏 chips 与「电机状态」表共用同一口径（在线/运行/冲突/故障），
// 避免两处各算一遍导致漂移。
export function countMotors() {
  let online = 0, fault = 0, active = 0, conflict = 0;
  for (const [id,r] of store.motors) {
    const view = describeMotorState(store,id);
    const st = r.status || {}, t = r.topo || {};
    const isOnline = view.online;
    if (isOnline) online++;
    if (view.fault) fault++;
    if (view.active) active++;
    if (t.conflict || st.lifecycle === 7) conflict++;
  }
  return {online, total: store.motors.size, fault, active, conflict};
}

// 曲线数据入缓冲：每通道独立时间轴（缺失 NaN 保留，绘制时跳过）。
// 上限 1500 样本/通道 = 30 s × 50 Hz，恰好是最大时间窗；超过就从头丢。
// 上限放在这里而不是绘制侧：没有曲线面板时缓冲也必须收敛，否则切屏一整天后内存只涨不落。
const CHART_CAP = 1500;

export function chartPush(id, ch, val, t) {
  let channels = store.chartBuf.get(id);
  if (!channels) {
    channels = {};
    store.chartBuf.set(id, channels);
  }
  const b = channels[ch] || (channels[ch] = {t: [], values: []});
  b.values.push(Number.isFinite(val) ? val : NaN);
  b.t.push(t);
  if (b.t.length > CHART_CAP) {
    b.t.shift();
    b.values.shift();
  }
}

// URL 参数：换模型或换接线不改代码（?joints=1:joint_link1,2:joint_link2&model=<模型目录名>）
export const URLQ = new URLSearchParams(location.search);
// 手工覆盖（?joints=1:joint_link1,2:joint_link2）——最高优先，给了就不问服务器。
// 保留它是为了非常规接线与排查，不是常规路径。
const JOINT_OVERRIDE = (() => {
  const q = URLQ.get('joints');
  if (!q) return null;
  const m = {};
  for (const pair of q.split(',')) {
    const [id, j] = pair.split(':');
    if (id && j) m[parseInt(id, 10)] = j;
  }
  return m;
})();

// 兼容旧面板的单电机单关节图。耦合传动组的物理关系只保存在 modelTransmissions，
// 不能按数组长度 zip 成一对一。新代码选择、反馈与命令均应读 groups。
export const JOINT_OF_MOTOR = JOINT_OVERRIDE || {
  1: 'joint_link1', 2: 'joint_link2', 3: 'joint_link3', 4: 'joint_link4',
  5: 'joint_link5', 6: 'joint_link6', 7: 'joint_link7', 8: 'joint_link8',
  9: 'joint_link9',
  10: 'joint_link1.1', 11: 'joint_link2.2', 12: 'joint_link3.3',
  13: 'joint_link4.4', 14: 'joint_link5.5', 15: 'joint_link6.6',
  16: 'joint_link7.7', 17: 'joint_link8.8', 18: 'joint_link9.9',
};
export const jointMapSource = {kind: JOINT_OVERRIDE ? 'url' : 'builtin', error: ''};
let modelLoadGeneration = 0;

function publicGroups(groups) {
  return (Array.isArray(groups) ? groups : []).map((g) => ({
    ...g,
    motors: Array.isArray(g.motors) ? g.motors.map((m) => ({...m})) : [],
    joints: Array.isArray(g.joints) ? [...g.joints] : [],
  }));
}

function selectedForModel(groups, current='') {
  if (groups.some((g) => g.name === current)) return current;
  return groups[0]?.name || '';
}

// 从驱动或只读电机拓扑读取分组，保留硬件可用状态。响应代次与当前
// MODEL 同时校验，晚到的旧模型响应不能覆盖页面当前显示的模型。
export async function loadModelTransmissions(name=MODEL) {
  const generation = ++modelLoadGeneration;
  store.modelTransmissions = {model: name, driver_file: '', groups: [], error: '',
    driverFile: '', directJoints: {}, direct_mapping: {}};
  store.selectedGroup = '';
  store.selectedJoint = '';
  jointMapSource.kind = 'loading';
  jointMapSource.error = '';
  markDirty(DIRTY.MODEL, DIRTY.TOPOLOGY);
  try {
    const resp = await fetch(
      `/api/model-transmissions?model=${encodeURIComponent(name)}`, {cache: 'no-store'});
    if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
    const data = await resp.json();
    if (generation !== modelLoadGeneration || name !== MODEL) return store.modelTransmissions;
    if (data.model !== name || !Array.isArray(data.groups)) {
      throw new Error('接口返回的传动组无效');
    }
    const groups = publicGroups(data.groups);
    const driverFile = typeof data.driver_file === 'string' ? data.driver_file : '';
    const direct = data.direct_joints && typeof data.direct_joints === 'object'
      ? {...data.direct_joints} : {};
    store.modelTransmissions = {model: data.model, driver_file: driverFile, groups,
      error: '', driverFile, directJoints: direct, direct_mapping: {...direct},
      mapping_kind: data.mapping_kind, hardware_ready: data.hardware_ready,
      hardware_block_reason: data.hardware_block_reason,
      policy_defaults: data.policy_defaults || {}};
    store.selectedGroup = selectedForModel(groups);
    jointMapSource.kind = data.hardware_ready === false ? 'topology' : 'api';
    markDirty(DIRTY.MODEL, DIRTY.TOPOLOGY);
  } catch (e) {
    if (generation !== modelLoadGeneration || name !== MODEL) return store.modelTransmissions;
    jointMapSource.kind = 'unavailable';
    jointMapSource.error = String((e && e.message) || e);
    store.modelTransmissions.error = jointMapSource.error;
    console.warn(`[web_host] ${name} 传动组读取失败：${jointMapSource.error}`);
    markDirty(DIRTY.MODEL, DIRTY.TOPOLOGY);
  }
  return store.modelTransmissions;
}

// 旧入口只用于尚未迁移的调用方；不再影响臂姿态或关节选择。
export async function loadModelJoints() {
  const model = await loadModelTransmissions(MODEL);
  if (model.model !== MODEL) return jointMapSource;
  for (const k of Object.keys(JOINT_OF_MOTOR)) delete JOINT_OF_MOTOR[k];
  for (const [id, joint] of Object.entries(JOINT_OVERRIDE || model.directJoints)) {
    JOINT_OF_MOTOR[parseInt(id, 10)] = joint;
  }
  return jointMapSource;
}

// 默认模型与 robot_sim 的 launch 默认值保持一致（half_body_robot，四个 launch 都是它）。
const DEFAULT_MODEL = 'half_body_robot';
const MODEL_STORAGE_KEY = 'web_host.model';
function readSavedModel() {
  try { return localStorage.getItem(MODEL_STORAGE_KEY) || ''; } catch (_e) { return ''; }
}
// 当前显示的模型：?model= 最高优先，其次上次页内切换（localStorage），最后默认值。
export let MODEL = URLQ.get('model') || readSavedModel() || DEFAULT_MODEL;

export function jointFeedbackMatchesModel() {
  const feedback = store.bench.feedback;
  if (!feedback) return true;
  if (feedback.source === 'firmware') {
    return feedback.state === 'running' && feedback.model === MODEL
      && feedback.desired_model === MODEL;
  }
  if (feedback.source === 'simulation') return store.bench.simulation?.config?.model === MODEL;
  if (feedback.source === 'external') {
    const {data,last} = store.transmissionStatus, age = Date.now()-last;
    return data?.model === MODEL && last>0 && age>=0 && age<=2500;
  }
  return false;
}

// 页内切换模型：臂视图重载（DIRTY.MODEL）+ 重取关节映射。?joints= 手工覆盖仍最高
// 优先（loadModelJoints 直接返回，换模型不改变手工映射）。
export function setModel(name) {
  if (!name || name === MODEL) return;
  MODEL = name;
  store.modelJoints = [];
  store.jointStates.byName = {};
  store.jointStates.jointNames = [];
  store.jointStates.last = 0;
  store.tf.clear();
  try { localStorage.setItem(MODEL_STORAGE_KEY, name); } catch (_e) { /* 存储禁用时仅本次会话生效 */ }
  markDirty(DIRTY.MODEL);
  loadModelJoints();
}

// 点击一个或多个传动组：一次选择全部涉及组，臂等宽姿态必须同组下发。
export function selectGroups(names) {
  const chosen = [...new Set(names)].filter(
    (n) => store.modelTransmissions.groups.some((g) => g.name === n));
  if (!chosen.length) return;
  const value = chosen.join(',');
  if (store.selectedGroup === value) return;
  store.selectedGroup = value;
  if (!store.modelTransmissions.groups.some(
    (g) => g.joints.includes(store.selectedJoint))) store.selectedJoint = '';
  markDirty(DIRTY.SELECTED, DIRTY.STATUS);
}

// 选中关节供主代理的关节控件读取；关节必须属于模型传动组。
export function selectJoint(name) {
  const exists = store.modelTransmissions.groups.some((g) => g.joints.includes(name));
  store.selectedJoint = exists ? name : '';
  markDirty(DIRTY.SELECTED, DIRTY.STATUS);
}

// 臂视图可切换的模型（有 urdf/<名>.urdf 的目录，由 static_server 的 /api/models 枚举）。
export async function availableModels() {
  const resp = await fetch('/api/models', {cache: 'no-store'});
  if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
  const data = await resp.json();
  return Array.isArray(data && data.models) ? data.models : [];
}

// 页面上最后发出的目标值（history[0]）。⚠️ 这不是固件的设定点：固件侧的钳位、
// 软停、整包拒收页面都看不到；轨迹在播期间（target_source == 2）目标由固件插值，
// 页面无从得知，此时宁可不画（调用方负责判 target_source）。
export function targetOf(id) {
  const h = store.history[0];
  if (!h) return NaN;
  const t = h.targets.find((x) => x.id === id);
  return t ? t.pos : NaN;
}

// 两张表独立传入 showAll，不共享复选框状态。
export function paramRowIds(showAll) {
  const all = [...store.paramCache.keys()].sort((a, b) => a - b);
  return (showAll || !store.selectedId) ? all : all.filter((i) => i === store.selectedId);
}

export function paramReadState(id) {
  const entry = store.paramCache.get(id);
  const fields = ['pos_min', 'pos_max', 'vel_limit', 'torque_limit', 'limit_protect', 'direction', 't_max', 'node_id'];
  const count = (source) => fields.filter(field => Number.isFinite(entry?.[source]?.[field])).length;
  const device = count('dev'), record = count('rec'), usable = count('pick');
  return {device, record, usable, verified: entry?.state === 2 && device === fields.length};
}
