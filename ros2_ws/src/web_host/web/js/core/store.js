// 共享状态：所有跨屏数据都住在这里。面板只读它、只通过 ui 动作改它。
// 订阅在 main.js 全局建立并写这里，与屏是否可见无关——所以切屏不丢曲线缓冲与事件列表。
// 脏标记与 rAF 渲染循环也在本文件内实现，不另开模块。
import { devSlotNames, devRecoveryText } from './format.js';

export const store = {
  // text：顶栏连接文案（setConn 写它，renderShell 画它）
  conn: {connected: false, text: '未连接', url: '', agent: 'stopped', agentDevice: '',
         agentError: '', agentReturncode: null, canStop: false, lastMsgAt: 0,
         autoLoaded: false, benchLoaded: false},
  // motors: id -> {topo, motion, status}（与重构前的 state.motors 同形状，搬迁期不改字段名）
  motors: new Map(),
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
  // 置顶序（点击过的电机，最近在前）。左栏关节列表与右栏状态表共用一个次序：
  // 点一台 → 两处都排最上，观察与操作对照不用来回找。
  pinned: [],
  // 曲线：motor_id -> {t:[], position:[], velocity:[], effort:[], current:[]}
  chartBuf: new Map(),
  chart: {running: true, windowMs: 10000},
  history: [],
  paramCache: new Map(),
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
          gravity_feedforward: null},
  // /joint_states（关节侧实测构型）：页面只关心它的**新鲜度**——叠加节点的 τ_g 全靠
  // 它，超时即整帧直通（见 core/format.js 的 jointStates 话题注释）。
  jointStates: {count: 0, last: 0},
  // 命令节拍节点（/motor_command/status）：`data.output_topic` 是命令**当前实际发往**
  // 哪里——改道由服务端做（ADR 0014），页面只有读它才能核对开关与接线是否一致。
  commandStatus: {last: 0, data: null},
};

// 置顶：把 id 提到 pinned 最前（LIFO）。已在最前时不动。
export function pinMotor(id) {
  const i = store.pinned.indexOf(id);
  if (i === 0) return;
  if (i > 0) store.pinned.splice(i, 1);
  store.pinned.unshift(id);
}

// 面板行序：置顶优先（最近点击的在前），其余按 id 升序。陈旧 id 无害（motors 里
// 没有就排不上来）。
export function orderedMotorIds() {
  return [...store.motors.keys()].sort((a, b) => {
    const pa = store.pinned.indexOf(a), pb = store.pinned.indexOf(b);
    if (pa >= 0 && pb >= 0) return pa - pb;
    if (pa >= 0) return -1;
    if (pb >= 0) return 1;
    return a - b;
  });
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
  let frameId = 0;
  const frame = () => {
    if (!active) return;
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
// agent 段顺带同步进 conn.agent*（顶栏文案由 main.js 的 renderShell 画）；
// 页面本地的「starting/stopping」瞬时态会被下一帧服务端真实状态覆盖，属预期。
// 命令节拍节点状态入 store（/motor_command/status）。整段留原样：页面不解释它的
// 字段，只把 output_topic 与开关的意图对账，其余原样显示给操作者。
export function applyCommandStatus(snapshot) {
  if (!snapshot || typeof snapshot !== 'object') return;
  store.commandStatus.data = snapshot;
  store.commandStatus.last = Date.now();
  markDirty(DIRTY.COMMAND);
}

export function applyBenchStatus(snapshot) {
  if (!snapshot || typeof snapshot !== 'object') return;
  const bench = store.bench;
  if (snapshot.simulation) bench.simulation = snapshot.simulation;
  if (snapshot.motor_backend) bench.motor_backend = snapshot.motor_backend;
  if (snapshot.gravity_feedforward) {
    bench.gravity_feedforward = snapshot.gravity_feedforward;
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
  markDirty(DIRTY.BENCH);
}

// 电机汇总计数：顶栏 chips 与「电机状态」表共用同一口径（在线/运行/冲突/故障），
// 避免两处各算一遍导致漂移。
export function countMotors() {
  let online = 0, fault = 0, active = 0, conflict = 0;
  for (const r of store.motors.values()) {
    const st = r.status || {}, t = r.topo || {};
    const isOnline = st.online !== undefined ? st.online : t.online;
    if (isOnline) online++;
    if (st.unified_fault_code) fault++;
    if (st.lifecycle === 4) active++;                       // ACTIVE = 固件此刻真在运动
    if (t.conflict || st.lifecycle === 7) conflict++;
  }
  return {online, total: store.motors.size, fault, active, conflict};
}

// 曲线数据入缓冲：每通道独立时间轴（缺失 NaN 保留，绘制时跳过）。
// 上限 1500 样本/通道 = 30 s × 50 Hz，恰好是最大时间窗；超过就从头丢。
// 上限放在这里而不是绘制侧：没有曲线面板时缓冲也必须收敛，否则切屏一整天后内存只涨不落。
const CHART_CAP = 1500;

export function chartPush(id, ch, val, t) {
  let b = store.chartBuf.get(id);
  if (!b) {
    b = {t: [], position: [], velocity: [], effort: [], current: []};
    store.chartBuf.set(id, b);
  }
  b[ch].push(Number.isFinite(val) ? val : NaN);
  b.t.push(t);
  if (b.t.length > CHART_CAP) {
    b.t.shift();
    for (const k of ['position', 'velocity', 'effort', 'current']) b[k].shift();
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

// motor_id → 关节名。**权威来源是模型自己的 config/motor_driver.yaml**，
// 由 static_server 的 /api/model-joints 提供，启动时经 loadModelJoints() 装入。
//
// 经典入口在首次加载前保留默认结构；正式读取会先清空，失败时不回退。
// 对象就地更新，保持臂视图持有的引用有效。
export const JOINT_OF_MOTOR = JOINT_OVERRIDE || {
  1: 'joint_link1', 2: 'joint_link2', 3: 'joint_link3', 4: 'joint_link4',
  5: 'joint_link5', 6: 'joint_link6', 7: 'joint_link7', 8: 'joint_link8',
  9: 'joint_link9',
  10: 'joint_link1.1', 11: 'joint_link2.2', 12: 'joint_link3.3',
  13: 'joint_link4.4', 14: 'joint_link5.5', 15: 'joint_link6.6',
  16: 'joint_link7.7', 17: 'joint_link8.8', 18: 'joint_link9.9',
};

// 映射来源：url 手工覆盖 / api 模型配置 / loading / unavailable；builtin 只在首次加载前存在。
export const jointMapSource = {kind: JOINT_OVERRIDE ? 'url' : 'builtin', error: ''};
let modelLoadGeneration = 0;

// 清空后读取当前模型映射；失败保持空表，旧请求不得覆盖新的模型。
export async function loadModelJoints() {
  if (JOINT_OVERRIDE) return jointMapSource;
  const generation = ++modelLoadGeneration;
  const requestedModel = MODEL;
  for (const k of Object.keys(JOINT_OF_MOTOR)) delete JOINT_OF_MOTOR[k];
  jointMapSource.kind = 'loading';
  jointMapSource.error = '';
  markDirty(DIRTY.TOPOLOGY);
  try {
    const resp = await fetch(
      `/api/model-joints?model=${encodeURIComponent(requestedModel)}`, {cache: 'no-store'});
    if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
    const data = await resp.json();
    if (generation !== modelLoadGeneration) return jointMapSource;
    const joints = data && data.joints;
    if (!joints || typeof joints !== 'object' || !Object.keys(joints).length) {
      throw new Error('接口返回的映射为空');
    }
    for (const k of Object.keys(JOINT_OF_MOTOR)) delete JOINT_OF_MOTOR[k];
    for (const [id, joint] of Object.entries(joints)) {
      JOINT_OF_MOTOR[parseInt(id, 10)] = joint;
    }
    jointMapSource.kind = 'api';
    jointMapSource.error = '';
    markDirty(DIRTY.TOPOLOGY);
  } catch (e) {
    if (generation !== modelLoadGeneration) return jointMapSource;
    jointMapSource.kind = 'unavailable';
    jointMapSource.error = String((e && e.message) || e);
    console.warn(
      `[web_host] /api/model-joints?model=${requestedModel} 取不到（${jointMapSource.error}）：`
      + '该模型仅提供几何预览，不沿用其他模型的设备映射。');
    markDirty(DIRTY.TOPOLOGY);
  }
  return jointMapSource;
}
// 默认模型与 robot_sim 的 launch 默认值保持一致（half_body_robot，四个 launch 都是它）。
// 旧默认值（zhoubuguanjie_youbi_001 / chest_upper_00 / robot_all）对应的模型都已删除，
// 留着会让臂视图默认 404。
const DEFAULT_MODEL = 'half_body_robot';
const MODEL_STORAGE_KEY = 'web_host.model';
function readSavedModel() {
  try { return localStorage.getItem(MODEL_STORAGE_KEY) || ''; } catch (_e) { return ''; }
}
// 当前显示的模型：?model= 最高优先，其次上次页内切换（localStorage），最后默认值。
export let MODEL = URLQ.get('model') || readSavedModel() || DEFAULT_MODEL;

// 页内切换模型：臂视图重载（DIRTY.MODEL）+ 重取关节映射。?joints= 手工覆盖仍最高
// 优先（loadModelJoints 直接返回，换模型不改变手工映射）。
export function setModel(name) {
  if (!name || name === MODEL) return;
  MODEL = name;
  try { localStorage.setItem(MODEL_STORAGE_KEY, name); } catch (_e) { /* 存储禁用时仅本次会话生效 */ }
  markDirty(DIRTY.MODEL);
  loadModelJoints();
}

// 臂视图可切换的模型（有 urdf/<名>.urdf 的目录，由 static_server 的 /api/models 枚举）。
export async function availableModels() {
  const resp = await fetch('/api/models', {cache: 'no-store'});
  if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
  const data = await resp.json();
  return Array.isArray(data && data.models) ? data.models : [];
}

// 任意模型的 motor_id → 关节名（按名字缓存）。与 loadModelJoints 同源同接口
// （/api/model-joints，权威是模型自己的 config/motor_driver.yaml），区别只在**不问
// 当前模型**：重力前馈面板可以选一个不同于臂视图的模型——HIL 会话是操作者手起的，
// 页面没有别的途径知道台上是哪台。
const modelJointsCache = new Map();

export async function modelJointsFor(name) {
  if (!name) throw new Error('未指定模型');
  if (!modelJointsCache.has(name)) {
    const resp = await fetch(
      `/api/model-joints?model=${encodeURIComponent(name)}`, {cache: 'no-store'});
    if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
    const data = await resp.json();
    const joints = data && data.joints;
    if (!joints || typeof joints !== 'object' || !Object.keys(joints).length) {
      throw new Error(`模型 ${name} 没有可用关节映射`);
    }
    modelJointsCache.set(name, joints);
  }
  return modelJointsCache.get(name);
}

// `motor_id:关节名,电机:关节…`——服务端（bench_manager）逐条校验的表形状。
export function jointMapSpec(joints) {
  return Object.entries(joints)
    .map(([id, joint]) => `${id}:${joint}`)
    .join(',');
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

// 状态列悬停提示：DEV_INIT 事件带来的槽位位图。state=3(MISMATCH) 位图是"对不上"
// 的槽位，state=4(UNVERIFIED) 位图是"没读到"的槽位（见 MotorEvent.msg）。
export function devDetailTitle(id) {
  const info = store.devInit.get(id);
  if (!info || !info.mask) return '';
  const slots = devSlotNames(info.mask).join('、');
  if (info.state === 3) {
    return `设备参数与固件记录不一致：${slots}（${devRecoveryText(info.mask)}）`;
  }
  if (info.state === 4) {
    return `设备参数回读未完成，未收到的对象：${slots}（重连或再点「读取」重试）`;
  }
  return '';
}

// 行筛选（两张表各自的「显示全部」都走它）。原来直接读旧复选框的选中态，现在改为
// 调用方传入 showAll —— 重构后设备参数表与使能表分处两屏，各自一个复选框，不再
// 共用一个 DOM 元素。
export function paramRowIds(showAll) {
  const all = [...store.paramCache.keys()].sort((a, b) => a - b);
  return (showAll || !store.selectedId) ? all : all.filter((i) => i === store.selectedId);
}
