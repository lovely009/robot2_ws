// 共享状态：所有跨屏数据都住在这里。面板只读它、只通过 ui 动作改它。
// 订阅在 main.js 全局建立并写这里，与屏是否可见无关——所以切屏不丢曲线缓冲与事件列表。
// 脏标记与 rAF 渲染循环也在本文件内实现，不另开模块。
import { devSlotNames, devRecoveryText } from './format.js';

export const store = {
  // text：顶栏连接文案（setConn 写它，renderShell 画它）
  conn: {connected: false, text: '未连接', url: '', agent: 'stopped', agentDevice: '',
         agentError: '', agentReturncode: null, canStop: false, lastMsgAt: 0,
         autoLoaded: false},
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
  selectedId: 0,
  // 曲线：motor_id -> {t:[], position:[], velocity:[], effort:[], current:[]}
  chartBuf: new Map(),
  chart: {running: true, windowMs: 10000},
  history: [],
  paramCache: new Map(),
  // 写事务的结果文本（key -> {ok, text, t}）：同一事务的按钮分布在设备屏与使能屏，
  // 结果不能只写给某一个 DOM。core/device.js 的 writeResult 写它。
  results: new Map(),
  traj: {activeTrajectoryId: 0, waitingState: false, segmentIndex: 0, segmentCount: 0},
};

let dirty = new Set();
let dirtyListener = null;

// 标脏：面板的 render(dirty) 会收到自上次渲染以来的键集合
export function markDirty(...keys) {
  for (const k of keys) dirty.add(k);
}
export function onDirty(fn) { dirtyListener = fn; }

// 单一 rAF 循环：没有任何面板标脏时不做任何事，因此未挂载的屏不空转。
export function runRenderLoop() {
  const frame = () => {
    if (dirty.size && dirtyListener) {
      const batch = dirty;
      dirty = new Set();
      dirtyListener(batch);
    }
    requestAnimationFrame(frame);
  };
  requestAnimationFrame(frame);
}

// 脏标记的键（面板按这些键判断要重绘什么）
export const DIRTY = {
  CONN: 'conn', TOPOLOGY: 'topology', MOTION: 'motion', STATUS: 'status',
  EVENTS: 'events', HISTORY: 'history', PARAMS: 'params', TRAJ: 'traj',
  IMU: 'imu', POLICY: 'policy', SELECTED: 'selected', ALERTS: 'alerts',
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

// 电机汇总计数：顶栏 chips 与状态翼共用同一口径（在线/运行/冲突/故障），避免
// 两处各算一遍导致漂移。
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
export const JOINT_OF_MOTOR = (() => {
  const q = URLQ.get('joints');
  if (!q) return {1: 'joint_link1', 2: 'joint_link2', 3: 'joint_link3', 4: 'joint_link4'};
  const m = {};
  for (const pair of q.split(',')) {
    const [id, j] = pair.split(':');
    if (id && j) m[parseInt(id, 10)] = j;
  }
  return m;
})();
// 默认模型与 robot_arm_gazebo/launch/gazebo.launch.py 的默认值保持一致；
// _001 已随模型改名退役（c549d29 删除了它的 URDF），旧默认值会让臂视图默认 404。
export const MODEL = URLQ.get('model') || 'zhoubuguanjie_youbi_002';

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
