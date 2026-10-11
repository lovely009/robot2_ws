// Robot2 Web 上位机：监控（topology/motion/status/events/imu）+ 控制下发
// （command 快照 / trajectory 多点轨迹；下行配置话题已随固件裁剪移除，
// 型号绑定/授权为固件编译期）。
// 话题与 can_bridge_msgs 对齐固件；QoS 由 rosbridge 2.x 按发布方自动协商。
(() => {
'use strict';

// ---------- 常量 ----------
const TOPICS = {
  topology: {name: '/micro_ros_up/motor_topology', type: 'can_bridge_msgs/msg/MotorTopology'},
  motion: {name: '/micro_ros_up/motion_states', type: 'can_bridge_msgs/msg/MotorMotionStateArray'},
  status: {name: '/micro_ros_up/motor_status_array', type: 'can_bridge_msgs/msg/MotorStatusArray'},
  events: {name: '/micro_ros_up/motor_events', type: 'can_bridge_msgs/msg/MotorEvent'},
  imu: {name: '/micro_ros_up/imu/data', type: 'sensor_msgs/msg/Imu'},
  temp: {name: '/micro_ros_up/imu/temp', type: 'std_msgs/msg/Float32'},
  command: {name: '/micro_ros_do/motor_command_array', type: 'can_bridge_msgs/msg/MotorCommandArray'},
  trajectory: {name: '/micro_ros_do/motor_trajectory', type: 'can_bridge_msgs/msg/MotorTrajectory'},
  trajectoryState: {name: '/micro_ros_up/trajectory_state', type: 'can_bridge_msgs/msg/MotorTrajectoryState'},
  policy: {name: '/micro_ros_do/motor_policy', type: 'can_bridge_msgs/msg/MotorPolicyArray'},
  // 参数控制面只走以下四条 topic：命令下行、状态上行；不使用 ROS 参数树或服务。
  // 这四项必须与固件的 topic 契约保持一致。
  devParams: {name: '/micro_ros_up/device_param_report', type: 'can_bridge_msgs/msg/DeviceParamReport'},
  sysStatus: {name: '/micro_ros_up/system_status', type: 'can_bridge_msgs/msg/SystemStatus'},
  paramSet: {name: '/micro_ros_do/param_set', type: 'can_bridge_msgs/msg/ParamSetArray'},
  sysCmd: {name: '/micro_ros_do/system_command', type: 'can_bridge_msgs/msg/SystemCommand'},
};

const LIFECYCLE = ['ABSENT', 'DISCOVERED', 'IDENTIFYING', 'READY_DISABLED',
  'ACTIVE', 'SAFE_STOP', 'FAULT', 'CONFIG_CONFLICT', 'OFFLINE'];
const EVENT_NAMES = ['DISCOVERED', 'PROFILE_BOUND', 'PROFILE_REJECTED', 'READY',
  'ACTIVE', 'SOFT_STOP', 'DISABLE', 'FAULT', 'CONFIG_CONFLICT', 'OFFLINE',
  'RECONNECTED', 'AUTHORIZATION_LOST', 'SNAPSHOT_REJECTED', 'CONFIG_RESULT',
  'REGISTRY_FULL', 'DEV_INIT', 'TRAJECTORY_REJECTED'];
// 索引即故障码。8 是历史值（ZERO_STALE，2026-09-20 按真机实测删除）——
// 空着不复用，免得旧日志/事件里的 8 被认成别的意思。
const FAULT_NAMES = ['无', '协议', '厂家', 'Profile', '冲突', '离线',
  '设备参数', '策略代数', undefined, '缺武装确认'];
// 设备参数槽位名（DEV_INIT 事件 request_id 携带的槽位位图，1u<<slot）。
// ⚠️ 这是与固件位图的**隐式契约**：槽位顺序必须逐位对齐 firmware 的
// device_raw 槽索引（MOTOR_PROTOCOL_PARAM_SLOT_*），权威定义在
// micro_ros_agent/src/can_bridge_msgs/msg/MotorEvent.msg 的 EVENT_DEV_INIT
// 注释块。固件新增槽位时，这里与那条注释必须同步改——只改一边会静默错位。
const DEV_SLOT_NAMES = ['位置下限', '位置上限', '最大转速', '峰值电流',
  '保护开关(0x2650:00)', '方向(0x2541)'];
// DeviceParamEntry.dev_init_state。6 项全部可写（含此前写不回去的槽 5 方向）。
const DEV_INIT_STATE = ['无设备对象', '待回读', '已核对', '与记录不一致',
  '回读不成功', '本机无记录'];
// ParamSetTarget.field 的取值。⚠️ 这是 param_device_slot_t 的次序（= 上面
// DEV_SLOT_NAMES 的下标、DeviceParamEntry 的行序、mismatch_mask 的位序），
// **不是** param_field_t（那里 3 = pos_min、5 = vel_limit）。两套里 field=3 都
// 合法 → 互用不会被拒绝，会静默写错槽位。别手打数字，一律走这张表。
const PARAM_FIELD = {POS_MIN: 0, POS_MAX: 1, VEL_LIMIT: 2,
  TORQUE_LIMIT: 3, LIMIT_PROTECT: 4, DIRECTION: 5};
// SystemCommand.command
const SYS_CMD = {SAVE: 0, REFRESH: 1, SET_ZERO: 2, CLEAR_FAULT: 3, REPORT: 4};
const MOTOR_ID_NONE = 65535;
const STREAM_PERIOD_MS = 10; // 100Hz 连发
const CHART_COLORS = ['#4da3ff', '#37d67a', '#f5b94a', '#ff6b6b', '#c792ea',
  '#f78c6c', '#82aaff', '#ff9e64', '#7e8bab'];

// ---------- 工具 ----------
const $ = (id) => document.getElementById(id);
const fmt = (v, d = 2) => (v === null || v === undefined || !Number.isFinite(v))
  ? '--' : v.toFixed(d);
const CONTROLLER_PORT_STORAGE_KEY = 'robot2.controllerSerialPort';
const ageText = (ms) => {
  if (ms === null || ms === undefined) return '--';
  if (ms === 0xFFFFFFFF) return '从未';
  return ms < 1000 ? ms + ' ms' : (ms / 1000).toFixed(1) + ' s';
};
function quatToEuler(x, y, z, w) {
  const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  const sinp = 2 * (w * y - z * x);
  const pitch = sinp >= 1 ? Math.PI / 2 : (sinp <= -1 ? -Math.PI / 2 : Math.asin(sinp));
  const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return {roll, pitch, yaw};
}
const rad2deg = (r) => r * 180 / Math.PI;
const ROS_URL_STORAGE_KEY = 'robot2.rosbridgeUrl';

// ---------- 状态 ----------
// motors: id -> {topo, motion, status}
const state = {
  connected: false,
  motors: new Map(),
  lastError: 0,       // system_status.last_error 快照（总览条与失败文案翻译用）
  lastErrorMotor: -1,
  // system_status 的四个完成序号 + 保存进行标志。所有「发命令 → 等它做完」的事务
  // 都拿这里的值当基线，再轮询等它变大（见 sysSeq）。
  sysStatus: {refresh_seq: 0, set_zero_seq: 0, clear_fault_seq: 0,
              save_seq: 0, save_in_progress: false, seen: false},
  // device_param_report 的帧计数（单调即可）：「读取」(REPORT) 与写事务都靠
  // "记下基线 → 等它变大"确认帧到了，没有对应完成序号。
  devParamSeq: 0,
  devInit: new Map(), // motor_id -> {state, mask}：DEV_INIT 事件带来的核对结论
  policy: new Map(),  // motor_id -> {enabled, kp, kd}：页面**最后下发**的值
  events: [],
  imu: {count: 0, last: 0, roll: 0, pitch: 0, yaw: 0, q: null},
  temp: undefined,
  tempLast: 0,
  motionCount: 0,
  selectedId: 0,      // 左栏 / 臂上选中的电机 id（0 = 未选中）
};

// 电机 id → URDF 关节名。默认对齐默认模型 chest_upper_00（9 台电机同一条 chest
// 总线，id N ↔ joint_linkN，见该模型 config/simulation_topology.yaml）。换模型或换
// 接线用 URL 参数覆盖，页面不改：?joints=1:joint_link1,2:joint_link2&model=<模型目录名>
const URLQ = new URLSearchParams(location.search);
const JOINT_OF_MOTOR = (() => {
  const q = URLQ.get('joints');
  if (!q) {
    return {
      1: 'joint_link1', 2: 'joint_link2', 3: 'joint_link3', 4: 'joint_link4',
      5: 'joint_link5', 6: 'joint_link6', 7: 'joint_link7', 8: 'joint_link8',
      9: 'joint_link9',
    };
  }
  const m = {};
  for (const pair of q.split(',')) {
    const [id, j] = pair.split(':');
    if (id && j) m[parseInt(id, 10)] = j;
  }
  return m;
})();
// 与新版 store.js 及 gazebo.launch.py 的默认值一致（最新模型 chest_upper_00）：
// _001 已退役（c549d29 删了它的 URDF），旧默认值会让臂视图默认 404。
const MODEL = URLQ.get('model') || 'chest_upper_00';

// 曲线缓冲：motor_id -> {t:[], position:[], velocity:[], effort:[], current:[]}
const chartBuf = new Map();
const chart = {running: true, windowMs: 10000};

// 命令历史：{t, seq, targets:[...]}
const history = [];

// ---------- rosbridge ----------
let ros = null;
let subs = [];
const publishers = {};

function setConn(ok, text) {
  state.connected = ok;
  $('connDot').classList.toggle('on', ok);
  $('connText').textContent = text || (ok ? '已连接' : '未连接');
}

function setAgentStatus(status) {
  const el = $('agentText');
  el.classList.remove('running', 'failed');
  if (status.state === 'running') {
    el.textContent = `Agent 进程运行中${status.device ? ` · ${status.device}` : ''}`;
    el.classList.add('running');
    el.title = '由 web_host 管理的 micro-ROS Agent 进程';
    $('btnDisconnect').disabled = false;
  } else if (status.state === 'failed') {
    el.textContent = `Agent 已退出${status.returncode === null ? '' :
      ` · code ${status.returncode}`}`;
    el.classList.add('failed');
    el.title = status.error || status.device || 'Agent 进程已退出';
    $('btnDisconnect').disabled = true;
  } else if (status.state === 'error') {
    el.textContent = status.canStop
      ? `Agent 已运行${status.device ? ` · ${status.device}` : ''}`
      : (status.error ? `Agent 启动失败 · ${status.error}` : 'Agent 启动失败');
    el.classList.add('failed');
    el.title = status.error || '启动 Agent 失败';
    $('btnDisconnect').disabled = !status.canStop;
  } else {
    el.textContent = 'Agent 未启动';
    el.title = '';
    $('btnDisconnect').disabled = true;
  }
}

async function agentApi(path, options = {}) {
  const response = await fetch(path, {cache: 'no-store', ...options});
  let body;
  try { body = await response.json(); }
  catch (_e) { throw new Error(`Agent API 返回无效响应（HTTP ${response.status}）`); }
  if (!response.ok) {
    const error = new Error(body.error || `Agent API 失败（HTTP ${response.status}）`);
    error.status = response.status;
    error.body = body;
    throw error;
  }
  return body;
}

async function refreshAgentControls() {
  const select = $('serialPort');
  const savedPort = localStorage.getItem(CONTROLLER_PORT_STORAGE_KEY) || '';
  select.disabled = true;
  try {
    const [portResult, status] = await Promise.all([
      agentApi('/api/serial-ports'), agentApi('/api/agent/status'),
    ]);
    setAgentStatus(status);
    const ports = Array.isArray(portResult.ports) ? portResult.ports : [];
    const activeDevice = status.state === 'running' ? status.device : '';
    const known = new Set(ports.map((port) => port.path));
    if (activeDevice && !known.has(activeDevice)) {
      ports.push({path: activeDevice,
        label: `${activeDevice}（Agent 正在使用，设备当前未枚举）`, accessible: true});
    }

    select.replaceChildren();
    const placeholder = document.createElement('option');
    placeholder.value = '';
    placeholder.textContent = ports.length ? '选择控制板串口' : '未检测到串口';
    select.append(placeholder);
    for (const port of ports) {
      const option = document.createElement('option');
      option.value = port.path;
      option.textContent = port.label + (port.accessible === false ? '（无读写权限）' : '');
      option.disabled = port.accessible === false && port.path !== activeDevice;
      select.append(option);
    }

    const selected = activeDevice || savedPort;
    if (selected && known.has(selected)) select.value = selected;
    else if (activeDevice) select.value = activeDevice;
    if (select.value) localStorage.setItem(CONTROLLER_PORT_STORAGE_KEY, select.value);
  } catch (error) {
    select.replaceChildren();
    const option = document.createElement('option');
    option.value = '';
    option.textContent = '无法读取主机串口列表';
    select.append(option);
    setAgentStatus({state: 'error', error: error.message, returncode: null});
  } finally {
    select.disabled = false;
  }
}

async function refreshAgentStatus() {
  try { setAgentStatus(await agentApi('/api/agent/status')); }
  catch (_error) { /* 串口列表刷新会显示后端 API 错误；状态轮询不刷屏。 */ }
}

async function startAgent() {
  const device = $('serialPort').value;
  if (!device) {
    setAgentStatus({state: 'error', error: '请先选择控制板串口', returncode: null});
    return;
  }
  const button = $('btnConnect');
  button.disabled = true;
  $('agentText').textContent = '正在启动 Agent…';
  $('agentText').classList.remove('running', 'failed');
  try {
    const status = await agentApi('/api/agent/start', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({device}),
    });
    localStorage.setItem(CONTROLLER_PORT_STORAGE_KEY, device);
    setAgentStatus(status);
    // Process creation is immediate; give a fast startup failure a chance to surface.
    setTimeout(() => refreshAgentControls(), 700);
  } catch (error) {
    setAgentStatus({state: 'error', error: error.message,
      canStop: error.body?.status?.state === 'running',
      device: error.body?.status?.device || '', returncode: null});
  } finally {
    button.disabled = false;
  }
}

async function stopAgent() {
  const button = $('btnDisconnect');
  button.disabled = true;
  $('agentText').textContent = '正在断开 Agent…';
  $('agentText').classList.remove('running', 'failed');
  try {
    setAgentStatus(await agentApi('/api/agent/stop', {method: 'POST'}));
  } catch (error) {
    setAgentStatus({state: 'error', error: error.message,
      canStop: true, returncode: null});
  }
}

function pub(key, message) {
  if (!state.connected) return false;
  ensurePublisher(key).publish(new ROSLIB.Message(message));
  return true;
}

// rosbridge 的 advertise 是异步 WebSocket 操作。若在第一次 advertise 后
// 立刻 publish，低频的一次性 command（尤其是示教前的授权快照）可能在
// micro-ROS 订阅端完成发现前丢掉；随后 trajectory_state 仍会回执，造成
// “轨迹已完成但没有运动帧”的假成功。连接建立时先把三个下行发布端注册好，
// 后续按钮/示教只发送已经存在的 publisher。
function ensurePublisher(key) {
  if (!publishers[key]) {
    const t = TOPICS[key];
    publishers[key] = new ROSLIB.Topic({ros, name: t.name, messageType: t.type});
    publishers[key].advertise();
  }
  return publishers[key];
}

// 自动重连：roslibjs 自身**不重连**。rosbridge 一重启（改 msg、改 launch 都会重启），
// 页面就静默变哑——表还在（ID 来自参数缓存），但型号/生命周期/位置全空，看起来像
// "固件没数据"。第一次踩这个坑花了不少时间，所以默认按 2s 退避自动重连；
let reconnectTimer = null;

function connect() {
  if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null; }
  closeSocket();
  const url = $('rosUrl').value.trim();
  try {
    ros = new ROSLIB.Ros({url});
  } catch (e) {
    setConn(false, 'URL 无效');
    return;
  }
  ros.on('connection', () => {
    setConn(true);
    subscribeAll();
    ['command', 'trajectory', 'policy', 'paramSet', 'sysCmd'].forEach(ensurePublisher);
    // 参数是默认页签：连上后自动读取，避免打开看到空表。
    // 先等首帧 system_status 确认固件侧已在总线上（订阅刚建时 DDS 发现可能
    // 还没完成，REPORT 太早发出可能白等）；真没固件时由 loadParams 的超时兜底。
    waitSysStatus().then(() => loadParams());
  });
  ros.on('error', () => setConn(false, '连接错误'));
  ros.on('close', () => {
    if (!ros) return; // 幂等：closeSocket 里已经收过尾，重复触发不再排重连
    closeSocket();
    // 掉线即清场：陈旧的表比空表更容易误导（曾把"页面掉线"看成"固件没数据"）。
    state.motors.clear();
    paramCache.clear();
    // 序号是固件侧单调递增的，重连后第一帧就比掉线前的基线大——不清 seen 的话
    // 一个还在飞的轮询会拿旧基线立刻"成功"。次帧到达前拒绝轮询。
    state.sysStatus.seen = false;
    renderMotors();
    renderParamRows();
    // !reconnectTimer：同一时刻只允许一个待触发的重连（否则会叠出重连风暴，
    // 表现为页面一直连不上、看着"卡"）。
    if (!reconnectTimer) {
      setConn(false, '已断开，2s 后重连…');
      reconnectTimer = setTimeout(() => { reconnectTimer = null; connect(); }, 2000);
    }
  });
}

// 只关 socket，不动连接意图（自动重连要复用）。
function closeSocket() {
  stopStream();
  if (ros) { const r = ros; ros = null; try { r.close(); } catch (e) {} }
  subs = [];
  Object.values(publishers).forEach((p) => { try { p.unadvertise(); } catch (e) {} });
  Object.keys(publishers).forEach((k) => delete publishers[k]);
}

function sub(key, cb) {
  const t = TOPICS[key];
  const topic = new ROSLIB.Topic({
    ros, name: t.name, messageType: t.type, throttle_rate: 0, queue_length: 1,
  });
  topic.subscribe(cb);
  subs.push(topic);
}

function subscribeAll() {
  subs.forEach((s) => s.unsubscribe());
  subs = [];

  sub('topology', (m) => {
    const seen = new Set();
    for (const e of (m.entries || [])) {
      seen.add(e.motor_id);
      const row = state.motors.get(e.motor_id) || {};
      row.topo = e;
      state.motors.set(e.motor_id, row);
    }
    // 拓扑消失的电机移出表（复位/换拓扑）。
    for (const id of [...state.motors.keys()]) {
      if (!seen.has(id)) state.motors.delete(id);
    }
  });

  sub('motion', (m) => {
    state.motionCount++;
    const t = Date.now();
    for (const s of (m.states || [])) {
      const row = state.motors.get(s.motor_id) || {};
      row.motion = s;
      state.motors.set(s.motor_id, row);
      chartPush(s.motor_id, 'position', s.position, t);
      chartPush(s.motor_id, 'velocity', s.velocity, t);
      chartPush(s.motor_id, 'effort', s.effort, t);
    }
    // 臂的关节角就来自这里（理由见 arm3dSyncJoints）。
    arm3dSyncJoints();
    arm3dSyncGhost();
  });

  sub('status', (m) => {
    const t = Date.now();
    for (const s of (m.statuses || [])) {
      const row = state.motors.get(s.motor_id) || {};
      row.status = s;
      state.motors.set(s.motor_id, row);
      chartPush(s.motor_id, 'current', s.current, t);
    }
  });

  sub('trajectoryState', (m) => {
    // 轨迹状态是 10Hz best-effort 回执。回执里的 trajectory_id 就是**这一段**
    // 的 id：长示教被切成多条独立轨迹串行下发，页面等这一段的 completed 才发
    // 下一段（一次只让一条轨迹消息在飞，见 TRAJ_MAX_POINTS 上方的分片窗口）。
    if (!teach.playing || teach.mode !== 'trajectory' || !teach.waitingState) return;
    if (Number(m.trajectory_id) !== teach.activeTrajectoryId || !m.completed) return;
    teachOnTrajectoryCompleted(m);
  });

  sub('events', (m) => {
    state.events.unshift({
      t: Date.now(),
      name: EVENT_NAMES[m.event_type] !== undefined
        ? EVENT_NAMES[m.event_type] : 'EVENT_' + m.event_type,
      id: m.motor_id,
      reason: m.reason,
      request_id: m.request_id || 0,
    });
    if (state.events.length > 2000) state.events.pop();
    // DEV_INIT(type 15)：记下核对结论与"哪几项对不上"（request_id = 槽位位图）。
    if (m.event_type === 15) {
      state.devInit.set(m.motor_id, {
        state: m.reason,
        mask: m.request_id || 0,
      });
      renderParamSummary();
    }
    renderEvents();
  });

  sub('imu', (m) => {
    const q = m.orientation;
    const e = quatToEuler(q.x, q.y, q.z, q.w);
    state.imu.roll = e.roll;
    state.imu.pitch = e.pitch;
    state.imu.yaw = e.yaw;
    state.imu.q = q;
    state.imu.count++;
    state.imu.last = Date.now();
  });

  sub('temp', (m) => { state.temp = m.data; state.tempLast = Date.now(); });

  // 设备参数报告（reliable，按需：REPORT 命令 / 写事务收尾 / 设备初始化结论）：
  // 一帧带全部台的**双行**——rec_* 固件记录（权威）+ dev_* 驱动板回读。
  // 表里显示 dev_*（"优先显示驱动板"），该台没回读到（NaN，含电机不在线）时
  // 才退回 rec_*（即板内参数表）。写事务收尾后固件即时发一帧，所以
  // 「写 → 等报告变」就是这一侧的完成判据。
  sub('devParams', (m) => {
    state.devParamSeq++;
    // count 是 entries.size 的冗余声明（同 MotorPolicyArray.count）；不认它
    // 会在固件侧 count 与数组长度不一致时读到 0 条。
    const n = Math.min(m.count === undefined ? 0 : m.count, (m.entries || []).length);
    for (let i = 0; i < n; ++i) {
      const e = m.entries[i];
      // 保护开关要**反转**：报文字段 rec_safety/dev_safety 是设备原值
      // （0x2650:00 读回字节，**0 = 保护开**），而参数面 m<id>.limit_protect 与
      // 本页这一格 checkbox 用的都是正向语义（1 = 保护生效）——两者相反。
      // 契约见 DeviceParamEntry.msg；固件侧是原值透传（cerb_adapter.c 的
      // CERB_VALUE_SAFETY_RAW），不改。两行一起转，rec/dev 仍彼此可比
      // （rowsHint 拿它们对账），也仍与写回去的 field 同口径。
      // 未回读到该对象时保持 NaN——不能落 0，那会被读成"保护关"（危险方向）。
      const safety = (v) => (Number.isFinite(v) ? (v === 0 ? 1 : 0) : NaN);
      const rec = {pos_min: e.rec_pos_min, pos_max: e.rec_pos_max,
        vel_limit: e.rec_vel_limit, torque_limit: e.rec_torque_limit,
        limit_protect: safety(e.rec_safety), direction: e.rec_direction};
      const dev = {pos_min: e.dev_pos_min, pos_max: e.dev_pos_max,
        vel_limit: e.dev_vel_limit, torque_limit: e.dev_torque_limit,
        limit_protect: safety(e.dev_safety), direction: e.dev_direction};
      const pick = {};
      for (const f of Object.keys(rec)) {
        pick[f] = Number.isFinite(dev[f]) ? dev[f] : rec[f];
      }
      paramCache.set(e.motor_id, {rec, dev, pick,
        state: e.dev_init_state, dirty: e.record_dirty, mask: e.mismatch_mask});
    }
    // 有未下发的手改就跳过重绘——每帧重建 innerHTML 会把操作员正在敲的值抹掉。
    // 两张表分别判：策略面板的勾选/增益不在 #paramRows 里，只判 dirty 会漏掉它。
    if (!hasDirtyParamInput()) renderDevRows();
    if (!hasPendingPolicyEdit()) renderPolicyRows();
  });

  // 系统状态（best-effort，5 Hz + 变化即发）。这是**电平**不是事件：last_error
  // 保持到下次错误，四个 *_seq 是单调递增的完成计数（判完成 = 记下"发命令时读到
  // 的值"再等它变大）。
  sub('sysStatus', (m) => {
    state.lastError = m.last_error;
    state.lastErrorMotor = m.last_error_motor === MOTOR_ID_NONE ? -1 : m.last_error_motor;
    state.sysStatus = {
      refresh_seq: m.refresh_seq, set_zero_seq: m.set_zero_seq,
      clear_fault_seq: m.clear_fault_seq, save_seq: m.save_seq,
      save_in_progress: !!m.save_in_progress, seen: true,
    };
  });
}

// 曲线数据入缓冲；每通道独立时间轴（缺失 NaN 保留，绘制时跳过）。
function chartPush(id, ch, val, t) {
  let b = chartBuf.get(id);
  if (!b) {
    b = {t: [], position: [], velocity: [], effort: [], current: []};
    chartBuf.set(id, b);
  }
  b[ch].push(Number.isFinite(val) ? val : NaN);
  b.t.push(t);
}

// ---------- 渲染 ----------
function lifecyclePill(lc) {
  if (lc === undefined || lc === null) return '--';
  const name = LIFECYCLE[lc] !== undefined ? LIFECYCLE[lc] : 'S' + lc;
  const cls = lc === 4 ? 'on' : (lc === 5 ? 'warn' : (lc === 6 || lc === 7 || lc === 8 ? 'off' : 'mid'));
  return `<span class="pill ${cls}">${name}</span>`;
}

function busLabel(index) {
  return Number.isInteger(index) && index >= 0 && index < 3
    ? `FDCAN${index + 1}` : 'CAN --';
}

function renderMotors() {
  // 参数页总览条依赖电机状态流（在线/生命周期），随之刷新；表未加载时是空操作。
  if (paramCache.size > 0) renderParamSummary();
  const list = $('motorRows');
  const rows = [...state.motors.entries()]
    .sort((a, b) => a[0] - b[0])
    .map(([id, r]) => {
      const t = r.topo || {}, st = r.status || {};
      const online = st.online !== undefined ? st.online : t.online;
      const lc = st.lifecycle !== undefined ? st.lifecycle : t.lifecycle;
      const fault = st.unified_fault_code || 0;
      const dot = !online ? 'var(--muted)'
        : ((fault || lc === 6 || lc === 7) ? 'var(--err)'
          : (lc === 4 ? 'var(--ok)' : 'var(--warn)'));
      const stateText = fault ? (FAULT_NAMES[fault] || fault)
        : (LIFECYCLE[lc] !== undefined ? LIFECYCLE[lc] : '--');
      const mo = r.motion || {};
      // 优先取状态流字段；字段缺失时回退到低频拓扑消息。
      const busIndex = Number.isInteger(st.bus_index) ? st.bus_index : t.bus_index;
      return `<li class="${id === state.selectedId ? 'on' : ''}" data-id="${id}"
          title="${devDetailTitle(id)}">
        <span class="jdot" style="background:${dot}"></span>
        <span><span class="jid">m${id}</span><span class="jbus">${busLabel(busIndex)}</span><span class="jmodel">${t.model_name || '未绑定'}</span></span>
        <span class="jpos">${fmt(mo.position, 3)}</span>
        <span class="jstate ${fault ? 'warn' : 'muted'}">${stateText}</span>
      </li>`;
    });
  list.innerHTML = rows.length
    ? rows.join('')
    : '<li class="empty">暂无数据（等待 motor_topology）</li>';

  // 汇总 chips。
  let onlineN = 0, faultN = 0, activeN = 0, conflictN = 0;
  for (const r of state.motors.values()) {
    const st = r.status || {}, t = r.topo || {};
    const online = st.online !== undefined ? st.online : t.online;
    if (online) onlineN++;
    if (st.unified_fault_code) faultN++;
    if (st.lifecycle === 4) activeN++;                       // ACTIVE = 固件此刻真在运动
    if (t.conflict || st.lifecycle === 7) conflictN++;
  }
  $('chipOnline').textContent = `在线 ${onlineN}/${state.motors.size}`;
  $('chipActive').textContent = `运行 ${activeN}`;
  $('chipActive').classList.toggle('ok', activeN > 0);
  $('chipConflict').textContent = `冲突 ${conflictN}`;
  $('chipConflict').classList.toggle('warn', conflictN > 0);
  $('chipFault').textContent = `故障 ${faultN}`;
  $('chipFault').classList.toggle('err', faultN > 0);
  renderSelected();
  arm3dSyncColors();
}

// ---------- 面板折叠 ----------
// 折叠状态记进 localStorage：收起是操作员的意图，刷新不该被重置。收起后标题栏
// 由 renderFoldSummaries() 补摘要，所以"收起"不等于变成一根没信息的横条。
const FOLD_KEY = 'web_host.fold';
let foldState = {};

function initFold() {
  try { foldState = JSON.parse(localStorage.getItem(FOLD_KEY) || '{}') || {}; }
  catch (e) { foldState = {}; }
  document.querySelectorAll('details[data-fold]').forEach((d) => {
    const k = d.dataset.fold;
    if (foldState[k] !== undefined) d.open = foldState[k];
    d.addEventListener('toggle', () => {
      foldState[k] = d.open;
      try { localStorage.setItem(FOLD_KEY, JSON.stringify(foldState)); } catch (e) {}
    });
  });
}

function renderFoldSummaries() {
  let online = 0, conflict = 0, fault = 0;
  for (const r of state.motors.values()) {
    const st = r.status || {}, t = r.topo || {};
    if (st.online !== undefined ? st.online : t.online) online++;
    if (t.conflict || st.lifecycle === 7) conflict++;
    if (st.unified_fault_code) fault++;
  }
  $('sumJoints').textContent = state.motors.size
    ? `${online}/${state.motors.size} 在线`
      + (conflict ? ` · 冲突 ${conflict}` : '') + (fault ? ` · 故障 ${fault}` : '')
    : '--';
  $('sumDev').textContent = paramCache.size
    ? `${paramCache.size} 台已读` + (conflict ? ` · 未初始化 ${conflict}` : '')
    : '未读取';
  $('sumPol').textContent = (state.selectedId ? `m${state.selectedId}` : '全部')
    + (state.policy.size ? ` · 已下发 ${state.policy.size} 台` : '');
  const nCmd = $('cmdRows').querySelectorAll('tr').length;
  $('sumSend').textContent = `${nCmd} 台目标` + (streamTimer ? ' · 连发中' : '');
  $('sumEvents').textContent = state.events.length
    ? `${state.events.length} 条` : '空';
  $('sumChart').textContent = (chart.running ? '实时' : '暂停') + ' · ' + $('chartCh').value;
  $('sumHist').textContent = history.length ? `${history.length} 条` : '空';
}

// ---------- 臂视图 ----------
// 关节角来源是**固件的 motion_states**，不是 /joint_states。理由：页面本来就在
// 订阅它，字段就是关节侧 rad；而 /joint_states 要额外起 firmware_topic_bridge，
// 且在 Gazebo 台架上会被 gz 自己的 joint_state_broadcaster 抢同名话题。
function arm3dSyncJoints() {
  if (!window.Arm3D) return;
  const angles = {};
  for (const [idStr, joint] of Object.entries(JOINT_OF_MOTOR)) {
    const mo = (state.motors.get(Number(idStr)) || {}).motion;
    if (mo && Number.isFinite(mo.position)) angles[joint] = mo.position;
  }
  window.Arm3D.setJoints(angles);
}

// 幽灵 / 跟随误差用的「目标」= 页面最后一次真正发出去的目标位置（history[0]）。
// 这不是固件的设定点——固件会钳位、软停、整包拒收，那些结论页面看不到，
// 所以图例与提示里都写明了出处，不冒充固件真值。
function targetOf(id) {
  const h = history[0];
  if (!h) return NaN;
  const t = h.targets.find((x) => x.motor_id === id);
  return t ? t.position : NaN;
}

function arm3dSyncGhost() {
  if (!window.Arm3D) return;
  const angles = {};
  let any = false;
  for (const [idStr, joint] of Object.entries(JOINT_OF_MOTOR)) {
    const id = Number(idStr);
    const mo = (state.motors.get(id) || {}).motion;
    // 轨迹在播期间目标是固件插值出来的，页面无从得知（固件不发设定点）——
    // 宁可不画，也不画一个错的目标。target_source: 0=无 1=快照 2=轨迹
    if (mo && mo.target_source === 2) continue;
    const v = targetOf(id);
    if (Number.isFinite(v)) { angles[joint] = v; any = true; }
  }
  window.Arm3D.setTargets(any ? angles : null);
}

function arm3dSyncColors() {
  if (!window.Arm3D) return;
  const colors = {};
  for (const [idStr, joint] of Object.entries(JOINT_OF_MOTOR)) {
    const r = state.motors.get(Number(idStr)) || {};
    const st = r.status || {}, t = r.topo || {};
    const online = st.online !== undefined ? st.online : t.online;
    const lc = st.lifecycle !== undefined ? st.lifecycle : t.lifecycle;
    const fault = st.unified_fault_code || 0;
    colors[joint] = !online ? '#7e8bab'
      : ((fault || lc === 6 || lc === 7) ? '#ff6b6b'
        : (lc === 4 ? '#37d67a' : '#f5b94a'));
  }
  window.Arm3D.setJointColors(colors);
}

function selectMotor(id) {
  state.selectedId = id || 0;
  renderMotors();   // 内含 renderSelected + 颜色同步
  if (paramCache.size > 0) renderParamRows();
}

// 右栏「选中关节」：纯显示，不参与任何下发。
function renderSelected() {
  const id = state.selectedId;
  if (!id) {
    $('selTitle').textContent = '未选中关节（点左栏或臂上的小球）';
    for (const k of ['selPosition', 'selVelocity', 'selEffort', 'selCurrent',
                     'selTemp', 'selErr', 'selSource', 'selFault', 'selAge']) {
      $(k).textContent = '--';
    }
    if (window.Arm3D) window.Arm3D.setSelected(null);
    return;
  }
  const r = state.motors.get(id) || {};
  const t = r.topo || {}, mo = r.motion || {}, st = r.status || {};
  const lc = st.lifecycle !== undefined ? st.lifecycle : t.lifecycle;
  const busIndex = Number.isInteger(st.bus_index) ? st.bus_index : t.bus_index;
  $('selTitle').textContent =
    `m${id} · ${busLabel(busIndex)} · ${t.model_name || '未绑定'} · ${LIFECYCLE[lc] || '--'}`;
  $('selPosition').textContent = fmt(mo.position, 3) + ' rad';
  $('selVelocity').textContent = fmt(mo.velocity, 3) + ' rad/s';
  $('selEffort').textContent = fmt(mo.effort, 3) + ' N·m';
  $('selCurrent').textContent = fmt(st.current, 2) + ' A';
  $('selTemp').textContent = fmt(st.temperature, 1) + ' °C';
  const tgt = targetOf(id);
  $('selErr').textContent =
    (mo.target_source === 2 || !Number.isFinite(tgt) || !Number.isFinite(mo.position))
      ? '--' : fmt(tgt - mo.position, 4) + ' rad';
  $('selSource').textContent = ['无', '快照', '轨迹'][mo.target_source] || '--';
  const fault = st.unified_fault_code || 0;
  $('selFault').textContent = fault ? (FAULT_NAMES[fault] || fault) : '无';
  $('selAge').textContent = ageText(mo.state_age_ms);
  if (window.Arm3D) window.Arm3D.setSelected(JOINT_OF_MOTOR[id] || null);
}

function arm3dStart() {
  const canvas = $('arm3d');
  if (!canvas || !window.Arm3D) return;
  const hud = $('armHud');
  window.Arm3D.init(canvas, {
    urdfUrl: `/robot_arm_gazebo/models/${MODEL}/urdf/${MODEL}.urdf`,
    // package://robot_arm_gazebo/... → static_server 挂在 URL 根部的 share 目录
    // （web/robot_arm_gazebo 目录符号链接已随重构退役，见 ADR 0010）
    packages: {robot_arm_gazebo: '/robot_arm_gazebo'},
    onSelect: (joint) => {
      for (const [idStr, j] of Object.entries(JOINT_OF_MOTOR)) {
        if (j === joint) { selectMotor(Number(idStr)); return; }
      }
    },
    // LoadingManager 的回调签名是 (url, loaded, total)；不保证一定拿到 total。
    onProgress: (url, loaded, total) => {
      if (Number.isFinite(loaded) && Number.isFinite(total) && total > 0) {
        hud.textContent = `臂视图加载中 ${Math.round(loaded / total * 100)}%`;
      }
    },
    onReady: (joints) => {
      // 没进映射表的 URDF 关节要点名——否则「这根不动」会被当成故障去排查。
      const mapped = Object.values(JOINT_OF_MOTOR);
      const extra = joints.filter((j) => !mapped.includes(j));
      hud.textContent = `臂视图就绪 · ${joints.length} 个关节`
        + (extra.length ? ` · 未映射：${extra.join(' ')}` : '')
        + ' · STL 约 20 MB（首次慢，之后走浏览器缓存）';
      arm3dSyncColors();
      arm3dSyncJoints();
      arm3dSyncGhost();
      setTimeout(() => { hud.style.opacity = '0.3'; }, 5000);
    },
    onError: (err) => {
      hud.className = 'hud err';
      hud.textContent = '臂视图加载失败：' + (err && err.message ? err.message : err)
        + '（查 static_server 是否挂载 /robot_arm_gazebo、static server 日志）';
    },
  });
  window.addEventListener('resize', () => window.Arm3D.resize());
  document.addEventListener('visibilitychange', () =>
    window.Arm3D.setRunning(!document.hidden));
}

function renderEvents() {
  $('eventLog').innerHTML = state.events.slice(0, 50).map((e) =>
    `<li><span class="muted">${new Date(e.t).toLocaleTimeString()}</span> `
    + `<b>${e.name}</b> · id=${e.id}${e.reason ? ' · reason=' + e.reason : ''}`
    + (e.request_id ? ` · req#${e.request_id}` : '') + '</li>'
  ).join('') || '<li class="muted">暂无事件</li>';
}

function renderImu() {
  const d = (v) => fmt(rad2deg(v), 1) + ' °';
  const r = d(state.imu.roll), p = d(state.imu.pitch), y = d(state.imu.yaw);
  $('valRoll').textContent = r;
  $('valPitch').textContent = p;
  $('valYaw').textContent = y;
  $('valRollBig').textContent = r;
  $('valPitchBig').textContent = p;
  $('valYawBig').textContent = y;
  $('valQuat').textContent = state.imu.q
    ? `${fmt(state.imu.q.x, 3)} ${fmt(state.imu.q.y, 3)} ${fmt(state.imu.q.z, 3)} ${fmt(state.imu.q.w, 3)}`
    : '--';
  $('valTemp').textContent = state.temp !== undefined ? fmt(state.temp, 1) + ' °C' : '--';
  const ageMs = Date.now() - state.imu.last;
  $('imuAge').textContent = state.imu.last
    ? (ageMs < 3000 ? ageMs + ' ms' : '超时')
    : '--';
  drawImu3d();
  drawHorizon();
}

// ---------- 控制：命令快照 ----------
// sequence 仅作日志关联；用页面启动时间降低刷新页面后复用旧 trajectory_id
// 的概率（固件状态回执是低频 best-effort，不能接受旧回执误推进分段）。
let seq = (Date.now() & 0x7fffffff) >>> 0;
let streamTimer = null;

function addCmdRow(id = 1, pos = 0, vel = 0, eff = 0) {
  const tr = document.createElement('tr');
  tr.innerHTML =
    `<td><input type="number" class="c-id" value="${id}" min="1" max="65535" step="1"></td>`
    + `<td><input type="number" class="c-pos" value="${pos}" step="0.01"></td>`
    + `<td><input type="number" class="c-vel" value="${vel}" step="0.01"></td>`
    + `<td><input type="number" class="c-eff" value="${eff}" step="0.01"></td>`
    + `<td><button class="c-del secondary">删</button></td>`;
  tr.querySelector('.c-del').onclick = () => tr.remove();
  $('cmdRows').appendChild(tr);
}

// 读取表格 → targets；返回 {error} 或 {targets}。
function readCmdTargets() {
  const targets = [];
  const seen = new Set();
  for (const tr of $('cmdRows').querySelectorAll('tr')) {
    const id = parseInt(tr.querySelector('.c-id').value, 10);
    const pos = parseFloat(tr.querySelector('.c-pos').value);
    const vel = parseFloat(tr.querySelector('.c-vel').value);
    const eff = parseFloat(tr.querySelector('.c-eff').value);
    if (!Number.isInteger(id) || id < 1 || id > 65535) return {error: `motor_id 非法：${id}`};
    if (seen.has(id)) return {error: `motor_id 重复：${id}`};
    seen.add(id);
    if (![pos, vel, eff].every(Number.isFinite)) {
      return {error: `电机 ${id} 的数值必须为有限数（NaN/Inf 会被固件整包拒收）`};
    }
    targets.push({motor_id: id, position: pos, velocity: vel, effort: eff});
  }
  if (!targets.length) return {error: '快照为空（会导致全部受控电机软停）'};
  return {targets};
}

function setCmdResult(ok, text) {
  const el = $('cmdResult');
  el.className = 'result tiny ' + (ok ? 'ok' : 'err');
  el.textContent = text;
}

// 目标里含"未初始化（不可使能）"或离线的电机时点名提示：这些台不会动，而且
// **不会有任何报错回显**（固件只是不给它们发运动帧）——不提示的话，操作者会以为
// 是别的原因（命令没发出去 / 参数不对）。
function noteBlockedTargets(targets) {
  const blocked = [];
  for (const t of targets) {
    const row = state.motors.get(t.motor_id) || {};
    const online = row.status?.online ?? row.topo?.online;
    const lc = row.status?.lifecycle ?? row.topo?.lifecycle;
    if (online === false) blocked.push(`${t.motor_id}(离线)`);
    else if (lc === 7) blocked.push(`${t.motor_id}(未初始化)`);
    else if (lc === undefined) blocked.push(`${t.motor_id}(未上报状态)`);
  }
  return blocked;
}

// 示教不是普通的“发布成功”操作：trajectory_state 的 accepted/completed 只代表
// MCU 轨迹 mailbox 的状态，不能替代实际电机在线和生命周期检查。播放前必须确认
// 每个目标都处于 READY_DISABLED 或 ACTIVE；否则页面会出现“播放完成”但机构不动。
function playbackReadinessError(ids) {
  const blocked = [];
  for (const id of [...new Set(ids)].sort((a, b) => a - b)) {
    const row = state.motors.get(id);
    if (!row) {
      blocked.push(`m${id}(未发现)`);
      continue;
    }
    const st = row.status || {}, topo = row.topo || {};
    const online = st.online !== undefined ? st.online : topo.online;
    const lc = st.lifecycle !== undefined ? st.lifecycle : topo.lifecycle;
    const fault = Number(st.unified_fault_code || 0);
    if (online !== true) blocked.push(`m${id}(离线)`);
    else if (fault) blocked.push(`m${id}(故障${fault})`);
    else if (lc !== 3 && lc !== 4) {
      const name = LIFECYCLE[lc] !== undefined ? LIFECYCLE[lc] : '未就绪';
      blocked.push(`m${id}(${name})`);
    }
  }
  return blocked.length
    ? `电机未就绪：${blocked.join('、')}；请先启动仿真后等待四台 READY_DISABLED，再播放`
    : '';
}

function sendSnapshot() {
  if (!state.connected) { setCmdResult(false, '未连接 rosbridge'); return false; }
  const {targets, error} = readCmdTargets();
  if (error) { setCmdResult(false, error); return false; }
  const res = sendTargets(targets);
  // 连发时只在启动那一拍提示一次，避免 100Hz 刷 DOM。
  const blocked = streamTimer ? [] : noteBlockedTargets(targets);
  setCmdResult(res.ok, `已发 ${res.msg}` +
    (blocked.length ? ` ｜ ⚠ ${blocked.join('、')} 不会动（未初始化或离线）` : ''));
  return res.ok;
}

// 发送完整快照（命令表与示教共用）；记录历史。
function sendTargets(targets) {
  if (!state.connected) return {ok: false, msg: '未连接 rosbridge'};
  if (!targets.length) return {ok: false, msg: '快照为空'};
  seq = (seq + 1) >>> 0;
  pub('command', {
    // uint64：rosbridge 2.x 入站要求 JSON number（float64 会丢失末几位
    // 精度；该字段仅用于跨系统日志对齐，freshness 以 MCU 本地 tick 为准）。
    sync_time_ns: Date.now() * 1e6,
    sequence: seq,
    count: targets.length,
    targets,
  });
  history.unshift({
    t: Date.now(),
    seq,
    targets: targets.map((x) => ({...x})),
  });
  if (history.length > 500) history.pop();
  renderHistory();
  return {ok: true, msg: `seq=${seq}，${targets.length} 台`};
}

// 发布一条多点轨迹：固件在自己的 5ms/200Hz 控制边界按整条绝对时间线做
// 非均匀单调三次 Hermite 插值。轨迹声明集合内的电机由轨迹**独占**——同一时刻
// 到达的 100Hz 快照对它们既不下发也不软停，所以播放前必须停掉连发，否则
// "命令表说的"与"机器人做的"会分叉。空轨迹（无路点）= 取消：固件释放归属
// 并软停这些电机。
// 固件单条轨迹容量（CONTROL_TRAJECTORY_MAILBOX_MAX_POINTS / _MAX_MOTORS）：
// 4 台电机时 256 点只够 64 个时刻。超限时固件是**静默丢弃**——trajectory_cb 在
// 入口 return，既不发事件也不计数，所以页面必须自查，否则会报"已托管给 MCU"
// 而板子根本没收到。这是硬约束：路点数组在内核/mailbox 双缓冲/assembly/bridge
// 各存一份，加到 1024 要约 84 KB RAM，DTCM 池放不下。
const TRAJ_MAX_POINTS = 256;
const TRAJ_MAX_MOTORS = 64;
// 更长的示教由页面切成多条**独立轨迹**串行下发（见 buildTrajectorySegments）。
// 为什么不能一次塞下、也不能背靠背发多条：MCU 的 XRCE 输入流是
// MTU(512) × STREAM_HISTORY(16) = 8 KB 的**分片**窗口，一条 256 点消息约
// 5.2 KB（≈11 片）放得下，三条 128 点就吃到 ≈18 片——落在窗口外的分片被丢，
// 固件拼不齐即整组拒收（既不发事件也不回执），页面只能等到超时（2026-09-22
// 台架实测形状：≤128 点 1 段、132–252 点 2 段、**恰好 256 点才 3 段**）。
// 所以串行是唯一的办法：一次只让一条消息在飞，等 completed 回执再发下一条。
const TRAJ_STATE_TIMEOUT_PAD_MS = 3000;

// 超过单条上限不再是拒绝：播放时切段串行下发（段尾挑在运动最安静的行上，见
// pickSegmentEnd）。只有"台数"超 64 才是硬拒绝。
function trajSizeNote(rows) {
  const {points, ids} = teachRowsToTrajectory(rows);
  if (ids.length > TRAJ_MAX_MOTORS) {
    return `；⚠ ${ids.length} 台超固件上限（${TRAJ_MAX_MOTORS} 台）——必须减台`;
  }
  if (points.length <= TRAJ_MAX_POINTS) return '';
  const built = buildTrajectorySegments(rows);
  return built.error
    ? `；⚠ ${built.error}`
    : `；ℹ ${points.length} 点超单条上限（${TRAJ_MAX_POINTS}），`
      + `播放时自动切成 ${built.segments.length} 段串行下发`;
}

// 发布**一条**轨迹（= 一条消息，segment_index 0 / segment_count 1）。更长的
// 示教由 buildTrajectorySegments 切成多条这样的独立轨迹串行下发。
function sendTrajectory(points, targetIds) {
  if (!state.connected) return {ok: false, msg: '未连接 rosbridge'};
  const ids = [...new Set(targetIds ?? points.map((x) => x.motor_id))]
    .sort((a, b) => a - b);
  if (points.length > TRAJ_MAX_POINTS) {
    return {ok: false, msg: `轨迹 ${points.length} 点超过固件单条上限 ${TRAJ_MAX_POINTS}`};
  }
  if (ids.length > TRAJ_MAX_MOTORS) {
    return {ok: false, msg: `轨迹 ${ids.length} 台超过固件上限 ${TRAJ_MAX_MOTORS}`};
  }
  // 半条轨迹（有路点无归属 / 有归属无路点）固件整条拒收；只有空/空才是取消。
  if (!points.length !== !ids.length) {
    return {ok: false, msg: '轨迹路点与受控电机要么同时为空（取消），要么同时非空'};
  }
  seq = (seq + 1) >>> 0;
  const trajectoryId = seq;
  const published = pub('trajectory', {
    sync_time_ns: Date.now() * 1e6, // 仅用于跨系统日志对齐
    sequence: seq,
    trajectory_id: trajectoryId,
    segment_index: 0,
    segment_count: 1,
    count: points.length,
    target_count: ids.length,
    target_ids: ids,
    points,
  });
  if (!published) return {ok: false, msg: '未连接 rosbridge'};
  const total = points.length ? points[points.length - 1].time_from_start_ms : 0;
  return {ok: true, trajectoryId,
    msg: `轨迹 seq=${seq}：${points.length} 个路点 / ${ids.length} 台` +
      (points.length ? `，时长 ${total} ms` : '（取消归属）')};
}

// 轨迹和快照共用固件的全局 freshness/授权门。示教播放会停止 100Hz 快照流，
// 因此必须在第一段轨迹前补一份新鲜完整快照；否则页面虽然能看到
// trajectory_state=accepted/completed，固件仍会因「从未有新快照」或会话超时门
// 不生成 MIT 运动帧。轨迹受理后所属电机由轨迹独占，后续快照不会夺权。
function primeTrajectorySnapshot(segment) {
  const first = new Map();
  for (const point of segment.points) {
    if (point.time_from_start_ms !== 0 || first.has(point.motor_id)) continue;
    first.set(point.motor_id, {
      motor_id: point.motor_id,
      position: point.position,
      velocity: point.velocity,
      effort: point.effort,
    });
  }
  const targets = [...first.values()];
  return targets.length
    ? sendTargets(targets)
    : {ok: false, msg: '轨迹第一时刻没有可用于授权的目标'};
}

function cancelTrajectory() {
  return sendTrajectory([], []);
}

function renderHistory() {
  const tbody = $('histRows');
  tbody.innerHTML = history.slice(0, 20).map((h) =>
    `<tr>
      <td class="num">${new Date(h.t).toLocaleTimeString()}</td>
      <td class="num">${h.seq}</td>
      <td>${h.targets.map((x) => `${x.motor_id}:${fmt(x.position,1)},${fmt(x.velocity,1)},${fmt(x.effort,1)}`).join(' ')}</td>
      <td><span class="pill on">已发</span></td>
    </tr>`
  ).join('') || '<tr class="empty"><td colspan="4">暂无记录</td></tr>';
}

// ---------- 导出工具 ----------
function download(name, text, mime) {
  const blob = new Blob([text], {type: mime || 'application/json'});
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = name;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 3000);
}

function exportEvents(csv) {
  const rows = state.events.map((e) => csv
    ? [new Date(e.t).toISOString(), e.name, e.id, e.reason ?? '', e.request_id ?? ''].join(',')
    : JSON.stringify(e));
  download(
    'events-' + Date.now() + (csv ? '.csv' : '.json'),
    rows.join('\n'),
    csv ? 'text/csv' : 'application/json');
}

function exportHistory() {
  download('cmd-history-' + Date.now() + '.json',
    JSON.stringify(history.slice().reverse(), null, 2));
}

function startStream() {
  if (streamTimer) return;
  if (!sendSnapshot()) { $('chkStream').checked = false; return; }
  streamTimer = setInterval(sendSnapshot, STREAM_PERIOD_MS);
}
function stopStream() {
  if (streamTimer) { clearInterval(streamTimer); streamTimer = null; }
  const chk = $('chkStream');
  if (chk) chk.checked = false;
}

// ---------- 曲线图 ----------
function chartTrim() {
  const cutoff = Date.now() - 60000; // 物理裁剪窗口上限 60s
  for (const b of chartBuf.values()) {
    while (b.t.length && b.t[0] < cutoff) {
      b.t.shift();
      b.position.shift(); b.velocity.shift(); b.effort.shift(); b.current.shift();
    }
  }
}

function fillChartMotorSelect() {
  const sel = $('chartMotor');
  const cur = sel.value;
  const ids = [...state.motors.keys()].sort((a, b) => a - b);
  sel.innerHTML = '<option value="0">全部</option>'
    + ids.map((id) => `<option value="${id}">电机 ${id}</option>`).join('');
  if (cur && ids.includes(parseInt(cur, 10))) sel.value = cur;
}

function drawChart() {
  const canvas = $('chart');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  const W = canvas.width, H = canvas.height;
  const padL = 52, padR = 10, padT = 12, padB = 22;
  const plotW = W - padL - padR, plotH = H - padT - padB;
  const now = Date.now();
  const start = now - chart.windowMs;
  const ch = $('chartCh').value;
  const selId = parseInt($('chartMotor').value, 10) || 0;

  ctx.fillStyle = '#0a0f1e';
  ctx.fillRect(0, 0, W, H);
  // 网格
  ctx.strokeStyle = '#1c2540';
  ctx.lineWidth = 1;
  for (let i = 0; i <= 4; i++) {
    const y = padT + plotH * i / 4;
    ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(W - padR, y); ctx.stroke();
  }
  for (let i = 0; i <= 6; i++) {
    const x = padL + plotW * i / 6;
    ctx.beginPath(); ctx.moveTo(x, padT); ctx.lineTo(x, padT + plotH); ctx.stroke();
  }

  // 收集序列
  const series = [];
  for (const [id, b] of chartBuf) {
    if (selId && id !== selId) continue;
    if (!b[ch] || !b.t.length) continue;
    series.push({id, b});
  }
  let min = Infinity, max = -Infinity;
  for (const {b} of series) {
    for (let i = 0; i < b.t.length; i++) {
      if (b.t[i] < start) continue;
      const v = b[ch][i];
      if (Number.isFinite(v)) { if (v < min) min = v; if (v > max) max = v; }
    }
  }
  ctx.fillStyle = '#7e8bab';
  ctx.font = '12px sans-serif';
  ctx.textAlign = 'left';
  if (!series.length || !Number.isFinite(min)) {
    ctx.fillText('暂无数据（等待运动/状态流）', padL + 8, padT + 18);
    $('chartHint').textContent = '等待数据…';
    return;
  }
  if (min === max) { min -= 1; max += 1; }
  const span = max - min;
  min -= span * 0.08; max += span * 0.08;
  // y 轴标签
  ctx.textAlign = 'right';
  for (let i = 0; i <= 4; i++) {
    const v = max - (max - min) * i / 4;
    ctx.fillText(fmt(v, 2), padL - 6, padT + plotH * i / 4 + 4);
  }
  // 时间标签
  ctx.textAlign = 'center';
  for (let i = 0; i <= 2; i++) {
    const t = start + chart.windowMs * i / 2;
    const x = padL + plotW * i / 2;
    ctx.fillText('-' + Math.round((now - t) / 1000) + 's', x, H - 6);
  }
  const px = (t) => padL + (t - start) / chart.windowMs * plotW;
  const py = (v) => padT + (max - v) / (max - min) * plotH;
  let idx = 0;
  for (const {id, b} of series) {
    const color = CHART_COLORS[idx++ % CHART_COLORS.length];
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.6;
    ctx.beginPath();
    let pen = false;
    for (let i = 0; i < b.t.length; i++) {
      if (b.t[i] < start) continue;
      const v = b[ch][i];
      if (!Number.isFinite(v)) { pen = false; continue; }
      const x = px(b.t[i]), y = py(v);
      if (pen) ctx.lineTo(x, y); else { ctx.moveTo(x, y); pen = true; }
    }
    ctx.stroke();
    ctx.fillStyle = color;
    ctx.textAlign = 'left';
    ctx.fillText('#' + id, padL + 8, padT + 12 + 14 * idx);
  }
  ctx.textAlign = 'left';
  ctx.fillStyle = '#7e8bab';
  ctx.font = '11px sans-serif';
  const legY = padT + 12 + 14 * idx;
  ctx.fillText($('chartCh').selectedOptions[0].text
    + ' · ' + (chart.windowMs / 1000) + 's 窗口' + (chart.running ? '' : '（已暂停）'),
    padL, idx ? legY + 2 : padT + 18);
  $('chartHint').textContent = series.length
    ? `绘制 ${series.length} 条 · 范围 [${fmt(min, 2)}, ${fmt(max, 2)}]`
    : '无数据';
}

// ---------- IMU 3D 姿态 ----------
function drawImu3d() {
  const canvas = $('imu3d');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  const W = canvas.width, H = canvas.height;
  const cx = W / 2, cy = H / 2;
  ctx.fillStyle = '#0a0f1e';
  ctx.fillRect(0, 0, W, H);
  ctx.textAlign = 'center';
  if (!state.imu.q) {
    ctx.fillStyle = '#7e8bab';
    ctx.font = '11px sans-serif';
    ctx.fillText('等待 IMU…', cx, cy);
    return;
  }
  const {x, y, z, w} = state.imu.q;
  // body→world 旋转矩阵（四元数）
  const R = [
    [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
  ];
  const r = 42;
  const verts = [];
  for (const sx of [-1, 1]) for (const sy of [-1, 1]) for (const sz of [-1, 1]) {
    verts.push([sx * r, sy * r, sz * r]);
  }
  const proj = (v) => {
    const xw = R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2];
    const yw = R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2];
    const zw = R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2];
    return [cx + xw, cy - yw, zw];
  };
  const p = verts.map(proj);
  const edges = [[0, 1], [0, 2], [0, 4], [1, 3], [1, 5], [2, 3], [2, 6],
    [3, 7], [4, 5], [4, 6], [5, 7], [6, 7]];
  const sorted = edges
    .map(([a, b]) => ({a, b, z: (p[a][2] + p[b][2]) / 2}))
    .sort((m, n) => m.z - n.z);
  ctx.strokeStyle = '#4da3ff';
  ctx.lineWidth = 1.5;
  for (const {a, b} of sorted) {
    ctx.beginPath(); ctx.moveTo(p[a][0], p[a][1]); ctx.lineTo(p[b][0], p[b][1]); ctx.stroke();
  }
  ctx.fillStyle = '#dce4f5';
  for (const pt of p) { ctx.beginPath(); ctx.arc(pt[0], pt[1], 1.6, 0, Math.PI * 2); ctx.fill(); }
  // 三轴：X 前(红) Y 上(绿) Z 右(蓝)
  const axes = [[[1, 0, 0], [255, 107, 107]], [[0, 1, 0], [107, 255, 122]],
    [[0, 0, 1], [107, 168, 255]]];
  const o = proj([0, 0, 0]);
  for (const [v, col] of axes) {
    const q = proj([v[0] * r * 1.25, v[1] * r * 1.25, v[2] * r * 1.25]);
    ctx.strokeStyle = `rgb(${col.join(',')})`;
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(o[0], o[1]); ctx.lineTo(q[0], q[1]); ctx.stroke();
  }
}

// 人工地平仪：按 roll 旋转、pitch 平移地平线。
function drawHorizon() {
  const canvas = $('attitudeHorizon');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  const W = canvas.width, H = canvas.height;
  const cx = W / 2, cy = H / 2;
  ctx.fillStyle = '#0a0f1e';
  ctx.fillRect(0, 0, W, H);
  if (!state.imu.q) {
    ctx.fillStyle = '#7e8bab';
    ctx.font = '11px sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('等待 IMU…', cx, cy);
    return;
  }
  const roll = state.imu.roll, pitch = state.imu.pitch;
  const pitchPx = pitch / Math.PI * H * 0.9;
  const grad = ctx.createLinearGradient(0, 0, 0, H);
  grad.addColorStop(0, '#12223f');
  grad.addColorStop(1, '#22303f');
  ctx.fillStyle = grad;
  ctx.fillRect(0, 0, W, H);
  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(roll);
  // 地平线
  ctx.strokeStyle = '#f5b94a';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(-W, -pitchPx);
  ctx.lineTo(W, -pitchPx);
  ctx.stroke();
  // 俯仰刻度线（±30°）
  ctx.strokeStyle = '#7e8bab';
  ctx.lineWidth = 1;
  for (let deg = -30; deg <= 30; deg += 10) {
    if (deg === 0) continue;
    const yy = -deg / 45 * H * 0.35 - pitchPx;
    ctx.beginPath(); ctx.moveTo(-46, yy); ctx.lineTo(46, yy); ctx.stroke();
  }
  ctx.restore();
  // 机身符号
  ctx.strokeStyle = '#ff6b6b';
  ctx.lineWidth = 2.5;
  ctx.beginPath(); ctx.moveTo(cx - 32, cy); ctx.lineTo(cx + 32, cy); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(cx, cy - 16); ctx.lineTo(cx, cy + 16); ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(cx, cy - 16); ctx.lineTo(cx - 5, cy - 8);
  ctx.moveTo(cx, cy - 16); ctx.lineTo(cx + 5, cy - 8);
  ctx.stroke();
  ctx.fillStyle = '#37d67a';
  ctx.beginPath(); ctx.arc(cx, cy, 3, 0, Math.PI * 2); ctx.fill();
}

// ---------- 示教（点位序列） ----------
const teach = {
  playing: false, timer: null, idx: 0, mode: 'idle',
  segments: [], segmentIndex: 0, activeTrajectoryId: 0,
  waitingState: false, trajectoryHeld: false,
};

function setTeach(cls, text) {
  const el = $('teachStatus');
  el.className = 'muted tiny ' + (cls === 'err' ? 'warn' : '');
  el.textContent = text;
}

function teachAddRow(wait = 1000, targetsText = '') {
  const tr = document.createElement('tr');
  tr.innerHTML =
    `<td class="num">${$('teachRows').querySelectorAll('tr').length + 1}</td>`
    + `<td><input type="number" class="t-wait" value="${wait}" min="0" step="100"></td>`
    + `<td><input type="text" class="t-targets" placeholder="1:0,0,0 2:0,0,0" value="${targetsText}"></td>`
    + `<td><button class="t-del secondary">删</button></td>`;
  tr.querySelector('.t-del').onclick = () => { tr.remove(); teachRenumber(); };
  $('teachRows').appendChild(tr);
}

function teachRenumber() {
  [...$('teachRows').querySelectorAll('tr')].forEach((tr, i) => {
    tr.querySelector('td.num').textContent = i + 1;
  });
}

function teachReadRows() {
  const rows = [];
  for (const tr of $('teachRows').querySelectorAll('tr')) {
    const wait = parseInt(tr.querySelector('.t-wait').value, 10);
    const text = tr.querySelector('.t-targets').value.trim();
    const targets = [];
    if (text) {
      for (const token of text.split(/[\s;]+/)) {
        const m = /^(\d+):([-\d.]+),([-\d.]+),([-\d.]+)$/.exec(token);
        if (!m) return {error: `目标格式非法：${token}（应为 ID:pos,vel,eff）`};
        const target = {motor_id: +m[1], position: +m[2], velocity: +m[3], effort: +m[4]};
        if (![target.motor_id, target.position, target.velocity, target.effort]
            .every(Number.isFinite)) {
          return {error: `目标数值非法：${token}`};
        }
        targets.push(target);
      }
    }
    rows.push({wait: Number.isFinite(wait) && wait >= 0 ? wait : 0, targets});
  }
  return {rows};
}

function teachStopTimer() {
  if (teach.timer) { clearTimeout(teach.timer); teach.timer = null; }
}

// 示教行 → 轨迹路点：每行的 wait 是"从上一行到本行"的时长，累加成
// time_from_start_ms。同一行的多台电机共用同一时刻（固件允许不同电机共用
// 时间戳），行内重复 ID 取后者。
function teachRowsToTrajectory(rows) {
  const points = [];
  const ids = new Set();
  let t = 0;
  for (const row of rows) {
    for (const x of row.targets) {
      const point = {time_from_start_ms: t, motor_id: x.motor_id,
        position: x.position, velocity: x.velocity, effort: x.effort};
      const dup = points.findIndex(
        (p) => p.time_from_start_ms === t && p.motor_id === x.motor_id);
      if (dup >= 0) points[dup] = point; else points.push(point);
      ids.add(x.motor_id);
    }
    t += row.wait;
  }
  return {points, ids: [...ids]};
}

// 示教行 → 一段**可独立下发**的轨迹。时间从 t=0 起算：固件把"受理时刻"当轨迹
// 起点，段内的时间必须相对本段，否则第一点之前的那段时间会被钳在首点上当停顿
// （轨迹起点是受理时刻，绝对时间没有意义）。
function segmentRowsToTrajectory(rows, targetIds) {
  const result = teachRowsToTrajectory(rows);
  const lastTime = new Map();
  for (const point of result.points) {
    const previous = lastTime.get(point.motor_id);
    if (previous !== undefined && point.time_from_start_ms <= previous) {
      return {error: `motor ${point.motor_id} 的时间未严格递增；请把等待 ms 设为正数`};
    }
    lastTime.set(point.motor_id, point.time_from_start_ms);
  }
  return {
    points: result.points,
    ids: (targetIds ?? result.ids).slice().sort((a, b) => a - b),
    totalMs: result.points.length
      ? result.points[result.points.length - 1].time_from_start_ms : 0,
  };
}

// 两行之间的总位移（只算两行都出现过的电机）：用来找"运动最安静"的段边界。
function rowsMove(a, b) {
  if (!a || !b) return 0;
  const previous = new Map(a.targets.map((t) => [t.motor_id, t.position]));
  let move = 0;
  for (const t of b.targets) {
    const p = previous.get(t.motor_id);
    if (p !== undefined) move += Math.abs(t.position - p);
  }
  return move;
}

// 挑本段的末行（含）：容量窗口（≤ TRAJ_MAX_POINTS）内选**进出位移都最小**的一行。
// 每条独立轨迹的首末端点切矢归零（固件的 F–C 规则），段边界速度必然降到 0——
// 把边界落在真停顿/换向处，顿挫就基本看不出来。
function pickSegmentEnd(rows, start, targetIds) {
  const covered = new Set();
  let hi = -1;
  for (let end = start; end < rows.length; ++end) {
    for (const t of rows[end].targets) covered.add(t.motor_id);
    const probe = segmentRowsToTrajectory(rows.slice(start, end + 1), targetIds);
    if (probe.error) return {error: probe.error};
    if (probe.points.length > TRAJ_MAX_POINTS) break;
    // 固件要求声明的每台至少有一个路点（trajectory_validate 拒"有归属无路点"），
    // 覆盖不全的行不能当段尾。
    if (covered.size === targetIds.length) hi = end;
  }
  if (hi < 0) {
    return {error: `第 ${start + 1} 行起切不出 ≤${TRAJ_MAX_POINTS} 点的段：`
      + `单行路点就超上限，或某台电机在这几行里一个路点都没有`};
  }
  // 只在窗口**尾部**挑安静行：段界必然要停（等回执的那几十毫秒也停不住），
  // 所以"少切几段"比"挑到最安静那一行"值钱。取最后 1/8 是为了留点余地让停顿
  // 落在真停顿/换向处。lo ≥ start+1 让候选至少推进一行；hi == start 时窗口为空、
  // 循环一次都不跑，直接返回 hi，由调用方的"必须推进"护栏判错（不死循环）。
  const lo = Math.max(start + 1, hi - Math.max(1, Math.floor((hi - start) / 8)));
  // 段尾只剩一两行时不为它单开一段：那一段的两次停顿比它走的位移还显眼。
  // 全部候选都被这条挡掉时退回 hi（默认值），不会切不出段。
  const tailGuard = 3;
  let best = hi;
  let bestMove = Infinity;
  for (let end = lo; end <= hi; ++end) {
    const remaining = rows.length - 1 - end;
    if (remaining > 0 && remaining < tailGuard) continue;
    const move = rowsMove(rows[end - 1], rows[end]) + rowsMove(rows[end], rows[end + 1]);
    if (move < bestMove) { bestMove = move; best = end; }
  }
  return {end: best};
}

// 整条示教 → 一串可独立播放的轨迹段：每段 ≤ TRAJ_MAX_POINTS（固件单条容量），
// 段与段之间**重复边界行**（上段的终点就是下段的起点 → 段界位置连续），一个接
// 一个等 trajectory_state.completed 回执再发下一条。
//
// 每段都声明**整条时间线**的目标集合：轨迹独占它声明的电机，某段少声明一台就
// 等于把那台交还给快照通道（而播放期间连发已经停了）→ 那台会掉下来。
function buildTrajectorySegments(rows) {
  // 空行只表达等待；轨迹必须从第一个有效目标起算，前导空行没有可执行语义。
  const source = rows.map((row) => ({
    wait: Math.max(0, Number(row.wait) || 0),
    targets: (row.targets || []).map((x) => ({...x})),
  }));
  while (source.length && !source[0].targets.length) source.shift();
  while (source.length && !source[source.length - 1].targets.length) source.pop();
  if (!source.length) return {error: '时间线没有任何目标点'};

  const complete = teachRowsToTrajectory(source);
  if (!complete.points.length) return {error: '时间线没有任何路点'};
  if (complete.ids.length > TRAJ_MAX_MOTORS) {
    return {error: `轨迹 ${complete.ids.length} 台超过固件上限 ${TRAJ_MAX_MOTORS} 台`};
  }
  const targetIds = complete.ids.slice().sort((a, b) => a - b);

  const segments = [];
  let start = 0;
  while (start < source.length) {
    const picked = pickSegmentEnd(source, start, targetIds);
    if (picked.error) return {error: picked.error};
    const segment = segmentRowsToTrajectory(
      source.slice(start, picked.end + 1), targetIds);
    if (segment.error) return {error: segment.error};
    segments.push(segment);
    if (picked.end >= source.length - 1) break; // 本段已吃到时间线末尾
    // 护栏：段尾必须比段首靠后，否则 start 不前进 → 死循环。上面 `hi < 0` 那条
    // 挡不到这里（最后一行单独成段是合法的），所以另判一次。
    if (picked.end <= start) {
      return {error: `第 ${start + 1} 行起无法继续切段：剩下的行凑不出一次推进`};
    }
    start = picked.end; // 边界行重叠：既是本段终点，又是下段起点
  }
  return {segments, totalPoints: complete.points.length, ids: targetIds};
}

function teachPlay() {
  const {rows, error} = teachReadRows();
  if (error) { setTeach('err', error); return; }
  if (!rows.length) { setTeach('err', '没有点位'); return; }
  const built = buildTrajectorySegments(rows);
  if (built.error) { setTeach('err', built.error); return; }
  if (!built.segments.length) { setTeach('err', '时间线为空'); return; }
  const readinessError = playbackReadinessError(built.ids);
  if (readinessError) { setTeach('err', `播放前检查失败：${readinessError}`); return; }
  stopStream();
  teachStopTimer();
  // 授权快照只在第一段之前补一次：轨迹受理本身清 require_new_snapshot，播出期间
  // 快照的 30/100ms 级联也不作用于轨迹所属电机；段与段之间那几十毫秒悬空由
  // "上一段 completed 仍持所有权"兜住。
  const prime = primeTrajectorySnapshot(built.segments[0]);
  if (!prime.ok) { setTeach('err', `轨迹授权快照下发失败：${prime.msg}`); return; }
  teach.playing = true;
  teach.mode = 'trajectory';
  teach.segments = built.segments;
  teach.totalPoints = built.totalPoints;
  teach.segmentIndex = 0;
  teach.waitingState = false;
  teach.trajectoryHeld = false;
  teach.idx = 0;
  teachSendTrajectorySegment();
}

function teachSendTrajectorySegment() {
  if (!teach.playing || teach.mode !== 'trajectory') return;
  const segment = teach.segments[teach.segmentIndex];
  const readinessError = playbackReadinessError(segment.ids);
  if (readinessError) {
    teach.playing = false;
    teach.mode = 'idle';
    teach.waitingState = false;
    setTeach('err', `轨迹段 ${teach.segmentIndex + 1} 下发前检查失败：${readinessError}`);
    return;
  }
  const res = sendTrajectory(segment.points, segment.ids);
  if (!res.ok) {
    teach.playing = false;
    teach.mode = 'idle';
    setTeach('err', `轨迹段 ${teach.segmentIndex + 1} 下发失败：${res.msg}`);
    return;
  }
  teach.activeTrajectoryId = res.trajectoryId;
  teach.waitingState = true;
  teach.trajectoryHeld = true;
  setTeach('ok', `▸ 轨迹段 ${teach.segmentIndex + 1}/${teach.segments.length}：` +
    `${segment.points.length} 点（共 ${teach.totalPoints} 点），等待 MCU 完成回执`);
  teachStopTimer();
  teach.timer = setTimeout(() => {
    if (!teach.playing || !teach.waitingState ||
        teach.activeTrajectoryId !== res.trajectoryId) return;
    teach.playing = false;
    teach.mode = 'idle';
    teach.waitingState = false;
    const cancel = cancelTrajectory();
    setTeach('err', `轨迹段 ${teach.segmentIndex + 1} 未收到 trajectory_state 回执` +
      (cancel.ok ? '，已发送取消' : '，取消也未发布'));
  }, Math.max(TRAJ_STATE_TIMEOUT_PAD_MS, segment.totalMs + TRAJ_STATE_TIMEOUT_PAD_MS));
}

// 一段播完 → 立刻发下一段（同一次播放里的下一段，段界位置连续、速度按固件
// 端点规则归零）。只有整条时间线走完才收尾。
function teachOnTrajectoryCompleted() {
  if (!teach.playing || teach.mode !== 'trajectory') return;
  teachStopTimer();
  teach.waitingState = false;
  // 每段下发前重查：段间悬空期间某台掉线/失能，继续发下一段等于让机构带病走。
  const readinessError = playbackReadinessError(teach.segments[teach.segmentIndex].ids);
  if (readinessError) {
    teach.playing = false;
    teach.mode = 'idle';
    const cancel = cancelTrajectory();
    teach.trajectoryHeld = false;
    setTeach('err', `收到轨迹完成回执，但执行状态异常：${readinessError}` +
      (cancel.ok ? '，已发送取消' : '，取消也未发布'));
    return;
  }
  if (teach.segmentIndex + 1 < teach.segments.length) {
    teach.segmentIndex++;
    teachSendTrajectorySegment();
    return;
  }
  if ($('chkTeachLoop').checked) {
    teach.segmentIndex = 0;
    teachSendTrajectorySegment();
    return;
  }
  teach.playing = false;
  teach.mode = 'idle';
  // 播放完成后 MCU 仍保持末点，直到用户点暂停/停止取消轨迹。
  setTeach('ok', `✓ 播放完成：${teach.totalPoints} 点 / ${teach.segments.length} 段；`
    + '末点保持，点“暂停”软停');
}

// 停止 = 发空轨迹取消：固件释放归属并软停这些电机。播完不会自动取消
// （固件保持末点，机构无抱闸，软停会掉臂），所以"停"必须显式点。
function teachPause() {
  const hadTrajectory = teach.trajectoryHeld;
  teach.playing = false;
  teach.mode = 'idle';
  teach.waitingState = false;
  teachStopTimer();
  if (!hadTrajectory) { setTeach('ok', '未在播放'); return; }
  const res = cancelTrajectory();
  teach.trajectoryHeld = false;
  setTeach(res.ok ? 'ok' : 'err',
    res.ok ? '✓ 已取消轨迹（这些电机软停并交回快照通道）' : `✗ ${res.msg}`);
}

function teachStep() {
  teach.playing = false;
  teach.mode = 'idle';
  teach.waitingState = false;
  teachStopTimer();
  const {rows, error} = teachReadRows();
  if (error) { setTeach('err', error); return; }
  if (!rows.length) { setTeach('err', '没有点位'); return; }
  teach.rows = rows;
  if (teach.idx >= rows.length) teach.idx = 0;
  const row = rows[teach.idx];
  const readinessError = playbackReadinessError(row.targets.map((x) => x.motor_id));
  if (readinessError) { setTeach('err', `单步前检查失败：${readinessError}`); return; }
  // 单步也走轨迹通道；先取消旧归属，再以 t=0 的单点轨迹保持目标。
  cancelTrajectory();
  let res = {ok: true, msg: '空点位（不下发）'};
  if (row.targets.length) {
    const points = row.targets.map((x) => ({time_from_start_ms: 0, ...x}));
    const prime = sendTargets(row.targets);
    if (!prime.ok) {
      setTeach('err', `单步授权快照下发失败：${prime.msg}`);
      teach.idx++;
      if (teach.idx >= rows.length) teach.idx = 0;
      return;
    }
    res = sendTrajectory(points, [...new Set(row.targets.map((x) => x.motor_id))]);
    teach.trajectoryHeld = res.ok;
  } else {
    teach.trajectoryHeld = false;
  }
  setTeach(res.ok ? 'ok' : 'err', `单步 ${teach.idx + 1}/${rows.length}: ${res.msg}`);
  teach.idx++;
  if (teach.idx >= rows.length) teach.idx = 0;
}

function teachRecordCmd() {
  const {targets, error} = readCmdTargets();
  if (error) { setTeach('err', error); return; }
  if (!targets.length) { setTeach('err', '命令表为空'); return; }
  teachAddRow(1000, targetsToText(targets));
  setTeach('ok', '已记录当前命令为点位');
}

function teachExport() {
  const {rows, error} = teachReadRows();
  if (error) { setTeach('err', error); return; }
  download('teach-' + Date.now() + '.json', JSON.stringify(rows, null, 2));
}

// 数字进目标文本框前一律按定点写：正则 `[-\d.]+` 不接受指数，而
// String(1e-7) 就是 "1e-7"——按默认 toString 写进去会把自己导出的文件弄成
// 导入失败。9 位小数远超设备分辨率（182 cnt/° ≈ 5.5e-5 rad）。
function fmtTeachNum(v) {
  const n = Number(v);
  if (!Number.isFinite(n)) return '0.0';
  return n.toFixed(9).replace(/0+$/, '').replace(/\.$/, '.0');
}

function targetsToText(targets) {
  return (targets || []).map((x) => `${x.motor_id}:${fmtTeachNum(x.position)},` +
    `${fmtTeachNum(x.velocity)},${fmtTeachNum(x.effort)}`).join(' ');
}

// MotorTrajectory 话题载荷 → 示教行。示教表是"整快照序列"，轨迹是"扁平
// (时刻, 电机) 路点表"，两者只差一个分组：同一时刻的各台合成一行，wait 取
// 相邻时刻之差（示教行 wait 的语义是"本行到下一行"的时长，与轨迹的相对时间
// 基准一致）。示教第一行恒在 t=0，故文件首点之前的时间偏移表达不出来，丢弃。
function rowsFromTrajectory(msg) {
  const byTime = new Map();
  for (const p of Array.isArray(msg.points) ? msg.points : []) {
    const t = Math.round(Number(p.time_from_start_ms));
    const id = Number(p.motor_id);
    if (!Number.isFinite(t) || !Number.isFinite(id)) continue;
    if (!byTime.has(t)) byTime.set(t, []);
    byTime.get(t).push({
      motor_id: id,
      position: Number(p.position) || 0,
      // 固件执行侧不读这个字段（速度由位置时间线求导），原样带过来只作留痕。
      velocity: Number(p.velocity) || 0,
      // 字段名以 MotorTrajectoryPoint.msg 为准是 effort；torque 是手写文件的
      // 常见误写，兼容读入（rosbridge 遇到未知字段会整包拒发，不会静默忽略）。
      effort: Number(p.effort ?? p.torque) || 0,
    });
  }
  // 升序：固件要求时间非递减，而 Map 保留的是文件顺序——乱序文件在这里理顺。
  const times = [...byTime.keys()].sort((a, b) => a - b);
  return times.map((t, i) => {
    const next = i + 1 < times.length
      ? times[i + 1] : t + (t - (times[i - 1] ?? t));
    return {wait: Math.max(0, next - t), targets: byTime.get(t)};
  }); // 末行的 wait 后面没有点可等，用上一段时长兜底（便于继续加行）
}

function teachImport() {
  const input = document.createElement('input');
  input.type = 'file';
  input.accept = 'application/json';
  input.onchange = () => {
    const f = input.files[0];
    if (!f) return;
    const reader = new FileReader();
    reader.onload = () => {
      try {
        const data = JSON.parse(reader.result);
        // 两种形状：本页导出的行数组，或 MotorTrajectory 话题载荷（有 points）。
        const rows = Array.isArray(data) ? data : rowsFromTrajectory(data);
        if (!Array.isArray(data) && !rows.length) {
          throw new Error('既不是行数组，也没有可用的 points');
        }
        $('teachRows').innerHTML = '';
        for (const r of rows) {
          teachAddRow(r.wait || 1000, targetsToText(r.targets));
        }
        setTeach('ok', `导入 ${rows.length} 个点位`
          + (Array.isArray(data) ? '' : '（轨迹载荷 → 按时刻合成行）')
          + trajSizeNote(rows));
      } catch (e) {
        setTeach('err', '导入失败：' + e.message);
      }
    };
    reader.readAsText(f);
  };
  input.click();
}

// ---------- 定时刷新 ----------
setInterval(renderMotors, 200);
// 「当前位置」列就地刷新（见 updateDevCurPos）：跟着状态流的拍子走，不重建表格。
setInterval(updateDevCurPos, 200);
setInterval(renderFoldSummaries, 500);
setInterval(renderImu, 200);
setInterval(chartTrim, 1000);
setInterval(drawChart, 200);
setInterval(fillChartMotorSelect, 1000);

let lastMotionCount = 0;
setInterval(() => {
  const hz = state.motionCount - lastMotionCount;
  lastMotionCount = state.motionCount;
  $('chipHz').textContent = `运动流 ${state.connected ? hz + ' Hz' : '-- Hz'}`;
}, 1000);

let lastImuCount = 0;
setInterval(() => {
  const hz = state.imu.count - lastImuCount;
  lastImuCount = state.imu.count;
  $('imuHz').textContent = state.connected ? hz + ' Hz' : '-- Hz';
}, 1000);

// ---------- Tab 切换 ----------

// ---------- 逐电机策略（/micro_ros_do/motor_policy）----------
// 使能与 MIT 增益走策略话题，**不在参数树里**：它们确实是固件 param_field_t 的
// 头三项，但没进 kParamMotorFields[] 注册表（每电机只注册 5 项限值类），而参数面
// 总容量只有 64 项——再加 3 项会把可支持台数从 ~11 压到 ~8。策略这条路还带
// 「代数严格前进」的防复权门。
// 稀疏语义：只影响下发的那些电机，其余保持原状。
// 固件**不回读** kp/kd，所以表格里显示的是页面最后一次发出去的值（state.policy）。
// 固件的代数门是**跨页面持久**的：表项进 FAULT/OFFLINE/CONFIG_CONFLICT 时把当时
// 的代数锁存下来，之后「enabled=true 且代数 <= 锁存值」一律拒绝（见
// MotorPolicyArray.msg）。所以这个计数器不能从 0 起——刷新页面/重开浏览器只是把
// 我们这一侧归零，固件那边记得的仍是上一轮的值，于是每一条「武装」都被静默拒
// （只有事件日志的 CONFIG_RESULT reason=2 能看出），而轨迹照旧受理、照旧
// completed：表现就是"播放完成但机构不动"。与 seq 同法，用启动时间把起点垫高。
// 代价：代数不再连续可读，判"是不是新意图"看的是它是否前进，而不是它等于几。
let policyGeneration = (Date.now() & 0x7fffffff) >>> 0;

function polInputs(id) {
  const q = (f) => $('polRows').querySelector(
    `[data-pol-field="${f}"][data-id="${id}"]`);
  return {enable: q('enabled'), kp: q('kp'), kd: q('kd')};
}

// 逐台下发（表格每行一个按钮）：改哪台发哪台。
// 方向切换：写 m<id>.direction 的反值。翻转只改驱动器的力矩方向，**不改位置
// 坐标系**（真机实测），所以零点与限位都不用重标；真正的风险是位置环反向成正
// 反馈——使能后关节会朝远离目标的方向冲，直到撞上设备限位为止。
async function flipDirection(id, cur) {
  const next = cur === 1 ? 0 : 1;
  const name = next === 1 ? 'CCW' : 'CW';
  if (!window.confirm(`把 m${id} 的方向切成 ${name}？\n\n` +
      '方向一翻，驱动器的力矩方向反了：位置环会变成正反馈，一旦使能关节会朝' +
      '远离目标的方向冲，直到撞上设备限位。确认要翻吗？')) {
    return;
  }
  const req = sendParamSet([{motor_id: id, field: PARAM_FIELD.DIRECTION, value: next}]);
  if (!req) return;
  setParamResult(true, `m${id} 方向已下发 ${name}（req#${req}）；` +
    '受理/拒绝是异步的——写事务通过才生效，看 CONFIG_RESULT');
  // 写事务（写 + 读回核验）收尾后固件即时发一帧 device_param_report，这里只需重绘。
  setTimeout(renderParamRows, 500);
}

function sendPolicyRow(id) {
  if (!state.connected) { setPolResult(false, '未连接 rosbridge'); return; }
  const el = polInputs(id);
  if (!el.enable || !el.kp || !el.kd) { return; }
  const kp = parseFloat(el.kp.value);
  const kd = parseFloat(el.kd.value);
  if (!Number.isFinite(kp) || !Number.isFinite(kd) || kp < 0 || kd < 0) {
    setPolResult(false, `m${id}：kp/kd 必须是非负有限值`);
    return;
  }
  // 保护被关掉的台：该关节没有任何限位在跑（CERB 是设备侧限位，固件没有软限位
  // 兜底），所以每次武装都要显式确认。固件侧有同一道门禁（缺确认位直接拒），
  // 这里只是把"为什么按不下去"提前说清楚。
  const pc = paramCache.get(id);
  const protect = Number(pc ? pc.pick.limit_protect : NaN);
  const unprotected = Number.isFinite(protect) && protect === 0;
  if (el.enable.checked && unprotected) {
    if (!window.confirm(`m${id} 的设备侧限位保护当前是关闭的：\n\n` +
        '该关节没有任何限位在执行（含看门狗自锁）。确认要武装它吗？')) {
      el.enable.checked = false;
      return;
    }
  }
  policyGeneration = (policyGeneration + 1) >>> 0;
  seq = (seq + 1) >>> 0;
  pub('policy', {
    sync_time_ns: Date.now() * 1e6,
    request_id: seq,
    policy_generation: policyGeneration,
    count: 1,
    targets: [{motor_id: id, enabled: el.enable.checked, kp, kd,
               confirm_unprotected: el.enable.checked && unprotected}],
  });
  state.policy.set(id, {enabled: el.enable.checked, kp, kd});
  setPolResult(true, `策略已发 seq=${seq} gen=${policyGeneration}：m${id} ` +
    (el.enable.checked ? '武装' : '失能') + ` kp=${kp} kd=${kd}` +
    '。受理/拒绝是异步的，看事件日志的 CONFIG_RESULT');
}

// ---------- 参数页（rcl_interfaces 参数服务直连）----------
// 参数页的两条通道：读 = device_param_report 订阅，写 = param_set / system_command
// 话题。已无参数服务——不要在这里复活 ROSLIB.Service。
// id -> {rec, dev, pick, state, dirty, mask}，见 devParams 订阅。
const paramCache = new Map();

// 写事务的关联 id。两个高位都不能碰：bit31 是内核身份读取的命名空间，bit30 是
// 固件自发脉冲的命名空间（置位会被本地拒）。0 表示「固件代分配」，所以绕开它。
let paramReq = (Date.now() & 0x3fffffff) >>> 0;
function nextParamReq() {
  paramReq = (paramReq + 1) & 0x3fffffff;
  return paramReq === 0 ? (paramReq = 1) : paramReq;
}

// 下发一条稀疏写（整条受理或整条拒绝）。返回 request_id，失败返回 0。
function sendParamSet(targets) {
  if (!state.connected) { setParamResult(false, '未连接 rosbridge'); return 0; }
  if (!targets.length) { setParamResult(false, '没有要下发的改动'); return 0; }
  // [<=16] 是内核 CONTROL_MAILBOX_CONFIG_QUEUE_CAPACITY 的映射，不是带宽限制：
  // 每个目标一次 motor_rt_submit_config 提交。调用方负责分块（见 chunkedParamSet）。
  if (targets.length > 16) { setParamResult(false, '内部错误：单条超过 16 个目标'); return 0; }
  const request_id = nextParamReq();
  pub('paramSet', {
    sync_time_ns: Date.now() * 1e6, // uint64：rosbridge 入站要 JSON number
    request_id, count: targets.length, targets,
  });
  return request_id;
}

// 按台切块：4 台 × 6 字段 = 24 > 16，必须拆。同一台**不得跨块**——位置限位成对的
// 两边落在两块里就是两条各缺一边的消息，两条都被整条拒。
function chunkedParamSet(byMotor) {
  const chunks = [];
  let cur = [];
  for (const targets of byMotor.values()) {
    if (cur.length + targets.length > 16) { chunks.push(cur); cur = []; }
    cur.push(...targets);
  }
  if (cur.length) chunks.push(cur);
  return chunks;
}

function sendSystemCommand(command, motorId) {
  if (!state.connected) { setParamResult(false, '未连接 rosbridge'); return 0; }
  const request_id = nextParamReq();
  pub('sysCmd', {
    sync_time_ns: Date.now() * 1e6, request_id, command,
    motor_id: motorId === undefined ? MOTOR_ID_NONE : motorId,
  });
  return request_id;
}

// 「发命令 → 等它做完」的唯一判据：记下发送时读到的序号，等它变大。
// 首帧未到达前拒绝（重连后旧基线会让还在飞的轮询立刻假成功，见 close 处理）。
async function waitSysSeq(key, baseline, timeoutMs) {
  const deadline = Date.now() + timeoutMs;
  while (Date.now() < deadline) {
    if (!state.sysStatus.seen) return -2;      // 收不到 system_status
    if (state.sysStatus[key] > baseline) return 0;
    if (!state.connected) return -2;
    await new Promise((r) => setTimeout(r, 150));
  }
  return -3; // 超时未受理
}

// last_error 的中文说法。码表与固件零迁移（SystemStatus.msg）。
const SYS_ERR_TEXT = {
  1: '内核故障', 2: '内核故障', 3: '内核故障', 4: '内核故障', 5: '内核故障', 6: '内核故障',
  10: '本地校验拒绝（多半是该台正在运动中，或位置限位没成对）',
  11: '提交失败（配置队列满或链路异常）',
  12: '未知电机（该台未绑定型号）',
  20: '保存失败',
  30: '回读失败',
  40: '与该台固件记录不一致', 41: '回读不成功', 42: '本机无该台记录',
};
function lastErrorText() {
  const e = state.lastError;
  if (!e) return '无错误';
  return (SYS_ERR_TEXT[e] || `错误码 ${e}`) +
    (state.lastErrorMotor >= 0 ? `（m${state.lastErrorMotor}）` : '');
}

function writeResult(id, ok, text) {
  const el = $(id);
  if (!el) return;
  el.className = 'result tiny ' + (ok ? 'ok' : 'err');
  el.textContent = text;
}
// 参数结果写在「全部参数」表里，策略结果写在「使能与增益」抽屉行里——两块面板
// 各自可收起，结果必须落在按下去的那个按钮旁边。
function setParamResult(ok, text) { writeResult('paramResult', ok, text); }
function setPolResult(ok, text) { writeResult('polResult', ok, text); }

// 两张表共用同一套行筛选（「显示全部」那个勾选框在**全部参数**卡里）：设备限值
// （可回读）与使能/增益（只写）分处两行、行序仍按同一套 id 排，勾「显示全部」
// 看整条臂。默认只展开选中的那一台。
function paramRowIds() {
  const all = [...paramCache.keys()].sort((a, b) => a - b);
  const showAll = $('chkAllParams') && $('chkAllParams').checked;
  return (showAll || !state.selectedId)
    ? all : all.filter((i) => i === state.selectedId);
}

function renderParamRows() { renderDevRows(); renderPolicyRows(); }

// ---------- 全部参数（以驱动板为准，可回读）----------
function renderDevRows() {
  const tbody = $('paramRows');
  const ids = paramRowIds();
  if (ids.length === 0) {
    tbody.innerHTML = '<tr class="empty"><td colspan="13">'
      + (paramCache.size === 0
        ? '参数集为空（等待 device_param_report）'
        : `m${state.selectedId} 暂无参数记录（未发现或未回读）——勾「显示全部」看其余`)
      + '</td></tr>';
    return;
  }
  tbody.innerHTML = ids.map((id) => {
    const v = paramCache.get(id);
    const pick = v.pick; // 逐字段：驱动板回读优先，没回读到才用固件记录
    const row = state.motors.get(id) || {};
    const model = row.topo?.model_name || '';
    const online = row.status?.online ?? row.topo?.online;
    const lc = row.status?.lifecycle ?? row.topo?.lifecycle;
    // 来源：只按 online 判会把"型号占位/未核对"也标成驱动板——生命周期
    // CONFIG_CONFLICT(7) 表示设备参数未核对（未回读完 / 与记录不一致 / 回读
    // 失败），此时这行的数字不是设备真值，必须如实标出来。
    const lcVerified = online === true && lc !== 7;
    const src = online === undefined ? '<span class="muted">--</span>'
      : (!online ? '<span class="pill off">记录</span>'
                 : (lcVerified ? '<span class="pill mid">驱动板</span>'
                               : '<span class="pill off">未核对</span>'));
    // 设备端原值提示：页面已知"参数单位 → 设备原值"的换算（设备对象就是 cnt / mA），
    // 直接写在输入框 title 里，省去对着 CAN 日志换算。位置参数本身就是 rad
    // （与状态、URDF 同一套单位），cnt 用设备侧 65536 cnt/360° 折。速度的电机端
    // RPM 还要按减速比折算，页面拿不到该系数，故不给。
    const isCerb = row.topo?.protocol === 1;
    const raw = (f, value) => {
      if (!isCerb || !Number.isFinite(value)) return '';
      if (f === 'pos_min' || f === 'pos_max') {
        return `设备端 ${Math.round(value * 65536 / (2 * Math.PI))} cnt`;
      }
      if (f === 'torque_limit') return `设备端 ${Math.round(value * 1000)} mA`;
      return '';
    };
    // 两行对照：显示的是 pick（驱动板优先），另一个值放 title。不一致时明说，
    // 免得操作员把"记录与设备对不上"看成页面读错。
    const rowsHint = (f) => (v.rec[f] === v.dev[f]) ? ''
      : `；记录 ${fmtParam(v.rec[f])} / 驱动板 ${fmtParam(v.dev[f])}`;
    const cell = (f) => {
      const hint = raw(f, Number(pick[f])) + rowsHint(f);
      return `<input data-id="${id}" data-field="${f}" class="pnum" value="${fmtParam(pick[f])}"` +
        (hint ? ` title="${hint}"` : '') + `>`;
    };
    // 限位教学：把该台**当前关节位置**写进对应的限位槽。厂商手册要求设置限位时
    // 电机失能、人掰到限位再设——内核也只受理未运动状态（device_limit_write_allowed），
    // 所以运动中的行直接禁用按钮，别让操作员白按。
    const moving = lc === 4 || lc === 5; // ACTIVE / SAFE_STOP
    const limBtn = (side, label, full) =>
      `<button class="tiny-btn" data-setlim="${id}" data-lim="${side}"` +
      (moving ? ' disabled' : '') +
      ` title="${moving ? `该台正在运动中，限位只能在失能状态设置（先点「策略」失能）`
        : `把 m${id} 的**当前位置**设为${full}。设置时电机必须失能；超出 ±180° 会被设备侧钳制。`}">` +
      `${label}</button>`;
    const limCell = limBtn('min', '设负', '位置下限 pos_min（负向 / 左）') +
      limBtn('max', '设正', '位置上限 pos_max（正向 / 右）');
    // 保护开关：参数 1 = 保护生效（设备侧原值 0）。设备还没回读到该对象时
    // 固件给 NaN —— 这时勾选框禁用，不能让操作员以为自己看到了真值。
    const protect = Number(pick['limit_protect']);
    const protectCell = Number.isFinite(protect)
      ? `<input type="checkbox" data-id="${id}" data-field="limit_protect"` +
        (protect === 1 ? ' checked' : '') +
        ' title="限位保护：勾选 = 设备侧看门狗 + 限位保护生效；取消 = 关闭' +
        '（该关节将没有任何限位在执行）。仅未运动时受理。' +
        '上电默认开（固件在回读比对之前写回）；取消后该台每次武装都要二次确认，' +
        '且下次上电会自动恢复为开。">'
      : '<input type="checkbox" disabled title="设备还没回读到该对象">';
    // 方向：原值直通（0=CW / 1=CCW），点一下切换。翻转只改力矩方向，不改位置
    // 坐标系（实测）；风险是位置环反向，故仅在未运动时可改。
    const dir = Number(pick['direction']);
    const dirCell = Number.isFinite(dir)
      ? `<button class="tiny-btn" data-dir="${id}" data-dir-cur="${dir}"` +
        ' title="点击切换 CW / CCW。翻转只改力矩方向（位置坐标不变），' +
        '但位置环会反向：使能后会朝远离目标的方向冲。仅未运动时受理。">' +
        `${dir === 1 ? 'CCW' : 'CW'}</button>`
      : '<button class="tiny-btn" disabled title="设备还没回读到该对象">--</button>';
    // 保护确知为关的行标红：该关节此刻没有任何限位在跑。
    const unprotected = Number.isFinite(protect) && protect === 0;
    // 当前位置：与限位同单位（rad），取 50 Hz 状态流。初始值渲染一次，
    // 之后由 updateDevCurPos（200 ms 拍）就地改文本——整表 innerHTML 重建
    // 会抹掉操作员正在敲的值，不能拿它来刷这个 50 Hz 量。
    const curPos = Number(row.motion?.position);
    return `<tr${unprotected ? ' class="unprotected"' : ''}` +
      `${unprotected ? ' title="保护已关：该关节无任何限位"' : ''}>
      <td class="num">${id}</td>
      <td>${model || '<span class="muted">--</span>'}</td>
      <td>${lifecyclePill(lc)}</td>
      <td>${src}</td>
      <td>${cell('pos_min')}</td><td>${cell('pos_max')}</td>
      <td class="num curpos" data-cur="${id}">${fmt(curPos, 4)}</td>
      <td>${limCell}</td>
      <td>${cell('vel_limit')}</td><td>${cell('torque_limit')}</td>
      <td>${protectCell}</td>
      <td>${dirCell}</td>
      <td><button class="tiny-btn" data-zero="${id}" title="把该台当前位置设为设备零点（仅失能态受理）">设零</button>
        <button class="tiny-btn" data-clearfault="${id}" title="清除故障锁存（仅非运动态受理；身份冲突锁存不可解除）">清错</button></td>
    </tr>`;
  }).join('');
  tbody.querySelectorAll('input').forEach((inp) => {
    inp.addEventListener('input', () => { inp.dataset.dirty = '1'; });
    inp.addEventListener('change', () => { inp.dataset.dirty = '1'; });
  });
  tbody.querySelectorAll('[data-dir]').forEach((btn) => {
    btn.onclick = () => flipDirection(parseInt(btn.dataset.dir, 10),
      parseInt(btn.dataset.dirCur, 10));
  });
  tbody.querySelectorAll('[data-zero]').forEach((btn) => {
    btn.onclick = () => setMotorZero(parseInt(btn.dataset.zero, 10));
  });
  tbody.querySelectorAll('[data-clearfault]').forEach((btn) => {
    btn.onclick = () => clearMotorFault(parseInt(btn.dataset.clearfault, 10));
  });
  tbody.querySelectorAll('[data-setlim]').forEach((btn) => {
    btn.onclick = () => setLimitFromPosition(parseInt(btn.dataset.setlim, 10),
      btn.dataset.lim);
  });
  renderParamSummary();
}

// 「当前位置」列的就地刷新：只改单元格文本，**不重建 innerHTML**——整表重建会把
// 操作员正在敲的限位值抹掉（同 hasDirtyParamInput 那条理由），所以这个 50 Hz 量
// 走独立的文本更新路径，由 200 ms 定时器驱动。
function updateDevCurPos() {
  const tb = $('paramRows');
  if (!tb) return;
  tb.querySelectorAll('td.curpos').forEach((td) => {
    const row = state.motors.get(parseInt(td.dataset.cur, 10)) || {};
    const text = fmt(Number(row.motion?.position), 4);
    if (td.textContent !== text) td.textContent = text;
  });
}

// 表里有没有还没下发的改动。device_param_report 按需发帧（REPORT 命令 +
// 写事务收尾），每帧重建 innerHTML 会把操作员正在敲的值抹掉——有 dirty 就跳过重绘。
function hasDirtyParamInput() {
  const tb = $('paramRows');
  return !!(tb && tb.querySelector('input[data-dirty]'));
}

// 策略面板里有没有「改了还没按策略」的输入。判据 = 控件当前值 ≠ state.policy 里
// **最后一次下发**的记录；没记录过按 0 / 未勾对待，与 renderPolicyRows 的呈现口径
// 一致。
// 为什么需要它：device_param_report 是按需电平帧，每帧都重建两张表的 innerHTML，
// 而 hasDirtyParamInput 只看 #paramRows（策略面板在 #polRows）——漏判的后果是
// 操作员勾上「启用」，不到一秒就被下一帧重建抹掉，看着像"勾不上/启用不了"。
function hasPendingPolicyEdit() {
  const tb = $('polRows');
  if (!tb) return false;
  for (const inp of tb.querySelectorAll('[data-pol-field]')) {
    const last = state.policy.get(parseInt(inp.dataset.id, 10)) || {};
    if (inp.dataset.polField === 'enabled') {
      if (inp.checked !== (last.enabled === true)) return true;
    } else if (parseFloat(inp.value) !== (last[inp.dataset.polField] !== undefined
        ? last[inp.dataset.polField] : 0)) {
      return true; // 清空输入框时 parseFloat 是 NaN，恒 != 记录 → 同样算未下发
    }
  }
  return false;
}

// ---------- 使能与增益（只写，固件不回读）----------
// 走策略话题、不在参数树里：enable/kp/kd 确实是固件 param_field_t 的头三项，但没进
// kParamMotorFields[] 注册表（每电机只注册 5 项限值类），而参数面总容量只有 64 项。
// 每条策略只带一台（稀疏），改哪台发哪台。
function renderPolicyRows() {
  const tbody = $('polRows');
  const ids = paramRowIds();
  if (ids.length === 0) {
    tbody.innerHTML = '<tr class="empty"><td colspan="6">连接后点「读取」载入电机列表</td></tr>';
    return;
  }
  tbody.innerHTML = ids.map((id) => {
    const row = state.motors.get(id) || {};
    const model = row.topo?.model_name || '';
    const pol = state.policy.get(id) || {};
    const polNum = (f) => `<input class="pnum" type="number" min="0" step="0.1"
      data-id="${id}" data-pol-field="${f}" value="${pol[f] !== undefined ? pol[f] : 0}"
      title="页面最后一次下发的值；固件不回读 kp/kd。设备参数未核对的台会被直接拒">`;
    const polEnable = `<input type="checkbox" data-id="${id}" data-pol-field="enabled"
      ${pol.enabled ? ' checked' : ''}
      title="页面最后一次下发的值。设备参数未核对的台会被直接拒（CONFIG_RESULT reason=2）">`;
    return `<tr>
      <td class="num">${id}</td>
      <td>${model || '<span class="muted">--</span>'}</td>
      <td>${polEnable}</td>
      <td>${polNum('kp')}</td><td>${polNum('kd')}</td>
      <td><button class="tiny-btn primary" data-pol="${id}" title="把这一行的 启用/kp/kd 作为一条策略下发（代数自动 +1）">策略</button></td>
    </tr>`;
  }).join('');
  tbody.querySelectorAll('[data-pol]').forEach((btn) => {
    btn.onclick = () => sendPolicyRow(parseInt(btn.dataset.pol, 10));
  });
}
// 位图 → 槽位名列表（槽位顺序即 DEV_SLOT_NAMES 顺序）。
function devSlotNames(mask) {
  return DEV_SLOT_NAMES.filter((_, i) => (mask & (1 << i)) !== 0);
}

// 恢复口径（MISMATCH 用）：槽 0..4 在参数面，写回对应项即可；只剩槽 5
// （0x2541 方向）没有参数项——它只能拿 cangaroo 或厂商上位机改回设备对象，
// 再重连重读。
// 恢复口径：六个槽位都在参数面里了。
function devRecoveryText(mask) {
  if ((mask & (1 << 5)) !== 0) {
    return '方向(0x2541)无基线：在参数表点一次方向（CW/CCW）即建立基线并恢复';
  }
  return '写回对应项即可恢复';
}

// 状态列悬停提示：DEV_INIT 事件带来的槽位位图。state=3(MISMATCH) 位图是"对不上"
// 的槽位，state=4(UNVERIFIED) 位图是"没读到"的槽位（见 MotorEvent.msg）。
function devDetailTitle(id) {
  const info = state.devInit.get(id);
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

// 参数值显示：内核把这些量存成 float32，拓宽成 double 后会出现
// 0.20000000298023224 这类尾巴——显示时收敛到有效位，别让人误以为是精度问题。
// 只影响显示；未编辑的字段不会下发（applyParams 只发 dirty 字段）。
function fmtParam(value) {
  if (value === undefined || value === null || value === '') return '';
  const n = Number(value);
  if (!Number.isFinite(n)) return '';
  const s = Math.abs(n) >= 1e6 || (n !== 0 && Math.abs(n) < 1e-4)
    ? n.toExponential(6)
    : String(Number(n.toPrecision(7)));
  return s;
}

// 设备参数核对总览：把"上电核对"的结论放到最显眼处。判据是内核把未完成核对的台
// 置 CONFIG_CONFLICT(7)，具体是"与记录不一致(40)"还是"回读不成功(41)"看
// system.last_error；其余在线台按已核对显示（来源=驱动板）。
function renderParamSummary() {
  const el = $('paramSummary');
  if (!el) return;
  const ids = [...paramCache.keys()].sort((a, b) => a - b);
  if (ids.length === 0) {
    el.className = 'summary tiny';
    el.textContent = '设备参数核对：暂无电机';
    return;
  }
  const offline = [], blocked = [], ok = [];
  for (const id of ids) {
    const row = state.motors.get(id) || {};
    const online = row.status?.online ?? row.topo?.online;
    const lc = row.status?.lifecycle ?? row.topo?.lifecycle;
    if (online === false) offline.push(id);
    else if (lc === 7) blocked.push(id);
    else ok.push(id);
  }
  const parts = [`<b>设备参数核对：共 ${ids.length} 台</b>`,
    `<span class="pill mid">已核对 ${ok.length}</span>`];
  if (blocked.length) {
    const code = Number(state.lastError || 0);
    const why = (code === 40) ? '与固件记录不一致'
      : (code === 41 ? '回读不成功' : (code === 42 ? '本机无该台记录' : '未完成'));
    // 逐台指出"哪几项"：位图来自 DEV_INIT 事件（收到前只显示结论）。code 40 的
    // 位图是"对不上"的槽位、41 是"没读到"的槽位——两者都只是槽位名，靠 why 区分。
    let blockedMask = 0;
    const per = blocked.map((id) => {
      const info = state.devInit.get(id);
      const mask = info ? info.mask : 0;
      blockedMask |= mask;
      if (!mask) return `${id}`;
      return `${id}(${devSlotNames(mask).join('、')})`;
    });
    const suffix = (code === 41)
      ? '括号内是未收到的对象——重连或再点该行「回读」重试'
      : devRecoveryText(blockedMask);
    parts.push(`<span class="pill off">未初始化 ${blocked.length}</span>` +
      `<span class="muted">（${per.join(' ')}：${why}——${suffix}）</span>`);
  }
  if (offline.length) {
    parts.push(`<span class="pill off">离线 ${offline.length}</span>` +
      `<span class="muted">（${offline.join('/')}：显示固件记录回落值）</span>`);
  }
  el.className = 'summary tiny';
  el.innerHTML = parts.join(' ');
}

// 等第一个 system_status。它是 5Hz 的**电平**帧，实测「订阅 → 首帧」约 100 ms
// （还是 DDS 发现已完成的**热**态；冷启动更久）。所以连上后同一 tick 就读必然撞上
// seen=false——那是"订阅刚建、第一帧还没到"，不是固件故障。
//
// 为什么是"等"而不是"跳过检查"：轮询的完成判据是 `*_seq > 发命令时的基线`，而未
// 收到任何帧时基线是初值 0，会让一个还在飞的轮询立刻判成"已完成"（见 closeSocket
// 里清 seen 的注释）。所以首帧必须真等到，不能拿 0 当基线。
function waitSysStatus(timeoutMs = 2000) {
  if (state.sysStatus.seen) return Promise.resolve(true);
  return new Promise((resolve) => {
    const deadline = Date.now() + timeoutMs;
    const timer = setInterval(() => {
      if (state.sysStatus.seen) { clearInterval(timer); resolve(true); }
      else if (Date.now() >= deadline) { clearInterval(timer); resolve(false); }
    }, 50);
  });
}

// 读取 = 发 SystemCommand(REPORT)，固件把**当前手里的参数视图**立刻发一帧
// device_param_report（不触发 SDO、不访问设备）。电机在线且此前核对过，
// dev_* 就是驱动板真值；电机不在线/没核对，dev_* 为 NaN，页面呈现 rec_*
// 即驱动板 W25 参数表。帧到达后订阅回调自行渲染，这里只等帧、给结论。
async function loadParams() {
  if (!state.connected) return;
  const before = state.devParamSeq;
  if (!sendSystemCommand(SYS_CMD.REPORT, MOTOR_ID_NONE)) return;
  setParamResult(true, '读取中…');
  const rc = await waitDevParamSeq(before, 2000);
  if (rc === -2) {
    setParamResult(false, '读取失败：未连接 rosbridge');
    return;
  }
  if (rc === -3) {
    setParamResult(false, '读取超时：未收到 device_param_report——固件/agent 未就绪？');
    return;
  }
  let withDev = 0;
  for (const v of paramCache.values()) {
    if (Number.isFinite(v.dev.pos_min)) withDev++;
  }
  setParamResult(true, `已读取参数表：共 ${paramCache.size} 台，其中 ${withDev} 台` +
    '有驱动板回读值（显示驱动板真值），其余显示板内记录');
}

// 等下一帧 device_param_report（帧计数变大）。无完成序号可用，REPORT/写事务
// 都靠它确认帧到了。
async function waitDevParamSeq(baseline, timeoutMs) {
  const deadline = Date.now() + timeoutMs;
  while (Date.now() < deadline) {
    if (state.devParamSeq > baseline) return 0;
    if (!state.connected) return -2;
    await new Promise((r) => setTimeout(r, 100));
  }
  return -3;
}

// 下发手改项。三条硬约束（固件 validate_param_set，整条受理或整条拒绝）：
//   1. count === targets.length
//   2. 位置限位必须**成对**在同一条消息里，且 min < max
//   3. ≤ 16 个 target（内核配置 FIFO 深度），故按台分块
// 违反任一条整条被拒（last_error=10）——所以能在这里判的先在这里判，
// 别让一个台的错配对把别的台的合法改动一起带走。
async function applyParams() {
  if (!state.connected) return;
  const dirty = [...$('paramRows').querySelectorAll('input[data-dirty]')];
  if (dirty.length === 0) { setParamResult(true, '没有修改'); return; }

  // 先按台归拢。任一字段值非法就整批不发（局部下发会留下"改了一半"的表）。
  const byMotor = new Map();
  const note = (id) => {
    if (!byMotor.has(id)) byMotor.set(id, new Map());
    return byMotor.get(id);
  };
  for (const inp of dirty) {
    const id = parseInt(inp.dataset.id, 10);
    const field = inp.dataset.field;
    const slot = PARAM_FIELD[field.toUpperCase()];
    if (slot === undefined) continue;
    if (inp.type !== 'checkbox' && inp.value.trim() === '') {
      setParamResult(false, `m${id}.${field} 为空——填个数或刷新页面放弃这项改动`);
      return;
    }
    const value = inp.type === 'checkbox' ? (inp.checked ? 1 : 0) : parseFloat(inp.value);
    if (!Number.isFinite(value)) {
      setParamResult(false, `m${id}.${field} 不是数字：${inp.value}`);
      return;
    }
    note(id).set(slot, value);
  }

  // 成对补齐 + 本地判 min < max。
  for (const [id, fields] of byMotor) {
    const v = paramCache.get(id);
    const hasMin = fields.has(PARAM_FIELD.POS_MIN);
    const hasMax = fields.has(PARAM_FIELD.POS_MAX);
    if (hasMin || hasMax) {
      const lo = hasMin ? fields.get(PARAM_FIELD.POS_MIN) : (v ? v.pick.pos_min : NaN);
      const hi = hasMax ? fields.get(PARAM_FIELD.POS_MAX) : (v ? v.pick.pos_max : NaN);
      if (!Number.isFinite(lo) || !Number.isFinite(hi)) {
        setParamResult(false, `m${id} 只想改一侧限位，但另一侧没有可用值——` +
          '先点「读取」把该台参数读回来');
        return;
      }
      if (!(lo < hi)) {
        setParamResult(false, `m${id} 位置下限 ${lo.toFixed(4)} 不小于上限 ${hi.toFixed(4)}：` +
          '固件要求 min < max，整条会被拒。请先改正再下发。');
        return;
      }
      // 两侧都塞进去——只发一侧会被整条拒（不做半边暂存）。
      fields.set(PARAM_FIELD.POS_MIN, lo);
      fields.set(PARAM_FIELD.POS_MAX, hi);
    }
  }

  const motorTargets = new Map();
  for (const [id, fields] of byMotor) {
    motorTargets.set(id, [...fields].map(([field, value]) => ({motor_id: id, field, value})));
  }
  const chunks = chunkedParamSet(motorTargets);
  let sent = 0;
  for (let i = 0; i < chunks.length; i++) {
    const req = sendParamSet(chunks[i]);
    if (!req) return; // sendParamSet 已写结果
    sent += chunks[i].length;
    // 串行：块间不并发，块 2 被拒时不会与块 1 的结果混在一起。
    await new Promise((r) => setTimeout(r, 250));
  }
  dirty.forEach((inp) => { delete inp.dataset.dirty; });
  renderParamRows();
  setParamResult(true, `已下发 ${byMotor.size} 台 / ${sent} 项` +
    (chunks.length > 1 ? `（分 ${chunks.length} 条）` : '') +
    '。受理/拒绝是异步的，最终成败看 CONFIG_RESULT 事件');
}

async function saveParamsToFlash() {
  if (!state.connected) return;
  if (!state.sysStatus.seen) { setParamResult(false, '还没收到 system_status'); return; }
  // 完成判据是 save_seq 前进**且** save_in_progress 已转 false。参数面时期
  // system.save 在 W25 落盘之前就被清掉，报的是假完成——这里等的是真终态。
  // W25 要擦扇区（量级 ~1s/扇区），给足 10s。
  const before = state.sysStatus.save_seq;
  // SAVE 只接受 MOTOR_ID_NONE；给具体电机 id 会被本地拒（last_error=10）。
  if (!sendSystemCommand(SYS_CMD.SAVE, MOTOR_ID_NONE)) return;
  setParamResult(true, '保存中…（写 W25，约 1–2s）');
  const rc = await waitSysSeq('save_seq', before, 10000);
  if (rc === 0) {
    const deadline = Date.now() + 5000;
    while (state.sysStatus.save_in_progress && Date.now() < deadline) {
      await new Promise((r) => setTimeout(r, 150));
    }
    if (state.sysStatus.save_in_progress) {
      setParamResult(false, 'save_seq 已前进，但 save_in_progress 仍是 true——' +
        'W25 落盘未出终态，不要断电');
      return;
    }
    renderParamRows();
    setParamResult(true, `已保存到 W25（save_seq=${state.sysStatus.save_seq}，落盘已出终态）`);
  } else if (rc === -2) {
    setParamResult(false, '收不到 system_status，无法判断保存是否完成（勿断电）');
  } else {
    setParamResult(false, `保存超时未收尾：${lastErrorText()}——勿断电，重试一次`);
  }
}

// 限位教学：把该台**当前关节位置**写进对应的限位槽。厂商手册要求"设置时电机失能、
// 掰到限位再设置"，内核也只受理未运动状态（device_limit_write_allowed），所以
// 先本地挡一道，再让固件的拒绝兜底（渲染与点击之间有竞态）。
// 写事务（写 + 读回核验）收尾后固件即时发一帧 device_param_report，顺带把
// 内核的钳制（CERB mit_position_limit_rad = ±π）暴露出来——超出 ±180° 的
// 位置会被钳到 ±180°，帧里就是钳后的真值，无需再单独发 REFRESH。
async function setLimitFromPosition(id, side) {
  if (!state.connected) { setParamResult(false, '未连接 rosbridge'); return; }
  const row = state.motors.get(id) || {};
  const lc = row.status?.lifecycle ?? row.topo?.lifecycle;
  if (lc === 4 || lc === 5) {
    setParamResult(false, `m${id} 正在运动中（${LIFECYCLE[lc]}）：限位只能在失能状态设置`);
    return;
  }
  const mo = row.motion;
  const pos = mo ? Number(mo.position) : NaN;
  if (!Number.isFinite(pos)) {
    setParamResult(false, `m${id} 还没有位置读数（motion_states 未到）——无法取当前位置`);
    return;
  }
  const v = paramCache.get(id);
  if (!v) {
    setParamResult(false, `m${id} 还没有参数记录——先点「读取」`);
    return;
  }
  // 另一侧取**显示中的现值**（驱动板优先），不是输入框里的值。
  const lo = side === 'min' ? pos : v.pick.pos_min;
  const hi = side === 'max' ? pos : v.pick.pos_max;
  if (!Number.isFinite(lo) || !Number.isFinite(hi)) {
    setParamResult(false, `m${id} 的另一侧限位没有可用值——先点「回读」`);
    return;
  }
  const which = side === 'min' ? '负限位（位置下限）' : '正限位（位置上限）';
  if (!(lo < hi)) {
    setParamResult(false, `m${id} 当前位置 ${pos.toFixed(4)} rad 作${which}后` +
      `下限 ${lo.toFixed(4)} 不小于上限 ${hi.toFixed(4)}：固件要求 min < max，会被整条拒。\n` +
      '先把关节掰到限位另一侧再设，或手动改另一侧的限位。');
    return;
  }
  if (!window.confirm(`把 m${id} 的当前位置设为${which}？\n\n` +
      `当前位置：${pos.toFixed(4)} rad（${(pos * 180 / Math.PI).toFixed(2)}°）\n` +
      `设完：下限 ${lo.toFixed(4)} / 上限 ${hi.toFixed(4)} rad\n\n` +
      '· 只能在该台失能时设置，请确认已经失能\n' +
      '· 超出 ±180° 会被设备侧钳制到 ±180°\n' +
      '· 设完自动回读一次，读回的才是设备真值')) {
    return;
  }
  const before = state.devParamSeq;
  if (!sendParamSet([{motor_id: id, field: PARAM_FIELD.POS_MIN, value: lo},
                     {motor_id: id, field: PARAM_FIELD.POS_MAX, value: hi}])) return;
  setParamResult(true, `m${id} ${which}已下发（${pos.toFixed(4)} rad），写事务收尾自动刷新…`);
  const rc = await waitDevParamSeq(before, 6000);
  if (rc === 0) {
    const now = paramCache.get(id);
    setParamResult(true, `m${id} ${which}写事务已收尾：下限 ` +
      `${fmtParam(now ? now.pick.pos_min : NaN)} / 上限 ` +
      `${fmtParam(now ? now.pick.pos_max : NaN)} rad`);
  } else if (rc === -3) {
    setParamResult(false, `m${id} 限位已下发，但写事务收尾帧超时未到：${lastErrorText()}`);
  } else {
    setParamResult(false, `m${id} 限位下发后连接中断：${lastErrorText()}`);
  }
}

// 设零点（设备对象 0x2531 写 1，字典表「设置电机输出端零位」）：
// 厂家要求**失能状态**下设置，运行中会被固件拒绝（不隐式失能）。
// 零点一变，设备里 0x607D 的限位也随之换了物理位置，须重新确认限位与机械行程。
// 是否掉电持久：字典未给"零位"对应的 0x2539 保存码，待实测确认。
async function setMotorZero(id) {
  if (!state.connected) return;
  const ok = window.confirm(
    `把电机 ${id} 的当前位置设为设备零点？\n\n` +
    '· 仅在该台失能时受理（运行中会被固件拒绝）\n' +
    '· 零点改变后设备内的位置限位也随之移动，请重新确认限位与机械行程\n' +
    '· 立即写入驱动器；是否掉电保存尚未实测确认');
  if (!ok) return;

  if (!state.sysStatus.seen) { setParamResult(false, '收不到 system_status，无法判断是否收尾'); return; }
  const before = state.sysStatus.set_zero_seq;
  if (!sendSystemCommand(SYS_CMD.SET_ZERO, id)) return;
  setParamResult(true, `电机 ${id} 设零点已请求…`);
  const rc = await waitSysSeq('set_zero_seq', before, 6000);
  if (rc === -2) {
    setParamResult(false, '收不到 system_status，无法判断设零点是否收尾');
    return;
  }
  if (rc === -3) {
    // 超时也要给出固件侧的错误码：请求可能在 mailbox/内核被拒（例如 FIFO 门、
    // 参数槽、kind 校验），那时序号永远不动，只看"未收尾"查不出原因。
    setParamResult(false, `电机 ${id} 设零点未收尾（已等 6 秒）：${lastErrorText()}` +
      (state.lastError === 0 ? '——错误码为 0 说明该版本没有这条命令（版本旧？）' : ''));
    return;
  }
  if (state.lastErrorMotor === id && state.lastError !== 0) {
    setParamResult(false, `电机 ${id} 设零点失败：${lastErrorText()}` +
      '（10 = 被拒——该台在运行中？厂家要求失能状态设置）');
  } else {
    setParamResult(true, `电机 ${id} 已设零点：当前姿态即零点` +
      '——设备内限位随零点移动，请重新确认限位与机械行程');
  }
  await loadParams();
}

// 故障清除（固件参数 `system.clear_fault` = <电机 id>，脉冲；完成序号
// `system.clear_fault_seq`）。内核按**成因**区分：
//   · 身份冲突锁存（同 motor_id 跨总线/协议身份不符）刻意不可解除 → 拒绝；
//   · 设备参数未初始化（last_error=40/41）→ 走一次回读重比对，真不一致仍锁；
//   · 厂家故障 → 发厂家清错帧（CERB 的 MIT 控制字 bit5；Vaxor 无清错位 → 拒绝）。
// 仅非运动态受理；清完仍须显式勾 enable + 新的完整快照/轨迹才可能回运动
// （ARD §36「失败不自动恢复运动授权」）。
async function clearMotorFault(id) {
  if (!state.connected) return;
  const ok = window.confirm(
    `清除电机 ${id} 的故障锁存？\n\n` +
    '· 仅在该台非运动态受理（运动中会被固件拒绝）\n' +
    '· 身份冲突锁存（跨总线/协议身份不符）无法清除，会被固件拒绝\n' +
    '· 设备参数未初始化会触发一次回读重比对\n' +
    '· 清完仍须显式使能 + 新的完整快照/轨迹才可能回运动');
  if (!ok) return;

  if (!state.sysStatus.seen) { setParamResult(false, '收不到 system_status，无法判断是否收尾'); return; }
  const before = state.sysStatus.clear_fault_seq;
  if (!sendSystemCommand(SYS_CMD.CLEAR_FAULT, id)) return;
  setParamResult(true, `电机 ${id} 故障清除已请求…`);
  const rc = await waitSysSeq('clear_fault_seq', before, 6000);
  if (rc === -2) {
    setParamResult(false, '收不到 system_status，无法判断故障清除是否收尾');
    return;
  }
  if (rc === -3) {
    setParamResult(false, `电机 ${id} 故障清除未收尾（已等 6 秒）：${lastErrorText()}` +
      '——看 motor_events 的 CONFIG_RESULT 定位是哪一层拒的');
    return;
  }
  if (state.lastErrorMotor === id && state.lastError !== 0) {
    setParamResult(false, `电机 ${id} 故障清除未成功：${lastErrorText()}` +
      '（身份冲突锁存不可解除、该台在运动中、或该协议无清错手段，都会被拒）');
  } else {
    setParamResult(true, `电机 ${id} 故障清除已受理：厂家故障需设备状态帧回报 ` +
      'fault_active=false 才收敛到 READY_DISABLED，届时再显式使能');
  }
  await loadParams();
}

// ---------- 初始化 ----------
function init() {
  const defaultRosUrl = (location.protocol === 'https:' ? 'wss://' : 'ws://')
    + (location.hostname || 'localhost') + ':9090';
  const rosUrl = $('rosUrl');
  rosUrl.value = localStorage.getItem(ROS_URL_STORAGE_KEY) || defaultRosUrl;
  rosUrl.addEventListener('input', () => {
    localStorage.setItem(ROS_URL_STORAGE_KEY, rosUrl.value.trim());
  });
  $('btnConnect').onclick = startAgent;
  $('btnDisconnect').onclick = stopAgent;
  $('btnPortRefresh').onclick = refreshAgentControls;
  $('serialPort').addEventListener('change', (e) => {
    if (e.target.value) localStorage.setItem(CONTROLLER_PORT_STORAGE_KEY, e.target.value);
  });
  // 左栏关节索引：事件委托（列表每次 renderMotors 都会重建）。
  $('motorRows').addEventListener('click', (e) => {
    const li = e.target.closest('li[data-id]');
    if (li) selectMotor(parseInt(li.dataset.id, 10));
  });
  $('chkAllParams').onchange = renderParamRows;
  initFold();

  // 臂视图是 ES module（默认 defer），到这里可能还没加载完——用事件交接，
  // 不赌脚本顺序。
  if (window.Arm3D) arm3dStart();
  else window.addEventListener('arm3d-ready', arm3dStart, {once: true});

  $('btnAddRow').onclick = () => addCmdRow();
  $('btnFillOnline').onclick = () => {
    // 只填拓扑中在线的电机，目标取当前实测位置（保持原位语义）；
    // 未收到运动状态时位置填 0 并给出提示色由用户核对后再发。
    const online = [];
    for (const [id, row] of state.motors) {
      if (row.topo && row.topo.online) online.push([id, row.motion ? row.motion.position : 0]);
    }
    if (online.length === 0) { alert('拓扑中暂无在线电机'); return; }
    $('cmdRows').innerHTML = '';
    for (const [id, pos] of online.sort((a, b) => a[0] - b[0])) addCmdRow(id, pos, 0, 0);
  };
  $('btnClearRows').onclick = () => { stopStream(); $('cmdRows').innerHTML = ''; };
  $('btnSendOnce').onclick = () => { stopStream(); sendSnapshot(); };
  $('chkStream').onchange = (e) => e.target.checked ? startStream() : stopStream();

  // 曲线
  $('chartCh').onchange = drawChart;
  $('chartMotor').onchange = drawChart;
  $('chartWindow').onchange = () => {
    chart.windowMs = parseInt($('chartWindow').value, 10);
    drawChart();
  };
  $('btnChartPause').onclick = () => {
    chart.running = !chart.running;
    $('btnChartPause').textContent = chart.running ? '暂停' : '继续';
  };
  $('btnChartClear').onclick = () => chartBuf.clear();

  // 事件日志 / 历史导出
  $('btnExportEvents').onclick = () => exportEvents(false);
  $('btnExportEventsCsv').onclick = () => exportEvents(true);
  $('btnClearEvents').onclick = () => { state.events = []; renderEvents(); };
  $('btnExportHist').onclick = exportHistory;
  $('btnClearHist').onclick = () => { history.length = 0; renderHistory(); };

  // 示教
  $('btnTeachPlay').onclick = teachPlay;
  $('btnTeachPause').onclick = teachPause;
  $('btnTeachStep').onclick = teachStep;
  $('btnTeachRecord').onclick = teachRecordCmd;
  $('btnTeachAdd').onclick = () => teachAddRow();
  $('btnTeachExport').onclick = teachExport;
  $('btnTeachImport').onclick = teachImport;

  $('btnParamLoad').onclick = loadParams;
  $('btnParamApply').onclick = applyParams;
  $('btnParamSave').onclick = saveParamsToFlash;

  addCmdRow(1, 0, 0, 0);
  teachAddRow(1000, '1:0,0,0');

  // rosbridge 开页即连；自定义地址保存在浏览器本地，刷新后自动重连。
  // 启动和断开按钮只管理 micro-ROS Agent，不影响 WebSocket。
  refreshAgentControls();
  setInterval(refreshAgentStatus, 2000);
  connect();
}

init();
})();
