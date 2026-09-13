// Robot2 Web 上位机：监控（topology/motion/status/events/imu）+ 控制下发
// （bind / 全局 enable / 逐电机 policy / command 快照）。
// 话题与 can_bridge_msgs 对齐固件；QoS 由 rosbridge 2.x 按发布方自动协商。
(() => {
'use strict';

// ---------- 常量 ----------
const TOPICS = {
  topology: {name: '/micro_ros/motor_topology', type: 'can_bridge_msgs/msg/MotorTopology'},
  motion: {name: '/micro_ros/motion_states', type: 'can_bridge_msgs/msg/MotorMotionStateArray'},
  status: {name: '/micro_ros/motor_status_array', type: 'can_bridge_msgs/msg/MotorStatusArray'},
  events: {name: '/micro_ros/motor_events', type: 'can_bridge_msgs/msg/MotorEvent'},
  imu: {name: '/micro_ros/imu/data', type: 'sensor_msgs/msg/Imu'},
  temp: {name: '/micro_ros/imu/temp', type: 'std_msgs/msg/Float32'},
  bind: {name: '/micro_ros/motor_bind', type: 'can_bridge_msgs/msg/MotorBind'},
  policy: {name: '/micro_ros/motor_policy', type: 'can_bridge_msgs/msg/MotorPolicy'},
  enable: {name: '/micro_ros/motor_enable', type: 'std_msgs/msg/Bool'},
  command: {name: '/motor_command_array', type: 'can_bridge_msgs/msg/MotorCommandArray'},
};

const LIFECYCLE = ['ABSENT', 'DISCOVERED', 'IDENTIFYING', 'READY_DISABLED',
  'ACTIVE', 'SAFE_STOP', 'FAULT', 'CONFIG_CONFLICT', 'OFFLINE'];
const PROTOCOL = ['VAXOR', 'CERB'];
const EVENT_NAMES = ['DISCOVERED', 'PROFILE_BOUND', 'PROFILE_REJECTED', 'READY',
  'ACTIVE', 'SOFT_STOP', 'DISABLE', 'FAULT', 'CONFIG_CONFLICT', 'OFFLINE',
  'RECONNECTED', 'AUTHORIZATION_LOST', 'SNAPSHOT_REJECTED', 'CONFIG_RESULT',
  'REGISTRY_FULL'];
const FAULT_NAMES = ['无', '协议', '厂家', 'Profile', '冲突', '离线'];
const STREAM_PERIOD_MS = 10; // 100Hz 连发

// ---------- 工具 ----------
const $ = (id) => document.getElementById(id);
const fmt = (v, d = 2) => (v === null || v === undefined || !Number.isFinite(v))
  ? '--' : v.toFixed(d);
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

// ---------- 状态 ----------
// motors: id -> {topo, motion, status}
const state = {
  connected: false,
  motors: new Map(),
  events: [],
  imu: {count: 0, last: 0, roll: 0, pitch: 0, yaw: 0},
  temp: undefined,
  tempLast: 0,
  motionCount: 0,
};

// ---------- rosbridge ----------
let ros = null;
let subs = [];
const publishers = {};

function setConn(ok, text) {
  state.connected = ok;
  $('connDot').classList.toggle('on', ok);
  $('connText').textContent = text || (ok ? '已连接' : '未连接');
}

function pub(key, message) {
  if (!state.connected) return false;
  if (!publishers[key]) {
    const t = TOPICS[key];
    publishers[key] = new ROSLIB.Topic({ros, name: t.name, messageType: t.type});
    publishers[key].advertise();
  }
  publishers[key].publish(new ROSLIB.Message(message));
  return true;
}

function connect() {
  disconnect();
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
  });
  ros.on('error', () => setConn(false, '连接错误'));
  ros.on('close', () => {
    setConn(false);
    stopStream();
    subs = [];
    Object.values(publishers).forEach((p) => { try { p.unadvertise(); } catch (e) {} });
    Object.keys(publishers).forEach((k) => delete publishers[k]);
  });
}

function disconnect() {
  stopStream();
  if (ros) { try { ros.close(); } catch (e) {} ros = null; }
  setConn(false);
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
    for (const s of (m.states || [])) {
      const row = state.motors.get(s.motor_id) || {};
      row.motion = s;
      state.motors.set(s.motor_id, row);
    }
  });

  sub('status', (m) => {
    for (const s of (m.statuses || [])) {
      const row = state.motors.get(s.motor_id) || {};
      row.status = s;
      state.motors.set(s.motor_id, row);
    }
  });

  sub('events', (m) => {
    state.events.unshift({
      t: Date.now(),
      name: EVENT_NAMES[m.event_type] !== undefined
        ? EVENT_NAMES[m.event_type] : 'EVENT_' + m.event_type,
      id: m.motor_id,
      reason: m.reason,
    });
    if (state.events.length > 50) state.events.pop();
    renderEvents();
  });

  sub('imu', (m) => {
    const q = m.orientation;
    const e = quatToEuler(q.x, q.y, q.z, q.w);
    state.imu.roll = e.roll;
    state.imu.pitch = e.pitch;
    state.imu.yaw = e.yaw;
    state.imu.count++;
    state.imu.last = Date.now();
  });

  sub('temp', (m) => { state.temp = m.data; state.tempLast = Date.now(); });
}

// ---------- 渲染 ----------
function lifecyclePill(lc) {
  if (lc === undefined || lc === null) return '--';
  const name = LIFECYCLE[lc] !== undefined ? LIFECYCLE[lc] : 'S' + lc;
  const cls = lc === 4 ? 'on' : (lc === 5 ? 'warn' : (lc === 6 || lc === 7 || lc === 8 ? 'off' : 'mid'));
  return `<span class="pill ${cls}">${name}</span>`;
}

function renderMotors() {
  const tbody = $('motorRows');
  const rows = [...state.motors.entries()]
    .sort((a, b) => a[0] - b[0])
    .map(([id, r]) => {
      const t = r.topo || {}, mo = r.motion || {}, st = r.status || {};
      const online = st.online !== undefined ? st.online : t.online;
      const lc = st.lifecycle !== undefined ? st.lifecycle : t.lifecycle;
      const fault = st.unified_fault_code || 0;
      const model = t.model_name || '';
      const proto = t.protocol !== undefined ? PROTOCOL[t.protocol] : '--';
      const age = mo.state_age_ms !== undefined ? mo.state_age_ms : st.state_age_ms;
      return `<tr${fault ? ' class="faultrow"' : ''}>
        <td class="num">${id}</td>
        <td>${proto}</td>
        <td>${model ? model : '<span class="muted">未绑定</span>'}</td>
        <td>${online === undefined ? '--'
          : `<span class="pill ${online ? 'on' : 'off'}">${online ? '是' : '否'}</span>`}</td>
        <td>${lifecyclePill(lc)}</td>
        <td class="num">${fmt(mo.position, 3)}</td>
        <td class="num">${fmt(mo.velocity, 3)}</td>
        <td class="num">${fmt(mo.effort, 3)}</td>
        <td class="num">${fmt(st.current, 2)}</td>
        <td>${fault ? `<span class="pill off">${FAULT_NAMES[fault] || fault}</span>`
                    : '<span class="muted">无</span>'}</td>
        <td class="num">${ageText(age)}</td>
      </tr>`;
    });
  tbody.innerHTML = rows.length
    ? rows.join('')
    : '<tr class="empty"><td colspan="11">暂无数据（等待 motor_topology）</td></tr>';

  // 汇总 chips。
  let onlineN = 0, faultN = 0;
  for (const r of state.motors.values()) {
    const online = r.status && r.status.online !== undefined ? r.status.online
      : (r.topo ? r.topo.online : false);
    if (online) onlineN++;
    if (r.status && r.status.unified_fault_code) faultN++;
  }
  $('chipOnline').textContent = `在线 ${onlineN}/${state.motors.size}`;
  $('chipFault').textContent = `故障 ${faultN}`;
}

function renderEvents() {
  $('eventLog').innerHTML = state.events.map((e) =>
    `<li><span class="muted">${new Date(e.t).toLocaleTimeString()}</span> `
    + `<b>${e.name}</b> · id=${e.id}${e.reason ? ' · reason=' + e.reason : ''}</li>`
  ).join('') || '<li class="muted">暂无事件</li>';
}

function renderImu() {
  $('valRoll').textContent = fmt(rad2deg(state.imu.roll), 1) + ' °';
  $('valPitch').textContent = fmt(rad2deg(state.imu.pitch), 1) + ' °';
  $('valYaw').textContent = fmt(rad2deg(state.imu.yaw), 1) + ' °';
  $('valTemp').textContent = state.temp !== undefined ? fmt(state.temp, 1) + ' °C' : '--';
  const ageMs = Date.now() - state.imu.last;
  $('imuAge').textContent = state.imu.last
    ? (ageMs < 3000 ? ageMs + ' ms' : '超时')
    : '--';
}

// ---------- 控制：绑定 / 授权 / 使能 ----------
function doBind() {
  const id = parseInt($('bindId').value, 10);
  const model = $('bindModel').value;
  if (!Number.isInteger(id) || id < 1 || id > 65535) { alert('motor_id 非法'); return; }
  pub('bind', {motor_id: id, model_name: model});
}

function doGlobal(enable) {
  pub('enable', {data: enable});
}

function doPolicy(enable) {
  const id = parseInt($('polId').value, 10);
  if (!Number.isInteger(id) || id < 1 || id > 65535) { alert('motor_id 非法'); return; }
  // 只翻 enable 位，set_gains/set_limits 不置位，其余字段保持原值。
  pub('policy', {
    motor_id: id,
    enable,
    set_gains: false, kp: 0.0, kd: 0.0,
    set_limits: false, min_position_rad: 0.0, max_position_rad: 0.0,
  });
}

// ---------- 控制：命令快照 ----------
let seq = 0;
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

function sendSnapshot() {
  if (!state.connected) { setCmdResult(false, '未连接 rosbridge'); return false; }
  const {targets, error} = readCmdTargets();
  if (error) { setCmdResult(false, error); return false; }
  seq = (seq + 1) >>> 0;
  pub('command', {
    // uint64：rosbridge 2.x 入站要求 JSON number（float64 会丢失末几位
    // 精度；该字段仅用于跨系统日志对齐，freshness 以 MCU 本地 tick 为准）。
    sync_time_ns: Date.now() * 1e6,
    sequence: seq,
    count: targets.length,
    targets,
  });
  setCmdResult(true, `已发 seq=${seq}，${targets.length} 台`);
  return true;
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

// ---------- 定时刷新 ----------
setInterval(renderMotors, 200);
setInterval(renderImu, 200);

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

// ---------- 初始化 ----------
function init() {
  $('rosUrl').value = (location.protocol === 'https:' ? 'wss://' : 'ws://')
    + (location.hostname || 'localhost') + ':9090';
  $('btnConnect').onclick = connect;
  $('btnDisconnect').onclick = disconnect;
  $('rosUrl').addEventListener('keydown', (e) => { if (e.key === 'Enter') connect(); });

  $('btnBind').onclick = doBind;
  $('btnGlobalEnable').onclick = () => doGlobal(true);
  $('btnGlobalDisable').onclick = () => doGlobal(false);
  $('btnPolEnable').onclick = () => doPolicy(true);
  $('btnPolDisable').onclick = () => doPolicy(false);

  $('btnAddRow').onclick = () => addCmdRow();
  $('btnFillOnline').onclick = () => {
    $('cmdRows').innerHTML = '';
    for (let id = 1; id <= 9; id++) addCmdRow(id, 0, 0, 0);
  };
  $('btnClearRows').onclick = () => { stopStream(); $('cmdRows').innerHTML = ''; };
  $('btnSendOnce').onclick = () => { stopStream(); sendSnapshot(); };
  $('chkStream').onchange = (e) => e.target.checked ? startStream() : stopStream();

  addCmdRow(1, 0, 0, 0);
}

init();
})();
