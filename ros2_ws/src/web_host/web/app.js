(() => {
'use strict';

// ---------- 常量 ----------
const LIFECYCLE = ['ABSENT', 'DISCOVERED', 'IDENTIFYING', 'READY_DISABLED',
  'ACTIVE', 'SAFE_STOP', 'FAULT', 'CONFIG_CONFLICT', 'OFFLINE'];
const EVENTS = ['DISCOVERED', 'PROFILE_BOUND', 'PROFILE_REJECTED', 'READY',
  'ACTIVE', 'SOFT_STOP', 'DISABLE', 'FAULT', 'CONFIG_CONFLICT', 'OFFLINE',
  'RECONNECTED', 'AUTHORIZATION_LOST', 'SNAPSHOT_REJECTED', 'CONFIG_RESULT',
  'REGISTRY_FULL'];
const UNIFIED_FAULT = ['NONE', '协议', '厂家', 'Profile', '冲突', '离线'];

const TOPICS = {
  heartbeat: {name: '/firmware_heartbeat', type: 'std_msgs/UInt32'},
  imu: {name: '/micro_ros/imu/data', type: 'sensor_msgs/Imu'},
  temp: {name: '/micro_ros/imu/temp', type: 'std_msgs/Float32'},
  motorStatus: {name: '/motor_status', type: 'can_bridge_msgs/MotorStatusArray'},
  motorMotion: {name: '/motor_motion_states', type: 'can_bridge_msgs/MotorMotionStateArray'},
  motorTopo: {name: '/motor_topology', type: 'can_bridge_msgs/MotorTopology'},
  motorEvent: {name: '/motor_events', type: 'can_bridge_msgs/MotorEvent'},
  motorCmd: {name: '/motor_command', type: 'can_bridge_msgs/MotorCommandArray'},
};

// ---------- 工具 ----------
const $ = (id) => document.getElementById(id);

function quatToEuler(x, y, z, w) {
  const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  const sinp = 2 * (w * y - z * x);
  const pitch = sinp >= 1 ? Math.PI / 2 : (sinp <= -1 ? -Math.PI / 2 : Math.asin(sinp));
  const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return {roll, pitch, yaw};
}
const rad2deg = (r) => r * 180 / Math.PI;
const fmt = (v, d = 2) => (v === null || v === undefined || !Number.isFinite(v)) ? '--' : v.toFixed(d);
const ageText = (t) => {
  if (!t) return '--';
  const ms = Date.now() - t;
  return ms < 3000 ? ms + ' ms' : (ms / 1000).toFixed(1) + ' s';
};

// ---------- 状态 ----------
const state = {
  imuCount: 0, imuHz: 0, imuLastT: 0,
  hbCount: null, hbLastT: 0, hbHz: 0,
  tempLastT: 0,
  euler: {roll: 0, pitch: 0, yaw: 0},
  curves: {roll: [], pitch: [], yaw: []},  // 环形缓存，最大 600 点
  motors: new Map(),   // id -> 融合后的状态行
  events: [],          // 最近 50 条
  motorStatusT: 0,
  dirty: {curves: false, motors: false},
};

function pushCurve(arr, v) {
  arr.push(v);
  if (arr.length > 600) arr.splice(0, arr.length - 600);
}

// ---------- rosbridge 连接 ----------
let ros = null;
let cmdTopic = null;
let connected = false;

function setConn(ok, text) {
  connected = ok;
  $('connDot').classList.toggle('on', ok);
  $('connText').textContent = text || (ok ? '已连接' : '未连接');
}

function connect() {
  const url = $('rosUrl').value.trim();
  disconnect();
  try {
    ros = new ROSLIB.Ros({url});
  } catch (e) {
    setConn(false, 'URL 无效');
    return;
  }
  ros.on('connection', () => {
    setConn(true);
    subscribeAll();
    $('sysNote').textContent = '已连接 rosbridge，等待话题数据…';
  });
  ros.on('error', () => setConn(false, '连接错误'));
  ros.on('close', () => {
    setConn(false);
    if (cmdTopic) { cmdTopic.unsubscribe && cmdTopic.unsubscribe(); cmdTopic = null; }
  });
}

function disconnect() {
  if (ros) { try { ros.close(); } catch (e) { /* 忽略重复关闭 */ } ros = null; }
  cmdTopic = null;
  setConn(false);
}

function makeSub(key, cb, throttleMs) {
  const t = TOPICS[key];
  const topic = new ROSLIB.Topic({
    ros,
    name: t.name,
    messageType: t.type,
    throttle_rate: throttleMs || 0,
    queue_length: 1,
  });
  topic.subscribe(cb);
  return topic;
}

let subs = [];
function subscribeAll() {
  subs.forEach((s) => s.unsubscribe());
  subs = [
    makeSub('heartbeat', (m) => {
      state.hbCount = m.data;
      state.hbLastT = Date.now();
    }),
    makeSub('imu', (m) => {
      const q = m.orientation;
      const e = quatToEuler(q.x, q.y, q.z, q.w);
      state.euler = e;
      pushCurve(state.curves.roll, rad2deg(e.roll));
      pushCurve(state.curves.pitch, rad2deg(e.pitch));
      pushCurve(state.curves.yaw, rad2deg(e.yaw));
      state.imuCount++;
      state.imuLastT = Date.now();
      state.dirty.curves = true;
    }),
    makeSub('temp', (m) => {
      $('valTemp').textContent = fmt(m.data, 1) + ' °C';
      state.tempLastT = Date.now();
    }),
    makeSub('motorStatus', (m) => {
      for (const s of (m.statuses || [])) {
        const row = state.motors.get(s.motor_id) || {id: s.motor_id};
        Object.assign(row, {
          temperature: s.temperature, current: s.current, voltage: s.voltage,
          online: s.online, lifecycle: s.lifecycle,
          unified_fault_code: s.unified_fault_code,
          protection_flags: s.protection_flags, limit_flags: s.limit_flags,
          statusT: Date.now(),
        });
        state.motors.set(s.motor_id, row);
      }
      state.dirty.motors = true;
      state.motorStatusT = Date.now();
    }),
    makeSub('motorMotion', (m) => {
      for (const s of (m.states || [])) {
        const row = state.motors.get(s.motor_id) || {id: s.motor_id};
        Object.assign(row, {
          position: s.position, velocity: s.velocity, effort: s.effort,
          state_age_ms: s.state_age_ms, motionT: Date.now(),
        });
        state.motors.set(s.motor_id, row);
      }
      state.dirty.motors = true;
    }),
    makeSub('motorEvent', (m) => {
      const t = Date.now();
      const name = EVENTS[m.event_type] !== undefined ? EVENTS[m.event_type] : ('EVENT_' + m.event_type);
      state.events.unshift({t, name, id: m.motor_id, reason: m.reason});
      if (state.events.length > 50) state.events.pop();
      renderEvents();
    }),
  ];
  cmdTopic = new ROSLIB.Topic({
    ros, name: TOPICS.motorCmd.name, messageType: TOPICS.motorCmd.type,
  });
}

// ---------- 渲染：姿态球 ----------
function renderAttitude() {
  const {roll, pitch} = state.euler;
  const rd = rad2deg(roll), pd = rad2deg(pitch);
  const ball = $('attBall');
  ball.style.transform = `rotate(${-rd}deg) translateY(${pd * 1.6}px)`;
  $('valRoll').textContent = fmt(rd, 1) + ' °';
  $('valPitch').textContent = fmt(pd, 1) + ' °';
  $('valYaw').textContent = fmt(rad2deg(state.euler.yaw), 1) + ' °';
}

// ---------- 渲染：IMU 曲线 ----------
const chart = $('imuChart');
const ctx = chart.getContext('2d');
const CURVE_STYLE = [
  {key: 'roll', color: '#ff6b6b'},
  {key: 'pitch', color: '#4da3ff'},
  {key: 'yaw', color: '#37d67a'},
];

function drawChart() {
  const dpr = window.devicePixelRatio || 1;
  const w = chart.clientWidth, h = chart.clientHeight;
  if (chart.width !== w * dpr) {
    chart.width = w * dpr;
    chart.height = h * dpr;
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);

  let min = -30, max = 30;
  for (const c of CURVE_STYLE) {
    for (const v of state.curves[c.key]) {
      if (v < min) min = v;
      if (v > max) max = v;
    }
  }
  const pad = (max - min) * 0.1 || 1;
  min -= pad; max += pad;

  ctx.strokeStyle = '#26314a';
  ctx.beginPath();
  ctx.moveTo(0, h / 2);
  ctx.lineTo(w, h / 2);
  ctx.stroke();

  const n = Math.max(...CURVE_STYLE.map((c) => state.curves[c.key].length), 1);
  for (const c of CURVE_STYLE) {
    const data = state.curves[c.key];
    if (data.length < 2) continue;
    ctx.strokeStyle = c.color;
    ctx.lineWidth = 1.4;
    ctx.beginPath();
    for (let i = 0; i < data.length; i++) {
      const x = (i / (n - 1)) * w;
      const y = h - ((data[i] - min) / (max - min)) * h;
      i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    }
    ctx.stroke();
  }
}

// ---------- 渲染：电机表 ----------
function lifecyclePill(lc) {
  const name = LIFECYCLE[lc] !== undefined ? LIFECYCLE[lc] : ('S' + lc);
  const cls = lc === 4 ? 'on' : (lc === 6 || lc === 8 ? 'off' : 'mid');
  return `<span class="pill ${cls}">${name}</span>`;
}

function renderMotors() {
  const tbody = $('motorRows');
  const rows = [...state.motors.values()].sort((a, b) => a.id - b.id);
  if (!rows.length) {
    tbody.innerHTML = '<tr class="empty"><td colspan="10">暂无数据（等待 /motor_status 与 /motor_motion_states）</td></tr>';
    return;
  }
  tbody.innerHTML = rows.map((m) => {
    const flags = [];
    if (m.unified_fault_code) flags.push(UNIFIED_FAULT[m.unified_fault_code] || ('code' + m.unified_fault_code));
    if (m.protection_flags) flags.push('prot:0x' + m.protection_flags.toString(16));
    if (m.limit_flags) flags.push('lim:0x' + m.limit_flags.toString(16));
    const age = m.state_age_ms === undefined ? '--'
      : (m.state_age_ms === 0xFFFFFFFF ? '从未' : m.state_age_ms + ' ms');
    return `<tr>
      <td class="num">${m.id}</td>
      <td>${m.online === undefined ? '--' : `<span class="pill ${m.online ? 'on' : 'off'}">${m.online ? '是' : '否'}</span>`}</td>
      <td>${m.lifecycle === undefined ? '--' : lifecyclePill(m.lifecycle)}</td>
      <td class="num">${fmt(m.temperature, 1)}</td>
      <td class="num">${fmt(m.current, 2)}</td>
      <td class="num">${fmt(m.voltage, 2)}</td>
      <td class="num">${fmt(m.velocity, 3)}</td>
      <td class="num">${fmt(m.position, 3)}</td>
      <td>${flags.length ? flags.join(' ') : '<span class="muted">无</span>'}</td>
      <td class="num">${age}</td>
    </tr>`;
  }).join('');
}

function renderEvents() {
  $('eventLog').innerHTML = state.events.map((e) =>
    `<li><span class="muted">${new Date(e.t).toLocaleTimeString()}</span>
     ${e.name} · motor_id=${e.id}${e.reason ? ' · reason=' + e.reason : ''}</li>`
  ).join('') || '<li class="muted">暂无事件</li>';
}

// ---------- 命令面板 ----------
let seq = 0;

function addCmdRow(id = 1, pos = 0, vel = 0, eff = 0) {
  const tr = document.createElement('tr');
  tr.innerHTML = `
    <td><input type="number" class="c-id" value="${id}" min="1" max="65535" step="1"></td>
    <td><input type="number" class="c-pos" value="${pos}" step="0.01"></td>
    <td><input type="number" class="c-vel" value="${vel}" step="0.01"></td>
    <td><input type="number" class="c-eff" value="${eff}" step="0.01"></td>
    <td><button class="c-del secondary">删</button></td>`;
  tr.querySelector('.c-del').onclick = () => tr.remove();
  $('cmdRows').appendChild(tr);
}

function setCmdResult(ok, text) {
  const el = $('cmdResult');
  el.className = ok ? 'ok' : 'err';
  el.textContent = text;
}

function sendCmd() {
  if (!connected) { setCmdResult(false, '未连接 rosbridge'); return; }
  const targets = [];
  for (const tr of $('cmdRows').querySelectorAll('tr')) {
    const id = parseInt(tr.querySelector('.c-id').value, 10);
    const pos = parseFloat(tr.querySelector('.c-pos').value);
    const vel = parseFloat(tr.querySelector('.c-vel').value);
    const eff = parseFloat(tr.querySelector('.c-eff').value);
    if (!Number.isInteger(id) || id < 1 || id > 65535) {
      setCmdResult(false, `motor_id 非法：${id}`); return;
    }
    // ARD-0007：任一 NaN/Inf → 整快照拒收，发送前拦截
    if (![pos, vel, eff].every(Number.isFinite)) {
      setCmdResult(false, `电机 ${id} 的数值必须为有限数`); return;
    }
    targets.push({motor_id: id, position: pos, velocity: vel, effort: eff});
  }
  if (!targets.length) { setCmdResult(false, '快照为空（将导致全部软停），请先添加电机'); return; }

  seq = (seq + 1) >>> 0;
  cmdTopic.publish({
    // uint64 字段以字符串传输，避免 JS number 精度丢失（rosbridge 端 int() 还原）
    sync_time_ns: (BigInt(Date.now()) * 1000000n).toString(),
    sequence: seq,
    count: targets.length,
    targets,
  });
  setCmdResult(true, `已发送快照 seq=${seq}，${targets.length} 台电机`);
}

// ---------- 周期刷新 ----------
setInterval(() => {
  $('hbCount').textContent = state.hbCount === null ? '--' : state.hbCount;
  $('hbAge').textContent = ageText(state.hbLastT);
  $('imuAge').textContent = ageText(state.imuLastT);
  $('tempAge').textContent = $('valTemp').textContent === '--'
    ? '--' : '最近 ' + ageText(state.tempLastT);
  if (state.hbLastT && Date.now() - state.hbLastT < 3000) {
    $('hbAge').classList.add('pulse-ok');
  } else {
    $('hbAge').classList.remove('pulse-ok');
  }
}, 300);

// Hz 统计（每秒）
let lastImuCount = 0;
setInterval(() => {
  state.imuHz = state.imuCount - lastImuCount;
  lastImuCount = state.imuCount;
  $('imuHz').textContent = state.imuHz + ' Hz';
  const ms = document.querySelector('#cardMotors h2 .hz');
  if (state.motorStatusT && Date.now() - state.motorStatusT < 3000) {
    ms.textContent = '状态流活跃';
  } else {
    ms.textContent = '';
  }
}, 1000);

// 渲染循环
function frame() {
  renderAttitude();
  if (state.dirty.curves) { drawChart(); state.dirty.curves = false; }
  if (state.dirty.motors) { renderMotors(); state.dirty.motors = false; }
  requestAnimationFrame(frame);
}

// ---------- 初始化 ----------
function init() {
  const defaultUrl = (location.protocol === 'https:' ? 'wss://' : 'ws://')
    + (location.hostname || 'localhost') + ':9090';
  $('rosUrl').value = defaultUrl;

  $('btnConnect').onclick = connect;
  $('btnDisconnect').onclick = disconnect;
  $('rosUrl').addEventListener('keydown', (e) => { if (e.key === 'Enter') connect(); });
  $('btnAddRow').onclick = () => addCmdRow();
  $('btnSendCmd').onclick = sendCmd;
  addCmdRow();

  window.addEventListener('resize', () => { state.dirty.curves = true; });
  requestAnimationFrame(frame);
}

init();
})();
