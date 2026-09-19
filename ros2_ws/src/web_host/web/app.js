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
  policy: {name: '/micro_ros_do/motor_policy', type: 'can_bridge_msgs/msg/MotorPolicyArray'},
};

const LIFECYCLE = ['ABSENT', 'DISCOVERED', 'IDENTIFYING', 'READY_DISABLED',
  'ACTIVE', 'SAFE_STOP', 'FAULT', 'CONFIG_CONFLICT', 'OFFLINE'];
const PROTOCOL = ['VAXOR', 'CERB'];
const EVENT_NAMES = ['DISCOVERED', 'PROFILE_BOUND', 'PROFILE_REJECTED', 'READY',
  'ACTIVE', 'SOFT_STOP', 'DISABLE', 'FAULT', 'CONFIG_CONFLICT', 'OFFLINE',
  'RECONNECTED', 'AUTHORIZATION_LOST', 'SNAPSHOT_REJECTED', 'CONFIG_RESULT',
  'REGISTRY_FULL', 'DEV_INIT', 'TRAJECTORY_REJECTED'];
const FAULT_NAMES = ['无', '协议', '厂家', 'Profile', '冲突', '离线'];
// 设备参数槽位名（DEV_INIT 事件 request_id 携带的槽位位图，1u<<slot）。
// ⚠️ 这是与固件位图的**隐式契约**：槽位顺序必须逐位对齐 firmware 的
// device_raw 槽索引（MOTOR_PROTOCOL_PARAM_SLOT_*），权威定义在
// micro_ros_agent/src/can_bridge_msgs/msg/MotorEvent.msg 的 EVENT_DEV_INIT
// 注释块。固件新增槽位时，这里与那条注释必须同步改——只改一边会静默错位。
const DEV_SLOT_NAMES = ['位置下限', '位置上限', '最大转速', '峰值电流',
  '保护开关(0x2650:00)', '方向(0x2541)'];
// 参数面能改的槽位：0..3（限值四项）+ 4（保护开关，m<id>.limit_protect）。
// 只剩槽 5（方向 0x2541）不在参数面——它没有对应的参数项，写不回去。
const DEV_SLOT_PARAM_MASK = 0x1F;
const DEV_SLOT_EXTERNAL_MASK = 0x20;
const DEV_INIT_STATE = ['无设备对象', '待回读', '已核对', '与记录不一致', '回读不成功'];
const STREAM_PERIOD_MS = 10; // 100Hz 连发
const CHART_COLORS = ['#4da3ff', '#37d67a', '#f5b94a', '#ff6b6b', '#c792ea',
  '#f78c6c', '#82aaff', '#ff9e64', '#7e8bab'];

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
  lastError: 0,       // system.last_error 快照（总览条与失败文案翻译用）
  lastErrorMotor: -1,
  devInit: new Map(), // motor_id -> {state, mask}：DEV_INIT 事件带来的核对结论
  events: [],
  imu: {count: 0, last: 0, roll: 0, pitch: 0, yaw: 0, q: null},
  temp: undefined,
  tempLast: 0,
  motionCount: 0,
};

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
    loadParams(); // 参数是默认页签：连上后自动读取，避免打开看到空表
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
    const t = Date.now();
    for (const s of (m.states || [])) {
      const row = state.motors.get(s.motor_id) || {};
      row.motion = s;
      state.motors.set(s.motor_id, row);
      chartPush(s.motor_id, 'position', s.position, t);
      chartPush(s.motor_id, 'velocity', s.velocity, t);
      chartPush(s.motor_id, 'effort', s.effort, t);
    }
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

function renderMotors() {
  // 参数页总览条依赖电机状态流（在线/生命周期），随之刷新；表未加载时是空操作。
  if (paramCache.size > 0) renderParamSummary();
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
        <td title="${devDetailTitle(id)}">${lifecyclePill(lc)}</td>
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
// sequence 仅作日志关联（固件序号门已移除，2026-09-15）。
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

// 发布一条多点轨迹：固件在自己的 20ms 控制边界按 time_from_start_ms 段内
// 线性插值执行，速度取段斜率。轨迹声明集合内的电机由轨迹**独占**——同一时刻
// 到达的 100Hz 快照对它们既不下发也不软停，所以播放前必须停掉连发，否则
// "命令表说的"与"机器人做的"会分叉。空轨迹（无路点）= 取消：固件释放归属
// 并软停这些电机。
function sendTrajectory(points, targetIds) {
  if (!state.connected) return {ok: false, msg: '未连接 rosbridge'};
  const ids = targetIds ?? [...new Set(points.map((x) => x.motor_id))];
  seq = (seq + 1) >>> 0;
  pub('trajectory', {
    sync_time_ns: Date.now() * 1e6, // 仅用于跨系统日志对齐
    sequence: seq,
    count: points.length,
    target_count: ids.length,
    target_ids: ids,
    points,
  });
  const total = points.length ? points[points.length - 1].time_from_start_ms : 0;
  return {ok: true, msg: `轨迹 seq=${seq}：${points.length} 个路点 / ${ids.length} 台` +
    (points.length ? `，总时长 ${total} ms` : '')};
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
const teach = {playing: false, timer: null, idx: 0};

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
        targets.push({motor_id: +m[1], position: +m[2], velocity: +m[3], effort: +m[4]});
      }
    }
    rows.push({wait: Number.isFinite(wait) && wait >= 0 ? wait : 0, targets});
  }
  return {rows};
}

function teachStopTimer() {
  if (teach.timer) { clearTimeout(teach.timer); teach.timer = null; }
}

function teachStepGo() {
  if (!teach.playing) return;
  const row = teach.rows[teach.idx];
  let res = {ok: true, msg: '空点位（不下发）'};
  if (row.targets.length) res = sendTargets(row.targets);
  setTeach(res.ok ? 'ok' : 'err',
    (res.ok ? '✓' : '✗') + ` 点位 ${teach.idx + 1}/${teach.rows.length}: ${res.msg}`);
  let next = teach.idx + 1;
  if (next >= teach.rows.length) {
    if ($('chkTeachLoop').checked) { next = 0; }
    else {
      teach.playing = false;
      setTeach('ok', '播放完成');
      return;
    }
  }
  teach.idx = next;
  teach.timer = setTimeout(teachStepGo, row.wait);
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

// 播放改为把整条时间线交给 MCU 执行（不再由浏览器逐点 setTimeout）：
// 定时抖动不再影响运动质量，松手/切页也不会中断。
function teachPlay() {
  const {rows, error} = teachReadRows();
  if (error) { setTeach('err', error); return; }
  if (!rows.length) { setTeach('err', '没有点位'); return; }
  const {points, ids} = teachRowsToTrajectory(rows);
  if (!points.length) { setTeach('err', '所有点位都是空的'); return; }
  stopStream(); // 轨迹独占这些电机：先停 100Hz 连发，避免两条命令源分叉
  teachStopTimer();
  teach.playing = true;
  teach.rows = rows;
  const res = sendTrajectory(points, ids);
  setTeach(res.ok ? 'ok' : 'err',
    (res.ok ? '✓ 已托管给 MCU：' : '✗ ') + res.msg);
}

// 停止 = 发空轨迹取消：固件释放归属并软停这些电机。播完不会自动取消
// （固件保持末点，机构无抱闸，软停会掉臂），所以"停"必须显式点。
function teachPause() {
  const wasPlaying = teach.playing;
  teach.playing = false;
  teachStopTimer();
  if (!wasPlaying) { setTeach('ok', '未在播放'); return; }
  const res = cancelTrajectory();
  setTeach(res.ok ? 'ok' : 'err',
    res.ok ? '✓ 已取消轨迹（这些电机软停并交回快照通道）' : `✗ ${res.msg}`);
}

function teachStep() {
  teach.playing = false;
  teachStopTimer();
  const {rows, error} = teachReadRows();
  if (error) { setTeach('err', error); return; }
  if (!rows.length) { setTeach('err', '没有点位'); return; }
  teach.rows = rows;
  if (teach.idx >= rows.length) teach.idx = 0;
  // 单步走快照通道：必须先取消轨迹，否则这些电机归轨迹管，快照会被挡下。
  cancelTrajectory();
  const row = rows[teach.idx];
  let res = {ok: true, msg: '空点位（不下发）'};
  if (row.targets.length) res = sendTargets(row.targets);
  setTeach(res.ok ? 'ok' : 'err', `单步 ${teach.idx + 1}/${rows.length}: ${res.msg}`);
  teach.idx++;
  if (teach.idx >= rows.length) teach.idx = 0;
}

function teachRecordCmd() {
  const {targets, error} = readCmdTargets();
  if (error) { setTeach('err', error); return; }
  if (!targets.length) { setTeach('err', '命令表为空'); return; }
  const text = targets
    .map((x) => `${x.motor_id}:${x.position},${x.velocity},${x.effort}`).join(' ');
  teachAddRow(1000, text);
  setTeach('ok', '已记录当前命令为点位');
}

function teachExport() {
  const {rows, error} = teachReadRows();
  if (error) { setTeach('err', error); return; }
  download('teach-' + Date.now() + '.json', JSON.stringify(rows, null, 2));
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
        const rows = JSON.parse(reader.result);
        if (!Array.isArray(rows)) throw new Error('不是数组');
        $('teachRows').innerHTML = '';
        for (const r of rows) {
          const text = (r.targets || [])
            .map((x) => `${x.motor_id}:${x.position},${x.velocity},${x.effort}`).join(' ');
          teachAddRow(r.wait || 1000, text);
        }
        setTeach('ok', `导入 ${rows.length} 个点位`);
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
// enable 与 MIT 增益**不在参数树里**：它们是动态量（ARD §19.4 禁止进 100Hz
// 快照）。固件侧是稀疏语义——只影响下发的那些电机，其余保持原状。
// 代数必须严格前进才受理「从未武装到武装」，挡的是「周期重发同一条策略」在
// 故障后自动复权；每次真正改变意图时自然 +1 即可。
let policyGeneration = 0;

function setPolResult(ok, text) {
  const el = $('polResult');
  el.className = 'result tiny' + (ok ? '' : ' warn');
  el.textContent = text;
}

function sendPolicyFromForm() {
  if (!state.connected) { setPolResult(false, '未连接 rosbridge'); return; }
  const id = parseInt($('polId').value, 10);
  const kp = parseFloat($('polKp').value);
  const kd = parseFloat($('polKd').value);
  if (!Number.isInteger(id) || id < 1) { setPolResult(false, 'ID 非法'); return; }
  if (!Number.isFinite(kp) || !Number.isFinite(kd) || kp < 0 || kd < 0) {
    setPolResult(false, 'kp/kd 必须是非负有限值');
    return;
  }
  policyGeneration = (policyGeneration + 1) >>> 0;
  seq = (seq + 1) >>> 0;
  pub('policy', {
    sync_time_ns: Date.now() * 1e6,
    request_id: seq,
    policy_generation: policyGeneration,
    count: 1,
    targets: [{motor_id: id, enabled: $('polEnable').checked, kp, kd}],
  });
  setPolResult(true, `已发 seq=${seq} gen=${policyGeneration}：电机 ${id} ` +
    ($('polEnable').checked ? '武装' : '失能') +
    '。受理/拒绝是异步的，看事件表的 CONFIG_RESULT');
}

// ---------- 参数页（rcl_interfaces 参数服务直连）----------
// 固件 rclc_parameter 为 low-mem 模式：每次 get/set 只能一个参数，
// 不支持 ros2 param dump 式批量 get；逐参数调用。
const PARAM_NODE = '/micro_ros_node';
// 注意：ROS 参数服务名是蛇形小写（类型名才是驼峰）：
//   list_parameters/get_parameters/set_parameters（含 describe_parameters 等）。
function paramServiceName(typeName) {
  return typeName.replace(/([a-z0-9])([A-Z])/g, '$1_$2').toLowerCase();
}
// 参数树只放静态量（限位/限速/限流）。enable/kp/kd 是动态量，走 /micro_ros_do/motor_policy
// 话题（固件起把这三项从参数面退役）。
const PARAM_NUM_FIELDS = ['pos_min', 'pos_max', 'vel_limit', 'torque_limit',
  'limit_protect'];
// 参数服务调用结果：id -> {enable, kp, ...}（字符串原值，NaN 为空）
const paramCache = new Map();

function paramService(service, request) {
  return new Promise((resolve, reject) => {
    const client = new ROSLIB.Service({
      ros, name: `${PARAM_NODE}/${paramServiceName(service)}`,
      serviceType: `rcl_interfaces/srv/${service}`,
    });
    client.callService(new ROSLIB.ServiceRequest(request), resolve,
      (e) => reject(new Error(String(e))));
  });
}

async function paramGet(name) {
  const res = await paramService('GetParameters', {names: [name]});
  const v = res.values[0];
  if (!v || v.type === 0) return null; // NOT_SET
  if (v.type === 1) return v.bool_value;
  if (v.type === 2) return v.integer_value;
  return v.double_value; // type 3
}

function paramSetValue(name, type, key, value) {
  const pv = {type};
  if (type === 1) pv.bool_value = !!value;
  else if (type === 2) pv.integer_value = value;
  else pv.double_value = value; // 3 = DOUBLE
  return paramService('SetParameters', {parameters: [{name, value: pv}]});
}

function setParamResult(ok, text) {
  const el = $('paramResult');
  el.className = 'result tiny ' + (ok ? 'ok' : 'err');
  el.textContent = text;
}

function renderParamRows() {
  const tbody = $('paramRows');
  const ids = [...paramCache.keys()].sort((a, b) => a - b);
  if (ids.length === 0) {
    tbody.innerHTML = '<tr class="empty"><td colspan="10">参数集为空（等待电机发现）</td></tr>';
    return;
  }
  tbody.innerHTML = ids.map((id) => {
    const v = paramCache.get(id);
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
    // 设备端原值提示：页面已知"工程单位 → 设备原值"的换算（设备对象就是 cnt / mA），
    // 直接写在输入框 title 里，省去对着 CAN 日志换算。速度的电机端 RPM 还要按减速比
    // 折算，页面拿不到该系数，故不给。
    const isCerb = row.topo?.protocol === 1;
    const raw = (f, value) => {
      if (!isCerb || !Number.isFinite(value)) return '';
      if (f === 'pos_min' || f === 'pos_max') {
        return `设备端 ${Math.round(value * 65536 / 360)} cnt`;
      }
      if (f === 'torque_limit') return `设备端 ${Math.round(value * 1000)} mA`;
      return '';
    };
    const cell = (f) => {
      const hint = raw(f, Number(v[f]));
      return `<input data-id="${id}" data-field="${f}" class="pnum" value="${fmtParam(v[f])}"` +
        (hint ? ` title="${hint}"` : '') + `>`;
    };
    // 保护开关：参数 1 = 保护生效（设备侧原值 0）。设备还没回读到该对象时
    // 固件给 NaN —— 这时勾选框禁用，不能让操作员以为自己看到了真值。
    const protect = Number(v['limit_protect']);
    const protectCell = Number.isFinite(protect)
      ? `<input type="checkbox" data-id="${id}" data-field="limit_protect"` +
        (protect === 1 ? ' checked' : '') +
        ' title="限位保护：勾选 = 设备侧看门狗 + 限位保护生效；取消 = 关闭' +
        '（该关节将没有任何限位在执行）。仅未运动时受理。">'
      : '<input type="checkbox" disabled title="设备还没回读到该对象">';
    return `<tr>
      <td class="num">${id}</td>
      <td>${model || '<span class="muted">--</span>'}</td>
      <td>${lifecyclePill(lc)}</td>
      <td>${src}</td>
      <td>${cell('pos_min')}</td><td>${cell('pos_max')}</td>
      <td>${cell('vel_limit')}</td><td>${cell('torque_limit')}</td>
      <td>${protectCell}</td>
      <td><button class="tiny-btn" data-refresh="${id}">回读</button>
        <button class="tiny-btn" data-zero="${id}" title="把该台当前位置设为设备零点（仅失能态受理）">设零</button>
        <button class="tiny-btn" data-clearfault="${id}" title="清除故障锁存（仅非运动态受理；身份冲突锁存不可解除）">清错</button></td>
    </tr>`;
  }).join('');
  tbody.querySelectorAll('input').forEach((inp) => {
    inp.addEventListener('input', () => { inp.dataset.dirty = '1'; });
    inp.addEventListener('change', () => { inp.dataset.dirty = '1'; });
  });
  tbody.querySelectorAll('[data-refresh]').forEach((btn) => {
    btn.onclick = () => refreshDeviceParams(parseInt(btn.dataset.refresh, 10));
  });
  tbody.querySelectorAll('[data-zero]').forEach((btn) => {
    btn.onclick = () => setMotorZero(parseInt(btn.dataset.zero, 10));
  });
  tbody.querySelectorAll('[data-clearfault]').forEach((btn) => {
    btn.onclick = () => clearMotorFault(parseInt(btn.dataset.clearfault, 10));
  });
  renderParamSummary();
}

// 位图 → 槽位名列表（槽位顺序即 DEV_SLOT_NAMES 顺序）。
function devSlotNames(mask) {
  return DEV_SLOT_NAMES.filter((_, i) => (mask & (1 << i)) !== 0);
}

// 恢复口径（MISMATCH 用）：槽 0..4 在参数面，写回对应项即可；只剩槽 5
// （0x2541 方向）没有参数项——它只能拿 cangaroo 或厂商上位机改回设备对象，
// 再重连重读。
function devRecoveryText(mask) {
  if ((mask & DEV_SLOT_EXTERNAL_MASK) !== 0) {
    return '写回对应参数项即可；方向(0x2541)没有参数项，须用 cangaroo 或厂商工具改回后重连重读';
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
    return `设备参数回读未完成，未收到的对象：${slots}（重连或再点「回读」重试）`;
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
      : (code === 41 ? '回读不成功' : '未完成');
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

// last_error 快照：总览条与失败文案用它把错误码翻译成人话。
async function refreshLastError() {
  try {
    const [code, motor] = await Promise.all(
      ['system.last_error', 'system.last_error_motor'].map(paramGet));
    state.lastError = Number(code) || 0;
    state.lastErrorMotor = Number(motor);
  } catch (e) { /* 读不到就保留旧值 */ }
  return state.lastError;
}

// 读取 = 读驱动板 + 读参数。参数以驱动板为准，所以先把在线台逐台回读一遍
// （拿到设备真值并触发核对结论），再取参数表——这样"读取"得到的永远是设备真值，
// 而且核对结论会随 DEV_INIT 事件带到页面上（指出哪几项不一致）。
async function loadParams({skipDeviceRead = false} = {}) {
  if (!state.connected) return;
  setParamResult(true, '读取中…');
  if (!skipDeviceRead) {
    const online = [...state.motors.entries()]
      .filter(([, r]) => (r.status?.online ?? r.topo?.online) === true)
      .map(([id]) => id)
      .sort((a, b) => a - b);
    if (online.length) {
      let done = 0, bad = 0;
      for (const id of online) {
        setParamResult(true, `读取中… 回读驱动板 ${id}（${done + 1}/${online.length}）`);
        const r = await requestDeviceRead(id, 3000);
        done++;
        if (!r.ok) bad++;
      }
      if (bad) {
        setParamResult(false, `已回读 ${done} 台，其中 ${bad} 台未通过核对（见下表「状态」与错误码）`);
      }
    }
  }
  try {
    const res = await paramService('ListParameters', {prefixes: [], depth: 1});
    const names = res.result.names.filter((n) => /^m\d+\./.test(n));
    paramCache.clear();
    for (const name of names) {
      const [, idStr, field] = name.match(/^m(\d+)\.(.+)$/);
      const id = parseInt(idStr, 10);
      if (!PARAM_NUM_FIELDS.includes(field)) continue;
      if (!paramCache.has(id)) paramCache.set(id, {});
      paramCache.get(id)[field] = await paramGet(name);
    }
    await refreshLastError();
    renderParamRows();
    setParamResult(true, `已读取 ${paramCache.size} 台电机的参数`);
  } catch (e) {
    setParamResult(false, '读取失败：' + e.message);
  }
}

async function applyParams() {
  if (!state.connected) return;
  const dirty = [...$('paramRows').querySelectorAll('input[data-dirty]')];
  if (dirty.length === 0) { setParamResult(true, '没有修改'); return; }
  setParamResult(true, `下发 ${dirty.length} 项中…`);
  let fail = 0;
  for (const inp of dirty) {
    const id = parseInt(inp.dataset.id, 10);
    const field = inp.dataset.field;
    const name = `m${id}.${field}`;
    try {
      let value;
      if (inp.type === 'checkbox') {
        value = inp.checked ? 1 : 0;
      } else {
        if (inp.value.trim() === '') { delete inp.dataset.dirty; continue; }
        value = parseFloat(inp.value);
        if (!Number.isFinite(value)) throw new Error(`${name} 数值非法`);
      }
      const type = 3; // double
      const r = await paramSetValue(name, type, field, value);
      if (!r.results[0].successful) {
        fail++;
        setParamResult(false, `${name} 被拒：${r.results[0].reason || '见 last_error'}`);
      }
      delete inp.dataset.dirty;
    } catch (e) {
      fail++;
      setParamResult(false, `${name} 失败：${e.message}`);
    }
  }
  // 读固件侧最近错误码（设备类写最终成败是异步的）。
  setTimeout(async () => {
    try {
      const [code, motor] = await Promise.all(
        ['system.last_error', 'system.last_error_motor'].map(paramGet));
      if (code) setParamResult(false, `完成，last_error=${code} motor=${motor}`);
      else if (!fail) setParamResult(true, '下发已受理（设备限值最终成败见 CONFIG_RESULT 事件）');
    } catch (e) { /* 忽略错误码读取 */ }
  }, 400);
}

async function saveParamsToFlash() {
  if (!state.connected) return;
  try {
    await paramSetValue('system.save', 1, 'save', true);
    setParamResult(true, '保存请求已提交（Flash 写入约 0.5s）');
  } catch (e) {
    setParamResult(false, '保存失败：' + e.message);
  }
}

// 设备参数回读：固件侧是异步 SDO 事务（严格一问一答，4 个对象逐条读，最坏
// 含超时重试要几秒）。完成判据用只读的 system.refresh_seq（单调递增）——只看
// last_error(_motor) 分不清"这一轮"和"上一轮留下的"，会把没读当成成功。
// 请求一次设备参数回读并等它收尾。返回 {ok, code}：
//  code=0 一致；40/41 见内核；-1 固件未受理；-2 服务调用失败；-3 超时未收尾。
async function requestDeviceRead(id, timeoutMs) {
  const budget = timeoutMs || 9000;
  let seqBefore = 0;
  try {
    seqBefore = Number(await paramGet('system.refresh_seq')) || 0;
  } catch (e) { /* 读不到就按 0 处理，靠超时兜底 */ }
  try {
    const r = await paramSetValue('system.refresh', 2, 'refresh', id);
    if (r.results && r.results[0] && !r.results[0].successful) {
      return {ok: false, code: -1, reason: r.results[0].reason};
    }
  } catch (e) {
    return {ok: false, code: -2, reason: e.message};
  }
  const rounds = Math.max(1, Math.ceil(budget / 150));
  for (let i = 0; i < rounds; i++) {
    await new Promise((r) => setTimeout(r, 150));
    let seq;
    try {
      seq = Number(await paramGet('system.refresh_seq'));
    } catch (e) {
      continue;
    }
    if (seq === seqBefore) continue; // 未收尾
    await refreshLastError();
    return {ok: state.lastError === 0, code: state.lastError};
  }
  return {ok: false, code: -3, timeout: true};
}

async function refreshDeviceParams(id) {
  if (!state.connected) return;
  setParamResult(true, `电机 ${id} 设备参数回读已请求，等待驱动板应答…`);

  const r = await requestDeviceRead(id);
  if (r.code === -1) {
    setParamResult(false, `固件未受理 system.refresh（${r.reason || '见 last_error'}）`);
  } else if (r.code === -2) {
    setParamResult(false, '回读请求失败：' + r.reason);
  } else if (r.code === -3) {
    setParamResult(false, `电机 ${id} 回读仍未收尾（超时）：固件最近错误码 ` +
      `${state.lastError}（电机 ${state.lastErrorMotor}）` +
      (state.lastError === 11 ? '——11 是「提交失败」，请求在 mailbox 被拒' : ''));
  } else if (Number(state.lastErrorMotor) !== id) {
    setParamResult(true, `电机 ${id} 回读已结束（固件最近错误码 ${state.lastError} 属于电机 ${state.lastErrorMotor}）`);
  } else if (r.code === 0) {
    setParamResult(true, `电机 ${id} 设备参数已从驱动板读回（驱动板真值）`);
  } else if (r.code === 40) {
    const info = state.devInit.get(id);
    const slots = info && info.mask
      ? '（不一致项：' + devSlotNames(info.mask).join('、') + '）'
      : '';
    const recovery = info && info.mask ? devRecoveryText(info.mask) : '写回对应项即可恢复';
    setParamResult(false, `电机 ${id} 参数不一致${slots}：固件记录与驱动板不符 → 该台未初始化，` +
      recovery);
  } else if (r.code === 41) {
    const info = state.devInit.get(id);
    const missing = info && info.mask
      ? '（未收到的对象：' + devSlotNames(info.mask).join('、') + '）'
      : '';
    setParamResult(false, `电机 ${id} 回读不成功（last_error=41）${missing}：驱动板未应答。` +
      '该台初始化未完成，不可使能——重连或再点一次「回读」');
  } else {
    setParamResult(false, `电机 ${id} 回读失败：last_error=${r.code}`);
  }
  await loadParams({skipDeviceRead: true});
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

  let seqBefore = 0;
  try {
    seqBefore = Number(await paramGet('system.set_zero_seq')) || 0;
  } catch (e) { /* 读不到就靠超时兜底 */ }
  try {
    const r = await paramSetValue('system.set_zero', 2, 'set_zero', id);
    if (r.results && r.results[0] && !r.results[0].successful) {
      // 固件侧被拒（例如未失能）或该版本根本没有这个参数——直接说清楚，
      // 不要落到"未收尾"那种含糊超时。
      setParamResult(false, `固件未受理 system.set_zero（${r.results[0].reason || '见 last_error'}）：` +
        '该台可能未失能，或固件不是含「设零点」的版本');
      return;
    }
  } catch (e) {
    setParamResult(false, '设零点请求失败：' + e.message + '（固件是否已刷入含「设零点」的版本？）');
    return;
  }
  setParamResult(true, `电机 ${id} 设零点已请求…`);
  for (let i = 0; i < 40; i++) {
    await new Promise((r) => setTimeout(r, 150));
    let seq;
    try {
      seq = Number(await paramGet('system.set_zero_seq'));
    } catch (e) {
      continue;
    }
    if (seq === seqBefore) continue; // 事务未收尾

    let code = 0, motor = -1;
    try {
      [code, motor] = await Promise.all(
        ['system.last_error', 'system.last_error_motor'].map(paramGet));
    } catch (e) { /* 读不到结论码：按成功兜底 */ }
    const c = Number(code);
    if (Number(motor) === id && c !== 0) {
      setParamResult(false, `电机 ${id} 设零点失败：last_error=${c}` +
        (c === 2 ? '（参数被拒——该台在运行中？厂家要求失能状态设置）' : ''));
    } else {
      setParamResult(true, `电机 ${id} 已设零点：当前姿态即零点` +
        '——设备内限位随零点移动，请重新确认限位与机械行程');
    }
    await loadParams();
    return;
  }
  // 超时也要给出固件侧的错误码：请求可能在 mailbox/内核被拒（例如 FIFO 门、
  // 参数槽、kind 校验），那时序号永远不动，只看"未收尾"查不出原因。
  let code = 0, motor = -1;
  try {
    [code, motor] = await Promise.all(
      ['system.last_error', 'system.last_error_motor'].map(paramGet));
  } catch (e) { /* 读不到就算了 */ }
  setParamResult(false, `电机 ${id} 设零点未收尾（已等 6 秒）：固件最近错误码 ` +
    `${Number(code)}（电机 ${Number(motor)}）` +
    (Number(code) === 11 ? '——11 是「提交失败」，请求在 mailbox 被拒' : '') +
    (Number(code) === 0 ? '——错误码为 0 说明固件可能根本没有该参数（版本旧？）' : ''));
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

  let seqBefore = 0;
  try { seqBefore = Number(await paramGet('system.clear_fault_seq')) || 0; } catch (e) { /* 靠超时兜底 */ }
  try {
    const r = await paramSetValue('system.clear_fault', 2, 'clear_fault', id);
    if (r.results && r.results[0] && !r.results[0].successful) {
      setParamResult(false, `固件未受理 system.clear_fault（${r.results[0].reason || '见 last_error'}）：` +
        '该台可能在运动中，或固件不是含「故障清除」的版本');
      return;
    }
  } catch (e) {
    setParamResult(false, '故障清除请求失败：' + e.message + '（固件是否已刷入含「故障清除」的版本？）');
    return;
  }
  setParamResult(true, `电机 ${id} 故障清除已请求…`);
  for (let i = 0; i < 40; i++) {
    await new Promise((r) => setTimeout(r, 150));
    let seq;
    try { seq = Number(await paramGet('system.clear_fault_seq')); } catch (e) { continue; }
    if (seq === seqBefore) continue; // 事务未收尾

    let code = 0, motor = -1;
    try {
      [code, motor] = await Promise.all(
        ['system.last_error', 'system.last_error_motor'].map(paramGet));
    } catch (e) { /* 读不到结论码：按成功兜底 */ }
    const c = Number(code);
    if (Number(motor) === id && c !== 0) {
      setParamResult(false, `电机 ${id} 故障清除未成功：last_error=${c}` +
        '（身份冲突锁存不可解除、该台在运动中、或该协议无清错手段，都会被拒）');
    } else {
      setParamResult(true, `电机 ${id} 故障清除已受理：厂家故障需设备状态帧回报 ` +
        'fault_active=false 才收敛到 READY_DISABLED，届时再显式使能');
    }
    await loadParams();
    return;
  }
  setParamResult(false, `电机 ${id} 故障清除未收尾（已等 6 秒）：` +
    '看 system.last_error 与 motor_events 定位是哪一层拒的');
}

function switchTab(page) {
  document.querySelectorAll('.tab').forEach((b) =>
    b.classList.toggle('on', b.dataset.page === page));
  document.querySelectorAll('.page').forEach((p) =>
    p.classList.toggle('on', p.id === 'page-' + page));
}

// ---------- 初始化 ----------
function init() {
  $('rosUrl').value = (location.protocol === 'https:' ? 'wss://' : 'ws://')
    + (location.hostname || 'localhost') + ':9090';
  $('btnConnect').onclick = connect;
  $('btnDisconnect').onclick = disconnect;
  $('rosUrl').addEventListener('keydown', (e) => { if (e.key === 'Enter') connect(); });

  document.querySelectorAll('.tab').forEach((b) => {
    b.onclick = () => switchTab(b.dataset.page);
  });

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
  $('btnPolSend').onclick = sendPolicyFromForm;
  $('btnParamApply').onclick = applyParams;
  $('btnParamSave').onclick = saveParamsToFlash;

  addCmdRow(1, 0, 0, 0);
  teachAddRow(1000, '1:0,0,0');
}

init();
})();
