// 下发面板：命令快照（/micro_ros_do/motor_command_array）与 100Hz 连发。
// 搬自重构前 app.js 的控制段；seq/streamTimer 收进本模块（不再跨面板共享）。
import { store, markDirty, DIRTY } from '../core/store.js';
import { $, STREAM_PERIOD_MS, LIFECYCLE } from '../core/format.js';
import { pub, isConnected } from '../core/ros.js';

// sequence 仅作日志关联；用页面启动时间降低刷新页面后复用旧 trajectory_id
// 的概率（固件状态回执是低频 best-effort，不能接受旧回执误推进分段）。
let sequence = (Date.now() & 0x7fffffff) >>> 0;
let streamTimer = null;

function addCmdRow(id = 1, pos = 0, vel = 0, eff = 0) {
  const tr = document.createElement('tr');
  tr.innerHTML =
    `<td><input type="number" class="c-id" value="${id}" min="1" max="65535" step="1"></td>`
    + `<td><input type="number" class="c-pos" value="${pos}" step="0.01"></td>`
    + `<td><input type="number" class="c-vel" value="${vel}" step="0.01"></td>`
    + `<td><input type="number" class="c-eff" value="${eff}" step="0.01"></td>`
    + `<td><button class="c-del secondary">删</button></td>`;
  tr.querySelector('.c-del').onclick = () => { tr.remove(); updateSummary(); };
  $('cmdRows').appendChild(tr);
  updateSummary();
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
    const row = store.motors.get(t.motor_id) || {};
    const online = row.status?.online ?? row.topo?.online;
    const lc = row.status?.lifecycle ?? row.topo?.lifecycle;
    if (online === false) blocked.push(`${t.motor_id}(离线)`);
    else if (lc === 7) blocked.push(`${t.motor_id}(未初始化)`);
    else if (lc === undefined) blocked.push(`${t.motor_id}(未上报状态)`);
  }
  return blocked;
}

// 动作不是普通的“发布成功”操作：trajectory_state 的 accepted/completed 只代表
// MCU 轨迹 mailbox 的状态，不能替代实际电机在线和生命周期检查。播放前必须确认
// 每个目标都处于 READY_DISABLED 或 ACTIVE；否则页面会出现“播放完成”但机构不动。
export function playbackReadinessError(ids) {
  const blocked = [];
  for (const id of [...new Set(ids)].sort((a, b) => a - b)) {
    const row = store.motors.get(id);
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
    ? `电机未就绪：${blocked.join('、')}；请先连接/启动仿真，等目标电机到 `
      + `READY_DISABLED（或 ACTIVE）后重试`
    : '';
}

function sendSnapshot() {
  if (!isConnected()) { setCmdResult(false, '未连接 rosbridge'); return false; }
  const {targets, error} = readCmdTargets();
  if (error) { setCmdResult(false, error); return false; }
  const res = sendTargets(targets);
  // 连发时只在启动那一拍提示一次，避免 100Hz 刷 DOM。
  const blocked = streamTimer ? [] : noteBlockedTargets(targets);
  setCmdResult(res.ok, `已发 ${res.msg}` +
    (blocked.length ? ` ｜ ⚠ ${blocked.join('、')} 不会动（未初始化或离线）` : ''));
  return res.ok;
}

// 发送完整快照（命令表与动作共用）；记录历史。
export function sendTargets(targets) {
  if (!isConnected()) return {ok: false, msg: '未连接 rosbridge'};
  if (!targets.length) return {ok: false, msg: '快照为空'};
  sequence = (sequence + 1) >>> 0;
  pub('command', {
    // uint64：rosbridge 2.x 入站要求 JSON number（float64 会丢失末几位
    // 精度；该字段仅用于跨系统日志对齐，freshness 以 MCU 本地 tick 为准）。
    sync_time_ns: Date.now() * 1e6,
    sequence,
    count: targets.length,
    targets,
  });
  store.history.unshift({
    t: Date.now(),
    seq: sequence,
    targets: targets.map((x) => ({...x})),
  });
  if (store.history.length > 500) store.history.pop();
  markDirty(DIRTY.HISTORY);
  return {ok: true, msg: `seq=${sequence}，${targets.length} 台`};
}

function startStream() {
  if (streamTimer) return;
  if (!sendSnapshot()) { $('chkStream').checked = false; return; }
  streamTimer = setInterval(sendSnapshot, STREAM_PERIOD_MS);
  markDirty(DIRTY.HISTORY);
}
function stopStream() {
  if (streamTimer) { clearInterval(streamTimer); streamTimer = null; markDirty(DIRTY.HISTORY); }
  const chk = $('chkStream');
  if (chk) chk.checked = false;
}

// 摘要（搬 renderFoldSummaries 的 sumSend 段）：目标台数与连发状态。
function updateSummary() {
  const rows = $('cmdRows');
  if (!rows) return;
  const n = rows.querySelectorAll('tr').length;
  $('sumSend').textContent = `${n} 台目标` + (streamTimer ? ' · 连发中' : '');
}

export function mount(root, ctx) {
  $('btnAddRow').onclick = () => addCmdRow();
  $('btnFillOnline').onclick = () => {
    // 只填拓扑中在线的电机，目标取当前实测位置（保持原位语义）；
    // 未收到运动状态时位置填 0 并给出提示色由用户核对后再发。
    const online = [];
    for (const [id, row] of store.motors) {
      if (row.topo && row.topo.online) online.push([id, row.motion ? row.motion.position : 0]);
    }
    if (online.length === 0) { alert('拓扑中暂无在线电机'); return; }
    $('cmdRows').innerHTML = '';
    for (const [id, pos] of online.sort((a, b) => a[0] - b[0])) addCmdRow(id, pos, 0, 0);
  };
  $('btnClearRows').onclick = () => { stopStream(); $('cmdRows').innerHTML = ''; updateSummary(); };
  $('btnSendOnce').onclick = () => { stopStream(); sendSnapshot(); };
  $('chkStream').onchange = (e) => e.target.checked ? startStream() : stopStream();

  addCmdRow(1, 0, 0, 0);      // 等价旧 UI 的初始行

  const render = (dirty) => {
    if (dirty.has(DIRTY.CONN) && !isConnected()) stopStream();  // 断线即停连发（旧 closeSocket 行为）
    if (dirty.has(DIRTY.HISTORY)) updateSummary();
  };
  render(new Set());
  // 连发定时器与 unmount：切屏必须停连发——否则离开运行屏后 100 Hz 还在发，
  // 而按钮已经不在 DOM 上、用户没法停。
  return {
    render,
    unmount() { stopStream(); },
  };
}

// 供 teach 面板「记录当前命令」用（面板依赖面板的唯一一处：动作记录的正是下发
// 快照里的目标值）。command 面板未挂载时 #cmdRows 不在 DOM 上，返回空数组而不是抛错。
export function currentTargets() {
  return $('cmdRows') ? readCmdTargets() : [];
}
