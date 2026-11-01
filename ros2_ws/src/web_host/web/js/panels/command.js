// 页面只提交低频 P/V/E 目标，由 manual_controller 在 ros2_control 周期执行。
// 显式停止停用运动控制器；清空表格不会停止。
import { store, MODEL, markDirty, DIRTY, syncWriteButtons } from '../core/store.js';
import { $, LIFECYCLE } from '../core/format.js';
import { controlTargetsError, describeMotorState } from '../core/motor_state.js';
import { limitsError } from '../core/host_limits.js';
import { pub, isConnected } from '../core/ros.js';
import { stopPlayback } from './actionLibrary.js';
import { controlSessionError, rawCouplingError } from '../core/joint_control.js';
import { mount as mountJointControl } from '../components/jointControl.js';

// sequence 仅作日志关联；以页面启动时间降低跨会话复用的概率。
let sequence = (Date.now() & 0x7fffffff) >>> 0;

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
  if (!el) return;
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

// JTC 成功结果不替代电机就绪检查；武装前允许 READY_DISABLED。
export function playbackReadinessError(ids) {
  const blocked = [];
  for (const id of [...new Set(ids)].sort((a, b) => a - b)) {
    const row = store.motors.get(id);
    if (!row) {
      blocked.push(`m${id}(未发现)`);
      continue;
    }
    if (store.controlStates?.seen) {
      const view=describeMotorState(store,id);
      const control=store.controlStates.byId.get(id);
      if (!view.online || view.connection!=='在线' || view.fault
          || ![2,3].includes(control?.control_lifecycle)) blocked.push(`m${id}(${view.connection} · ${view.lifecycle} · ${view.health})`);
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
  const blocked = noteBlockedTargets(targets);
  setCmdResult(res.ok, `${res.ok ? '已提交' : '未提交'} ${res.msg}` +
    (blocked.length ? ` ｜ ⚠ ${blocked.join('、')} 不会动（未初始化或离线）` : ''));
  return res.ok;
}

// JTC cancel 只保持位置；显式全部软停还必须停用运动控制器。
export function sendStop() {
  stopPlayback();
  if (!isConnected()) { setCmdResult(false, '未连接 rosbridge'); return false; }
  const sent = pub('stop', {});
  setCmdResult(sent, sent ? '已请求全部软停（停用运动控制器），请核对设备反馈' : '停止请求未发送');
  return sent;
}

// 发送完整快照（命令表与动作共用）；记录历史。
export function sendTargets(targets) {
  if (!isConnected()) return {ok: false, msg: '未连接 rosbridge'};
  if (!targets.length) return {ok: false, msg: '快照为空'};
  const session=controlSessionError(store,MODEL);
  if (session) return {ok:false,msg:session};
  const coupled=rawCouplingError(store,MODEL,targets.map(t=>t.motor_id));
  if (coupled) return {ok:false,msg:coupled};
  const authority = controlTargetsError(store,targets.map(t=>t.motor_id));
  if (authority) return {ok:false,msg:authority};
  const blocked = limitsError(targets, MODEL);
  if (blocked) return {ok: false, msg: blocked};
  sequence = (sequence + 1) >>> 0;
  pub('setpoint', {
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

// 摘要（搬 renderFoldSummaries 的 sumSend 段）：目标台数。
function updateSummary() {
  const rows = $('cmdRows');
  if (!rows) return;
  const n = rows.querySelectorAll('tr').length;
  $('sumSend').textContent = `${n} 台目标`;
}

function renderCommandStatus() {
  const el = $('cmdStatus');
  if (!el) return;
  const {data, last} = store.commandStatus;
  const age = Date.now()-last;
  if (!data || !last || age < 0 || age > 2500) {
    el.textContent = 'ros2_control：状态未上报或已过期';
    el.className = 'result tiny err';
    return;
  }
  const parts = [`${data.model} · 模式 ${data.mode}`, `输出 ${data.output_topic}`,
    `JTC ${data.trajectory_output_topic}`, data.reason || '--'];
  const state = store.traj.state;
  if (state) parts.push(`JTC #${state.trajectory_id}：${state.active ? '执行中'
    : state.completed ? '成功' : '未成功（取消/拒绝/失败）'}`);
  if (data.reject) parts.push(`错误：${data.reject}`);
  if (store.transmissionStatus.data?.last_error) parts.push(store.transmissionStatus.data.last_error);
  el.textContent = `ros2_control：${parts.join(' · ')}`;
  el.className = `result tiny${data.reject ? ' err' : ''}`;
}

export function mount(root, ctx) {
  const summary = root.querySelector('.sendbar > summary');
  if (summary) summary.title = '/control/setpoint → ros2_control manual_controller';
  $('btnSendOnce').title = '提交 P/V/E 目标，由 manual_controller 持续执行';
  $('btnStopCmd').title = '全部软停：停用运动控制器，不自动恢复旧目标';
  const jointControl=mountJointControl(root,{onStop:sendStop});
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
  // 清空表格**不停台架**：目标还在主机侧保持（要停请按「停止」）。
  $('btnClearRows').onclick = () => { $('cmdRows').innerHTML = ''; updateSummary(); };
  $('btnSendOnce').onclick = () => sendSnapshot();
  $('btnStopCmd').onclick = () => sendStop();

  addCmdRow(1, 0, 0, 0);      // 等价旧 UI 的初始行

  const render = (dirty) => {
    jointControl.render();
    syncWriteButtons(['btnSendOnce', 'btnStopCmd'], isConnected());
    if (dirty.has(DIRTY.HISTORY)) updateSummary();
    if ([DIRTY.COMMAND, DIRTY.CONN, DIRTY.TRAJ, DIRTY.STATUS].some(key => dirty.has(key))) renderCommandStatus();
  };
  render(new Set([DIRTY.COMMAND]));
  const ageTimer = setInterval(renderCommandStatus, 1000);
  // 切屏只释放界面资源，主机目标由显式停止操作释放。
  return { render, unmount(){clearInterval(ageTimer);jointControl.unmount();} };
}

// 供 teach 面板「记录当前命令」用（面板依赖面板的唯一一处：动作记录的正是下发
// 快照里的目标值）。command 面板未挂载时 #cmdRows 不在 DOM 上，返回空数组而不是抛错。
export function currentTargets() {
  return $('cmdRows') ? readCmdTargets() : [];
}
