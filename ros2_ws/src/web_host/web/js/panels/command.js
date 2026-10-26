// 页面只下设定值，command_streamer 以 100 Hz 持续发送完整快照（ADR 0014）。
// 浏览器后台定时器会降至 1 Hz，不能承载固件 30 ms 软停 / 100 ms 失能的节拍。
// 「下发」会持续保持；「停止」以空目标列表软停全部电机，清空表格不停车。
import { store, MODEL, markDirty, DIRTY, syncWriteButtons } from '../core/store.js';
import { $, LIFECYCLE } from '../core/format.js';
import { controlTargetsError, describeMotorState } from '../core/motor_state.js';
import { limitsError } from '../core/host_limits.js';
import { pub, isConnected } from '../core/ros.js';
import { rawCouplingError } from '../core/joint_control.js';
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
  setCmdResult(res.ok, `已发 ${res.msg}` +
    (blocked.length ? ` ｜ ⚠ ${blocked.join('、')} 不会动（未初始化或离线）` : ''));
  return res.ok;
}

// 停止台架：空目标列表在完整快照语义下是「全部软停」这条**真消息**，显式停止不靠
// 断流实现（固件收到后发阻尼帧，100 ms 后失能）。
export function sendStop() {
  if (!isConnected()) { setCmdResult(false, '未连接 rosbridge'); return false; }
  sequence = (sequence + 1) >>> 0;
  const cancel={sync_time_ns:Date.now()*1e6,sequence,trajectory_id:sequence,
    segment_index:0,segment_count:1,count:0,target_count:0,target_ids:[],points:[]};
  pub('convertedTrajectory', cancel);
  pub('trajectory', cancel);
  pub('setpoint', {
    sync_time_ns: Date.now() * 1e6,
    sequence,
    count: 0,
    targets: [],
  });
  setCmdResult(true, `已发全部软停（seq=${sequence}）`);
  return true;
}

// 发送完整快照（命令表与动作共用）；记录历史。
export function sendTargets(targets) {
  if (!isConnected()) return {ok: false, msg: '未连接 rosbridge'};
  if (!targets.length) return {ok: false, msg: '快照为空'};
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

// 命令节拍节点的状态：**命令当前实际发往哪里**、有没有在发。
// 改道（直连固件 ↔ 经叠加节点）是服务端做的，页面切不了，所以这一行是操作者核对
// 「开关亮着但接线没切」的唯一入口。
const REASON_TEXT = {
  no_setpoint: '未收到设定值',
  no_client: '页面未连接（自动停流）',
  ok: '保持中',
};

function renderCommandStatus() {
  const el = $('cmdStatus');
  if (!el) return;
  const {data, last} = store.commandStatus;
  if (!data || !last) {
    el.textContent = '命令节拍：未上报（command_streamer 没在跑？）';
    el.className = 'result tiny err';
    return;
  }
  const parts = [`发往 ${data.output_topic}`];
  parts.push(REASON_TEXT[data.reason] || data.reason || '--');
  if (data.clients === null || data.clients === undefined) {
    parts.push('无 /client_count');
  } else {
    parts.push(`${data.clients} 个客户端`);
  }
  if (data.reject) parts.push(`⚠ 设定值被拒：${data.reject}`);
  el.textContent = `命令节拍：${parts.join(' · ')}`;
  el.className = `result tiny${data.reject ? ' err' : ''}`;
}

export function mount(root, ctx) {
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
    // 断线不再需要页面做任何事：rosbridge 一断，/client_count 归零，主机侧的
    // command_streamer 自己停流，固件 100 ms 内失能——失效方向是安全的。
    if (dirty.has(DIRTY.COMMAND) || dirty.has(DIRTY.CONN)) renderCommandStatus();
  };
  render(new Set());
  // 切屏只释放界面资源，主机目标由显式停止操作释放。
  return { render, unmount(){jointControl.unmount();} };
}

// 供 teach 面板「记录当前命令」用（面板依赖面板的唯一一处：动作记录的正是下发
// 快照里的目标值）。command 面板未挂载时 #cmdRows 不在 DOM 上，返回空数组而不是抛错。
export function currentTargets() {
  return $('cmdRows') ? readCmdTargets() : [];
}
