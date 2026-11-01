// 参数读写事务层：设备屏按钮与「连接后自动读取」共用。
// 这些函数既被设备屏的按钮调用，也被连接建立后的自动读取调用（见 main.js）——
// 自动读取不能住在面板里：store.paramCache 是使能面板列电机列表的依据，
// 切到「运行」屏时设备屏并未挂载。
// 搬迁自旧 app.js 的参数页段；DOM 只剩「读当前表格里哪些输入被改过」这层查询。
import { store, markDirty, DIRTY, paramReadState } from './store.js';
import { $, PARAM_FIELD, SYS_CMD, MOTOR_ID_NONE, LIFECYCLE, fmtParam,
  devSlotNames, devRecoveryText } from './format.js';
import { pub, isConnected, waitDevParamSeq, waitConfigResult, getConnectionGeneration } from './ros.js';

// 写事务的关联 id。两个高位都不能碰：bit31 是内核身份读取的命名空间，bit30 是
// 固件自发脉冲的命名空间（置位会被本地拒）。0 表示「固件代分配」，所以绕开它。
let paramReq = (Date.now() & 0x3fffffff) >>> 0;
function nextParamReq() {
  paramReq = (paramReq + 1) & 0x3fffffff;
  return paramReq === 0 ? (paramReq = 1) : paramReq;
}

const CONFIG_STATUS = {PENDING: 0, SUCCEEDED: 1, FAILED: 2, BUSY: 3, CONFLICT: 4};

async function waitRequest(requestId, timeoutMs, label) {
  const outcome = await waitConfigResult(requestId, timeoutMs);
  if (outcome.outcome !== 'terminal') {
    return {ok: false, outcome: outcome.outcome, text: `${label}：${{
      timeout: '等待设备终态超时（设备可能仍在处理，不能据此判断已回滚）',
      disconnected: '连接中断，未收到本次请求终态',
      session_changed: '设备会话已更换，旧请求等待失效',
      invalid_request: 'request_id 无效',
    }[outcome.outcome] || '请求未完成'}`};
  }
  const result = outcome.result;
  if (Number(result.status) === CONFIG_STATUS.SUCCEEDED) {
    return {ok: true, result, text: `${label}成功（完成 ${result.completed_targets} 项）`};
  }
  const statusText = Number(result.status) === CONFIG_STATUS.BUSY ? '设备忙' :
    Number(result.status) === CONFIG_STATUS.CONFLICT ? 'request_id 冲突' : '设备拒绝';
  return {ok: false, outcome: 'failed', result,
    text: `${label}${statusText}：error_code=${result.error_code}，` +
      `失败 ${result.failed_targets} 项 / 完成 ${result.completed_targets} 项`};
}

// 结果文本进 store 不进 DOM：同一个事务的结果可能来自任一屏的按钮。
export function writeResult(key, ok, text) {
  store.results.set(key, {ok, text, t: Date.now()});
  markDirty(DIRTY.PARAMS);
}
export function setParamResult(ok, text) { writeResult('paramResult', ok, text); }

// 下发一条稀疏写（整条受理或整条拒绝）。返回 request_id，失败返回 0。
export function sendParamSet(targets) {
  if (!isConnected()) { setParamResult(false, '未连接 rosbridge'); return 0; }
  if (!targets.length) { setParamResult(false, '没有要下发的改动'); return 0; }
  // [<=16] 是内核 CONTROL_MAILBOX_CONFIG_QUEUE_CAPACITY 的映射，不是带宽限制：
  // 每个目标一次 motor_rt_submit_config 提交。调用方负责分块（见 chunkedParamSet）。
  if (targets.length > 16) { setParamResult(false, '内部错误：单条超过 16 个目标'); return 0; }
  const request_id = nextParamReq();
  const sent = pub('paramSet', {
    sync_time_ns: Date.now() * 1e6, // uint64：rosbridge 入站要 JSON number
    request_id, count: targets.length, targets,
  });
  return sent ? request_id : 0;
}

// 按台切块：4 台 × 6 字段 = 24 > 16，必须拆。同一台**不得跨块**——位置限位成对的
// 两边落在两块里就是两条各缺一边的消息，两条都被整条拒。
export function chunkedParamSet(byMotor) {
  const chunks = [];
  let cur = [];
  for (const targets of byMotor.values()) {
    if (cur.length + targets.length > 16) { chunks.push(cur); cur = []; }
    cur.push(...targets);
  }
  if (cur.length) chunks.push(cur);
  return chunks;
}

export function sendSystemCommand(command, motorId) {
  if (!isConnected()) { setParamResult(false, '未连接 rosbridge'); return 0; }
  const request_id = nextParamReq();
  const sent = pub('sysCmd', {
    sync_time_ns: Date.now() * 1e6, request_id, command,
    motor_id: motorId === undefined ? MOTOR_ID_NONE : motorId,
  });
  return sent ? request_id : 0;
}

// 兼容旧调用者的 system_status 序号等待。新配置事务必须使用 waitConfigResult，
// 因为这些序号是全局计数，不能关联到某一次请求。
export async function waitSysSeq(key, baseline, timeoutMs) {
  const deadline = Date.now() + timeoutMs;
  while (Date.now() < deadline) {
    if (!store.sysStatus.seen) return -2;      // 收不到 system_status
    if (store.sysStatus[key] > baseline) return 0;
    if (!store.conn.connected) return -2;
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
export function lastErrorText() {
  const e = store.lastError;
  if (!e) return '无错误';
  return (SYS_ERR_TEXT[e] || `错误码 ${e}`) +
    (store.lastErrorMotor >= 0 ? `（m${store.lastErrorMotor}）` : '');
}

// 表里有没有还没下发的改动。device_param_report 按需发帧（REPORT 命令 +
// 写事务收尾），每帧重建 innerHTML 会把操作员正在敲的值抹掉——有 dirty 就跳过重绘。
export function hasDirtyParamInput() {
  const tb = $('paramRows');
  return !!(tb && tb.querySelector('input[data-dirty]'));
}

// 读取 = 发 SystemCommand(REPORT)，固件把**当前手里的参数视图**立刻发一帧
// device_param_report（不触发 SDO、不访问设备）。电机在线且此前核对过，
// dev_* 就是驱动板真值；电机不在线/没核对，dev_* 为 NaN，页面呈现 rec_*
// 即驱动板 W25 参数表。帧到达后订阅回调自行渲染，这里只等帧、给结论。
export async function loadParams() {
  if (!isConnected()) return;
  const before = store.devParamSeq;
  const requestId = sendSystemCommand(SYS_CMD.REPORT, MOTOR_ID_NONE);
  if (!requestId) return;
  setParamResult(true, '读取中…');
  const request = await waitRequest(requestId, 6000, '参数读取请求');
  if (!request.ok) { setParamResult(false, request.text); return; }
  // REPORT 的 SUCCEEDED 只表示桥已 take 报告请求，必须再等实际数据帧。
  const rc = await waitDevParamSeq(before, 2000);
  if (rc === -2) {
    setParamResult(false, '读取失败：未连接 rosbridge');
    return;
  }
  if (rc === -3) {
    setParamResult(false, '读取超时：未收到 device_param_report——固件/agent 未就绪？');
    return;
  }
  const states = [...store.paramCache.keys()].map(id => ({id, ...paramReadState(id)}));
  const missing = states.filter(state => state.usable < 8);
  if (!states.length || missing.length) {
    setParamResult(false, `参数读取不完整：${states.length ? missing.map(state => `电机 ${state.id}（${state.usable}/8 项）`).join('、') : '没有电机参数'}。` +
      '请检查电机型号识别和设备回读；ID/T_max 需要新版固件与桥接支持。收到报告不代表已读到有效参数。');
    return;
  }
  const withDev = states.filter(state => state.device === 8).length;
  setParamResult(true, `已收到 ${states.length} 台有效参数：${withDev} 台完整驱动板回读，` +
    `${states.length - withDev} 台含固件记录回落值。`);
}

export async function refreshMotorParams() {
  if (!isConnected()) return;
  const ids = [...new Set([...store.motors.keys(), ...store.paramCache.keys()])].sort((a, b) => a - b);
  const id = ids.includes(store.selectedId) ? store.selectedId : ids[0];
  if (id === undefined) { setParamResult(false, '尚未发现电机，无法回读参数'); return; }
  const requestId = sendSystemCommand(SYS_CMD.REFRESH, id);
  if (!requestId) return;
  setParamResult(true, `正在回读电机 ${id} 的驱动板参数…`);
  const request = await waitRequest(requestId, 6000, '电机参数回读');
  if (!request.ok) {
    setParamResult(false, request.result?.error_code === 12
      ? `电机 ${id} 尚未识别型号或未绑定，固件拒绝回读（error_code=12）。请先检查设备身份读取与型号绑定。`
      : request.text);
    return;
  }
  await loadParams();
}

// 下发手改项。三条硬约束（固件 validate_param_set，整条受理或整条拒绝）：
//   1. count === targets.length
//   2. 位置限位必须**成对**在同一条消息里，且 min < max
//   3. ≤ 16 个 target（内核配置 FIFO 深度），故按台分块
// 违反任一条整条被拒（last_error=10）——所以能在这里判的先在这里判，
// 别让一个台的错配对把别的台的合法改动一起带走。
export async function applyParams() {
  if (!isConnected()) return;
  const tb = $('paramRows');
  if (!tb) return;
  const dirty = [...tb.querySelectorAll('input[data-dirty]')];
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
    if (slot === PARAM_FIELD.NODE_ID || slot === PARAM_FIELD.T_MAX) {
      const control = store.controlStates.byId.get(id);
      const motor = store.motors.get(id);
      if (control?.policy_enabled || control?.motion_enabled || [4, 5].includes(motor?.status?.lifecycle)) {
        setParamResult(false, `电机 ${id} 必须先失能才能修改 ID 或 T_max`); return;
      }
      const max = slot === PARAM_FIELD.NODE_ID ? 64 : 16777215;
      if (!Number.isInteger(value) || value < 1 || value > max) {
        setParamResult(false, `电机 ${id} 的 ${field} 必须是 1～${max} 的整数`); return;
      }
      if (slot === PARAM_FIELD.NODE_ID && value !== id &&
          (store.motors.has(value) || store.paramCache.has(value) ||
           [...store.paramCache.entries()].some(([other, cached]) => other !== id && cached.pick.node_id === value))) {
        setParamResult(false, `电机 ID ${value} 已占用或待生效`); return;
      }
    }
    note(id).set(slot, value);
  }

  // 成对补齐 + 本地判 min < max。
  for (const [id, fields] of byMotor) {
    const v = store.paramCache.get(id);
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

  const destinations = [...byMotor.values()].filter(fields => fields.has(PARAM_FIELD.NODE_ID))
    .map(fields => fields.get(PARAM_FIELD.NODE_ID));
  if (new Set(destinations).size !== destinations.length) {
    setParamResult(false, '多台电机的新 ID 重复，未下发'); return;
  }
  const motorTargets = new Map();
  for (const [id, fields] of byMotor) {
    motorTargets.set(id, [...fields].map(([field, value]) => ({motor_id: id, field, value})));
  }
  // 同一台设备只允许一个在途 SDO 事务；逐项等待，位置上下限保持同请求。
  const chunks = [];
  for (const targets of motorTargets.values()) {
    const positions = targets.filter(t => t.field <= PARAM_FIELD.POS_MAX);
    if (positions.length) chunks.push(positions);
    for (const target of targets.filter(t => t.field > PARAM_FIELD.POS_MAX)) chunks.push([target]);
  }
  let sent = 0;
  for (let i = 0; i < chunks.length; i++) {
    const req = sendParamSet(chunks[i]);
    if (!req) return; // sendParamSet 已写结果
    sent += chunks[i].length;
    const result = await waitRequest(req, 10000, `参数块 ${i + 1}/${chunks.length}`);
    if (!result.ok) { setParamResult(false, result.text); return; }
  }
  dirty.forEach((inp) => { delete inp.dataset.dirty; });
  markDirty(DIRTY.PARAMS);   // 重绘交给面板（dirty 标志已清，重建不会抹掉输入）
  setParamResult(true, `参数写入成功：${byMotor.size} 台 / ${sent} 项` +
    (chunks.length > 1 ? `（${chunks.length} 条请求均已完成）` : '') +
    (destinations.length ? '；ID 已写入并回读，重启电机后生效，当前通信地址保持不变。' : ''));
}

export async function saveParamsToFlash() {
  if (!isConnected()) return;
  // 本次 ConfigResult SUCCEEDED 表示设备与 W25 保存事务均已完成。system_status
  // 仅作页面状态展示，不能用它可能滞后的电平覆盖本次关联终态。
  // SAVE 只接受 MOTOR_ID_NONE；给具体电机 id 会被本地拒（last_error=10）。
  const requestId = sendSystemCommand(SYS_CMD.SAVE, MOTOR_ID_NONE);
  if (!requestId) return;
  setParamResult(true, '保存中…（写 W25，约 1–2s）');
  const result = await waitRequest(requestId, 15000, '保存请求');
  if (result.ok) {
    markDirty(DIRTY.PARAMS);
    setParamResult(true, `已保存到 W25（本次 ConfigResult 成功；system_status save_seq=${
      store.sysStatus.save_seq}）`);
  } else {
    setParamResult(false, `${result.text}——勿断电；不要自动重发 SAVE`);
  }
}

// 限位教学：把该台**当前关节位置**写进对应的限位槽。厂商手册要求"设置时电机失能、
// 掰到限位再设置"，内核也只受理未运动状态（device_limit_write_allowed），所以
// 先本地挡一道，再让固件的拒绝兜底（渲染与点击之间有竞态）。
// 写事务（写 + 读回核验）收尾后固件即时发一帧 device_param_report，
// 报告内核按该型号位置边界钳制后的真值，无需再单独发 REFRESH。
export async function setLimitFromPosition(id, side) {
  if (!isConnected()) { setParamResult(false, '未连接 rosbridge'); return; }
  const row = store.motors.get(id) || {};
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
  const v = store.paramCache.get(id);
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
  const before = store.devParamSeq;
  const requestId = sendParamSet([{motor_id: id, field: PARAM_FIELD.POS_MIN, value: lo},
                                  {motor_id: id, field: PARAM_FIELD.POS_MAX, value: hi}]);
  if (!requestId) return;
  setParamResult(true, `m${id} ${which}已下发（${pos.toFixed(4)} rad），写事务收尾自动刷新…`);
  const request = await waitRequest(requestId, 10000, `m${id} 限位写入`);
  if (!request.ok) { setParamResult(false, request.text); return; }
  const rc = await waitDevParamSeq(before, 6000);
  if (rc === 0) {
    const now = store.paramCache.get(id);
    setParamResult(true, `m${id} ${which}写事务已收尾：下限 ` +
      `${fmtParam(now ? now.pick.pos_min : NaN)} / 上限 ` +
      `${fmtParam(now ? now.pick.pos_max : NaN)} rad`);
  } else if (rc === -3) {
    setParamResult(true, `m${id} ${which}写入成功，但 device_param_report 回显超时`);
  } else {
    setParamResult(true, `m${id} ${which}写入成功；连接中断，参数回显尚未刷新`);
  }
}

// 设零点（设备对象 0x2531 写 1，字典表「设置电机输出端零位」）：
// 厂家要求**失能状态**下设置，运行中会被固件拒绝（不隐式失能）。
// 零点一变，设备里 0x607D 的限位也随之换了物理位置，须重新确认限位与机械行程。
// 是否掉电持久：字典未给"零位"对应的 0x2539 保存码，待实测确认。
export async function setMotorZero(id) {
  if (!isConnected()) return;
  const ok = window.confirm(
    `把电机 ${id} 的当前位置设为设备零点？\n\n` +
    '· 仅在该台失能时受理（运行中会被固件拒绝）\n' +
    '· 零点改变后设备内的位置限位也随之移动，请重新确认限位与机械行程\n' +
    '· 立即写入驱动器；是否掉电保存尚未实测确认');
  if (!ok) return;

  const requestId = sendSystemCommand(SYS_CMD.SET_ZERO, id);
  if (!requestId) return;
  setParamResult(true, `电机 ${id} 设零点已请求…`);
  const result = await waitRequest(requestId, 10000, `电机 ${id} 设零点`);
  if (!result.ok) { setParamResult(false, result.text); return; }
  setParamResult(true, `电机 ${id} 已设零点：当前姿态即零点` +
    '——设备内限位随零点移动，请重新确认限位与机械行程');
  await loadParams();
}

// 故障清除（固件参数 `system.clear_fault` = <电机 id>，脉冲；完成序号
// `system.clear_fault_seq`）。内核按**成因**区分：
//   · 身份冲突锁存（同 motor_id 跨总线/协议身份不符）刻意不可解除 → 拒绝；
//   · 设备参数未初始化（last_error=40/41）→ 走一次回读重比对，真不一致仍锁；
//   · 厂家故障 → 发厂家清错帧（CERB 的 MIT 控制字 bit5；Vaxor 无清错位 → 拒绝）。
// 仅非运动态受理；清完仍须显式勾 enable + 新的完整快照/轨迹才可能回运动
// （ARD §36「失败不自动恢复运动授权」）。
export async function clearMotorFault(id) {
  if (!isConnected()) return;
  const ok = window.confirm(
    `清除电机 ${id} 的故障锁存？\n\n` +
    '· 仅在该台非运动态受理（运动中会被固件拒绝）\n' +
    '· 身份冲突锁存（跨总线/协议身份不符）无法清除，会被固件拒绝\n' +
    '· 设备参数未初始化会触发一次回读重比对\n' +
    '· 清完仍须显式使能 + 新的完整快照/轨迹才可能回运动');
  if (!ok) return;

  const requestId = sendSystemCommand(SYS_CMD.CLEAR_FAULT, id);
  if (!requestId) return;
  setParamResult(true, `电机 ${id} 故障清除已请求…`);
  const result = await waitRequest(requestId, 10000, `电机 ${id} 故障清除`);
  if (!result.ok) { setParamResult(false, result.text); return; }
  setParamResult(true, `电机 ${id} 故障清除已受理：厂家故障需设备状态帧回报 ` +
    'fault_active=false 才收敛到 READY_DISABLED，届时再显式使能');
  await loadParams();
}

// 逐台方向切换（行内按钮）：写 m<id>.direction 的反值。翻转只改驱动器的力矩方向，
// **不改位置坐标系**（真机实测），所以零点与限位都不用重标；真正的风险是位置环反向
// 成正反馈——使能后关节会朝远离目标的方向冲，直到撞上设备限位为止。
export async function flipDirection(id, cur) {
  const next = cur === 1 ? 0 : 1;
  const name = next === 1 ? 'CCW' : 'CW';
  if (!window.confirm(`把 m${id} 的方向切成 ${name}？\n\n` +
      '方向一翻，驱动器的力矩方向反了：位置环会变成正反馈，一旦使能关节会朝' +
      '远离目标的方向冲，直到撞上设备限位。确认要翻吗？')) {
    return;
  }
  const req = sendParamSet([{motor_id: id, field: PARAM_FIELD.DIRECTION, value: next}]);
  if (!req) return;
  setParamResult(true, `m${id} 方向写入中 ${name}（req#${req}）…`);
  const result = await waitRequest(req, 10000, `m${id} 方向写入`);
  if (!result.ok) { setParamResult(false, result.text); return; }
  markDirty(DIRTY.PARAMS);
  setParamResult(true, `m${id} 方向已切换为 ${name}`);
}

// 固件频率配置沿用配置 topic / request_id 终态，参数值由系统状态 topic 回读。
export const FIRMWARE_FIELDS = ['system.agent_ping_hz', 'system.motion_report_hz',
  'system.status_report_hz', 'system.imu_report_hz', 'system.temperature_report_hz', 'system.motor_command_hz'];
async function waitFirmwareReport(after = -1) {
  const started = Date.now();
  const session = store.configSessionId;
  const generation = getConnectionGeneration();
  while (Date.now() - started < 3000) {
    if (!isConnected() || getConnectionGeneration() !== generation ||
        (session !== null && store.configSessionId !== session)) throw new Error('固件配置连接或会话已更换');
    const report = store.firmwareReport;
    if (report.values && report.seq > after && Date.now() - report.at < 1000) return report;
    await new Promise(resolve => setTimeout(resolve, 50));
  }
  throw new Error('未收到新的固件参数 topic 回读，请确认控制板固件版本');
}
export async function readFirmwareParameter(name) {
  const report = await waitFirmwareReport();
  if (name === 'system.unsaved') return report.unsaved;
  if (name === 'system.can_nominal_kbps') return 1000;
  if (name === 'system.can_data_kbps') return 5000;
  const field = FIRMWARE_FIELDS.indexOf(name);
  if (field < 0) throw new Error(`未知固件参数 ${name}`);
  return report.values[field];
}
export async function writeFirmwareParameter(name, value) {
  const field = FIRMWARE_FIELDS.indexOf(name);
  if (field < 0 || !Number.isInteger(value) || value < 1 || value > 1000) throw new Error('固件参数字段或数值无效');
  if (!isConnected()) throw new Error('未连接 rosbridge');
  const before = store.firmwareReport.seq;
  const request_id = nextParamReq();
  const sent = pub('firmwareConfig', {sync_time_ns: Date.now() * 1e6, session_id: 0, request_id,
    kind: 2, command: 0, motor_id: MOTOR_ID_NONE, count: 1,
    targets: [{motor_id: MOTOR_ID_NONE, field, value}]});
  if (!sent) throw new Error('维护写入被拒绝：请核对控制权、武装与 MCU 失能状态');
  const result = await waitRequest(request_id, 6000, '固件参数下发');
  if (!result.ok) throw new Error(result.text);
  await waitFirmwareReport(before);
}
export async function saveFirmwareParameters() {
  const before = store.firmwareReport.seq;
  const id = sendSystemCommand(SYS_CMD.SAVE, MOTOR_ID_NONE);
  const result = await waitRequest(id, 10000, '保存');
  if (!result.ok) throw new Error(`保存未确认成功：${result.text}`);
  await waitFirmwareReport(before);
}
