// 参数读写事务层：设备屏按钮与「连接后自动读取」共用。
// 这些函数既被设备屏的按钮调用，也被连接建立后的自动读取调用（见 main.js）——
// 自动读取不能住在面板里：store.paramCache 是使能面板列电机列表的依据，
// 切到「运行」屏时设备屏并未挂载。
// 搬迁自旧 app.js 的参数页段；DOM 只剩「读当前表格里哪些输入被改过」这层查询。
import { store, markDirty, DIRTY } from './store.js';
import { $, PARAM_FIELD, SYS_CMD, MOTOR_ID_NONE, LIFECYCLE, fmtParam,
  devSlotNames, devRecoveryText } from './format.js';
import { pub, isConnected, waitSysStatus, waitDevParamSeq } from './ros.js';

// 写事务的关联 id。两个高位都不能碰：bit31 是内核身份读取的命名空间，bit30 是
// 固件自发脉冲的命名空间（置位会被本地拒）。0 表示「固件代分配」，所以绕开它。
let paramReq = (Date.now() & 0x3fffffff) >>> 0;
function nextParamReq() {
  paramReq = (paramReq + 1) & 0x3fffffff;
  return paramReq === 0 ? (paramReq = 1) : paramReq;
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
  pub('paramSet', {
    sync_time_ns: Date.now() * 1e6, // uint64：rosbridge 入站要 JSON number
    request_id, count: targets.length, targets,
  });
  return request_id;
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
  pub('sysCmd', {
    sync_time_ns: Date.now() * 1e6, request_id, command,
    motor_id: motorId === undefined ? MOTOR_ID_NONE : motorId,
  });
  return request_id;
}

// 「发命令 → 等它做完」的唯一判据：记下发送时读到的序号，等它变大。
// 首帧未到达前拒绝（重连后旧基线会让还在飞的轮询立刻假成功，见 close 处理）。
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
  for (const v of store.paramCache.values()) {
    if (Number.isFinite(v.dev.pos_min)) withDev++;
  }
  setParamResult(true, `已读取参数表：共 ${store.paramCache.size} 台，其中 ${withDev} 台` +
    '有驱动板回读值（显示驱动板真值），其余显示板内记录');
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
  markDirty(DIRTY.PARAMS);   // 重绘交给面板（dirty 标志已清，重建不会抹掉输入）
  setParamResult(true, `已下发 ${byMotor.size} 台 / ${sent} 项` +
    (chunks.length > 1 ? `（分 ${chunks.length} 条）` : '') +
    '。受理/拒绝是异步的，最终成败看 CONFIG_RESULT 事件');
}

export async function saveParamsToFlash() {
  if (!isConnected()) return;
  if (!store.sysStatus.seen) { setParamResult(false, '还没收到 system_status'); return; }
  // 完成判据是 save_seq 前进**且** save_in_progress 已转 false。参数面时期
  // system.save 在 W25 落盘之前就被清掉，报的是假完成——这里等的是真终态。
  // W25 要擦扇区（量级 ~1s/扇区），给足 10s。
  const before = store.sysStatus.save_seq;
  // SAVE 只接受 MOTOR_ID_NONE；给具体电机 id 会被本地拒（last_error=10）。
  if (!sendSystemCommand(SYS_CMD.SAVE, MOTOR_ID_NONE)) return;
  setParamResult(true, '保存中…（写 W25，约 1–2s）');
  const rc = await waitSysSeq('save_seq', before, 10000);
  if (rc === 0) {
    const deadline = Date.now() + 5000;
    while (store.sysStatus.save_in_progress && Date.now() < deadline) {
      await new Promise((r) => setTimeout(r, 150));
    }
    if (store.sysStatus.save_in_progress) {
      setParamResult(false, 'save_seq 已前进，但 save_in_progress 仍是 true——' +
        'W25 落盘未出终态，不要断电');
      return;
    }
    markDirty(DIRTY.PARAMS);
    setParamResult(true, `已保存到 W25（save_seq=${store.sysStatus.save_seq}，落盘已出终态）`);
  } else if (rc === -2) {
    setParamResult(false, '收不到 system_status，无法判断保存是否完成（勿断电）');
  } else {
    setParamResult(false, `保存超时未收尾：${lastErrorText()}——勿断电，重试一次`);
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
  if (!sendParamSet([{motor_id: id, field: PARAM_FIELD.POS_MIN, value: lo},
                     {motor_id: id, field: PARAM_FIELD.POS_MAX, value: hi}])) return;
  setParamResult(true, `m${id} ${which}已下发（${pos.toFixed(4)} rad），写事务收尾自动刷新…`);
  const rc = await waitDevParamSeq(before, 6000);
  if (rc === 0) {
    const now = store.paramCache.get(id);
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
export async function setMotorZero(id) {
  if (!isConnected()) return;
  const ok = window.confirm(
    `把电机 ${id} 的当前位置设为设备零点？\n\n` +
    '· 仅在该台失能时受理（运行中会被固件拒绝）\n' +
    '· 零点改变后设备内的位置限位也随之移动，请重新确认限位与机械行程\n' +
    '· 立即写入驱动器；是否掉电保存尚未实测确认');
  if (!ok) return;

  if (!store.sysStatus.seen) { setParamResult(false, '收不到 system_status，无法判断是否收尾'); return; }
  const before = store.sysStatus.set_zero_seq;
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
      (store.lastError === 0 ? '——错误码为 0 说明该版本没有这条命令（版本旧？）' : ''));
    return;
  }
  if (store.lastErrorMotor === id && store.lastError !== 0) {
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
export async function clearMotorFault(id) {
  if (!isConnected()) return;
  const ok = window.confirm(
    `清除电机 ${id} 的故障锁存？\n\n` +
    '· 仅在该台非运动态受理（运动中会被固件拒绝）\n' +
    '· 身份冲突锁存（跨总线/协议身份不符）无法清除，会被固件拒绝\n' +
    '· 设备参数未初始化会触发一次回读重比对\n' +
    '· 清完仍须显式使能 + 新的完整快照/轨迹才可能回运动');
  if (!ok) return;

  if (!store.sysStatus.seen) { setParamResult(false, '收不到 system_status，无法判断是否收尾'); return; }
  const before = store.sysStatus.clear_fault_seq;
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
  if (store.lastErrorMotor === id && store.lastError !== 0) {
    setParamResult(false, `电机 ${id} 故障清除未成功：${lastErrorText()}` +
      '（身份冲突锁存不可解除、该台在运动中、或该协议无清错手段，都会被拒）');
  } else {
    setParamResult(true, `电机 ${id} 故障清除已受理：厂家故障需设备状态帧回报 ` +
      'fault_active=false 才收敛到 READY_DISABLED，届时再显式使能');
  }
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
  setParamResult(true, `m${id} 方向已下发 ${name}（req#${req}）；` +
    '受理/拒绝是异步的——写事务通过才生效，看 CONFIG_RESULT');
  // 写事务（写 + 读回核验）收尾后固件即时发一帧 device_param_report，这里只需重绘。
  setTimeout(() => markDirty(DIRTY.PARAMS), 500);
}
