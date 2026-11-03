// 参数查询、失能态设零与故障清除：设备屏和连接后自动读取共用。
// 这些函数既被设备屏的按钮调用，也被连接建立后的自动读取调用（由 Vue 工作台调度）——
// 自动读取不能住在面板里：store.paramCache 是使能面板列电机列表的依据，
// 切到「运行」屏时设备屏并未挂载。
import { store, markDirty, DIRTY, paramReadState } from './store.js';
import { SYS_CMD, MOTOR_ID_NONE } from './format.js';
import { pub, isConnected, waitDevParamSeq, waitConfigResult, getConnectionGeneration } from './ros.js';
import { describeMotorState } from './motor_state.js';

// 查询/维护事务的关联 id。两个高位都不能碰：bit31 是内核身份读取的命名空间，bit30 是
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
export function sendSystemCommand(command, motorId) {
  if (![SYS_CMD.REFRESH, SYS_CMD.SET_ZERO, SYS_CMD.CLEAR_FAULT, SYS_CMD.REPORT].includes(command)) return 0;
  if (!isConnected()) { setParamResult(false, '未连接 rosbridge'); return 0; }
  if (command === SYS_CMD.SET_ZERO) {
    const error = motorZeroError(motorId);
    if (error) {setParamResult(false, error);return 0;}
  }
  const syncTime=store.controlStates.sourceTimeNs || 0;
  if (command!==SYS_CMD.REPORT && !syncTime) {setParamResult(false,'尚未收到 MCU 同步源时间，未发送维护请求');return 0;}
  const request_id = nextParamReq();
  const sent = pub('sysCmd', {
    sync_time_ns: syncTime, request_id, command,
    motor_id: motorId === undefined ? MOTOR_ID_NONE : motorId,
  });
  return sent ? request_id : 0;
}

const zeroRequests = new Set();
export function motorZeroError(id) {
  if (!isConnected()) return '未连接 rosbridge';
  if (store.runtimeMode === 'readonly') return '当前工作台为只读模式';
  if (!Number.isInteger(id) || id < 1 || id > 64) return '请选择有效电机';
  if (zeroRequests.has(id)) return `电机 ${id} 设零处理中，请等待设备回执`;
  const actual = describeMotorState(store,id);
  const control = store.controlStates.byId.get(id);
  if (!actual.online) return `电机 ${id} 离线或状态超时`;
  if (actual.fault || !control || control.vendor_fault_code || control.control_lifecycle !== 2 || control.policy_enabled
      || control.motion_enabled || control.execution_state !== 1) return `请先失能电机 ${id} 并清除故障，再设零`;
  const host = store.commandStatus.data;
  if (host && (['switching','stopping','manual','trajectory','error'].includes(host.mode)
      || host.active_controllers?.length || host.pending_goal_ids?.length)) return '请先停止主机关节控制，再设零';
  return '';
}

export async function setMotorZero(id) {
  const error = motorZeroError(id);
  if (error) {setParamResult(false,error);return;}
  if (!window.confirm(`将电机 ${id} 的当前位置设为设备零点？\n设备位置限位对应的机械位置会随零点改变，请确认当前姿态与机械行程。`)) return;
  const requestId = sendSystemCommand(SYS_CMD.SET_ZERO,id);
  if (!requestId) return;
  zeroRequests.add(id);
  setParamResult(true,`电机 ${id} 设零已请求，等待设备确认…`);
  try {
    const result = await waitRequest(requestId,10000,`电机 ${id} 设零`);
    setParamResult(result.ok,result.ok
      ? `电机 ${id} 设零成功（设备已确认），请核对实际位置与机械行程`
      : result.text + (Number(result.result?.error_code) === 5
        ? '；设备零位可能已改变，请核对实际反馈，勿直接重试' : ''));
  } finally {zeroRequests.delete(id);markDirty(DIRTY.PARAMS);}
}

// 读取 = 发 SystemCommand(REPORT)，固件把**当前手里的参数视图**立刻发一帧
// device_param_report（不触发 SDO、不访问设备）。电机在线且此前核对过，
// dev_* 就是驱动板真值；电机不在线/没核对，dev_* 为 NaN，页面呈现 rec_*
// 即上电初始化基准。帧到达后订阅回调自行渲染，这里只等帧、给结论。
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
