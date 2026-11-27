// 参数查询、失能态方向设置/驱动板保存、设零与故障清除。
import { store, markDirty, DIRTY, paramReadState } from './store';
import { SYS_CMD, MOTOR_ID_NONE, nsToHeader } from './format';
import { executeConfig, fetchDeviceInfo, isConnected, getConnectionGeneration } from './ros';
import { describeMotorState, policyControlError } from './motor_state';

let paramReq = (Date.now() & 0x3fffffff) >>> 0;
function nextParamReq(): number {
  paramReq = (paramReq + 1) & 0x3fffffff;
  return paramReq === 0 ? (paramReq = 1) : paramReq;
}

const CONFIG_STATUS = { PENDING: 0, SUCCEEDED: 1, FAILED: 2, BUSY: 3, CONFLICT: 4 };

function formatConfigResult(result: any, label: string) {
  if (!result) return { ok: false, outcome: 'timeout', text: `${label}：服务未返回结果` };
  if (Number(result.status) === CONFIG_STATUS.SUCCEEDED) {
    return { ok: true, result, text: `${label}成功（完成 ${result.completed_targets} 项）` };
  }
  const statusText = Number(result.status) === CONFIG_STATUS.PENDING
    ? '等待设备终态超时（设备可能仍在处理，不能据此判断已回滚）'
    : Number(result.status) === CONFIG_STATUS.BUSY ? '设备忙'
    : Number(result.status) === CONFIG_STATUS.CONFLICT ? 'request_id 冲突' : '设备拒绝';
  return {
    ok: false, outcome: 'failed', result,
    text: `${label}${statusText}：error_code=${result.error_code}，` +
      `失败 ${result.failed_targets} 项 / 完成 ${result.completed_targets} 项`,
  };
}

export function writeResult(key: string, ok: boolean, text: string): void {
  store.results.set(key, { ok, text, t: Date.now() });
  markDirty(DIRTY.PARAMS);
}
export function setParamResult(ok: boolean, text: string): void { writeResult('paramResult', ok, text); }

async function runConfigService(request: Record<string, any>, timeoutMs: number, label: string) {
  try {
    const result = await executeConfig(request, timeoutMs);
    return formatConfigResult(result, label);
  } catch (error: any) {
    return { ok: false, outcome: 'error', text: `${label}：${error?.message || '服务调用失败'}` };
  }
}

async function runSystemCommand(command: number, motorId: number | undefined, timeoutMs: number, label: string) {
  if (![SYS_CMD.REFRESH, SYS_CMD.SET_ZERO, SYS_CMD.CLEAR_FAULT, SYS_CMD.REPORT, SYS_CMD.SAVE_DEVICE].includes(command)) {
    return { ok: false, text: `${label}：未知系统命令`, result: undefined as any };
  }
  if (!isConnected()) { setParamResult(false, '未连接 rosbridge'); return null; }
  const syncTime = store.controlStates.sourceTimeNs || 0;
  if (command !== SYS_CMD.REPORT && !syncTime) {
    setParamResult(false, '尚未收到 MCU 同步源时间，未发送维护请求');
    return null;
  }
  return runConfigService({
    header: nsToHeader(syncTime),
    session_id: Number(store.configSessionId) || 0,
    request_id: nextParamReq(),
    kind: 1,
    count: 0,
    targets: [],
    command,
    motor_id: motorId === undefined ? MOTOR_ID_NONE : motorId,
  }, timeoutMs, label);
}

const zeroRequests = new Set<number>();
const deviceRequests = new Set<number>();

export function motorZeroError(id: number): string {
  if (!isConnected()) return '未连接 rosbridge';
  if (store.runtimeMode === 'readonly') return '当前工作台为只读模式';
  if (!Number.isInteger(id) || id < 1 || id > 64) return '请选择有效电机';
  if (deviceRequests.has(id)) return `电机 ${id} 参数事务处理中，请等待设备回执`;
  if (zeroRequests.has(id)) return `电机 ${id} 设零处理中，请等待设备回执`;
  const actual = describeMotorState(store, id);
  const control = store.controlStates.byId.get(id);
  if (!actual.online) return `电机 ${id} 离线或状态超时`;
  if (actual.fault || !control || control.vendor_fault_code || control.control_lifecycle !== 2 || control.policy_enabled
      || control.motion_enabled || control.execution_state !== 1) return `请先失能电机 ${id} 并清除故障，再设零`;
  const host = store.commandStatus.data;
  if (host && (['switching', 'stopping', 'manual', 'trajectory', 'error'].includes(host.mode)
      || host.active_controllers?.length || host.pending_goal_ids?.length)) return '请先停止主机关节控制，再设零';
  return '';
}

export function motorDeviceParameterError(id: number): string {
  const error = motorZeroError(id);
  if (error) return error.replaceAll('设零', '设置参数');
  if (store.motors.get(id)?.topo?.protocol !== 1) return '仅 CERB 驱动板支持方向设置与参数保存';
  const pending = policyControlError(store, [id]);
  if (pending) return pending;
  const cache = store.controlStates;
  const state = cache.byId.get(id);
  if (!cache.valid || !state || state.state_age_ms + Date.now() - cache.at > 1500)
    return '请等待新的 MCU 电机反馈';
  if (!cache.sourceTimeNs || !store.configSessionId || String(cache.session) !== String(store.configSessionId)) return '请等待 MCU 同步时间与会话';
  if (!store.commandStatus.data || Date.now() - store.commandStatus.last > 2500) return '请等待新的主机控制状态';
  return '';
}

export async function setMotorDirection(id: number, value: number) {
  const error = motorDeviceParameterError(id);
  if (error || ![0, 1].includes(value)) { setParamResult(false, error || '方向值无效'); return; }
  if (!window.confirm(`将电机 ${id} 驱动力矩方向设置为 ${value ? 'CCW' : 'CW'}？\n不改变位置坐标、零点或限位。方向为易失参数，驱动板保存不能固化方向。`)) return;
  const changed = motorDeviceParameterError(id);
  if (changed) { setParamResult(false, changed); return; }
  deviceRequests.add(id);
  setParamResult(true, `电机 ${id} 方向已请求，等待设备回读核验…`);
  try {
    const result = await runConfigService({
      header: nsToHeader(store.controlStates.sourceTimeNs),
      session_id: Number(store.configSessionId) || 0,
      request_id: nextParamReq(),
      kind: 0,
      count: 1,
      targets: [{ motor_id: id, field: 5, value }],
      command: 0,
      motor_id: MOTOR_ID_NONE,
    }, 10000, `电机 ${id} 方向设置`);
    setParamResult(result.ok, result.ok ? `电机 ${id} 方向已回读核验；方向易失，断电后需重新设置` : result.text);
  } finally { deviceRequests.delete(id); markDirty(DIRTY.PARAMS); }
}

export async function saveMotorDeviceParameters(id: number) {
  const error = motorDeviceParameterError(id);
  if (error) { setParamResult(false, error); return; }
  if (!window.confirm(`保存电机 ${id} 的驱动板参数？\n仅保存位置上下限、速度与电流上限；不保存方向，不写控制板存储。`)) return;
  deviceRequests.add(id);
  setParamResult(true, `电机 ${id} 驱动板保存已请求，等待设备事务终态…`);
  try {
    const result = await runSystemCommand(SYS_CMD.SAVE_DEVICE, id, 10000, `电机 ${id} 驱动板保存`);
    if (!result) return;
    setParamResult(result.ok, result.ok ? `电机 ${id} 已完成发送驱动板保存指令；厂家协议无保存应答，未验证掉电保持` : result.text);
  } finally { deviceRequests.delete(id); markDirty(DIRTY.PARAMS); }
}

export async function setMotorZero(id: number) {
  const error = motorZeroError(id);
  if (error) { setParamResult(false, error); return; }
  if (!window.confirm(`将电机 ${id} 的当前位置设为设备零点？\n设备位置限位对应的机械位置会随零点改变，请确认当前姿态与机械行程。`)) return;
  zeroRequests.add(id);
  setParamResult(true, `电机 ${id} 设零已请求，等待设备确认…`);
  try {
    const result = await runSystemCommand(SYS_CMD.SET_ZERO, id, 10000, `电机 ${id} 设零`);
    if (!result) return;
    setParamResult(result.ok, result.ok
      ? `电机 ${id} 设零成功（设备已确认），请核对实际位置与机械行程`
      : result.text + (Number(result.result?.error_code) === 5
        ? '；设备零位可能已改变，请核对实际反馈，勿直接重试' : ''));
  } finally { zeroRequests.delete(id); markDirty(DIRTY.PARAMS); }
}

export async function loadParams() {
  if (!isConnected()) return;
  setParamResult(true, '读取中…');
  try {
    await fetchDeviceInfo(0);
  } catch (error: any) {
    setParamResult(false, `读取失败：${error?.message || '未收到 device_info——固件/agent 未就绪？'}`);
    return;
  }
  const states = [...store.paramCache.keys()].map(id => ({ id, ...paramReadState(id) }));
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
  setParamResult(true, `正在回读电机 ${id} 的驱动板参数…`);
  const request = await runSystemCommand(SYS_CMD.REFRESH, id, 6000, '电机参数回读');
  if (!request) return;
  if (!request.ok) {
    setParamResult(false, request.result?.error_code === 12
      ? `电机 ${id} 尚未识别型号或未绑定，固件拒绝回读（error_code=12）。请先检查设备身份读取与型号绑定。`
      : request.text);
    return;
  }
  await loadParams();
}

export async function clearMotorFault(id: number) {
  if (!isConnected()) return;
  const ok = window.confirm(
    `清除电机 ${id} 的故障锁存？\n\n` +
    '· 仅在该台非运动态受理（运动中会被固件拒绝）\n' +
    '· 身份冲突锁存（跨总线/协议身份不符）无法清除，会被固件拒绝\n' +
    '· 设备参数未初始化会触发一次回读重比对\n' +
    '· 清完仍须显式使能 + 新的完整快照/轨迹才可能回运动');
  if (!ok) return;

  setParamResult(true, `电机 ${id} 故障清除已请求…`);
  const result = await runSystemCommand(SYS_CMD.CLEAR_FAULT, id, 10000, `电机 ${id} 故障清除`);
  if (!result) return;
  if (!result.ok) { setParamResult(false, result.text); return; }
  setParamResult(true, `电机 ${id} 故障清除已受理：厂家故障需设备状态帧回报 ` +
    'fault_active=false 才收敛到 READY_DISABLED，届时再显式使能');
  await loadParams();
}

export const FIRMWARE_FIELDS = [
  'system.agent_ping_hz', 'system.motion_report_hz',
  'system.status_report_hz', 'system.imu_report_hz',
  'system.temperature_report_hz', 'system.motor_command_hz',
] as const;

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

export async function readFirmwareParameter(name: string): Promise<number | boolean> {
  const report = await waitFirmwareReport();
  if (name === 'system.unsaved') return report.unsaved;
  if (name === 'system.can_nominal_kbps') return 1000;
  if (name === 'system.can_data_kbps') return 5000;
  const field = FIRMWARE_FIELDS.indexOf(name as any);
  if (field < 0) throw new Error(`未知固件参数 ${name}`);
  return report.values![field]!;
}
