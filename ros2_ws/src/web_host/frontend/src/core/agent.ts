// 台架控制通道（bench_manager 的 /bench/* ROS 服务）+ 顶栏 Agent 控件。
import { store, markDirty, DIRTY, applyBenchStatus } from './store';
import { $, CONTROLLER_PORT_STORAGE_KEY, headerStampNs } from './format';
import { callService } from './ros';

export function describeAgentStatus(state: any, now = Date.now()) {
  const unknown = { link: '未连接', clock: '未知', authority: '未知', detail: '' };
  if (!state.conn.connected) return unknown;
  const report = state.agentWireStatus;
  if (!report?.data) return { ...unknown, link: '等待状态' };
  let age = now - report.at;
  if (!Number.isFinite(age) || age < 0 || age > 300) return { ...unknown, link: '状态陈旧' };
  const m = report.data;
  if (m.schema_version !== 6) return { ...unknown, link: '版本不匹配' };
  if (m.clock_valid === true) {
    const stampNs = headerStampNs(m);
    const sourceMs = (stampNs > 0 ? stampNs : Number(m.snapshot_stamp)) / 1e6;
    if (!Number.isFinite(sourceMs) || sourceMs <= 0 || sourceMs - now > 20)
      return { ...unknown, link: '在线', clock: '时间基准异常' };
    age = Math.max(age, now - sourceMs);
    if (age > 300) return { ...unknown, link: '状态陈旧' };
  }
  const syncAge = Number(m.clock_sync_age_ms) + age;
  const clockOk = m.clock_valid === true && Number.isFinite(syncAge) && syncAge >= 0 && syncAge <= 30000;
  const remaining = (m.lease_valid_until_tick_ms - m.snapshot_tick_ms) >>> 0;
  const authorized = clockOk && m.lease_active === true && remaining > age && remaining <= 300;
  return {
    link: '在线', clock: clockOk ? '正常' : '异常', authority: authorized ? '有效' : '未授权',
    detail: `状态年龄 ${age} ms；校时年龄 ${syncAge} ms；校时失败 ${m.clock_sync_failures ?? 0} 次`,
  };
}

// 服务名/类型与 bench_manager/node.py 的注册一一对应（不做 remap）。
const SVC = {
  options: ['/bench/options', 'bench_msgs/srv/GetBenchOptions'],
  agentStatus: ['/bench/agent_status', 'bench_msgs/srv/GetAgentStatus'],
  startSimulation: ['/bench/start_simulation', 'bench_msgs/srv/StartSimulation'],
  stopSimulation: ['/bench/stop_simulation', 'bench_msgs/srv/StopSimulation'],
  startAgent: ['/bench/start_agent', 'bench_msgs/srv/StartAgent'],
  stopAgent: ['/bench/stop_agent', 'bench_msgs/srv/StopAgent'],
  startMotorBackend: ['/bench/start_motor_backend', 'bench_msgs/srv/StartMotorBackend'],
  stopMotorBackend: ['/bench/stop_motor_backend', 'bench_msgs/srv/StopMotorBackend'],
  motorBackendCommand: ['/bench/motor_backend_command', 'bench_msgs/srv/MotorBackendCommand'],
  unitLog: ['/bench/get_unit_log', 'bench_msgs/srv/GetUnitLog'],
  startGravityFeedforward: ['/bench/start_gravity_feedforward',
    'bench_msgs/srv/StartGravityFeedforward'],
  stopGravityFeedforward: ['/bench/stop_gravity_feedforward',
    'bench_msgs/srv/StopGravityFeedforward'],
  setGravityFeedforwardGain: ['/bench/set_gravity_feedforward_gain',
    'bench_msgs/srv/SetGravityFeedforwardGain'],
} as const;

export const UNIT_SIMULATION = 'bench-simulation.service';
export const UNIT_MOTOR_BACKEND = 'bench-motor-backend.service';

function parseJson<T>(text: string, fallback: T): T {
  try { return JSON.parse(text); } catch (_e) { return fallback; }
}

async function call(entry: readonly [string, string], request: Record<string, any>) {
  const response = await callService(entry[0], entry[1], request);
  if (response.status_json) {
    try { applyBenchStatus(JSON.parse(response.status_json)); }
    catch (_e) { /* ignore */ }
  }
  if (!response.ok) throw new Error(response.error || '台架服务失败');
  return response;
}

export async function getBenchOptions() {
  const response = await callService(SVC.options[0], SVC.options[1], {});
  return {
    can_interfaces: parseJson(response.can_interfaces_json, [] as any[]),
    serial_ports: parseJson(response.serial_ports_json, [] as any[]),
    usbcanfd_devices: parseJson(response.usbcanfd_devices_json, [] as any[]),
    models: (response.simulation_models || []) as string[],
    mujoco_models: (response.mujoco_models || []) as string[],
  };
}

export async function getAgentStatus() {
  const response = await callService(SVC.agentStatus[0], SVC.agentStatus[1], {});
  const status = parseJson(response.status_json, {} as Record<string, any>);
  applyBenchStatus({ agent: status });
  return status;
}

export function startSimulation(request: Record<string, any>) { return call(SVC.startSimulation, request); }
export function stopSimulation() { return call(SVC.stopSimulation, {}); }
export function startAgentService(device: string) { return call(SVC.startAgent, { device }); }
export async function setFeedbackModel(model: string) {
  const response = await callService('/bench_manager/set_parameters_atomically',
    'rcl_interfaces/srv/SetParametersAtomically', {
      parameters: [{ name: 'feedback_model', value: { type: 4, string_value: model } }],
    });
  if (!response.result?.successful) {
    throw new Error(response.result?.reason || '反馈模型同步失败');
  }
}
export function stopAgentService() { return call(SVC.stopAgent, {}); }

export function startMotorBackend(request: Record<string, any>) {
  return call(SVC.startMotorBackend, {
    model: request.model,
    transport: request.transport,
    sim_interface: request.sim_interface || '',
    sim_tty: request.sim_tty || '',
    sim_channels_json: request.sim_channels_json || '',
    physical_bus_isolated: Boolean(request.physical_bus_isolated),
  });
}
export function stopMotorBackend() { return call(SVC.stopMotorBackend, {}); }

export function startGravityFeedforward(request: Record<string, any>) {
  return call(SVC.startGravityFeedforward, {
    model: request.model,
    joint_map: request.joint_map,
    effort_gain: Number(request.effort_gain),
  });
}
export function stopGravityFeedforward() {
  return call(SVC.stopGravityFeedforward, {});
}
export function setGravityFeedforwardGain(effortGain: number) {
  return call(SVC.setGravityFeedforwardGain, { effort_gain: Number(effortGain) });
}

export async function getUnitLog(unitName: string, lines = 80): Promise<string> {
  const response = await callService(SVC.unitLog[0], SVC.unitLog[1],
    { unit_name: unitName, lines });
  if (!response.ok) throw new Error(response.error || '读取日志失败');
  return response.text || '';
}

export async function motorBackendCommand(command: Record<string, any>) {
  const response = await callService(
    SVC.motorBackendCommand[0], SVC.motorBackendCommand[1],
    { request_json: JSON.stringify(command) });
  if (!response.ok) throw new Error(response.error || '控制通道不可用');
  const payload = parseJson(response.response_json, {} as Record<string, any>);
  if (payload && payload.ok === false) {
    throw new Error(payload.error || '模拟器拒绝了该命令');
  }
  return payload;
}

function setAgentStatus(status: Record<string, any> | null) {
  applyBenchStatus({ agent: status || { state: 'stopped' } });
}

export async function refreshAgentControls() {
  const select = $('serialPort') as HTMLSelectElement;
  const savedPort = localStorage.getItem(CONTROLLER_PORT_STORAGE_KEY) || '';
  select.disabled = true;
  try {
    const [options, status] = await Promise.all([
      getBenchOptions(), getAgentStatus(),
    ]);
    const ports = Array.isArray(options.serial_ports) ? options.serial_ports : [];
    const activeDevice = status.state === 'running' ? status.device : '';
    const known = new Set(ports.map((port) => port.path));
    if (activeDevice && !known.has(activeDevice)) {
      ports.push({
        path: activeDevice,
        label: `${activeDevice}（Agent 正在使用，设备当前未枚举）`, accessible: true,
      });
    }

    select.replaceChildren();
    const placeholder = document.createElement('option');
    placeholder.value = '';
    placeholder.textContent = ports.length ? '选择控制板串口' : '未检测到串口';
    select.append(placeholder);
    for (const port of ports) {
      const option = document.createElement('option');
      option.value = port.path;
      option.textContent = port.label + (port.accessible === false ? '（无读写权限）' : '');
      option.disabled = port.accessible === false && port.path !== activeDevice;
      select.append(option);
    }

    const selected = activeDevice || savedPort;
    if (selected && known.has(selected)) select.value = selected;
    else if (activeDevice) select.value = activeDevice;
    else {
      const stm32Ports = ports.filter((port) => port.accessible !== false
        && /stm32/i.test(`${port.path} ${port.label || ''}`));
      if (stm32Ports.length === 1) select.value = stm32Ports[0].path;
    }
    if (select.value) localStorage.setItem(CONTROLLER_PORT_STORAGE_KEY, select.value);
    select.title = select.selectedOptions[0]?.textContent || '';
  } catch (error: any) {
    select.replaceChildren();
    const option = document.createElement('option');
    option.value = '';
    option.textContent = '无法读取主机串口列表';
    select.append(option);
    select.title = '';
    setAgentStatus({ state: 'error', error: error.message, returncode: null });
  } finally {
    select.disabled = ['running', 'starting', 'stopping'].includes(store.conn.agent);
  }
}

export async function startAgent() {
  const device = ($('serialPort') as HTMLSelectElement).value;
  if (!device) {
    setAgentStatus({ state: 'error', error: '请先选择控制板串口', returncode: null });
    return;
  }
  store.conn.agent = 'starting';
  store.conn.agentError = '';
  markDirty(DIRTY.CONN);
  try {
    await startAgentService(device);
    localStorage.setItem(CONTROLLER_PORT_STORAGE_KEY, device);
    setTimeout(() => refreshAgentControls(), 700);
  } catch (error: any) {
    const last = store.bench.agent || {};
    setAgentStatus({
      state: 'error', error: error.message,
      canStop: last.state === 'running',
      device: last.device || '', returncode: null,
    });
  }
}

export async function stopAgent() {
  store.conn.agent = 'stopping';
  store.conn.agentError = '';
  markDirty(DIRTY.CONN);
  try {
    await stopAgentService();
  } catch (error: any) {
    setAgentStatus({
      state: 'error', error: error.message,
      canStop: true, returncode: null,
    });
  }
}
