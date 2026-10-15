// 台架控制通道（bench_manager 的 /bench/* ROS 服务）+ 顶栏 Agent 控件。
//
// 旧版走 web_host 的私有 HTTP API；现在进程管理（仿真 / Agent / 电机后端）归
// bench_manager 节点，页面只依赖 rosbridge。状态不在这里：/bench/status 话题
// （core/ros.js 订阅）与命令响应的 status_json 都经 store.js 的 applyBenchStatus
// 写 store.bench；顶栏文案由 main.js 的 renderShell 画，串口选择框是本模块的
// 控件（refreshAgentControls 直接维护）。
import { store, markDirty, DIRTY, applyBenchStatus } from './store.js';
import { $, CONTROLLER_PORT_STORAGE_KEY } from './format.js';
import { callService } from './ros.js';

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
};

function parseJson(text, fallback) {
  try { return JSON.parse(text); } catch (_e) { return fallback; }
}

// 统一收尾：状态快照先入 store（失败时也带，供页面回显），再按 ok 抛错；错误保持
// 服务端原文（管理器给出的原因比笼统的"请求失败"有用得多）。
async function call(entry, request) {
  const response = await callService(entry[0], entry[1], request);
  if (response.status_json) {
    try { applyBenchStatus(JSON.parse(response.status_json)); }
    catch (_e) { /* 快照损坏不该吞掉真正要报的业务错误 */ }
  }
  if (!response.ok) throw new Error(response.error || '台架服务失败');
  return response;
}

// ---- 台架服务（仿真面板与顶栏共用）---------------------------------------

export async function getBenchOptions() {
  const response = await callService(SVC.options[0], SVC.options[1], {});
  return {
    can_interfaces: parseJson(response.can_interfaces_json, []),
    serial_ports: parseJson(response.serial_ports_json, []),
    models: response.simulation_models || [],
    mujoco_models: response.mujoco_models || [],
  };
}

export async function getAgentStatus() {
  const response = await callService(SVC.agentStatus[0], SVC.agentStatus[1], {});
  const status = parseJson(response.status_json, {});
  applyBenchStatus({agent: status});
  return status;
}

export function startSimulation(request) { return call(SVC.startSimulation, request); }
export function stopSimulation() { return call(SVC.stopSimulation, {}); }
export function startAgentService(device) { return call(SVC.startAgent, {device}); }
export function stopAgentService() { return call(SVC.stopAgent, {}); }

export function startMotorBackend(request) {
  return call(SVC.startMotorBackend, {
    model: request.model,
    transport: request.transport,
    sim_interface: request.sim_interface || '',
    sim_tty: request.sim_tty || '',
    physical_bus_isolated: Boolean(request.physical_bus_isolated),
  });
}
export function stopMotorBackend() { return call(SVC.stopMotorBackend, {}); }

// 转发一条模拟器控制命令（add/remove/link/status…）；返回控制通道响应对象。
// 服务层 ok 只管转发本身；模拟器给出的**语义**拒绝（id 冲突、范围外……）是
// 响应体里的 ok:false，这里一并转成异常。
export async function motorBackendCommand(command) {
  const response = await callService(
    SVC.motorBackendCommand[0], SVC.motorBackendCommand[1],
    {request_json: JSON.stringify(command)});
  if (!response.ok) throw new Error(response.error || '控制通道不可用');
  const payload = parseJson(response.response_json, {});
  if (payload && payload.ok === false) {
    throw new Error(payload.error || '模拟器拒绝了该命令');
  }
  return payload;
}

// ---- 顶栏 Agent 控件 ------------------------------------------------------

function setAgentStatus(status) {
  applyBenchStatus({agent: status || {state: 'stopped'}});
}

export async function refreshAgentControls() {
  const select = $('serialPort');
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
      ports.push({path: activeDevice,
        label: `${activeDevice}（Agent 正在使用，设备当前未枚举）`, accessible: true});
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
    if (select.value) localStorage.setItem(CONTROLLER_PORT_STORAGE_KEY, select.value);
  } catch (error) {
    select.replaceChildren();
    const option = document.createElement('option');
    option.value = '';
    option.textContent = '无法读取主机串口列表';
    select.append(option);
    setAgentStatus({state: 'error', error: error.message, returncode: null});
  } finally {
    select.disabled = false;
  }
}

export async function startAgent() {
  const device = $('serialPort').value;
  if (!device) {
    setAgentStatus({state: 'error', error: '请先选择控制板串口', returncode: null});
    return;
  }
  // 「正在启动…」是瞬时态：写 store 让 renderShell 画（按钮禁用也在那里）。
  store.conn.agent = 'starting';
  markDirty(DIRTY.CONN);
  try {
    await startAgentService(device);
    localStorage.setItem(CONTROLLER_PORT_STORAGE_KEY, device);
    // Process creation is immediate; give a fast startup failure a chance to surface.
    setTimeout(() => refreshAgentControls(), 700);
  } catch (error) {
    const last = store.bench.agent || {};
    setAgentStatus({state: 'error', error: error.message,
      canStop: last.state === 'running',
      device: last.device || '', returncode: null});
  }
}

export async function stopAgent() {
  // 同 startAgent：「正在断开…」也是瞬时态。
  store.conn.agent = 'stopping';
  markDirty(DIRTY.CONN);
  try {
    await stopAgentService();
  } catch (error) {
    setAgentStatus({state: 'error', error: error.message,
      canStop: true, returncode: null});
  }
}
