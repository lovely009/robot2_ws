// micro-ROS Agent 进程管理：只操作由本页面启动的 Agent 进程（后端 HTTP API），
// 不碰 rosbridge 连接。状态写 store.conn，顶栏文案由 main.js 的 renderShell 画；
// 串口选择框是本模块的控件，仍由 refreshAgentControls 直接维护。
// 搬迁自重构前的 app.js（setAgentStatus/agentApi/refreshAgentControls/
// refreshAgentStatus/startAgent/stopAgent），setAgentStatus 改为写 store。
import { store, markDirty, DIRTY } from './store.js';
import { $, CONTROLLER_PORT_STORAGE_KEY } from './format.js';

function setAgentStatus(status) {
  const c = store.conn;
  c.agent = status.state;
  c.agentDevice = status.device || '';
  c.agentError = status.error || '';
  c.agentReturncode = status.returncode === undefined ? null : status.returncode;
  c.canStop = !!status.canStop;
  markDirty(DIRTY.CONN);
}

async function agentApi(path, options = {}) {
  const response = await fetch(path, {cache: 'no-store', ...options});
  let body;
  try { body = await response.json(); }
  catch (_e) { throw new Error(`Agent API 返回无效响应（HTTP ${response.status}）`); }
  if (!response.ok) {
    const error = new Error(body.error || `Agent API 失败（HTTP ${response.status}）`);
    error.status = response.status;
    error.body = body;
    throw error;
  }
  return body;
}

export async function refreshAgentControls() {
  const select = $('serialPort');
  const savedPort = localStorage.getItem(CONTROLLER_PORT_STORAGE_KEY) || '';
  select.disabled = true;
  try {
    const [portResult, status] = await Promise.all([
      agentApi('/api/serial-ports'), agentApi('/api/agent/status'),
    ]);
    setAgentStatus(status);
    const ports = Array.isArray(portResult.ports) ? portResult.ports : [];
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

export async function refreshAgentStatus() {
  try { setAgentStatus(await agentApi('/api/agent/status')); }
  catch (_error) { /* 串口列表刷新会显示后端 API 错误；状态轮询不刷屏。 */ }
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
    const status = await agentApi('/api/agent/start', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({device}),
    });
    localStorage.setItem(CONTROLLER_PORT_STORAGE_KEY, device);
    setAgentStatus(status);
    // Process creation is immediate; give a fast startup failure a chance to surface.
    setTimeout(() => refreshAgentControls(), 700);
  } catch (error) {
    setAgentStatus({state: 'error', error: error.message,
      canStop: error.body?.status?.state === 'running',
      device: error.body?.status?.device || '', returncode: null});
  }
}

export async function stopAgent() {
  // 同 startAgent：「正在断开…」也是瞬时态。
  store.conn.agent = 'stopping';
  markDirty(DIRTY.CONN);
  try {
    setAgentStatus(await agentApi('/api/agent/stop', {method: 'POST'}));
  } catch (error) {
    setAgentStatus({state: 'error', error: error.message,
      canStop: true, returncode: null});
  }
}
