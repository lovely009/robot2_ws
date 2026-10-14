// 仿真启动器面板：控制端 / CAN 接口 / 模型 / 端点选择，启动与停止
// robot_sim 的 simulation.launch.py，子进程状态与最近日志来自
// /api/simulation/*（HTTP，与 rosbridge 无关）。因此本面板不订阅、不写 store（只取
// core/store.js 的 MODEL 常量）；
// 切屏后状态与日志由后端 status 接口整段恢复，日志不重复追加。
// 搬自重构前的 web/simulation.js（页面级 IIFE 单例），只改挂载方式：ui 映射与
// 全部逻辑移进 mount 作用域（切屏会反复挂载，模块级 ui 会指向已卸载的 DOM），
// 1.8 s 状态轮询由 mount 持有句柄并在 unmount 清理——旧代码是页面级单例，
// 切屏会重复注册轮询。
import { $ } from '../core/format.js';
// 默认模型名与臂视图共用一份（core/store.js 的 MODEL），别在这里再写一个字符串。
import { MODEL } from '../core/store.js';

const STORAGE_KEY = 'robot2.simulation.options.v1';
const stateLabels = {
  stopped: '未启动',
  starting: '启动中',
  running: '启动进程运行中',
  stopping: '停止中',
  failed: '启动失败',
};
const backendLabels = { gazebo: 'Gazebo', mujoco: 'MuJoCo' };

export function mount(root, ctx) {
  const ui = {
    backend: $('simBackend'),
    mode: $('simMode'),
    transport: $('simTransport'),
    model: $('simModel'),
    vcanName: $('simVcanName'),
    simInterface: $('simInterface'),
    driverInterface: $('simDriverInterface'),
    simTty: $('simTty'),
    driverTty: $('simDriverTty'),
    vcanFields: $('simVcanFields'),
    socketFields: $('simSocketFields'),
    driverCanField: $('simDriverCanField'),
    slcanFields: $('simSlcanFields'),
    driverTtyField: $('simDriverTtyField'),
    physicalGuard: $('simPhysicalGuard'),
    hostBusHint: $('simHostBusHint'),
    busIsolated: $('simBusIsolated'),
    firmwareHint: $('simFirmwareHint'),
    start: $('btnSimStart'),
    stop: $('btnSimStop'),
    refresh: $('btnSimRefresh'),
    summary: $('simSummary'),
    state: $('simState'),
    result: $('simResult'),
    log: $('simLog'),
    backendSummary: $('motorBackendSummary'),
    backendState: $('motorBackendState'),
    backendResult: $('motorBackendResult'),
    backendLog: $('motorBackendLog'),
    backendStart: $('btnMotorBackendStart'),
    backendStop: $('btnMotorBackendStop'),
    motorPanel: $('simMotorPanel'),
    motorHint: $('simMotorHint'),
    motorList: $('simMotorList'),
    motorAddId: $('simMotorAddId'),
    motorAddKind: $('simMotorAddKind'),
    motorAddModel: $('simMotorAddModel'),
    motorAddModelField: $('simMotorAddModelField'),
    motorAdd: $('btnSimMotorAdd'),
  };

  let deviceOptions = { can_interfaces: [], serial_ports: [], models: [], mujoco_models: [] };
  let optionsLoaded = false;
  let latestStatus = { state: 'stopped', log: [] };
  let motorBackend = { state: 'stopped', log: [] };
  let motorSim = { motors: [] };
  let polling = false;
  let pollTimer = null;

  function readSaved() {
    try {
      return JSON.parse(localStorage.getItem(STORAGE_KEY) || '{}') || {};
    } catch (_error) {
      return {};
    }
  }

  const saved = readSaved();

  function saveOptions() {
    try {
      localStorage.setItem(STORAGE_KEY, JSON.stringify({
        backend: ui.backend.value,
        controller_mode: ui.mode.value,
        transport: ui.transport.value,
        model: ui.model.value,
        vcan_interface: ui.vcanName.value,
        sim_interface: ui.simInterface.value,
        driver_interface: ui.driverInterface.value,
        sim_tty: ui.simTty.value,
        driver_tty: ui.driverTty.value,
      }));
    } catch (_error) {
      // The simulation controls remain usable when browser storage is disabled.
    }
  }

  async function requestJson(path, options = {}) {
    const response = await fetch(path, {
      cache: 'no-store',
      ...options,
      headers: {
        ...(options.body ? { 'Content-Type': 'application/json' } : {}),
        ...(options.headers || {}),
      },
    });
    let payload;
    try {
      payload = await response.json();
    } catch (_error) {
      payload = {};
    }
    if (!response.ok) {
      // 旧版 static_server 没有新 API 时会退到静态文件处理（HTML 体、无 error
      // 字段）——这是"改了 .py 没重启 web_host.launch.py"的固定指纹，直接点名
      const message = payload.error
        || (response.status === 404
          ? '后端没有该接口：web_host 进程可能是旧版，请重启 web_host.launch.py'
          : `HTTP ${response.status}`);
      const error = new Error(message);
      error.payload = payload;
      throw error;
    }
    return payload;
  }

  function setResult(message, kind = '') {
    ui.result.textContent = message;
    ui.result.className = `result tiny${kind ? ` ${kind}` : ''}`;
  }

  function replaceOptions(select, entries, placeholder, preferredValue = '') {
    const previous = select.value;
    const desired = previous || preferredValue;
    select.replaceChildren();
    const empty = document.createElement('option');
    empty.value = '';
    empty.textContent = placeholder;
    select.append(empty);

    for (const entry of entries) {
      const option = document.createElement('option');
      option.value = entry.value;
      option.textContent = entry.label;
      option.disabled = Boolean(entry.disabled);
      if (entry.title) option.title = entry.title;
      select.append(option);
    }
    if ([...select.options].some((option) => option.value === desired)) {
      select.value = desired;
    } else if (entries.length) {
      const firstEnabled = entries.find((entry) => !entry.disabled);
      if (firstEnabled) select.value = firstEnabled.value;
    }
  }

  // MuJoCo 后端的被控对象是模型目录自带的 MJCF，不是 ros2_control：只能列有
  // mujoco/<模型名>.xml 的模型（后端也做同样校验，这里只是别让人选到必然报错的组合）。
  function modelEntries() {
    const names = ui.backend.value === 'mujoco'
      ? (deviceOptions.mujoco_models || [])
      : (deviceOptions.models || []);
    return names.map((model) => ({ value: model, label: model }));
  }

  function refreshModelOptions(preferredValue) {
    const placeholder = ui.backend.value === 'mujoco'
      ? '该后端没有可用模型（需要模型自带 MJCF）'
      : '选择仿真模型';
    replaceOptions(ui.model, modelEntries(), placeholder, preferredValue);
  }

  function loadDeviceOptions(options) {
    deviceOptions = options;
    optionsLoaded = true;
    // 兜底默认取 MODEL（与臂视图同一个默认值；不在列表里的值由 replaceOptions 落到
    // 第一个可用模型，旧的写死字符串已随模型改名失效过两次）。
    refreshModelOptions(saved.model || MODEL);

    const cans = (options.can_interfaces || []).map((item) => ({
      value: item.name,
      label: item.label || `${item.name} (${item.up ? 'UP' : 'DOWN'})`,
      disabled: !item.up,
      title: item.up ? '' : '接口当前为 DOWN',
    }));
    replaceOptions(
      ui.simInterface, cans, '选择 SocketCAN 接口', saved.sim_interface || 'can1');
    replaceOptions(
      ui.driverInterface, cans, '选择 SocketCAN 接口', saved.driver_interface || 'can0');

    const rawSerials = options.serial_ports || [];
    const serials = rawSerials.map((item) => ({
      value: item.path,
      label: item.accessible ? item.label : `${item.label}（无读写权限）`,
      disabled: !item.accessible,
      title: item.accessible ? '' : 'web_host 用户没有该串口的读写权限',
    }));
    replaceOptions(ui.simTty, serials, '选择模拟器串口', saved.sim_tty || '');
    const secondAccessibleSerial = rawSerials
      .filter((item) => item.accessible)[1];
    replaceOptions(
      ui.driverTty, serials, '选择驱动端串口',
      saved.driver_tty || (secondAccessibleSerial && secondAccessibleSerial.path) || '');

    if (!ui.vcanName.value || ui.vcanName.value === 'robot_vcan0') {
      ui.vcanName.value = saved.vcan_interface || 'robot_vcan0';
    }
    syncFields();
  }

  function syncFields() {
    const mode = ui.mode.value;
    const transport = ui.transport.value;
    const isHost = mode === 'host';
    ui.vcanFields.hidden = transport !== 'vcan';
    ui.socketFields.hidden = transport !== 'socketcan';
    ui.driverCanField.hidden = transport !== 'socketcan' || !isHost;
    ui.slcanFields.hidden = transport !== 'slcan';
    ui.driverTtyField.hidden = transport !== 'slcan' || !isHost;
    ui.physicalGuard.hidden = transport === 'vcan';
    ui.hostBusHint.hidden = transport === 'vcan' || !isHost;
    ui.firmwareHint.hidden = mode !== 'firmware';

    const vcanOption = [...ui.transport.options]
      .find((option) => option.value === 'vcan');
    if (vcanOption) vcanOption.disabled = mode === 'firmware';
    if (mode === 'firmware' && transport === 'vcan') {
      ui.transport.value = deviceOptions.can_interfaces.some((item) => item.up)
        ? 'socketcan'
        : 'slcan';
      syncFields();
      return;
    }
    // 后端决定模型候选集：切换时重列，当前模型仍在新列表里就保留
    if (optionsLoaded) refreshModelOptions();
    saveOptions();
    // vcan（纯软）下电机后端不可用；切传输时立即反映，不等下一次轮询
    ui.backendStart.disabled = activeState(motorBackend.state)
      || !motorBackendStartable();
  }

  function activeState(state) {
    return ['starting', 'running', 'stopping'].includes(state);
  }

  function describeConfig(config) {
    if (!config) return '';
    const mode = {
      host: '主机驱动',
      external: '外部控制器',
      firmware: 'STM32 HIL',
    }[config.controller_mode] || config.controller_mode;
    const backend = backendLabels[config.backend] || config.backend || '';
    const endpoint = config.vcan_interface
      || config.sim_interface
      || config.sim_tty
      || '';
    return [backend, mode, config.transport, endpoint].filter(Boolean).join(' · ');
  }

  function renderStatus(status) {
    latestStatus = status || { state: 'stopped', log: [] };
    const state = latestStatus.state || 'stopped';
    const label = stateLabels[state] || state;
    const active = activeState(state);
    ui.summary.textContent = latestStatus.config && active
      ? `${label} · ${latestStatus.config.transport}`
      : label;
    ui.state.textContent = latestStatus.config && active
      ? `${label} · ${describeConfig(latestStatus.config)}`
      : state === 'failed'
        ? `${label} · exit ${latestStatus.returncode}`
        : `仿真进程${label}`;
    ui.start.disabled = active;
    ui.stop.disabled = !active;

    for (const control of [
      ui.backend, ui.mode, ui.transport, ui.model, ui.vcanName, ui.simInterface,
      ui.driverInterface, ui.simTty, ui.driverTty, ui.busIsolated,
    ]) {
      control.disabled = active;
    }

    const lines = latestStatus.log || [];
    ui.log.textContent = lines.length ? lines.join('\n') : '等待启动';
    ui.log.scrollTop = ui.log.scrollHeight;
  }

  // ---- 电机后端（独立进程：fdcan_motor_sim.py）------------------------
  // HIL 启动仿真时由后端默认联动拉起（auto_started），停止时联动收起；
  // 也可脱离仿真单独起停（只连板子验证固件行为）。参数取仿真抽屉的选择。
  function describeBackendConfig(config) {
    if (!config) return '';
    const endpoint = config.sim_tty || config.sim_interface || '';
    return [config.transport, config.model, endpoint].filter(Boolean).join(' · ');
  }

  function motorBackendStartable() {
    return ui.transport.value !== 'vcan';
  }

  function renderMotorBackend(status) {
    motorBackend = status || { state: 'stopped', log: [] };
    const state = motorBackend.state || 'stopped';
    const label = stateLabels[state] || state;
    const active = activeState(state);
    ui.backendSummary.textContent = label;
    ui.backendState.textContent = motorBackend.config
      ? `${label} · ${describeBackendConfig(motorBackend.config)}`
        + (motorBackend.auto_started ? ' · 随仿真' : '')
      : state === 'failed'
        ? `${label} · exit ${motorBackend.returncode}`
        : `电机后端${label}`;
    ui.backendStart.disabled = active || !motorBackendStartable();
    ui.backendStop.disabled = !active;
    const lines = motorBackend.log || [];
    ui.backendLog.textContent = lines.length ? lines.join('\n') : '等待启动';
    ui.backendLog.scrollTop = ui.backendLog.scrollHeight;
  }

  function setBackendResult(message, kind = '') {
    ui.backendResult.textContent = message;
    ui.backendResult.className = `result tiny${kind ? ` ${kind}` : ''}`;
  }

  async function refreshMotorBackend() {
    try {
      renderMotorBackend(await requestJson('/api/motor-sim/process'));
    } catch (error) {
      setBackendResult(`读取电机后端状态失败：${error.message}`, 'err');
    }
  }

  function motorBackendRequest() {
    const transport = ui.transport.value;
    const request = {
      model: ui.model.value,
      transport,
      physical_bus_isolated: ui.busIsolated.checked,
    };
    if (transport === 'socketcan') request.sim_interface = ui.simInterface.value;
    else request.sim_tty = ui.simTty.value;
    return request;
  }

  async function startMotorBackend() {
    const request = motorBackendRequest();
    if (!request.model) {
      setBackendResult('先在上方「仿真」里选择模型。', 'err');
      return;
    }
    if (request.transport === 'vcan') {
      setBackendResult('纯软 vcan 不需要电机后端（总线电机由 motor_sim 节点扮演）。', 'err');
      return;
    }
    if (!request.physical_bus_isolated) {
      setBackendResult('请先勾选「物理 CAN 总线仅连接仿真器与控制端」。', 'err');
      return;
    }
    if (request.transport === 'socketcan' && !request.sim_interface) {
      setBackendResult('先在上方选择模拟器 SocketCAN 接口。', 'err');
      return;
    }
    if (request.transport === 'slcan' && !request.sim_tty) {
      setBackendResult('先在上方选择模拟器串口。', 'err');
      return;
    }
    ui.backendStart.disabled = true;
    setBackendResult('正在启动电机后端…');
    try {
      renderMotorBackend(await requestJson('/api/motor-sim/process/start', {
        method: 'POST',
        body: JSON.stringify(request),
      }));
      setBackendResult('电机后端进程已启动；CAN 就绪情况见下方日志。', 'ok');
    } catch (error) {
      if (error.payload && error.payload.status) renderMotorBackend(error.payload.status);
      setBackendResult(error.message, 'err');
    } finally {
      await refreshMotorBackend();
      await refreshMotorSim();
    }
  }

  async function stopMotorBackend() {
    ui.backendStop.disabled = true;
    setBackendResult('正在停止电机后端…');
    try {
      renderMotorBackend(await requestJson('/api/motor-sim/process/stop', { method: 'POST' }));
      setBackendResult('电机后端已停止', 'ok');
    } catch (error) {
      setBackendResult(`停止失败：${error.message}`, 'err');
    } finally {
      await refreshMotorBackend();
      await refreshMotorSim();
    }
  }

  // ---- 电机表与增删/断连（后端内模拟器的控制通道）--------------------
  function motorPanelActive() {
    return activeState(motorBackend.state);
  }

  function renderMotorSim(note) {
    const active = motorPanelActive();
    ui.motorPanel.hidden = !active;
    if (!active) return;
    ui.motorHint.textContent = note || `${motorSim.motors.length} 台在模拟`;
    ui.motorList.replaceChildren();
    for (const motor of motorSim.motors) {
      const row = document.createElement('div');
      row.className = motor.linked ? 'sim-motor-row' : 'sim-motor-row offline';
      const label = document.createElement('span');
      label.className = 'sim-motor-label';
      const kindText = motor.kind.toUpperCase()
        + (motor.model ? ` ${motor.model.toUpperCase()}` : '');
      label.textContent = `#${motor.id} ${kindText}`;
      const where = document.createElement('span');
      where.className = 'sim-motor-joint tiny muted';
      where.textContent = motor.joint || '裸节点';
      where.title = motor.joint
        ? `Gazebo 关节：${motor.joint}`
        : '不在 joint-map：只走线协议的假节点（反馈取内部假 plant）';
      const state = document.createElement('span');
      state.className = motor.linked ? 'ok' : 'warn';
      state.textContent = !motor.linked ? '已断开'
        : (motor.enabled ? '在线·使能' : '在线');
      const link = document.createElement('button');
      link.className = 'secondary';
      link.dataset.action = 'link';
      link.dataset.id = String(motor.id);
      link.dataset.up = motor.linked ? '0' : '1';
      link.textContent = motor.linked ? '断开' : '恢复';
      link.title = motor.linked
        ? '停止该台的一切帧（反馈/SDO 应答），固件将判其离线'
        : '恢复发帧；固件会重新识别该台（RECONNECTED）';
      const remove = document.createElement('button');
      remove.className = 'danger';
      remove.dataset.action = 'remove';
      remove.dataset.id = String(motor.id);
      remove.textContent = '移除';
      remove.title = '从模拟集合删除；再加回来是全新电机（SDO/位置回初值）';
      row.append(label, where, state, link, remove);
      ui.motorList.append(row);
    }
  }

  async function refreshMotorSim() {
    if (!motorPanelActive()) {
      motorSim = { motors: [] };
      renderMotorSim();
      return;
    }
    try {
      const payload = await requestJson('/api/motor-sim/status');
      motorSim = { motors: payload.motors || [] };
      renderMotorSim();
    } catch (error) {
      motorSim = { motors: [] };
      renderMotorSim(`控制通道不可用：${error.message}`);
    }
  }

  async function sendMotorCommand(command, labelText) {
    try {
      await requestJson('/api/motor-sim/command', {
        method: 'POST',
        body: JSON.stringify(command),
      });
      setResult(`${labelText} 已下发`, 'ok');
    } catch (error) {
      setResult(`${labelText} 失败：${error.message}`, 'err');
    } finally {
      await refreshMotorSim();
    }
  }

  function onMotorListClick(event) {
    const button = event.target.closest('button[data-action]');
    if (!button) return;
    const id = Number(button.dataset.id);
    if (button.dataset.action === 'link') {
      const up = button.dataset.up === '1';
      sendMotorCommand({ cmd: 'link', id, up },
        `电机 ${id} ${up ? '恢复' : '断开'}`);
    } else if (button.dataset.action === 'remove') {
      sendMotorCommand({ cmd: 'remove', id }, `移除电机 ${id}`);
    }
  }

  function addMotor() {
    const id = Number(ui.motorAddId.value);
    if (!Number.isInteger(id) || id <= 0) {
      setResult('添加电机需要一个正整数 ID', 'err');
      return;
    }
    const command = { cmd: 'add', id, kind: ui.motorAddKind.value };
    if (command.kind === 'cerb') command.model = ui.motorAddModel.value;
    ui.motorAddId.value = '';
    sendMotorCommand(command, `添加电机 ${id}`);
  }

  async function refreshOptions() {
    ui.refresh.disabled = true;
    try {
      loadDeviceOptions(await requestJson('/api/simulation/options'));
      setResult('设备与模型列表已刷新', 'ok');
    } catch (error) {
      setResult(`读取设备失败：${error.message}`, 'err');
    } finally {
      ui.refresh.disabled = false;
    }
  }

  async function refreshStatus() {
    if (polling) return;
    polling = true;
    try {
      renderStatus(await requestJson('/api/simulation/status'));
      await refreshMotorBackend();
      await refreshMotorSim();
    } catch (error) {
      setResult(`读取仿真状态失败：${error.message}`, 'err');
    } finally {
      polling = false;
    }
  }

  function buildStartRequest() {
    return {
      backend: ui.backend.value,
      controller_mode: ui.mode.value,
      transport: ui.transport.value,
      model: ui.model.value,
      vcan_interface: ui.vcanName.value.trim(),
      sim_interface: ui.simInterface.value,
      driver_interface: ui.driverInterface.value,
      sim_tty: ui.simTty.value,
      driver_tty: ui.driverTty.value,
      physical_bus_isolated: ui.busIsolated.checked,
    };
  }

  async function startSimulation() {
    const request = buildStartRequest();
    if (!request.model) {
      setResult(
        request.backend === 'mujoco'
          ? '该后端没有可用模型：MuJoCo 需要模型自带 mujoco/<模型名>.xml。'
          : '没有可用仿真模型；请先构建并 source robot_sim。',
        'err');
      return;
    }
    if (request.transport !== 'vcan' && !request.physical_bus_isolated) {
      setResult('请先确认物理总线上没有真机电机。', 'err');
      return;
    }
    ui.start.disabled = true;
    setResult('正在启动仿真 launch…');
    try {
      renderStatus(await requestJson('/api/simulation/start', {
        method: 'POST',
        body: JSON.stringify(request),
      }));
      setResult('仿真 launch 进程已启动；就绪情况见下方日志。', 'ok');
      saveOptions();
    } catch (error) {
      if (error.payload && error.payload.status) renderStatus(error.payload.status);
      setResult(error.message, 'err');
    } finally {
      await refreshStatus();
      if (!activeState(latestStatus.state)) ui.start.disabled = false;
    }
  }

  async function stopSimulation() {
    ui.stop.disabled = true;
    setResult('正在停止仿真…');
    try {
      renderStatus(await requestJson('/api/simulation/stop', { method: 'POST' }));
      setResult('仿真已停止', 'ok');
    } catch (error) {
      setResult(`停止失败：${error.message}`, 'err');
    } finally {
      await refreshStatus();
    }
  }

  ui.backend.value = saved.backend || 'gazebo';
  ui.mode.value = saved.controller_mode || 'host';
  ui.transport.value = saved.transport || 'vcan';
  ui.vcanName.value = saved.vcan_interface || 'robot_vcan0';
  for (const select of [ui.mode, ui.transport, ui.simInterface,
    ui.driverInterface, ui.simTty, ui.driverTty]) {
    select.addEventListener('change', () => {
      ui.busIsolated.checked = false;
      syncFields();
    });
  }
  ui.model.addEventListener('change', syncFields);
  // 后端只改变模型候选集，不涉及物理总线，所以不清安全确认勾选
  ui.backend.addEventListener('change', syncFields);
  ui.vcanName.addEventListener('input', saveOptions);
  ui.start.addEventListener('click', startSimulation);
  ui.stop.addEventListener('click', stopSimulation);
  ui.refresh.addEventListener('click', refreshOptions);
  ui.backendStart.addEventListener('click', startMotorBackend);
  ui.backendStop.addEventListener('click', stopMotorBackend);
  ui.motorAdd.addEventListener('click', addMotor);
  ui.motorList.addEventListener('click', onMotorListClick);
  ui.motorAddKind.addEventListener('change', () => {
    ui.motorAddModelField.hidden = ui.motorAddKind.value !== 'cerb';
  });

  syncFields();
  ui.motorAddModelField.hidden = ui.motorAddKind.value !== 'cerb';
  renderStatus(latestStatus);
  renderMotorBackend(motorBackend);
  refreshOptions();
  refreshStatus();
  pollTimer = window.setInterval(refreshStatus, 1800);

  return {
    // 本面板的数据全部来自 HTTP 轮询，与 store 脏键无关；render 只满足面板契约。
    render() {},
    unmount() {
      if (pollTimer !== null) {
        clearInterval(pollTimer);
        pollTimer = null;
      }
    },
  };
}
