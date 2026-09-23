(() => {
  'use strict';

  const byId = (id) => document.getElementById(id);
  const ui = {
    mode: byId('simMode'),
    transport: byId('simTransport'),
    model: byId('simModel'),
    vcanName: byId('simVcanName'),
    simInterface: byId('simInterface'),
    driverInterface: byId('simDriverInterface'),
    simTty: byId('simTty'),
    driverTty: byId('simDriverTty'),
    vcanFields: byId('simVcanFields'),
    socketFields: byId('simSocketFields'),
    driverCanField: byId('simDriverCanField'),
    slcanFields: byId('simSlcanFields'),
    driverTtyField: byId('simDriverTtyField'),
    physicalGuard: byId('simPhysicalGuard'),
    hostBusHint: byId('simHostBusHint'),
    busIsolated: byId('simBusIsolated'),
    firmwareHint: byId('simFirmwareHint'),
    start: byId('btnSimStart'),
    stop: byId('btnSimStop'),
    refresh: byId('btnSimRefresh'),
    summary: byId('simSummary'),
    state: byId('simState'),
    result: byId('simResult'),
    log: byId('simLog'),
  };

  const STORAGE_KEY = 'robot2.simulation.options.v1';
  const stateLabels = {
    stopped: '未启动',
    starting: '启动中',
    running: '启动进程运行中',
    stopping: '停止中',
    failed: '启动失败',
  };
  let deviceOptions = { can_interfaces: [], serial_ports: [], models: [] };
  let latestStatus = { state: 'stopped', log: [] };
  let polling = false;

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
      const error = new Error(payload.error || `HTTP ${response.status}`);
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

  function loadDeviceOptions(options) {
    deviceOptions = options;
    const models = (options.models || []).map((model) => ({
      value: model,
      label: model,
    }));
    replaceOptions(ui.model, models, '选择仿真模型', saved.model || 'zhoubuguanjie_youbi_001');

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
    saveOptions();
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
    const endpoint = config.vcan_interface
      || config.sim_interface
      || config.sim_tty
      || '';
    return `${mode} · ${config.transport} · ${endpoint}`;
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
      ui.mode, ui.transport, ui.model, ui.vcanName, ui.simInterface,
      ui.driverInterface, ui.simTty, ui.driverTty, ui.busIsolated,
    ]) {
      control.disabled = active;
    }

    const lines = latestStatus.log || [];
    ui.log.textContent = lines.length ? lines.join('\n') : '等待启动';
    ui.log.scrollTop = ui.log.scrollHeight;
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
    } catch (error) {
      setResult(`读取仿真状态失败：${error.message}`, 'err');
    } finally {
      polling = false;
    }
  }

  function buildStartRequest() {
    return {
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
      setResult('没有可用仿真模型；请先构建并 source robot_arm_gazebo。', 'err');
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
  ui.vcanName.addEventListener('input', saveOptions);
  ui.start.addEventListener('click', startSimulation);
  ui.stop.addEventListener('click', stopSimulation);
  ui.refresh.addEventListener('click', refreshOptions);

  syncFields();
  renderStatus(latestStatus);
  refreshOptions();
  refreshStatus();
  window.setInterval(refreshStatus, 1800);
})();
