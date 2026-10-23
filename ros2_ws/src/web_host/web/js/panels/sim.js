// 仿真启动器面板：控制端 / CAN 接口 / 模型 / 端点选择，启动与停止
// robot_sim 的 simulation.launch.py。状态/日志来自 store.bench（bench_manager 的
// /bench/status 话题，1 Hz）；命令走 /bench/* 服务（core/agent.js 的台架服务层），
// 响应里的快照会立刻入 store——面板不再自己轮询状态，只按 DIRTY.BENCH 渲染。
// 切屏后状态与日志由 store 里的快照整段恢复，日志不重复追加。
// 搬自重构前的 web/simulation.js（页面级 IIFE 单例），只改挂载方式：ui 映射与
// 全部逻辑移进 mount 作用域（切屏会反复挂载，模块级 ui 会指向已卸载的 DOM），
// 电机表轮询由 mount 持有句柄并在 unmount 清理——旧代码是页面级单例，切屏会重复注册。
import { $ } from '../core/format.js';
// 默认模型名与臂视图共用一份（core/store.js 的 MODEL），别在这里再写一个字符串。
import { store, DIRTY, MODEL } from '../core/store.js';
import * as bench from '../core/agent.js';

const STORAGE_KEY = 'robot2.simulation.options.v1';
// 四态由后端派生（unit 状态），页面只负责翻译；`external` 随「launch 内嵌模拟器」
// 那条路径一起删了——现在进程归属在 systemd，只有「本页面管得着」这一种。
const stateLabels = {
  stopped: '未启动',
  running: '运行中',
  failed: '启动失败',
};
const LOG_LINES = 80;
const backendLabels = { gazebo: 'Gazebo', mujoco: 'MuJoCo' };

export function mount(root, ctx) {
  const ui = {
    backend: $('simBackend'),
    mode: $('simMode'),
    transport: $('simTransport'),
    model: $('simModel'),
    modelSummary: $('simModelSummary'),
    modelHint: $('simModelHint'),
    spawnZ: $('simSpawnZ'),
    gravity: $('simGravity'),
    fixedBase: $('simFixedBase'),
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
    backendTransport: $('motorBackendTransport'),
    backendInterface: $('motorBackendInterface'),
    backendTty: $('motorBackendTty'),
    backendCanField: $('motorBackendCanField'),
    backendTtyField: $('motorBackendTtyField'),
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
  let latestStatus = { state: 'stopped' };
  let motorBackend = { state: 'stopped' };
  let motorSim = { motors: [] };
  let pollTimer = null;

  // 日志不进状态快照，按需拉（见 core/agent.js 的 getUnitLog）。文本活在挂载作用域里，
  // 切屏重挂会重新拉一次。
  function paintLog(element, text, error) {
    element.textContent = error ? `日志不可用：${error}` : (text || '暂无输出');
    element.scrollTop = element.scrollHeight;
  }

  async function refreshLogs() {
    const targets = [
      [bench.UNIT_SIMULATION, ui.log],
      [bench.UNIT_MOTOR_BACKEND, ui.backendLog],
    ];
    await Promise.all(targets.map(async ([unit, element]) => {
      try {
        paintLog(element, await bench.getUnitLog(unit, LOG_LINES));
      } catch (error) {
        paintLog(element, '', error.message);
      }
    }));
  }

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
        spawn_z: ui.spawnZ.value,
        gravity: ui.gravity.checked,
        fixed_base: ui.fixedBase.checked,
      }));
    } catch (_error) {
      // The simulation controls remain usable when browser storage is disabled.
    }
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

  // 生成高度/重力/固定基座只有 Gazebo 有实现（MuJoCo 在 MJCF 生成期固定，launch
  // 侧也拒绝）；运行中连同其它控件一起锁住。摘要栏常显当前模型。
  function syncModelParams(active = false) {
    const gazebo = ui.backend.value === 'gazebo';
    for (const control of [ui.spawnZ, ui.gravity, ui.fixedBase]) {
      control.disabled = active || !gazebo;
    }
    ui.modelHint.textContent = gazebo
      ? '生成高度按模型几何的底面定，换模型要核对'
      : 'MuJoCo 的世界参数在 MJCF 生成期固定：生成高度/重力/固定基座不可调';
    ui.modelSummary.textContent = ui.model.value
      ? `${ui.model.value}${gazebo ? ` · z ${ui.spawnZ.value || '0'}` : ''}`
      : '--';
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
    replaceOptions(ui.backendInterface, cans, '选择 SocketCAN 接口', '');

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
    replaceOptions(ui.backendTty, serials, '选择模拟器串口', '');

    if (!ui.vcanName.value || ui.vcanName.value === 'robot_vcan0') {
      ui.vcanName.value = saved.vcan_interface || 'robot_vcan0';
    }
    syncFields();
  }

  // firmware 模式必须走物理总线，两个传输选择器的 vcan 项一起禁掉。
  function syncVcanOption(select, disabled) {
    const option = [...select.options].find((item) => item.value === 'vcan');
    if (option) option.disabled = disabled;
  }

  // 电机后端面板的接口控件不是第二份配置，只是同一份值的第二个入口：值由上方
  // 单向推下来（反向在上面的 change 监听里推回去），可见项也照上方那套规则走。
  // 单向推不会来回弹——程序化赋值不触发 change。
  function syncBackendInterface() {
    ui.backendTransport.value = ui.transport.value;
    ui.backendInterface.value = ui.simInterface.value;
    ui.backendTty.value = ui.simTty.value;
    const transport = ui.transport.value;
    ui.backendCanField.hidden = transport !== 'socketcan';
    ui.backendTtyField.hidden = transport !== 'slcan';
    syncVcanOption(ui.backendTransport, ui.mode.value === 'firmware');
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

    syncVcanOption(ui.transport, mode === 'firmware');
    if (mode === 'firmware' && transport === 'vcan') {
      ui.transport.value = deviceOptions.can_interfaces.some((item) => item.up)
        ? 'socketcan'
        : 'slcan';
      syncFields();
      return;
    }
    // 后端决定模型候选集：切换时重列，当前模型仍在新列表里就保留
    if (optionsLoaded) refreshModelOptions();
    syncModelParams();
    saveOptions();
    syncBackendInterface();
    // vcan（纯软）下电机后端不可用；切传输时立即反映，不等下一次轮询
    ui.backendStart.disabled = activeState(motorBackend.state)
      || !motorBackendStartable();
  }

  // unit 状态只有 stopped / running / failed 三种——起停是同步的服务调用，
  // 响应回来时状态已经落定，不存在「启动中」这种中间态。
  function activeState(state) {
    return state === 'running';
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
    // 模型参数只在偏离默认值时报出来（生成高度错了正是要一眼看见的那类事）
    const params = [];
    if (config.gravity === false) params.push('无重力');
    if (config.fixed_base === false) params.push('浮动基座');
    if (typeof config.spawn_z === 'number' && config.spawn_z !== 0) {
      params.push(`z ${config.spawn_z}`);
    }
    return [backend, mode, config.transport, endpoint, ...params]
      .filter(Boolean).join(' · ');
  }

  function renderStatus(status) {
    latestStatus = status || { state: 'stopped' };
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
      ui.backendTransport, ui.backendInterface, ui.backendTty,
    ]) {
      control.disabled = active;
    }
    syncModelParams(active);
  }

  // ---- 电机后端（独立进程：fdcan_motor_sim.py）------------------------
  // HIL 启动仿真时由后端默认联动拉起（auto_started），停止时联动收起；
  // 也可脱离仿真单独起停（只连板子验证固件行为）。参数取仿真抽屉的选择。
  function describeBackendConfig(config) {
    if (!config) return '';
    const endpoint = config.sim_tty || config.sim_interface
      || config.motor_sim_control_socket || '';
    return [config.transport, config.model, endpoint].filter(Boolean).join(' · ');
  }

  function motorBackendStartable() {
    return ui.transport.value !== 'vcan';
  }

  function renderMotorBackend(status) {
    motorBackend = status || { state: 'stopped' };
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
  }

  function setBackendResult(message, kind = '') {
    ui.backendResult.textContent = message;
    ui.backendResult.className = `result tiny${kind ? ` ${kind}` : ''}`;
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
      setBackendResult('先选择模拟器 SocketCAN 接口。', 'err');
      return;
    }
    if (request.transport === 'slcan' && !request.sim_tty) {
      setBackendResult('先选择模拟器串口。', 'err');
      return;
    }
    ui.backendStart.disabled = true;
    setBackendResult('正在启动电机后端…');
    try {
      await bench.startMotorBackend(request);
      setBackendResult('电机后端进程已启动；CAN 就绪情况见下方日志。', 'ok');
    } catch (error) {
      setBackendResult(error.message, 'err');
    } finally {
      await refreshMotorSim();
      await refreshLogs();
    }
  }

  async function stopMotorBackend() {
    ui.backendStop.disabled = true;
    setBackendResult('正在停止电机后端…');
    try {
      await bench.stopMotorBackend();
      setBackendResult('电机后端已停止', 'ok');
    } catch (error) {
      setBackendResult(`停止失败：${error.message}`, 'err');
    } finally {
      await refreshMotorSim();
      await refreshLogs();
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
      const payload = await bench.motorBackendCommand({cmd: 'status'});
      motorSim = { motors: payload.motors || [] };
      renderMotorSim();
    } catch (error) {
      motorSim = { motors: [] };
      renderMotorSim(`控制通道不可用：${error.message}`);
    }
  }

  async function sendMotorCommand(command, labelText) {
    try {
      await bench.motorBackendCommand(command);
      setResult(`${labelText} 已下发`, 'ok');
    } catch (error) {
      setResult(`${labelText} 失败：${error.message}`, 'err');
    } finally {
      await refreshMotorSim();
      await refreshLogs();
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

  let optionsReady = false;
  let optionsLoading = false;
  async function refreshOptions() {
    if (optionsLoading) return;
    optionsLoading = true;
    ui.refresh.disabled = true;
    try {
      loadDeviceOptions(await bench.getBenchOptions());
      optionsReady = true;
      setResult('设备与模型列表已刷新', 'ok');
    } catch (error) {
      optionsReady = false;
      setResult(`读取设备失败：${error.message}`, 'err');
    } finally {
      optionsLoading = false;
      ui.refresh.disabled = false;
    }
  }

  // 仿真/电机后端的状态与日志全部来自 store.bench（话题 + 命令响应的快照）；
  // 只在 DIRTY.BENCH 脏帧里整段重绘，不做状态轮询。
  function renderFromStore() {
    renderStatus(store.bench.simulation || { state: 'stopped' });
    renderMotorBackend(store.bench.motor_backend || { state: 'stopped' });
  }

  // 只在有会话在跑时跟日志：跑起来才需要看进展，停着的时候没必要每秒拉一遍。
  async function poll() {
    if (!store.conn.connected) { optionsReady = false; return; }
    if (!optionsReady) await refreshOptions();
    await refreshMotorSim();
    if (activeState((store.bench.simulation || {}).state)
        || activeState((store.bench.motor_backend || {}).state)) {
      await refreshLogs();
    }
  }

  function buildStartRequest() {
    const request = {
      backend: ui.backend.value,
      controller_mode: ui.mode.value,
      transport: ui.transport.value,
      model: ui.model.value,
      vcan_interface: ui.vcanName.value.trim(),
      sim_interface: ui.simInterface.value,
      driver_interface: ui.driverInterface.value,
      sim_tty: ui.simTty.value,
      driver_tty: ui.driverTty.value,
      // 必须进请求：bench_manager 拿它当非 vcan 的放行条件（见 simulation_manager
      // 的校验）。页面自己拦一道只是快速反馈，不等于服务端收到过。
      physical_bus_isolated: ui.busIsolated.checked,
    };
    // 模型参数只对 Gazebo 有意义（MuJoCo 侧由服务端显式拒绝，见 syncModelParams）；
    // 空串 = 未给该参数（缺省不进命令行）。
    if (request.backend === 'gazebo') {
      request.gravity = ui.gravity.checked ? 'true' : 'false';
      request.fixed_base = ui.fixedBase.checked ? 'true' : 'false';
      request.spawn_z = ui.spawnZ.value.trim();
    }
    return request;
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
    if (request.transport !== 'vcan' && !ui.busIsolated.checked) {
      setResult('请先确认物理总线上没有真机电机。', 'err');
      return;
    }
    if (request.backend === 'gazebo') {
      const spawnZ = Number(request.spawn_z);
      if (!Number.isFinite(spawnZ) || Math.abs(spawnZ) > 10) {
        setResult('生成高度必须是 -10 ~ 10 m 之间的数字。', 'err');
        return;
      }
    }
    ui.start.disabled = true;
    setResult('正在启动仿真 launch…');
    try {
      await bench.startSimulation(request);
      setResult('仿真 launch 进程已启动；就绪情况见下方日志。', 'ok');
      saveOptions();
    } catch (error) {
      setResult(error.message, 'err');
    } finally {
      await refreshLogs();
      if (!activeState((store.bench.simulation || {}).state)) {
        ui.start.disabled = false;
      }
    }
  }

  async function stopSimulation() {
    ui.stop.disabled = true;
    setResult('正在停止仿真…');
    try {
      await bench.stopSimulation();
      setResult('仿真已停止', 'ok');
    } catch (error) {
      setResult(`停止失败：${error.message}`, 'err');
    } finally {
      await refreshMotorSim();
      await refreshLogs();
    }
  }

  ui.backend.value = saved.backend || 'gazebo';
  ui.mode.value = saved.controller_mode || 'host';
  ui.transport.value = saved.transport || 'vcan';
  ui.vcanName.value = saved.vcan_interface || 'robot_vcan0';
  ui.spawnZ.value = saved.spawn_z !== undefined ? String(saved.spawn_z) : '0';
  ui.gravity.checked = saved.gravity !== false;
  ui.fixedBase.checked = saved.fixed_base !== false;
  for (const select of [ui.mode, ui.transport, ui.simInterface,
    ui.driverInterface, ui.simTty, ui.driverTty]) {
    select.addEventListener('change', () => {
      ui.busIsolated.checked = false;
      syncFields();
    });
  }
  // 面板里的接口控件是同一份值的第二个入口：写回上方，其余连带影响（清物理总线
  // 确认、重列模型、落盘）全部交给 syncFields，与在上方改完全一致。
  for (const [primary, mirror] of [
    [ui.transport, ui.backendTransport],
    [ui.simInterface, ui.backendInterface],
    [ui.simTty, ui.backendTty],
  ]) {
    mirror.addEventListener('change', () => {
      primary.value = mirror.value;
      ui.busIsolated.checked = false;
      syncFields();
    });
  }
  ui.model.addEventListener('change', syncFields);
  // 后端只改变模型候选集，不涉及物理总线，所以不清安全确认勾选
  ui.backend.addEventListener('change', syncFields);
  ui.vcanName.addEventListener('input', saveOptions);
  ui.spawnZ.addEventListener('input', () => {
    syncModelParams();
    saveOptions();
  });
  ui.gravity.addEventListener('change', saveOptions);
  ui.fixedBase.addEventListener('change', saveOptions);
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
  renderFromStore();
  refreshOptions();
  refreshMotorSim();
  refreshLogs();
  // 电机表只有一问一答的模拟器查询（无对应话题），保持低频轮询；有会话在跑时顺带
  // 跟一次日志。仿真/电机后端的状态由 DIRTY.BENCH 脏帧重绘，不走轮询。
  pollTimer = window.setInterval(poll, 1800);

  return {
    render(dirty) {
      if (!store.conn.connected) optionsReady = false;
      else if (!optionsReady) refreshOptions();
      if (dirty.has(DIRTY.BENCH)) renderFromStore();
    },
    unmount() {
      if (pollTimer !== null) {
        clearInterval(pollTimer);
        pollTimer = null;
      }
    },
  };
}
