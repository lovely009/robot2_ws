<script setup lang="ts">
import { computed, onBeforeUnmount, ref, watch } from 'vue';
import * as bench from '../core/agent';
import { MODEL, store } from '../core/store';
import { benchRevision, runtime, snapshot } from '../runtime';

const props = defineProps<{ active: boolean }>();

const STORAGE_KEY = 'robot2.simulation.options.v1';
const LOG_LINES = 80;
const stateLabels: Record<string, string> = {
  stopped: '未启动',
  running: '运行中',
  failed: '启动失败',
};
const backendLabels: Record<string, string> = { gazebo: 'Gazebo', mujoco: 'MuJoCo' };

interface RouteEntry {
  bus: number;
  enabled: boolean;
  transport: string;
  device: string;
  channel: string;
  endpoint: string;
  ids: string;
}

interface DeviceOptions {
  can_interfaces: { name: string; label?: string; up?: boolean }[];
  serial_ports: { path: string; label: string; accessible: boolean }[];
  usbcanfd_devices: { index: number; label: string; accessible: boolean }[];
  models: string[];
  mujoco_models: string[];
}

function readSaved(): Record<string, any> {
  try {
    return JSON.parse(localStorage.getItem(STORAGE_KEY) || '{}') || {};
  } catch {
    return {};
  }
}

const saved = readSaved();

const backend = ref<string>(saved.backend || 'gazebo');
const mode = ref<string>(['firmware', 'external'].includes(saved.controller_mode) ? saved.controller_mode : 'firmware');
const transport = ref<string>(saved.transport || 'vcan');
const model = ref<string>('');
const vcanName = ref<string>(saved.vcan_interface || 'robot_vcan0');
const simInterface = ref<string>('');
const driverInterface = ref<string>('');
const simTty = ref<string>('');
const driverTty = ref<string>('');
const spawnZ = ref<string>(saved.spawn_z !== undefined ? String(saved.spawn_z) : '0');
const gravity = ref<boolean>(saved.gravity !== false);
const fixedBase = ref<boolean>(saved.fixed_base !== false);
const busIsolated = ref<boolean>(false);

const motorAddId = ref<string>('');
const motorAddKind = ref<string>('cerb');
const motorAddModel = ref<string>('r48');
const motorAddBus = ref<string>('');

const simResult = ref<{ text: string; kind: '' | 'ok' | 'err' }>({ text: '', kind: '' });
const backendResult = ref<{ text: string; kind: '' | 'ok' | 'err' }>({ text: '', kind: '' });
const simLog = ref<string>('等待启动');
const backendLog = ref<string>('等待启动');

const deviceOptions = ref<DeviceOptions>({
  can_interfaces: [],
  serial_ports: [],
  usbcanfd_devices: [],
  models: [],
  mujoco_models: [],
});
const optionsLoaded = ref(false);
const optionsReady = ref(false);
const optionsLoading = ref(false);
const startingSim = ref(false);
const stoppingSim = ref(false);
const startingBackend = ref(false);
const stoppingBackend = ref(false);

const motorSim = ref<{ motors: any[]; channels?: any[] }>({ motors: [], channels: [] });
const motorHint = ref<string>('');
let restoredBackendRoutes = '';

function initRoutes(savedRoutes: any[]): RouteEntry[] {
  const list = Array.isArray(savedRoutes) ? savedRoutes : [
    { bus: 0, transport: 'usbcanfd', device: 0, channel: 0, ids: '*' },
    { bus: 1, transport: 'usbcanfd', device: 0, channel: 1, ids: '' },
  ];
  return [0, 1, 2, 3].map(bus => {
    const r = list.find((item: any) => item.bus === bus);
    return {
      bus,
      enabled: Boolean(r),
      transport: r?.transport || (bus < 2 ? 'usbcanfd' : 'socketcan'),
      device: r?.device !== undefined && r?.device !== null ? String(r.device) : '0',
      channel: String(r?.channel ?? (bus % 2)),
      endpoint: r?.endpoint || '',
      ids: r?.ids ?? '',
    };
  });
}

const routes = ref<RouteEntry[]>(initRoutes(saved.sim_channels));

function activeState(state?: string) {
  return state === 'running';
}

const latestStatus = computed(() => {
  void benchRevision.value;
  return store.bench.simulation || { state: 'stopped' };
});
const motorBackend = computed(() => {
  void benchRevision.value;
  return store.bench.motor_backend || { state: 'stopped' };
});
const simActive = computed(() => activeState(latestStatus.value.state));
const backendActive = computed(() => activeState(motorBackend.value.state));
const routeLocked = computed(() => simActive.value || backendActive.value);

const canEntries = computed(() =>
  (deviceOptions.value.can_interfaces || []).map(item => ({
    value: item.name,
    label: item.label || `${item.name} (${item.up ? 'UP' : 'DOWN'})`,
    disabled: !item.up,
    title: item.up ? '' : '接口当前为 DOWN',
  })),
);

const serialEntries = computed(() =>
  (deviceOptions.value.serial_ports || []).map(item => ({
    value: item.path,
    label: item.accessible ? item.label : `${item.label}（无读写权限）`,
    disabled: !item.accessible,
    title: item.accessible ? '' : 'web_host 用户没有该串口的读写权限',
  })),
);

const usbcanfdEntries = computed(() =>
  (deviceOptions.value.usbcanfd_devices || []).map(d => ({
    value: String(d.index),
    label: d.label + (d.accessible ? '' : '（无读写权限）'),
    disabled: !d.accessible,
  })),
);

const modelEntries = computed(() => {
  const names = backend.value === 'mujoco'
    ? (deviceOptions.value.mujoco_models || [])
    : (deviceOptions.value.models || []);
  return names.map(name => ({ value: name, label: name }));
});

const modelPlaceholder = computed(() =>
  backend.value === 'mujoco'
    ? '该后端没有可用模型（需要模型自带 MJCF）'
    : '选择仿真模型',
);

const motorBusEntries = computed(() =>
  (motorSim.value.channels || []).map((c: any) => ({
    value: String(c.bus),
    label: `MCU CAN${c.bus + 1}`,
    disabled: Boolean(c.error),
  })),
);

function pickValue(current: string, entries: { value: string; disabled?: boolean }[], preferred = ''): string {
  const desired = current || preferred;
  if (entries.some(e => e.value === desired)) return desired;
  if (entries.length) {
    const firstEnabled = entries.find(e => !e.disabled);
    if (firstEnabled) return firstEnabled.value;
  }
  return '';
}

function readRoutes() {
  return routes.value.filter(r => r.enabled).map(r => {
    const route: Record<string, any> = {
      bus: r.bus,
      transport: r.transport,
      ids: r.ids.trim(),
    };
    if (r.transport === 'usbcanfd') {
      route.device = r.device === '' ? null : Number(r.device);
      route.channel = Number(r.channel);
    } else {
      route.endpoint = r.endpoint;
    }
    return route;
  });
}

function saveOptions() {
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify({
      backend: backend.value,
      controller_mode: mode.value,
      transport: transport.value,
      model: model.value,
      vcan_interface: vcanName.value,
      sim_interface: simInterface.value,
      driver_interface: driverInterface.value,
      sim_tty: simTty.value,
      driver_tty: driverTty.value,
      sim_channels: readRoutes(),
      spawn_z: spawnZ.value,
      gravity: gravity.value,
      fixed_base: fixedBase.value,
    }));
  } catch {
    // ignore storage errors
  }
}

function syncFields(clearIsolated = true) {
  if (clearIsolated) busIsolated.value = false;
  if (mode.value !== 'firmware' && transport.value === 'multi') {
    transport.value = 'vcan';
  }
  if (mode.value === 'firmware' && transport.value === 'vcan') {
    transport.value = deviceOptions.value.can_interfaces.some(item => item.up) ? 'socketcan' : 'slcan';
  }
  if (optionsLoaded.value) {
    model.value = pickValue(model.value, modelEntries.value, saved.model || MODEL);
  }
  saveOptions();
}

function syncRouteFields(route: RouteEntry) {
  if (route.transport === 'usbcanfd') {
    route.device = pickValue(route.device, usbcanfdEntries.value, '0');
  } else if (route.transport === 'socketcan') {
    route.endpoint = pickValue(route.endpoint, canEntries.value, '');
  } else {
    route.endpoint = pickValue(route.endpoint, serialEntries.value, '');
  }
  busIsolated.value = false;
  saveOptions();
}

function restoreRoutes(list: any[]) {
  routes.value = initRoutes(list);
  for (const r of routes.value) {
    if (r.transport === 'usbcanfd') {
      r.device = pickValue(r.device, usbcanfdEntries.value, '0');
    } else if (r.transport === 'socketcan') {
      r.endpoint = pickValue(r.endpoint, canEntries.value, '');
    } else {
      r.endpoint = pickValue(r.endpoint, serialEntries.value, '');
    }
  }
}

function loadDeviceOptions(options: DeviceOptions) {
  deviceOptions.value = options;
  optionsLoaded.value = true;
  model.value = pickValue(model.value, modelEntries.value, saved.model || MODEL);
  simInterface.value = pickValue(simInterface.value, canEntries.value, saved.sim_interface || 'can1');
  driverInterface.value = pickValue(driverInterface.value, canEntries.value, saved.driver_interface || 'can0');
  simTty.value = pickValue(simTty.value, serialEntries.value, saved.sim_tty || '');
  const secondAccessibleSerial = (options.serial_ports || []).filter(item => item.accessible)[1];
  driverTty.value = pickValue(
    driverTty.value,
    serialEntries.value,
    saved.driver_tty || (secondAccessibleSerial && secondAccessibleSerial.path) || '',
  );
  if (!vcanName.value || vcanName.value === 'robot_vcan0') {
    vcanName.value = saved.vcan_interface || 'robot_vcan0';
  }
  for (const r of routes.value) {
    if (r.transport === 'usbcanfd') {
      r.device = pickValue(r.device, usbcanfdEntries.value, '0');
    } else if (r.transport === 'socketcan') {
      r.endpoint = pickValue(r.endpoint, canEntries.value, '');
    } else {
      r.endpoint = pickValue(r.endpoint, serialEntries.value, '');
    }
  }
  syncFields(false);
}

watch(
  () => motorBackend.value,
  (status) => {
    if (activeState(status?.state) && status?.config?.channels) {
      const signature = JSON.stringify(status.config.channels);
      if (signature !== restoredBackendRoutes) {
        restoreRoutes(status.config.channels);
        restoredBackendRoutes = signature;
      }
    }
  },
  { immediate: true },
);

const gazebo = computed(() => backend.value === 'gazebo');
const isHost = computed(() => mode.value === 'host');
const modelSummary = computed(() =>
  model.value ? `${model.value}${gazebo.value ? ` · z ${spawnZ.value || '0'}` : ''}` : '--',
);
const modelHint = computed(() =>
  gazebo.value
    ? '生成高度按模型几何的底面定，换模型要核对'
    : 'MuJoCo 的世界参数在 MJCF 生成期固定：生成高度/重力/固定基座不可调',
);

function describeConfig(config: any) {
  if (!config) return '';
  const modeText = ({ host: '主机驱动', external: '外部控制器', firmware: 'STM32 HIL' } as Record<string, string>)[config.controller_mode] || config.controller_mode;
  const backendText = backendLabels[config.backend] || config.backend || '';
  const endpoint = config.vcan_interface || config.sim_interface || config.sim_tty || '';
  const params: string[] = [];
  if (config.gravity === false) params.push('无重力');
  if (config.fixed_base === false) params.push('浮动基座');
  if (typeof config.spawn_z === 'number' && config.spawn_z !== 0) params.push(`z ${config.spawn_z}`);
  return [backendText, modeText, config.transport, endpoint, ...params].filter(Boolean).join(' · ');
}

const simSummary = computed(() => {
  const st = latestStatus.value;
  const state = st.state || 'stopped';
  const label = stateLabels[state] || state;
  return st.config && simActive.value ? `${label} · ${st.config.transport}` : label;
});

const simStateText = computed(() => {
  const st = latestStatus.value;
  const state = st.state || 'stopped';
  const label = stateLabels[state] || state;
  if (st.config && simActive.value) return `${label} · ${describeConfig(st.config)}`;
  if (state === 'failed') return `${label} · exit ${st.returncode}`;
  return `仿真进程${label}`;
});

function describeBackendConfig(config: any) {
  if (!config) return '';
  const endpoint = config.channels?.map((r: any) => `CAN${r.bus + 1}`).join(' / ')
    || config.sim_tty || config.sim_interface || config.motor_sim_control_socket || '';
  return [config.transport, config.model, endpoint].filter(Boolean).join(' · ');
}

const backendSummary = computed(() => {
  const state = motorBackend.value.state || 'stopped';
  return stateLabels[state] || state;
});

const backendStateText = computed(() => {
  const st = motorBackend.value;
  const state = st.state || 'stopped';
  const label = stateLabels[state] || state;
  if (st.config) {
    return `${label} · ${describeBackendConfig(st.config)}${st.auto_started ? ' · 随仿真' : ''}`;
  }
  if (state === 'failed') return `${label} · exit ${st.returncode}`;
  return `电机后端${label}`;
});

async function refreshLogs() {
  await Promise.all([
    (async () => {
      try {
        simLog.value = (await bench.getUnitLog(bench.UNIT_SIMULATION, LOG_LINES)) || '暂无输出';
      } catch (error: any) {
        simLog.value = `日志不可用：${error.message}`;
      }
    })(),
    (async () => {
      try {
        backendLog.value = (await bench.getUnitLog(bench.UNIT_MOTOR_BACKEND, LOG_LINES)) || '暂无输出';
      } catch (error: any) {
        backendLog.value = `日志不可用：${error.message}`;
      }
    })(),
  ]);
}

async function refreshMotorSim() {
  if (!backendActive.value) {
    motorSim.value = { motors: [], channels: [] };
    motorHint.value = '';
    return;
  }
  try {
    const payload = await bench.motorBackendCommand({ cmd: 'status' });
    motorSim.value = { motors: payload.motors || [], channels: payload.channels || [] };
    const channels = motorSim.value.channels || [];
    const failed = channels.filter((c: any) => c.error);
    let hint = `${motorSim.value.motors.length} 台在模拟`;
    if (failed.length) {
      hint += ` · ${failed.map((c: any) => `CAN${c.bus + 1} 故障：${c.error}`).join('；')}`;
    }
    motorHint.value = hint;
    motorAddBus.value = pickValue(motorAddBus.value, motorBusEntries.value, '');
  } catch (error: any) {
    motorSim.value = { motors: [], channels: [] };
    motorHint.value = `控制通道不可用：${error.message}`;
  }
}

async function refreshOptions() {
  if (optionsLoading.value) return;
  optionsLoading.value = true;
  try {
    loadDeviceOptions(await bench.getBenchOptions());
    optionsReady.value = true;
    simResult.value = { text: '设备与模型列表已刷新', kind: 'ok' };
  } catch (error: any) {
    optionsReady.value = false;
    simResult.value = { text: `读取设备失败：${error.message}`, kind: 'err' };
  } finally {
    optionsLoading.value = false;
  }
}

async function startSimulation() {
  const request: Record<string, any> = {
    backend: backend.value,
    controller_mode: mode.value,
    transport: transport.value,
    sim_channels_json: transport.value === 'multi' ? JSON.stringify(readRoutes()) : '',
    model: model.value,
    vcan_interface: vcanName.value.trim(),
    sim_interface: simInterface.value,
    driver_interface: driverInterface.value,
    sim_tty: simTty.value,
    driver_tty: driverTty.value,
    physical_bus_isolated: busIsolated.value,
  };
  if (request.backend === 'gazebo') {
    request.gravity = gravity.value ? 'true' : 'false';
    request.fixed_base = fixedBase.value ? 'true' : 'false';
    request.spawn_z = spawnZ.value.trim();
  }
  if (!request.model) {
    simResult.value = {
      text: request.backend === 'mujoco'
        ? '该后端没有可用模型：MuJoCo 需要模型自带 mujoco/<模型名>.xml。'
        : '没有可用仿真模型；请先构建并 source robot_sim。',
      kind: 'err',
    };
    return;
  }
  if (request.transport !== 'vcan' && !busIsolated.value) {
    simResult.value = { text: '请先确认物理总线上没有真机电机。', kind: 'err' };
    return;
  }
  if (request.backend === 'gazebo') {
    const z = Number(request.spawn_z);
    if (!Number.isFinite(z) || Math.abs(z) > 10) {
      simResult.value = { text: '生成高度必须是 -10 ~ 10 m 之间的数字。', kind: 'err' };
      return;
    }
  }
  startingSim.value = true;
  simResult.value = { text: '正在启动仿真 launch…', kind: '' };
  try {
    await bench.startSimulation(request);
    if (request.backend === 'mujoco' && request.model === 'full_body_robot') {
      runtime.setModel(request.model);
    }
    simResult.value = { text: '仿真 launch 进程已启动；就绪情况见下方日志。', kind: 'ok' };
    saveOptions();
  } catch (error: any) {
    simResult.value = { text: error.message, kind: 'err' };
  } finally {
    startingSim.value = false;
    await refreshLogs();
  }
}

async function stopSimulation() {
  stoppingSim.value = true;
  simResult.value = { text: '正在停止仿真…', kind: '' };
  try {
    await bench.stopSimulation();
    simResult.value = { text: '仿真已停止', kind: 'ok' };
  } catch (error: any) {
    simResult.value = { text: `停止失败：${error.message}`, kind: 'err' };
  } finally {
    stoppingSim.value = false;
    await refreshMotorSim();
    await refreshLogs();
  }
}

async function startMotorBackend() {
  const request: Record<string, any> = {
    model: model.value,
    transport: transport.value,
    physical_bus_isolated: busIsolated.value,
  };
  if (transport.value === 'multi') request.sim_channels_json = JSON.stringify(readRoutes());
  else if (transport.value === 'socketcan') request.sim_interface = simInterface.value;
  else request.sim_tty = simTty.value;

  if (!request.model) {
    backendResult.value = { text: '先在上方「仿真」里选择模型。', kind: 'err' };
    return;
  }
  if (request.transport === 'vcan') {
    backendResult.value = { text: '纯软 vcan 不需要电机后端（总线电机由 motor_sim 节点扮演）。', kind: 'err' };
    return;
  }
  if (!request.physical_bus_isolated) {
    backendResult.value = { text: '请先勾选「物理 CAN 总线仅连接仿真器与控制端」。', kind: 'err' };
    return;
  }
  if (request.transport === 'socketcan' && !request.sim_interface) {
    backendResult.value = { text: '先选择模拟器 SocketCAN 接口。', kind: 'err' };
    return;
  }
  if (request.transport === 'slcan' && !request.sim_tty) {
    backendResult.value = { text: '先选择模拟器串口。', kind: 'err' };
    return;
  }
  if (request.transport === 'multi' && (!readRoutes().length || readRoutes().some(r =>
    r.transport === 'usbcanfd' ? r.device === null : !r.endpoint))) {
    backendResult.value = { text: '请启用通道，并为每路选择可访问的端点。', kind: 'err' };
    return;
  }

  startingBackend.value = true;
  backendResult.value = { text: '正在启动电机后端…', kind: '' };
  try {
    await bench.startMotorBackend(request);
    backendResult.value = { text: '电机后端进程已启动；CAN 就绪情况见下方日志。', kind: 'ok' };
  } catch (error: any) {
    backendResult.value = { text: error.message, kind: 'err' };
  } finally {
    startingBackend.value = false;
    await refreshMotorSim();
    await refreshLogs();
  }
}

async function stopMotorBackend() {
  stoppingBackend.value = true;
  backendResult.value = { text: '正在停止电机后端…', kind: '' };
  try {
    await bench.stopMotorBackend();
    backendResult.value = { text: '电机后端已停止', kind: 'ok' };
  } catch (error: any) {
    backendResult.value = { text: `停止失败：${error.message}`, kind: 'err' };
  } finally {
    stoppingBackend.value = false;
    await refreshMotorSim();
    await refreshLogs();
  }
}

async function sendMotorCommand(command: Record<string, any>, labelText: string) {
  try {
    await bench.motorBackendCommand(command);
    simResult.value = { text: `${labelText} 已下发`, kind: 'ok' };
  } catch (error: any) {
    simResult.value = { text: `${labelText} 失败：${error.message}`, kind: 'err' };
  } finally {
    await refreshMotorSim();
    await refreshLogs();
  }
}

function addMotor() {
  const id = Number(motorAddId.value);
  if (!Number.isInteger(id) || id <= 0) {
    simResult.value = { text: '添加电机需要一个正整数 ID', kind: 'err' };
    return;
  }
  const command: Record<string, any> = { cmd: 'add', id, kind: motorAddKind.value };
  if (motorSim.value.channels?.length) command.bus = Number(motorAddBus.value);
  if (command.kind === 'cerb') command.model = motorAddModel.value;
  motorAddId.value = '';
  void sendMotorCommand(command, `添加电机 ${id}`);
}

function toggleMotorLink(motor: any) {
  const up = !motor.linked;
  void sendMotorCommand({ cmd: 'link', id: motor.id, up }, `电机 ${motor.id} ${up ? '恢复' : '断开'}`);
}

function removeMotor(motor: any) {
  void sendMotorCommand({ cmd: 'remove', id: motor.id }, `移除电机 ${motor.id}`);
}

let pollTimer: ReturnType<typeof setInterval> | null = null;
let pollInFlight = false;
let lastOptionsPollAt = 0;

async function poll() {
  if (pollInFlight || !props.active || !snapshot.value.connected) return;
  pollInFlight = true;
  try {
    const nowMs = Date.now();
    const editingSelect = document.activeElement?.tagName === 'SELECT';
    if (!optionsReady.value || (!simActive.value && !backendActive.value && !document.hidden && !editingSelect && nowMs - lastOptionsPollAt >= 3000)) {
      lastOptionsPollAt = nowMs;
      await refreshOptions();
    }
    await refreshMotorSim();
    if (simActive.value || backendActive.value) {
      await refreshLogs();
    }
  } finally {
    pollInFlight = false;
  }
}

function stopPoll() {
  if (pollTimer !== null) {
    clearInterval(pollTimer);
    pollTimer = null;
  }
}

watch(
  () => [props.active, snapshot.value.connected] as const,
  ([active, connected]) => {
    if (!connected) optionsReady.value = false;
    stopPoll();
    if (active && connected) {
      if (!optionsReady.value) void refreshOptions();
      void refreshMotorSim();
      void refreshLogs();
      pollTimer = setInterval(() => { void poll(); }, 1800);
    }
  },
  { immediate: true },
);

onBeforeUnmount(() => {
  stopPoll();
});
</script>

<template>
  <section v-show="active" data-workbench-panel="sim" class="sim-panel-host">
    <details class="card" data-fold="sim" open>
      <summary>仿真<span class="spacer" /><span id="simSummary" class="sum">{{ simSummary }}</span></summary>
      <div class="sim-panel">
        <div class="sim-grid">
          <label class="sim-field">仿真后端
            <select id="simBackend" v-model="backend" :disabled="simActive" @change="syncFields(false)">
              <option value="gazebo">Gazebo（gz sim）</option>
              <option value="mujoco">MuJoCo（本地窗口）</option>
            </select>
          </label>
          <label class="sim-field">控制端
            <select id="simMode" v-model="mode" :disabled="simActive" @change="syncFields(true)">
              <option value="external">外部控制器</option>
              <option value="firmware">STM32 固件 HIL</option>
            </select>
          </label>
          <label class="sim-field">CAN 接口
            <select id="simTransport" v-model="transport" :disabled="simActive" @change="syncFields(true)">
              <option value="vcan" :disabled="mode === 'firmware'">vcan（纯软件）</option>
              <option value="socketcan">SocketCAN</option>
              <option value="slcan">SLCAN / CANable</option>
              <option value="multi" :disabled="mode !== 'firmware'">多路 CAN（含创芯 USB-CAN FD）</option>
            </select>
          </label>
          <div id="simVcanFields" class="sim-field" :hidden="transport !== 'vcan'">
            <label for="simVcanName">vcan 接口名</label>
            <input id="simVcanName" v-model="vcanName" type="text" maxlength="15" spellcheck="false" :disabled="simActive" @input="saveOptions" />
          </div>
          <label id="simSocketFields" class="sim-field" :hidden="transport !== 'socketcan'">模拟器 CAN
            <select id="simInterface" v-model="simInterface" :disabled="simActive" @change="syncFields(true)">
              <option value="">{{ canEntries.length ? '选择 SocketCAN 接口' : (optionsReady ? '未检测到 SocketCAN 接口' : '扫描 SocketCAN…') }}</option>
              <option v-for="entry in canEntries" :key="entry.value" :value="entry.value" :disabled="entry.disabled" :title="entry.title">{{ entry.label }}</option>
            </select>
          </label>
          <label id="simDriverCanField" class="sim-field" :hidden="transport !== 'socketcan' || !isHost">驱动端 CAN
            <select id="simDriverInterface" v-model="driverInterface" :disabled="simActive" @change="syncFields(true)">
              <option value="">{{ canEntries.length ? '选择 SocketCAN 接口' : (optionsReady ? '未检测到 SocketCAN 接口' : '扫描 SocketCAN…') }}</option>
              <option v-for="entry in canEntries" :key="entry.value" :value="entry.value" :disabled="entry.disabled" :title="entry.title">{{ entry.label }}</option>
            </select>
          </label>
          <label id="simSlcanFields" class="sim-field" :hidden="transport !== 'slcan'">模拟器串口
            <select id="simTty" v-model="simTty" :disabled="simActive" @change="syncFields(true)">
              <option value="">{{ serialEntries.length ? '选择模拟器串口' : (optionsReady ? '未检测到串口' : '扫描串口…') }}</option>
              <option v-for="entry in serialEntries" :key="entry.value" :value="entry.value" :disabled="entry.disabled" :title="entry.title">{{ entry.label }}</option>
            </select>
          </label>
          <label id="simDriverTtyField" class="sim-field" :hidden="transport !== 'slcan' || !isHost">驱动端串口
            <select id="simDriverTty" v-model="driverTty" :disabled="simActive" @change="syncFields(true)">
              <option value="">{{ serialEntries.length ? '选择驱动端串口' : (optionsReady ? '未检测到串口' : '扫描串口…') }}</option>
              <option v-for="entry in serialEntries" :key="entry.value" :value="entry.value" :disabled="entry.disabled" :title="entry.title">{{ entry.label }}</option>
            </select>
          </label>
        </div>
        <div id="simHostBusHint" class="sim-hint tiny muted" :hidden="transport === 'vcan' || !isHost">
          主机驱动闭环的驱动端与模拟器端要接在同一条 CAN 总线上。
        </div>
        <label id="simPhysicalGuard" class="check sim-safety" :hidden="transport === 'vcan'" title="SocketCAN/SLCAN 会接入物理总线；启动前确认该总线上没有真实电机">
          <input id="simBusIsolated" v-model="busIsolated" type="checkbox" :disabled="simActive" />
          物理 CAN 总线仅连接仿真器与控制端，没有真机电机
        </label>
        <div id="simFirmwareHint" class="sim-hint tiny muted" :hidden="mode !== 'firmware'">
          启动后在顶栏选择 STM32 USB CDC 串口，再点「启动并连接」接入 micro-ROS Agent。
        </div>
        <div class="ctrl-row wrap">
          <button id="btnSimStart" class="primary" :disabled="simActive || startingSim" @click="startSimulation">启动仿真</button>
          <button id="btnSimStop" class="danger" :disabled="!simActive || stoppingSim" @click="stopSimulation">停止仿真</button>
          <button id="btnSimRefresh" class="secondary" :disabled="optionsLoading" @click="refreshOptions">刷新设备</button>
          <span class="spacer" />
          <span id="simState" class="tiny muted" aria-live="polite">{{ simStateText }}</span>
        </div>
        <div id="simResult" class="result tiny" :class="simResult.kind" role="status" aria-live="polite">{{ simResult.text }}</div>
        <pre id="simLog" class="sim-log" aria-label="仿真启动日志">{{ simLog }}</pre>
      </div>
    </details>

    <details class="card" data-fold="model" open>
      <summary>模型<span class="spacer" /><span id="simModelSummary" class="sum">{{ modelSummary }}</span></summary>
      <div class="sim-panel">
        <div class="sim-grid">
          <label class="sim-field">加载模型
            <select id="simModel" v-model="model" :disabled="simActive" @change="syncFields(false)">
              <option value="">{{ optionsLoaded ? modelPlaceholder : '读取模型中…' }}</option>
              <option v-for="entry in modelEntries" :key="entry.value" :value="entry.value">{{ entry.label }}</option>
            </select>
          </label>
          <label class="sim-field" title="Gazebo 模型生成高度（m），按模型几何底面定：默认模型底面在 z = 0，故为 0">
            生成高度 z (m)
            <input id="simSpawnZ" v-model="spawnZ" type="number" step="0.001" inputmode="decimal" :disabled="simActive || !gazebo" @input="saveOptions" />
          </label>
        </div>
        <div class="ctrl-row wrap">
          <label class="check tiny" title="第一链连杆是否受 Gazebo 重力（第二链恒关）；位置轨迹验收可关">
            <input id="simGravity" v-model="gravity" type="checkbox" :disabled="simActive || !gazebo" @change="saveOptions" /> 重力
          </label>
          <label class="check tiny" title="把基座焊到世界原点；关闭 = 自由浮动基座">
            <input id="simFixedBase" v-model="fixedBase" type="checkbox" :disabled="simActive || !gazebo" @change="saveOptions" /> 固定基座
          </label>
          <span class="spacer" />
          <span id="simModelHint" class="tiny muted">{{ modelHint }}</span>
        </div>
      </div>
    </details>

    <details class="card" data-fold="motorSim" open>
      <summary>电机后端<span class="spacer" /><span id="motorBackendSummary" class="sum">{{ backendSummary }}</span></summary>
      <div class="sim-panel">
        <div id="motorBackendHint" class="sim-hint tiny muted">
          接口与设备在这里改和在上方「仿真」里改是同一份值；模型仍取上方「仿真」。
          纯软 vcan 的总线电机由 motor_sim 节点扮演，不需要本后端；HIL 启动仿真时
          自动拉起，也可只连板子单独起停（验证固件发现 / 离线 / 重连）。
        </div>
        <div class="sim-grid">
          <label class="sim-field">CAN 接口
            <select id="motorBackendTransport" v-model="transport" aria-label="电机后端 CAN 接口" :disabled="simActive" @change="syncFields(true)">
              <option value="vcan" :disabled="mode === 'firmware'">vcan（纯软件）</option>
              <option value="socketcan">SocketCAN</option>
              <option value="slcan">SLCAN / CANable</option>
              <option value="multi" :disabled="mode !== 'firmware'">多路 CAN（含创芯 USB-CAN FD）</option>
            </select>
          </label>
          <label id="motorBackendCanField" class="sim-field" :hidden="transport !== 'socketcan'">模拟器 CAN
            <select id="motorBackendInterface" v-model="simInterface" aria-label="电机后端模拟器 CAN" :disabled="simActive" @change="syncFields(true)">
              <option value="">{{ canEntries.length ? '选择 SocketCAN 接口' : (optionsReady ? '未检测到 SocketCAN 接口' : '扫描 SocketCAN…') }}</option>
              <option v-for="entry in canEntries" :key="entry.value" :value="entry.value" :disabled="entry.disabled" :title="entry.title">{{ entry.label }}</option>
            </select>
          </label>
          <label id="motorBackendTtyField" class="sim-field" :hidden="transport !== 'slcan'">模拟器串口
            <select id="motorBackendTty" v-model="simTty" aria-label="电机后端模拟器串口" :disabled="simActive" @change="syncFields(true)">
              <option value="">{{ serialEntries.length ? '选择模拟器串口' : (optionsReady ? '未检测到串口' : '扫描串口…') }}</option>
              <option v-for="entry in serialEntries" :key="entry.value" :value="entry.value" :disabled="entry.disabled" :title="entry.title">{{ entry.label }}</option>
            </select>
          </label>
        </div>

        <div id="simMultiFields" class="sim-panel" :hidden="transport !== 'multi'">
          <p class="sim-hint tiny muted">每路独立收发。节点 ID 可写 1-9,12，只加载填写的电机；* 加载其余模型电机，留空只开启通道。不同通道不能重复分配节点。</p>
          <div v-for="route in routes" :key="route.bus" class="sim-grid sim-channel-route" :data-sim-route="route.bus" role="group" :aria-label="`MCU CAN${route.bus + 1}`">
            <label class="check"><input v-model="route.enabled" type="checkbox" data-route-enabled :disabled="routeLocked" @change="syncRouteFields(route)" />MCU CAN{{ route.bus + 1 }}{{ route.bus === 3 ? ' (SPI)' : '' }}</label>
            <label class="sim-field">接口类型
              <select v-model="route.transport" data-route-transport :disabled="routeLocked || !route.enabled" @change="syncRouteFields(route)">
                <option value="usbcanfd">创芯 USB-CAN FD</option>
                <option value="socketcan">SocketCAN</option>
                <option value="slcan">SLCAN / CANable</option>
              </select>
            </label>
            <label class="sim-field" data-route-device-field :hidden="route.transport !== 'usbcanfd'">分析仪
              <select v-model="route.device" data-route-device :disabled="routeLocked || !route.enabled" @change="syncRouteFields(route)">
                <option value="">选择分析仪</option>
                <option v-for="entry in usbcanfdEntries" :key="entry.value" :value="entry.value" :disabled="entry.disabled">{{ entry.label }}</option>
              </select>
            </label>
            <label class="sim-field" data-route-channel-field :hidden="route.transport !== 'usbcanfd'">分析仪通道
              <select v-model="route.channel" data-route-channel :disabled="routeLocked || !route.enabled" @change="syncRouteFields(route)">
                <option value="0">CAN1</option>
                <option value="1">CAN2</option>
              </select>
            </label>
            <label class="sim-field" data-route-endpoint-field :hidden="route.transport === 'usbcanfd'">CAN / 串口端点
              <select v-model="route.endpoint" data-route-endpoint :disabled="routeLocked || !route.enabled" @change="syncRouteFields(route)">
                <option value="">选择端点</option>
                <option v-for="entry in (route.transport === 'socketcan' ? canEntries : serialEntries)" :key="entry.value" :value="entry.value" :disabled="entry.disabled">{{ entry.label }}</option>
              </select>
            </label>
            <label class="sim-field">模拟节点 ID
              <input v-model="route.ids" data-route-ids type="text" placeholder="例如 1-9、12 或 *" spellcheck="false" :disabled="routeLocked || !route.enabled" @change="syncRouteFields(route)" />
            </label>
          </div>
        </div>

        <div class="ctrl-row wrap">
          <button id="btnMotorBackendStart" class="primary" :disabled="backendActive || startingBackend || transport === 'vcan'" @click="startMotorBackend">启动电机后端</button>
          <button id="btnMotorBackendStop" class="danger" :disabled="!backendActive || stoppingBackend" @click="stopMotorBackend">停止电机后端</button>
          <span class="spacer" />
          <span id="motorBackendState" class="tiny muted" aria-live="polite">{{ backendStateText }}</span>
        </div>
        <div id="motorBackendResult" class="result tiny" :class="backendResult.kind" role="status" aria-live="polite">{{ backendResult.text }}</div>
        <pre id="motorBackendLog" class="sim-log" aria-label="电机后端日志">{{ backendLog }}</pre>

        <div id="simMotorPanel" class="sim-motors" :hidden="!backendActive">
          <div class="sim-motors-head">
            <span>电机模拟</span>
            <span class="spacer" />
            <span id="simMotorHint" class="tiny muted" aria-live="polite">{{ motorHint }}</span>
          </div>
          <div class="sim-motor-add">
            <label id="simMotorAddBusField" :hidden="!motorBusEntries.length">通道
              <select id="simMotorAddBus" v-model="motorAddBus">
                <option value="">选择通道</option>
                <option v-for="entry in motorBusEntries" :key="entry.value" :value="entry.value" :disabled="entry.disabled">{{ entry.label }}</option>
              </select>
            </label>
            <label>ID<input id="simMotorAddId" v-model="motorAddId" type="number" min="1" max="127" step="1" inputmode="numeric" /></label>
            <label>协议
              <select id="simMotorAddKind" v-model="motorAddKind">
                <option value="cerb">CERB</option>
                <option value="vaxor">VAXOR</option>
              </select>
            </label>
            <label id="simMotorAddModelField" :hidden="motorAddKind !== 'cerb'">型号
              <select id="simMotorAddModel" v-model="motorAddModel">
                <option value="r48">R48</option>
                <option value="r58">R58</option>
              </select>
            </label>
            <button id="btnSimMotorAdd" class="secondary" @click="addMotor">添加电机</button>
          </div>
          <div id="simMotorList" class="sim-motor-list">
            <div v-for="motor in motorSim.motors" :key="motor.id" :class="motor.linked ? 'sim-motor-row' : 'sim-motor-row offline'">
              <span class="sim-motor-label">#{{ motor.id }} {{ motor.kind.toUpperCase() }}{{ motor.model ? ` ${motor.model.toUpperCase()}` : '' }}{{ Number.isInteger(motor.bus) ? ` · CAN${motor.bus + 1}` : '' }}</span>
              <span class="sim-motor-joint tiny muted" :title="motor.joint ? `Gazebo 关节：${motor.joint}` : '不在 joint-map：只走线协议的假节点（反馈取内部假 plant）'">{{ motor.joint || '裸节点' }}</span>
              <span :class="motor.linked ? 'ok' : 'warn'">{{ !motor.linked ? '已断开' : motor.enabled ? '在线·使能' : '在线' }}</span>
              <button class="secondary" data-action="link" :data-id="motor.id" :data-up="motor.linked ? '0' : '1'" :title="motor.linked ? '停止该台的一切帧（反馈/SDO 应答），固件将判其离线' : '恢复发帧；固件会重新识别该台（RECONNECTED）'" @click="toggleMotorLink(motor)">{{ motor.linked ? '断开' : '恢复' }}</button>
              <button class="danger" data-action="remove" :data-id="motor.id" title="从模拟集合删除；再加回来是全新电机（SDO/位置回初值）" @click="removeMotor(motor)">移除</button>
            </div>
          </div>
        </div>
      </div>
    </details>
  </section>
</template>
