import { ref, shallowRef } from "vue";
import type { WorkbenchSnapshot, PanelId, MountedPanel, EventLogEntry } from "./contracts";
import { PanelScheduler } from "./panel_scheduler";
import * as data from "./core/store";
import * as ros from "./core/ros";
import * as agent from "./core/agent";
import * as device from "./core/device";
import * as format from "./core/format";
import * as actions from "./core/playback";
import * as gravity from "./core/gravity_control";
import * as jointControl from "./core/joint_control";
import * as jointList from "./panels/jointList";
import * as arm from "./panels/arm";
import * as readout from "./panels/readout";
import * as actionLibrary from "./panels/actionLibrary";
import * as curve from "./panels/curve";
import * as imu from "./panels/imu";
import { createMotionPreview } from "./panels/motion_preview";
import { pinia } from "./stores";
import { useLeaseStore } from "./stores/lease";

export const initialized = ref(false);
export const startupError = ref("");
export const notice = ref("");
export const configuredRosUrl = ref("");
export const eventLog = shallowRef<readonly EventLogEntry[]>([]);
export const benchRevision = ref(0);
export const paramRevision = ref(0);
export const statusRevision = ref(0);
export const motionRevision = ref(0);

export const snapshot = shallowRef<WorkbenchSnapshot>({
  firmwareConnectionKey: '',
  connected: false,
  connectionText: "未连接",
  agent: "stopped",
  agentDevice: "",
  agentError: "",
  canStopAgent: false,
  model: "",
  selectedId: 0,
  motors: [],
  total: 0,
  online: 0,
  active: 0,
  fault: 0,
  motionAt: null,
  communication: { link: '未连接', clock: '未知', authority: '未知', detail: '' },
  gravity: { enabled: null, active: false, blocked: '等待主机状态', pending: false },
  session: "未获台架状态",
  mappingError: "",
});

const panelModules: Record<PanelId, { mount(root: HTMLElement, ctx: any): MountedPanel }> = {
  jointList,
  arm,
  readout,
  actionLibrary,
  curve,
  imu,
};

export function installTestHooks() {
  window.__workbench = {
    data,
    store: data.store,
    markDirty: data.markDirty,
    DIRTY: data.DIRTY,
    ros,
    jointList,
    arm,
    createMotionPreview,
  };
}

let starting: Promise<void> | undefined;
let resolveReady: () => void;
let rejectReady: (error: unknown) => void;
const ready = new Promise<void>((resolve, reject) => {
  resolveReady = resolve;
  rejectReady = reject;
});
const panelIds: PanelId[] = [
  "jointList",
  "arm",
  "readout",
  "actionLibrary",
  "curve",
  "imu",
];
const mounted = new Set<PanelId>();
const requiredPanels = new Set<PanelId>(["jointList"]);
let requiredPanelError = "";
let resolvePanels: () => void;
const panelsReady = new Promise<void>((resolve) => {
  resolvePanels = resolve;
});
let disposed = false;
let generation = 0;
let navigation: (screen: string) => void = () => {};
let frame = 0;
let lastMotionCount = 0;
let lastMotionAt: number | null = null;
let lastMotionBumpAt = 0;
let wasConnected = false;
let feedbackSyncKey = '';
let feedbackSyncPending = false;
let feedbackSyncFailed = false;
let feedbackSyncAt = 0;
const scheduler = new PanelScheduler((id, error) => {
  notice.value = `面板 ${id}：${error instanceof Error ? error.message : String(error)}`;
  console.error(notice.value, error);
});

function updateSnapshot() {
  const s = data.store;
  if (s.motionCount !== lastMotionCount) {
    lastMotionCount = s.motionCount;
    lastMotionAt = Date.now();
  }
  if (!s.conn.connected) lastMotionAt = null;
  const simulation = s.bench.simulation;
  snapshot.value = {
    firmwareConnectionKey: `${generation}:${s.configSessionId ?? 0}`,
    connected: s.conn.connected,
    connectionText: s.conn.text,
    agent: s.conn.agent,
    agentDevice: s.conn.agentDevice,
    agentError: s.conn.agentError,
    canStopAgent: s.conn.canStop,
    model: data.MODEL,
    selectedId: s.selectedId,
    motors: [...new Set<number>([...s.motors.keys(), ...s.paramCache.keys()])]
      .sort((a, b) => a - b)
      .map(id => ({ id, model: s.motors.get(id)?.topo?.model_name || '' })),
    ...data.countMotors(),
    motionAt: lastMotionAt,
    communication: agent.describeAgentStatus(s),
    gravity: gravity.describeGravityControl(s, data.MODEL),
    session: simulation
      ? `仿真进程：${simulation.state || "未知"}${s.conn.agent === "running" ? " · Agent 运行中" : ""}`
      : s.conn.agent === "running"
        ? "Agent 运行中 · 控制对象待核对"
        : "未获台架状态",
    mappingError:
      data.jointMapSource.kind === "unavailable"
        ? `模型无设备映射：${data.jointMapSource.error}`
        : data.jointMapSource.kind === "topology"
          ? jointControl.controlSessionError(s, data.MODEL)
            ? `${data.store.modelTransmissions.groups.reduce((count: number, group: any) => count + group.motors.length, 0)} 台电机映射已加载；传动参数待标定，真机控制未开放`
            : `主机控制会话已接入 ${s.transmissionStatus.data.joint_names.length} 个关节；各关节按 MCU 使能与反馈状态开放`
          : "",
  };
}

function dirty(batch: Set<string>, now = Date.now()) {
  if (disposed) return;
  const s = data.store;
  if (wasConnected !== s.conn.connected) {
    generation++;
    wasConnected = s.conn.connected;
  }
  const feedbackKey = `${generation}:${data.MODEL}`;
  const feedbackNeedsModel = s.bench.feedback && !s.bench.feedback.desired_model;
  if (s.conn.connected && !feedbackSyncPending &&
      (feedbackKey !== feedbackSyncKey ||
        ((feedbackNeedsModel || feedbackSyncFailed) && now - feedbackSyncAt > 5000))) {
    const epoch = generation, model = data.MODEL;
    feedbackSyncKey = feedbackKey;
    feedbackSyncAt = now;
    feedbackSyncPending = true;
    void agent.setFeedbackModel(model).then(() => {
      if (epoch === generation && model === data.MODEL) {
        feedbackSyncFailed = false;
      }
    }).catch((error: unknown) => {
      if (epoch === generation && model === data.MODEL) {
        feedbackSyncFailed = true;
        console.warn('关节反馈模型同步失败', error);
      }
    }).finally(() => { feedbackSyncPending = false; });
  }
  if (s.conn.connected && !s.conn.benchLoaded) {
    s.conn.benchLoaded = true;
    void agent.refreshAgentControls();
  }
  if (s.conn.connected && !s.conn.autoLoaded) {
    s.conn.autoLoaded = true;
    const epoch = generation;
    void ros
      .waitSysStatus()
      .then((seen: boolean) => {
        if (seen && !disposed && epoch === generation && s.conn.connected)
          return device.loadParams();
      })
      .catch((error: unknown) => {
        notice.value = String(error);
      });
  }
  if (!s.selectedId && s.motors.size) {
    s.selectedId = s.motors.keys().next().value!;
    batch.add(data.DIRTY.SELECTED);
  }
  if (batch.has(data.DIRTY.EVENTS)) eventLog.value = s.events.slice();
  if (batch.has(data.DIRTY.BENCH)) benchRevision.value++;
  if (batch.has(data.DIRTY.PARAMS) || batch.has(data.DIRTY.SELECTED) || batch.has(data.DIRTY.TOPOLOGY)) {
    paramRevision.value++;
  }
  if (batch.has(data.DIRTY.STATUS) || batch.has(data.DIRTY.CONN) || batch.has(data.DIRTY.COMMAND) || batch.has(data.DIRTY.POLICY)) {
    statusRevision.value++;
  }
  if (batch.has(data.DIRTY.MOTION) && now - lastMotionBumpAt >= 200) {
    lastMotionBumpAt = now;
    motionRevision.value++;
  }
  updateSnapshot();
  scheduler.invalidate(batch);
}

async function start() {
  if (starting) return starting;
  starting = (async () => {
    try {
      installTestHooks();
      const leaseStore = useLeaseStore(pinia);
      ros.setBffObserver({
        onConfigure: (cfg) => leaseStore.configureTransport(cfg),
        onConnected: (hello) => {
          leaseStore.manualReleased = false;
          if (hello?.lease) leaseStore.applyLeaseState(hello.lease);
          void leaseStore.tryAutoAcquire(hello?.control_mode || 'bench');
        },
        onLease: (leaseState) => leaseStore.applyLeaseState(leaseState),
        onRtt: (rttMs) => leaseStore.setRtt(rttMs),
      });
      const configuration = await fetch("/api/runtime", { cache: "no-store" });
      if (disposed) return;
      if (!configuration.ok) throw new Error('无法读取工作台运行配置，控制保持关闭');
      const runtimeConfig = await configuration.json();
      if (runtimeConfig.rosbridge_port) configuredRosUrl.value = `${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.hostname}:${runtimeConfig.rosbridge_port}`;
      ros.configureControl(runtimeConfig);
      if (runtimeConfig.robot_model) data.setModel(runtimeConfig.robot_model);
      const render = (now: number) => {
        if (!disposed) {
          const wallNow = Date.now();
          const batch = data.consumeDirty(wallNow);
          if (batch) dirty(batch, wallNow);
          scheduler.flush(now);
          frame = requestAnimationFrame(render);
        }
      };
      frame = requestAnimationFrame(render);
      await data.loadModelJoints();
      if (disposed) return;
      eventLog.value = data.store.events.slice();
      updateSnapshot();
      resolveReady();
      await panelsReady;
      if (disposed) return;
      if (requiredPanelError) throw new Error(requiredPanelError);
      initialized.value = true;
      ros.connect();
    } catch (error) {
      startupError.value =
        error instanceof Error ? error.message : String(error);
      rejectReady(error);
    }
  })();
  return starting;
}

async function mountPanel(
  id: PanelId,
  root: HTMLElement,
  visible: () => boolean,
  signal: AbortSignal,
) {
  try {
    return await mountPanelContent(id, root, visible, signal);
  } catch (error) {
    if (requiredPanels.has(id))
      requiredPanelError = `关键面板 ${id} 初始化失败，未连接 ROS：${String(error)}`;
    throw error;
  } finally {
    mounted.add(id);
    if (panelIds.every((panelId) => mounted.has(panelId))) resolvePanels();
  }
}

async function mountPanelContent(
  id: PanelId,
  root: HTMLElement,
  visible: () => boolean,
  signal: AbortSignal,
) {
  await ready;
  if (signal.aborted || disposed) return () => {};
  const module = panelModules[id];
  if (!module) throw new Error(`未知面板：${id}`);
  const instance: MountedPanel = module.mount(root, {
    store: data.store,
    markDirty: data.markDirty,
    ui: {
      selectMotor: runtime.selectMotor,
      setModel(name: string) {
        data.setModel(name);
      },
      gotoScreen(targetId: string) {
        navigation(targetId);
      },
      toast(text: string) {
        notice.value = text;
      },
    },
  });
  const release = scheduler.add(id, instance, visible);
  scheduler.invalidate(new Set(Object.values(data.DIRTY) as string[]));
  return () => {
    release();
  };
}

export const runtime = {
  start,
  mountPanel,
  setModel(name: string) {
    if (disposed) return;
    data.setModel(name);
  },
  async setGravityEnabled(enabled: boolean) {
    if (!initialized.value || disposed) throw new Error('工作区尚未就绪');
    await gravity.setGravityEnabled(enabled);
  },
  async readFirmwareParameter(name: string) {
    if (!initialized.value || disposed) throw new Error('工作区尚未就绪');
    return device.readFirmwareParameter(name);
  },
  clearEvents() {
    if (disposed) return;
    data.store.events.length = 0;
    data.markDirty(data.DIRTY.EVENTS);
  },
  selectMotor(id: number) {
    if (disposed) return;
    data.store.selectedId = id;
    data.markDirty(data.DIRTY.SELECTED);
  },
  setNavigation(callback: (screen: string) => void) {
    navigation = callback;
  },
  wake() {
    if (!initialized.value || disposed) return;
    scheduler.syncVisibility();
    scheduler.retryFailed();
    scheduler.invalidate(
      new Set(Object.values(data.DIRTY) as string[]),
    );
    window.dispatchEvent(new Event("resize"));
  },
  connect(url: string) {
    if (!initialized.value || disposed) return;
    const trimmed = url.trim();
    const input = document.getElementById("rosUrl") as HTMLInputElement | null;
    if (input) input.value = trimmed;
    lastMotionAt = null;
    localStorage.setItem(format.ROS_URL_STORAGE_KEY, trimmed);
    generation++;
    ros.connect(trimmed);
  },
  async startAgent() {
    if (initialized.value) await agent.startAgent();
  },
  async stopAgent() {
    if (initialized.value) await agent.stopAgent();
  },
  async refreshAgentControls(background = false) {
    if (initialized.value) await agent.refreshAgentControls(background);
  },
  setActiveChannels(channels: string[]) {
    ros.setActiveChannels(channels);
  },
  stopMotion() {
    if (!initialized.value || !ros.isConnected()) return false;
    const sent = ros.pub(format.TOPICS.stop ? 'stop' : 'stop', {}) || actions.stopAll();
    notice.value = sent
      ? "已请求全部软停（停用运动控制器）；请核对设备反馈"
      : "停止请求未发送";
    return sent;
  },
  dispose() {
    disposed = true;
    generation++;
    cancelAnimationFrame(frame);
    resolveReady();
    resolvePanels();
    scheduler.dispose();
    data.onDirty(null);
    ros.disconnect();
  },
};
