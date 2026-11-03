import { ref, shallowRef } from "vue";
import type { WorkbenchSnapshot, PanelId, MountedPanel } from "./contracts";
import { PanelScheduler } from "./panel_scheduler";

export const initialized = ref(false);
export const startupError = ref("");
export const notice = ref("");
export const configuredRosUrl = ref("");
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
  communication: {link: '未连接', clock: '未知', authority: '未知', detail: ''},
  gravity: {enabled: null, active: false, blocked: '等待主机状态', pending: false},
  session: "未获台架状态",
  mappingError: "",
});

// 迁移缝隙仅在此接触原生模块；Vue 外壳只消费带类型的快照与操作。
type LegacyModule = Record<string, any>;
const load = (path: string): Promise<LegacyModule> =>
  import(/* @vite-ignore */ `/js/${path}.js`);
let modules: {
  data: LegacyModule;
  ros: LegacyModule;
  agent: LegacyModule;
  device: LegacyModule;
  format: LegacyModule;
  actions: LegacyModule;
  gravity: LegacyModule;
};
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
  "deviceParams",
  "actionLibrary",
  "sim",
  "curve",
  "events",
  "imu",
];
const mounted = new Set<PanelId>();
const requiredPanels = new Set<PanelId>(["jointList", "deviceParams"]);
let requiredPanelError = "";
let resolvePanels: () => void;
const panelsReady = new Promise<void>((resolve) => {
  resolvePanels = resolve;
});
let disposed = false;
let generation = 0;
let navigation: (screen: string) => void = () => {};
let frame = 0;
let stopDirtyLoop: (() => void) | undefined;
let lastMotionCount = 0;
let lastMotionAt: number | null = null;
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
  const { data } = modules,
    s = data.store;
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
      .map(id => ({id, model: s.motors.get(id)?.topo?.model_name || ''})),
    ...data.countMotors(),
    motionAt: lastMotionAt,
    communication: modules.agent.describeAgentStatus(s),
    gravity: modules.gravity.describeGravityControl(s, data.MODEL),
    session: simulation
      ? `仿真进程：${simulation.state || "未知"}${s.conn.agent === "running" ? " · Agent 运行中" : ""}`
      : s.conn.agent === "running"
        ? "Agent 运行中 · 控制对象待核对"
        : "未获台架状态",
    mappingError:
      data.jointMapSource.kind === "unavailable"
        ? `模型无设备映射：${data.jointMapSource.error}`
        : "",
  };
}

function dirty(batch: Set<string>) {
  if (disposed) return;
  const { data, ros, agent, device } = modules,
    s = data.store;
  if (wasConnected !== s.conn.connected) {
    generation++;
    wasConnected = s.conn.connected;
  }
  const feedbackKey = `${generation}:${data.MODEL}`;
  // 只补齐未选模型的主机；其他页面已选模型时不周期性抢回归属。
  const feedbackNeedsModel = s.bench.feedback && !s.bench.feedback.desired_model;
  if (s.conn.connected && !feedbackSyncPending &&
      (feedbackKey !== feedbackSyncKey ||
        ((feedbackNeedsModel || feedbackSyncFailed) && Date.now() - feedbackSyncAt > 5000))) {
    const epoch = generation, model = data.MODEL;
    feedbackSyncKey = feedbackKey;
    feedbackSyncAt = Date.now();
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
    s.selectedId = s.motors.keys().next().value;
    batch.add(data.DIRTY.SELECTED);
  }
  updateSnapshot();
  scheduler.invalidate(batch);
}

async function start() {
  if (starting) return starting;
  starting = (async () => {
    try {
      const [response, configuration, data, ros, agent, device, format, actions, gravity] =
        await Promise.all([
          fetch("/panel_templates.html", { cache: "no-store" }),
          fetch("/api/runtime", { cache: "no-store" }),
          load("core/store"),
          load("core/ros"),
          load("core/agent"),
          load("core/device"),
          load("core/format"),
          load("panels/actionLibrary"),
          load("core/gravity_control"),
        ]);
      if (!response.ok)
        throw new Error(`面板模板请求失败：HTTP ${response.status}`);
      const html = await response.text();
      if (disposed) return;
      modules = { data, ros, agent, device, format, actions, gravity };
      if (!configuration.ok) throw new Error('无法读取工作台运行配置，控制保持关闭');
      const runtimeConfig = await configuration.json();
      if (runtimeConfig.rosbridge_port) configuredRosUrl.value = `${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.hostname}:${runtimeConfig.rosbridge_port}`;
      ros.configureControl(runtimeConfig);
      if (runtimeConfig.robot_model) data.setModel(runtimeConfig.robot_model);
      const documentSource = new DOMParser().parseFromString(html, "text/html");
      const templates = document.createElement("div");
      templates.hidden = true;
      templates.dataset.workbenchTemplates = "";
      for (const template of documentSource.querySelectorAll(
        "template[data-panel-template]",
      ))
        templates.append(template.cloneNode(true));
      document.body.append(templates);
      data.onDirty(dirty);
      stopDirtyLoop = data.runRenderLoop();
      const render = (now: number) => {
        if (!disposed) {
          scheduler.flush(now);
          frame = requestAnimationFrame(render);
        }
      };
      frame = requestAnimationFrame(render);
      await data.loadModelJoints();
      if (disposed) return;
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
  const module = await load(`panels/${id}`);
  if (signal.aborted || disposed) return () => {};
  const template = document.querySelector<HTMLTemplateElement>(
    `template[data-panel-template="${id}"]`,
  );
  if (!template) throw new Error(`缺少面板模板：${id}`);
  root.append(template.content.cloneNode(true));
  const data = modules.data;
  const instance: MountedPanel = module.mount(root, {
    store: data.store,
    markDirty: data.markDirty,
    ui: {
      selectMotor: runtime.selectMotor,
      setModel(name: string) {
        data.setModel(name);
      },
      gotoScreen(id: string) {
        navigation(id);
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
    root.replaceChildren();
  };
}

export const runtime = {
  start,
  mountPanel,
  async setGravityEnabled(enabled: boolean) {
    if (!initialized.value || disposed) throw new Error('工作区尚未就绪');
    await modules.gravity.setGravityEnabled(enabled);
  },
  async readFirmwareParameter(name: string) {
    if (!initialized.value || disposed) throw new Error('工作区尚未就绪');
    return modules.device.readFirmwareParameter(name);
  },
  selectMotor(id: number) {
    if (!modules || disposed) return;
    modules.data.store.selectedId = id;
    modules.data.markDirty(modules.data.DIRTY.SELECTED);
  },
  setNavigation(callback: (screen: string) => void) {
    navigation = callback;
  },
  wake() {
    if (!initialized.value || disposed) return;
    scheduler.retryFailed();
    scheduler.invalidate(
      new Set(Object.values(modules.data.DIRTY) as string[]),
    );
    window.dispatchEvent(new Event("resize"));
  },
  connect(url: string) {
    if (!initialized.value || disposed) return;
    const input = document.getElementById("rosUrl") as HTMLInputElement;
    input.value = url.trim();
    lastMotionAt = null;
    localStorage.setItem(modules.format.ROS_URL_STORAGE_KEY, input.value);
    generation++;
    modules.ros.connect();
  },
  async startAgent() {
    if (initialized.value) await modules.agent.startAgent();
  },
  async stopAgent() {
    if (initialized.value) await modules.agent.stopAgent();
  },
  async refreshAgentControls() {
    if (initialized.value) await modules.agent.refreshAgentControls();
  },
  stopMotion() {
    if (!initialized.value || !modules.ros.isConnected()) return false;
    const sent = modules.actions.stopAll();
    notice.value = sent
      ? "已请求全部软停（停用运动控制器）；请核对设备反馈"
      : "停止请求未发送";
    return sent;
  },
  dispose() {
    disposed = true;
    generation++;
    cancelAnimationFrame(frame);
    stopDirtyLoop?.();
    resolveReady();
    resolvePanels();
    scheduler.dispose();
    if (modules) {
      modules.data.onDirty(null);
      modules.ros.disconnect();
    }
    document.querySelector("[data-workbench-templates]")?.remove();
  },
};
