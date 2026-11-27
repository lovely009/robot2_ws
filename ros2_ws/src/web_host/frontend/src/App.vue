<script setup lang="ts">
import {
  computed,
  nextTick,
  onBeforeUnmount,
  onMounted,
  ref,
  watch,
} from "vue";
import {
  Box,
  SlidersHorizontal,
  Cpu,
  Workflow,
  Cable,
  RefreshCw,
  CircleStop,
  PanelLeftClose,
  PanelLeftOpen,
  Film,
} from "@lucide/vue";
import ReadoutPanel from "./components/ReadoutPanel.vue";
import JointListPanel from "./components/JointListPanel.vue";
import ArmStagePanel from "./components/ArmStagePanel.vue";
import ActionLibraryPanel from "./components/ActionLibraryPanel.vue";
import CurvePanel from "./components/CurvePanel.vue";
import ImuPanel from "./components/ImuPanel.vue";
import ControlLeaseBar from "./components/ControlLeaseBar.vue";
import DiagnosticsRecorderPanel from "./components/DiagnosticsRecorderPanel.vue";
import EventLog from "./components/EventLog.vue";
import FirmwareParameters from "./components/FirmwareParameters.vue";
import DeviceParameters from "./components/DeviceParameters.vue";
import ResizeHandle from "./components/ResizeHandle.vue";
import MotionWorkbench from "./components/MotionWorkbench.vue";
import GravityControl from "./components/GravityControl.vue";
import { useLeaseStore } from "./stores/lease";
import {
  runtime,
  snapshot,
  initialized,
  startupError,
  notice,
  configuredRosUrl,
} from "./runtime";
import type { Screen } from "./contracts";

const leaseStore = useLeaseStore();

const resizablePanelMinHeight = 64;
const normalize = (hash: string): Screen => {
  const value = hash.replace(/^#\/?/, "");
  return value === "teach" || value === "motion" ? value : "run";
};
const screen = ref<Screen>(normalize(location.hash));
watch(screen, (value) => {
  mobilePane.value = value === "teach" ? "objects" : "stage";
});
const observation = ref("curve");
const runInspectorTab = ref<'joints' | 'actions' | 'motors'>('joints');
const mobilePane = ref('stage');
const connectionSettings = ref(false);
const mobilePanes = [
  {id:'objects', title:'电机'}, {id:'stage', title:'工作区'},
  {id:'observer', title:'观察'}, {id:'parameters', title:'参数'},
];
const observe = ref(true);
const navCollapsed = ref(
  localStorage.getItem("robot2.navCollapsed") === null
    ? window.matchMedia("(max-width: 1250px)").matches
    : localStorage.getItem("robot2.navCollapsed") === "true",
);
watch(navCollapsed, (value) => {
  localStorage.setItem("robot2.navCollapsed", String(value));
});
const deviceParameters = ref<'firmware' | 'motor'>('motor');
const motorListOpen = ref(true);
const selectedParameterMotor = computed(() => snapshot.value.motors.find(
  motor => motor.id === snapshot.value.selectedId)?.id ?? snapshot.value.motors[0]?.id ?? 0);
function toggleMotorList() {
  motorListOpen.value = deviceParameters.value !== 'motor' || !motorListOpen.value;
  deviceParameters.value = 'motor';
}
const now = ref(Date.now());
const rosUrl = ref(
  localStorage.getItem("robot2.rosbridgeUrl") ||
    `${location.protocol === "https:" ? "wss" : "ws"}://${location.hostname}:9090`,
);
watch(configuredRosUrl, value => { if (value) rosUrl.value = value; });
const scenes = [
  {
    id: "run" as const,
    title: "调试工作台",
    icon: SlidersHorizontal,
  },
  {
    id: "teach" as const,
    title: "设备工作台",
    icon: Cpu,
  },
  {
    id: "motion" as const,
    title: "动作工作台",
    icon: Film,
  },
];
const title = computed(() => scenes.find((item) => item.id === screen.value)!);
const fresh = computed(
  () =>
    snapshot.value.connected &&
    snapshot.value.motionAt !== null &&
    now.value - snapshot.value.motionAt < 1500,
);
const agentLabel = computed(
  () =>
    ({
      running: "进程运行中",
      starting: "正在启动",
      stopping: "正在停止",
      stopped: "未启动",
      failed: "进程已退出",
      error: "操作失败",
    })[snapshot.value.agent] || snapshot.value.agent,
);
let clock: ReturnType<typeof setInterval>;
function go(id: string) {
  if (id === "device") {
    id = "teach";
  }
  if (id === "diag") {
    observe.value = true;
    observation.value = "imu";
    id = "run";
  }
  screen.value = normalize(id);
  location.hash = `/${screen.value}`;
}
function reloadWorkspace() { window.location.reload(); }
function hashchange() {
  screen.value = normalize(location.hash);
}
function keys(event: KeyboardEvent) {
  if (
    event.ctrlKey ||
    event.metaKey ||
    event.altKey ||
    (event.target as HTMLElement)?.closest(
      "input,select,textarea,[contenteditable]",
    )
  )
    return;
  if ((event.key === " " || event.key === "Escape") && snapshot.value.connected) {
    event.preventDefault();
    runtime.stopMotion();
    return;
  }
  const index = "1234".indexOf(event.key);
  if (index >= 0) go(scenes[index]!.id);
}
function activeChannels(): string[] {
  const ch = [
    "telemetry:control",
    "telemetry:transmission",
    "telemetry:bench",
    "telemetry:motor_state",
  ];
  if (screen.value === "run") {
    ch.push("telemetry:joints", "telemetry:motor_motion", "telemetry:events");
    if (observe.value && observation.value === "imu") ch.push("telemetry:imu", "telemetry:imu_temp");
  } else if (screen.value === "sim") {
    ch.push("telemetry:joints", "telemetry:sim_position");
  } else if (screen.value === "teach") {
    ch.push("telemetry:motor_motion", "telemetry:events");
  }
  return ch;
}
watch(
  [screen, observation, observe],
  async () => {
    runtime.setActiveChannels(activeChannels());
    await nextTick();
    runtime.wake();
  },
  { immediate: true },
);
onMounted(() => {
  runtime.setNavigation(go);
  clock = setInterval(() => {
    now.value = Date.now();
  }, 250);
  window.addEventListener("hashchange", hashchange);
  window.addEventListener("keydown", keys);
  void runtime.start();
});
onBeforeUnmount(() => {
  clearInterval(clock);
  window.removeEventListener("hashchange", hashchange);
  window.removeEventListener("keydown", keys);
  runtime.dispose();
});
</script>

<template>
  <div class="workbench" :data-mobile-pane="mobilePane" :class="{ 'observer-hidden': !observe, 'nav-collapsed': navCollapsed, 'viewer-readonly': leaseStore.isBff && !leaseStore.isOwner }">
    <header class="app-header studio-topbar" :class="{'settings-open': connectionSettings}">
      <a class="brand" href="#/run" @click="go('run')"
        ><span class="brand-mark"><Box :size="18" /></span
        ><span>元与定序具身智能<span class="brand-light"> 控制台</span></span></a
      >
      <template v-if="!leaseStore.isBff">
        <button class="connection-settings secondary compact" :aria-expanded="connectionSettings" aria-controls="rosUrl" @click="connectionSettings = !connectionSettings">连接设置</button>
        <label class="session-item connection-address"
          ><Cable :size="14" /><span class="session-label">ROS</span
          ><input
            id="rosUrl"
            v-model="rosUrl"
            aria-label="rosbridge WebSocket 地址"
            spellcheck="false"
        /></label>
        <button
          class="secondary compact reconnect-button"
          :disabled="!initialized"
          @click="runtime.connect(rosUrl)"
        >
          重新连接
        </button>
      </template>
      <ControlLeaseBar v-else />
      <span class="session-item connection"
        ><i class="dot" :class="snapshot.connected ? 'green' : 'amber'" />{{
          snapshot.connectionText
        }}</span
      >
      <span class="session-separator" />
      <details class="hardware-popover">
        <summary :title="snapshot.agentError || snapshot.agentDevice">
          <i class="dot" :class="snapshot.agent === 'running' ? 'green' : 'amber'" />
          硬件 · Agent：{{ agentLabel }}
        </summary>
        <div class="hardware-popover-body">
          <div class="hardware-popover-row">
            <label class="session-item"
              ><span class="session-label">控制板</span
              ><select
                id="serialPort"
                aria-label="控制板串口"
                :disabled="
                  !snapshot.connected ||
                  snapshot.agent === 'running' ||
                  snapshot.agent === 'starting' ||
                  snapshot.agent === 'stopping'
                "
              >
                <option value="">等待串口枚举</option>
              </select></label
            >
            <button
              id="btnPortRefresh"
              class="icon-button"
              aria-label="刷新控制板串口"
              :disabled="!snapshot.connected"
              @click="runtime.refreshAgentControls()"
            >
              <RefreshCw :size="13" />
            </button>
          </div>
          <div class="hardware-popover-actions">
            <button
              id="btnConnect"
              class="secondary compact"
              :disabled="
                !snapshot.connected ||
                snapshot.agent === 'running' ||
                snapshot.agent === 'starting' ||
                snapshot.agent === 'stopping'
              "
              @click="runtime.startAgent()"
            >
              启动 Agent
            </button>
            <button
              id="btnDisconnect"
              class="secondary compact"
              :disabled="!snapshot.connected || !snapshot.canStopAgent"
              @click="runtime.stopAgent()"
            >
              停止 Agent
            </button>
          </div>
        </div>
      </details>
      <span class="header-spacer" />
      <div v-show="screen === 'run'" class="topbar-status">
        <span class="micro" :title="snapshot.communication.detail">
          <i class="dot" :class="fresh ? 'green' : 'amber'" />{{ snapshot.online }}/{{ snapshot.total }} · MCU:{{ snapshot.communication.link }}
        </span>
        <span
          v-if="leaseStore.isBff && !leaseStore.isOwner"
          class="info-badge warn"
          tabindex="0"
          role="status"
          title="当前为只读围观模式；如需拖动关节、使能电机或启停仿真，请先点击顶部「申请控制权」。"
          aria-label="只读围观模式提示"
        >!</span>
        <span
          v-if="snapshot.mappingError"
          class="info-badge warn"
          tabindex="0"
          role="status"
          :title="`${snapshot.mappingError}；几何预览仍可使用。`"
          :aria-label="snapshot.mappingError"
        >!</span>
      </div>
      <button
        class="stop-button"
        title="全部软停（快捷键：空格 / Esc）"
        :disabled="!snapshot.connected"
        @click="runtime.stopMotion()"
      >
        <CircleStop :size="14" />全部软停
      </button>
    </header>

    <nav class="main-nav" aria-label="工作场景">
      <button
        class="nav-toggle"
        :aria-label="navCollapsed ? '展开侧栏' : '收起侧栏'"
        :title="navCollapsed ? '展开侧栏' : '收起侧栏'"
        :aria-expanded="!navCollapsed"
        @click="navCollapsed = !navCollapsed"
      >
        <component :is="navCollapsed ? PanelLeftOpen : PanelLeftClose" :size="18" />
        <span>收起侧栏</span>
      </button>
      <button
        v-for="(item, index) in scenes"
        :key="item.id"
        :aria-label="item.title"
        :title="item.title"
        :class="{ active: screen === item.id }"
        :aria-current="screen === item.id ? 'page' : undefined"
        @click="go(item.id)"
      >
        <component :is="item.icon" :size="18" /><span>{{ item.title }}</span
        ><small>{{ index + 1 }}</small>
      </button>
    </nav>

    <main class="main-workspace">
      <div v-show="screen !== 'run'" class="workspace-heading">
        <h1>{{ title.title }}</h1>
        <div v-show="screen !== 'teach' && screen !== 'motion'" class="workspace-summary">
          <span
            ><i class="dot" :class="fresh ? 'green' : 'amber'" />{{
              snapshot.online
            }}
            / {{ snapshot.total }}<small>设备在线</small></span
          >
          <span class="micro" :title="snapshot.communication.detail">
            MCU：{{ snapshot.communication.link }} · 时钟：{{ snapshot.communication.clock }} · 控制权：{{ snapshot.communication.authority }}
          </span>
        </div>
      </div>
      <div v-if="startupError" class="panel-error" role="alert">
        <strong>工作区加载失败</strong><span>{{ startupError }}</span>
        <button class="secondary compact" @click="reloadWorkspace()">重新加载页面</button>
      </div>
      <div v-else-if="!initialized" class="startup-status" role="status" aria-busy="true">
        正在加载工作区与控制模块…
      </div>
      <div v-if="snapshot.fault > 0 && screen !== 'motion'" class="fault-banner" role="alert">
        <span>检测到 {{ snapshot.fault }} 台电机存在故障</span>
        <button type="button" class="tiny-btn secondary" @click="go('diag')">查看事件日志</button>
      </div>
      <nav v-show="screen !== 'motion'" class="mobile-pane-tabs" aria-label="工作区面板">
        <button v-for="item in mobilePanes.filter(p => screen === 'teach' ? ['objects','parameters'].includes(p.id) : p.id !== 'parameters')" :key="item.id"
          :aria-pressed="mobilePane === item.id" @click="mobilePane = item.id; if (item.id === 'observer') observe = true; if (item.id === 'objects' && screen === 'run') runInspectorTab = 'motors'">
          {{ item.title }}<small v-if="item.id === 'objects'">{{ snapshot.total }}</small>
        </button>
      </nav>
      <MotionWorkbench v-if="screen === 'motion'" :current-model="snapshot.model" />
      <div v-show="screen !== 'motion'" class="workspace-grid" :class="{ 'run-workspace': screen === 'run', 'device-workspace': screen === 'teach' }">
        <ResizeHandle v-show="screen === 'run'" class="resize-control" label="调整右侧检查器宽度" target=".inspector-dock" variable="--control-width" axis="width" :min="320" reverse />
        <ResizeHandle v-show="screen === 'run' && observe" class="resize-observer" label="调整观察区高度" target=".observer-area" variable="--observer-height" axis="height" :min="resizablePanelMinHeight" reverse />
        <section v-show="screen === 'teach'" class="objects-area surface">
          <div class="panel-title">
            <span>设备查询</span>
            <small class="micro">电机反馈</small>
          </div>
          <nav v-if="screen === 'teach'" class="device-parameter-nav" aria-label="设备参数查询">
            <button :class="{ active: deviceParameters === 'firmware' }" :aria-pressed="deviceParameters === 'firmware'" @click="deviceParameters = 'firmware'">
              <Cpu :size="14" />固件参数
            </button>
            <button :class="{ active: deviceParameters === 'motor' }" :aria-pressed="deviceParameters === 'motor'" :aria-expanded="deviceParameters === 'motor' && motorListOpen" aria-controls="device-motor-list" aria-label="电机参数" @click="toggleMotorList">
              <SlidersHorizontal :size="14" />电机参数<span class="motor-list-chevron">{{ deviceParameters === 'motor' && motorListOpen ? '▾' : '▸' }}</span>
            </button>
            <div v-show="deviceParameters === 'motor' && motorListOpen" id="device-motor-list" class="device-motor-list" role="group" aria-label="电机列表">
              <button v-for="motor in snapshot.motors" :key="motor.id" :class="{ active: selectedParameterMotor === motor.id }" :aria-pressed="selectedParameterMotor === motor.id" @click="runtime.selectMotor(motor.id)">
                <span>电机 {{ motor.id }}</span><small>{{ motor.model || '型号待识别' }}</small>
              </button>
              <p v-if="!snapshot.motors.length" class="micro">等待电机反馈</p>
            </div>
          </nav>
          <div class="tree-footer">选择电机，查看设备反馈与参数</div>
        </section>
        <div v-show="screen !== 'teach'" class="stage-area" :class="`stage-${screen}`">
          <div v-show="screen !== 'run'" class="stage-toolbar">
            <div class="breadcrumb">
              <Box :size="14" /><span>{{
                snapshot.model || "正在读取模型"
              }}</span>
            </div>
            <span class="tiny-tag"
              >{{ snapshot.active }} 控制激活 · {{ snapshot.fault }} 故障</span
            >
          </div>
          <div class="stage-body">
            <ArmStagePanel :active="screen !== 'teach' && screen !== 'motion'" class="robot-panel">
              <GravityControl v-show="screen === 'run'" />
            </ArmStagePanel>
          </div>
        </div>
        <aside v-show="screen === 'run'" class="inspector-dock surface" aria-label="右侧检查器">
          <div class="inspector-tabs" role="tablist" aria-label="检查器切换">
            <button type="button" role="tab" :aria-selected="runInspectorTab === 'actions'" :class="{ active: runInspectorTab === 'actions' }" @click="runInspectorTab = 'actions'">
              动作库
            </button>
            <button type="button" role="tab" :aria-selected="runInspectorTab === 'joints'" :class="{ active: runInspectorTab === 'joints' }" @click="runInspectorTab = 'joints'">
              关节 <small class="tab-count">{{ snapshot.active }}/{{ snapshot.total }}</small>
            </button>
            <button type="button" role="tab" :aria-selected="runInspectorTab === 'motors'" :class="{ active: runInspectorTab === 'motors' }" @click="runInspectorTab = 'motors'">
              电机 <small class="tab-count">{{ snapshot.online }}</small>
            </button>
          </div>
          <div class="inspector-body">
            <div v-show="runInspectorTab === 'joints'" id="run-joints" class="joint-list-panel">
              <JointListPanel :active="screen === 'run' && runInspectorTab === 'joints'" />
            </div>
            <ActionLibraryPanel
              v-show="runInspectorTab === 'actions'"
              :active="screen === 'run'"
              class="action-panel-host"
            />
            <div v-show="runInspectorTab === 'motors'" class="motor-inspector-panel">
              <ReadoutPanel :active="screen === 'run'" />
            </div>
          </div>
        </aside>
        <aside v-show="screen === 'teach'" class="device-details-area inspector surface">
          <div class="panel-title"><span>{{ deviceParameters === 'firmware' ? '固件固定配置' : selectedParameterMotor ? `电机 ${selectedParameterMotor} 详细参数` : '电机详细参数' }}</span></div>
          <div class="inspector-scroll">
            <FirmwareParameters :active="screen === 'teach' && deviceParameters === 'firmware'" />
            <DeviceParameters :active="screen === 'teach' && deviceParameters === 'motor'" />
          </div>
        </aside>
        <section v-show="screen === 'run'" class="observer-area observer surface">
          <div class="observer-head">
            <button
              class="observer-fold"
              :aria-label="observe ? '收起观察区' : '展开观察区'"
              :aria-expanded="observe"
              aria-controls="observation-content"
              @click="observe = !observe"
            >{{ observe ? '▾' : '▸' }}</button>
            <div class="observer-tabs">
              <button
                v-for="item in [
                  { id: 'curve', title: '曲线' },
                  { id: 'events', title: '事件日志' },
                  { id: 'imu', title: 'IMU' },
                  { id: 'diag', title: '诊断与录播' },
                ]"
                :key="item.id"
                :class="{ active: observation === item.id }"
                @click="observation = item.id; observe = true"
              >
                {{ item.title }}
              </button>
            </div>
            <div class="observer-controls">
              <span class="micro">{{ fresh ? "实时" : "等待 / 陈旧" }}</span>
            </div>
          </div>
          <div id="observation-content" v-show="observe" class="observation-body">
            <CurvePanel
              :active="screen === 'run' && observe && observation === 'curve'"
            />
            <EventLog v-show="screen === 'run' && observe && observation === 'events'" />
            <ImuPanel :active="screen === 'run' && observe && observation === 'imu'" />
            <DiagnosticsRecorderPanel
              :active="screen === 'run' && observe && observation === 'diag'"
            />
          </div>
        </section>
      </div>
    </main>

    <div v-if="notice" class="toast" role="status">
      <span>{{ notice }}</span
      ><button class="icon-button" aria-label="关闭提示" @click="notice = ''">
        ×
      </button>
    </div>
  </div>
</template>
