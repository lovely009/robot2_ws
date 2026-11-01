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
import HostLimits from "./components/HostLimits.vue";
import PanelHost from "./components/PanelHost.vue";
import CommandHistory from "./components/CommandHistory.vue";
import DeviceConnection from "./components/DeviceConnection.vue";
import FirmwareParameters from "./components/FirmwareParameters.vue";
import ResizeHandle from "./components/ResizeHandle.vue";
import MotionWorkbench from "./components/MotionWorkbench.vue";
import {
  runtime,
  snapshot,
  initialized,
  startupError,
  notice,
} from "./runtime";
import type { Screen } from "./contracts";

const normalize = (hash: string): Screen => {
  const value = hash.replace(/^#\/?/, "");
  return value === "teach" || value === "sim" || value === "motion" ? value : "run";
};
const screen = ref<Screen>(normalize(location.hash));
const inspector = ref(screen.value === "teach" ? "parameters" : "control");
watch(screen, (value) => {
  inspector.value = value === "teach" ? "parameters" : "control";
  mobilePane.value = value === "teach" ? "objects" : "stage";
});
const observation = ref("curve");
const mobilePane = ref('stage');
const connectionSettings = ref(false);
const mobilePanes = [
  {id:'objects', title:'电机'}, {id:'stage', title:'工作区'},
  {id:'inspector', title:'检查器'}, {id:'observer', title:'观察'},
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
const directDevice = ref(false);
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
const scenes = [
  {
    id: "run" as const,
    title: "调试工作台",
    icon: SlidersHorizontal,
    subtitle: "核对设备，调整参数，观察反馈",
  },
  {
    id: "teach" as const,
    title: "设备工作台",
    icon: Cpu,
    subtitle: "查看设备状态，调整固件与电机参数",
  },
  {
    id: "sim" as const,
    title: "仿真工作台",
    icon: Workflow,
    subtitle: "配置后端、控制端与台架会话",
  },
  {
    id: "motion" as const,
    title: "动作工作台",
    icon: Film,
    subtitle: "导入动作与机器人模型，转换并预览",
  },
];
const title = computed(() => scenes.find((item) => item.id === screen.value)!);
const motionAge = computed(() =>
  snapshot.value.motionAt === null
    ? "等待运动数据"
    : `${Math.max(0, now.value - snapshot.value.motionAt)} ms`,
);
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
    inspector.value = "parameters";
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
  const index = "1234".indexOf(event.key);
  if (index >= 0) go(scenes[index]!.id);
}
watch([screen, inspector, observation, observe], async () => {
  await nextTick();
  runtime.wake();
});
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
  <div class="workbench" :data-mobile-pane="mobilePane" :class="{ 'observer-hidden': !observe, 'nav-collapsed': navCollapsed }">
    <header class="app-header">
      <a class="brand" href="#/run" @click="go('run')"
        ><span class="brand-mark"><Box :size="22" /></span
        ><span>ROBOT2<span class="brand-light"> 工程控制台</span></span></a
      >
      <span class="header-spacer" /><span class="micro">{{
        snapshot.session
      }}</span>
    </header>

    <div class="session-bar" :class="{'settings-open': connectionSettings}">
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
      <span class="session-item connection"
        ><i class="dot" :class="snapshot.connected ? 'green' : 'amber'" />{{
          snapshot.connectionText
        }}</span
      >
      <template v-if="!snapshot.managed.enabled">
      <span class="session-separator" />
      <label class="session-item"
        ><span class="session-label">控制板</span
        ><select id="serialPort" aria-label="控制板串口">
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
      <button
        id="btnConnect"
        class="secondary compact"
        :disabled="
          !snapshot.connected ||
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
      <span
        class="session-item agent-status"
        :title="snapshot.agentError || snapshot.agentDevice"
        >Agent：{{ agentLabel }}</span>
      </template>
      <span class="header-spacer" /><button
        class="stop-button"
        :disabled="!snapshot.connected"
        @click="runtime.stopMotion()"
      >
        <CircleStop :size="14" />{{ snapshot.managed.enabled ? '整机停止' : '全部软停' }}
      </button>
    <div v-if="snapshot.managed.enabled" class="managed-controls" aria-label="整机托管控制">
      <strong>MCU 执行</strong><span class="authority-label">本页控制权：{{ snapshot.managed.ownsControl ? '已取得' : '未取得' }}</span><span class="authority-label" :class="{confirmed: snapshot.managed.mcuAuthorized}">运动授权：{{ snapshot.managed.mcuAuthorized ? 'MCU 已确认' : '未确认' }}</span>
      <button :disabled="!initialized || !snapshot.managed.ready || snapshot.managed.pending || snapshot.managed.ownsControl"
        @click="runtime.managed('acquire')">申请控制</button>
      <button :disabled="!snapshot.managed.ownsControl || snapshot.managed.armed"
        @click="runtime.managed('arm')">武装</button>
      <button :disabled="!snapshot.managed.ownsControl" @click="runtime.managed('disarm')">解除武装</button>
      <button :disabled="!snapshot.managed.ownsControl" @click="runtime.managed('release')">释放控制</button>
      <button :disabled="!snapshot.connected" @click="runtime.managed('reset')">复位故障</button>
      <span role="status">{{ snapshot.managed.message }}</span>
      <span v-if="snapshot.managed.armed && !snapshot.managed.mcuAuthorized">等待 MCU 授权确认</span>
      <span v-if="snapshot.managed.blockers.length" class="managed-blockers">未就绪：{{ snapshot.managed.blockers.join(' · ') }}</span>
    </div>
    </div>

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
      <div class="nav-section-label">工作场景</div>
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
      <div class="workspace-heading">
        <div>
          <div class="eyebrow">工作场景 / {{ screen.toUpperCase() }}</div>
          <h1>
            {{ title.title }}<span class="heading-slash">/</span
            ><span class="heading-description">{{ title.subtitle }}</span>
          </h1>
        </div>
        <div v-show="screen !== 'teach' && screen !== 'motion'" class="workspace-summary">
          <span
            ><i class="dot" :class="fresh ? 'green' : 'amber'" />{{
              snapshot.online
            }}
            / {{ snapshot.total }}<small>设备在线</small></span
          ><span class="micro" :class="{ stale: !fresh }"
            >运动流：{{ motionAge }}</span
          >
        </div>
      </div>
      <div v-if="startupError" class="panel-error" role="alert">
        <strong>工作区加载失败</strong><span>{{ startupError }}</span>
        <button class="secondary compact" @click="reloadWorkspace()">重新加载页面</button>
      </div>
      <div v-else-if="!initialized" class="startup-status" role="status" aria-busy="true">
        正在加载工作区与控制模块…
      </div>
      <div v-if="snapshot.mappingError && screen !== 'teach' && screen !== 'motion'" class="mapping-warning" role="status">
        {{ snapshot.mappingError }}；几何预览仍可使用。
      </div>

      <nav v-show="screen !== 'motion'" class="mobile-pane-tabs" aria-label="工作区面板">
        <button v-for="item in mobilePanes.filter(p => screen !== 'teach' || ['objects', 'inspector'].includes(p.id))" :key="item.id"
          :aria-pressed="mobilePane === item.id" @click="mobilePane = item.id; if (item.id === 'observer') observe = true">
          {{ item.title }}<small v-if="item.id === 'objects'">{{ snapshot.total }}</small>
        </button>
      </nav>
      <div v-show="screen === 'teach'">
        <DeviceConnection mcu-only @direct="directDevice = $event" />
      </div>
      <MotionWorkbench v-if="screen === 'motion'" />
      <div v-show="screen !== 'motion' && (screen !== 'teach' || !directDevice)" class="workspace-grid" :class="{ 'device-workspace': screen === 'teach' }">
        <ResizeHandle v-show="screen !== 'teach'" class="resize-objects" label="调整对象浏览器宽度" target=".objects-area" variable="--objects-width" axis="width" :min="140" />
        <ResizeHandle v-show="screen !== 'teach'" class="resize-inspector" label="调整对象检查器宽度" target=".inspector-area" variable="--inspector-width" axis="width" :min="210" reverse />
        <ResizeHandle v-show="screen !== 'teach' && observe" class="resize-observer" label="调整观察区高度" target=".observer-area" variable="--observer-height" axis="height" :min="120" reverse />
        <section class="objects-area surface">
          <div class="panel-title">
            <span>对象浏览器</span>
            <small class="micro">{{ screen !== 'sim' ? '电机反馈' : 'URDF × TF' }}</small>
          </div>
          <nav v-if="screen === 'teach'" class="device-parameter-nav" aria-label="参数设置">
            <div class="eyebrow">参数设置</div>
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
          <PanelHost id="readout" :active="screen === 'run'" />
          <Teleport to="#run-joints" :disabled="screen !== 'run'" defer>
            <PanelHost id="jointList" :active="screen !== 'teach' && screen !== 'motion'" />
          </Teleport>
          <div class="tree-footer">{{ screen === 'teach' ? '选择电机，查看设备反馈与参数' : '关节数据与设备测量分别显示' }}</div>
        </section>
        <div v-show="screen !== 'teach'" class="stage-area" :class="`stage-${screen}`">
          <div class="stage-toolbar">
            <div class="breadcrumb">
              <Box :size="14" /><span>{{
                snapshot.model || "正在读取模型"
              }}</span
              ><span>/</span><b>{{ title.title }}</b>
            </div>
            <span class="tiny-tag"
              >{{ snapshot.active }} 控制激活 · {{ snapshot.fault }} 故障</span
            >
          </div>
          <div class="stage-body">
            <PanelHost id="arm" :active="screen !== 'teach' && screen !== 'motion'" class="robot-panel" />
            <ResizeHandle v-show="screen === 'run'" class="resize-joints" label="调整关节面板高度" target="#run-joints" variable="--joints-height" axis="height" :min="75" reverse />
            <div
              id="run-joints"
              v-show="screen === 'run'"
              class="joint-list-panel"
            />
            <ResizeHandle v-show="screen === 'run'" class="resize-actions" label="调整动作库高度" target=".action-panel-host" variable="--actions-height" axis="height" :min="170" reverse />
            <PanelHost
              id="actionLibrary"
              :active="screen === 'run'"
              class="action-panel-host"
            />
            <ResizeHandle v-show="screen === 'sim'" class="resize-simulation" label="调整仿真面板高度" target=".sim-panel-host" variable="--sim-height" axis="height" :min="240" reverse />
            <PanelHost
              id="sim"
              :active="screen === 'sim'"
              class="sim-panel-host"
            />
          </div>
        </div>
        <aside class="inspector-area inspector surface">
          <div class="panel-title">
            <span>{{ screen === 'teach' ? (deviceParameters === 'firmware' ? '固件参数设置' : selectedParameterMotor ? `电机 ${selectedParameterMotor} 详细参数` : '电机详细参数') : '对象检查器' }}</span><SlidersHorizontal :size="14" />
          </div>
          <div v-show="screen !== 'teach'" class="object-heading">
            <div class="eyebrow">
              {{
                snapshot.selectedId
                  ? `ACTUATOR / ${snapshot.selectedId}`
                  : "NO SELECTION"
              }}
            </div>
            <h2>
              {{
                snapshot.selectedName ||
                (snapshot.selectedId
                  ? `电机 ${snapshot.selectedId}`
                  : "未选择设备")
              }}
            </h2>
            <span
              class="status-chip"
              :class="snapshot.selectedCanCommand && fresh ? 'success' : ''"
              >{{ snapshot.selectedState
              }}{{ snapshot.connected ? "" : " · 数据未连接" }}</span
            >
          </div>
          <p v-if="screen !== 'teach' && snapshot.managed.enabled && !snapshot.managed.mcuAuthorized" class="control-lock-note">控制已锁定：申请控制权、武装并等待 MCU 确认后，才能下发使能与目标。</p>
          <div v-show="screen !== 'teach'" class="inspector-tabs">
            <button
              :class="{ active: inspector === 'control' }"
              @click="inspector = 'control'"
            >
              控制</button
            ><button
              :class="{ active: inspector === 'parameters' }"
              @click="inspector = 'parameters'"
            >
              参数</button
            ><button
              :class="{ active: inspector === 'compensation' }"
              @click="inspector = 'compensation'"
            >
              补偿
            </button>
            <button :class="{ active: inspector === 'limits' }" @click="inspector = 'limits'">主机限位</button>
          </div>
          <div class="inspector-scroll">
            <FirmwareParameters :active="screen === 'teach' && deviceParameters === 'firmware'" />
            <HostLimits v-show="screen !== 'teach'" :editing="inspector === 'limits'" />
            <PanelHost
              id="policy"
              :active="screen !== 'teach' && screen !== 'motion' && inspector === 'control'"
            /><PanelHost
              id="command"
              :active="screen !== 'teach' && screen !== 'motion' && inspector === 'control'"
            /><PanelHost
              id="deviceParams"
              :active="screen !== 'motion' && (screen === 'teach' ? !directDevice && deviceParameters === 'motor' : inspector === 'parameters')"
            /><PanelHost
              id="feedforward"
              :active="screen !== 'teach' && screen !== 'motion' && inspector === 'compensation'"
            />
          </div>
        </aside>
        <section v-show="screen !== 'teach'" class="observer-area observer surface">
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
                  { id: 'history', title: '命令记录' },
                  { id: 'imu', title: 'IMU' },
                ]"
                :key="item.id"
                :class="{ active: observation === item.id }"
                @click="observation = item.id; observe = true"
              >
                {{ item.title }}
              </button>
            </div>
            <div class="observer-controls">
              <span class="micro">{{ fresh ? "LIVE" : "等待 / 陈旧" }}</span>
            </div>
          </div>
          <div id="observation-content" v-show="observe" class="observation-body">
            <PanelHost
              id="curve"
              :active="screen !== 'teach' && screen !== 'motion' && observe && observation === 'curve'"
            /><PanelHost
              id="events"
              :active="screen !== 'teach' && screen !== 'motion' && observe && observation === 'events'"
            /><CommandHistory
              v-show="screen !== 'teach' && observe && observation === 'history'"
            /><PanelHost id="imu" :active="screen !== 'teach' && screen !== 'motion' && observe && observation === 'imu'" />
          </div>
        </section>
      </div>
    </main>

    <footer class="app-footer">
      <span
        ><i class="dot" :class="snapshot.connected ? 'green' : 'amber'" />ROS
        {{ snapshot.connected ? "已连接" : "未连接"
        }}<span class="footer-separator">|</span>运动流：{{ motionAge }}</span
      ><span class="actual-route"
        :title="snapshot.outputTopic || '尚未收到输出状态'">目标 50 / 50 / 200 Hz · 整体验收待确认</span
      >
    </footer>
    <div v-if="notice" class="toast" role="status">
      <span>{{ notice }}</span
      ><button class="icon-button" aria-label="关闭提示" @click="notice = ''">
        ×
      </button>
    </div>
  </div>
</template>
