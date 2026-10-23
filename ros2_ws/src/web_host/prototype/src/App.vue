<script setup lang="ts">
import { computed, onMounted, onBeforeUnmount, ref } from "vue";
import {
  Box,
  SlidersHorizontal,
  Film,
  Workflow,
  ChevronRight,
  ChevronLeft,
  Radio,
  PanelBottom,
  CircleHelp,
  FlaskConical,
  Cable,
  X,
  CircleStop,
  LayoutDashboard,
  ArrowUpRight,
  WifiOff,
  Check,
  Command,
} from "@lucide/vue";
import JointTree from "./components/JointTree.vue";
import RobotViewport from "./components/RobotViewport.vue";
import Inspector from "./components/Inspector.vue";
import DeviceTable from "./components/DeviceTable.vue";
import Observer from "./components/Observer.vue";
import ActionPanel from "./components/ActionPanel.vue";
import SimPanel from "./components/SimPanel.vue";
import {
  demo,
  models,
  model,
  motors,
  connected,
  labels,
  go,
  choose_model,
  set_variant,
  tick,
  disconnect,
  log,
  stop,
  cleanup,
} from "./demo";
import type { RobotModel, Screen, Variant } from "./types";
const dev_mode = import.meta.env.DEV;
const ready = ref(false),
  failure = ref(""),
  help = ref(false),
  observe = ref(true);
const scenes = [
  {
    id: "run",
    icon: SlidersHorizontal,
    number: "01",
    description: "核对设备，调整参数，观察反馈",
  },
  {
    id: "teach",
    icon: Film,
    number: "02",
    description: "组织姿态与关键帧，预览动作",
  },
  {
    id: "sim",
    icon: Workflow,
    number: "03",
    description: "配置后端与控制端，验证连接关系",
  },
] as const;
const variants: { id: Variant; label: string }[] = [
  { id: "A", label: "均衡工作台" },
  { id: "B", label: "3D 优先" },
  { id: "C", label: "表格优先" },
];
const variant_name = computed(
  () => variants.find((v) => v.id === demo.variant)?.label,
);
const variant_index = computed(
  () => variants.findIndex((v) => v.id === demo.variant) + 1,
);
const description = computed(
  () => scenes.find((s) => s.id === demo.screen)?.description,
);
let timer: ReturnType<typeof setInterval>;
function cycle(delta: number) {
  const i = variants.findIndex((v) => v.id === demo.variant);
  set_variant(variants[(i + delta + 3) % 3]!.id);
}
function keyboard(e: KeyboardEvent) {
  if (
    e.ctrlKey ||
    e.metaKey ||
    e.altKey ||
    e.target instanceof HTMLInputElement ||
    e.target instanceof HTMLTextAreaElement ||
    e.target instanceof HTMLSelectElement ||
    (e.target as HTMLElement)?.isContentEditable
  )
    return;
  if (e.key === "ArrowRight" || e.key === "ArrowLeft") {
    e.preventDefault();
    cycle(e.key === "ArrowRight" ? 1 : -1);
  }
  if (e.key === "Escape") {
    stop();
    help.value = false;
  }
}
function hashchange() {
  const s = location.hash.slice(2);
  if (["run", "teach", "sim"].includes(s)) demo.screen = s as Screen;
}
onMounted(async () => {
  try {
    const response = await fetch("/api/prototype-models");
    if (!response.ok) throw new Error(`模型请求失败 ${response.status}`);
    models.value = (await response.json()) as RobotModel[];
    const initial =
      models.value.find((m) => m.joints.some((j) => j.motor)) ||
      models.value[0];
    if (!initial) throw new Error("没有可用的模型资产");
    choose_model(initial.name);
    ready.value = true;
    log("交互原型已就绪 · 无 ROS 连接 · 所有控制仅改变模拟状态", "ok");
    timer = setInterval(() => tick(0.1), 100);
  } catch (error) {
    failure.value = String(error);
  }
  window.addEventListener("keydown", keyboard);
  window.addEventListener("hashchange", hashchange);
});
onBeforeUnmount(() => {
  clearInterval(timer);
  cleanup();
  window.removeEventListener("keydown", keyboard);
  window.removeEventListener("hashchange", hashchange);
});
</script>

<template>
  <div
    class="workbench"
    :class="[`variant-${demo.variant}`, { 'observer-hidden': !observe }]"
  >
    <header class="app-header">
      <a class="brand" href="#/run" @click="go('run')"
        ><span class="brand-mark"><Box :size="22" :stroke-width="1.5" /></span
        ><span>ROBOT<span class="brand-light"> WORKBENCH</span></span></a
      >
      <div class="header-divider" />
      <span class="workspace-name"
        >机器人实验室<ChevronRight :size="12" /><span>开发工作区</span></span
      >
      <span class="header-spacer" />
      <span class="prototype-badge"
        ><FlaskConical :size="12" />交互原型 · 模拟数据</span
      >
      <button
        class="icon-button help-button"
        aria-label="使用说明"
        @click="help = true"
      >
        <CircleHelp :size="17" />
      </button>
      <span class="avatar">LY</span>
    </header>

    <div class="session-bar">
      <div class="session-item model-picker">
        <Box :size="14" /><span class="session-label">模型</span
        ><select
          :value="demo.model"
          aria-label="机器人模型"
          @change="choose_model(($event.target as HTMLSelectElement).value)"
        >
          <option v-for="m in models" :key="m.name" :value="m.name">
            {{ m.name }}
          </option>
        </select>
      </div>
      <span class="session-separator" />
      <div class="session-item">
        <span class="session-label">控制会话</span
        ><span class="mode-chip">{{ demo.target }} · 模拟</span>
      </div>
      <div class="session-item connection">
        <i class="dot" :class="connected ? 'green' : 'amber'" /><span>{{
          connected ? "模拟数据流正常" : "模拟数据流已停止"
        }}</span
        ><span v-if="connected" class="micro mono">10 Hz</span>
      </div>
      <span class="header-spacer" /><button
        class="session-button"
        @click="disconnect"
      >
        <Cable v-if="demo.connected" :size="13" /><WifiOff
          v-else
          :size="13"
        />{{ demo.connected ? "模拟断连" : "恢复连接" }}</button
      ><button
        class="stop-button"
        :disabled="!demo.executing && !demo.preview"
        @click="
          stop();
          log('用户停止原型动作', 'warn');
        "
      >
        <CircleStop :size="14" />停止动作
      </button>
    </div>

    <nav class="main-nav" aria-label="工作场景">
      <div class="nav-section-label">工作场景</div>
      <button
        v-for="s in scenes"
        :key="s.id"
        :class="{ active: demo.screen === s.id }"
        @click="go(s.id)"
      >
        <component :is="s.icon" :size="18" /><span>{{ labels[s.id] }}</span
        ><small>{{ s.number }}</small>
      </button>
      <div class="nav-bottom">
        <button @click="observe = !observe">
          <PanelBottom :size="17" /><span>{{
            observe ? "收起观察区" : "展开观察区"
          }}</span></button
        ><button @click="help = true">
          <CircleHelp :size="17" /><span>使用说明</span>
        </button>
        <div class="branch-label">
          <i class="dot green" />WEB / REFACTOR<span>本地原型 · v0.1</span>
        </div>
      </div>
    </nav>

    <main class="main-workspace">
      <div class="workspace-heading">
        <div>
          <div class="eyebrow">
            ROBOTICS /
            {{
              demo.screen === "run"
                ? "COMMISSIONING"
                : demo.screen === "teach"
                  ? "MOTION STUDIO"
                  : "SIMULATION LAB"
            }}
          </div>
          <h1>
            {{ labels[demo.screen] }}<span class="heading-slash">/</span
            ><span class="heading-description">{{ description }}</span>
          </h1>
        </div>
        <div class="workspace-summary">
          <span
            ><i class="dot" :class="connected ? 'green' : 'muted'" />{{
              connected ? motors.length : 0
            }}<small>映射设备有模拟数据</small></span
          ><button class="secondary compact" @click="observe = !observe">
            <LayoutDashboard :size="13" />{{ variant_name }}
          </button>
        </div>
      </div>
      <div v-if="failure" class="load-failure">{{ failure }}</div>
      <div v-else-if="!ready" class="load-failure">正在读取模型和拓扑…</div>
      <div v-else class="workspace-grid">
        <JointTree class="objects-area" />
        <div class="stage-area" :class="`stage-${demo.screen}`">
          <div class="stage-toolbar">
            <div class="breadcrumb">
              <Box :size="14" /><span>{{ demo.model }}</span
              ><ChevronRight :size="12" /><b>{{
                demo.screen === "run"
                  ? "设备调试"
                  : demo.screen === "teach"
                    ? "姿态与轨迹"
                    : "闭环会话"
              }}</b>
            </div>
            <span class="tiny-tag">{{
              demo.screen === "sim" ? demo.backend : "URDF"
            }}</span>
          </div>
          <div class="stage-body">
            <RobotViewport /><DeviceTable
              v-if="demo.screen === 'run'"
            /><ActionPanel v-else-if="demo.screen === 'teach'" /><SimPanel
              v-else
            />
          </div>
        </div>
        <Inspector class="inspector-area" />
        <Observer v-if="observe" class="observer-area" />
      </div>
    </main>

    <footer class="app-footer">
      <span
        ><i class="dot green" />素材：仓库模型 / YAML<span
          class="footer-separator"
          >|</span
        >实时状态：本地模拟</span
      ><span
        >命令去向：仅原型内存<span class="footer-separator">|</span
        ><Command :size="11" /> Esc 停止原型动作</span
      >
    </footer>
    <div v-if="dev_mode" class="prototype-switcher">
      <span class="switcher-tag">布局实验</span
      ><button aria-label="上一个布局" @click="cycle(-1)">
        <ChevronLeft :size="16" /></button
      ><span
        ><b>{{ demo.variant }}</b
        >{{ variant_name }}</span
      ><button aria-label="下一个布局" @click="cycle(1)">
        <ChevronRight :size="16" /></button
      ><span class="switcher-count">{{ variant_index }} / 3</span>
    </div>
    <div v-if="demo.message" class="toast" role="status">
      <Check :size="15" /><span>{{ demo.message }}</span
      ><button
        class="icon-button"
        aria-label="关闭提示"
        @click="demo.message = ''"
      >
        <X :size="14" />
      </button>
    </div>
    <div v-if="help" class="modal-backdrop" @click.self="help = false">
      <section
        class="help-modal"
        role="dialog"
        aria-modal="true"
        aria-label="原型说明"
      >
        <div class="panel-title">
          <span><FlaskConical :size="16" />工作台交互原型</span
          ><button
            class="icon-button"
            aria-label="关闭说明"
            @click="help = false"
          >
            <X :size="17" />
          </button>
        </div>
        <h2>先试操作，再定布局。</h2>
        <p>
          模型与设备映射来自当前仓库，位置曲线和所有操作反馈都由本地模拟生成。此页面没有
          ROS 连接，不会启动进程或驱动设备。
        </p>
        <ol>
          <li>点击对象树或 3D 关节点，查看右侧详情和底部曲线。</li>
          <li>修改目标位置，再到动作工作台添加关键帧并预览。</li>
          <li>在仿真页停止模拟会话后，更换后端或控制端。</li>
          <li>用底部切换器比较三种布局，也可以按左右方向键。</li>
        </ol>
        <p class="muted-copy">
          模型缺少设备映射时仅提供预览。所有草稿仅在当前页面内存中保存，刷新即重置。
        </p>
        <button class="primary full" @click="help = false">
          开始体验<ArrowUpRight :size="14" />
        </button>
      </section>
    </div>
  </div>
</template>
