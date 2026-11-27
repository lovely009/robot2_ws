<script setup lang="ts">
import { nextTick, onBeforeUnmount, onMounted, ref, watch } from "vue";
import { runtime } from "../runtime";

const props = defineProps<{ active: boolean }>();
const root = ref<HTMLElement>();
const error = ref("");
const controller = new AbortController();
let release: (() => void) | undefined;

onMounted(async () => {
  try {
    if (root.value) {
      release = await runtime.mountPanel(
        "jointList",
        root.value,
        () => props.active,
        controller.signal,
      );
    }
  } catch (reason) {
    error.value = reason instanceof Error ? reason.message : String(reason);
  }
});

watch(
  () => props.active,
  async () => {
    await nextTick();
    runtime.wake();
  },
);

onBeforeUnmount(() => {
  controller.abort();
  release?.();
});
</script>

<template>
  <section v-show="active" data-workbench-panel="jointList" class="legacy-panel">
    <div v-if="error" class="panel-error" role="alert">
      jointList 加载失败：{{ error }}
    </div>
    <div ref="root" class="panel-content">
      <details class="card" data-fold="joints" open>
        <summary title="关节表来自模型 URDF；数值与年龄来自 /joint_states，TF 由 robot_state_publisher 发布">
          关节 <span class="spacer" />
          <label class="joint-control-toggle" title="使能当前模型的在线电机，允许部分传动组；状态以 MCU 回执为准">
            <input id="jointControlEnabled" type="checkbox" />使能控制</label>
          <button id="jointControlDisable" type="button" hidden>失能电机</button>
          <button id="btnJointUnit" type="button" class="tiny-btn secondary" aria-label="切换角度单位">rad</button>
          <span id="sumJoints" class="sum">--</span>
        </summary>
        <div id="jointControlResult" class="tiny muted" role="status" />
        <div id="jointFeedbackSource" class="tiny muted" />
        <input id="jointFilter" type="search" class="joint-filter" placeholder="筛选关节…" aria-label="筛选关节" hidden />
        <ul id="jointRows" class="joints">
          <li class="empty">等待模型 URDF…</li>
        </ul>
      </details>
    </div>
  </section>
</template>
