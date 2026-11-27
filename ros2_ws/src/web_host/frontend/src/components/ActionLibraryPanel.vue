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
        "actionLibrary",
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
  <section v-show="active" data-workbench-panel="actionLibrary" class="legacy-panel">
    <div v-if="error" class="panel-error" role="alert">
      actionLibrary 加载失败：{{ error }}
    </div>
    <div ref="root" class="panel-content">
      <details class="card" data-fold="actionLibrary" open>
        <summary>动作库<span id="actHint" class="info-badge" tabindex="0" aria-label="动作播放说明">!</span><span class="spacer" /><span id="actionSummary" class="sum">--</span></summary>
        <div class="sim-panel">
          <div class="ctrl-row wrap">
            <label class="check" title="播放前自动武装（发 policy：enable=true + kp/kd）；取消勾选则不动策略，需自行保证电机已使能">
              <input id="actEnable" type="checkbox" checked /> 使能
            </label>
            <label class="check" title="播完自动从头再播，直到点停止">
              <input id="actLoop" type="checkbox" /> 循环
            </label>
            <label class="sim-field act-gain">kp<input id="actKp" type="number" value="30" min="0" step="1" /></label>
            <label class="sim-field act-gain">kd<input id="actKd" type="number" value="1" min="0" step="0.1" /></label>
            <span class="spacer" />
            <input id="actFile" type="file" accept=".json,application/json" />
            <button id="btnActImport" class="secondary">导入到动作库</button>
            <button id="btnActStop" class="danger" disabled>停止</button>
          </div>
          <div id="actResult" class="result tiny" role="status" aria-live="polite" />
          <div id="actList" class="act-list" />
        </div>
      </details>
    </div>
  </section>
</template>
