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
        "readout",
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
  <section v-show="active" data-workbench-panel="readout" class="legacy-panel">
    <div v-if="error" class="panel-error" role="alert">
      readout 加载失败：{{ error }}
    </div>
    <div ref="root" class="panel-content">
      <details class="card" data-fold="sel" open>
        <summary><span id="selTitle">电机状态</span><span class="spacer" /><span id="selSum" class="sum">--</span></summary>
        <div id="motorCards" class="motor-cards" hidden />
        <div class="table-wrap">
          <table class="jstates">
            <thead>
              <tr><th>ID</th><th class="num">位置</th><th class="num">力矩</th><th class="num">温度</th><th>状态</th></tr>
            </thead>
            <tbody id="selRows">
              <tr class="empty"><td colspan="5">等待 motor_topology…</td></tr>
            </tbody>
          </table>
        </div>
      </details>
    </div>
  </section>
</template>
