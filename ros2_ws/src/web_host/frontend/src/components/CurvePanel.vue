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
        "curve",
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
  <section v-show="active" data-workbench-panel="curve" class="legacy-panel">
    <div v-if="error" class="panel-error" role="alert">
      curve 加载失败：{{ error }}
    </div>
    <div ref="root" class="panel-content">
      <div class="ctrl-row curve-controls">
        <div class="curve-field">
          <span id="chartMotorLabel" class="curve-field-label">电机</span>
          <details id="chartMotor" class="curve-multiselect">
            <summary aria-labelledby="chartMotorLabel" aria-haspopup="true">
              <span data-chart-motor-summary>电机 1</span>
            </summary>
            <div class="curve-multiselect-menu" data-chart-motor-options role="group" aria-labelledby="chartMotorLabel">
              <label class="curve-option">
                <input type="checkbox" name="chart-motor" value="1" data-motor-id="1" checked />
                <span>电机 1</span>
              </label>
            </div>
          </details>
        </div>
        <div class="curve-field">
          <span id="chartChannelLabel" class="curve-field-label">通道</span>
          <details id="chartCh" class="curve-multiselect">
            <summary aria-labelledby="chartChannelLabel" aria-haspopup="true">
              <span data-chart-channel-summary>位置 rad</span>
            </summary>
            <div class="curve-multiselect-menu" role="group" aria-labelledby="chartChannelLabel">
              <label class="curve-option">
                <input type="checkbox" name="chart-channel" value="position" checked />
                <span>位置</span><span class="curve-unit">rad</span>
              </label>
              <label class="curve-option">
                <input type="checkbox" name="chart-channel" value="velocity" />
                <span>速度</span><span class="curve-unit">rad/s</span>
              </label>
              <label class="curve-option">
                <input type="checkbox" name="chart-channel" value="effort" />
                <span>力矩</span><span class="curve-unit">N·m</span>
              </label>
              <label class="curve-option">
                <input type="checkbox" name="chart-channel" value="current" />
                <span>电流</span><span class="curve-unit">A</span>
              </label>
            </div>
          </details>
        </div>
        <span id="chartHint" class="muted tiny">10 s · 滚轮调整 1–60 s</span>
        <span class="spacer" />
        <button id="btnChartPause" class="tiny-btn">暂停</button>
        <button id="btnChartClear" class="tiny-btn secondary">清空</button>
        <button id="btnChartCsv" class="tiny-btn secondary">导出 CSV</button>
        <button id="btnChartPng" class="tiny-btn secondary">保存图</button>
      </div>
      <canvas
        id="chart"
        width="980"
        height="380"
        class="chart"
        tabindex="0"
        aria-label="电机通道曲线，滚轮调整 1 到 60 秒时间范围"
      />
      <div id="chartLegend" class="chart-legend" aria-label="曲线图例" />
    </div>
  </section>
</template>
