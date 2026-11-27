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
        "imu",
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
  <section v-show="active" data-workbench-panel="imu" class="legacy-panel">
    <div v-if="error" class="panel-error" role="alert">
      imu 加载失败：{{ error }}
    </div>
    <div ref="root" class="panel-content">
      <details class="card" data-fold="imu" open>
        <summary>IMU<span class="spacer" /><span id="imuHz" class="sum">-- Hz</span></summary>
        <div class="imu-wrap">
          <div class="kv">
            <div><span>Roll</span><b id="imuRoll">--</b></div>
            <div><span>Pitch</span><b id="imuPitch">--</b></div>
            <div><span>Yaw</span><b id="imuYaw">--</b></div>
            <div><span>温度</span><b id="imuTemp">--</b></div>
            <div><span>更新</span><b id="imuAge">--</b></div>
            <div><span>四元数</span><b id="imuQuat" class="muted">--</b></div>
          </div>
          <canvas id="imu3d" width="200" height="200" class="imu3d" title="3D 姿态" />
          <canvas id="attitudeHorizon" width="260" height="150" class="horizon" title="人工地平仪" />
        </div>
      </details>
    </div>
  </section>
</template>
