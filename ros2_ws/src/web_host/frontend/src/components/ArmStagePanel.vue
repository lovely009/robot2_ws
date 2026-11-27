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
        "arm",
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
  <section v-show="active" data-workbench-panel="arm" class="legacy-panel">
    <div v-if="error" class="panel-error" role="alert">
      arm 加载失败：{{ error }}
    </div>
    <div ref="root" class="panel-content">
      <div class="arm-topbar">
        <select
          id="armModel"
          class="arm-model"
          title="臂视图与关节映射使用的模型；选择会记住（刷新后保持），不影响仿真屏的模型选择"
        />
        <div id="armGravityControl"><slot /></div>
        <span class="spacer" />
        <div class="arm-cam-views" role="group" aria-label="预设视角">
          <button type="button" class="tiny-btn secondary" data-cam-view="front" aria-label="正视图">正</button>
          <button type="button" class="tiny-btn secondary" data-cam-view="side" aria-label="侧视图">侧</button>
          <button type="button" class="tiny-btn secondary" data-cam-view="top" aria-label="顶视图">顶</button>
          <button type="button" class="tiny-btn secondary" data-cam-view="reset" aria-label="复位视角">复位</button>
        </div>
      </div>
      <div id="armStage" class="stage">
        <div class="legend">
          <span><i class="sw" style="background:#3d9bff" />实际</span>
          <span><i class="sw ghost" />目标</span>
          <span><i class="dt" style="background:#2fd07a" />就绪</span>
          <span><i class="dt" style="background:#e0a93a" />过渡</span>
          <span><i class="dt" style="background:#ff5f5f" />故障</span>
          <span class="info-badge" tabindex="0" title="拖拽旋转 · 滚轮缩放 · 点击关节选中" aria-label="视图操作说明">!</span>
        </div>
      </div>
    </div>
  </section>
</template>
