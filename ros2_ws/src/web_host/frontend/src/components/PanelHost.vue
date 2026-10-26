<script setup lang="ts">
import { onBeforeUnmount, onMounted, ref, watch, nextTick } from "vue";
import type { PanelId } from "../contracts";
import { runtime } from "../runtime";

const props = defineProps<{ id: PanelId; active: boolean }>();
const root = ref<HTMLElement>();
const error = ref("");
const controller = new AbortController();
let release: (() => void) | undefined;
onMounted(async () => {
  try {
    if (root.value)
      release = await runtime.mountPanel(
        props.id,
        root.value,
        () => props.active,
        controller.signal,
      );
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
  <section v-show="active" :data-workbench-panel="id" class="legacy-panel">
    <div v-if="error" class="panel-error" role="alert">
      {{ id }} 加载失败：{{ error }}
    </div>
    <div ref="root" class="panel-content" />
  </section>
</template>
