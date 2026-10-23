<script setup lang="ts">
import {
  Plus,
  Play,
  Square,
  Film,
  Trash2,
  Send,
  GripVertical,
} from "@lucide/vue";
import {
  demo,
  frame_end,
  capture,
  toggle_preview,
  execute,
  stop,
  connected,
  motors,
} from "../demo";
function remove(index: number) {
  if (demo.executing || index === 0) return;
  demo.frames.splice(index, 1);
  demo.frames.forEach((f, i) => (f.time = i * 2));
  demo.frame = Math.min(demo.frame, frame_end.value);
}
</script>
<template>
  <section class="action-panel surface">
    <div class="panel-title">
      <span><Film :size="14" />动作编辑器</span
      ><span class="micro">本地草稿 · {{ demo.frames.length }} 帧</span>
    </div>
    <div class="action-toolbar">
      <div>
        <b>未命名动作</b
        ><small>编辑关节目标 → 添加关键帧 → 预览 → 模拟执行</small>
      </div>
      <button class="secondary" :disabled="demo.executing" @click="capture">
        <Plus :size="13" />添加关键帧
      </button>
    </div>
    <div class="frame-strip">
      <div
        v-for="(f, i) in demo.frames"
        role="button"
        tabindex="0"
        :key="i"
        class="frame-card"
        :class="{ active: Math.abs(demo.frame - f.time) < 0.1 }"
        :aria-disabled="demo.executing"
        @click="
          if (!demo.executing) {
            demo.preview = false;
            demo.frame = f.time;
          }
        "
        @keydown.enter="
          if (!demo.executing) {
            demo.preview = false;
            demo.frame = f.time;
          }
        "
      >
        <span
          ><GripVertical :size="12" />{{ String(i + 1).padStart(2, "0")
          }}<button
            v-if="i"
            class="icon-button"
            :disabled="demo.executing"
            :aria-label="`删除${f.name}`"
            @click.stop="remove(i)"
          >
            <Trash2 :size="11" /></button></span
        ><b>{{ f.name }}</b
        ><small class="mono">{{ f.time.toFixed(1) }} s</small>
      </div>
    </div>
    <div class="playback">
      <button
        class="icon-button"
        :disabled="demo.executing || demo.frames.length < 2"
        :aria-label="demo.preview ? '暂停预览' : '开始预览'"
        @click="toggle_preview"
      >
        <Play :size="15" /></button
      ><button
        class="icon-button"
        aria-label="停止预览或模拟动作"
        @click="stop"
      >
        <Square :size="12" /></button
      ><input
        v-model.number="demo.frame"
        type="range"
        min="0"
        :max="frame_end"
        step="0.01"
        :disabled="demo.executing"
        aria-label="动作时间轴"
        @input="demo.preview = false"
      /><span class="mono"
        >{{ demo.frame.toFixed(1) }} /
        {{ demo.frames.at(-1)?.time.toFixed(1) || "0.0" }} s</span
      ><button
        class="primary"
        :disabled="
          !connected ||
          !motors.length ||
          demo.frames.length < 2 ||
          demo.executing
        "
        @click="execute"
      >
        <Send :size="12" />{{ demo.executing ? "模拟执行中" : "模拟执行" }}
      </button>
    </div>
    <div class="action-note">
      <i class="dot" :class="demo.executing ? 'amber' : 'green'" />{{
        demo.executing
          ? `模拟执行进度 ${Math.round(demo.progress * 100)}% · 不向设备发送命令`
          : demo.preview
            ? "本地预览中 · 仅目标叠影运动"
            : "拖动时间轴只预览目标姿态，不下发命令"
      }}
    </div>
  </section>
</template>
