<script lang="ts">
// Stable across start/stop and scene changes in this tab; another viewer has its own receiver claim.
const viewer = Array.from(crypto.getRandomValues(new Uint8Array(16)), n => n.toString(16).padStart(2, '0')).join('');
</script>

<script setup lang="ts">
import { onBeforeUnmount, onMounted, ref } from 'vue';
import { createMotionPreview } from '../panels/motion_preview';

const props = defineProps<{ model: string }>();
const canvas = ref<HTMLCanvasElement>();
const ready = ref(false);
const state = ref('connecting');
const detail = ref('');
const endpoint = ref('');
const frameIndex = ref<number>();
const frequency = ref<number>();
const modelStatus = ref('');
const labels: Record<string, string> = {
  connecting: '正在连接动捕接收端', waiting: '等待 DreamsCapStudio 播放', streaming: '实时跟随中',
  paused: '播放已暂停，等待新帧', disconnected: '广播中断，等待恢复', invalid: '已跳过无效帧',
  error: '实时跟随已中断',
};
type Rig = { names: string[]; parents: number[] };
type Scene = { source_to_robot: number[][]; source: Rig; robot: Rig & { joint_names: string[]; urdf: string } };
type Frame = { index: number; frequency: number; source: number[][]; robot: number[][];
  joint_positions: number[]; root_quaternion: number[] };
let controller: AbortController | undefined;
let display: { updateLiveFrame(pose: Frame): void; dispose(): void } | undefined;
let animation = 0;

function release() {
  controller?.abort();
  cancelAnimationFrame(animation);
  animation = 0;
  display?.dispose();
  display = undefined;
}

async function connect() {
  release();
  const abort = new AbortController();
  controller = abort;
  state.value = 'connecting'; detail.value = ''; ready.value = false;
  frameIndex.value = undefined; endpoint.value = ''; modelStatus.value = '';
  let scene: Scene | undefined, latest: Frame | undefined;
  let flushLastFrame: (() => void) | undefined;
  function fail(error: unknown) {
    if (abort.signal.aborted) return;
    state.value = 'error';
    detail.value = error instanceof Error ? error.message : String(error);
    abort.abort();
    cancelAnimationFrame(animation);
    animation = 0;
  }
  try {
    const response = await fetch('/api/motion/live/vdmocap', { method: 'POST', signal: abort.signal,
      headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ model: props.model, viewer }) });
    if (!response.ok) throw Error((await response.json()).error || `HTTP ${response.status}`);
    if (!response.body) throw Error('浏览器未提供动捕数据流');
    const reader = response.body.pipeThrough(new TextDecoderStream()).getReader();
    let pending = '';
    const paint = () => {
      animation = 0;
      if (!latest || !scene || !canvas.value || abort.signal.aborted) return;
      try {
        const pose = latest;
        if (display) display.updateLiveFrame(pose);
        else {
          display = createMotionPreview(canvas.value, { times: [0], duration: 0, source_to_robot: scene.source_to_robot,
            source: { ...scene.source, frames: [pose.source] }, robot: { ...scene.robot, frames: [pose.robot],
              joint_positions: [pose.joint_positions], root_quaternions: [pose.root_quaternion] } },
            { live: true, onModelStatus: (text: string) => { modelStatus.value = text; } });
        }
        frameIndex.value = pose.index; frequency.value = pose.frequency; ready.value = true;
      } catch (error) { fail(error); }
    };
    flushLastFrame = paint;
    while (!abort.signal.aborted) {
      const { done, value } = await reader.read();
      if (done) throw Error('接收端已关闭连接');
      pending += value;
      const lines = pending.split('\n');
      pending = lines.pop()!;
      for (const line of lines) {
        if (!line) continue;
        const event = JSON.parse(line);
        if (event.type === 'error') throw Error(event.message);
        if (event.type === 'scene') scene = event;
        if (event.type === 'state') {
          state.value = event.state;
          detail.value = event.message || '';
          if (event.endpoint) endpoint.value = event.endpoint;
        }
        if (event.type === 'frame') {
          if (!scene) throw Error('接收端尚未发送骨架');
          state.value = 'streaming'; detail.value = '';
          latest = event;
          if (!animation) animation = requestAnimationFrame(paint);
        }
      }
    }
  } catch (error) {
    if (animation && !abort.signal.aborted) {
      cancelAnimationFrame(animation);
      flushLastFrame?.();
    }
    fail(error);
  }
}

onMounted(connect);
onBeforeUnmount(release);
</script>

<template>
  <div class="live-preview">
    <canvas v-show="ready" ref="canvas" aria-label="实时人体与机器人三维预览" />
    <div class="live-status" :class="{ overlay: ready }">
      <strong role="status">{{ labels[state] }}</strong>
      <span v-if="endpoint">{{ endpoint }} / UDP</span>
      <span v-if="frameIndex !== undefined" aria-label="实时源帧">帧 {{ frameIndex }} · {{ frequency }} Hz</span>
      <span v-if="ready">{{ modelStatus }}</span>
      <span v-if="detail">{{ detail }}</span>
      <button v-if="state === 'error'" type="button" @click="connect">重新连接</button>
    </div>
  </div>
</template>

<style scoped>
.live-preview, canvas { position: absolute; inset: 0; width: 100%; height: 100%; }
.live-status { position: absolute; inset: 0; display: flex; gap: 7px; flex-direction: column; align-items: center; justify-content: center; color: var(--muted); font-size: 13px; pointer-events: none; }
.live-status strong { color: var(--text); }
.live-status.overlay { inset: auto auto 12px 12px; background: #101719ec; border: 1px solid var(--line-2); border-radius: 5px; align-items: start; padding: 8px 12px; font-size: 12px; }
.live-status button { pointer-events: auto; }
</style>
