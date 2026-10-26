<script setup lang="ts">
import { onMounted, ref } from 'vue';
const props = defineProps<{
  label: string;
  target: string;
  variable: string;
  axis: 'width' | 'height';
  min: number;
  reverse?: boolean;
}>();
const handle = ref<HTMLElement>();
let origin = 0;
let initial = 0;
let active = false;
const scope = () => handle.value?.closest<HTMLElement>('.workspace-grid');
const key = () => `robot2.layout.${props.variable}`;
function apply(size: number) {
  const root = scope();
  if (!root) return;
  const container = props.axis === 'height' && props.target !== '.observer-area'
    ? root.querySelector('.stage-body') : root;
  const available = container?.getBoundingClientRect()[props.axis] || 600;
  let maximum = available * 0.6;
  if (props.axis === 'width') {
    const other = root.querySelector(props.target === '.objects-area' ? '.inspector-area' : '.objects-area');
    maximum = available - (other?.getBoundingClientRect().width || 0) - 280 - 24;
  } else if (props.target !== '.observer-area') {
    const other = root.querySelector(props.target === '#run-joints' ? '.action-panel-host' : '#run-joints');
    maximum = available - (other?.getBoundingClientRect().height || 0) - 200 - 24;
  }
  const value = Math.round(Math.max(props.min, Math.min(maximum, size)));
  root.style.setProperty(props.variable, `${value}px`);
  localStorage.setItem(key(), String(value));
}
function start(event: PointerEvent) {
  if (event.button !== 0) return;
  const target = scope()?.querySelector(props.target);
  if (!target) return;
  initial = target.getBoundingClientRect()[props.axis];
  origin = props.axis === 'width' ? event.clientX : event.clientY;
  active = true;
  (event.currentTarget as HTMLElement).setPointerCapture(event.pointerId);
  event.preventDefault();
}
function move(event: PointerEvent) {
  if (!active) return;
  const delta = (props.axis === 'width' ? event.clientX : event.clientY) - origin;
  apply(initial + delta * (props.reverse ? -1 : 1));
}
function stop() { active = false; }
function keyboard(event: KeyboardEvent) {
  const negative = props.axis === 'width' ? 'ArrowLeft' : 'ArrowUp';
  const positive = props.axis === 'width' ? 'ArrowRight' : 'ArrowDown';
  if (event.key !== negative && event.key !== positive) return;
  const target = scope()?.querySelector(props.target);
  if (!target) return;
  event.preventDefault();
  apply(target.getBoundingClientRect()[props.axis]
    + (event.key === positive ? 16 : -16) * (props.reverse ? -1 : 1));
}
onMounted(() => {
  const saved = Number(localStorage.getItem(key()));
  if (Number.isFinite(saved) && saved >= props.min)
    scope()?.style.setProperty(props.variable, `${saved}px`);
});
</script>

<template>
  <div ref="handle" class="resize-handle" :class="axis === 'width' ? 'resize-vertical' : 'resize-horizontal'"
    role="separator" tabindex="0" :aria-label="label" :title="label"
    :aria-orientation="axis === 'width' ? 'vertical' : 'horizontal'"
    @pointerdown="start" @pointermove="move" @pointerup="stop" @pointercancel="stop"
    @lostpointercapture="stop" @keydown="keyboard" />
</template>
