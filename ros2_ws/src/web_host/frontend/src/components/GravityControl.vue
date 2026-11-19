<script setup lang="ts">
import {computed, ref, watch} from 'vue';
import {runtime, snapshot} from '../runtime';

const busy = ref(false);
const error = ref('');
const state = computed(() => snapshot.value.gravity);
const message = computed(() => busy.value ? '等待主机确认' : error.value
  || (state.value.pending ? '等待主机确认'
    : state.value.enabled === true ? state.value.active ? '补偿中' : '已开启 · 待运动'
    : state.value.blocked || '已关闭'));
const context = computed(() => `${snapshot.value.firmwareConnectionKey}:${snapshot.value.model}`);
watch(context, () => {error.value='';});
async function change(event: Event) {
  const input = event.target as HTMLInputElement;
  const enabled = input.checked;
  const startedIn = context.value;
  input.checked = state.value.enabled === true;
  busy.value = true; error.value = '';
  try { await runtime.setGravityEnabled(enabled); }
  catch (cause) {
    if (context.value === startedIn) error.value = cause instanceof Error ? cause.message : String(cause);
  }
  finally { busy.value = false; }
}
</script>

<template>
  <div class="gravity-control" :aria-busy="busy || state.pending">
    <label title="对连续跟随与动作播放叠加重力补偿（系数 1.0）；不自动使能电机">
      <input type="checkbox" :checked="state.enabled === true" :indeterminate="state.enabled === null"
        :disabled="busy || state.pending || state.enabled === null || (state.enabled === false && !!state.blocked)"
        aria-describedby="gravityStatus" @change="change">重力补偿
    </label>
    <span id="gravityStatus" class="gravity-status" role="status" :title="message">{{ message }}</span>
  </div>
</template>
