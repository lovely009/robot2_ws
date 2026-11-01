<script setup lang="ts">
import { ref, watch } from 'vue';
import { snapshot } from '../runtime';
type Limit = { id: number; enabled: boolean; min: number; max: number };
defineProps<{editing: boolean}>();
const rows = ref<Limit[]>([]);
const saved = ref<Limit[]>([]);
const message = ref('');
const loaded = ref(false);
const modulePath = '/js/core/host_limits.js';
let api: {readLimits(model: string): Limit[]; saveLimits(model: string, rows: Limit[]): void};
watch(() => snapshot.value.model, async (model) => {
  loaded.value = false;
  if (!model) return;
  try {
    api = await import(/* @vite-ignore */ modulePath);
    if (model !== snapshot.value.model) return;
    saved.value = api.readLimits(model);
    rows.value = saved.value.map(row => ({...row}));
    message.value = '';
  } catch (error) {
    saved.value = [];
    rows.value = [];
    message.value = `配置读取失败：${String(error)}；检查设置后重新保存`;
  }
  loaded.value = true;
}, {immediate: true});
function add() {
  const id = snapshot.value.selectedId || 1;
  if (rows.value.some(row => row.id === id)) {
    message.value = `m${id} 已有设置，请编辑对应行`;
    return;
  }
  rows.value.push({id, enabled: false, min: 0, max: 0});
  message.value = '请填写实际行程，再勾选启用并保存';
}
function save() {
  try {
    api.saveLimits(snapshot.value.model, rows.value.map(row => ({...row})));
    saved.value = api.readLimits(snapshot.value.model);
    message.value = '已保存，后续 Web 下发按此设置检查';
  } catch (error) { message.value = String(error); }
}
</script>
<template>
  <section v-if="editing || !loaded || saved.length || message" class="host-limits">
    <div class="saved-limits" aria-label="当前已保存主机限位">
      <strong>当前已保存主机限位 · rad</strong>
      <p v-if="!loaded">正在读取…</p>
      <p v-else-if="!saved.length">暂无已保存限位</p>
      <p v-for="row in saved" :key="row.id">
        m{{ row.id }}：{{ row.min }} ～ {{ row.max }} rad · {{ row.enabled ? '已启用' : '未启用' }}
      </p>
    </div>
    <template v-if="editing">
    <h3>编辑主机侧角度限位</h3>
    <p>模型：{{ snapshot.model || '等待模型' }} · 单位：rad</p>
    <p>仅约束本 Web 的手动目标及动作播放。越界拒绝下发，设置保存在当前浏览器，按模型区分。请停止动作后修改；保存不会停止或修改已下发的命令。</p>
    <div v-for="(row, index) in rows" :key="index" class="host-limit-row">
      <label>电机 ID<input v-model.number="row.id" type="number" min="1" max="65535" step="1" /></label>
      <label>最小位置 rad<input v-model.number="row.min" type="number" step="0.001" /></label>
      <label>最大位置 rad<input v-model.number="row.max" type="number" step="0.001" /></label>
      <label><input v-model="row.enabled" type="checkbox" />启用</label>
      <button @click="rows.splice(index, 1)">删除</button>
    </div>
    <div class="host-limit-buttons">
      <button :disabled="!loaded" @click="add">添加电机</button>
      <button :disabled="!loaded" @click="save">保存限位</button>
    </div>
    <p role="status">{{ message }}</p>
    </template>
  </section>
</template>
<style scoped>
.saved-limits { padding: 10px; border: 1px solid var(--border); border-radius: 6px; overflow-wrap: anywhere; }
.host-limits { padding: 4px; font-size: 12px; }
.host-limits p { line-height: 1.7; color: var(--muted); }
.host-limit-row { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; padding: 14px 0; border-bottom: 1px solid var(--border); }
.host-limit-row label { display: flex; flex-direction: column; gap: 6px; }
.host-limit-row input[type="number"] { width: 100%; min-width: 0; }
.host-limit-row label:has(input[type="checkbox"]) { flex-direction: row; align-items: center; }
.host-limit-buttons { display: flex; gap: 10px; margin-top: 14px; }
</style>
