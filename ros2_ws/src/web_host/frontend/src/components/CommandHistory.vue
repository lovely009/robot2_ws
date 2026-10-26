<script setup lang="ts">
import { computed } from "vue";
import { commandHistory, runtime } from "../runtime";

const rows = computed(() => commandHistory.value.slice(0, 20));
const number = (value: number) => Number.isFinite(value) ? value.toFixed(3) : "--";
function exportHistory() {
  const content = JSON.stringify(commandHistory.value.slice().reverse(), null, 2);
  const url = URL.createObjectURL(new Blob([content], { type: "application/json" }));
  const link = document.createElement("a");
  link.href = url;
  link.download = `cmd-history-${Date.now()}.json`;
  link.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}
</script>

<template>
  <section class="command-history" data-workbench-panel="history" aria-label="命令记录">
    <div class="history-toolbar">
      <span>{{ commandHistory.length }} 条 · 最近 20 条</span>
      <span class="history-actions">
        <button :disabled="!commandHistory.length" @click="exportHistory">导出 JSON</button>
        <button :disabled="!commandHistory.length" @click="runtime.clearCommandHistory()">清空</button>
      </span>
    </div>
    <p class="history-note">仅记录本页面发送的命令，不代表设备已接收或执行。位置 rad · 速度 rad/s · 力矩 N·m</p>
    <div class="history-table">
      <table>
        <thead><tr><th>时间</th><th>序号</th><th>电机目标：位置 / 速度 / 力矩</th></tr></thead>
        <tbody>
          <tr v-for="(record, index) in rows" :key="`${record.t}-${record.seq}-${index}`">
            <td>{{ new Date(record.t).toLocaleTimeString() }}</td>
            <td>{{ record.seq }}</td>
            <td><div v-for="target in record.targets" :key="target.motor_id">
              电机 {{ target.motor_id }}：{{ number(target.position) }} / {{ number(target.velocity) }} / {{ number(target.effort) }}
            </div></td>
          </tr>
          <tr v-if="!rows.length"><td colspan="3" class="history-empty">暂无记录</td></tr>
        </tbody>
      </table>
    </div>
  </section>
</template>

<style scoped>
.command-history { height: 100%; min-height: 0; display: flex; flex-direction: column; padding: 8px 12px; }
.history-toolbar, .history-actions { display: flex; align-items: center; gap: 8px; }
.history-toolbar { justify-content: space-between; flex-wrap: wrap; }
.history-note { margin: 6px 0; color: var(--muted); font-size: 11px; }
.history-table { min-height: 0; overflow: auto; }
table { width: 100%; border-collapse: collapse; font-size: 12px; }
th, td { padding: 5px 8px; text-align: left; vertical-align: top; }
td { font-variant-numeric: tabular-nums; }
.history-empty { text-align: center; }
</style>
