<script setup lang="ts">
import {computed} from 'vue';
import {eventLog, runtime} from '../runtime';

const visibleEvents = computed(() => eventLog.value.slice(0, 50));

function exportEvents(csv: boolean) {
  // 保留现有导出契约：JSON 为逐行记录，CSV 无表头，均导出完整缓冲。
  const rows = eventLog.value.map(event => csv
    ? [new Date(event.t).toISOString(), event.name, event.id, event.reason ?? '', event.request_id ?? ''].join(',')
    : JSON.stringify(event));
  const url = URL.createObjectURL(new Blob([rows.join('\n')], {
    type: csv ? 'text/csv' : 'application/json',
  }));
  const link = document.createElement('a');
  link.href = url;
  link.download = `events-${Date.now()}${csv ? '.csv' : '.json'}`;
  link.click();
  setTimeout(() => URL.revokeObjectURL(url), 3000);
}
</script>

<template>
  <section class="event-log" data-workbench-panel="events" aria-label="事件日志">
    <div class="ctrl-row wrap">
      <span class="sum" title="/motor/events">{{ eventLog.length ? `${eventLog.length} 条` : '空' }}</span>
      <span class="spacer"></span>
      <button class="secondary" @click="exportEvents(false)">导出 JSON</button>
      <button class="secondary" @click="exportEvents(true)">导出 CSV</button>
      <button class="secondary" @click="runtime.clearEvents()">清空</button>
    </div>
    <ul class="events">
      <li v-for="(event, index) in visibleEvents" :key="`${event.t}:${index}`">
        <span class="muted">{{ new Date(event.t).toLocaleTimeString() }}</span>
        <b>{{ event.name }}</b> · id={{ event.id }}<template v-if="event.reason"> · reason={{ event.reason }}</template><template v-if="event.request_id"> · req#{{ event.request_id }}</template>
      </li>
      <li v-if="!eventLog.length" class="muted">暂无事件</li>
    </ul>
  </section>
</template>

<style scoped>
.event-log { height: 100%; min-height: 0; overflow: auto; }
</style>
