<script setup lang="ts">
import { computed, onBeforeUnmount, ref, watch } from "vue";
import { Activity, Download, Trash2, Video, Square } from "@lucide/vue";
import { bffClient } from "../api/bff_client";
import { useDiagnosticsStore } from "../stores/diagnostics";
import { useLeaseStore } from "../stores/lease";

const props = defineProps<{ active: boolean }>();
const diagStore = useDiagnosticsStore();
const leaseStore = useLeaseStore();
const bagName = ref("");
let pollTimer: ReturnType<typeof setInterval> | null = null;

const topicRows = computed(() =>
  Object.values(diagStore.snapshot?.topics || {}).sort((a, b) => a.topic.localeCompare(b.topic)),
);

function stopPolling() {
  if (pollTimer) {
    clearInterval(pollTimer);
    pollTimer = null;
  }
}

function startPolling() {
  stopPolling();
  if (!props.active || !leaseStore.isBff) return;
  void diagStore.refreshDiagnostics();
  void diagStore.refreshRecordings();
  pollTimer = setInterval(() => {
    void diagStore.refreshDiagnostics();
    if (diagStore.recordingStatus.active) {
      void diagStore.refreshRecordings();
    }
  }, 1500);
}

watch(
  () => [props.active, leaseStore.isBff] as const,
  ([isActive, isBff]) => {
    if (isActive && isBff) startPolling();
    else stopPolling();
  },
  { immediate: true },
);

onBeforeUnmount(stopPolling);

async function handleStartRecording() {
  try {
    await diagStore.startRecording(bagName.value.trim());
    bagName.value = "";
  } catch {
    /* error rendered from store */
  }
}

function formatBytes(bytes: number): string {
  if (bytes < 1024) return `${bytes} B`;
  if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KB`;
  return `${(bytes / (1024 * 1024)).toFixed(2)} MB`;
}
</script>

<template>
  <section v-show="active" class="diag-recorder" aria-label="链路诊断与工程录播">
    <p v-if="!leaseStore.isBff" class="tiny muted">
      当前运行在遗留 rosbridge 模式；切换为 <code>bff_server</code> 后可启用链路诊断与 rosbag2 录播。
    </p>
    <template v-else>
      <div class="diag-summary">
        <span class="diag-stat"><Activity :size="14" />ROS：<b>{{ diagStore.snapshot?.ros_connected ? '就绪' : '未就绪' }}</b></span>
        <span class="diag-stat">事件循环抖动：<b>{{ diagStore.snapshot?.event_loop_lag_ms ?? '--' }} ms</b></span>
        <span class="diag-stat">WS 在线：<b>{{ diagStore.snapshot?.ws_clients ?? 0 }}</b></span>
        <span class="diag-stat">扇出频率：<b>{{ diagStore.snapshot?.ws_broadcast_hz ?? 0 }} Hz</b></span>
        <span class="diag-stat">WS RTT：<b>{{ leaseStore.wsRttMs !== null ? `${leaseStore.wsRttMs} ms` : '--' }}</b></span>
      </div>

      <div class="diag-grid">
        <div class="diag-card">
          <h4>话题新鲜度</h4>
          <div class="table-wrap">
            <table class="topic-health-table">
              <thead>
                <tr><th>话题</th><th class="num">频率</th><th class="num">年龄</th><th class="num">累计</th></tr>
              </thead>
              <tbody>
                <tr v-for="row in topicRows" :key="row.topic">
                  <td><code>{{ row.topic }}</code></td>
                  <td class="num">{{ row.rx_hz.toFixed(1) }} Hz</td>
                  <td class="num">{{ row.age_ms !== null && row.age_ms !== undefined ? `${row.age_ms.toFixed(0)} ms` : '--' }}</td>
                  <td class="num">{{ row.rx_count }}</td>
                </tr>
              </tbody>
            </table>
          </div>
        </div>

        <div class="diag-card">
          <h4>工程录播 (rosbag2 / {{ (diagStore.recordingStatus.storage_format || 'mcap').toUpperCase() }})</h4>
          <div class="ctrl-row wrap">
            <input
              v-model="bagName"
              type="text"
              placeholder="录播名称（留空自动命名）"
              aria-label="录播名称"
              :disabled="diagStore.recordingStatus.active || diagStore.busy"
            />
            <button
              v-if="!diagStore.recordingStatus.active"
              type="button"
              class="compact"
              :disabled="diagStore.busy"
              @click="handleStartRecording"
            >
              <Video :size="14" />开始录制
            </button>
            <button
              v-else
              type="button"
              class="danger compact"
              :disabled="diagStore.busy"
              @click="diagStore.stopRecording()"
            >
              <Square :size="14" />停止录制 ({{ diagStore.recordingStatus.message_count ?? 0 }} 条 / {{ (diagStore.recordingStatus.duration_s ?? 0).toFixed(1) }}s)
            </button>
          </div>
          <p v-if="diagStore.error" class="lease-err tiny" role="alert">{{ diagStore.error }}</p>
          <ul class="recording-list">
            <li v-for="rec in diagStore.recordings" :key="rec.name" class="recording-item">
              <span class="rec-name">{{ rec.name }}</span>
              <span class="tiny-tag">{{ rec.storage_format.toUpperCase() }}</span>
              <span class="tiny muted">{{ rec.message_count }} 条 · {{ formatBytes(rec.size_bytes) }}</span>
              <span class="spacer" />
              <a :href="bffClient.recordingDownloadUrl(rec.name)" class="tiny-btn secondary" download>
                <Download :size="13" />下载
              </a>
              <button
                type="button"
                class="tiny-btn danger"
                aria-label="删除录制"
                @click="diagStore.deleteRecording(rec.name)"
              >
                <Trash2 :size="13" />
              </button>
            </li>
            <li v-if="!diagStore.recordings.length" class="tiny muted">暂无已保存录制</li>
          </ul>
        </div>
      </div>
    </template>
  </section>
</template>
