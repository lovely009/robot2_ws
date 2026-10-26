<script setup lang="ts">
import { computed, ref } from "vue";
import {
  Activity,
  AlignLeft,
  Terminal,
  ChevronDown,
  Pause,
  Play,
  Download,
} from "@lucide/vue";
import { demo, selected, measured, log, connected } from "../demo";
const paused = ref(false),
  paused_time = ref(0);
const end = computed(() => (paused.value ? paused_time.value : demo.clock));
const trace = computed(() => {
  if (!selected.value?.motor) return "";
  const j = selected.value;
  return Array.from(
    { length: 180 },
    (_, i) =>
      `${(i / 179) * 900},${65 - (measured(j, end.value - 10 + (i / 179) * 10) - Math.max(j.lower, Math.min(j.upper, 0))) * 2800}`,
  ).join(" ");
});
const events = computed(() =>
  demo.events.filter(
    (e) => demo.observer !== "commands" || /发送|受理|执行|连接/.test(e.text),
  ),
);
const tabs = [
  { id: "curve", name: "曲线", icon: Activity },
  { id: "events", name: "事件日志", icon: AlignLeft },
  { id: "commands", name: "命令记录", icon: Terminal },
];
function toggle() {
  paused_time.value = demo.clock;
  paused.value = !paused.value;
}
function download() {
  const data = demo.events
    .map((e) => `${e.at}\t${e.level}\t${e.text}`)
    .join("\n");
  const url = URL.createObjectURL(
    new Blob([data], { type: "text/plain;charset=utf-8" }),
  );
  const a = document.createElement("a");
  a.href = url;
  a.download = "prototype-events.txt";
  a.click();
  URL.revokeObjectURL(url);
  log("已导出原型事件日志");
}
</script>
<template>
  <section class="observer surface">
    <div class="observer-head">
      <div class="observer-tabs">
        <button
          v-for="t in tabs"
          :key="t.id"
          :class="{ active: demo.observer === t.id }"
          @click="demo.observer = t.id"
        >
          <component :is="t.icon" :size="13" />{{ t.name
          }}<span v-if="t.id === 'events'" class="count">{{
            demo.events.length
          }}</span>
        </button>
      </div>
      <div class="button-group">
        <span class="micro">{{ connected ? "DEMO STREAM" : "STALE" }}</span
        ><button
          class="icon-button"
          :aria-label="paused ? '恢复曲线' : '暂停曲线'"
          @click="toggle"
        >
          <Play v-if="paused" :size="13" /><Pause v-else :size="13" /></button
        ><button
          class="icon-button"
          aria-label="导出原型日志"
          @click="download"
        >
          <Download :size="13" />
        </button>
      </div>
    </div>
    <div v-if="demo.observer === 'curve'" class="chart-content">
      <div class="chart-caption">
        <span class="series-label"
          ><i />{{ selected?.name || "未选择" }} <small>位置 · rad</small></span
        ><span class="micro"
          >{{ paused ? "已暂停观察" : "最近 10 s" }}<ChevronDown :size="12"
        /></span>
      </div>
      <div v-if="!selected?.motor" class="empty">该关节没有设备数据源</div>
      <div v-else class="chart-wrap">
        <div class="chart-y mono">
          <span>+0.02</span><span>0.00</span><span>−0.02</span>
        </div>
        <svg
          viewBox="0 0 900 130"
          preserveAspectRatio="none"
          aria-label="模拟位置曲线"
        >
          <defs>
            <linearGradient id="area" x1="0" x2="0" y1="0" y2="1">
              <stop offset="0%" stop-color="#69d8ae" stop-opacity=".16" />
              <stop offset="100%" stop-color="#69d8ae" stop-opacity="0" />
            </linearGradient>
          </defs>
          <path
            d="M0 15H900M0 65H900M0 115H900M150 0V130M300 0V130M450 0V130M600 0V130M750 0V130"
            stroke="#293439"
            stroke-width="1"
          />
          <polygon
            v-if="connected"
            :points="`0,130 ${trace} 900,130`"
            fill="url(#area)"
          />
          <polyline
            v-if="connected"
            :points="trace"
            fill="none"
            stroke="#75dcb8"
            stroke-width="1.8"
            vector-effect="non-scaling-stroke"
          />
          <text v-else x="420" y="65" fill="#b7a06c" font-size="13">
            数据陈旧
          </text>
        </svg>
        <div class="chart-x mono">
          <span>−10 s</span><span>−8 s</span><span>−6 s</span><span>−4 s</span
          ><span>−2 s</span><span>现在</span>
        </div>
      </div>
    </div>
    <div v-else class="event-content">
      <div v-for="(e, i) in events" :key="i" class="event-row">
        <span class="mono">{{ e.at }}</span
        ><span class="event-level" :class="e.level">{{
          e.level.toUpperCase()
        }}</span
        ><span>{{ e.text }}</span>
      </div>
      <div v-if="!demo.events.length" class="empty">暂无事件</div>
    </div>
  </section>
</template>
