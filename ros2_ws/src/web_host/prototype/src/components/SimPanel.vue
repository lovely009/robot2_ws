<script setup lang="ts">
import { computed } from "vue";
import {
  Box,
  Cpu,
  Network,
  ArrowRight,
  Play,
  Square,
  CheckCircle2,
  Circle,
} from "@lucide/vue";
import { demo, model, log, stop } from "../demo";
const supported = computed(() =>
  demo.backend === "MuJoCo" ? model.value?.mujoco : model.value?.gazebo,
);
function start() {
  if (!supported.value) return;
  demo.session = "running";
  demo.target = demo.controller === "STM32 HIL" ? "HIL" : "仿真";
  log(
    `模拟启动 ${demo.backend} · ${demo.controller} · ${demo.transport}`,
    "ok",
  );
}
function end() {
  demo.session = "stopped";
  stop();
  log("模拟会话已停止", "warn");
}
</script>
<template>
  <section class="sim-panel surface">
    <div class="panel-title">
      <span>会话配置与拓扑</span
      ><span
        class="status-chip"
        :class="demo.session === 'running' ? 'success' : ''"
        >{{ demo.session === "running" ? "模拟运行中" : "已停止" }}</span
      >
    </div>
    <div class="sim-config">
      <label class="field"
        ><span>物理后端</span
        ><select
          v-model="demo.backend"
          aria-label="物理后端"
          :disabled="demo.session === 'running'"
        >
          <option :disabled="!model?.mujoco">MuJoCo</option>
          <option :disabled="!model?.gazebo">Gazebo</option>
        </select></label
      ><label class="field"
        ><span>控制端</span
        ><select
          v-model="demo.controller"
          aria-label="控制端"
          :disabled="demo.session === 'running'"
        >
          <option>主机驱动</option>
          <option>外部控制器</option>
          <option>STM32 HIL</option>
        </select></label
      ><label class="field"
        ><span>传输</span
        ><select
          v-model="demo.transport"
          aria-label="传输"
          :disabled="demo.session === 'running'"
        >
          <option :disabled="demo.controller === 'STM32 HIL'">vcan</option>
          <option>SocketCAN</option>
          <option>SLCAN</option>
        </select></label
      >
    </div>
    <div class="topology-flow">
      <div class="topology-node">
        <span class="node-icon"><Cpu :size="22" /></span
        ><b>{{ demo.controller }}</b
        ><small>控制端</small
        ><span class="node-state"
          ><i
            class="dot"
            :class="demo.session === 'running' ? 'green' : 'muted'"
          />{{ demo.session === "running" ? "模拟就绪" : "已停止" }}</span
        >
      </div>
      <div class="topology-edge">
        <span>{{ demo.transport }}</span
        ><ArrowRight :size="20" />
      </div>
      <div class="topology-node">
        <span class="node-icon"><Network :size="22" /></span><b>电机后端</b
        ><small>设备协议模拟</small
        ><span class="node-state"
          ><i
            class="dot"
            :class="demo.session === 'running' ? 'green' : 'muted'"
          />{{ demo.session === "running" ? "模拟就绪" : "已停止" }}</span
        >
      </div>
      <div class="topology-edge">
        <span>关节反馈</span><ArrowRight :size="20" />
      </div>
      <div class="topology-node">
        <span class="node-icon"><Box :size="22" /></span
        ><b>{{ demo.backend }}</b
        ><small>{{ demo.model }}</small
        ><span class="node-state"
          ><i
            class="dot"
            :class="demo.session === 'running' ? 'green' : 'muted'"
          />{{ demo.session === "running" ? "模拟就绪" : "已停止" }}</span
        >
      </div>
    </div>
    <div class="session-checks">
      <span
        ><CheckCircle2 :size="14" />模型资产
        {{ model?.joints.length }} 关节</span
      ><span
        ><component :is="supported ? CheckCircle2 : Circle" :size="14" />{{
          supported ? "后端资产已找到" : "该模型缺少后端资产"
        }}</span
      ><span><Circle :size="14" />真实进程未接入</span>
    </div>
    <div
      v-if="demo.controller === 'STM32 HIL' && demo.transport === 'vcan'"
      class="notice warning"
    >
      STM32 HIL 不支持 vcan，请停止会话后更换传输。
    </div>
    <div class="sim-actions">
      <p>运行状态、进程存活和数据就绪分别呈现。</p>
      <button v-if="demo.session === 'running'" class="secondary" @click="end">
        <Square :size="12" />模拟停止</button
      ><button
        v-else
        class="primary"
        :disabled="
          !supported ||
          (demo.controller === 'STM32 HIL' && demo.transport === 'vcan')
        "
        @click="start"
      >
        <Play :size="13" />模拟启动会话
      </button>
    </div>
  </section>
</template>
