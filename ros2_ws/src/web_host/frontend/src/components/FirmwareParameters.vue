<script setup lang="ts">
import { ref, watch } from 'vue';
import { initialized, runtime, snapshot } from '../runtime';

const props = defineProps<{active: boolean}>();
const fields = [
  ['system.agent_ping_hz', 'Agent 链路探测'],
  ['system.motor_command_hz', '电机控制频率'],
  ['system.motion_report_hz', '电机运动反馈'],
  ['system.status_report_hz', '电机与控制状态'],
  ['system.imu_report_hz', 'IMU 姿态'],
  ['system.temperature_report_hz', 'IMU 温度'],
] as const;
const values = ref<Record<string, number | boolean>>({});
const busy = ref(false);
const message = ref('等待控制板状态');
let generation = 0;
async function read() {
  if (busy.value) return;
  const current = generation;
  busy.value = true;
  try {
    const next: Record<string, number | boolean> = {};
    for (const [name] of fields) next[name] = await runtime.readFirmwareParameter(name);
    if (current !== generation) return;
    values.value = next;
    message.value = '已读取控制板固定频率';
  } catch (error) {
    if (current === generation) message.value = error instanceof Error ? error.message : String(error);
  } finally { busy.value = false; }
}
watch(() => snapshot.value.firmwareConnectionKey, () => {
  generation++; values.value = {};
});
watch(() => props.active && initialized.value && snapshot.value.connected, active => {
  if (active) void read();
  else { generation++; values.value = {}; message.value = '等待控制板状态'; }
}, {immediate: true});
</script>

<template>
  <section v-show="active" class="firmware-parameters" aria-label="固件系统参数" :aria-busy="busy">
    <p class="firmware-config-notice">控制与上报频率固定于固件；运行期仅支持读取，不支持设置或保存。</p>
    <button class="secondary compact" :disabled="!initialized || !snapshot.connected || busy" @click="read">读取固件参数</button>
    <p class="micro" role="status">{{ message }}</p>
    <dl>
      <div><dt>主机通信</dt><dd>micro-ROS / USB CDC</dd></div>
      <div><dt>电机总线</dt><dd>CAN FD · BRS · 1000 / 5000 kbps</dd></div>
      <div v-for="[name, label] in fields" :key="name"><dt>{{ label }}</dt>
        <dd>{{ values[name] === undefined ? '未读取' : `${values[name]} Hz` }}</dd></div>
    </dl>
  </section>
</template>
