<script setup lang="ts">
import { computed, ref, watch } from 'vue';
import { initialized, runtime, snapshot } from '../runtime';

const props = defineProps<{active: boolean}>();
const fields = [
  {name: 'system.agent_ping_hz', label: 'Agent 链路探测', options: [10, 20, 50, 100]},
  {name: 'system.motion_report_hz', label: '电机运动反馈', options: [20, 25, 50]},
  {name: 'system.status_report_hz', label: '电机与控制状态', options: [10, 20, 25, 50]},
  {name: 'system.imu_report_hz', label: 'IMU 姿态', options: [10, 20, 25, 50, 100]},
  {name: 'system.temperature_report_hz', label: 'IMU 温度', options: [1, 2]},
  {name: 'system.motor_command_hz', label: '电机控制频率', options: [50, 100, 125, 200, 250, 500, 1000]},
];
const frequencyGroups = [
  {title: '控制频率', fields: fields.slice(5)},
  {title: '上报频率', fields: fields.slice(1, 5)},
];
const values = ref<Record<string, number | boolean>>({});
const draft = ref<Record<string, number>>({});
const loaded = ref(false);
const busy = ref(false);
const message = ref('尚未读取控制板参数');
const error = ref(false);
const changed = computed(() => fields.filter(f => draft.value[f.name] !== values.value[f.name]));
let epoch = 0;

async function readValues(generation: number) {
  const next: Record<string, number | boolean> = {};
  // 固件低内存参数服务每次只处理一个参数。
  for (const name of [...fields.map(f => f.name), 'system.can_nominal_kbps', 'system.can_data_kbps', 'system.unsaved']) {
    next[name] = await runtime.readFirmwareParameter(name);
    if (generation !== epoch) throw new Error('连接已更换，请重新读取');
  }
  for (const field of fields) {
    if (!field.options.includes(Number(next[field.name]))) throw new Error(`固件返回了不支持的 ${field.label} 值`);
  }
  values.value = next;
  draft.value = Object.fromEntries(fields.map(f => [f.name, Number(next[f.name])]));
  loaded.value = true;
}
async function operate(action: 'read' | 'apply' | 'save') {
  if (busy.value) return;
  const generation = epoch;
  busy.value = true; error.value = false;
  message.value = action === 'read' ? '正在读取控制板参数…' : action === 'apply' ? '正在逐项下发…' : '正在保存 W25…';
  let completed = 0;
  const changes = changed.value.map(field => ({...field, value: draft.value[field.name]!}));
  try {
    if (action === 'apply') {
      for (const field of changes) {
        const value = field.value;
        if (!field.options.includes(value)) throw new Error(`${field.label} 数值无效`);
        await runtime.writeFirmwareParameter(field.name, value);
        if (generation !== epoch) throw new Error('连接已更换，请重新读取');
        completed++;
      }
    }
    if (action === 'save') await runtime.saveFirmwareParameters();
    await readValues(generation);
    if (action === 'apply' && changes.some(field => values.value[field.name] !== field.value))
      throw new Error('下发后的控制板回读值与修改不一致');
    if (action === 'save' && values.value['system.unsaved']) throw new Error('保存后仍有未落盘修改，请重新核对');
    message.value = action === 'read' ? '已从控制板读取' : action === 'apply' ? `已下发并回读 ${completed} 项，尚需保存 W25` : 'W25 保存完成并已回读核对';
  } catch (reason) {
    if (generation !== epoch) return;
    error.value = true;
    message.value = `${completed ? `已下发 ${completed} 项；` : ''}${reason instanceof Error ? reason.message : String(reason)}。请重新读取核对。`;
    loaded.value = false;
  } finally { busy.value = false; }
}
watch(() => snapshot.value.connected, connected => {
  if (connected) return;
  epoch++; loaded.value = false; values.value = {}; draft.value = {};
  message.value = '连接已断开，请重新读取控制板参数';
});
watch(() => snapshot.value.firmwareConnectionKey, (key, previous) => {
  // 首次获知会话号并非换板；断链代数及已知会话变化仍作废草稿与等待。
  if (previous?.endsWith(':0') && previous.split(':')[0] === key.split(':')[0]) return;
  epoch++; loaded.value = false; values.value = {}; draft.value = {};
  message.value = '控制板会话已更换，请重新读取参数';
});
watch(() => props.active && initialized.value && snapshot.value.connected, active => {
  if (active && !loaded.value && !busy.value) void operate('read');
}, {immediate: true});
</script>

<template>
  <section v-show="active" class="firmware-parameters" aria-label="固件系统参数" :aria-busy="busy">
    <p class="firmware-config-notice">控制与上报频率在线生效；仅电机失能、无活动轨迹时允许修改。保存 W25 同时保存已下发的电机参数。</p>
    <div class="firmware-parameter-actions">
      <button class="secondary compact" :disabled="!initialized || !snapshot.connected || busy" @click="operate('read')">读取固件参数</button>
      <button class="primary compact" :disabled="!loaded || !snapshot.connected || busy || !changed.length" @click="operate('apply')">下发固件参数</button>
      <button class="secondary compact" :disabled="!loaded || !snapshot.connected || busy || !!changed.length || !values['system.unsaved']" @click="operate('save')">保存固件参数 W25</button>
      <span v-if="loaded" class="micro">{{ values['system.unsaved'] ? '有未保存修改' : '与已保存配置一致' }}</span>
    </div>
    <p class="micro" :class="{'panel-error': error}" role="status">{{ message }}</p>
    <h3>通信设置</h3>
    <dl>
      <div><dt>主机通信</dt><dd>micro-ROS / USB CDC</dd></div>
      <div><dt>电机总线</dt><dd>CAN FD · BRS</dd></div>
      <div><dt>CAN 仲裁段速率（只读）</dt><dd>{{ loaded ? `${values['system.can_nominal_kbps']} kbps` : '未读取' }}</dd></div>
      <div><dt>CAN 数据段速率（只读）</dt><dd>{{ loaded ? `${values['system.can_data_kbps']} kbps` : '未读取' }}</dd></div>
      <div><dt><label :for="fields[0]!.name">{{ fields[0]!.label }}</label></dt><dd>
        <select :id="fields[0]!.name" v-model="draft[fields[0]!.name]" :disabled="!loaded || !snapshot.connected || busy">
          <option v-if="!loaded" :value="undefined">未读取</option>
          <option v-for="hz in fields[0]!.options" :key="hz" :value="hz">{{ hz }} Hz</option>
        </select>
      </dd></div>
    </dl>
    <template v-for="group in frequencyGroups" :key="group.title">
    <h3>{{ group.title }}</h3>
    <dl>
      <div v-for="field in group.fields" :key="field.name">
        <dt><label :for="field.name">{{ field.label }}</label></dt><dd>
          <select :id="field.name" v-model="draft[field.name]" :disabled="!loaded || !snapshot.connected || busy">
            <option v-if="!loaded" :value="undefined">未读取</option>
            <option v-for="hz in field.options" :key="hz" :value="hz">{{ hz }} Hz</option>
          </select>
          <small v-if="loaded">{{ 1000 / draft[field.name]! }} ms</small>
        </dd>
      </div>
    </dl>
    </template>
  </section>
</template>
