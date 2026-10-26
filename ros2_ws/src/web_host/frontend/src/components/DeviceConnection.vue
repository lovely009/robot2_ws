<script setup lang="ts">
import { computed, onBeforeUnmount, onMounted, ref } from 'vue';
import MotorSpecifications from './MotorSpecifications.vue';
const emit = defineEmits<{ direct: [value: boolean] }>();
const transport = ref(sessionStorage.getItem('device-can-transport') || 'firmware');
const endpoint = ref('');
const options = ref<Record<string, string[]>>({socketcan: [], slcan: []});
const session = ref(sessionStorage.getItem('device-can-session') || '');
const connectedTransport = ref('');
const connectedEndpoint = ref('');
const busy = ref(false);
const error = ref('');
interface Field { key: string; label: string; object: string; unit: string; value?: string | number; error?: string }
interface Motor { id: number; key: string; protocol: string; online: boolean; fields: Field[]; read_at: number; age_ms?: number }
const motors = ref<Motor[]>([]);
const selected = ref('');
const current = computed(() => motors.value.find(m => m.key === selected.value));
const library = ref<Record<string, Record<string, unknown>>>({});
const selectedModel = computed(() => {
  const identity = current.value?.fields.find(f => f.key === 'identity')?.value;
  if (typeof identity !== 'string' || !identity) return '';
  return Object.entries(library.value).find(([, spec]) =>
    spec.protocol === current.value?.protocol && spec.device_identity === identity)?.[0] || '';
});
let timer: ReturnType<typeof setInterval>;
let generation = 0;
let polling = false;
let mounted = true;
async function api(action: string, body: Record<string, unknown>) {
  const response = await fetch(`/api/device-can/${action}`, {
    method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body),
  });
  const data = await response.json();
  if (!response.ok) throw new Error(data.error || `HTTP ${response.status}`);
  return data;
}
async function refreshOptions() {
  try {
    const response = await fetch('/api/device-can/options');
    if (!response.ok) throw new Error('无法枚举 CAN 接口');
    options.value = await response.json();
    if (!options.value[transport.value]?.includes(endpoint.value)) endpoint.value = options.value[transport.value]?.[0] || '';
  } catch (reason) { error.value = String(reason); }
}
function changeSource() {
  generation++;
  motors.value = []; selected.value = '';
  endpoint.value = options.value[transport.value]?.[0] || '';
  error.value = '';
  sessionStorage.setItem('device-can-transport', transport.value);
  emit('direct', transport.value !== 'firmware');
}
function merge(items: Motor[], replace = false) {
  const old = new Map(motors.value.map(m => [m.key, m]));
  motors.value = items.map(m => ({...m, fields: !replace && m.protocol === 'cerb'
    ? [...(old.get(m.key)?.fields || []).filter(f => !m.fields.some(n => n.key === f.key)), ...m.fields] : m.fields}));
  if (!motors.value.some(m => m.key === selected.value)) selected.value = motors.value[0]?.key || '';
}
async function discover() {
  if (!session.value || busy.value || polling) return;
  polling = true;
  const epoch = generation;
  try {
    const data = await api('discover', {session: session.value});
    if (epoch === generation && mounted) merge(data.motors);
  } catch (reason) { if (epoch === generation && mounted) error.value = String(reason); }
  finally { polling = false; }
}
async function connect() {
  if (busy.value) return;
  busy.value = true; error.value = '';
  try {
    const data = await api('connect', {transport: transport.value, endpoint: endpoint.value});
    session.value = data.session;
    connectedTransport.value = data.transport; connectedEndpoint.value = data.endpoint;
    sessionStorage.setItem('device-can-session', data.session);
    motors.value = []; selected.value = '';
  } catch (reason) { error.value = String(reason); }
  finally { busy.value = false; }
  await discover();
}
async function disconnect() {
  if (busy.value) return;
  busy.value = true; error.value = '';
  try {
    await api('disconnect', {session: session.value});
    generation++;
    session.value = ''; sessionStorage.removeItem('device-can-session');
    connectedTransport.value = ''; connectedEndpoint.value = '';
    motors.value = []; selected.value = '';
  } catch (reason) { error.value = String(reason); }
  finally { busy.value = false; }
}
async function read() {
  if (!session.value || busy.value) return;
  busy.value = true; error.value = '';
  const epoch = ++generation;
  try {
    const key = selected.value;
    const data = await api('read', {session: session.value, device: key});
    if (epoch === generation && mounted) {
      const updates = new Map<string, Motor>(data.motors.map((m: Motor) => [m.key, m]));
      merge(motors.value.map(m => updates.get(m.key) || m));
    }
  } catch (reason) { if (epoch === generation && mounted) error.value = String(reason); }
  finally { busy.value = false; }
}
async function select(motor: Motor) {
  selected.value = motor.key;
  if (motor.protocol === 'cerb' && motor.online && !motor.fields.some(f => f.key === 'identity')) await read();
}
onMounted(async () => {
  emit('direct', transport.value !== 'firmware');
  await refreshOptions();
  try {
    const response = await fetch('/api/motor-specs');
    if (response.ok) library.value = (await response.json()).motor_types || {};
  } catch { /* 实际回读不依赖型号库。 */ }
  if (session.value) {
    try {
      const data = await api('heartbeat', {session: session.value});
      transport.value = connectedTransport.value = data.transport;
      connectedEndpoint.value = data.endpoint;
      changeSource(); endpoint.value = data.endpoint;
      await discover();
    } catch { session.value = ''; sessionStorage.removeItem('device-can-session'); }
  }
  timer = setInterval(discover, 500);
});
onBeforeUnmount(() => { mounted = false; generation++; clearInterval(timer); });
</script>
<template>
  <section class="device-connection surface">
    <div class="device-connection-controls">
      <label>通信来源
        <select v-model="transport" aria-label="设备通信来源" :disabled="busy || !!session" @change="changeSource">
          <option value="firmware">经控制板固件</option>
          <option value="socketcan">直接 SocketCAN</option>
          <option value="slcan">直接 SLCAN</option>
        </select>
      </label>
      <template v-if="transport !== 'firmware'">
        <label>{{ transport === 'socketcan' ? 'CAN 接口' : '串口' }}
          <select v-model="endpoint" aria-label="直连设备端点" :disabled="busy || !!session">
            <option v-for="item in options[transport] || []" :key="item" :value="item">{{ item }}</option>
            <option v-if="!options[transport]?.length" value="">未发现可用设备</option>
          </select>
        </label>
        <button :disabled="busy || !!session" @click="refreshOptions">刷新接口</button>
        <button v-if="!session" :disabled="busy || !endpoint" @click="connect">连接 CAN</button>
        <button v-else :disabled="busy" @click="disconnect">断开 CAN</button>
        <button :disabled="busy || !session || !current?.online" @click="read">{{ busy ? '处理中…' : '读取电机参数' }}</button>
      </template>
    </div>
    <p v-if="transport !== 'firmware'" class="micro">CAN FD 1M/5M · CERB SYNC 探测 / VAXOR 自动上报 · 不经过控制板固件。{{ session ? `${connectedTransport} / ${connectedEndpoint} 已连接` : '未连接' }}</p>
    <p v-if="transport !== 'firmware'" class="micro">自动发现总线电机，无需填写节点 ID。CERB 可读取设备参数；VAXOR 显示主动上报值。SocketCAN 使用现有位率，SLCAN 使用 CANable2 FD 方言。</p>
    <p v-if="error" role="alert" class="panel-error">{{ error }}</p>
  </section>
  <div v-show="transport !== 'firmware'" class="direct-device-grid">
    <section class="surface direct-motors">
      <div class="panel-title">CAN 电机</div>
      <button v-for="motor in motors" :key="motor.key" class="direct-motor" :aria-pressed="selected === motor.key" @click="select(motor)">
        <strong>{{ motor.protocol.toUpperCase() }} · m{{ motor.id }}</strong>
        <span>{{ motor.online ? '在线' : '反馈已超时' }} · {{ motor.age_ms ?? 0 }} ms</span>
      </button>
      <p v-if="!motors.length" class="micro">连接 CAN 后自动发现电机。</p>
    </section>
    <section class="surface direct-parameters">
      <div class="panel-title">{{ current ? `电机 ${current.id} · 设备详细参数` : '请选择电机' }}</div>
      <template v-if="current">
        <p class="micro">{{ current.protocol.toUpperCase() }} · {{ current.online ? '在线' : '反馈已超时' }} · 反馈年龄 {{ current.age_ms ?? 0 }} ms · 参数为最近回读值</p>
        <table class="device-detail-table">
          <thead><tr><th>参数</th><th>设备对象</th><th>实际值</th><th>单位</th></tr></thead>
          <tbody><tr v-for="field in current.fields" :key="field.key"><td>{{ field.label }}</td><td>{{ field.object }}</td><td>{{ field.error || field.value }}</td><td>{{ field.unit }}</td></tr></tbody>
        </table>
        <MotorSpecifications :model="selectedModel" :motor-id="current.id" />
      </template>
    </section>
  </div>
</template>
