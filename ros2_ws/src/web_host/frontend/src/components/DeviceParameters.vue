<script setup lang="ts">
import { computed, onMounted, reactive, ref } from 'vue';
import {
  clearMotorFault,
  motorDeviceParameterError,
  motorZeroError,
  refreshMotorParams,
  saveMotorDeviceParameters,
  setMotorDirection,
  setMotorZero,
} from '../core/device';
import { fmt, fmtParam } from '../core/format';
import { describeMotorState } from '../core/motor_state';
import { DIRTY, markDirty, paramReadState, paramRowIds, store } from '../core/store';
import { motionRevision, paramRevision, statusRevision } from '../runtime';

const props = defineProps<{ active: boolean }>();

const motorSpecs = ref<Record<string, any> | null>(null);
const directionDrafts = reactive<Record<number, number>>({});

onMounted(() => {
  fetch('/api/motor-specs')
    .then(r => r.json())
    .then(d => {
      motorSpecs.value = d.motor_types || {};
      markDirty(DIRTY.PARAMS);
    })
    .catch(() => {
      motorSpecs.value = {};
    });
});

const connected = computed(() => {
  void statusRevision.value;
  return store.conn.connected;
});

const allIds = computed(() => {
  void paramRevision.value;
  void statusRevision.value;
  return [...new Set([...store.motors.keys(), ...store.paramCache.keys()])].sort((a, b) => a - b);
});

const selectedMotor = computed(() => {
  void paramRevision.value;
  void statusRevision.value;
  return allIds.value.includes(store.selectedId) ? store.selectedId : allIds.value[0];
});

const paramIds = computed(() => {
  void paramRevision.value;
  return paramRowIds(true);
});

const selectionHintText = computed(() => {
  const selected = selectedMotor.value;
  if (selected === undefined) return '等待电机反馈后读取参数。';
  return `电机 ${selected} 暂无参数，请读取电机参数。`;
});

const showSelectionHint = computed(() => {
  const selected = selectedMotor.value;
  return !(selected !== undefined && store.paramCache.has(selected));
});

const sumDevText = computed(() => {
  void paramRevision.value;
  void statusRevision.value;
  let conflict = 0;
  for (const r of store.motors.values()) {
    const st = r.status || {}, t = r.topo || {};
    if (t.conflict || st.lifecycle === 7) conflict++;
  }
  return store.paramCache.size
    ? `${store.paramCache.size} 台已报告${conflict ? ` · 未初始化 ${conflict}` : ''}`
    : '未读取';
});

const paramResult = computed(() => {
  void paramRevision.value;
  return store.results.get('paramResult') || null;
});

const saveError = computed(() => {
  void paramRevision.value;
  void statusRevision.value;
  return motorDeviceParameterError(store.selectedId);
});

function rawValue(row: any, field: string, value: number): string {
  const isCerb = row.topo?.protocol === 1;
  if (!isCerb || !Number.isFinite(value)) return '';
  if (field === 'pos_min' || field === 'pos_max') {
    return `${Math.round(value * 65536 / (2 * Math.PI))} cnt`;
  }
  if (field === 'torque_limit') return `${Math.round(value * 1000)} mA`;
  const model = row.topo?.model_name || '';
  const gear = motorSpecs.value && motorSpecs.value[model]
    ? Number(motorSpecs.value[model].gear_ratio) : NaN;
  if (field === 'vel_limit' && Number.isFinite(gear) && gear > 0) {
    return `${Math.round(value * gear * 60 / (2 * Math.PI))} RPM`;
  }
  return '';
}

function fieldHint(v: any, row: any, f: string): string {
  const dev = rawValue(row, f, v.pick[f]);
  const mismatch = v.rec[f] === v.dev[f] ? '' : `；记录 ${fmtParam(v.rec[f])} / 驱动板 ${fmtParam(v.dev[f])}`;
  return (dev ? `设备端 ${dev}` : '') + mismatch;
}

const motorItems = computed(() => {
  void paramRevision.value;
  void statusRevision.value;
  void motorSpecs.value;
  return paramIds.value.map((id) => {
    const v = store.paramCache.get(id);
    const pick = v.pick;
    const row = store.motors.get(id) || {};
    const model = row.topo?.model_name || '';
    const view = describeMotorState(store, id);
    const availability = paramReadState(id);
    const srcText = availability.verified ? '驱动板 · 已核对'
      : availability.device === 8 ? '驱动板 · 完整回读'
        : availability.device ? '部分设备回读 · 未核对'
          : availability.record ? '固件记录 · 未核对' : '无有效参数';
    const pillClass = view.fault || !view.online ? 'off' : view.active ? 'on' : 'mid';
    const pillTitle = `${view.connection} · ${view.execution} · ${view.protocol}`;
    const unprotected = pick.limit_protect === 0;
    const protectCell = Number.isFinite(pick.limit_protect)
      ? (pick.limit_protect === 1 ? '开启' : '关闭') : '未读取';
    const dirValue = directionDrafts[id] ?? pick.direction;
    const dirSelectValue = [0, 1].includes(dirValue) ? String(dirValue) : '';
    const devError = motorDeviceParameterError(id);
    const zeroError = motorZeroError(id);
    return {
      id,
      v,
      pick,
      row,
      model,
      view,
      availability,
      srcText,
      pillClass,
      pillTitle,
      unprotected,
      protectCell,
      dirSelectValue,
      devError,
      zeroError,
      visible: selectedMotor.value === id,
    };
  });
});

function currentPositionText(id: number): string {
  void motionRevision.value;
  const row = store.motors.get(id) || {};
  return fmt(row.motion?.position, 4);
}

function onDirectionChange(id: number, event: Event) {
  const value = (event.target as HTMLSelectElement).value;
  if (value !== '') directionDrafts[id] = Number(value);
}

function onSetDirection(id: number) {
  const item = motorItems.value.find(entry => entry.id === id);
  const val = item?.dirSelectValue;
  if (val !== undefined && val !== '') {
    void setMotorDirection(id, Number(val));
  }
}
</script>

<template>
  <section v-show="active" data-workbench-panel="deviceParams" class="motor-parameters">
    <p class="firmware-config-notice">查询设备参数与初始化核验状态；方向设置和驱动板保存须先失能。</p>
    <div class="firmware-parameter-actions">
      <button id="btnParamLoad" class="secondary compact" :disabled="!connected" @click="refreshMotorParams">读取电机参数</button>
      <button
        id="btnParamSaveDevice"
        class="secondary compact"
        :disabled="!connected || Boolean(saveError)"
        :title="saveError || `保存电机 ${store.selectedId} 驱动板的位置、速度与电流上限（不保存方向）`"
        @click="saveMotorDeviceParameters(store.selectedId)"
      >
        保存电机参数
      </button>
      <span id="sumDev" class="micro">{{ sumDevText }}</span>
    </div>
    <div id="paramResult" class="result tiny" :class="{ ok: paramResult?.ok, err: paramResult && !paramResult.ok }" role="status">{{ paramResult?.text || '' }}</div>
    <p id="paramSelectionHint" class="micro" :hidden="!showSelectionHint">{{ selectionHintText }}</p>
    <div id="paramRows" class="motor-parameter-list">
      <details
        v-for="item in motorItems"
        :key="item.id"
        class="motor-parameter-item"
        :class="{ unprotected: item.unprotected }"
        :data-motor="item.id"
        :hidden="!item.visible"
        :open="item.visible"
      >
        <summary>
          <strong>电机 {{ item.id }}</strong>
          <span class="motor-parameter-model">{{ item.model || '型号待识别' }}</span>
          <span class="pill" :class="item.pillClass" :title="item.pillTitle">{{ item.view.lifecycle }}</span>
          <span class="pill" :class="item.availability.verified ? 'mid' : 'off'">{{ item.srcText }}</span>
        </summary>
        <div class="motor-parameter-body">
          <p v-if="!item.availability.usable" class="panel-error">
            电机 {{ item.id }} {{ !item.model ? '型号尚未识别，固件尚未绑定型号；' : '' }}未返回有效参数，不能视为读取成功。
          </p>
          <p v-if="item.unprotected" class="panel-error">保护已关：该关节无任何限位。</p>
          <h3>电机参数</h3>
          <p class="micro">除方向外参数值只读；设置和设零须先失能，完成后核对实际位置与机械行程。</p>
          <dl>
            <div>
              <dt>电机 ID</dt>
              <dd>
                <input
                  aria-label="电机 ID"
                  type="number"
                  readonly
                  placeholder="未读取"
                  :value="fmtParam(item.pick.node_id)"
                  :title="fieldHint(item.v, item.row, 'node_id') || undefined"
                />
                <span v-if="rawValue(item.row, 'node_id', item.pick.node_id)" class="rawval muted tiny">{{ rawValue(item.row, 'node_id', item.pick.node_id) }}</span>
              </dd>
            </div>
            <div>
              <dt>T_max（N·m）</dt>
              <dd>
                <input
                  aria-label="T_max（N·m）"
                  type="number"
                  readonly
                  placeholder="未读取"
                  :value="fmtParam(item.pick.t_max)"
                  :title="fieldHint(item.v, item.row, 't_max') || undefined"
                />
                <span v-if="rawValue(item.row, 't_max', item.pick.t_max)" class="rawval muted tiny">{{ rawValue(item.row, 't_max', item.pick.t_max) }}</span>
              </dd>
            </div>
            <div>
              <dt>当前通信 ID</dt>
              <dd>{{ item.id }}{{ Number.isFinite(item.pick.node_id) && item.pick.node_id !== item.id ? '（新 ID 重启电机后生效）' : '' }}</dd>
            </div>
            <div>
              <dt>位置下限（rad）</dt>
              <dd>
                <input
                  aria-label="位置下限（rad）"
                  type="number"
                  readonly
                  placeholder="未读取"
                  :value="fmtParam(item.pick.pos_min)"
                  :title="fieldHint(item.v, item.row, 'pos_min') || undefined"
                />
                <span v-if="rawValue(item.row, 'pos_min', item.pick.pos_min)" class="rawval muted tiny">{{ rawValue(item.row, 'pos_min', item.pick.pos_min) }}</span>
              </dd>
            </div>
            <div>
              <dt>位置上限（rad）</dt>
              <dd>
                <input
                  aria-label="位置上限（rad）"
                  type="number"
                  readonly
                  placeholder="未读取"
                  :value="fmtParam(item.pick.pos_max)"
                  :title="fieldHint(item.v, item.row, 'pos_max') || undefined"
                />
                <span v-if="rawValue(item.row, 'pos_max', item.pick.pos_max)" class="rawval muted tiny">{{ rawValue(item.row, 'pos_max', item.pick.pos_max) }}</span>
              </dd>
            </div>
            <div>
              <dt>当前位置（rad）</dt>
              <dd class="curpos" :data-cur="item.id">{{ currentPositionText(item.id) }}</dd>
            </div>
            <div>
              <dt>速度上限（rad/s）</dt>
              <dd>
                <input
                  aria-label="速度上限（rad/s）"
                  type="number"
                  readonly
                  placeholder="未读取"
                  :value="fmtParam(item.pick.vel_limit)"
                  :title="fieldHint(item.v, item.row, 'vel_limit') || undefined"
                />
                <span v-if="rawValue(item.row, 'vel_limit', item.pick.vel_limit)" class="rawval muted tiny">{{ rawValue(item.row, 'vel_limit', item.pick.vel_limit) }}</span>
              </dd>
            </div>
            <div>
              <dt>电流上限（A）</dt>
              <dd>
                <input
                  aria-label="电流上限（A）"
                  type="number"
                  readonly
                  placeholder="未读取"
                  :value="fmtParam(item.pick.torque_limit)"
                  :title="fieldHint(item.v, item.row, 'torque_limit') || undefined"
                />
                <span v-if="rawValue(item.row, 'torque_limit', item.pick.torque_limit)" class="rawval muted tiny">{{ rawValue(item.row, 'torque_limit', item.pick.torque_limit) }}</span>
              </dd>
            </div>
            <div>
              <dt>限位保护</dt>
              <dd>{{ item.protectCell }}</dd>
            </div>
            <div>
              <dt>电机方向</dt>
              <dd>
                <select
                  :aria-label="`电机 ${item.id} 方向`"
                  :data-direction="item.id"
                  :value="item.dirSelectValue"
                  :disabled="!connected || Boolean(item.devError)"
                  @change="onDirectionChange(item.id, $event)"
                >
                  <option value="">未读取</option>
                  <option value="0">CW</option>
                  <option value="1">CCW</option>
                </select>
                <button
                  class="tiny-btn"
                  :data-setdirection="item.id"
                  :disabled="!connected || Boolean(item.devError) || item.dirSelectValue === ''"
                  :title="item.devError || '设置驱动力矩方向并回读核验'"
                  @click="onSetDirection(item.id)"
                >
                  设置方向
                </button>
                <span class="micro">仅改变驱动力矩方向；易失，驱动板保存不固化方向。</span>
              </dd>
            </div>
            <div>
              <dt>维护操作</dt>
              <dd>
                <button
                  class="tiny-btn"
                  :data-zero="item.id"
                  :disabled="!connected || Boolean(item.zeroError)"
                  :title="item.zeroError || '把该台当前位置设为设备零点（仅失能态受理）'"
                  @click="setMotorZero(item.id)"
                >
                  设零
                </button>
                <button
                  class="tiny-btn"
                  :data-clearfault="item.id"
                  :disabled="!connected"
                  title="清除故障锁存（仅非运动态受理；身份冲突锁存不可解除）"
                  @click="clearMotorFault(item.id)"
                >
                  清错
                </button>
              </dd>
            </div>
          </dl>
        </div>
      </details>
    </div>
  </section>
</template>
