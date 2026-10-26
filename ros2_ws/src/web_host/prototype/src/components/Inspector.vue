<script setup lang="ts">
import { ref } from "vue";
import {
  SlidersHorizontal,
  ArrowUpRight,
  Send,
  Check,
  Info,
} from "@lucide/vue";
import {
  demo,
  selected,
  draft,
  measured,
  can_command,
  connected,
  send,
  notify,
} from "../demo";
const tab = ref("control");
</script>

<template>
  <aside class="inspector surface">
    <div class="panel-title">
      <span>对象检查器</span><SlidersHorizontal :size="14" />
    </div>
    <template v-if="selected && draft">
      <div class="object-heading">
        <div class="eyebrow">
          {{
            selected.motor
              ? `ACTUATOR / ${String(selected.motor.id).padStart(2, "0")}`
              : "JOINT / PREVIEW"
          }}
        </div>
        <h2>{{ selected.name }}</h2>
        <span
          class="status-chip"
          :class="connected && selected.motor ? 'success' : ''"
          ><i
            class="dot"
            :class="connected && selected.motor ? 'green' : 'muted'"
          />{{
            selected.motor
              ? connected
                ? "READY_DISABLED · 模拟"
                : "数据陈旧"
              : "未映射设备"
          }}</span
        >
      </div>
      <div class="inspector-tabs">
        <button :class="{ active: tab === 'control' }" @click="tab = 'control'">
          控制</button
        ><button
          :class="{ active: tab === 'parameters' }"
          @click="tab = 'parameters'"
        >
          参数</button
        ><button
          :class="{ active: tab === 'details' }"
          @click="tab = 'details'"
        >
          详情
        </button>
      </div>
      <div class="inspector-scroll">
        <template v-if="tab === 'control'">
          <div class="metric-pair">
            <div>
              <small>当前位置 <span>rad</span></small
              ><strong class="mono">{{
                connected && selected.motor
                  ? measured(selected).toFixed(4)
                  : "—"
              }}</strong>
            </div>
            <div>
              <small>目标位置 <span>rad</span></small
              ><strong class="mono target-value">{{
                Number(draft.target).toFixed(4)
              }}</strong>
            </div>
          </div>
          <div class="section-label">
            位置设定 <span class="tiny-tag">草稿</span>
          </div>
          <label class="field"
            ><span>目标位置 <small>rad</small></span
            ><input
              v-model.number="draft.target"
              type="number"
              step="0.01"
              :min="selected.lower"
              :max="selected.upper"
              aria-label="目标位置"
          /></label>
          <input
            v-model.number="draft.target"
            class="position-range"
            type="range"
            :min="selected.lower"
            :max="selected.upper"
            step="0.001"
            aria-label="目标位置滑块"
          />
          <div class="range-labels mono">
            <span>{{ selected.lower.toFixed(2) }}</span
            ><button
              class="text-button"
              @click="
                draft.target = Math.max(
                  selected.lower,
                  Math.min(selected.upper, 0),
                )
              "
            >
              零位</button
            ><span>{{ selected.upper.toFixed(2) }}</span>
          </div>
          <div class="section-label">
            策略草稿 <span class="micro">协议原值</span>
          </div>
          <div class="gain-fields">
            <label class="field"
              ><span>Kp</span
              ><input
                v-model.number="draft.kp"
                type="number"
                min="0"
                step="0.1"
                aria-label="Kp" /></label
            ><label class="field"
              ><span>Kd</span
              ><input
                v-model.number="draft.kd"
                type="number"
                min="0"
                step="0.01"
                aria-label="Kd"
            /></label>
          </div>
          <label class="toggle-row"
            ><span>使能意图 <small>仅编辑草稿</small></span
            ><input
              v-model="draft.enabled"
              type="checkbox"
              role="switch"
              aria-label="使能草稿" /><span class="toggle-track"
          /></label>
          <div class="notice">
            <Info :size="14" /><span
              >修改数值只更新草稿与叠影，设备实测姿态独立显示。</span
            >
          </div>
          <div v-if="!selected.motor" class="notice warning">
            当前模型没有关节与电机映射，仅支持几何和动作预览。
          </div>
          <button class="primary full" :disabled="!can_command" @click="send">
            <Send :size="14" />模拟下发到所选设备<ArrowUpRight :size="14" />
          </button>
          <div class="command-result">
            <Check :size="13" /><span
              >{{ demo.result
              }}<small v-if="demo.result_joint">{{
                demo.result_joint
              }}</small></span
            >
          </div>
        </template>
        <template v-else-if="tab === 'parameters'">
          <div class="section-label">
            模型限位 <span class="tiny-tag">URDF · 只读</span>
          </div>
          <dl class="detail-list">
            <div>
              <dt>位置下限</dt>
              <dd>{{ selected.lower.toFixed(4) }} rad</dd>
            </div>
            <div>
              <dt>位置上限</dt>
              <dd>{{ selected.upper.toFixed(4) }} rad</dd>
            </div>
            <div>
              <dt>关节类型</dt>
              <dd>{{ selected.type }}</dd>
            </div>
          </dl>
          <div class="notice">
            <Info :size="14" /><span
              >此处来自模型，不代表设备端限位已生效。</span
            >
          </div>
          <div class="section-label">设备参数事务</div>
          <div class="transaction-demo">
            <span>编辑</span><b>→</b><span>写入 RAM</span><b>→</b
            ><span>核验</span><b>→</b><span>保存</span>
          </div>
          <p class="muted-copy">
            正式接入后分别显示固件记录和设备回读，以及尚未保存的修改。
          </p>
          <button
            class="secondary full"
            @click="notify('原型参数说明：尚未接入设备读取、写入或保存')"
          >
            查看事务说明
          </button>
        </template>
        <template v-else>
          <div class="section-label">对象来源</div>
          <dl class="detail-list">
            <div>
              <dt>关节名称</dt>
              <dd>{{ selected.name }}</dd>
            </div>
            <div>
              <dt>父 link</dt>
              <dd>{{ selected.parent }}</dd>
            </div>
            <div>
              <dt>子 link</dt>
              <dd>{{ selected.child }}</dd>
            </div>
            <div>
              <dt>电机 ID</dt>
              <dd>{{ selected.motor?.id ?? "未映射" }}</dd>
            </div>
            <div>
              <dt>型号</dt>
              <dd>{{ selected.motor?.type ?? "未配置" }}</dd>
            </div>
            <div>
              <dt>总线</dt>
              <dd>{{ selected.motor?.bus ?? "未配置" }}</dd>
            </div>
            <div>
              <dt>素材来源</dt>
              <dd>源码 URDF / YAML</dd>
            </div>
            <div>
              <dt>实时数据</dt>
              <dd>本地生成的演示样本</dd>
            </div>
          </dl>
        </template>
      </div>
    </template>
    <div v-else class="empty">在对象树中选择一个关节</div>
  </aside>
</template>
