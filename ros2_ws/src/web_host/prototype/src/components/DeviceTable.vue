<script setup lang="ts">
import { ListFilter } from "@lucide/vue";
import { demo, model, measured, connected } from "../demo";
</script>
<template>
  <section class="device-table surface">
    <div class="panel-title">
      <span
        >设备监视
        <small class="micro"
          >{{ model?.joints.filter((j) => j.motor).length || 0 }} DEVICES</small
        ></span
      ><span class="micro"><ListFilter :size="12" /> 全部映射设备</span>
    </div>
    <div class="table-scroll">
      <table>
        <thead>
          <tr>
            <th>设备 / 关节</th>
            <th>位置 <small>rad</small></th>
            <th>速度 <small>rad/s</small></th>
            <th>状态</th>
          </tr>
        </thead>
        <tbody>
          <tr
            v-for="j in model?.joints.filter((j) => j.motor)"
            :key="j.name"
            :class="{ selected: demo.selected === j.name }"
            tabindex="0"
            @click="demo.selected = j.name"
            @keydown.enter="demo.selected = j.name"
          >
            <td>
              <span class="id-cell mono">{{
                String(j.motor?.id).padStart(2, "0")
              }}</span
              ><b>{{ j.name }}</b>
            </td>
            <td class="mono">{{ connected ? measured(j).toFixed(4) : "—" }}</td>
            <td class="mono">
              {{
                connected
                  ? (
                      Math.cos(demo.clock * 0.85 + (j.motor?.id || 0) * 0.3) *
                      0.0102
                    ).toFixed(4)
                  : "—"
              }}
            </td>
            <td>
              <span class="state-text"
                ><i class="dot" :class="connected ? 'green' : 'amber'" />{{
                  connected ? "待使能" : "数据陈旧"
                }}</span
              >
            </td>
          </tr>
        </tbody>
      </table>
      <div v-if="!model?.joints.some((j) => j.motor)" class="empty">
        当前模型未配置设备映射，可查看关节和几何。
      </div>
    </div>
  </section>
</template>
