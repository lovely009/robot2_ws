<script setup lang="ts">
import { computed } from "vue";
import { Search, ChevronDown, Box, Cpu, CircleDot } from "@lucide/vue";
import { demo, model, measured, connected } from "../demo";
const joints = computed(
  () =>
    model.value?.joints.filter((j) =>
      `${j.name} ${j.motor?.type || ""}`
        .toLowerCase()
        .includes(demo.query.toLowerCase()),
    ) || [],
);
</script>

<template>
  <section class="joint-tree surface">
    <div class="panel-title">
      <span>对象浏览器</span><span class="micro">URDF + YAML</span>
    </div>
    <label class="search-box"
      ><Search :size="14" /><input
        v-model="demo.query"
        placeholder="搜索关节、设备型号"
        aria-label="搜索对象"
      /><kbd>/</kbd></label
    >
    <div class="tree-root">
      <ChevronDown :size="13" /><Box :size="15" /><b>{{ demo.model }}</b>
    </div>
    <div class="tree-caption">
      <span>关节 / 映射设备</span><span>实测 rad</span>
    </div>
    <div class="joint-scroll">
      <button
        v-for="j in joints"
        :key="j.name"
        class="joint-row"
        :class="{ selected: demo.selected === j.name }"
        @click="demo.selected = j.name"
      >
        <span class="joint-symbol"><CircleDot :size="14" /></span>
        <span class="joint-info"
          ><b>{{ j.name }}</b
          ><small>{{
            j.motor
              ? `#${j.motor.id} · ${j.motor.type}`
              : "未映射设备 · 几何预览"
          }}</small></span
        >
        <span class="joint-number mono">{{
          connected && j.motor ? measured(j).toFixed(3) : "—"
        }}</span>
        <i class="dot" :class="connected && j.motor ? 'green' : 'muted'" />
      </button>
      <div v-if="!joints.length" class="empty">没有匹配的对象</div>
    </div>
    <footer class="tree-footer">
      <Cpu :size="14" /><span
        >{{ model?.joints.length || 0 }} 关节 ·
        {{ model?.joints.filter((j) => j.motor).length || 0 }} 映射设备</span
      >
    </footer>
  </section>
</template>
