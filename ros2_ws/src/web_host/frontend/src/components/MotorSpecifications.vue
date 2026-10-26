<script setup lang="ts">
import { computed, onMounted, ref } from 'vue';
import { runtime, snapshot } from '../runtime';
const props = defineProps<{ model?: string; motorId?: number }>();
const library = ref<Record<string, Record<string, unknown>>>({});
const error = ref('');
const model = computed(() => { void snapshot.value; return props.model ?? runtime.selectedMotorModel(); });
const parameters = computed(() => library.value[model.value] || {});
onMounted(async () => {
  try {
    const response = await fetch('/api/motor-specs');
    if (!response.ok) throw new Error('型号参数库读取失败');
    library.value = (await response.json()).motor_types || {};
  } catch (reason) { error.value = String(reason); }
});
</script>
<template>
  <section class="motor-specifications">
    <h3>电机 {{ props.motorId ?? (snapshot.selectedId || '未选择') }} · {{ model || '型号待识别' }}</h3>
    <p class="micro">型号参数库（规格值）；设备实际回读值另列显示。</p>
    <p v-if="error" role="status">{{ error }}</p>
    <dl class="specification-grid">
      <template v-for="(value, key) in parameters" :key="key">
        <dt>{{ key }}</dt><dd>{{ value }}</dd>
      </template>
    </dl>
  </section>
</template>
