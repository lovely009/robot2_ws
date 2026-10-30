<script setup lang="ts">
import { onMounted } from 'vue';
const emit = defineEmits<{ direct: [value: boolean] }>();
onMounted(() => {
  sessionStorage.removeItem('device-can-session');
  sessionStorage.removeItem('device-can-transport');
  emit('direct', false);
});
</script>
<template>
  <section class="device-connection surface">
    <strong>通信来源：micro-ROS MCU</strong>
    <p class="tiny">设备状态与参数由控制板回报；主机直连 CAN 入口已停用。托管运行期间维护参数只读。</p>
  </section>
</template>
