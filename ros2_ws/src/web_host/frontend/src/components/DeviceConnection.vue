<script setup lang="ts">
import { onMounted } from 'vue';
const props = defineProps<{mcuOnly?: boolean}>();
const emit = defineEmits<{ direct: [value: boolean] }>();
onMounted(() => {
  sessionStorage.removeItem('device-can-session');
  sessionStorage.removeItem('device-can-transport');
  emit('direct', false);
});
</script>
<template>
  <section v-if="mcuOnly" class="device-connection surface mcu-source-note">
    <strong>统一 MCU 接口</strong><span>电机经控制板与多路 CAN FD 接入；主机直连入口已停用。</span>
    <span>设备参数与页面草稿分别显示，回读成功不代表运动授权。</span>
  </section>
  <template v-else>
  <section class="device-connection surface">
    <strong>通信来源：micro-ROS MCU</strong>
    <p class="tiny">设备状态与参数由控制板回报；主机直连 CAN 入口已停用。托管运行期间维护参数只读。</p>
  </section>
  </template>
</template>
