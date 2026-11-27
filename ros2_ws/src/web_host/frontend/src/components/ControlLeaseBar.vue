<script setup lang="ts">
import { ref } from "vue";
import { ShieldCheck, ShieldAlert, Lock, Unlock } from "@lucide/vue";
import { useLeaseStore } from "../stores/lease";

const leaseStore = useLeaseStore();
const pinInput = ref("");

async function handleAcquire(force: boolean) {
  try {
    await leaseStore.acquire(force, leaseStore.pinRequired ? pinInput.value : null);
    pinInput.value = "";
  } catch {
    /* error displayed via leaseStore.error */
  }
}
</script>

<template>
  <div class="lease-bar" role="region" aria-label="控制权租约">
    <span
      class="lease-badge"
      :class="leaseStore.isOwner ? 'owner' : leaseStore.hasOtherOwner ? 'busy' : 'idle'"
    >
      <ShieldCheck v-if="leaseStore.isOwner" :size="14" />
      <ShieldAlert v-else-if="leaseStore.hasOtherOwner" :size="14" />
      <Lock v-else :size="14" />
      <template v-if="leaseStore.isOwner">当前持控中</template>
      <template v-else-if="leaseStore.hasOtherOwner">由 {{ leaseStore.ownerLabel }} 持控</template>
      <template v-else>只读围观 · {{ leaseStore.connectedClients }} 在线</template>
    </span>

    <span v-if="leaseStore.wsRttMs !== null" class="micro lease-rtt" title="WebSocket 心跳往返延迟">
      {{ leaseStore.wsRttMs }} ms
    </span>

    <input
      v-if="leaseStore.pinRequired && !leaseStore.isOwner"
      v-model="pinInput"
      type="password"
      class="lease-pin"
      placeholder="PIN"
      aria-label="控制口令"
      autocomplete="off"
    />

    <button
      v-if="!leaseStore.isOwner"
      type="button"
      class="compact"
      :class="leaseStore.hasOtherOwner ? 'secondary' : ''"
      :disabled="leaseStore.busy"
      @click="handleAcquire(leaseStore.hasOtherOwner)"
    >
      <Unlock :size="13" />
      {{ leaseStore.hasOtherOwner ? '强制接管' : '申请控制权' }}
    </button>

    <button
      v-else
      type="button"
      class="secondary compact"
      :disabled="leaseStore.busy"
      @click="leaseStore.release()"
    >
      释放控制权
    </button>

    <span v-if="leaseStore.error" class="lease-err tiny" role="alert">{{ leaseStore.error }}</span>
  </div>
</template>
