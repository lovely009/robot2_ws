import { defineStore } from "pinia";
import { computed, ref } from "vue";
import type { LeaseState } from "../api/bff";
import { bffClient } from "../api/bff_client";

export const useLeaseStore = defineStore("lease", () => {
  const transport = ref<"bff" | "rosbridge">("rosbridge");
  const clientId = ref(bffClient.clientId);
  const token = ref<string | null>(null);
  const lease = ref<LeaseState | null>(null);
  const wsRttMs = ref<number | null>(null);
  const pinRequired = ref(false);
  const busy = ref(false);
  const error = ref("");
  const manualReleased = ref(false);

  const isBff = computed(() => transport.value === "bff");
  const isOwner = computed(
    () => Boolean(token.value) && lease.value?.owner_id === clientId.value,
  );
  const hasOtherOwner = computed(
    () =>
      Boolean(lease.value?.owner_id) &&
      lease.value?.owner_id !== clientId.value,
  );
  const ownerLabel = computed(
    () => lease.value?.owner_label || lease.value?.owner_id || "",
  );
  const connectedClients = computed(() => lease.value?.connected_clients ?? 0);
  const activeControllers = computed(() => lease.value?.active_controllers ?? 0);

  function configureTransport(cfg: {
    transport?: string;
    pin_required?: boolean;
  } | null) {
    transport.value = cfg?.transport === "bff" ? "bff" : "rosbridge";
    pinRequired.value = Boolean(cfg?.pin_required);
  }

  function applyLeaseState(next: LeaseState) {
    lease.value = next;
    pinRequired.value = Boolean(next.pin_required);
    if (next.owner_id !== clientId.value && token.value) {
      token.value = null;
      bffClient.setToken(null);
    }
  }

  function setRtt(ms: number) {
    wsRttMs.value = Math.round(ms);
  }

  async function acquire(force = false, pin?: string | null, label?: string) {
    busy.value = true;
    error.value = "";
    manualReleased.value = false;
    try {
      const res = await bffClient.acquireLease({ force, pin, label });
      token.value = res.token;
      applyLeaseState(res.lease);
      return res;
    } catch (err) {
      error.value = err instanceof Error ? err.message : String(err);
      throw err;
    } finally {
      busy.value = false;
    }
  }

  async function release() {
    busy.value = true;
    error.value = "";
    manualReleased.value = true;
    try {
      const next = await bffClient.releaseLease();
      token.value = null;
      applyLeaseState(next);
      return next;
    } catch (err) {
      error.value = err instanceof Error ? err.message : String(err);
      throw err;
    } finally {
      busy.value = false;
    }
  }

  async function tryAutoAcquire(controlMode: string) {
    if (
      !isBff.value
      || controlMode !== "bench"
      || pinRequired.value
      || manualReleased.value
      || busy.value
      || isOwner.value
      || (lease.value && lease.value.active_controllers !== 0)
    ) {
      return;
    }
    try {
      await acquire(false);
    } catch {
      /* another client won the race; stay read-only */
    }
  }

  async function emergencyStop(reason = "operator_stop") {
    error.value = "";
    return bffClient.triggerStop(reason);
  }

  return {
    transport,
    clientId,
    token,
    lease,
    wsRttMs,
    pinRequired,
    busy,
    error,
    manualReleased,
    isBff,
    isOwner,
    hasOtherOwner,
    ownerLabel,
    connectedClients,
    activeControllers,
    configureTransport,
    applyLeaseState,
    setRtt,
    acquire,
    release,
    tryAutoAcquire,
    emergencyStop,
  };
});
