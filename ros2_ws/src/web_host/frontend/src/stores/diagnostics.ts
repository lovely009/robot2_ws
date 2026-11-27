import { defineStore } from "pinia";
import { ref } from "vue";
import type {
  DiagnosticsSnapshot,
  RecordingInfo,
  RecordingStatus,
} from "../api/bff";
import { bffClient } from "../api/bff_client";

export const useDiagnosticsStore = defineStore("diagnostics", () => {
  const snapshot = ref<DiagnosticsSnapshot | null>(null);
  const recordings = ref<RecordingInfo[]>([]);
  const recordingStatus = ref<RecordingStatus>({
    active: false,
    storage_format: "mcap",
  });
  const busy = ref(false);
  const error = ref("");

  async function refreshDiagnostics() {
    try {
      snapshot.value = await bffClient.getDiagnostics();
      error.value = "";
    } catch (err) {
      error.value = err instanceof Error ? err.message : String(err);
    }
  }

  async function refreshRecordings() {
    try {
      const res = await bffClient.listRecordings();
      recordings.value = res.recordings;
      recordingStatus.value = res.status;
      error.value = "";
    } catch (err) {
      error.value = err instanceof Error ? err.message : String(err);
    }
  }

  async function startRecording(name = "", topics: string[] = []) {
    busy.value = true;
    error.value = "";
    try {
      recordingStatus.value = await bffClient.startRecording(name, topics);
      await refreshRecordings();
    } catch (err) {
      error.value = err instanceof Error ? err.message : String(err);
      throw err;
    } finally {
      busy.value = false;
    }
  }

  async function stopRecording() {
    busy.value = true;
    error.value = "";
    try {
      const info = await bffClient.stopRecording();
      await refreshRecordings();
      return info;
    } catch (err) {
      error.value = err instanceof Error ? err.message : String(err);
      throw err;
    } finally {
      busy.value = false;
    }
  }

  async function deleteRecording(name: string) {
    busy.value = true;
    error.value = "";
    try {
      await bffClient.deleteRecording(name);
      await refreshRecordings();
    } catch (err) {
      error.value = err instanceof Error ? err.message : String(err);
      throw err;
    } finally {
      busy.value = false;
    }
  }

  return {
    snapshot,
    recordings,
    recordingStatus,
    busy,
    error,
    refreshDiagnostics,
    refreshRecordings,
    startRecording,
    stopRecording,
    deleteRecording,
  };
});
