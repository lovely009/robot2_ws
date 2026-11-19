import type { MountedPanel } from "./contracts";

export class PanelScheduler {
  private entries = new Map<
    string,
    {
      panel: MountedPanel;
      visible: () => boolean;
      pending: Set<string>;
      last: number | null;
      failures: number;
      retryAt: number;
    }
  >();
  constructor(private onError: (id: string, error: unknown) => void) {}
  add(id: string, panel: MountedPanel, visible: () => boolean) {
    if (this.entries.has(id)) throw new Error(`面板重复挂载：${id}`);
    const entry = {
      panel,
      visible,
      pending: new Set<string>(),
      last: null as number | null,
      failures: 0,
      retryAt: 0,
    };
    this.entries.set(id, entry);
    return () => {
      if (this.entries.get(id) !== entry) return;
      this.entries.delete(id);
      try {
        panel.unmount?.();
      } catch (error) {
        this.onError(id, error);
      }
    };
  }
  invalidate(dirty: Set<string>) {
    for (const entry of this.entries.values()) {
      for (const key of dirty) entry.pending.add(key);
    }
  }
  retryFailed() {
    for (const entry of this.entries.values()) {
      if (entry.failures >= 3) entry.failures = 0;
    }
  }
  flush(now: number) {
    for (const [id, entry] of this.entries) {
      if (!entry.pending.size || !entry.visible()) continue;
      if (entry.failures >= 3 || now < entry.retryAt) continue;
      if (
        entry.last !== null &&
        now - entry.last < (entry.panel.intervalMs || 0)
      )
        continue;
      const dirty = entry.pending;
      entry.pending = new Set();
      entry.last = now;
      try {
        entry.panel.render(dirty);
        entry.failures = 0;
        entry.retryAt = 0;
      } catch (error) {
        for (const key of dirty) entry.pending.add(key);
        entry.failures++;
        entry.retryAt = now + 250;
        this.onError(id, error);
      }
    }
  }
  dispose() {
    const entries = [...this.entries];
    this.entries.clear();
    for (const [id, { panel }] of entries) {
      try {
        panel.unmount?.();
      } catch (error) {
        this.onError(id, error);
      }
    }
  }
}
