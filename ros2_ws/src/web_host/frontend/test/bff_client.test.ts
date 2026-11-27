import { beforeEach, describe, expect, it, vi } from "vitest";
vi.hoisted(() => {
  (globalThis as any).location ??= { search: "", protocol: "http:", host: "localhost:8080" };
});
import { setActivePinia } from "pinia";
import { BffClient } from "../src/api/bff_client";
import { pinia } from "../src/stores";
import { useLeaseStore } from "../src/stores/lease";
import { useDiagnosticsStore } from "../src/stores/diagnostics";
import * as ros from "../src/core/ros";

describe("BffClient and Pinia stores", () => {
  beforeEach(() => {
    setActivePinia(pinia);
    vi.restoreAllMocks();
  });

  it("sends X-Client-Id and X-Lease-Token headers after acquiring lease", async () => {
    const calls: { url: string; headers: Record<string, string>; body?: string }[] = [];
    vi.stubGlobal("fetch", vi.fn(async (url: string, init: RequestInit = {}) => {
      calls.push({
        url: String(url),
        headers: (init.headers || {}) as Record<string, string>,
        body: init.body ? String(init.body) : undefined,
      });
      if (String(url) === "/api/control/lease/acquire") {
        return new Response(JSON.stringify({
          lease: {
            owner_id: "c-test",
            owner_label: "c-test",
            active_controllers: 1,
            connected_clients: 1,
            pin_required: false,
            epoch: 1,
          },
          token: "tok-123",
          preempted_previous: false,
        }), { status: 200 });
      }
      return new Response(JSON.stringify({ ok: true, service: "/bench/stop_simulation", result: { stopped: true } }), { status: 200 });
    }));

    const client = new BffClient("c-test");
    const acq = await client.acquireLease({ label: "Tablet" });
    expect(acq.token).toBe("tok-123");

    const res = await client.callDomainService("/bench/stop_simulation", {});
    expect(res).toEqual({ stopped: true });
    expect(calls[1]!.url).toBe("/api/bench/simulation/stop");
    expect(calls[1]!.headers["X-Client-Id"]).toBe("c-test");
    expect(calls[1]!.headers["X-Lease-Token"]).toBe("tok-123");
  });

  it("configures BFF transport on useLeaseStore and routes stop in readonly mode", async () => {
    const fetchMock = vi.fn(async () => new Response(JSON.stringify({
      accepted: true, published: true, reason: "operator_stop", timestamp_ms: 1,
    }), { status: 200 }));
    vi.stubGlobal("fetch", fetchMock);

    const leaseStore = useLeaseStore(pinia);
    ros.setBffObserver({
      onConfigure: (cfg) => leaseStore.configureTransport(cfg),
    });
    ros.configureControl({ control_mode: "readonly", transport: "bff", pin_required: true });
    expect(leaseStore.isBff).toBe(true);
    expect(leaseStore.pinRequired).toBe(true);

    ros.setConn(true, "已连接 (BFF)");
    expect(ros.ensurePublisher("jointTarget")).toBeNull();
    expect(ros.ensurePublisher("simJointTarget")).toBeNull();
    expect(ros.pub("stop", {})).toBe(true);
    expect(ros.pub("jointTarget", { name: ["j1"], position: [0.1] })).toBe(false);
    expect(fetchMock).toHaveBeenCalledWith("/api/control/stop", expect.anything());
  });

  it("refreshes diagnostics and recordings through useDiagnosticsStore", async () => {
    vi.stubGlobal("fetch", vi.fn(async (url: string) => {
      if (String(url) === "/api/diagnostics") {
        return new Response(JSON.stringify({
          uptime_s: 12.3,
          ros_connected: true,
          event_loop_lag_ms: 0.4,
          ws_clients: 2,
          active_controllers: 1,
          ws_broadcast_hz: 30,
          topics: {},
          recording_active: false,
        }), { status: 200 });
      }
      return new Response(JSON.stringify({
        recordings: [{ name: "rec_1", storage_format: "mcap", size_bytes: 1024, duration_s: 1.5, message_count: 42, topics: ["/joint_states"], created_at_ms: 100 }],
        status: { active: false, storage_format: "mcap" },
      }), { status: 200 });
    }));

    const diag = useDiagnosticsStore(pinia);
    await diag.refreshDiagnostics();
    await diag.refreshRecordings();
    expect(diag.snapshot?.ros_connected).toBe(true);
    expect(diag.recordings).toHaveLength(1);
    expect(diag.recordings[0]!.name).toBe("rec_1");
  });
});
