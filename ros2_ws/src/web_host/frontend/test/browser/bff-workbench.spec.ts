import { test, expect } from "@playwright/test";

test("BFF 模式支持控制租约申请/释放、全员软停与诊断录播面板", async ({ page }) => {
  let leaseState = {
    owner_id: null as string | null,
    owner_label: null as string | null,
    acquired_at_ms: null as number | null,
    expires_at_ms: null as number | null,
    active_controllers: 0,
    connected_clients: 1,
    pin_required: false,
    epoch: 1,
  };
  let recordingActive = false;
  let recordings: any[] = [];
  const subscribedChannels: string[][] = [];
  let stopCount = 0;

  await page.routeWebSocket("**/ws**", (ws) => {
    ws.send(JSON.stringify({
      op: "hello",
      client_id: "web-test",
      control_mode: "bench",
      lease: leaseState,
    }));
    ws.onMessage((raw) => {
      const msg = JSON.parse(String(raw));
      if (msg.op === "subscribe") {
        subscribedChannels.push(msg.channels);
        ws.send(JSON.stringify({ op: "subscribed", channels: msg.channels }));
        ws.send(JSON.stringify({
          channel: "telemetry:control",
          seq: 1,
          server_ts_ms: Date.now(),
          ros_stamp_ns: Date.now() * 1e6,
          source_hz: 10,
          age_ms: 2,
          data: { model: "half_body_robot", joint_names: ["joint_link1"], mode: "idle", reason: "", reject: "", feedback_fresh: true, controllers_ready: true, active_controllers: [] },
        }));
      } else if (msg.op === "heartbeat") {
        ws.send(JSON.stringify({
          op: "pong",
          client_ts_ms: msg.client_ts_ms,
          server_ts_ms: Date.now(),
          lease_ok: true,
        }));
      }
    });
  });

  await page.route("**/api/**", async (route) => {
    const url = new URL(route.request().url());
    const method = route.request().method();
    if (url.pathname === "/api/runtime") {
      return route.fulfill({ json: { control_mode: "bench", robot_model: "half_body_robot", transport: "bff", pin_required: false } });
    }
    if (url.pathname === "/api/models") return route.fulfill({ json: { models: ["half_body_robot"] } });
    if (url.pathname === "/api/model-transmissions") {
      return route.fulfill({
        json: {
          model: "half_body_robot",
          groups: [{ name: "g1", plugin: "SimpleTransmission", motors: [{ name: "m1", id: 1 }], joints: ["joint_link1"], direct_mapping: true }],
          direct_joints: { 1: "joint_link1" },
          policy_defaults: { 1: { kp: 30, kd: 1 } },
        },
      });
    }
    if (url.pathname === "/api/motor-specs") return route.fulfill({ json: { motor_types: {} } });
    if (url.pathname === "/api/actions") return route.fulfill({ json: { actions: [] } });
    if (url.pathname === "/api/motion/sources/status") return route.fulfill({ json: { sources: [] } });
    if (url.pathname === "/api/bench/options") return route.fulfill({ json: { ok: true, service: "/bench/options", result: { models: ["half_body_robot"] } } });
    if (url.pathname === "/api/bench/agent-status") return route.fulfill({ json: { ok: true, service: "/bench/agent_status", result: { running: false } } });
    if (url.pathname === "/api/bench/simulation/start" || url.pathname === "/api/control/parameters") {
      return route.fulfill({ json: { ok: true, service: url.pathname, result: {} } });
    }
    if (url.pathname === "/api/control/lease/acquire" && method === "POST") {
      const body = route.request().postDataJSON();
      leaseState = { ...leaseState, owner_id: body.client_id, owner_label: body.label, active_controllers: 1, epoch: leaseState.epoch + 1 };
      return route.fulfill({ json: { lease: leaseState, token: "tok-pw", preempted_previous: false } });
    }
    if (url.pathname === "/api/control/lease/release" && method === "POST") {
      leaseState = { ...leaseState, owner_id: null, owner_label: null, active_controllers: 0, epoch: leaseState.epoch + 1 };
      return route.fulfill({ json: leaseState });
    }
    if (url.pathname === "/api/control/stop" && method === "POST") {
      stopCount++;
      return route.fulfill({ json: { accepted: true, published: true, reason: "operator_stop", timestamp_ms: Date.now() } });
    }
    if (url.pathname === "/api/diagnostics") {
      return route.fulfill({
        json: {
          uptime_s: 15.2,
          ros_connected: true,
          event_loop_lag_ms: 0.25,
          ws_clients: 1,
          active_controllers: leaseState.active_controllers,
          ws_broadcast_hz: 30,
          topics: {
            "telemetry:joints": { topic: "/joint_states", channel: "telemetry:joints", rx_count: 100, rx_hz: 100.0, age_ms: 4.0 },
          },
          recording_active: recordingActive,
        },
      });
    }
    if (url.pathname === "/api/recordings") {
      return route.fulfill({
        json: {
          recordings,
          status: { active: recordingActive, name: recordingActive ? "pw_bag" : null, storage_format: "mcap", duration_s: 1.0, message_count: 12, topics: ["/joint_states"] },
        },
      });
    }
    if (url.pathname === "/api/recordings/start" && method === "POST") {
      recordingActive = true;
      return route.fulfill({ json: { active: true, name: "pw_bag", storage_format: "mcap", duration_s: 0.0, message_count: 0, topics: [] } });
    }
    if (url.pathname === "/api/recordings/stop" && method === "POST") {
      recordingActive = false;
      const info = { name: "pw_bag", storage_format: "mcap", size_bytes: 2048, duration_s: 1.2, message_count: 24, topics: ["/joint_states"], created_at_ms: Date.now() };
      recordings = [info];
      return route.fulfill({ json: info });
    }
    return route.fulfill({ json: {} });
  });

  await page.setViewportSize({ width: 1280, height: 720 });
  await page.goto("/workbench/#/run");
  // First visitor auto-acquires control when idle
  await expect(page.getByRole("region", { name: "控制权租约" })).toContainText("当前持控中");
  await expect(page.locator(".toast")).toHaveCount(0);
  await expect(page.locator("#rosUrl")).toHaveCount(0);

  // Inspector dock and observer area must be horizontally disjoint (no overlap)
  const dockBox = await page.locator(".inspector-dock").boundingBox();
  const obsBox = await page.locator(".observer-area").boundingBox();
  expect(dockBox && obsBox).toBeTruthy();
  expect(obsBox!.x + obsBox!.width).toBeLessThanOrEqual(dockBox!.x + 1);

  // Inspector dock 3-tab switching: 关节 -> 动作库 -> 电机 -> 关节
  await page.getByRole("tab", { name: "动作库" }).click();
  await expect(page.locator(".action-panel-host")).toBeVisible();
  await page.getByRole("tab", { name: /电机/ }).click();
  await expect(page.locator(".motor-inspector-panel")).toBeVisible();
  await page.getByRole("tab", { name: /关节/ }).click();
  await expect(page.locator("#run-joints")).toBeVisible();

  // Viewer can trigger universal E-Stop via button and Escape shortcut
  await page.getByRole("button", { name: "全部软停" }).click();
  await expect.poll(() => stopCount).toBe(1);
  await page.keyboard.press("Escape");
  await expect.poll(() => stopCount).toBe(2);

  // 3D view preset buttons & joint rad/deg toggle & curve export buttons
  await expect(page.getByRole("button", { name: "正视图" })).toBeVisible();
  await page.getByRole("button", { name: "复位视角" }).click();
  const unitBtn = page.getByRole("button", { name: "切换角度单位" });
  await expect(unitBtn).toHaveText("rad");
  await unitBtn.click();
  await expect(unitBtn).toHaveText("deg");
  await expect(page.getByRole("button", { name: "导出 CSV" })).toBeVisible();
  await expect(page.getByRole("button", { name: "保存图" })).toBeVisible();

  // Release and re-acquire lease
  await page.getByRole("button", { name: "释放控制权" }).click();
  await expect(page.getByRole("region", { name: "控制权租约" })).toContainText("只读围观");
  await page.getByRole("button", { name: "申请控制权" }).click();
  await expect(page.getByRole("region", { name: "控制权租约" })).toContainText("当前持控中");

  // Open Diagnostics & Recorder observer tab
  await page.getByRole("button", { name: "诊断与录播" }).click();
  const diag = page.getByRole("region", { name: "链路诊断与工程录播" });
  await expect(diag).toContainText("/joint_states");
  await expect(diag).toContainText("100.0 Hz");

  await page.getByRole("button", { name: "开始录制" }).click();
  await expect(page.getByRole("button", { name: /停止录制/ })).toBeVisible();
  await page.getByRole("button", { name: /停止录制/ }).click();
  await expect(diag).toContainText("pw_bag");
  expect(subscribedChannels.length).toBeGreaterThan(0);
});
