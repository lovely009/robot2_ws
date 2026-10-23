import { test, expect, type Page } from "@playwright/test";
import { readFileSync } from "node:fs";

async function connectFixture(page: Page) {
  const publishes: { topic: string; msg: Record<string, unknown> }[] = [];
  const errors: string[] = [];
  const connections = { count: 0 };
  page.on("pageerror", (error) => errors.push(error.message));
  await page.route("**/api/**", (route) => {
    const url = new URL(route.request().url());
    if (url.pathname === "/api/models")
      return route.fulfill({
        json: { models: ["half_body_robot", "robot_leg"] },
      });
    if (url.pathname === "/api/model-joints")
      return route.fulfill(
        url.searchParams.get("model") === "robot_leg"
          ? { status: 404, json: { error: "没有设备映射" } }
          : { json: { joints: { 1: "joint_link1" } } },
      );
    if (url.pathname === "/api/motor-specs")
      return route.fulfill({ json: { motor_types: {} } });
    return route.fulfill({ json: { actions: [] } });
  });
  const fixtures: Record<string, unknown> = {
    "/micro_ros_up/motor_topology": {
      entries: [
        {
          motor_id: 1,
          online: true,
          model_name: "cerb_r48_101",
          bus_index: 0,
          lifecycle: 3,
        },
      ],
    },
    "/micro_ros_up/motion_states": {
      states: [
        {
          motor_id: 1,
          position: 0.15,
          velocity: 0.2,
          effort: 0.1,
          target_source: 1,
          state_age_ms: 5,
        },
      ],
    },
    "/micro_ros_up/motor_status_array": {
      statuses: [
        {
          motor_id: 1,
          online: true,
          lifecycle: 3,
          current: 0.2,
          temperature: 30,
          unified_fault_code: 0,
        },
      ],
    },
    "/motor_command/status": {
      data: JSON.stringify({
        output_topic: "/motor/command_array",
        clients: 1,
        targets: 0,
      }),
    },
    "/bench/status": {
      data: JSON.stringify({
        agent: { state: "stopped" },
        simulation: { state: "stopped" },
        motor_backend: { state: "stopped" },
      }),
    },
    "/micro_ros_up/system_status": {
      refresh_seq: 0,
      set_zero_seq: 0,
      clear_fault_seq: 0,
      save_seq: 0,
      save_in_progress: false,
      last_error: 0,
    },
  };
  await page.routeWebSocket("ws://127.0.0.1:9090", (socket) => {
    connections.count++;
    socket.onMessage((raw) => {
      const message = JSON.parse(String(raw));
      if (message.op === "subscribe" && fixtures[message.topic])
        socket.send(
          JSON.stringify({
            op: "publish",
            topic: message.topic,
            msg: fixtures[message.topic],
          }),
        );
      if (message.op === "publish") {
        publishes.push(message);
        if (
          message.topic === "/motor/system_command" &&
          message.msg.command === 4
        )
          socket.send(
            JSON.stringify({
              op: "publish",
              topic: "/micro_ros_up/device_param_report",
              msg: { count: 0, entries: [] },
            }),
          );
      }
      if (message.op === "call_service") {
        const response =
          message.service === "/bench/options"
            ? {
                simulation_models: ["half_body_robot"],
                mujoco_models: ["half_body_robot"],
                serial_ports_json: "[]",
                can_interfaces_json: "[]",
              }
            : {
                ok: true,
                status_json: '{"state":"stopped"}',
                response_json: '{"ok":true,"motors":[]}',
              };
        socket.send(
          JSON.stringify({
            op: "service_response",
            id: message.id,
            service: message.service,
            result: true,
            values: response,
          }),
        );
      }
    });
  });
  return { publishes, errors, connections };
}

test("关键参数面板缺失时不建立 ROS 连接", async ({ page }) => {
  const fixture = await connectFixture(page);
  const html = readFileSync(
    new URL("../../../web/index.html", import.meta.url),
    "utf8",
  );
  await page.route("**/index.html", (route) =>
    route.fulfill({
      contentType: "text/html",
      body: html.replace(
        'data-panel-template="deviceParams"',
        'data-panel-template="missingParams"',
      ),
    }),
  );
  await page.goto("/workbench/#/run");
  await expect(page.locator("main > .panel-error")).toContainText("关键面板");
  expect(fixture.connections.count).toBe(0);
});

test("非关键观察面板失败时仍能读取设备状态", async ({ page }) => {
  const fixture = await connectFixture(page);
  const html = readFileSync(
    new URL("../../../web/index.html", import.meta.url),
    "utf8",
  );
  await page.route("**/index.html", (route) =>
    route.fulfill({
      contentType: "text/html",
      body: html.replace(
        'data-panel-template="curve"',
        'data-panel-template="missingCurve"',
      ),
    }),
  );
  await page.goto("/workbench/#/run");
  await expect(
    page.locator('[data-workbench-panel="curve"] .panel-error'),
  ).toContainText("缺少面板模板");
  await expect(page.locator("#selRows")).toContainText("0.150");
  expect(fixture.connections.count).toBe(1);
  expect(fixture.errors).toEqual([]);
});

test("A 工作台接入真实模块，切屏保留草稿且导航不产生控制消息", async ({
  page,
}) => {
  const fixture = await connectFixture(page);
  await page.goto("/workbench/#/run");
  await expect(page.locator("#selRows")).toContainText("0.150");
  await expect(page.locator("[data-workbench-panel]")).toHaveCount(13);
  await page.locator("#cmdRows .c-pos").fill("0.25");
  await page.locator(".main-nav button").filter({ hasText: "动作" }).click();
  await expect(
    page.locator('[data-workbench-panel="actionLibrary"]'),
  ).toBeVisible();
  await page.locator(".main-nav button").filter({ hasText: "仿真" }).click();
  await expect(page.locator('[data-workbench-panel="sim"]')).toBeVisible();
  await page.locator(".main-nav button").filter({ hasText: "真机" }).click();
  await expect(page.locator("#cmdRows .c-pos")).toHaveValue("0.25");
  expect(
    fixture.publishes.filter((p) => p.topic !== "/motor/system_command"),
  ).toEqual([]);
  expect(
    fixture.publishes.every(
      (p) => p.topic !== "/motor/system_command" || p.msg.command === 4,
    ),
  ).toBe(true);
  expect(fixture.errors).toEqual([]);
});

test("全局软停显式取消轨迹，并清空主机设定值", async ({ page }) => {
  const fixture = await connectFixture(page);
  await page.goto("/workbench/#/run");
  await expect(page.locator("#selRows")).toContainText("0.150");
  await page
    .getByRole("button", { name: /软停|停止运动|停止动作/ })
    .first()
    .click();
  await expect
    .poll(() =>
      fixture.publishes.some((p) => p.topic === "/motor_command/setpoint"),
    )
    .toBe(true);
  expect(
    fixture.publishes.find((p) => p.topic === "/motor/trajectory")?.msg,
  ).toMatchObject({ count: 0, target_count: 0 });
  expect(
    fixture.publishes.find((p) => p.topic === "/motor_command/setpoint")?.msg,
  ).toMatchObject({ count: 0, targets: [] });
  expect(fixture.errors).toEqual([]);
});

test("静态工作台在没有 ROS 连接时显示离线，不生成模拟设备", async ({
  page,
}) => {
  const errors: string[] = [];
  page.on("pageerror", (e) => errors.push(e.message));
  await page.route("**/api/**", (route) =>
    route.fulfill({
      json: { models: [], actions: [], joints: {}, motor_types: {} },
    }),
  );
  await page.routeWebSocket("ws://127.0.0.1:9090", (socket) => socket.close());
  await page.goto("/workbench/#/run");
  await expect(page.locator("#selRows")).toContainText("等待");
  await expect(page.locator("#selRows tr[data-id]")).toHaveCount(0);
  await expect(page.locator("#btnSendOnce")).toBeDisabled();
  for (const width of [1280, 800, 390]) {
    await page.setViewportSize({ width, height: 900 });
    expect(
      await page.evaluate(
        () => document.documentElement.scrollWidth <= innerWidth,
      ),
    ).toBe(true);
  }
  expect(errors).toEqual([]);
});
