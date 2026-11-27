import { expect, test } from "@playwright/test";

test("仿真与电机后端控件共享选项并双向同步", async ({ page }) => {
  const errors: string[] = [];
  page.on("pageerror", error => errors.push(error.message));
  await page.route("**/api/**", route => route.fulfill({ json:
    new URL(route.request().url()).pathname === "/api/runtime"
      ? { control_mode: "bench", robot_model: "half_body_robot" }
      : { groups: [], models: ["half_body_robot"] },
  }));
  await page.routeWebSocket("ws://127.0.0.1:9090", socket => {
    socket.onMessage(raw => {
      const message = JSON.parse(String(raw));
      if (message.op !== "call_service") return;
      const values = message.service === "/bench/options" ? {
        can_interfaces_json: JSON.stringify([
          { name: "can0", up: true }, { name: "can1", up: true },
          { name: "can2", up: false },
        ]),
        serial_ports_json: JSON.stringify([
          { path: "/dev/example0", label: "模拟器 0", accessible: true },
          { path: "/dev/example1", label: "模拟器 1", accessible: true },
          { path: "/dev/example2", label: "不可访问", accessible: false },
        ]),
        simulation_models: ["half_body_robot"],
        mujoco_models: ["half_body_robot"],
      } : { ok: true, status_json: "{}", response_json: '{"ok":true,"motors":[]}' };
      socket.send(JSON.stringify({
        op: "service_response", id: message.id, service: message.service,
        result: true, values,
      }));
    });
  });
  await page.goto("/workbench/#/sim");
  await expect(page.locator("#simModel")).toHaveValue("half_body_robot");

  const transport = page.locator("#simTransport");
  const backendTransport = page.locator("#motorBackendTransport");
  await backendTransport.selectOption("socketcan");
  await expect(transport).toHaveValue("socketcan");

  for (const [primary, mirror, first, second] of [
    ["simInterface", "motorBackendInterface", "can0", "can1"],
    ["simTty", "motorBackendTty", "/dev/example0", "/dev/example1"],
  ]) {
    if (primary === "simTty") {
      await transport.selectOption("slcan");
      await expect(backendTransport).toHaveValue("slcan");
    }
    const source = page.locator(`#${primary}`);
    const target = page.locator(`#${mirror}`);
    await expect(source.locator("option")).toHaveCount(4);
    await expect(target.locator("option")).toHaveCount(4);
    expect(await target.locator("option").evaluateAll(options => options.map(option => ({
      value: (option as HTMLOptionElement).value,
      disabled: (option as HTMLOptionElement).disabled,
    })))).toEqual(await source.locator("option").evaluateAll(options => options.map(option => ({
      value: (option as HTMLOptionElement).value,
      disabled: (option as HTMLOptionElement).disabled,
    }))));
    await target.selectOption(first!);
    await expect(source).toHaveValue(first!);
    await source.selectOption(second!);
    await expect(target).toHaveValue(second!);
  }
  expect(errors).toEqual([]);
});
