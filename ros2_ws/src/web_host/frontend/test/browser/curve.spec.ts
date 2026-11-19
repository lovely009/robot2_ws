import { expect, test, type Page } from "@playwright/test";

async function openWorkbenchCurve(page: Page) {
  await page.route("**/api/**", (route) => route.fulfill({ json: new URL(route.request().url()).pathname === '/api/runtime'
    ? {control_mode:'readonly',robot_model:''} : {} }));
  await page.routeWebSocket("ws://127.0.0.1:9090", (socket) => {
    socket.onMessage(() => {});
  });
  await page.goto("/workbench/#/run");
  await expect(page.locator("#chart")).toBeVisible();
  await expect.poll(() => page.evaluate(async () =>
    (await import('/js/core/store.js')).store.conn.connected)).toBe(true);
}

async function injectCurveData(page: Page, motorIds = [1, 2]) {
  await page.evaluate(async (ids) => {
    const { store, markDirty, DIRTY } = await import("/js/core/store.js");
    const now = Date.now();
    for (const id of ids) store.motors.set(id, {});
    for (const id of ids) {
      store.chartBuf.set(id, {
        position: { t: [now - 9000, now - 5000, now - 1000],
          values: [id * 0.1, id * 0.2, id * 0.3] },
        velocity: { t: [now - 8000, now - 4000, now - 500],
          values: [id * 0.5, id * 0.7, id * 0.9] },
        effort: { t: [now - 3000], values: [id * 0.05] },
        current: { t: [now - 2500], values: [id * 0.02] },
      });
    }
    markDirty(DIRTY.TOPOLOGY, DIRTY.MOTION, DIRTY.STATUS);
  }, motorIds);
  await expect(page.locator('input[name="chart-motor"]')).toHaveCount(motorIds.length);
  await expect(page.locator("#chartLegend .curve-legend-item")).toHaveCount(1);
}

async function chartWindowMs(page: Page) {
  return page.evaluate(async () => {
    const { store } = await import("/js/core/store.js");
    return store.chart.windowMs;
  });
}

test("工作台曲线默认值、多选组合和图例单位", async ({ page }) => {
  await openWorkbenchCurve(page);
  await expect(page.locator("#chartWindow")).toHaveCount(0);
  await expect(page.locator(".curve-controls select")).toHaveCount(0);
  await expect(page.locator("[data-chart-motor-summary]")).toHaveText("电机 1");
  await expect(page.locator("[data-chart-channel-summary]")).toHaveText("位置 rad");
  await expect(page.locator('input[name="chart-motor"][value="1"]')).toBeChecked();
  await expect(page.locator('input[name="chart-channel"][value="position"]')).toBeChecked();
  expect(await chartWindowMs(page)).toBe(10_000);

  await injectCurveData(page);
  const motorMenu = page.locator("#chartMotor");
  await motorMenu.locator("summary").click();
  await expect(motorMenu).toHaveJSProperty("open", true);
  await page.evaluate(async () => {
    const { store, markDirty, DIRTY } = await import("/js/core/store.js");
    store.motors.set(3, {});
    markDirty(DIRTY.TOPOLOGY, DIRTY.MOTION);
  });
  await expect(motorMenu).toHaveJSProperty("open", true);
  await expect(page.locator('input[name="chart-motor"][value="1"]')).toBeChecked();
  await page.locator('input[name="chart-motor"][value="2"]').check();

  const channelMenu = page.locator("#chartCh");
  await channelMenu.locator("summary").click();
  await expect(channelMenu).toHaveJSProperty("open", true);
  await page.locator('input[name="chart-channel"][value="velocity"]').check();
  await page.evaluate(async () => {
    const { markDirty, DIRTY } = await import("/js/core/store.js");
    markDirty(DIRTY.TOPOLOGY, DIRTY.MOTION, DIRTY.STATUS);
  });
  await expect(channelMenu).toHaveJSProperty("open", true);
  await expect(page.locator('input[name="chart-channel"][value="position"]')).toBeChecked();
  await expect(page.locator('input[name="chart-channel"][value="velocity"]')).toBeChecked();
  await expect(page.locator("[data-chart-channel-summary]")).toContainText("位置 + 速度");

  await expect(page.locator("#chartLegend .curve-legend-item")).toHaveCount(4);
  await expect(page.locator("#chartLegend")).toContainText("电机 1 · 位置");
  await expect(page.locator("#chartLegend")).toContainText("电机 2 · 速度");
  await expect(page.locator("#chartLegend .curve-unit")).toContainText(["rad", "rad/s", "rad", "rad/s"]);
});

test("canvas 滚轮改变窗口，切屏恢复后不重复注册监听", async ({ page }) => {
  await openWorkbenchCurve(page);
  await page.evaluate(() => {
    const canvas = document.querySelector<HTMLCanvasElement>("#chart");
    canvas?.dispatchEvent(new WheelEvent("wheel", { deltaY: -100, cancelable: true }));
  });
  await expect.poll(() => chartWindowMs(page)).toBe(9_000);
  await expect(page.locator("#chartHint")).toContainText("9 s");

  await page.evaluate(() => { location.hash = "#/teach"; });
  await expect(page.locator("#chart")).toBeHidden();
  await page.evaluate(() => { location.hash = "#/run"; });
  await expect(page.locator("#chart")).toBeVisible();
  expect(await chartWindowMs(page)).toBe(9_000);

  await page.evaluate(() => {
    const canvas = document.querySelector<HTMLCanvasElement>("#chart");
    canvas?.dispatchEvent(new WheelEvent("wheel", { deltaY: -100, cancelable: true }));
  });
  await expect.poll(() => chartWindowMs(page)).toBe(8_000);
});
