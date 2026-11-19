import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";

beforeEach(() => {
  vi.resetModules();
  vi.stubGlobal("location", { search: "" });
});
afterEach(() => {
  vi.unstubAllGlobals();
});

async function loadCurve() {
  return import("../../web/js/panels/curve.js");
}

describe("曲线面板纯逻辑", () => {
  it("默认只选 1 号电机和位置通道", async () => {
    const curve = await loadCurve();
    expect(curve.DEFAULT_CHART_MOTOR_IDS).toEqual([1]);
    expect(curve.DEFAULT_CHART_WINDOW_MS).toBe(10_000);
    expect(curve.CHART_CHANNELS.filter((channel: { key: string }) => channel.key === "position"))
      .toMatchObject([{ key: "position", label: "位置", unit: "rad" }]);
  });

  it("滚轮按 1 秒步进并限制在 1 到 60 秒", async () => {
    const curve = await loadCurve();
    expect(curve.nextChartWindowMs(10_000, -1)).toBe(9_000);
    expect(curve.nextChartWindowMs(10_000, 1)).toBe(11_000);
    expect(curve.nextChartWindowMs(1_000, -1)).toBe(1_000);
    expect(curve.nextChartWindowMs(60_000, 1)).toBe(60_000);
    expect(curve.clampChartWindowMs(Number.NaN)).toBe(10_000);
    expect(curve.clampChartWindowMs(600)).toBe(1_000);
    expect(curve.clampChartWindowMs(61_500)).toBe(60_000);
  });

  it("刷新拓扑时保留已选电机，并始终提供默认 1 号电机", async () => {
    const curve = await loadCurve();
    expect(curve.mergeMotorChoices([3], new Set([2]), [1])).toEqual([1, 2, 3]);
    expect(curve.mergeMotorChoices([], new Set([4]), [1])).toEqual([1, 4]);
  });

  it("按电机乘通道生成组合序列，图例文字包含单位", async () => {
    const curve = await loadCurve();
    const buf = new Map([
      [1, { position: { t: [1, 2], values: [0.1, 0.2] }, velocity: { t: [1], values: [2] } }],
      [2, { position: { t: [1], values: [0.3] } }],
    ]);
    const series = curve.collectChartSeries(buf, [2, 1], ["velocity", "position"]);
    expect(series.map((item: { id: number; channel: string }) => `${item.id}:${item.channel}`))
      .toEqual(["1:position", "1:velocity", "2:position", "2:velocity"]);
    expect(series[1].bucket.values).toEqual([2]);
    expect(series[3].bucket).toBeNull();
    expect(curve.seriesLegendText(series[0])).toContain("位置 (rad)");
    expect(curve.seriesLegendText(series[1])).toContain("速度 (rad/s)");
  });
});
