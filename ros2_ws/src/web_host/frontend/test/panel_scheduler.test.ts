import { describe, it, expect } from "vitest";
import { PanelScheduler } from "../src/panel_scheduler";

describe("面板刷新生命周期", () => {
  it("渲染暂时失败后保留一次性更新并自动有限重试", () => {
    let attempts = 0;
    const rendered: string[][] = [];
    const scheduler = new PanelScheduler(() => {});
    scheduler.add(
      "params",
      {
        render(dirty) {
          if (++attempts === 1) throw Error("transient");
          rendered.push([...dirty]);
        },
      },
      () => true,
    );
    scheduler.invalidate(new Set(["params"]));
    scheduler.flush(0);
    scheduler.flush(250);
    expect(rendered).toEqual([["params"]]);
  });
  it("持续失败不会在每个动画帧无限重试", () => {
    let attempts = 0;
    const scheduler = new PanelScheduler(() => {});
    scheduler.add(
      "broken",
      {
        render() {
          attempts++;
          throw Error("broken");
        },
      },
      () => true,
    );
    scheduler.invalidate(new Set(["params"]));
    for (const at of [0, 250, 500, 750, 1000]) scheduler.flush(at);
    expect(attempts).toBe(3);
    scheduler.invalidate(new Set(["motion"]));
    scheduler.flush(1250);
    scheduler.invalidate(new Set(["events"]));
    scheduler.flush(1500);
    expect(attempts).toBe(3);
    scheduler.retryFailed();
    scheduler.flush(1750);
    expect(attempts).toBe(4);
  });
  it("节流期间的一次参数回执在节流结束后仍刷新", () => {
    const rendered: string[][] = [];
    const scheduler = new PanelScheduler(() => {});
    scheduler.add(
      "params",
      { intervalMs: 200, render: (d) => rendered.push([...d]), unmount() {} },
      () => true,
    );
    scheduler.invalidate(new Set(["status"]));
    scheduler.flush(0);
    scheduler.invalidate(new Set(["params"]));
    scheduler.flush(50);
    expect(rendered).toEqual([["status"]]);
    scheduler.flush(200);
    expect(rendered).toEqual([["status"], ["params"]]);
  });
  it("隐藏页保留所有脏标记，显示后一起刷新", () => {
    let visible = false;
    const rendered: string[][] = [];
    const scheduler = new PanelScheduler(() => {});
    scheduler.add(
      "action",
      { render: (d) => rendered.push([...d]), unmount() {} },
      () => visible,
    );
    scheduler.invalidate(new Set(["trajectory"]));
    scheduler.flush(0);
    scheduler.invalidate(new Set(["events"]));
    scheduler.flush(20);
    expect(rendered).toEqual([]);
    visible = true;
    scheduler.flush(30);
    expect(rendered).toEqual([["trajectory", "events"]]);
  });
  it("某面板抛错不阻塞其余面板", () => {
    const errors: string[] = [],
      rendered: number[] = [];
    const scheduler = new PanelScheduler((id) => errors.push(id));
    scheduler.add(
      "broken",
      {
        render() {
          throw Error("bad");
        },
        unmount() {},
      },
      () => true,
    );
    scheduler.add(
      "healthy",
      {
        render() {
          rendered.push(1);
        },
        unmount() {},
      },
      () => true,
    );
    scheduler.invalidate(new Set(["status"]));
    scheduler.flush(0);
    expect(errors).toEqual(["broken"]);
    expect(rendered).toEqual([1]);
  });
  it("卸载幂等并禁止后续渲染", () => {
    let rendered = 0,
      disposed = 0;
    const scheduler = new PanelScheduler(() => {});
    const release = scheduler.add(
      "one",
      {
        render() {
          rendered++;
        },
        unmount() {
          disposed++;
        },
      },
      () => true,
    );
    scheduler.invalidate(new Set(["status"]));
    release();
    release();
    scheduler.flush(0);
    scheduler.dispose();
    expect(rendered).toBe(0);
    expect(disposed).toBe(1);
  });
});
