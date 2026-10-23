import { afterEach, beforeEach, expect, it, vi } from "vitest";

beforeEach(() => {
  vi.resetModules();
  vi.stubGlobal("location", { search: "" });
  vi.stubGlobal("localStorage", { getItem: () => null, setItem() {} });
  vi.spyOn(console, "warn").mockImplementation(() => {});
});
afterEach(() => {
  vi.unstubAllGlobals();
  vi.restoreAllMocks();
});

it("模型无映射时清除上一模型映射，不将上一模型设备配给新模型", async () => {
  const fetcher = vi
    .fn()
    .mockResolvedValueOnce({
      ok: true,
      json: async () => ({ joints: { 1: "joint_a" } }),
    })
    .mockResolvedValueOnce({ ok: false, status: 404 });
  vi.stubGlobal("fetch", fetcher);
  const store = await import("../../web/js/core/store.js");
  await store.loadModelJoints();
  expect(store.JOINT_OF_MOTOR).toEqual({ 1: "joint_a" });
  store.setModel("preview_only");
  await vi.waitFor(() => expect(store.jointMapSource.kind).toBe("unavailable"));
  expect(store.JOINT_OF_MOTOR).toEqual({});
});

it("晚到的旧模型响应不覆盖当前模型映射", async () => {
  let first!: (response: unknown) => void;
  const fetcher = vi
    .fn()
    .mockImplementationOnce(() => new Promise((resolve) => (first = resolve)))
    .mockResolvedValueOnce({
      ok: true,
      json: async () => ({ joints: { 9: "new_joint" } }),
    });
  vi.stubGlobal("fetch", fetcher);
  const store = await import("../../web/js/core/store.js");
  const pending = store.loadModelJoints();
  store.setModel("new_model");
  await vi.waitFor(() =>
    expect(store.JOINT_OF_MOTOR).toEqual({ 9: "new_joint" }),
  );
  first({ ok: true, json: async () => ({ joints: { 1: "old_joint" } }) });
  await pending;
  expect(store.JOINT_OF_MOTOR).toEqual({ 9: "new_joint" });
});

it("停止渲染循环会取消待执行帧，卸载后不再重新排队", async () => {
  let next = 0;
  const frames = new Map<number, () => void>();
  vi.stubGlobal("requestAnimationFrame", (callback: () => void) => {
    frames.set(++next, callback);
    return next;
  });
  vi.stubGlobal("cancelAnimationFrame", (id: number) => frames.delete(id));
  const store = await import("../../web/js/core/store.js");
  const stop = store.runRenderLoop();
  expect(frames.size).toBe(1);
  expect(typeof stop).toBe("function");
  stop();
  expect(frames.size).toBe(0);
});
