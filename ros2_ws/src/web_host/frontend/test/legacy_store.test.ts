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

it("曲线持续接收不同频率的通道时保留各自最新的有界样本", async () => {
  const { chartPush, store } = await import("../../web/js/core/store.js");
  for (let i = 0; i < 2000; i++) {
    chartPush(1, 'position', i, i * 20);
    chartPush(1, 'velocity', i + 0.1, i * 20);
    chartPush(1, 'effort', i + 0.2, i * 20);
    if (i % 5 === 0) chartPush(1, 'current', i + 0.3, i * 20 + 1);
  }
  const channels = store.chartBuf.get(1);
  expect(channels.position.t).toHaveLength(1500);
  expect(channels.position.t[0]).toBe(10000);
  expect(channels.position.values[0]).toBe(500);
  expect(channels.position.values.at(-1)).toBe(1999);
  expect(channels.current.t).toHaveLength(400);
  expect(channels.current.t.at(-1)).toBe(39901);
  expect(channels.current.values.at(-1)).toBe(1995.3);
});

it("模型无映射时清除上一模型映射，不将上一模型设备配给新模型", async () => {
  const fetcher = vi
    .fn()
    .mockResolvedValueOnce({
      ok: true,
      json: async () => ({ model: "half_body_robot", direct_joints: { 1: "joint_a" }, groups: [] }),
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
    .mockResolvedValue({
      ok: true,
      json: async () => ({ model: store.MODEL, direct_joints: { 9: "new_joint" }, groups: [] }),
    });
  vi.stubGlobal("fetch", fetcher);
  const store = await import("../../web/js/core/store.js");
  const pending = store.loadModelJoints();
  store.setModel("new_model");
  await vi.waitFor(() =>
    expect(store.JOINT_OF_MOTOR).toEqual({ 9: "new_joint" }),
  );
  first({ ok: true, json: async () => ({ model: "half_body_robot", direct_joints: { 1: "old_joint" }, groups: [] }) });
  await pending;
  expect(store.JOINT_OF_MOTOR).toEqual({ 9: "new_joint" });
});

it("传动组 metadata 保留耦合关系，选择不被压平成 motor 对 joint", async () => {
  const groups = [
    {
      name: "wrist",
      motors: [{ name: "m8", id: 8 }, { name: "m9", id: 9 }],
      joints: ["yaw", "pitch"],
      direct_mapping: false,
    },
    {
      name: "elbow",
      motors: [{ name: "m2", id: 2 }],
      joints: ["elbow_joint"],
      direct_mapping: true,
    },
  ];
  vi.stubGlobal("fetch", vi.fn().mockResolvedValue({
    ok: true,
    json: async () => ({
      model: "half_body_robot",
      driver_file: "/tmp/motor_driver.yaml",
      groups,
      direct_joints: { 2: "elbow_joint" },
    }),
  }));
  const store = await import("../../web/js/core/store.js");
  await store.loadModelJoints();
  expect(store.JOINT_OF_MOTOR).toEqual({ 2: "elbow_joint" });
  expect(store.store.modelTransmissions).toMatchObject({
    model: "half_body_robot",
    driver_file: "/tmp/motor_driver.yaml",
    direct_mapping: { 2: "elbow_joint" },
    error: "",
  });
  expect(store.store.modelTransmissions.groups).toEqual(groups);
  store.selectGroups(["wrist"]);
  store.selectJoint("pitch");
  expect(store.store.selectedGroup).toBe("wrist");
  expect(store.store.selectedJoint).toBe("pitch");
  store.selectJoint("invented_joint");
  expect(store.store.selectedJoint).toBe("");
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
