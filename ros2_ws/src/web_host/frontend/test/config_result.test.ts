import { afterEach, beforeEach, expect, it, vi } from "vitest";

class Socket {
  static instances: Socket[] = [];
  handlers = new Map<string, (() => void)[]>();
  constructor() { Socket.instances.push(this); }
  on(name: string, callback: () => void) {
    this.handlers.set(name, [...(this.handlers.get(name) || []), callback]);
  }
  emit(name: string) { for (const callback of this.handlers.get(name) || []) callback(); }
  close() {}
}

class Topic {
  static instances: Topic[] = [];
  static published: { topic: string; message: any }[] = [];
  name: string;
  options: { name: string; [key: string]: unknown };
  callback?: (message: any) => void;
  constructor(options: { name: string; [key: string]: unknown }) {
    this.name = options.name;
    this.options = options;
    Topic.instances.push(this);
  }
  subscribe(callback: (message: any) => void) { this.callback = callback; }
  publish(message: any) { Topic.published.push({ topic: this.name, message }); }
  unsubscribe() {}
  advertise() {}
  unadvertise() {}
}

beforeEach(() => {
  vi.resetModules();
  Socket.instances = [];
  Topic.instances = [];
  Topic.published = [];
  vi.stubGlobal("location", { search: "" });
  vi.stubGlobal("localStorage", { getItem: () => null });
  vi.stubGlobal("document", {
    getElementById: () => ({ value: "ws://example.test:9090" }),
    body: { classList: { toggle() {} } },
  });
  vi.stubGlobal("ROSLIB", {
    Ros: Socket,
    Topic,
    Message: class { constructor(value: object) { Object.assign(this, value); } },
  });
});

afterEach(() => {
  vi.useRealTimers();
  vi.unstubAllGlobals();
  vi.restoreAllMocks();
});

async function connected() {
  const ros = await import("../../web/js/core/ros.js");
  ros.connect();
  Socket.instances[0].emit("connection");
  const resultTopic = Topic.instances.find((topic) => topic.name === "/motor/config_result");
  expect(resultTopic).toBeDefined();
  return { ros, resultTopic: resultTopic! };
}

const result = (request_id: number, status: number, session_id = 9) => ({
  session_id, request_id, kind: 1, command: 0, status,
  error_code: 0, completed_targets: 1, failed_targets: 0,
});
const controlSnapshot = (session_id: number, sequence: number, changes = {}) => ({
  schema_version: 1, session_id, sequence, states: [], ...changes,
});
const realisticSessionId = 1792828800123;

it("PENDING 不会解锁等待，只有同 request 的终态成功", async () => {
  const { ros, resultTopic } = await connected();
  const pending = ros.waitConfigResult(42, 1000);
  resultTopic.callback?.(result(42, 0));
  await Promise.resolve();
  resultTopic.callback?.(result(42, 1));
  await expect(pending).resolves.toMatchObject({ outcome: "terminal", result: { status: 1 } });
  ros.disconnect();
});

it("其他 request 的终态不能解锁等待", async () => {
  const { ros, resultTopic } = await connected();
  const pending = ros.waitConfigResult(42, 1000);
  resultTopic.callback?.(result(43, 1));
  await Promise.resolve();
  resultTopic.callback?.(result(42, 1));
  await expect(pending).resolves.toMatchObject({ result: { request_id: 42 } });
  ros.disconnect();
});

it("失败结果返回错误码与失败数量", async () => {
  const { ros, resultTopic } = await connected();
  const pending = ros.waitConfigResult(42, 1000);
  resultTopic.callback?.({ ...result(42, 2), error_code: 10, failed_targets: 2 });
  await expect(pending).resolves.toMatchObject({
    outcome: "terminal", result: { status: 2, error_code: 10, failed_targets: 2 },
  });
  ros.disconnect();
});

it("超时明确返回 timeout，不代表设备回滚", async () => {
  vi.useFakeTimers();
  const { ros } = await connected();
  const pending = ros.waitConfigResult(42, 250);
  await vi.advanceTimersByTimeAsync(250);
  await expect(pending).resolves.toEqual({ outcome: "timeout", request_id: 42 });
  ros.disconnect();
});

it("断连使旧等待失效", async () => {
  const { ros } = await connected();
  const pending = ros.waitConfigResult(42, 1000);
  ros.disconnect();
  await expect(pending).resolves.toEqual({ outcome: "disconnected", request_id: 42 });
});

it("连接代次变化使旧等待失效", async () => {
  const { ros } = await connected();
  const pending = ros.waitConfigResult(42, 1000);
  ros.connect();
  await expect(pending).resolves.toEqual({ outcome: "disconnected", request_id: 42 });
  ros.disconnect();
});

it("session_id 改变清空缓存并使旧等待失效", async () => {
  const { ros, resultTopic } = await connected();
  resultTopic.callback?.(result(1, 1, 9));
  const { store } = await import("../../web/js/core/store.js");
  const pending = ros.waitConfigResult(42, 1000);
  resultTopic.callback?.(result(50, 1, 10));
  await expect(pending).resolves.toEqual({ outcome: "session_changed", request_id: 42 });
  expect(store.configResults.size).toBe(1);
  expect(store.configResults.has(1)).toBe(false);
  ros.disconnect();
});

it("结果缓存最多保留最近 64 个 request", async () => {
  const { ros, resultTopic } = await connected();
  const { store } = await import("../../web/js/core/store.js");
  for (let request_id = 1; request_id <= 70; request_id++) {
    resultTopic.callback?.(result(request_id, 1));
  }
  expect(store.configResults.size).toBe(64);
  expect(store.configResults.has(1)).toBe(false);
  expect(store.configResults.has(70)).toBe(true);
  ros.disconnect();
});

it("本次 SAVE ConfigResult 成功时忽略滞后的 save_in_progress", async () => {
  const { ros, resultTopic } = await connected();
  const { store } = await import("../../web/js/core/store.js");
  const device = await import("../../web/js/core/device.js");
  store.sysStatus = {
    refresh_seq: 0, set_zero_seq: 0, clear_fault_seq: 0,
    save_seq: 7, save_in_progress: true, seen: true,
  };
  const saving = device.saveParamsToFlash();
  const sent = Topic.published.filter((entry) => entry.topic === "/motor/system_command");
  expect(sent).toHaveLength(1);
  resultTopic.callback?.(result(sent[0].message.request_id, 1));
  await saving;
  expect(store.results.get("paramResult")).toMatchObject({ ok: true });
  expect(Topic.published.filter((entry) => entry.topic === "/motor/system_command")).toHaveLength(1);
  ros.disconnect();
}, 8000);

it("同 session 的迟到 PENDING 不覆盖 request 已缓存的终态", async () => {
  const { ros, resultTopic } = await connected();
  const { store } = await import("../../web/js/core/store.js");
  resultTopic.callback?.(result(42, 1));
  resultTopic.callback?.(result(42, 0));
  expect(store.configResults.get(42)?.status).toBe(1);
  await expect(ros.waitConfigResult(42, 1000)).resolves.toMatchObject({
    outcome: "terminal", result: { request_id: 42, status: 1 },
  });
  ros.disconnect();
});

it("有效控制快照切换 session 时立即使旧 ConfigResult waiter 失效", async () => {
  const { ros } = await connected();
  const controlTopic = Topic.instances.find((topic) => topic.name === "/motor/control_states")!;
  controlTopic.callback?.(controlSnapshot(realisticSessionId, 1));
  const pending = ros.waitConfigResult(42, 1000);
  controlTopic.callback?.(controlSnapshot(realisticSessionId + 1, 1));
  await expect(pending).resolves.toEqual({ outcome: "session_changed", request_id: 42 });
  ros.disconnect();
});

it("首次控制 session 只采纳，并由该 session 的 ConfigResult 正常结束首次等待", async () => {
  const { ros, resultTopic } = await connected();
  const { store } = await import("../../web/js/core/store.js");
  const controlTopic = Topic.instances.find((topic) => topic.name === "/motor/control_states")!;
  const pending = ros.waitConfigResult(42, 1000);
  controlTopic.callback?.(controlSnapshot(realisticSessionId, 1));
  expect(store.configSessionId).toBe(String(realisticSessionId));
  let settled = false;
  void pending.then(() => { settled = true; });
  await Promise.resolve();
  expect(settled).toBe(false);
  resultTopic.callback?.(result(42, 1, realisticSessionId));
  await expect(pending).resolves.toMatchObject({ outcome: "terminal", result: { status: 1 } });
  ros.disconnect();
});

it("坏帧及旧控制序号不清缓存或终止 ConfigResult waiter", async () => {
  const { ros, resultTopic } = await connected();
  const { store } = await import("../../web/js/core/store.js");
  resultTopic.callback?.(result(1, 1, realisticSessionId));
  const controlTopic = Topic.instances.find((topic) => topic.name === "/motor/control_states")!;
  controlTopic.callback?.(controlSnapshot(realisticSessionId, 5));
  const pending = ros.waitConfigResult(42, 1000);
  let settled = false;
  void pending.then(() => { settled = true; });
  controlTopic.callback?.(controlSnapshot(realisticSessionId + 1, 1, { schema_version: 2 }));
  controlTopic.callback?.(controlSnapshot(realisticSessionId, 4));
  controlTopic.callback?.(controlSnapshot(realisticSessionId - 1, 10));
  await Promise.resolve();
  expect(settled).toBe(false);
  expect(store.configResults.has(1)).toBe(true);
  controlTopic.callback?.(controlSnapshot(realisticSessionId + 1, 1));
  await expect(pending).resolves.toEqual({ outcome: "session_changed", request_id: 42 });
  expect(store.configResults.size).toBe(0);
  ros.disconnect();
});
