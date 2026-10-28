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

it("收到全 NaN 参数报告不能判为读取成功或已核对", async () => {
  const { ros, resultTopic } = await connected();
  const { store, paramReadState } = await import("../../web/js/core/store.js");
  const device = await import("../../web/js/core/device.js");
  const pending = device.loadParams();
  const sent = Topic.published.find(entry => entry.topic === '/motor/system_command')!;
  resultTopic.callback?.({...result(sent.message.request_id, 1), command: 4});
  Topic.instances.find(topic => topic.name === '/motor/device_param_report')!.callback?.({count: 1,
    entries: [{motor_id: 1, dev_init_state: 0, rec_pos_min: null, rec_pos_max: null,
      rec_vel_limit: null, rec_torque_limit: null, rec_safety: null, rec_direction: null,
      dev_pos_min: null, dev_pos_max: null, dev_vel_limit: null, dev_torque_limit: null,
      dev_safety: null, dev_direction: null}]});
  await pending;
  expect(paramReadState(1)).toEqual({device: 0, record: 0, usable: 0, verified: false});
  expect(store.results.get('paramResult')).toMatchObject({ok: false});
  expect(store.results.get('paramResult').text).toContain('0/8 项');
  ros.disconnect();
});

it("手动读取发只读 REFRESH，未绑定时显示拒绝原因且不伪装成功", async () => {
  const { ros, resultTopic } = await connected();
  const { store } = await import("../../web/js/core/store.js");
  const device = await import("../../web/js/core/device.js");
  store.motors.set(1, {topo: {model_name: ''}});
  const pending = device.refreshMotorParams();
  const sent = Topic.published.find(entry => entry.topic === '/motor/system_command')!;
  expect(sent.message).toMatchObject({command: 1, motor_id: 1});
  resultTopic.callback?.({...result(sent.message.request_id, 2), command: 1, error_code: 12});
  await pending;
  expect(store.results.get('paramResult')).toMatchObject({ok: false});
  expect(store.results.get('paramResult').text).toContain('尚未识别型号或未绑定');
  expect(Topic.published).toHaveLength(1);
  ros.disconnect();
});

it("REFRESH 成功后请求参数快照，完整有效回读才显示成功", async () => {
  const { ros, resultTopic } = await connected();
  const { store, paramReadState } = await import("../../web/js/core/store.js");
  const device = await import("../../web/js/core/device.js");
  store.motors.set(1, {topo: {model_name: 'cerb_r48_101'}});
  const pending = device.refreshMotorParams();
  resultTopic.callback?.({...result(Topic.published[0].message.request_id, 1), command: 1});
  await vi.waitFor(() => expect(Topic.published).toHaveLength(2));
  expect(Topic.published[1].message.command).toBe(4);
  resultTopic.callback?.({...result(Topic.published[1].message.request_id, 1), command: 4});
  Topic.instances.find(topic => topic.name === '/motor/device_param_report')!.callback?.({count: 1,
    entries: [{motor_id: 1, dev_init_state: 2, rec_pos_min: -1, rec_pos_max: 1,
      rec_vel_limit: 2, rec_torque_limit: 3, rec_safety: 0, rec_direction: 0,
      dev_pos_min: -1, dev_pos_max: 1, dev_vel_limit: 2, dev_torque_limit: 3,
      dev_safety: 0, dev_direction: 0, rec_t_max: 300, dev_t_max: 300, rec_node_id: 1, dev_node_id: 1}]});
  await pending;
  expect(paramReadState(1).verified).toBe(true);
  expect(store.results.get('paramResult')).toMatchObject({ok: true});
  expect(Topic.published.every(entry => entry.topic === '/motor/system_command')).toBe(true);
  ros.disconnect();
});

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
