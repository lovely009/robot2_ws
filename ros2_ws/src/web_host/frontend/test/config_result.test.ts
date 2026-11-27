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

class Service {
  static calls: { name: string; request: any; complete: (response: any) => void; fail: (err: any) => void }[] = [];
  name: string;
  constructor(options: { name: string }) { this.name = options.name; }
  callService(request: any, complete: (response: any) => void, fail: (err: any) => void) {
    Service.calls.push({ name: this.name, request, complete, fail });
  }
}

beforeEach(() => {
  vi.resetModules();
  Socket.instances = [];
  Topic.instances = [];
  Topic.published = [];
  Service.calls = [];
  vi.stubGlobal("location", { search: "" });
  vi.stubGlobal("localStorage", { getItem: () => null });
  vi.stubGlobal("document", {
    getElementById: () => ({ value: "ws://example.test:9090" }),
    body: { classList: { toggle() {} } },
  });
  vi.stubGlobal("ROSLIB", {
    Ros: Socket,
    Topic,
    Service,
    Message: class { constructor(value: object) { Object.assign(this, value); } },
  });
});

afterEach(() => {
  vi.useRealTimers();
  vi.unstubAllGlobals();
  vi.restoreAllMocks();
});

async function connected() {
  const ros = await import("../src/core/ros.ts");
  ros.configureControl({ control_mode: "bench" });
  ros.connect();
  Socket.instances[0].emit("connection");
  const { store } = await import("../src/core/store.ts");
  store.controlStates.sourceTimeNs = 9000000000;
  const stateTopic = Topic.instances.find((topic) => topic.name === "/motor/state")!;
  expect(stateTopic).toBeDefined();
  return { ros, stateTopic };
}

const result = (request_id: number, status: number, session_id = 9) => ({
  session_id, request_id, kind: 1, command: 0, status,
  error_code: 0, completed_targets: 1, failed_targets: 0,
});

it("executeConfig 调用 /motor/execute_config 并返回 ConfigResult", async () => {
  const { ros } = await connected();
  const pending = ros.executeConfig({ kind: 1, command: 4, motor_id: 65535, request_id: 42 }, 1000);
  const call = Service.calls.at(-1)!;
  expect(call.name).toBe("/motor/execute_config");
  expect(call.request.timeout_ms).toBe(1000);
  call.complete({ result: result(42, 1) });
  await expect(pending).resolves.toMatchObject({ status: 1, request_id: 42 });
  ros.disconnect();
});

it("已移除的参数写入与旧话题发布不创建下行请求", async () => {
  const { ros } = await connected();
  const before = Topic.published.length;
  expect(ros.pub("paramSet", { count: 1, targets: [{ motor_id: 1, field: 0, value: 1 }] })).toBe(false);
  expect(ros.pub("firmwareConfig", { count: 1, targets: [{ motor_id: 65535, field: 0, value: 50 }] })).toBe(false);
  expect(ros.pub("sysCmd", { command: 0, motor_id: 1 })).toBe(false);
  expect(Topic.published).toHaveLength(before);
  ros.disconnect();
});

function zeroReady(store: any, id = 2) {
  store.controlStates = {
    seen: true, valid: true, at: Date.now(), sourceTimeNs: 9000000000, session: 9, sequence: 1,
    byId: new Map([[id, {
      motor_id: id, online: true, control_lifecycle: 2, execution_state: 1,
      policy_enabled: false, motion_enabled: false, state_age_ms: 5, unified_fault_code: 0,
      vendor_fault_code: 0, last_failure_reason: 0, policy_generation: 1,
    }]]),
  };
}

it("设零只发所选电机一次，并等待对应服务终态，不恢复使能或目标", async () => {
  const { ros } = await connected();
  const { store } = await import("../src/core/store.ts");
  const device = await import("../src/core/device.ts");
  vi.stubGlobal("window", { confirm: vi.fn(() => true) });
  zeroReady(store, 64);
  const pending = device.setMotorZero(64);
  expect(Service.calls).toHaveLength(1);
  const call = Service.calls[0]!;
  expect(call.name).toBe("/motor/execute_config");
  expect(call.request.request).toMatchObject({ kind: 1, command: 2, motor_id: 64 });
  await device.setMotorZero(64);
  expect(Service.calls).toHaveLength(1);
  call.complete({ result: { ...result(call.request.request.request_id, 1), command: 2 } });
  await pending;
  expect(store.results.get("paramResult")).toMatchObject({ ok: true });
  expect(store.results.get("paramResult").text).toContain("设备已确认");
  expect(Topic.published).toHaveLength(0);
  ros.disconnect();
});

it.each(["enabled", "motion", "offline", "stale", "fault", "vendor_fault", "transition", "host_active", "host_pending", "readonly", "invalid", "canceled"])("设零拒绝 %s，不下发请求", async (change) => {
  const { ros } = await connected();
  const { store } = await import("../src/core/store.ts");
  const device = await import("../src/core/device.ts");
  vi.stubGlobal("window", { confirm: vi.fn(() => change !== "canceled") });
  zeroReady(store);
  const state = store.controlStates.byId.get(2)!;
  if (change === "enabled") state.policy_enabled = true;
  if (change === "motion") state.motion_enabled = true;
  if (change === "offline") state.online = false;
  if (change === "stale") store.controlStates.at = Date.now() - 3000;
  if (change === "fault") state.unified_fault_code = 5;
  if (change === "vendor_fault") state.vendor_fault_code = 257;
  if (change === "transition") state.execution_state = 6;
  if (change === "host_active") store.commandStatus.data = { mode: "trajectory", active_controllers: ["trajectory_controller"] };
  if (change === "host_pending") store.commandStatus.data = { mode: "stopped", pending_goal_ids: [1] };
  if (change === "readonly") ros.configureControl({ control_mode: "readonly" });
  await device.setMotorZero(change === "invalid" ? 0 : 2);
  expect(Service.calls).toHaveLength(0);
  ros.disconnect();
});

it.each([10, 5])("设零失败显示设备错误 %s，不误报成功", async (error_code) => {
  const { ros } = await connected();
  const { store } = await import("../src/core/store.ts");
  const device = await import("../src/core/device.ts");
  vi.stubGlobal("window", { confirm: () => true });
  zeroReady(store);
  const pending = device.setMotorZero(2);
  const call = Service.calls[0]!;
  call.complete({ result: { ...result(call.request.request.request_id, 2), command: 2, error_code, failed_targets: 1 } });
  await pending;
  expect(store.results.get("paramResult")).toMatchObject({ ok: false });
  expect(store.results.get("paramResult").text).toContain(`error_code=${error_code}`);
  if (error_code === 5) expect(store.results.get("paramResult").text).toContain("设备零位可能已改变");
  ros.disconnect();
});

it("收到全 NaN 参数报告不能判为读取成功或已核对", async () => {
  const { ros } = await connected();
  const { store, paramReadState } = await import("../src/core/store.ts");
  const device = await import("../src/core/device.ts");
  const pending = device.loadParams();
  const call = Service.calls[0]!;
  expect(call.name).toBe("/motor/get_device_info");
  call.complete({
    info: {
      params: [{
        motor_id: 1, dev_init_state: 0, rec_pos_min: null, rec_pos_max: null,
        rec_vel_limit: null, rec_torque_limit: null, rec_safety: null, rec_direction: null,
        dev_pos_min: null, dev_pos_max: null, dev_vel_limit: null, dev_torque_limit: null,
        dev_safety: null, dev_direction: null,
      }],
    },
  });
  await pending;
  expect(paramReadState(1)).toEqual({ device: 0, record: 0, usable: 0, verified: false });
  expect(store.results.get("paramResult")).toMatchObject({ ok: false });
  expect(store.results.get("paramResult").text).toContain("0/8 项");
  ros.disconnect();
});

it("手动读取发只读 REFRESH，未绑定时显示拒绝原因且不伪装成功", async () => {
  const { ros } = await connected();
  const { store } = await import("../src/core/store.ts");
  const device = await import("../src/core/device.ts");
  store.motors.set(1, { topo: { model_name: "" } });
  const pending = device.refreshMotorParams();
  const call = Service.calls[0]!;
  expect(call.request.request).toMatchObject({ kind: 1, command: 1, motor_id: 1 });
  call.complete({ result: { ...result(call.request.request.request_id, 2), command: 1, error_code: 12 } });
  await pending;
  expect(store.results.get("paramResult")).toMatchObject({ ok: false });
  expect(store.results.get("paramResult").text).toContain("尚未识别型号或未绑定");
  expect(Service.calls).toHaveLength(1);
  ros.disconnect();
});

it("REFRESH 成功后请求参数快照，完整有效回读才显示成功", async () => {
  const { ros } = await connected();
  const { store, paramReadState } = await import("../src/core/store.ts");
  const device = await import("../src/core/device.ts");
  store.motors.set(1, { topo: { model_name: "cerb_r48_101" } });
  const pending = device.refreshMotorParams();
  expect(Service.calls[0]!.name).toBe("/motor/execute_config");
  Service.calls[0]!.complete({ result: { ...result(Service.calls[0]!.request.request.request_id, 1), command: 1 } });
  await vi.waitFor(() => expect(Service.calls).toHaveLength(2));
  expect(Service.calls[1]!.name).toBe("/motor/get_device_info");
  Service.calls[1]!.complete({
    info: {
      params: [{
        motor_id: 1, dev_init_state: 2, rec_pos_min: -1, rec_pos_max: 1,
        rec_vel_limit: 2, rec_torque_limit: 3, rec_safety: 0, rec_direction: 0,
        dev_pos_min: -1, dev_pos_max: 1, dev_vel_limit: 2, dev_torque_limit: 3,
        dev_safety: 0, dev_direction: 0, rec_t_max: 300, dev_t_max: 300, rec_node_id: 1, dev_node_id: 1,
      }],
    },
  });
  await pending;
  expect(paramReadState(1).verified).toBe(true);
  expect(store.results.get("paramResult")).toMatchObject({ ok: true });
  ros.disconnect();
});

it("浏览器时钟偏移时，系统维护写只携带缓存MCU同步源时间", async () => {
  vi.useFakeTimers();
  vi.setSystemTime(777000);
  const { ros } = await connected();
  const { store } = await import("../src/core/store.ts");
  const device = await import("../src/core/device.ts");
  zeroReady(store);
  vi.stubGlobal("window", { confirm: () => true });
  const p1 = device.setMotorZero(2);
  Service.calls.at(-1)!.complete({ result: result(Service.calls.at(-1)!.request.request.request_id, 1) });
  await p1;
  const p2 = device.clearMotorFault(2);
  Service.calls.at(-1)!.complete({ result: result(Service.calls.at(-1)!.request.request.request_id, 2) });
  await p2;
  expect(Service.calls).toHaveLength(2);
  expect(Service.calls.every((entry) => entry.request.request.header.stamp.sec === 9 && entry.request.request.header.stamp.nanosec === 0)).toBe(true);
  ros.disconnect();
});

it("没有MCU同步源时间不发维护写，但get_device_info只读请求仍可发送", async () => {
  const { ros } = await connected();
  const { store } = await import("../src/core/store.ts");
  const device = await import("../src/core/device.ts");
  zeroReady(store);
  store.controlStates.sourceTimeNs = 0;
  vi.stubGlobal("window", { confirm: () => true });
  await device.setMotorZero(2);
  await device.clearMotorFault(2);
  expect(Service.calls).toHaveLength(0);
  const p = device.loadParams();
  expect(Service.calls).toHaveLength(1);
  expect(Service.calls[0]!.name).toBe("/motor/get_device_info");
  Service.calls[0]!.complete({ info: { params: [] } });
  await p;
  ros.disconnect();
});
