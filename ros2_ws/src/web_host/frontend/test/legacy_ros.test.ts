import { afterEach, beforeEach, expect, it, vi } from "vitest";

class Socket {
  static instances: Socket[] = [];
  handlers = new Map<string, (() => void)[]>();
  closed = false;
  constructor() {
    Socket.instances.push(this);
  }
  on(name: string, callback: () => void) {
    this.handlers.set(name, [...(this.handlers.get(name) || []), callback]);
  }
  emit(name: string) {
    for (const callback of this.handlers.get(name) || []) callback();
  }
  close() {
    this.closed = true;
  }
}
class Topic {
  static instances: Topic[] = [];
  static published: { topic: string; message: unknown }[] = [];
  name: string;
  callback?: (message: unknown) => void;
  constructor(options: { name: string }) {
    this.name = options.name;
    Topic.instances.push(this);
  }
  subscribe(callback: (message: unknown) => void) {
    this.callback = callback;
  }
  unsubscribe() {}
  advertise() {}
  unadvertise() {}
  publish(message: unknown) {
    Topic.published.push({ topic: this.name, message });
  }
}
class Service {
  static complete: (response: unknown) => void;
  callService(_request: unknown, complete: (response: unknown) => void) {
    Service.complete = complete;
  }
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
    Service,
    Message: class {
      constructor(value: object) {
        Object.assign(this, value);
      }
    },
  });
});

const action = {
  target_ids: [1],
  points: [
    { motor_id: 1, time_from_start_ms: 0, position: 0.1 },
    { motor_id: 1, time_from_start_ms: 100, position: 0.2 },
  ],
};

it("读取动作期间换连接，旧播放不得在新会话下发控制", async () => {
  vi.useFakeTimers();
  let respond!: (value: unknown) => void;
  vi.stubGlobal("fetch", () => new Promise((resolve) => (respond = resolve)));
  const ros = await import("../../web/js/core/ros.js");
  const { store } = await import("../../web/js/core/store.js");
  const { playAction } = await import("../../web/js/panels/actionLibrary.js");
  ros.connect();
  Socket.instances[0].emit("connection");
  const pending = playAction("old.json", {
    kp: 1,
    kd: 1,
    enable: false,
    loop: false,
  });
  ros.connect();
  Socket.instances[1].emit("connection");
  store.motors.set(1, {
    status: { online: true, lifecycle: 3 },
    motion: { position: 0.15 },
  });
  respond({ ok: true, json: async () => action });
  await vi.runAllTimersAsync();
  await pending;
  expect(Topic.published).toEqual([]);
  ros.disconnect();
});

it("武装等待期间换连接，旧播放不得继续对齐或发轨迹", async () => {
  vi.useFakeTimers();
  vi.stubGlobal("fetch", async () => ({ ok: true, json: async () => action }));
  const ros = await import("../../web/js/core/ros.js");
  const { store } = await import("../../web/js/core/store.js");
  const { playAction } = await import("../../web/js/panels/actionLibrary.js");
  ros.connect();
  Socket.instances[0].emit("connection");
  store.motors.set(1, {
    status: { online: true, lifecycle: 3 },
    motion: { position: 0.15 },
  });
  const pending = playAction("old.json", {
    kp: 1,
    kd: 1,
    enable: true,
    loop: false,
  });
  await vi.waitFor(() =>
    expect(Topic.published.some((p) => p.topic === "/motor/policy_array")).toBe(
      true,
    ),
  );
  const before = Topic.published.length;
  ros.connect();
  Socket.instances[1].emit("connection");
  store.motors.set(1, {
    status: { online: true, lifecycle: 3 },
    motion: { position: 0.15 },
  });
  await vi.runAllTimersAsync();
  await pending;
  expect(Topic.published).toHaveLength(before);
  ros.disconnect();
});
afterEach(() => {
  vi.useRealTimers();
  vi.unstubAllGlobals();
});

it("手动重连后晚到的旧 socket 关闭与错误事件不关闭新连接", async () => {
  const ros = await import("../../web/js/core/ros.js");
  const data = await import("../../web/js/core/store.js");
  ros.connect();
  const old = Socket.instances[0];
  old.emit("connection");
  ros.connect();
  const current = Socket.instances[1];
  current.emit("connection");
  old.emit("close");
  old.emit("error");
  expect(current.closed).toBe(false);
  expect(data.store.conn.connected).toBe(true);
  ros.disconnect();
});

it("显式销毁连接取消已经排队的自动重连", async () => {
  vi.useFakeTimers();
  const ros = await import("../../web/js/core/ros.js");
  ros.connect();
  Socket.instances[0].emit("connection");
  Socket.instances[0].emit("close");
  ros.disconnect();
  vi.advanceTimersByTime(3000);
  expect(Socket.instances).toHaveLength(1);
});

it("手动切换连接复位参数基线、台架和命令去向", async () => {
  const ros = await import("../../web/js/core/ros.js");
  const { store } = await import("../../web/js/core/store.js");
  ros.connect();
  Socket.instances[0].emit("connection");
  store.sysStatus.seen = true;
  store.conn.autoLoaded = true;
  store.conn.benchLoaded = true;
  store.paramCache.set(1, {});
  store.bench.simulation = { state: "running" };
  store.commandStatus.data = { output_topic: "/old/session" };
  ros.connect();
  expect(store.conn.connected).toBe(false);
  expect(store.sysStatus.seen).toBe(false);
  expect(store.conn.autoLoaded).toBe(false);
  expect(store.conn.benchLoaded).toBe(false);
  expect(store.paramCache.size).toBe(0);
  expect(store.bench.simulation).toBeNull();
  expect(store.commandStatus.data).toBeNull();
  ros.disconnect();
});

it("换连接后旧订阅排队的运动帧不再写入当前状态", async () => {
  const ros = await import("../../web/js/core/ros.js");
  const { store } = await import("../../web/js/core/store.js");
  ros.connect();
  Socket.instances[0].emit("connection");
  const motion = Topic.instances.find(
    (t) => t.name === "/micro_ros_up/motion_states",
  )!;
  ros.connect();
  Socket.instances[1].emit("connection");
  motion.callback?.({ states: [{ motor_id: 99, position: 1 }] });
  expect(store.motors.has(99)).toBe(false);
  ros.disconnect();
});

it("旧连接的服务响应不能被当作当前会话成功回执", async () => {
  const ros = await import("../../web/js/core/ros.js");
  ros.connect();
  Socket.instances[0].emit("connection");
  const pending = ros.callService(
    "/bench/agent_status",
    "bench_msgs/srv/GetAgentStatus",
    {},
  );
  ros.connect();
  Socket.instances[1].emit("connection");
  Service.complete({ ok: true });
  await expect(pending).rejects.toThrow("连接已更换");
  ros.disconnect();
});
