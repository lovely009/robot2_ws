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
  options: { name: string; [key: string]: unknown };
  callback?: (message: unknown) => void;
  constructor(options: { name: string; [key: string]: unknown }) {
    this.name = options.name;
    this.options = options;
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

beforeEach(async () => {
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
  // This file exercises the retained bench facade, not managed authorization.
  const ros = await import("../../web/js/core/ros.js");
  ros.configureControl({control_mode: 'bench'});
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
  const { store, MODEL } = await import("../../web/js/core/store.js");
  const { playAction } = await import("../../web/js/panels/actionLibrary.js");
  store.modelTransmissions = {
    model: MODEL,
    driver_file: '',
    groups: [{
      name: 'direct_m1',
      motors: [{ name: 'm1', id: 1 }],
      joints: ['joint_link1'],
      direct_mapping: true,
    }],
    error: '',
    direct_mapping: { 1: 'joint_link1' },
  };
  ros.connect();
  Socket.instances[0].emit("connection");
  store.transmissionStatus={last:Date.now(),data:{model:MODEL,joint_names:['joint_link1'],trajectory_enabled:true}};
  store.commandStatus={last:Date.now(),data:{model:MODEL,joint_names:['joint_link1'],
    output_topic:'/motor/host_command_array',
    trajectory_output_topic:'/host_control/trajectory_controller/follow_joint_trajectory'}};
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
  await vi.advanceTimersByTimeAsync(900);
  expect(Topic.published.some(p => p.topic === '/motor/policy_array')).toBe(true);
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
  vi.restoreAllMocks();
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
    (t) => t.name === "/motor/motion_states",
  )!;
  ros.connect();
  Socket.instances[1].emit("connection");
  motion.callback?.({ states: [{ motor_id: 99, position: 1 }] });
  expect(store.motors.has(99)).toBe(false);
  ros.disconnect();
});

it("连接只消费主机 facade 的固件上行并保持消息契约", async () => {
  const ros = await import("../../web/js/core/ros.js");
  ros.connect();
  Socket.instances[0].emit("connection");

  const uplinks = [
    ["/motor/motion_states", "can_bridge_msgs/msg/MotorMotionStateArray"],
    ["/motor/status_array", "can_bridge_msgs/msg/MotorStatusArray"],
    ["/motor/topology", "can_bridge_msgs/msg/MotorTopology"],
    ["/motor/events", "can_bridge_msgs/msg/MotorEvent"],
    ["/control/trajectory_state", "can_bridge_msgs/msg/MotorTrajectoryState"],
    ["/control/status", "std_msgs/msg/String"],
    ["/motor/device_param_report", "can_bridge_msgs/msg/DeviceParamReport"],
    ["/motor/system_status", "can_bridge_msgs/msg/SystemStatus"],
    ["/motor/control_states", "can_bridge_msgs/msg/MotorControlStateArray"],
    ["/imu/data", "sensor_msgs/msg/Imu"],
    ["/imu/temp", "std_msgs/msg/Float32"],
  ];
  for (const [name, messageType] of uplinks) {
    expect(Topic.instances).toContainEqual(expect.objectContaining({
      options: expect.objectContaining({ name, messageType }),
    }));
  }
  expect(Topic.instances.map((topic) => topic.name).filter(
    (name) => name.startsWith("/micro_ros_up/"))).toEqual([]);

  const downlinks = [
    ["/control/setpoint", "can_bridge_msgs/msg/MotorCommandArray"],
    ["/control/trajectory", "can_bridge_msgs/msg/MotorTrajectory"],
    ["/control/stop", "std_msgs/msg/Empty"],
    ["/motor/policy_array", "can_bridge_msgs/msg/MotorPolicyArray"],
    ["/motor/param_set", "can_bridge_msgs/msg/ParamSetArray"],
    ["/motor/system_command", "can_bridge_msgs/msg/SystemCommand"],
  ];
  for (const [name, messageType] of downlinks) {
    expect(Topic.instances).toContainEqual(expect.objectContaining({
      options: expect.objectContaining({ name, messageType }),
    }));
  }
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

it("切换模型清空旧关节值，主机模型同步完成前拒收旧反馈", async () => {
  vi.stubGlobal('fetch', async () => ({ok: false, status: 404}));
  vi.spyOn(console, 'warn').mockImplementation(() => {});
  const ros = await import('../../web/js/core/ros.js');
  const data = await import('../../web/js/core/store.js');
  ros.connect(); Socket.instances[0].emit('connection');
  const joints = Topic.instances.find(t => t.name === '/joint_states')!;
  data.store.bench.feedback = {source: 'firmware', state: 'running',
    model: data.MODEL, desired_model: data.MODEL};
  joints.callback?.({name: ['elbow'], position: [0.1], velocity: [0.2], effort: [0.3]});
  expect(data.store.jointStates.byName.elbow.position).toBe(0.1);
  data.store.modelJoints = [{name: 'elbow', lower: -1, upper: 1}];
  data.setModel('new_arm');
  expect(data.store.jointStates.byName).toEqual({});
  expect(data.store.modelJoints).toEqual([]);
  joints.callback?.({name: ['elbow'], position: [9]});
  expect(data.store.jointStates.byName).toEqual({});
  data.applyBenchStatus({feedback: {source: 'firmware', state: 'running',
    model: 'new_arm', desired_model: 'new_arm'}});
  joints.callback?.({name: ['elbow'], position: [0.4], velocity: [0.5], effort: [0.6]});
  expect(data.store.jointStates.byName.elbow).toMatchObject({position: 0.4, velocity: 0.5, effort: 0.6});
  ros.disconnect();
});

it('external 关节反馈必须等当前模型的新鲜主机状态，缺失或过期时拒收', async () => {
  const ros = await import('../../web/js/core/ros.js');
  const data = await import('../../web/js/core/store.js');
  ros.connect();Socket.instances[0].emit('connection');
  data.store.bench.feedback={source:'external'};
  const joints=Topic.instances.find(t=>t.name==='/joint_states')!;
  joints.callback?.({name:['elbow'],position:[9]});
  expect(data.store.jointStates.byName).toEqual({});
  data.store.transmissionStatus={last:Date.now()-3000,data:{model:data.MODEL}};
  joints.callback?.({name:['elbow'],position:[9]});
  expect(data.store.jointStates.byName).toEqual({});
  data.store.transmissionStatus={last:Date.now(),data:{model:data.MODEL}};
  joints.callback?.({name:['elbow'],position:[.4]});
  expect(data.store.jointStates.byName.elbow.position).toBe(.4);
  ros.disconnect();
});
