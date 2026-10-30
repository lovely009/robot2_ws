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

it('托管页面隐藏后使旧动作循环的连接代数失效，回到前台不恢复旧会话', async () => {
  const callbacks = new Map<string, () => void>();
  const document = globalThis.document as unknown as {hidden: boolean; addEventListener: unknown};
  document.hidden = false;
  document.addEventListener = (name: string, callback: () => void) => callbacks.set(name, callback);
  const ros = await import('../../web/js/core/ros.js');
  ros.configureControl({control_mode: 'managed'});
  const before = ros.getConnectionGeneration();
  document.hidden = true;
  callbacks.get('visibilitychange')!();
  expect(ros.getConnectionGeneration()).toBe(before + 1);
  document.hidden = false;
  callbacks.get('visibilitychange')!();
  expect(ros.getConnectionGeneration()).toBe(before + 1);
});

function feedforwardDocument() {
  const elements = new Map<string, any>();
  const element = (id: string) => {
    if (!elements.has(id)) elements.set(id, {
      value: id === 'rosUrl' ? 'ws://example.test:9090' : '',
      textContent: '', checked: false, disabled: false, hidden: false,
      replaceChildren() {}, append() {}, addEventListener() {},
    });
    return elements.get(id);
  };
  vi.stubGlobal('document', {
    getElementById: element,
    createElement: () => ({value: '', textContent: ''}),
    body: {classList: {toggle() {}}},
  });
  vi.stubGlobal('window', {setInterval});
  return element;
}

it('刷新后即使前馈面板未渲染，动作窗口也恢复主机的补偿路由', async () => {
  vi.useFakeTimers();
  const element = feedforwardDocument();
  vi.stubGlobal('fetch', async (url: string) => ({ok: true,
    json: async () => url === '/api/models' ? {models: ['half_body_robot']} : action}));
  const ros = await import('../../web/js/core/ros.js');
  ros.configureControl({control_mode: 'bench'});
  const {store} = await import('../../web/js/core/store.js');
  const {playAction} = await import('../../web/js/panels/actionLibrary.js');
  const feedforward = await import('../../web/js/panels/feedforward.js');
  ros.connect();
  Socket.instances[0].emit('connection');
  const receive = (name: string, data: object) => Topic.instances.find(t => t.name === name)!
    .callback?.({data: JSON.stringify(data)});
  receive('/motor_command/status', {output_topic: '/robot_dynamics/command_array'});
  receive('/bench/status', {agent: {state: 'running'},
    gravity_feedforward: {state: 'running', config: {model: 'half_body_robot', effort_gain: 1}}});
  store.modelTransmissions = {model: 'half_body_robot', groups: [
    {name: 'direct_m1', motors: [{id: 1}], joints: ['joint_link1']},
    {name: 'wrist', motors: [{id: 8}, {id: 9}], joints: ['joint_link8','joint_link9']},
  ]};
  store.motors.set(1, {status: {online: true, lifecycle: 3}, motion: {position: .15}});
  const pending = playAction('shoulder.json', {kp: 1, kd: 1, enable: false, loop: false});
  await vi.advanceTimersByTimeAsync(3400);
  const windows = Topic.published.filter(p => (p.message as any).points?.length);
  expect(windows).toHaveLength(1);
  expect(windows[0].topic).toBe('/robot_dynamics/trajectory');
  const panel = feedforward.mount({}, {});
  expect(element('ffRoute').checked).toBe(true);
  expect(element('ffSummary').textContent).toBe('经前馈');
  Topic.instances.find(t => t.name === '/motor/trajectory_state')!.callback?.({
    trajectory_id: (windows[0].message as any).trajectory_id,
    active: true, elapsed_ms: 100,
  });
  await vi.advanceTimersByTimeAsync(50);
  await pending;
  panel.unmount();
  ros.disconnect();
});

it('换连接后从新主机恢复路由和面板意图，周期状态不覆盖显式切换', async () => {
  vi.useFakeTimers();
  const element = feedforwardDocument();
  vi.stubGlobal('fetch', async () => ({ok: true, json: async () => ({models: ['half_body_robot']})}));
  const ros = await import('../../web/js/core/ros.js');
  ros.configureControl({control_mode: 'bench'});
  const {DIRTY} = await import('../../web/js/core/store.js');
  const feedforward = await import('../../web/js/panels/feedforward.js');
  const observe = (output_topic: string) => {
    Topic.instances.filter(t => t.name === '/motor_command/status').at(-1)!.callback?.({data: JSON.stringify({output_topic})});
    Topic.instances.filter(t => t.name === '/bench/status').at(-1)!.callback?.({data: JSON.stringify({
      agent: {state: 'running'}, gravity_feedforward: {state: 'running'},
    })});
  };
  ros.connect();
  Socket.instances[0].emit('connection');
  observe('/robot_dynamics/command_array');
  const panel = feedforward.mount({}, {});
  expect(element('ffRoute').checked).toBe(true);
  ros.connect();
  Socket.instances[1].emit('connection');
  observe('/motor/command_array');
  panel.render(new Set([DIRTY.CONN, DIRTY.COMMAND]));
  expect(element('ffRoute').checked).toBe(false);
  ros.pub('trajectory', {count: 1});
  expect(Topic.published.at(-1)?.topic).toBe('/motor/trajectory');
  ros.setDownlinkRoute('overlay');
  observe('/motor/command_array');
  ros.pub('trajectory', {count: 1});
  expect(Topic.published.at(-1)?.topic).toBe('/robot_dynamics/trajectory');
  panel.unmount();
  ros.disconnect();
});

it("读取动作期间换连接，旧播放不得在新会话下发控制", async () => {
  vi.useFakeTimers();
  let respond!: (value: unknown) => void;
  vi.stubGlobal("fetch", () => new Promise((resolve) => (respond = resolve)));
  const ros = await import("../../web/js/core/ros.js");
  ros.configureControl({control_mode: "bench"});
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
  ros.configureControl({control_mode: "bench"});
  const { store } = await import("../../web/js/core/store.js");
  const { playAction } = await import("../../web/js/panels/actionLibrary.js");
  store.modelTransmissions = {
    model: store.MODEL,
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
  await vi.runAllTimersAsync();
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
  ros.configureControl({control_mode: "bench"});
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
  ros.configureControl({control_mode: "bench"});
  ros.connect();
  Socket.instances[0].emit("connection");
  Socket.instances[0].emit("close");
  ros.disconnect();
  vi.advanceTimersByTime(3000);
  expect(Socket.instances).toHaveLength(1);
});

it("手动切换连接复位参数基线、台架和命令去向", async () => {
  const ros = await import("../../web/js/core/ros.js");
  ros.configureControl({control_mode: "bench"});
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
  ros.configureControl({control_mode: "bench"});
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
  ros.configureControl({control_mode: "bench"});
  ros.connect();
  Socket.instances[0].emit("connection");

  const uplinks = [
    ["/motor/motion_states", "can_bridge_msgs/msg/MotorMotionStateArray"],
    ["/motor/status_array", "can_bridge_msgs/msg/MotorStatusArray"],
    ["/motor/topology", "can_bridge_msgs/msg/MotorTopology"],
    ["/motor/events", "can_bridge_msgs/msg/MotorEvent"],
    ["/motor/trajectory_state", "can_bridge_msgs/msg/MotorTrajectoryState"],
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
    ["/motor_command/setpoint", "can_bridge_msgs/msg/MotorCommandArray"],
    ["/motor/trajectory", "can_bridge_msgs/msg/MotorTrajectory"],
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
  ros.configureControl({control_mode: "bench"});
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
  ros.configureControl({control_mode: 'bench'});
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
