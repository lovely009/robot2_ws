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
  static complete: (response: any) => void;
  static calls: {name: string; request: any; complete: (response: any) => void}[] = [];
  name: string;
  constructor(options: {name: string}) { this.name = options.name; }
  callService(request: unknown, complete: (response: any) => void) {
    Service.complete = complete;
    Service.calls.push({name:this.name,request,complete});
  }
}

beforeEach(async () => {
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
    Message: class {
      constructor(value: object) {
        Object.assign(this, value);
      }
    },
  });
  // This file exercises the workbench facade.
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

function controlSnapshot(generations: number[], sequence = 1) {
  return {schema_version: 2, session_id: 42, sync_time_ns: 9000000000, sequence,
    states: generations.map((policy_generation, index) => ({
      motor_id: index + 1, online: true, control_lifecycle: 2, execution_state: 1,
      policy_enabled: false, motion_enabled: false, state_age_ms: 0,
      unified_fault_code: 0, vendor_fault_code: 0, last_failure_reason: 0,
      policy_generation,
    }))};
}

const policyTarget = {motor_id:1, enabled:true, kp:30, kd:1, confirm_unprotected:false};
const policyTicket = (changes = {}) => ({accepted:true,session_id:42,intent_id:'host:1',
  policy_generation:900,request_id:7,reason:'',...changes});
async function policyConnection() {
  const ros=await import('../../web/js/core/ros.js');
  const {store}=await import('../../web/js/core/store.js');
  ros.connect();Socket.instances[0].emit('connection');
  const control=Topic.instances.find(t=>t.name==='/motor/control_states')!;
  control.callback?.(controlSnapshot([800]));
  return {ros,store,control};
}

it('浏览器只提交目标与最新MCU源时间，序号由主机返回且不直发policy话题',async()=>{
  const {ros,store}=await policyConnection();
  const pending=ros.submitPolicy([policyTarget]);
  const sent=Service.calls.at(-1)!;
  expect(sent.name).toBe('/motor/submit_policy');
  expect(sent.request).toEqual({session_id:42,source_time_ns:store.controlStates.sourceTimeNs,targets:[policyTarget]});
  expect(sent.request.source_time_ns).toBe(9000000000);
  expect(sent.request).not.toHaveProperty('request_id');
  expect(sent.request).not.toHaveProperty('policy_generation');
  sent.complete(policyTicket());
  await expect(pending).resolves.toMatchObject(policyTicket());
  expect(ros.pub('policy',{targets:[policyTarget]})).toBe(false);
  expect(Topic.published).toEqual([]);
  expect(Topic.instances.some(t=>t.name==='/motor/policy_array')).toBe(false);
  ros.disconnect();
});

it('多次人工使能使用主机返回的不同intent与序号，网页不自行递增',async()=>{
  const {ros}=await policyConnection();
  for(const ticket of [policyTicket(),policyTicket({intent_id:'host:2',request_id:11,policy_generation:1900})]) {
    const pending=ros.submitPolicy([policyTarget]);
    Service.calls.at(-1)!.complete(ticket);
    await expect(pending).resolves.toMatchObject(ticket);
  }
  expect(Service.calls).toHaveLength(2);expect(Topic.published).toEqual([]);
  ros.disconnect();
});

it.each([48,49])('策略提交与服务的48个目标容量一致：%s',async count=>{
  const {ros}=await policyConnection();
  const pending=ros.submitPolicy(Array.from({length:count},(_,index)=>({...policyTarget,motor_id:index+1})));
  if(count===48) {
    expect(Service.calls).toHaveLength(1);
    expect(Service.calls[0].request.targets).toHaveLength(48);
    Service.complete(policyTicket());
    await expect(pending).resolves.toMatchObject(policyTicket());
  } else {
    await expect(pending).rejects.toThrow('使能目标无效');
    expect(Service.calls).toHaveLength(0);
  }
  ros.disconnect();
});

it('同页面任一入口发起使能后，MCU先ACTIVE也不能越过本intent的成功ACK',async()=>{
  const {ros,store,control}=await policyConnection();
  const {controlTargetsError}=await import('../../web/js/core/motor_state.js');
  const pending=ros.submitPolicy([policyTarget]);
  control.callback?.({...controlSnapshot([900],2),states:controlSnapshot([900]).states.map(s=>
    ({...s,policy_enabled:true,control_lifecycle:3}))});
  expect(controlTargetsError(store,[1])).toBeTruthy();
  Service.complete(policyTicket());await pending;
  const result=Topic.instances.find(t=>t.name==='/motor/config_result')!;
  result.callback?.({...policyTicket(),kind:2,command:0,status:1,intent_id:'old-host:1'});
  expect(controlTargetsError(store,[1])).toBeTruthy();
  result.callback?.({...policyTicket(),kind:2,command:0,status:1});
  expect(controlTargetsError(store,[1])).toBe('');
  ros.disconnect();
});

it.each(['stale','session','time','readonly'])('使能拒绝缺失或陈旧源授权：%s',async change=>{
  const {ros,store}=await policyConnection();
  if(change==='stale')store.controlStates.at=Date.now()-101;
  if(change==='session')store.controlStates.session=0;
  if(change==='time')store.controlStates.sourceTimeNs=0;
  if(change==='readonly')ros.configureControl({control_mode:'readonly'});
  await expect(ros.submitPolicy([policyTarget])).rejects.toThrow();
  expect(Service.calls).toHaveLength(0);expect(Topic.published).toEqual([]);
  ros.disconnect();
});

it('失能允许无快照和零代数票据，不等待新的ACK或generation',async()=>{
  const {ros,store}=await policyConnection();
  store.controlStates.valid=false;store.controlStates.session=0;store.controlStates.sourceTimeNs=0;
  const pending=ros.submitPolicy([{...policyTarget,enabled:false}]);
  expect(Service.calls.at(-1)!.request).toMatchObject({session_id:0,source_time_ns:0});
  const ticket=policyTicket({session_id:0,request_id:0,policy_generation:0});
  Service.calls.at(-1)!.complete(ticket);
  await expect(pending).resolves.toMatchObject(ticket);
  ros.disconnect();
});

it.each(['connection','session'])('旧RPC成功响应不能跨越%s边界',async change=>{
  const {ros,control}=await policyConnection();
  const pending=ros.submitPolicy([policyTarget]);
  const rejected=expect(pending).rejects.toThrow();
  if(change==='connection'){ros.connect();Socket.instances[1].emit('connection');}
  else control.callback?.({...controlSnapshot([800],2),session_id:43});
  Service.complete(policyTicket());await rejected;
  expect(Topic.published).toEqual([]);ros.disconnect();
});

it.each([{accepted:false,reason:'expired'},{intent_id:''},{request_id:0},{policy_generation:0},{session_id:43}])(
  '拒绝错误或不完整的主机使能票据：%j',async changes=>{
    const {ros}=await policyConnection();
    const pending=ros.submitPolicy([policyTarget]);
    const rejected=expect(pending).rejects.toThrow();
    Service.complete(policyTicket(changes));await rejected;
    ros.disconnect();
  });

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
  store.controlStates={seen:true,valid:true,at:Date.now(),sourceTimeNs:9000000000,session:42,byId:new Map([[1,{motor_id:1,online:true,control_lifecycle:2,policy_enabled:false,state_age_ms:0,unified_fault_code:0,policy_generation:0}]])};
  const feedback=setInterval(()=>{store.controlStates.at=Date.now()},20);
  const pending = playAction("old.json", {
    kp: 1,
    kd: 1,
    enable: true,
    loop: false,
  });
  await vi.advanceTimersByTimeAsync(800);
  clearInterval(feedback);
  expect(Service.calls.at(-1)?.name).toBe('/motor/submit_policy');
  const before = Topic.published.length;
  ros.connect();
  Socket.instances[1].emit("connection");
  store.motors.set(1, {
    status: { online: true, lifecycle: 3 },
    motion: { position: 0.15 },
  });
  Service.complete(policyTicket());
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

it('原始动作按300原帧时序分包发布到主机raw话题，取消仍走普通空轨迹',async()=>{
  vi.useFakeTimers();
  const ros=await import('../../web/js/core/ros.js');
  const {store,MODEL}=await import('../../web/js/core/store.js');
  const actions=await import('../../web/js/panels/actionLibrary.js');
  ros.connect();Socket.instances[0].emit('connection');
  store.controlStates.sourceTimeNs=9000000000;
  expect(Topic.instances.filter(t=>t.name==='/control/trajectory_raw')).toHaveLength(1);
  const now=Date.now();
  store.modelTransmissions={model:MODEL,groups:[1,2].map(id=>({name:`m${id}`,
    motors:[{id}],joints:[`joint${id}`],direct_mapping:true})),error:''};
  store.transmissionStatus={last:now,data:{model:MODEL,joint_names:['joint1','joint2'],trajectory_enabled:true}};
  store.commandStatus={last:now,data:{model:MODEL,joint_names:['joint1','joint2'],
    output_topic:'/motor/host_command_array',trajectory_output_topic:'/host_control/trajectory_controller/follow_joint_trajectory'}};
  for(const id of [1,2])store.motors.set(id,{status:{online:true,lifecycle:4},motion:{position:.1}});
  const points=Array.from({length:300},(_,frame)=>[1,2].map(motor_id=>({motor_id,
    time_from_start_ms:Math.round(frame*1000/30),position:motor_id*.1+frame*.0001,
    velocity:frame===0?0:9.2,effort:0}))).flat();
  vi.stubGlobal('fetch',async()=>({ok:true,json:async()=>({interpolation:'linear',target_ids:[1,2],points})}));
  const running=actions.playAction('raw.json',{enable:false,loop:false,kp:30,kd:1});
  await vi.advanceTimersByTimeAsync(1000);
  const segments=Topic.published.filter(p=>p.topic==='/control/trajectory_raw').map(p=>p.message as any);
  expect(segments).toHaveLength(3);
  expect(segments.every(s=>s.count<=256 && s.segment_count===3)).toBe(true);
  const transmitted=segments.flatMap(s=>s.points);
  expect(transmitted).toEqual(points.map(p=>({...p,time_from_start_ms:2500+p.time_from_start_ms})));
  expect(Topic.published.some(p=>p.topic==='/motor/policy_array')).toBe(false);
  const empty=Topic.published.filter(p=>p.topic==='/control/trajectory').map(p=>p.message as any);
  expect(empty).toHaveLength(1);expect(empty[0]).toMatchObject({count:0,points:[],target_ids:[]});
  actions.stopPlayback();
  await vi.advanceTimersByTimeAsync(50);await running;
  const last=Topic.published.at(-1)!;
  expect(last.topic).toBe('/control/trajectory');
  expect(last.message).toMatchObject({count:0,points:[],target_ids:[]});
  ros.disconnect();
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
    (name) => name.startsWith("/micro_ros_up/"))).toEqual(["/micro_ros_up/status"]);

  const downlinks = [
    ["/control/setpoint", "can_bridge_msgs/msg/MotorCommandArray"],
    ["/control/trajectory", "can_bridge_msgs/msg/MotorTrajectory"],
    ["/control/trajectory_raw", "can_bridge_msgs/msg/MotorTrajectory"],
    ["/control/stop", "std_msgs/msg/Empty"],
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

it.each([null, NaN, '0.2', undefined])('无效关节位置 %s 清除旧读数，不伪造零或保留假新鲜反馈', async position => {
  const ros = await import('../../web/js/core/ros.js');
  const data = await import('../../web/js/core/store.js');
  ros.connect(); Socket.instances[0].emit('connection');
  data.store.bench.feedback = {source:'firmware',state:'running',
    model:data.MODEL,desired_model:data.MODEL};
  const joints = Topic.instances.find(t => t.name === '/joint_states')!;
  joints.callback?.({name:['elbow','wrist'],position:[.4,.6],velocity:[0,0],effort:[0,0]});
  expect(data.store.jointStates.byName.elbow.position).toBe(.4);
  joints.callback?.({name:['elbow','wrist'],position:[position,.7],velocity:[0,0],effort:[0,0]});
  expect(data.store.jointStates.byName.elbow).toBeUndefined();
  expect(data.store.jointStates.byName.wrist.position).toBe(.7);
  ros.disconnect();
});

it('真实关节零位置仍有效，缺失速度和力矩保持未知而非零', async () => {
  const ros = await import('../../web/js/core/ros.js');
  const data = await import('../../web/js/core/store.js');
  ros.connect(); Socket.instances[0].emit('connection');
  data.store.bench.feedback = {source:'firmware',state:'running',
    model:data.MODEL,desired_model:data.MODEL};
  const joints = Topic.instances.find(t => t.name === '/joint_states')!;
  for (const value of [null, NaN, '0.2', undefined]) {
    joints.callback?.({name:['elbow'],position:[0],velocity:[value],effort:[value]});
    expect(data.store.jointStates.byName.elbow.position).toBe(0);
    expect(data.store.jointStates.byName.elbow.velocity).toBeNaN();
    expect(data.store.jointStates.byName.elbow.effort).toBeNaN();
  }
  ros.disconnect();
});

it('浏览器观测限流并保留事件队列，避免反馈堵塞控制连接', async () => {
  const ros = await import('../../web/js/core/ros.js');
  ros.connect(); Socket.instances[0].emit('connection');
  expect(Topic.instances.find(t => t.name === '/motor/motion_states')?.options)
    .toMatchObject({throttle_rate:50, queue_length:1});
  expect(Topic.instances.find(t => t.name === '/motor/events')?.options)
    .toMatchObject({throttle_rate:0, queue_length:64});
  ros.disconnect();
});
