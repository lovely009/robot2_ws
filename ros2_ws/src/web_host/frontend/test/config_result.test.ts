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
  ros.configureControl({control_mode: "bench"});
  ros.connect();
  Socket.instances[0].emit("connection");
  const {store}=await import("../../web/js/core/store.js");
  store.controlStates.sourceTimeNs=9000000000;
  const resultTopic = Topic.instances.find((topic) => topic.name === "/motor/config_result");
  expect(resultTopic).toBeDefined();
  return { ros, resultTopic: resultTopic! };
}

const result = (request_id: number, status: number, session_id = 9) => ({
  session_id, request_id, kind: 1, command: 0, status,
  error_code: 0, completed_targets: 1, failed_targets: 0,
});
const controlSnapshot = (session_id: number, sequence: number, changes = {}) => ({
  schema_version: 2, session_id, sequence, states: [], ...changes,
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

it("已移除的参数写入与保存不创建下行请求", async () => {
  const { ros } = await connected();
  const before = Topic.published.length;
  expect(ros.pub('paramSet', {count:1,targets:[{motor_id:1,field:0,value:1}]})).toBe(false);
  expect(ros.pub('firmwareConfig', {count:1,targets:[{motor_id:65535,field:0,value:50}]})).toBe(false);
  expect(ros.pub('sysCmd',{command:0,motor_id:1})).toBe(false);
  expect(Topic.published).toHaveLength(before);
  ros.disconnect();
});

function zeroReady(store: any, id = 2) {
  store.controlStates = {seen:true,valid:true,at:Date.now(),sourceTimeNs:9000000000,session:9,sequence:1,
    byId:new Map([[id,{motor_id:id,online:true,control_lifecycle:2,execution_state:1,
      policy_enabled:false,motion_enabled:false,state_age_ms:5,unified_fault_code:0,
      vendor_fault_code:0,last_failure_reason:0,policy_generation:1}]])};
}

it('设零只发所选电机一次，并等待对应设备终态，不恢复使能或目标', async () => {
  const {ros,resultTopic} = await connected();
  const {store} = await import('../../web/js/core/store.js');
  const device = await import('../../web/js/core/device.js');
  vi.stubGlobal('window',{confirm:vi.fn(()=>true)});
  zeroReady(store,64);
  const pending=device.setMotorZero(64);
  const sent=Topic.published.find(entry=>entry.topic==='/motor/system_command')!;
  expect(sent.message).toMatchObject({command:2,motor_id:64});
  await device.setMotorZero(64);
  expect(Topic.published.filter(entry=>entry.message.command===2)).toHaveLength(1);
  resultTopic.callback?.({...result(sent.message.request_id,0),command:2});
  await Promise.resolve();
  expect(store.results.get('paramResult').text).not.toContain('成功');
  resultTopic.callback?.({...result(sent.message.request_id,1),command:2});
  await pending;
  expect(store.results.get('paramResult')).toMatchObject({ok:true});
  expect(store.results.get('paramResult').text).toContain('设备已确认');
  expect(Topic.published.every(entry=>entry.topic==='/motor/system_command')).toBe(true);
  ros.disconnect();
});

it.each(['enabled','motion','offline','stale','fault','vendor_fault','transition','host_active','host_pending','readonly','invalid','canceled'])('设零拒绝 %s，不下发请求',async change=>{
  const {ros}=await connected();
  const {store}=await import('../../web/js/core/store.js');
  const device=await import('../../web/js/core/device.js');
  vi.stubGlobal('window',{confirm:vi.fn(()=>change!=='canceled')});
  zeroReady(store);
  const state=store.controlStates.byId.get(2)!;
  if(change==='enabled')state.policy_enabled=true;
  if(change==='motion')state.motion_enabled=true;
  if(change==='offline')state.online=false;
  if(change==='stale')store.controlStates.at=Date.now()-3000;
  if(change==='fault')state.unified_fault_code=5;
  if(change==='vendor_fault')state.vendor_fault_code=257;
  if(change==='transition')state.execution_state=6;
  if(change==='host_active')store.commandStatus.data={mode:'trajectory',active_controllers:['trajectory_controller']};
  if(change==='host_pending')store.commandStatus.data={mode:'stopped',pending_goal_ids:[1]};
  if(change==='readonly')ros.configureControl({control_mode:'readonly'});
  await device.setMotorZero(change==='invalid'?0:2);
  expect(Topic.published).toHaveLength(0);
  ros.disconnect();
});

it.each([10,5])('设零失败显示设备错误 %s，不误报成功',async error_code=>{
  const {ros,resultTopic}=await connected();
  const {store}=await import('../../web/js/core/store.js');
  const device=await import('../../web/js/core/device.js');
  vi.stubGlobal('window',{confirm:()=>true});zeroReady(store);
  const pending=device.setMotorZero(2);
  const sent=Topic.published.find(entry=>entry.message.command===2)!;
  resultTopic.callback?.({...result(sent.message.request_id,2),command:2,error_code,failed_targets:1});
  await pending;
  expect(store.results.get('paramResult')).toMatchObject({ok:false});
  expect(store.results.get('paramResult').text).toContain(`error_code=${error_code}`);
  if(error_code===5)expect(store.results.get('paramResult').text).toContain('设备零位可能已改变');
  ros.disconnect();
});

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
  controlTopic.callback?.(controlSnapshot(realisticSessionId + 1, 1, { schema_version: 1 }));
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


it('主机实例复用request_id时，新intent的PENDING不会被旧终态挡掉',async()=>{
  const {ros,resultTopic}=await connected();
  const {store}=await import('../../web/js/core/store.js');
  resultTopic.callback?.({...result(7,1),intent_id:'old-host:1',kind:2});
  resultTopic.callback?.({...result(7,0),intent_id:'new-host:1',kind:2});
  expect(store.configResults.get(7)).toMatchObject({status:0,intent_id:'new-host:1'});
  resultTopic.callback?.({...result(7,1),intent_id:'new-host:1',kind:2});
  resultTopic.callback?.({...result(7,0),intent_id:'new-host:1',kind:2});
  expect(store.configResults.get(7)).toMatchObject({status:1,intent_id:'new-host:1'});
  ros.disconnect();
});


it('浏览器时钟偏移时，系统维护写只携带缓存MCU同步源时间',async()=>{
  vi.useFakeTimers();vi.setSystemTime(777000);
  const {ros}=await connected();
  const {store}=await import('../../web/js/core/store.js');
  const {sendSystemCommand}=await import('../../web/js/core/device.js');
  zeroReady(store);
  for(const command of [1,2,3])expect(sendSystemCommand(command,2)).toBeGreaterThan(0);
  expect(Topic.published).toHaveLength(3);
  expect(Topic.published.every(entry=>entry.message.sync_time_ns===9000000000)).toBe(true);
  expect(Topic.published[0].message.sync_time_ns).not.toBe(Date.now()*1e6);
  ros.disconnect();
});

it('没有MCU同步源时间不发维护写，但REPORT只读请求仍携带零时间发送',async()=>{
  const {ros}=await connected();
  const {store}=await import('../../web/js/core/store.js');
  const {sendSystemCommand}=await import('../../web/js/core/device.js');
  zeroReady(store);store.controlStates.sourceTimeNs=0;
  for(const command of [1,2,3])expect(sendSystemCommand(command,2)).toBe(0);
  expect(Topic.published).toHaveLength(0);
  expect(sendSystemCommand(4,65535)).toBeGreaterThan(0);
  expect(Topic.published).toHaveLength(1);
  expect(Topic.published[0].message).toMatchObject({command:4,sync_time_ns:0});
  ros.disconnect();
});
