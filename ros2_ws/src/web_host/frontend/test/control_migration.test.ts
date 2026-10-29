import { afterEach, beforeEach, expect, it, vi } from 'vitest';

const { pub } = vi.hoisted(() => ({pub: vi.fn(() => true)}));
vi.mock('../../web/js/core/ros.js', () => ({
  pub, isConnected: () => true, getConnectionGeneration: () => 1,
  nextPolicyGeneration: () => 1,
}));
beforeEach(() => {
  vi.resetModules(); pub.mockClear(); vi.useFakeTimers(); vi.setSystemTime(10000);
  vi.stubGlobal('location', {search: ''});
  vi.stubGlobal('localStorage', {getItem: () => null});
  vi.stubGlobal('document', {getElementById: () => null});
  vi.stubGlobal('window', {confirm: () => true});
});
afterEach(() => {vi.useRealTimers();vi.unstubAllGlobals();});

async function fixture() {
  const data = await import('../../web/js/core/store.js');
  const {store, MODEL} = data;
  store.conn.connected = true;
  store.modelTransmissions = {model: MODEL, groups: [
    {name:'direct',motors:[{id:1}],joints:['elbow'],direct_mapping:true}],error:''};
  store.transmissionStatus={last:10000,data:{model:MODEL,joint_names:['elbow'],trajectory_enabled:true}};
  data.applyCommandStatus({model:MODEL,joint_names:['elbow'],mode:'trajectory',
    output_topic:'/motor/host_command_array',
    trajectory_output_topic:'/host_control/trajectory_controller/follow_joint_trajectory',
    gravity_enabled:false,effort_gain:1});
  store.motors.set(1,{status:{online:true,lifecycle:4},motion:{position:.1}});
  const action = {target_ids:[1],points:Array.from({length:700},(_,i) =>
    ({motor_id:1,time_from_start_ms:i*10,position:.1+i*.0001}))};
  vi.stubGlobal('fetch', async () => ({ok:true,json:async () => action}));
  return {data, store, actions:await import('../../web/js/panels/actionLibrary.js'),
    command:await import('../../web/js/panels/command.js')};
}

it('keeps arbitrary action length and sends the next window only after JTC success', async () => {
  const {store,actions} = await fixture();
  const running = actions.playAction('long.json',{enable:false,loop:false,kp:30,kd:1});
  await vi.advanceTimersByTimeAsync(1000);
  const alignment=pub.mock.calls.find(([key,msg])=>key==='trajectory' && msg.count>0)![1];
  expect(alignment.session_id).toBe(10000);
  expect(alignment.points.map((p:any)=>p.time_from_start_ms)).toEqual([2500]);
  store.traj.state={trajectory_id:alignment.trajectory_id,active:false,completed:true,elapsed_ms:2500};
  await vi.advanceTimersByTimeAsync(50);
  const windows = () => pub.mock.calls.filter(([key,msg]) => key==='trajectory' && msg.count>0).slice(1);
  expect(windows()).toHaveLength(1);
  const first = windows()[0][1];
  expect(first.count).toBeLessThanOrEqual(256);
  store.traj.state={trajectory_id:first.trajectory_id,active:true,completed:false,elapsed_ms:10000};
  await vi.advanceTimersByTimeAsync(100);
  expect(windows()).toHaveLength(1);
  let delivered=1; // 首点已经由对齐 action 完成。
  for (let i=0;delivered<700;i++) {
    const window=windows()[i][1];
    expect(window.count).toBeLessThanOrEqual(256);
    expect(window.points.every((p:any)=>p.time_from_start_ms>0)).toBe(true);
    delivered+=window.count;
    // JTC 容许 0.005 的末点误差；下一窗不得用旧目标强制校验 t=0 连续性。
    store.motors.get(1).motion.position=window.points.at(-1).position-.005;
    store.traj.state={trajectory_id:window.trajectory_id,active:false,completed:true,
      elapsed_ms:window.points.at(-1).time_from_start_ms};
    await vi.advanceTimersByTimeAsync(50);
  }
  await running;
  expect(delivered).toBe(700);
  expect(windows().at(-1)![1].points.at(-1).position).toBeCloseTo(.1699);
  expect(windows().length).toBeGreaterThan(4);
});

it('JTC cancel holds through the adapter without a manual target or global stop', async () => {
  const {actions} = await fixture();
  actions.stopPlayback();
  expect(pub.mock.calls).toEqual([['trajectory',expect.objectContaining({count:0,target_ids:[],points:[]})]]);
});

it('explicit soft stop stops the controllers and prevents alignment from restarting playback', async () => {
  const {actions,command} = await fixture();
  const running=actions.playAction('long.json',{enable:false,loop:false,kp:30,kd:1});
  await vi.advanceTimersByTimeAsync(1000);
  expect(pub.mock.calls.some(([key,msg])=>key==='trajectory' && msg.count>0)).toBe(true);
  expect(command.sendStop()).toBe(true);
  expect(pub.mock.calls).toContainEqual(['stop',{}]);
  const count=pub.mock.calls.length;
  await vi.runAllTimersAsync(); await running;
  expect(pub).toHaveBeenCalledTimes(count);
  expect(pub.mock.calls.filter(([key,msg])=>key==='trajectory' && msg.count>0)).toHaveLength(1);
  expect(pub.mock.calls.some(([key])=>key==='setpoint')).toBe(false);
});

it('a JTC failure or rejection never advances to the next window', async () => {
  const {actions,store} = await fixture();
  const running=actions.playAction('long.json',{enable:false,loop:false,kp:30,kd:1});
  await vi.advanceTimersByTimeAsync(1000);
  const windows=()=>pub.mock.calls.filter(([key,msg])=>key==='trajectory' && msg.count>0);
  store.traj.state={trajectory_id:windows()[0][1].trajectory_id,active:false,completed:false,elapsed_ms:0};
  await vi.advanceTimersByTimeAsync(50);await running;
  expect(windows()).toHaveLength(1);
});

it('manual P/V/E inputs still submit a single low-frequency setpoint', async () => {
  const {command}=await fixture();
  const targets=[{motor_id:1,position:.2,velocity:.3,effort:.4}];
  expect(command.sendTargets(targets).ok).toBe(true);
  expect(pub.mock.calls).toEqual([['setpoint',expect.objectContaining({count:1,targets})]]);
});

it.each(['commandStatus','transmissionStatus'])('rejects model B with the same motor ID for manual input and playback: %s',async key=>{
  const {store,actions,command}=await fixture();
  store[key].data.model='model_B';
  expect(command.sendTargets([{motor_id:1,position:.2,velocity:0,effort:0}]).ok).toBe(false);
  await actions.playAction('long.json',{enable:true,loop:false,kp:30,kd:1});
  expect(pub).not.toHaveBeenCalled();
});

it.each(['commandStatus','transmissionStatus'])('rejects stale or inconsistent control identity before publishing: %s',async key=>{
  const {store,command}=await fixture();
  const targets=[{motor_id:1,position:.2,velocity:0,effort:0}];
  store[key].last=7000;
  expect(command.sendTargets(targets).ok).toBe(false);
  store[key].last=10000;store[key].data.joint_names=['other_joint'];
  expect(command.sendTargets(targets).ok).toBe(false);
  expect(pub).not.toHaveBeenCalled();
});

it.each(['page','controlStatus'])('a model switch during a JTC wait invalidates playback before the next window: %s',async source=>{
  const {data,store,actions}=await fixture();
  const running=actions.playAction('long.json',{enable:false,loop:false,kp:30,kd:1});
  await vi.advanceTimersByTimeAsync(1000);
  const alignment=pub.mock.calls.find(([key,msg])=>key==='trajectory' && msg.count>0)![1];
  store.traj.state={trajectory_id:alignment.trajectory_id,active:false,completed:true,elapsed_ms:2500};
  await vi.advanceTimersByTimeAsync(50);
  const first=pub.mock.calls.filter(([key,msg])=>key==='trajectory' && msg.count>0)[1][1];
  const published=pub.mock.calls.length;
  vi.stubGlobal('fetch',async()=>({ok:false,status:404}));
  vi.spyOn(console,'warn').mockImplementation(()=>{});
  if(source==='page') data.setModel('model_B');
  else store.commandStatus.data.model='model_B';
  store.traj.state={trajectory_id:first.trajectory_id,active:false,completed:true,
    elapsed_ms:first.points.at(-1).time_from_start_ms};
  await vi.advanceTimersByTimeAsync(50);await running;
  expect(pub).toHaveBeenCalledTimes(published);
  vi.restoreAllMocks();
});

it.each([1,62])('single-point actions and final one-point windows have positive duration: %s samples',async length=>{
  const {store,actions}=await fixture();
  vi.stubGlobal('fetch',async()=>({ok:true,json:async()=>({target_ids:[1],points:Array.from({length},(_,i)=>
    ({motor_id:1,time_from_start_ms:i*10,position:.1+i*.0001}))})}));
  const running=actions.playAction('short.json',{enable:false,loop:false,kp:30,kd:1});
  await vi.advanceTimersByTimeAsync(1000);
  const windows=()=>pub.mock.calls.filter(([key,msg])=>key==='trajectory' && msg.count>0).map(([,msg])=>msg);
  let delivered=0,index=0;
  while(delivered<length){
    const window=windows()[index++];
    expect(window.count).toBeGreaterThan(0);expect(window.count).toBeLessThanOrEqual(256);
    expect(window.points.every((p:any)=>p.time_from_start_ms>0)).toBe(true);
    delivered+=window.count;
    store.traj.state={trajectory_id:window.trajectory_id,active:false,completed:true,
      elapsed_ms:window.points.at(-1).time_from_start_ms};
    await vi.advanceTimersByTimeAsync(50);
  }
  await running;
  expect(delivered).toBe(length);
  expect(windows().at(-1).count).toBe(1);
});

it('invalid status clears the identity and cannot refresh an old controller session', async () => {
  const {data,store}=await fixture();
  data.applyCommandStatus({model:data.MODEL,joint_names:['elbow'],mode:'trajectory',
    output_topic:'/motor/host_command_array',
    trajectory_output_topic:'/host_control/trajectory_controller/follow_joint_trajectory',
    gravity_enabled:true,effort_gain:.8});
  expect(store.commandStatus.last).toBe(10000);
  data.applyCommandStatus({...store.commandStatus.data,gravity_enabled:null,effort_gain:null});
  expect(store.commandStatus.data?.mode).toBe('trajectory');
  data.applyCommandStatus({output_topic:'/motor/command_array'});
  expect(store.commandStatus).toEqual({last:0,data:null});
});
