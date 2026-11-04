import { afterEach, beforeEach, expect, it, vi } from 'vitest';

const { pub, submitPolicy, connection } = vi.hoisted(() => ({pub: vi.fn((..._args:any[]) => true),
  submitPolicy:vi.fn(),
  connection:{generation:1,connected:true}}));
vi.mock('../../web/js/core/ros.js', () => ({
  pub, isConnected: () => connection.connected, getConnectionGeneration: () => connection.generation,
  submitPolicy,
}));
beforeEach(() => {
  connection.generation=1;connection.connected=true;
  vi.resetModules(); pub.mockReset(); pub.mockReturnValue(true);
  submitPolicy.mockReset();submitPolicy.mockResolvedValue({accepted:true,session_id:12,intent_id:'host:1',request_id:7,policy_generation:1});vi.useFakeTimers(); vi.setSystemTime(10000);
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
  store.controlStates.sourceTimeNs=9000000000;
  store.modelTransmissions = {model: MODEL, groups: [
    {name:'direct',motors:[{id:1}],joints:['elbow'],direct_mapping:true}],error:''};
  store.transmissionStatus={last:Date.now(),data:{model:MODEL,joint_names:['elbow'],trajectory_enabled:true}};
  data.applyCommandStatus({model:MODEL,joint_names:['elbow'],mode:'trajectory',
    output_topic:'/motor/host_command_array',
    trajectory_output_topic:'/host_control/trajectory_controller/follow_joint_trajectory',
    gravity_enabled:false,effort_gain:1});
  store.motors.set(1,{status:{online:true,lifecycle:4},motion:{position:.1}});
  const action = {target_ids:[1],points:Array.from({length:700},(_,i) =>
    ({motor_id:1,time_from_start_ms:i*10,position:.1+i*.0001}))};
  vi.stubGlobal('fetch', async () => ({ok:true,json:async () => action}));
  return {data, store, actions:await import('../../web/js/panels/actionLibrary.js'),
    command:await import('../../web/js/panels/actionLibrary.js')};
}

async function policyFixture() {
  const context = await fixture();
  const {store,actions}=context;
  const elements=new Map<string,any>();
  const element=()=>({textContent:'',className:'',checked:false,
    addEventListener() {},querySelectorAll:()=>[],replaceChildren() {},append() {}});
  vi.stubGlobal('document',{getElementById(id:string) {
    if(!elements.has(id))elements.set(id,element());return elements.get(id);
  },createElement:element});
  const panel=actions.mount(null,{});
  store.controlStates={seen:true,valid:true,at:Date.now(),sourceTimeNs:9000000000,session:12,sequence:1,
    byId:new Map([[1,{motor_id:1,online:true,control_lifecycle:2,execution_state:1,
      policy_enabled:false,motion_enabled:false,state_age_ms:0,unified_fault_code:0,
      vendor_fault_code:0,last_failure_reason:0,policy_generation:0}]])};
  const sent=()=>pub.mock.calls.filter(([key,msg])=>key==='trajectory' && msg.count>0);
  const text=()=>{panel.render();return elements.get('actResult').textContent;};
  const receipt=(changes={})=>({session_id:12,request_id:7,intent_id:'host:1',kind:2,command:0,
    status:2,error_code:10,failed_targets:1,completed_targets:0,...changes});
  const confirm=(changes={})=>{
    store.controlStates.at=Date.now();
    Object.assign(store.controlStates.byId.get(1),{control_lifecycle:3,policy_enabled:true,
      policy_generation:1,...changes});
  };
  const finish=async(running:Promise<void>)=>{
    const message=sent()[0]![1];
    store.traj.state={trajectory_id:message.trajectory_id,active:false,completed:true,elapsed_ms:9490};
    await vi.advanceTimersByTimeAsync(50);await running;
  };
  return {...context,sent,text,receipt,confirm,finish};
}

it('rejects a failed enable publication immediately without writing an enabled draft',async()=>{
  const {store,actions,sent,text}=await policyFixture();
  submitPolicy.mockRejectedValue(new Error('使能请求未发送'));
  const running=actions.playAction('long.json',{enable:true,loop:false,kp:30,kd:1});
  await vi.advanceTimersByTimeAsync(900);
  expect(text()).toContain('使能请求未发送');
  expect(store.policy.has(1)).toBe(false);
  expect(sent()).toHaveLength(0);
  await running;
});

it.each([[2,'设备拒绝'],[3,'设备忙'],[4,'请求冲突']])(
  'shows terminal enable status %s instead of waiting for an impossible generation',async(status,label)=>{
    const {store,actions,sent,text,receipt}=await policyFixture();
    const running=actions.playAction('long.json',{enable:true,loop:false,kp:30,kd:1});
    await vi.advanceTimersByTimeAsync(900);
    store.configResults.set(7,receipt({status,error_code:21,failed_targets:1,completed_targets:0}));
    await vi.advanceTimersByTimeAsync(250);
    expect(text()).toContain(label);
    expect(text()).toContain('error_code=21');
    expect(text()).toContain('失败 1 项 / 完成 0 项');
    expect(sent()).toHaveLength(0);
    await running;
  });

it('successful enable ACK still requires fresh enabled state with the exact generation',async()=>{
  const {store,actions,sent,receipt,confirm,finish}=await policyFixture();
  const running=actions.playAction('long.json',{enable:true,loop:false,kp:30,kd:1});
  await vi.advanceTimersByTimeAsync(900);
  store.configResults.set(7,receipt({status:1,failed_targets:0,completed_targets:1}));
  await vi.advanceTimersByTimeAsync(250);
  expect(sent()).toHaveLength(0);
  confirm({policy_generation:0});
  await vi.advanceTimersByTimeAsync(50);
  expect(sent()).toHaveLength(0);
  confirm({state_age_ms:501});
  await vi.advanceTimersByTimeAsync(50);
  expect(sent()).toHaveLength(0);
  confirm({state_age_ms:0});
  await vi.advanceTimersByTimeAsync(50);
  expect(sent()).toHaveLength(3);
  await finish(running);
});

it.each([{session_id:11},{request_id:2},{intent_id:'old-host:1'},{kind:1},{command:3},{status:0}])(
  'ignores unrelated or pending enable receipts: %j',async changes=>{
    const {store,actions,sent,receipt,confirm,finish}=await policyFixture();
    const running=actions.playAction('long.json',{enable:true,loop:false,kp:30,kd:1});
    await vi.advanceTimersByTimeAsync(900);
    store.configResults.set(7,receipt(changes));
    confirm();
    await vi.advanceTimersByTimeAsync(250);
    expect(sent()).toHaveLength(0);
    store.configResults.set(7,receipt({status:1,failed_targets:0,completed_targets:1}));
    confirm();await vi.advanceTimersByTimeAsync(50);
    expect(sent()).toHaveLength(3);await finish(running);
  });

it.each(['stop','connection','session'])('迟到的使能RPC响应不能越过%s边界发轨迹',async change=>{
  const {store,actions,sent}=await policyFixture();
  let respond!:(ticket:any)=>void;
  submitPolicy.mockImplementation(()=>new Promise(resolve=>{respond=resolve}));
  const running=actions.playAction('long.json',{enable:true,loop:false,kp:30,kd:1});
  await vi.advanceTimersByTimeAsync(900);
  expect(submitPolicy).toHaveBeenCalledTimes(1);
  if(change==='stop')actions.stopPlayback();
  if(change==='connection')connection.generation++;
  if(change==='session')store.controlStates.session=13;
  respond({accepted:true,session_id:12,intent_id:'host:1',request_id:7,policy_generation:1});
  await vi.advanceTimersByTimeAsync(50);await running;
  expect(sent()).toHaveLength(0);
  if(change!=='session')expect(store.policy.get(1)?.enabled).not.toBe(true);
});

it.each(['pending','failed'])('取消使能的动作播放也不能绕过本页%s策略意图',async phase=>{
  const {store,actions,sent,text,confirm}=await policyFixture();
  confirm();
  const {describeMotorState}=await import('../../web/js/core/motor_state.js');
  store.policy.set(1,{enabled:true,intent:{ids:[1],enabled:true,confirmed:false,
    error:phase==='failed'?'之前策略已失败':'',ticket:{session_id:12,intent_id:'host:1',request_id:7,policy_generation:1}}});
  const feedback=setInterval(()=>{store.controlStates.at=Date.now()},20);
  try {
    const running=actions.playAction('long.json',{enable:false,loop:false,kp:30,kd:1});
    await vi.advanceTimersByTimeAsync(1000);await running;
    expect(describeMotorState(store,1).canCommand).toBe(true);
    expect(sent()).toHaveLength(0);
    expect(submitPolicy).not.toHaveBeenCalled();
    expect(text()).toContain(phase==='failed'?'之前策略已失败':'本次使能/失能尚未确认');
  } finally {clearInterval(feedback);}
});

it('reports a superseding MCU policy generation without sending a trajectory',async()=>{
  const {actions,sent,text,confirm}=await policyFixture();
  const running=actions.playAction('long.json',{enable:true,loop:false,kp:30,kd:1});
  await vi.advanceTimersByTimeAsync(900);
  confirm({policy_generation:2});
  await vi.advanceTimersByTimeAsync(250);
  expect(text()).toContain('使能策略已被更新');
  expect(sent()).toHaveLength(0);
  await running;
});

it('uploads one continuous timeline including the approach, splitting only transport packets', async () => {
  const {store,actions} = await fixture();
  const running=actions.playAction('long.json',{enable:false,loop:false,kp:30,kd:1});
  await vi.advanceTimersByTimeAsync(1000);
  const segments=pub.mock.calls.filter(([key,msg])=>key==='trajectory' && msg.count>0).map(([,msg])=>msg);
  expect(segments).toHaveLength(3);
  const first=segments[0], points=segments.flatMap(msg=>msg.points);
  expect(points).toHaveLength(700);
  expect(points[0]).toMatchObject({time_from_start_ms:2500,position:.1,velocity:0,effort:0});
  expect(points.at(-1)).toMatchObject({time_from_start_ms:9490,position:.1699});
  segments.forEach((msg,index)=>{
    expect(msg).toMatchObject({session_id:10000,trajectory_id:first.trajectory_id,
      sequence:first.sequence,sync_time_ns:first.sync_time_ns,segment_index:index,segment_count:3});
    expect(msg.count).toBeLessThanOrEqual(256);
  });
  expect(new Set(points.map(p=>p.time_from_start_ms)).size).toBe(700);
  // The approach and packet boundaries remain interior points, not extra final-goal checks.
  store.traj.state={trajectory_id:first.trajectory_id,active:true,completed:false,elapsed_ms:2500};
  await vi.advanceTimersByTimeAsync(100);
  expect(pub.mock.calls.filter(([key,msg])=>key==='trajectory' && msg.count>0)).toHaveLength(3);
  store.traj.state={trajectory_id:first.trajectory_id,active:false,completed:true,elapsed_ms:9490};
  await vi.advanceTimersByTimeAsync(50);await running;
});

it('loops only after success of the entire trajectory and never after final failure',async()=>{
  const {store,actions}=await fixture();
  const running=actions.playAction('long.json',{enable:false,loop:true,kp:30,kd:1});
  await vi.advanceTimersByTimeAsync(1000);
  const sent=()=>pub.mock.calls.filter(([key,msg])=>key==='trajectory' && msg.count>0).map(([,msg])=>msg);
  expect(sent()).toHaveLength(3);
  const first=sent()[0];
  store.traj.state={trajectory_id:first.trajectory_id,active:false,completed:true,elapsed_ms:9490};
  await vi.advanceTimersByTimeAsync(50);
  expect(sent()).toHaveLength(6);
  const second=sent()[3];
  expect(second.trajectory_id).not.toBe(first.trajectory_id);
  store.traj.state={trajectory_id:second.trajectory_id,active:false,completed:false,elapsed_ms:9490};
  await vi.advanceTimersByTimeAsync(50);await running;
  expect(sent()).toHaveLength(6);
});

it('rejects oversized plans before enabling motors or sending any segment',async()=>{
  const {actions}=await fixture();
  vi.stubGlobal('fetch',async()=>({ok:true,json:async()=>({target_ids:[1],points:Array.from({length:8193},(_,i)=>
    ({motor_id:1,time_from_start_ms:i*10,position:.1}))})}));
  await actions.playAction('oversized.json',{enable:true,loop:false,kp:30,kd:1});
  expect(pub).not.toHaveBeenCalled();
});

it.each(['spline',null,false])('rejects unknown interpolation %s before enabling or publishing',async interpolation=>{
  const {actions}=await fixture();
  const action={interpolation,target_ids:[1],points:[{motor_id:1,time_from_start_ms:0,position:.1}]};
  vi.stubGlobal('fetch',async()=>({ok:true,json:async()=>action}));
  expect(()=>actions.loadSeries(action)).toThrow(/插值/);
  await actions.playAction('unknown.json',{enable:true,loop:false,kp:30,kd:1});
  expect(pub).not.toHaveBeenCalled();
});

it('stops uploading after a rejected publish instead of submitting the remaining segments',async()=>{
  const {actions}=await fixture();
  pub.mockImplementation((key:any,msg:any)=>!(key==='trajectory' && msg.count>0 && msg.segment_index===1));
  const running=actions.playAction('long.json',{enable:false,loop:false,kp:30,kd:1});
  await vi.advanceTimersByTimeAsync(1000);await running;
  expect(pub.mock.calls.filter(([key,msg])=>key==='trajectory' && msg.count>0)).toHaveLength(2);
  expect(pub.mock.calls.at(-1)).toEqual(['trajectory',expect.objectContaining({count:0,points:[],target_ids:[]})]);
});

it('首段发布前校验失败不追加取消，避免撤销其他客户端轨迹',async()=>{
  const {store,actions}=await fixture();
  const running=actions.playAction('long.json',{enable:false,loop:false,kp:30,kd:1});
  await vi.advanceTimersByTimeAsync(700);
  store.controlStates.sourceTimeNs=0;
  await vi.advanceTimersByTimeAsync(300);await running;
  expect(pub.mock.calls).toHaveLength(1);
  expect(pub.mock.calls[0]).toEqual(['trajectory',expect.objectContaining({count:0})]);
});

it.each(['monotone','linear'])('结果回执超时后撤销已提交的 %s 轨迹，不发送全部软停',async interpolation=>{
  const {store,actions}=await fixture();
  vi.stubGlobal('fetch',async()=>({ok:true,json:async()=>({interpolation,target_ids:[1],
    points:[{motor_id:1,time_from_start_ms:0,position:.1}]})}));
  const running=actions.playAction('short.json',{enable:false,loop:true,kp:30,kd:1});
  await vi.advanceTimersByTimeAsync(1000);
  const channel=interpolation==='linear'?'rawTrajectory':'trajectory';
  const sent=pub.mock.calls.find(([key,msg])=>key===channel && msg.count>0)![1];
  store.traj.state={trajectory_id:sent.trajectory_id,active:true,completed:false,elapsed_ms:2500};
  const heartbeat=setInterval(()=>{
    store.commandStatus.last=store.transmissionStatus.last=Date.now();
  },100);
  try {await vi.advanceTimersByTimeAsync(6000);await running;}
  finally {clearInterval(heartbeat);}
  expect(pub.mock.calls.filter(([,msg])=>msg.count>0)).toHaveLength(1);
  expect(pub.mock.calls.at(-1)).toEqual(['trajectory',expect.objectContaining({count:0,points:[],target_ids:[]})]);
  expect(pub.mock.calls.filter(([key,msg])=>key==='trajectory' && msg.count===0)).toHaveLength(2);
  expect(pub.mock.calls.some(([key])=>key==='stop')).toBe(false);
});

it.each(['rejected','throws'])('异常撤销发送失败（%s）仍结束播放并显示原始错误',async failure=>{
  const {actions,text,confirm}=await policyFixture();
  confirm();
  pub.mockImplementation((key:any,msg:any)=>{
    if(key==='trajectory' && msg.count>0) throw new Error('轨迹传输异常');
    if(pub.mock.calls.some(([,message])=>message.count>0)) {
      if(failure==='throws') throw new Error('取消传输异常');
      return false;
    }
    return true;
  });
  const running=actions.playAction('long.json',{enable:false,loop:false,kp:30,kd:1});
  await vi.advanceTimersByTimeAsync(700);confirm();
  await vi.advanceTimersByTimeAsync(300);
  await expect(running).resolves.toBeUndefined();
  expect(pub.mock.calls.at(-1)).toEqual(['trajectory',expect.objectContaining({count:0})]);
  expect(text()).toContain('轨迹传输异常');
  expect(text()).toContain('取消未发送');
});

it('JTC cancel holds through the adapter without a manual target or global stop', async () => {
  const {actions} = await fixture();
  actions.stopPlayback();
  expect(pub.mock.calls).toEqual([['trajectory',expect.objectContaining({count:0,target_ids:[],points:[]})]]);
});

it('explicit soft stop stops the controllers and prevents playback from restarting', async () => {
  const {actions,command} = await fixture();
  const running=actions.playAction('long.json',{enable:false,loop:false,kp:30,kd:1});
  await vi.advanceTimersByTimeAsync(1000);
  expect(pub.mock.calls.some(([key,msg])=>key==='trajectory' && msg.count>0)).toBe(true);
  expect(command.stopAll()).toBe(true);
  expect(pub.mock.calls).toContainEqual(['stop',{}]);
  const count=pub.mock.calls.length;
  await vi.runAllTimersAsync(); await running;
  expect(pub).toHaveBeenCalledTimes(count);
  expect(pub.mock.calls.filter(([key,msg])=>key==='trajectory' && msg.count>0)).toHaveLength(3);
  expect(pub.mock.calls.some(([key])=>key==='setpoint')).toBe(false);
});

it('a JTC failure or rejection never starts another trajectory', async () => {
  const {actions,store} = await fixture();
  const running=actions.playAction('long.json',{enable:false,loop:false,kp:30,kd:1});
  await vi.advanceTimersByTimeAsync(1000);
  const windows=()=>pub.mock.calls.filter(([key,msg])=>key==='trajectory' && msg.count>0);
  store.traj.state={trajectory_id:windows()[0][1].trajectory_id,active:false,completed:false,elapsed_ms:0};
  await vi.advanceTimersByTimeAsync(50);await running;
  expect(windows()).toHaveLength(3);
});

it.each(['reject','reason'])('shows the host %s when the complete trajectory fails', async field => {
  const {actions,store} = await fixture();
  const elements = new Map<string,any>();
  const element = () => ({textContent:'',className:'',checked:false,
    addEventListener() {},querySelectorAll:()=>[],replaceChildren() {},append() {}});
  vi.stubGlobal('document',{
    getElementById(id:string) {
      if (!elements.has(id)) elements.set(id,element());
      return elements.get(id);
    },
    createElement:element,
  });
  const panel=actions.mount(null,{});
  const running=actions.playAction('long.json',{enable:false,loop:false,kp:30,kd:1});
  await vi.advanceTimersByTimeAsync(1000);
  const alignment=pub.mock.calls.find(([key,msg])=>key==='trajectory' && msg.count>0)![1];
  const reason='JTC 中止/失败 -5: joint_link2 position error 0.027 exceeds 0.020';
  store.commandStatus.data={...store.commandStatus.data,[field]:reason};
  store.traj.state={trajectory_id:alignment.trajectory_id,active:false,completed:false,elapsed_ms:2500};
  await vi.advanceTimersByTimeAsync(50);await running;
  panel.render();
  expect(elements.get('actResult').textContent).toContain(reason);
  expect(pub.mock.calls.filter(([key,msg])=>key==='trajectory' && msg.count>0)).toHaveLength(3);
});

it.each(['commandStatus','transmissionStatus'])('rejects model B with the same motor ID for manual input and playback: %s',async key=>{
  const {store,actions,command}=await fixture();
  store[key].data.model='model_B';
  await actions.playAction('long.json',{enable:true,loop:false,kp:30,kd:1});
  expect(pub).not.toHaveBeenCalled();
});

it.each(['page','controlStatus','metadata'])('invalidating the active model cancels its complete trajectory: %s',async source=>{
  const {data,store,actions}=await fixture();
  const running=actions.playAction('long.json',{enable:false,loop:true,kp:30,kd:1});
  await vi.advanceTimersByTimeAsync(1000);
  const first=pub.mock.calls.find(([key,msg])=>key==='trajectory' && msg.count>0)![1];
  const published=pub.mock.calls.length;
  vi.stubGlobal('fetch',async()=>({ok:false,status:404}));
  vi.spyOn(console,'warn').mockImplementation(()=>{});
  if(source==='page') data.setModel('model_B');
  else if(source==='metadata') store.modelTransmissions={...store.modelTransmissions};
  else store.commandStatus.data.model='model_B';
  store.traj.state={trajectory_id:first.trajectory_id,active:true,completed:false,elapsed_ms:2500};
  await vi.advanceTimersByTimeAsync(50);await running;
  expect(pub).toHaveBeenCalledTimes(published+1);
  expect(pub.mock.calls.at(-1)).toEqual(['trajectory',expect.objectContaining({count:0,points:[],target_ids:[]})]);
  vi.restoreAllMocks();
});

it.each(['reconnected','disconnected'])('an old trajectory never sends cancel into a changed connection: %s',async mode=>{
  const {actions}=await fixture();
  const running=actions.playAction('long.json',{enable:false,loop:true,kp:30,kd:1});
  await vi.advanceTimersByTimeAsync(1000);
  const published=pub.mock.calls.length;
  if(mode==='reconnected') connection.generation++;
  else connection.connected=false;
  await vi.advanceTimersByTimeAsync(50);await running;
  expect(pub).toHaveBeenCalledTimes(published);
});

it.each([1,62])('single-point and short actions include every sample on a single timeline: %s samples',async length=>{
  const {store,actions}=await fixture();
  vi.stubGlobal('fetch',async()=>({ok:true,json:async()=>({target_ids:[1],points:Array.from({length},(_,i)=>
    ({motor_id:1,time_from_start_ms:100+i*10,position:.1+i*.0001,velocity:i===0?.05:0}))})}));
  const running=actions.playAction('short.json',{enable:false,loop:false,kp:30,kd:1});
  await vi.advanceTimersByTimeAsync(1000);
  const segments=pub.mock.calls.filter(([key,msg])=>key==='trajectory' && msg.count>0).map(([,msg])=>msg);
  expect(segments).toHaveLength(1);
  const plan=segments[0];
  expect(plan.count).toBe(length);
  expect(plan.points[0].time_from_start_ms).toBe(2500);
  expect(plan.points[0].velocity).toBe(0);
  expect(plan.points.at(-1).time_from_start_ms).toBe(2500+(length-1)*10);
  store.traj.state={trajectory_id:plan.trajectory_id,active:false,completed:true,
    elapsed_ms:plan.points.at(-1).time_from_start_ms};
  await vi.advanceTimersByTimeAsync(50);await running;
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


it('浏览器时钟偏移时，整条动作与分包仍只使用缓存MCU同步源时间',async()=>{
  vi.setSystemTime(777000);
  const {store,actions}=await fixture();
  const sourceTime=store.controlStates.sourceTimeNs;
  const running=actions.playAction('long.json',{enable:false,loop:false,kp:30,kd:1});
  await vi.advanceTimersByTimeAsync(1000);
  const segments=pub.mock.calls.filter(([key,message])=>key==='trajectory' && message.count>0).map(([,message])=>message);
  expect(segments).toHaveLength(3);
  expect(segments.every(message=>message.sync_time_ns===sourceTime)).toBe(true);
  expect(segments[0].sync_time_ns).not.toBe(Date.now()*1e6);
  actions.stopPlayback();await vi.advanceTimersByTimeAsync(50);await running;
});

it('没有MCU同步源时间时拒发非空动作，取消与STOP仍可发送零时间',async()=>{
  const {store,actions}=await fixture();
  store.controlStates.sourceTimeNs=0;
  const running=actions.playAction('long.json',{enable:false,loop:false,kp:30,kd:1});
  await vi.advanceTimersByTimeAsync(1000);
  try {expect(pub.mock.calls.filter(([key,message])=>key==='trajectory' && message.count>0)).toHaveLength(0);}
  finally {actions.stopAll();await vi.advanceTimersByTimeAsync(50);await running;}
  expect(actions.stopAll()).toBe(true);
  expect(pub.mock.calls.some(([key])=>key==='stop')).toBe(true);
  expect(pub.mock.calls.filter(([key])=>key==='trajectory').every(([,message])=>message.sync_time_ns===0)).toBe(true);
});
