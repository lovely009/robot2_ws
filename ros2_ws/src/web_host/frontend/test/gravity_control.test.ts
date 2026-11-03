import {afterEach, beforeEach, expect, it, vi} from 'vitest';

const mocks = vi.hoisted(() => ({start:vi.fn(), stop:vi.fn(), generation:vi.fn()}));
vi.mock('../../web/js/core/agent.js', () => ({startGravityFeedforward:mocks.start, stopGravityFeedforward:mocks.stop}));
vi.mock('../../web/js/core/ros.js', () => ({getConnectionGeneration:mocks.generation}));

beforeEach(() => {
  vi.resetModules(); vi.useFakeTimers(); vi.setSystemTime(10000); vi.clearAllMocks();
  vi.stubGlobal('location', {search:''});
  vi.stubGlobal('localStorage', {getItem:()=>null, setItem:()=>{}});
  mocks.generation.mockReturnValue(1);
});
afterEach(() => {vi.useRealTimers(); vi.unstubAllGlobals();});

async function fixture() {
  const data = await import('../../web/js/core/store.js');
  const api = await import('../../web/js/core/gravity_control.js');
  const s = data.store, model = data.MODEL;
  s.conn.connected = true; s.runtimeMode = 'bench';
  s.modelTransmissions = {model,groups:[{name:'arm',joints:['joint'],motors:[{id:1}]}]};
  s.transmissionStatus = {last:10000,data:{model,joint_names:['joint'],trajectory_enabled:true}};
  s.commandStatus = {last:10000,data:{model,joint_names:['joint'],mode:'stopped',
    gravity_enabled:false,effort_gain:1,active_controllers:[],feedback_fresh:true,controllers_ready:true,
    output_topic:'/motor/host_command_array',trajectory_output_topic:'/host_control/trajectory_controller/follow_joint_trajectory'}};
  s.bench.gravity_feedforward = {fresh:true,pending:null,error:'',config:{model,effort_gain:1}};
  s.bench.gravity_feedforward_last=10000;
  mocks.start.mockImplementation(async () => {s.bench.gravity_feedforward.pending='start';});
  mocks.stop.mockImplementation(async () => {s.bench.gravity_feedforward.pending='stop';});
  const confirm = (enabled:boolean) => {
    s.commandStatus.last=Date.now();s.commandStatus.data.gravity_enabled=enabled;
    s.commandStatus.data.effort_gain=enabled?1:0;
    s.bench.gravity_feedforward.pending=null;
    s.bench.gravity_feedforward_last=Date.now();
  };
  return {data,s,model,api,confirm};
}

it('distinguishes configured, active, unknown and incomplete model feedback', async () => {
  const {s,model,api} = await fixture();
  expect(api.describeGravityControl(s,model)).toMatchObject({enabled:false,active:false,blocked:''});
  s.commandStatus.data.gravity_enabled=true;
  expect(api.describeGravityControl(s,model)).toMatchObject({enabled:true,active:false});
  for (const mode of ['follow','trajectory']) {
    s.commandStatus.data.mode=mode;
    s.commandStatus.data.active_controllers=[mode+'_controller','gravity_controller'];
    expect(api.describeGravityControl(s,model).active).toBe(true);
    s.commandStatus.data.active_controllers=[mode+'_controller'];
    expect(api.describeGravityControl(s,model).active).toBe(false);
  }
  expect(api.describeGravityControl(s,model,12501).enabled).toBeNull();
  s.bench.gravity_feedforward.pending='start';
  expect(api.describeGravityControl(s,model,12501).pending).toBe(false);
  expect(api.describeGravityControl(s,'other').enabled).toBeNull();
  s.modelTransmissions.groups.push({name:'missing',joints:['missing'],motors:[{id:2}]});
  expect(api.describeGravityControl(s,model).blocked).toContain('全部关节反馈');
  s.conn.connected=false;
  expect(api.describeGravityControl(s,model).enabled).toBeNull();
});

it('only explicit changes write parameters and acceptance alone is not success', async () => {
  const {s,model,api,confirm} = await fixture();
  api.describeGravityControl(s,model);
  expect(mocks.start).not.toHaveBeenCalled();
  let done=false;
  const pending=api.setGravityEnabled(true).then(()=>{done=true;});
  await vi.advanceTimersByTimeAsync(100);
  expect(mocks.start).toHaveBeenCalledExactlyOnceWith({model,joint_map:'',effort_gain:1});
  expect(done).toBe(false);
  s.commandStatus.last=Date.now();s.commandStatus.data.gravity_enabled=true;
  await vi.advanceTimersByTimeAsync(100);
  expect(done).toBe(false);
  confirm(true);await vi.advanceTimersByTimeAsync(50);await pending;
  const stopping=api.setGravityEnabled(false);
  await vi.advanceTimersByTimeAsync(100);
  expect(mocks.stop).toHaveBeenCalledTimes(1);
  confirm(false);await vi.advanceTimersByTimeAsync(50);await stopping;
});

it.each(['stale','model','subset','readonly','not_ready'])('rejects unsafe enable before calling services: %s', async kind => {
  const {s,api} = await fixture();
  if(kind==='stale') s.commandStatus.last=1;
  if(kind==='model') s.commandStatus.data.model='other';
  if(kind==='subset') s.modelTransmissions.groups.push({name:'missing',joints:['missing'],motors:[{id:2}]});
  if(kind==='readonly') s.runtimeMode='readonly';
  if(kind==='not_ready') s.commandStatus.data.controllers_ready=false;
  await expect(api.setGravityEnabled(true)).rejects.toThrow();
  expect(mocks.start).not.toHaveBeenCalled();
});

it.each(['rejection','timeout','disconnect','model','identity'])('never retries or confirms an interrupted request: %s', async kind => {
  const {s,api,confirm} = await fixture();
  const result=api.setGravityEnabled(true).catch(error=>error.message);
  await vi.advanceTimersByTimeAsync(100);
  if(kind==='rejection') s.bench.gravity_feedforward.error='controller refused';
  if(kind==='disconnect') {s.conn.connected=false;mocks.generation.mockReturnValue(2);}
  if(kind==='model') s.commandStatus.data.model='other';
  if(kind==='identity') s.commandStatus.data.joint_names=['other'];
  await vi.advanceTimersByTimeAsync(4100);
  expect(await result).toContain(kind==='rejection' ? 'controller refused'
    : kind==='timeout' ? '超时' : '控制会话已更换');
  confirm(true);await vi.advanceTimersByTimeAsync(100);
  expect(mocks.start).toHaveBeenCalledTimes(1);
});
