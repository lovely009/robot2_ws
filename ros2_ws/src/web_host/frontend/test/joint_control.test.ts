import { afterEach, expect, it, vi } from 'vitest';
afterEach(() => vi.useRealTimers());
const api = await import(/* @vite-ignore */ new URL('../../web/js/core/joint_control.js', import.meta.url).href);
function state() {
  return {conn:{connected:true},modelTransmissions:{model:'arm',groups:[{name:'wrist',motors:[{id:8},{id:9}],joints:['yaw','pitch']}]},
    modelJoints: [{name:'yaw',type:'revolute',lower:-1,upper:1},{name:'pitch',type:'revolute',lower:-1,upper:1}],
    transmissionStatus:{last:1000,data:{model:'arm',trajectory_enabled:true,joint_names:['yaw','pitch']}},
    commandStatus:{last:1000,data:{model:'arm',joint_names:['yaw','pitch'],mode:'trajectory',
      output_topic:'/motor/host_command_array',
      trajectory_output_topic:'/host_control/trajectory_controller/follow_joint_trajectory',gravity_enabled:true}},
    jointStates:{byName:{yaw:{position:.2,at:1000},pitch:{position:.1,at:1000}}},
    motors:new Map([[8,{status:{online:true,lifecycle:4,unified_fault_code:0}}],[9,{status:{online:true,lifecycle:4,unified_fault_code:0}}]])};
}
it('uses a joint name and lets the host preserve and transform the partner', () => {
  const message=api.buildJointTrajectory(state(),'arm','yaw',.3,1.,1100);
  expect(message.joint_names).toEqual(['yaw']);
  expect(message.points[0].positions).toEqual([.3]);
  expect(message.points[0].time_from_start).toEqual({sec:1,nanosec:0});
  expect(message).not.toHaveProperty('motor_id');
});
it('accepts JTC with gravity and rejects old control sources, identity changes and stale status', () => {
  const s=state();
  expect(api.jointControlError(s,'arm','yaw',1100)).toBe('');
  s.commandStatus.data.model='previous';
  expect(api.jointControlError(s,'arm','yaw',1100)).toContain('模型');
  s.commandStatus.data.model='arm';
  s.commandStatus.data.joint_names=['yaw'];
  expect(api.jointControlError(s,'arm','yaw',1100)).toContain('关节');
  s.commandStatus.data.joint_names=['yaw','pitch'];
  s.commandStatus.data.trajectory_output_topic='/motor/trajectory';
  expect(api.jointControlError(s,'arm','yaw',1100)).toContain('路由');
  s.commandStatus.data.trajectory_output_topic='/host_control/trajectory_controller/follow_joint_trajectory';
  s.commandStatus.last=4000;
  expect(api.jointControlError(s,'arm','yaw',1100)).toBeTruthy();
  s.commandStatus.last=1;
  s.transmissionStatus.last=3000;
  expect(api.jointControlError(s,'arm','yaw',3000)).toBeTruthy();
});
it('rejects wrong model, readonly bridge, stale or partial group', () => {
  const s=state();
  expect(api.jointControlError(s,'other','yaw',1100)).toBeTruthy();
  s.transmissionStatus.data.trajectory_enabled=false;
  expect(api.jointControlError(s,'arm','yaw',1100)).toContain('只读');
  s.transmissionStatus.data.trajectory_enabled=true;
  expect(api.jointControlError(s,'arm','yaw',4000)).toBeTruthy();
  delete s.jointStates.byName.pitch;
  expect(api.jointControlError(s,'arm','yaw',1100)).toBeTruthy();
});
it('raw motor controls cannot bypass the configured coupled group', () => {
  const s=state();
  expect(api.rawCouplingError(s,'arm',[8])).toContain('wrist');
  expect(api.rawCouplingError(s,'arm',[8,9])).toContain('关节');
  expect(api.rawCouplingError(s,'arm',[])).toBe('');
});
it('slider follow time bounds the cubic peak speed and uses a short horizon', () => {
  expect(api.jointSliderDuration({velocity:2}, -3, 3)).toBe(4.5);
  expect(api.jointSliderDuration({velocity:2}, 0, .1)).toBe(.15);
});
it('accepts a first joint target from a ready motor after an explicit enable policy', () => {
  const s = {...state(), policy: new Map([[8, {enabled:true}], [9, {enabled:true}]])};
  for (const motor of s.motors.values()) motor.status.lifecycle = 3;
  expect(api.jointControlError(s, 'arm', 'yaw', 1100)).toBe('');
  expect(api.buildJointTrajectory(s, 'arm', 'yaw', .3, 1, 1100).joint_names).toEqual(['yaw']);
  s.policy.set(9, {enabled:false});
  expect(api.jointControlError(s, 'arm', 'yaw', 1100)).toContain('使能');
});

it('slider submits only a future target so JTC starts at the receiving-time state', () => {
  const s=state();
  const message=api.buildJointSliderTrajectory(s,'arm',{name:'yaw',velocity:2},.3,1100);
  expect(message.joint_names).toEqual(['yaw']);
  expect(message.points).toHaveLength(1);
  expect(message.points[0]).toMatchObject({positions:[.3],time_from_start:{sec:0,nanosec:150000000}});
});
it('continuous slider sends immediately, coalesces rapid input and flushes the final value once', () => {
  vi.useFakeTimers();vi.setSystemTime(1000);
  const sent:number[]=[];
  const sender=api.createLatestTargetSender({send:(value:number)=>sent.push(value),isCurrent:()=>true});
  sender.update(.1,1); sender.update(.2,1); sender.update(.3,1);
  expect(sent).toEqual([.1]);
  vi.advanceTimersByTime(50);expect(sent).toEqual([.1,.3]);
  sender.update(.4,1); sender.update(.5,1,true);
  expect(sent).toEqual([.1,.3,.5]);
  sender.update(.5,1,true);vi.advanceTimersByTime(100);
  expect(sent).toEqual([.1,.3,.5]);sender.dispose();
});
it('queued slider targets expire on connection changes and cannot replay after cancel or dispose', () => {
  vi.useFakeTimers();vi.setSystemTime(1000);
  let connection=1;
  const sent:number[]=[];
  const sender=api.createLatestTargetSender({send:(value:number)=>sent.push(value),isCurrent:(token:number)=>token===connection});
  sender.update(.1,1);sender.update(.2,1);connection=2;
  vi.advanceTimersByTime(50);expect(sent).toEqual([.1]);
  sender.update(.3,2);sender.update(.4,2);sender.cancel();
  vi.advanceTimersByTime(100);expect(sent).toEqual([.1,.3]);
  sender.update(.5,2);sender.update(.6,2);sender.dispose();
  vi.advanceTimersByTime(100);sender.update(.7,2);
  expect(sent).toEqual([.1,.3,.5]);
});
