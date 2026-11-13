import {expect,it,vi} from 'vitest';
vi.stubGlobal('location',{search:''});
vi.stubGlobal('localStorage',{getItem:()=>null});
const api=await import('../../web/js/core/control_policy.js');
const motor=await import('../../web/js/core/motor_state.js');
function state(){
  const cache=motor.createControlStateCache();
  motor.applyControlState(cache,{schema_version:2,session_id:100,sequence:1,states:[{
    motor_id:2,online:true,control_lifecycle:2,execution_state:1,policy_enabled:false,
    motion_enabled:false,state_age_ms:5,unified_fault_code:0,vendor_fault_code:0,last_failure_reason:0,policy_generation:1}]},1000);
  return {conn:{connected:true},policy:new Map(),motors:new Map([[2,{}]]),controlStates:cache,
    modelTransmissions:{model:'arm',policy_defaults:{2:{kp:30,kd:1}},groups:[
      {name:'direct',joints:['elbow'],motors:[{id:2}]},
      {name:'wrist',joints:['yaw','pitch'],motors:[{id:8},{id:9}]}]},
    transmissionStatus:{last:1000,data:{model:'arm',trajectory_enabled:true,joint_names:['elbow']}},
    commandStatus:{last:1000,data:{model:'arm',joint_names:['elbow'],output_topic:'/motor/host_command_array',
      trajectory_output_topic:'/host_control/trajectory_controller/follow_joint_trajectory'}}};
}
it('enables only the actual complete control subset using model gains',()=>{
  expect(api.jointPolicyTargets(state(),'arm',true,1100)).toEqual([
    {motor_id:2,enabled:true,kp:30,kd:1,confirm_unprotected:false}]);
});
it('topology metadata blocks enable but preserves disable even with fresh host status',()=>{
  const s=state();
  Object.assign(s.modelTransmissions,{hardware_ready:false,hardware_block_reason:'传动参数待标定'});
  expect(()=>api.jointPolicyTargets(s,'arm',true,1100)).toThrow('待标定');
  expect(api.jointPolicyTargets(s,'arm',false,1100)[0].enabled).toBe(false);
});
it('does not enable disconnected, stale, faulted or incomplete group sessions',()=>{
  const s=state();s.conn.connected=false;expect(()=>api.jointPolicyTargets(s,'arm',true,1100)).toThrow();
  s.conn.connected=true;expect(()=>api.jointPolicyTargets(s,'arm',true,4000)).toThrow();
  s.controlStates.byId.get(2).unified_fault_code=2;expect(()=>api.jointPolicyTargets(s,'arm',true,1100)).toThrow();
  s.controlStates.byId.get(2).unified_fault_code=0;
  s.transmissionStatus.data.joint_names=['yaw'];s.commandStatus.data.joint_names=['yaw'];
  expect(()=>api.jointPolicyTargets(s,'arm',true,1100)).toThrow(/完整/);
});
it('allows disable with stale readiness and does not require gains',()=>{
  const s=state();s.policy.set(2,{kp:999,kd:99});
  expect(api.jointPolicyTargets(s,'arm',true,1100)[0].kp).toBe(30);
  delete s.modelTransmissions.policy_defaults[2];
  expect(api.jointPolicyTargets(s,'arm',false,4000)).toEqual([
    {motor_id:2,enabled:false,kp:0,kd:0,confirm_unprotected:false}]);
  expect(()=>api.jointPolicyTargets(s,'arm',true,1100)).toThrow(/增益/);
});

it('uses runtime driver gains when the offline model has none',()=>{
  const s=state();s.modelTransmissions.policy_defaults={};
  Object.assign(s.transmissionStatus.data,{policy_defaults:{2:{kp:12,kd:.5}}});
  expect(api.jointPolicyTargets(s,'arm',true,1100)[0]).toMatchObject({kp:12,kd:.5});
});
it('rejects missing or invalid runtime gains rather than falling back',()=>{
  const s=state();
  for (const policy_defaults of [{},{2:{kp:NaN,kd:1}},{2:{kp:30,kd:-1}}]) {
    Object.assign(s.transmissionStatus.data,{policy_defaults});
    expect(()=>api.jointPolicyTargets(s,'arm',true,1100)).toThrow(/增益/);
  }
});
