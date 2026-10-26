import {expect,it} from 'vitest';
const api = await import(/* @vite-ignore */ new URL('../../web/js/core/motor_state.js', import.meta.url).href);
const motor = (changes = {}) => ({motor_id:1,online:true,control_lifecycle:3,execution_state:1,
  policy_enabled:true,motion_enabled:false,state_age_ms:10,unified_fault_code:0,
  vendor_fault_code:0,last_failure_reason:0,policy_generation:1,...changes});
const message = (changes = {}) => ({schema_version:1,session_id:100,sequence:1,states:[motor()],...changes});
function state() {return {conn:{connected:true},motors:new Map([[1,{status:{online:true,lifecycle:3},statusAt:1000}]]),
  policy:new Map([[1,{enabled:true}]]),controlStates:api.createControlStateCache()};}
it('MCU activation without a target is active and idle, not moving', () => {
  const s=state();api.applyControlState(s.controlStates,message(),1000);
  const view=api.describeMotorState(s,1,1100);
  expect(view.lifecycle).toContain('已使能');expect(view.execution).toBe('空闲');
  expect(view.active).toBe(true);expect(view.canCommand).toBe(true);
});
it('local policy edits cannot activate authoritative inactive or stale states', () => {
  const s=state();api.applyControlState(s.controlStates,message({states:[motor({control_lifecycle:2,policy_enabled:false})]}),1000);
  expect(api.describeMotorState(s,1,1100).canCommand).toBe(false);
  expect(api.describeMotorState(s,1,4000).lifecycle).not.toContain('已使能');
  expect(api.describeMotorState(s,1,4000).canCommand).toBe(false);
});
it('offline is separate from finalized and old firmware is explicitly marked', () => {
  const s=state();expect(api.describeMotorState(s,1,1100).protocol).toBe('旧版状态');
  api.applyControlState(s.controlStates,message({states:[motor({online:false,control_lifecycle:2})]}),1000);
  const view=api.describeMotorState(s,1,1100);
  expect(view.connection).toBe('离线');expect(view.lifecycle).not.toBe('已终止');expect(view.canCommand).toBe(false);
});
it('a complete snapshot drops missing members and old sessions cannot revive', () => {
  const c=api.createControlStateCache();expect(api.applyControlState(c,message(),1000)).toBe(true);
  expect(api.applyControlState(c,message({sequence:0}),1100)).toBe(false);expect(c.at).toBe(1000);
  expect(api.applyControlState(c,message({session_id:101,states:[]}),1200)).toBe(true);
  expect(c.byId.size).toBe(0);
  expect(api.applyControlState(c,message({sequence:9}),1300)).toBe(false);
});
it('invalid snapshots fail closed and do not restore legacy control', () => {
  const s=state();api.applyControlState(s.controlStates,message(),1000);
  expect(api.applyControlState(s.controlStates,message({sequence:2,states:[motor(),motor()]}),1100)).toBe(false);
  expect(s.controlStates.seen).toBe(true);expect(api.describeMotorState(s,1,1100).canCommand).toBe(false);
  expect(api.applyControlState(s.controlStates,message({schema_version:2}),1200)).toBe(false);
});
it('snapshot capacity does not restrict the configured uint16 motor identities', () => {
  const s=state();s.motors.set(120,{status:{online:true,lifecycle:3},statusAt:1000});
  expect(api.applyControlState(s.controlStates,message({states:[motor({motor_id:120})]}),1000)).toBe(true);
  expect(api.describeMotorState(s,120,1100).canCommand).toBe(true);
});
