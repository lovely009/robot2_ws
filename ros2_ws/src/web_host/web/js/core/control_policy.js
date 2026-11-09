import { store } from './store.js';
import { controlSessionError } from './joint_control.js';
import { describeMotorState } from './motor_state.js';

export function isUnprotected(id) {
  const protect = Number(store.paramCache.get(id)?.pick?.limit_protect);
  return Number.isFinite(protect) && protect === 0;
}

export function jointPolicyTargets(state, model, enabled, now = Date.now()) {
  if (!state.conn.connected) throw new Error('未连接 rosbridge');
  if (enabled) {
    const error = controlSessionError(state, model, now);
    if (error) throw new Error(error);
  }
  const metadata = state.modelTransmissions;
  const names = state.transmissionStatus?.data?.joint_names;
  if (metadata?.model !== model || !Array.isArray(metadata.groups) || !Array.isArray(names) || !names.length) {
    throw new Error('没有可控制的关节会话');
  }
  const groups = metadata.groups.filter(group => group.joints.some(name => names.includes(name)));
  if (!groups.length || groups.some(group => !group.joints.every(name => names.includes(name)))) {
    throw new Error('控制会话必须包含完整传动组');
  }
  const ids = [...new Set(groups.flatMap(group => group.motors.map(motor => Number(motor.id))))].sort((a,b)=>a-b);
  if (!ids.length || ids.length > 48 || ids.some(id => !Number.isInteger(id) || id < 1 || id > 64)) {
    throw new Error('控制会话的电机 ID 无效');
  }
  return ids.map(id => {
    if (!enabled) return {motor_id:id,enabled:false,kp:0,kd:0,confirm_unprotected:false};
    const actual = describeMotorState(state,id,now);
    if (!actual.online || actual.fault) throw new Error(`m${id} 离线或存在故障`);
    const runtime = state.transmissionStatus.data;
    const defaults = Object.hasOwn(runtime, 'policy_defaults') ? runtime.policy_defaults : metadata.policy_defaults;
    const gains = defaults?.[id];
    if (![gains?.kp,gains?.kd].every(value => Number.isFinite(value) && value >= 0)) {
      throw new Error(`模型缺少 m${id} 的有效增益`);
    }
    return {motor_id:id,enabled:true,kp:gains.kp,kd:gains.kd,confirm_unprotected:false};
  });
}
