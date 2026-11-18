import { store } from './store.js';
import { describeMotorState } from './motor_state.js';

export function isUnprotected(id) {
  const protect = Number(store.paramCache.get(id)?.pick?.limit_protect);
  return Number.isFinite(protect) && protect === 0;
}

export function jointPolicyTargets(state, model, enabled, now = Date.now()) {
  if (!state.conn.connected) throw new Error('未连接 rosbridge');
  const metadata = state.modelTransmissions;
  if (metadata?.model !== model || metadata.error || !Array.isArray(metadata.groups)
      || !metadata.groups.length || metadata.groups.some(group => !Array.isArray(group?.motors) || !group.motors.length)) {
    throw new Error('模型电机映射未就绪');
  }
  if (enabled && [state.transmissionStatus?.data, state.commandStatus?.data]
      .some(status => status?.model && status.model !== model)) {
    throw new Error('页面模型与主机控制模型不一致，请等待模型切换完成');
  }
  const mapped = [...new Set(metadata.groups.flatMap(group => group.motors.map(motor => Number(motor.id))))];
  if (mapped.some(id => !Number.isInteger(id) || id < 1 || id > 64)) {
    throw new Error('模型电机 ID 无效');
  }
  // 使能按在线电机生成；失能覆盖已发现或曾请求使能的电机，允许掉线后撤权。
  const ids = mapped.filter(id => enabled ? describeMotorState(state,id,now).online
    : state.motors.has(id) || state.controlStates?.byId.has(id) || state.policy?.get(id)?.enabled)
    .sort((a,b)=>a-b);
  if (!ids.length) throw new Error('当前模型没有可控制的在线电机');
  if (ids.length > 48) throw new Error('单次使能策略最多支持 48 台电机');
  const runtime = state.transmissionStatus;
  const runtimeDefaults = runtime?.data?.model === model && Object.hasOwn(runtime.data, 'policy_defaults');
  if (enabled && runtimeDefaults && (!runtime.last || now-runtime.last < 0 || now-runtime.last > 2500)) {
    throw new Error('主机电机增益已过期');
  }
  const defaults = runtimeDefaults ? runtime.data.policy_defaults : metadata.policy_defaults;
  return ids.map(id => {
    if (!enabled) return {motor_id:id,enabled:false,kp:0,kd:0,confirm_unprotected:false};
    const actual = describeMotorState(state,id,now);
    if (!actual.online || actual.fault) throw new Error(`m${id} 离线或存在故障`);
    const gains = defaults?.[id];
    if (![gains?.kp,gains?.kd].every(value => Number.isFinite(value) && value >= 0)) {
      throw new Error(`模型缺少 m${id} 的有效增益`);
    }
    return {motor_id:id,enabled:true,kp:gains.kp,kd:gains.kd,confirm_unprotected:false};
  });
}
