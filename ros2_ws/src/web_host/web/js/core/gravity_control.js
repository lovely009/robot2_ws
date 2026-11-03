import {store, MODEL} from './store.js';
import {controlSessionError} from './joint_control.js';
import {startGravityFeedforward, stopGravityFeedforward} from './agent.js';
import {getConnectionGeneration} from './ros.js';

export function describeGravityControl(state, model, now = Date.now()) {
  const report = state.commandStatus, host = report?.data;
  const known = state.conn.connected && report?.last > 0
    && now - report.last >= 0 && now - report.last <= 2500 && host?.model === model
    && typeof host.gravity_enabled === 'boolean' && Number.isFinite(host.effort_gain);
  const managerAge = now - state.bench.gravity_feedforward_last;
  const managerFresh = state.bench.gravity_feedforward_last > 0 && managerAge >= 0 && managerAge <= 2500;
  let blocked = state.runtimeMode !== 'bench' ? '当前页面只读' : controlSessionError(state, model, now);
  if (!blocked && (!known || host.effort_gain < 0 || host.effort_gain > 1)) blocked = '主机重力配置未就绪';
  if (!blocked && host.controllers_ready !== true) blocked = '主机控制器未就绪';
  if (!blocked && host.feedback_fresh !== true) blocked = '主机关节反馈未就绪';
  if (!blocked && !managerFresh) blocked = '重力补偿服务状态已过期';
  const joints = state.modelTransmissions?.groups?.flatMap(group => group.joints) || [];
  if (!blocked && joints.some(joint => !host.joint_names.includes(joint))) {
    blocked = '重力补偿需要当前模型全部关节反馈';
  }
  const enabled = known ? host.gravity_enabled : null;
  const active = enabled === true && ['follow', 'trajectory'].includes(host.mode)
    && host.active_controllers?.includes(host.mode + '_controller')
    && host.active_controllers?.includes('gravity_controller');
  return {enabled, active: Boolean(active), blocked,
    pending: Boolean(managerFresh && state.bench.gravity_feedforward?.pending)};
}

let busy = false;
export async function setGravityEnabled(enabled) {
  if (typeof enabled !== 'boolean') throw new Error('重力补偿开关必须为布尔值');
  const view = describeGravityControl(store, MODEL);
  if (busy || view.pending) throw new Error('重力补偿请求尚未完成');
  if (!store.conn.connected || store.runtimeMode !== 'bench') throw new Error('未连接或当前页面只读');
  if (view.enabled === null || (enabled && view.blocked)) throw new Error(view.blocked || '主机重力配置未就绪');
  const model = MODEL, generation = getConnectionGeneration();
  const identity = JSON.stringify(store.commandStatus.data.joint_names);
  const started = Date.now();
  busy = true;
  try {
    if (enabled) await startGravityFeedforward({model, joint_map:'', effort_gain:1});
    else await stopGravityFeedforward();
    while (Date.now() - started < 4000) {
      const report = store.commandStatus, manager = store.bench.gravity_feedforward;
      if (!store.conn.connected || generation !== getConnectionGeneration() || MODEL !== model
          || report.data?.model !== model || JSON.stringify(report.data.joint_names) !== identity) {
        throw new Error('控制会话已更换，未确认重力补偿结果');
      }
      if (manager?.error) throw new Error(manager.error);
      const current = describeGravityControl(store, model);
      if (manager?.fresh && !manager.pending && store.bench.gravity_feedforward_last > started
          && Date.now() - store.bench.gravity_feedforward_last <= 2500 && manager.config?.model === model
          && report.last > started && current.enabled === enabled
          && report.data.effort_gain === (enabled ? 1 : 0)) return;
      await new Promise(resolve => setTimeout(resolve, 50));
    }
    throw new Error('等待主机确认重力补偿超时，请核对主机状态');
  } finally { busy = false; }
}
