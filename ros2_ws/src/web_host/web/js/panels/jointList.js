// 关节保持 URDF 顺序；反馈更新不替换正在操作的控件。
import { store, MODEL, DIRTY, markDirty } from '../core/store.js';
import { $ } from '../core/format.js';
import { createJointRow, STALE_MS } from '../components/jointRow.js';
import { buildJointTarget, jointControlError, controlSessionError } from '../core/joint_control.js';
import { jointLimitsParameter } from '../core/joint_limits.js';
import { pub, ensurePublisher, isConnected, getConnectionGeneration, callService, submitPolicy } from '../core/ros.js';

import { jointPolicyTargets, isUnprotected } from '../core/control_policy.js';
import { describeMotorState, policyRequestState, policyControlError } from '../core/motor_state.js';
import { stopAll } from './actionLibrary.js';

export { STALE_MS };

export function mount(root, ctx) {
  const list = $('jointRows'), summary = $('sumJoints');
  let entries = [], key = '', generation = -1, advertised = 0, pending = false, disposed = false;
  const toggle = root.querySelector('#jointControlEnabled');
  const toggleLabel = toggle.closest('label');
  const result = root.querySelector('#jointControlResult');
  let pendingPolicy = null, policyResult = '', submittingPolicy = false;
  result.textContent = '';
  const simulationSelected = () => {
    const config = store.bench.simulation?.config;
    return MODEL === 'full_body_robot' && config?.model === MODEL
      && config.backend === 'mujoco' && config.controller_mode === 'external';
  };
  const simulationSession = () => {
    const {data, last} = store.simPositionStatus;
    return simulationSelected() && store.bench.simulation.state === 'running'
      && Date.now()-last < 2000 && data?.model === MODEL
      && typeof data.session === 'string' && data.session ? data : null;
  };
  const simulationBounds = name => {
    if (!simulationSelected()) return null;
    const session = simulationSession();
    const bounds = session?.limits?.[name];
    if (!Array.isArray(bounds) || bounds.length !== 2 || !bounds.every(Number.isFinite)
        || bounds[0] >= bounds[1]) return {lower:null,upper:null};
    return {lower:bounds[0],upper:bounds[1],session:session.session};
  };
  const commandContext = () => `${getConnectionGeneration()}:${simulationSession()?.session || ''}`;
  const bridgeReady = () => store.conn.connected && store.transmissionStatus.data?.model === MODEL
    && Date.now()-store.transmissionStatus.last >= 0
    && Date.now()-store.transmissionStatus.last < 2500;
  toggle.disabled = true;
  toggle.closest('label').title = '仅对当前主机控制会话中的完整传动组下发使能；使用模型默认增益，勾选状态以 MCU 确认为准';
  toggle.closest('label').addEventListener('click', event => event.stopPropagation());
  toggle.onchange = async () => {
    if (simulationSelected() || submittingPolicy || pendingPolicy || disposed) return;
    const enabled = toggle.checked, model=MODEL, connection=getConnectionGeneration();
    try {
      const targets = jointPolicyTargets(store, MODEL, enabled);
      if (!enabled) {
        for (const entry of entries) entry.cancel();
        if (!stopAll()) throw new Error('停止请求未发送');
      }
      const unprotected = enabled ? targets.filter(target => isUnprotected(target.motor_id)) : [];
      if (unprotected.length && !window.confirm(`m${unprotected.map(target => target.motor_id).join('、m')} 的设备限位保护已关闭，确认使能？`)) {
        render(new Set([DIRTY.STATUS])); return;
      }
      for (const target of unprotected) target.confirm_unprotected = true;
      submittingPolicy=true;
      render(new Set([DIRTY.STATUS]));
      const ticket=await submitPolicy(targets);
      if (disposed || model!==MODEL || connection!==getConnectionGeneration()) throw new Error('连接或模型已变化，使能请求失效');
      for (const target of targets) {
        const previous = store.policy.get(target.motor_id) || {};
        store.policy.set(target.motor_id, {...previous,...(enabled ? target : {}),enabled,
          requestId:ticket.request_id,generation:ticket.policy_generation,receipt:null});
      }
      pendingPolicy = {enabled,ticket,ids:targets.map(target=>target.motor_id),deadline:Date.now()+3000,model,connection};
      policyResult = '请求已发送，等待 MCU 确认';
      markDirty(DIRTY.POLICY,DIRTY.STATUS);
    } catch (error) { policyResult = String(error.message || error); }
    finally { submittingPolicy=false; }
    if (disposed) return;
    render(new Set([DIRTY.STATUS]));
  };
  const applyParameters = async (parameters, expected) => {
    if (!bridgeReady() || pending) throw new Error('主机未就绪或正在更新关节控制设置');
    const model = MODEL, connection = getConnectionGeneration(), started = Date.now();
    for (const entry of entries) entry.cancel();
    pending = true;
    render(new Set([DIRTY.STATUS]));
    try {
      const service = store.transmissionStatus.data.control_service
        || '/firmware_topic_bridge/set_parameters_atomically';
      const reply = await callService(service, 'rcl_interfaces/srv/SetParametersAtomically', {parameters});
      if (disposed || model !== MODEL || connection !== getConnectionGeneration()) {
        throw new Error('连接或模型已变化，请重新提交关节控制设置');
      }
      if (!reply.result?.successful) throw new Error(reply.result?.reason || '主机拒绝更新关节控制设置');
      const deadline = Date.now()+2500;
      while (Date.now()<deadline) {
        const runtime=store.transmissionStatus;
        if (disposed || model!==MODEL || connection!==getConnectionGeneration() || !store.conn.connected
            || (runtime.data?.control_service || '/firmware_topic_bridge/set_parameters_atomically')!==service) {
          throw new Error('连接、模型或主机控制会话已变化，请重新提交限位');
        }
        if (runtime.data?.model===model && runtime.last>=started
            && Object.entries(expected).every(([name,bounds]) =>
              runtime.data.joint_limits?.[name]?.lower===bounds.lower
              && runtime.data.joint_limits?.[name]?.upper===bounds.upper)) return '主机关节限位已生效（已回读确认）';
        await new Promise(resolve=>setTimeout(resolve,50));
      }
      throw new Error('主机已受理，但限位回读未确认；请核对当前生效范围');
    } finally {
      pending = false;
      if (!disposed) render(new Set([DIRTY.STATUS]));
    }
  };
  const syncLimits = (joint, side, value) => {
    const parameter=jointLimitsParameter(store,MODEL,{name:joint.name,side,value});
    return applyParameters([parameter],JSON.parse(parameter.value.string_value).joint_limits);
  };
  const sendError = name => {
    if (simulationSelected()) {
      if (!store.conn.connected || store.runtimeMode === 'readonly') return '工作台未连接或只读';
      if (!simulationBounds(name)?.session) return '仿真位置控制未就绪';
      if (Date.now() - (store.jointStates.byName[name]?.at || 0) > 500) return '等待仿真关节反馈';
      return Date.now()-advertised < 300 ? '关节控制入口准备中' : '';
    }
    return policyControlError(store,store.modelTransmissions.groups
      .filter(group=>group.joints.includes(name)).flatMap(group=>group.motors.map(motor=>motor.id)))
      || (pending ? '正在更新关节控制设置' : '') || jointControlError(store, MODEL, name)
      || (Date.now()-advertised < 300 ? '关节控制入口准备中' : '');
  };
  const send = (joint, value) => {
    const blocked = sendError(joint.name);
    if (blocked) throw new Error(blocked);
    if (simulationSelected()) {
      const {lower, upper, session} = simulationBounds(joint.name);
      if (!Number.isFinite(value) || value < lower || value > upper) throw new Error('目标超出仿真范围');
      if (!pub('simJointTarget', {header:{frame_id:session},name:[joint.name],position:[value]})) {
        throw new Error('连接已断开，未提交目标');
      }
      return `已提交 ${value.toFixed(3)} rad`;
    }
    const message = buildJointTarget(store, MODEL, joint, value);
    if (!pub('jointTarget', message)) throw new Error('连接已断开，未提交目标');
    return `已提交 ${value.toFixed(3)} ${joint.type === 'prismatic' ? 'm' : 'rad'}，请核对实际反馈`;
  };
  const render = dirty => {
    if (![DIRTY.MOTION, DIRTY.MODEL, DIRTY.SELECTED, DIRTY.BENCH, DIRTY.STATUS,
      DIRTY.CONN, DIRTY.COMMAND, DIRTY.TF].some(k => dirty.has(k))) return;
    if (isConnected() && generation !== getConnectionGeneration()) {
      ensurePublisher('jointTarget');
      ensurePublisher('simJointTarget');
      generation = getConnectionGeneration();advertised = Date.now();
    }
    let controlled = [], controlError = '';
    try { controlled = jointPolicyTargets(store, MODEL, false).map(target=>target.motor_id); }
    catch (error) {controlError=String(error.message || error);}
    const allEnabled = controlled.length > 0 && controlled.every(id => describeMotorState(store,id).policyEnabled === true);
    if (pendingPolicy) {
      const request = pendingPolicy;
      const outcome=policyRequestState(store,request.ticket,request.ids,request.enabled);
      if (request.model!==MODEL || request.connection!==getConnectionGeneration() || !store.conn.connected) {
        policyResult='连接或模型已变化，请重新确认';pendingPolicy=null;
      } else if (outcome.error) {policyResult=outcome.error;pendingPolicy=null;}
      else if (outcome.confirmed) {policyControlError(store,request.ids);policyResult=`MCU 已确认${request.enabled ? '使能' : '失能'}`;pendingPolicy=null;}
      else if (Date.now()>request.deadline) {policyResult='MCU 尚未确认请求，请检查设备状态';pendingPolicy=null;}
    }
    const sessionError = controlSessionError(store, MODEL);
    const simSelected = simulationSelected(), sim = simulationSession();
    toggleLabel.hidden = simSelected;
    toggle.checked = allEnabled;
    toggle.indeterminate = submittingPolicy || Boolean(pendingPolicy);
    toggle.disabled = simSelected || !store.conn.connected || !controlled.length || submittingPolicy || Boolean(pendingPolicy) || (Boolean(sessionError) && !allEnabled);
    const reason = simSelected ? sim?.reject || (sim ? '' : '等待 MuJoCo 位置控制') : controlError || sessionError;
    result.textContent = simSelected ? reason || 'MuJoCo 位置控制'
      : submittingPolicy ? '正在提交使能请求…' : pendingPolicy ? policyResult : reason || policyResult;
    result.classList.toggle('warn',Boolean(reason));
    toggle.closest('label').title = reason || '使能当前控制会话中的完整传动组，状态以 MCU 确认为准';
    const source = $('jointFeedbackSource'), feedback = store.bench.feedback;
    if (source) {
      source.textContent = feedback ? (feedback.error ||
        `${feedback.reason}${feedback.model ? ' · ' + feedback.model : ''}`) : '';
      source.classList.toggle('warn', Boolean(feedback?.error));
    }
    const joints = store.modelJoints;
    const next = JSON.stringify([MODEL, joints.map(j => [j.name,j.type,j.lower,j.upper,j.velocity])]);
    if (next !== key) {
      key = next;
      for (const entry of entries) entry.dispose();
      entries = joints.map(joint => createJointRow(joint, {send, sendError, syncLimits, simulationBounds, commandContext}));
      list.replaceChildren(...entries.map(entry => entry.element));
      if (!joints.length) {
        const empty = document.createElement('li');empty.className = 'empty';
        empty.textContent = '等待模型 URDF…';list.append(empty);
      }
    }
    const now = Date.now();
    for (const entry of entries) entry.render(now);
    const fresh = joints.filter(j => {
      const reading = store.jointStates.byName[j.name];
      return reading && now-reading.at < STALE_MS;
    }).length;
    summary.textContent = joints.length ? `${fresh}/${joints.length} 关节` : '--';
  };
  render(new Set([DIRTY.MODEL]));
  const ageTimer = setInterval(() => render(new Set([DIRTY.MOTION])), 1000);
  return {render, intervalMs: 200, unmount() {
    disposed = true;clearInterval(ageTimer);for (const entry of entries) entry.dispose();
  }};
}
