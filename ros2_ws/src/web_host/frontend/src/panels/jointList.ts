// 关节保持 URDF 顺序；反馈更新不替换正在操作的控件。
import { store, MODEL, DIRTY, markDirty } from '../core/store';
import { $ } from '../core/format';
import { createJointRow, STALE_MS } from './jointRow';
import { buildJointTarget, jointControlError } from '../core/joint_control';
import { jointLimitsParameter } from '../core/joint_limits';
import { pub, ensurePublisher, isConnected, getConnectionGeneration, callService, submitPolicy } from '../core/ros';
import { jointPolicyTargets, isUnprotected } from '../core/control_policy';
import { describeMotorState, policyRequestState, policyControlError, applyPolicySubmission } from '../core/motor_state';
import { stopAll } from '../core/playback';
import type { MountedPanel } from '../contracts';

export { STALE_MS };

export function mount(root: HTMLElement, _ctx: any): MountedPanel {
  const list = $('jointRows') as HTMLElement;
  const summary = $('sumJoints') as HTMLElement;
  let entries: ReturnType<typeof createJointRow>[] = [];
  let key = '';
  let generation = -1;
  let advertised = 0;
  let pending = false;
  let disposed = false;

  const toggle = root.querySelector<HTMLInputElement>('#jointControlEnabled')!;
  const disableButton = root.querySelector<HTMLButtonElement>('#jointControlDisable')!;
  const unitButton = root.querySelector<HTMLButtonElement>('#btnJointUnit');
  const filterInput = root.querySelector<HTMLInputElement>('#jointFilter');
  let unitMode: 'rad' | 'deg' = localStorage.getItem('robot2.jointUnit') === 'deg' ? 'deg' : 'rad';
  if (unitButton) {
    unitButton.textContent = unitMode === 'deg' ? 'deg' : 'rad';
    unitButton.addEventListener('click', (event) => {
      event.preventDefault();
      event.stopPropagation();
      unitMode = unitMode === 'rad' ? 'deg' : 'rad';
      localStorage.setItem('robot2.jointUnit', unitMode);
      unitButton.textContent = unitMode === 'deg' ? 'deg' : 'rad';
      const now = Date.now();
      for (const entry of entries) entry.render(now);
    });
  }
  const applyFilter = () => {
    const q = (filterInput?.value || '').trim().toLowerCase();
    for (const entry of entries) {
      const name = (entry.element.dataset.joint || '').toLowerCase();
      entry.element.hidden = Boolean(q) && !name.includes(q);
    }
  };
  filterInput?.addEventListener('input', applyFilter);
  const toggleLabel = toggle.closest('label')!;
  const result = root.querySelector<HTMLElement>('#jointControlResult')!;
  let pendingPolicy: { enabled: boolean; ticket: any; ids: number[]; deadline: number; model: string; connection: number } | null = null;
  let policyResult = '';
  let submittingPolicy = false;
  result.textContent = '';

  const simulationSelected = () => {
    const config = store.bench.simulation?.config;
    return MODEL === 'full_body_robot' && config?.model === MODEL
      && config.backend === 'mujoco' && config.controller_mode === 'external';
  };
  const simulationSession = () => {
    const { data, last } = store.simPositionStatus;
    return simulationSelected() && store.bench.simulation.state === 'running'
      && Date.now() - last < 2000 && data?.model === MODEL
      && typeof data.session === 'string' && data.session ? data : null;
  };
  const simulationBounds = (name: string) => {
    if (!simulationSelected()) return null;
    const session = simulationSession();
    const bounds = session?.limits?.[name];
    if (!Array.isArray(bounds) || bounds.length !== 2 || !bounds.every(Number.isFinite)
        || bounds[0] > bounds[1]) return { lower: null, upper: null };
    return {
      lower: bounds[0], upper: bounds[1], session: session.session,
      error: bounds[0] === bounds[1] ? '当前姿态下该轴的可动范围为 0，已锁定' : '',
    };
  };
  const commandContext = () => `${getConnectionGeneration()}:${simulationSession()?.session || ''}`;
  const bridgeReady = () => store.conn.connected && store.transmissionStatus.data?.model === MODEL
    && Date.now() - store.transmissionStatus.last >= 0
    && Date.now() - store.transmissionStatus.last < 2500;

  toggle.disabled = true;
  toggleLabel.title = '使能当前模型的在线电机，允许部分传动组；使用模型默认增益，状态以 MCU 确认为准';
  toggleLabel.addEventListener('click', event => event.stopPropagation());

  const requestPolicy = async (enabled: boolean) => {
    if (simulationSelected() || submittingPolicy || pendingPolicy || disposed) return;
    const model = MODEL, connection = getConnectionGeneration();
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
      submittingPolicy = true;
      render(new Set([DIRTY.STATUS]));
      const ticket = await submitPolicy(targets);
      if (disposed || model !== MODEL || connection !== getConnectionGeneration()) throw new Error('连接或模型已变化，使能请求失效');
      applyPolicySubmission(store, targets, ticket);
      pendingPolicy = { enabled, ticket, ids: targets.map(target => target.motor_id), deadline: Date.now() + 3000, model, connection };
      policyResult = '请求已发送，等待 MCU 确认';
      markDirty(DIRTY.POLICY, DIRTY.STATUS);
    } catch (error: any) { policyResult = String(error.message || error); }
    finally { submittingPolicy = false; }
    if (disposed) return;
    render(new Set([DIRTY.STATUS]));
  };
  toggle.onchange = () => void requestPolicy(toggle.checked);
  disableButton.onclick = event => { event.stopPropagation(); void requestPolicy(false); };

  const applyParameters = async (parameters: any[], expected: Record<string, { lower: number; upper: number }>) => {
    if (!bridgeReady() || pending) throw new Error('主机未就绪或正在更新关节控制设置');
    const model = MODEL, connection = getConnectionGeneration(), started = Date.now();
    for (const entry of entries) entry.cancel();
    pending = true;
    render(new Set([DIRTY.STATUS]));
    try {
      const service = store.transmissionStatus.data.control_service
        || '/firmware_topic_bridge/set_parameters_atomically';
      const reply = await callService(service, 'rcl_interfaces/srv/SetParametersAtomically', { parameters });
      if (disposed || model !== MODEL || connection !== getConnectionGeneration()) {
        throw new Error('连接或模型已变化，请重新提交关节控制设置');
      }
      if (!reply.result?.successful) throw new Error(reply.result?.reason || '主机拒绝更新关节控制设置');
      const deadline = Date.now() + 2500;
      while (Date.now() < deadline) {
        const runtime = store.transmissionStatus;
        if (disposed || model !== MODEL || connection !== getConnectionGeneration() || !store.conn.connected
            || (runtime.data?.control_service || '/firmware_topic_bridge/set_parameters_atomically') !== service) {
          throw new Error('连接、模型或主机控制会话已变化，请重新提交限位');
        }
        if (runtime.data?.model === model && runtime.last >= started
            && Object.entries(expected).every(([name, bounds]) =>
              runtime.data.joint_limits?.[name]?.lower === bounds.lower
              && runtime.data.joint_limits?.[name]?.upper === bounds.upper)) return '主机关节限位已生效（已回读确认）';
        await new Promise(resolve => setTimeout(resolve, 50));
      }
      throw new Error('主机已受理，但限位回读未确认；请核对当前生效范围');
    } finally {
      pending = false;
      if (!disposed) render(new Set([DIRTY.STATUS]));
    }
  };
  const syncLimits = (joint: any, side: 'lower' | 'upper', value: number) => {
    const parameter = jointLimitsParameter(store, MODEL, { name: joint.name, side, value });
    return applyParameters([parameter], JSON.parse(parameter.value.string_value).joint_limits);
  };
  const sendError = (name: string) => {
    if (simulationSelected()) {
      if (!store.conn.connected || store.runtimeMode === 'readonly') return '工作台未连接或只读';
      const bounds = simulationBounds(name);
      if (!bounds?.session) return '仿真位置控制未就绪';
      if (bounds.error) return bounds.error;
      if (Date.now() - (store.jointStates.byName[name]?.at || 0) > 500) return '等待仿真关节反馈';
      return Date.now() - advertised < 300 ? '关节控制入口准备中' : '';
    }
    return policyControlError(store, store.modelTransmissions.groups
      .filter((group: any) => group.joints.includes(name)).flatMap((group: any) => group.motors.map((motor: any) => motor.id)))
      || (pending ? '正在更新关节控制设置' : '') || jointControlError(store, MODEL, name)
      || (Date.now() - advertised < 300 ? '关节控制入口准备中' : '');
  };
  const send = (joint: any, value: number) => {
    const blocked = sendError(joint.name);
    if (blocked) throw new Error(blocked);
    if (simulationSelected()) {
      const { lower, upper, session } = simulationBounds(joint.name)!;
      if (!Number.isFinite(value) || value < lower || value > upper) throw new Error('目标超出仿真范围');
      if (!pub('simJointTarget', { header: { frame_id: session }, name: [joint.name], position: [value] })) {
        throw new Error('连接已断开，未提交目标');
      }
      return `已提交 ${value.toFixed(3)} rad`;
    }
    const message = buildJointTarget(store, MODEL, joint, value);
    if (!pub('jointTarget', message)) throw new Error('连接已断开，未提交目标');
    return `已提交 ${value.toFixed(3)} ${joint.type === 'prismatic' ? 'm' : 'rad'}，请核对实际反馈`;
  };

  const render = (dirty: Set<string>) => {
    if (![DIRTY.MOTION, DIRTY.MODEL, DIRTY.SELECTED, DIRTY.BENCH, DIRTY.STATUS,
      DIRTY.CONN, DIRTY.COMMAND, DIRTY.TF].some(k => dirty.has(k))) return;
    if (isConnected() && generation !== getConnectionGeneration()) {
      ensurePublisher('jointTarget');
      ensurePublisher('simJointTarget');
      generation = getConnectionGeneration(); advertised = Date.now();
    }
    let controlled: number[] = [], controlError = '';
    try { controlled = jointPolicyTargets(store, MODEL, false).map(target => target.motor_id); }
    catch (error: any) { controlError = String(error.message || error); }
    const allEnabled = controlled.length > 0 && controlled.every(id => describeMotorState(store, id).policyEnabled === true);
    const anyEnabled = controlled.some(id => describeMotorState(store, id).policyEnabled === true);
    if (pendingPolicy) {
      const request = pendingPolicy;
      const outcome = policyRequestState(store, request.ticket, request.ids, request.enabled);
      if (request.model !== MODEL || request.connection !== getConnectionGeneration() || !store.conn.connected) {
        policyResult = '连接或模型已变化，请重新确认'; pendingPolicy = null;
      } else if (outcome.error) { policyResult = outcome.error; pendingPolicy = null; }
      else if (outcome.confirmed) { policyControlError(store, request.ids); policyResult = `MCU 已确认${request.enabled ? '使能' : '失能'}`; pendingPolicy = null; }
      else if (Date.now() > request.deadline) { policyResult = 'MCU 尚未确认请求，请检查设备状态'; pendingPolicy = null; }
    }
    if (!pendingPolicy && !submittingPolicy &&
        (policyResult === 'MCU 已确认使能' || policyResult === 'MCU 已确认失能' ||
         policyResult === 'MCU 已确认部分使能')) {
      policyResult = !controlled.length ? '' : !anyEnabled ? 'MCU 已确认失能'
        : allEnabled ? 'MCU 已确认使能' : 'MCU 已确认部分使能';
    }
    let enableError = '';
    try { jointPolicyTargets(store, MODEL, true); }
    catch (error: any) { enableError = String(error.message || error); }
    const simSelected = simulationSelected(), sim = simulationSession();
    toggleLabel.hidden = simSelected;
    toggle.checked = anyEnabled;
    toggle.indeterminate = submittingPolicy || Boolean(pendingPolicy) || (anyEnabled && !allEnabled);
    toggle.disabled = simSelected || !store.conn.connected || !controlled.length || submittingPolicy || Boolean(pendingPolicy) || (Boolean(enableError) && !anyEnabled);
    disableButton.hidden = simSelected || !controlled.length || !enableError || anyEnabled;
    disableButton.disabled = !store.conn.connected || submittingPolicy || Boolean(pendingPolicy);
    const hostReject = !simSelected && store.commandStatus.data?.model === MODEL
      ? String(store.commandStatus.data.reject || '') : '';
    const reason = simSelected ? sim?.reject || (sim ? '' : '等待 MuJoCo 位置控制')
      : hostReject ? `主机拒绝：${hostReject}` : controlError || enableError;
    result.textContent = simSelected ? reason || 'MuJoCo 位置控制'
      : submittingPolicy ? '正在提交使能请求…' : pendingPolicy ? policyResult : reason || policyResult;
    result.classList.toggle('warn', Boolean(reason));
    toggleLabel.title = reason || '使能当前模型的在线电机，允许部分传动组；失能覆盖已发现的模型电机，状态以 MCU 确认为准';
    const source = $('jointFeedbackSource'), feedback = store.bench.feedback;
    if (source) {
      source.textContent = feedback ? (feedback.error ||
        `${feedback.reason}${feedback.model ? ' · ' + feedback.model : ''}`) : '';
      source.classList.toggle('warn', Boolean(feedback?.error));
    }
    const joints = store.modelJoints;
    const next = JSON.stringify([MODEL, joints.map(j => [j.name, j.type, j.lower, j.upper, j.velocity])]);
    if (next !== key) {
      key = next;
      for (const entry of entries) entry.dispose();
      entries = joints.map(joint => createJointRow(joint, {
        send, sendError, syncLimits, simulationBounds, commandContext,
        displayDeg: () => unitMode === 'deg',
      }));
      list.replaceChildren(...entries.map(entry => entry.element));
      if (filterInput) filterInput.hidden = joints.length <= 6;
      applyFilter();
      if (!joints.length) {
        const empty = document.createElement('li'); empty.className = 'empty';
        empty.textContent = '等待模型 URDF…'; list.append(empty);
      }
    }
    const now = Date.now();
    for (const entry of entries) entry.render(now);
    const fresh = joints.filter(j => {
      const reading = store.jointStates.byName[j.name];
      return reading && now - reading.at < STALE_MS;
    }).length;
    summary.textContent = joints.length ? `${fresh}/${joints.length} 关节` : '--';
  };
  render(new Set([DIRTY.MODEL]));
  const ageTimer = setInterval(() => render(new Set([DIRTY.MOTION])), 1000);
  return {
    render,
    intervalMs: 200,
    unmount() {
      disposed = true; clearInterval(ageTimer); for (const entry of entries) entry.dispose();
    },
  };
}
