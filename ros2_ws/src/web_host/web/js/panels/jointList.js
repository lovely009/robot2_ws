// 关节保持 URDF 顺序；反馈更新不替换正在操作的控件。
import { store, MODEL, DIRTY } from '../core/store.js';
import { $ } from '../core/format.js';
import { createJointRow, STALE_MS } from '../components/jointRow.js';
import { buildJointSliderTrajectory, jointControlError } from '../core/joint_control.js';
import { pub, ensurePublisher, isConnected, getConnectionGeneration } from '../core/ros.js';

export { STALE_MS };

export function mount(root, ctx) {
  const list = $('jointRows'), summary = $('sumJoints');
  let entries = [], key = '', generation = -1, advertised = 0;
  const toggle = root.querySelector('#jointControlEnabled');
  const result = root.querySelector('#jointControlResult');
  result.textContent = '控制许可由 ros2_control 状态确认';
  const bridgeReady = () => store.conn.connected && store.transmissionStatus.data?.model === MODEL
    && Date.now()-store.transmissionStatus.last >= 0
    && Date.now()-store.transmissionStatus.last < 2500;
  toggle.disabled = true;
  toggle.closest('label').title = '控制许可由 /robot_transmission/status 上报；只读显示';
  toggle.closest('label').addEventListener('click', event => event.stopPropagation());
  const sendError = name => jointControlError(store, MODEL, name)
    || (Date.now()-advertised < 300 ? '关节控制入口准备中' : '');
  const send = (joint, value) => {
    const blocked = sendError(joint.name);
    if (blocked) throw new Error(blocked);
    const message = buildJointSliderTrajectory(store, MODEL, joint, value);
    if (!pub('jointTrajectory', message)) throw new Error('连接已断开，未提交目标');
    return `已提交 ${value.toFixed(3)} ${joint.type === 'prismatic' ? 'm' : 'rad'}，请核对实际反馈`;
  };
  const render = dirty => {
    if (![DIRTY.MOTION, DIRTY.MODEL, DIRTY.SELECTED, DIRTY.BENCH, DIRTY.STATUS,
      DIRTY.CONN, DIRTY.COMMAND, DIRTY.TF].some(k => dirty.has(k))) return;
    if (isConnected() && generation !== getConnectionGeneration()) {
      ensurePublisher('jointTrajectory');generation = getConnectionGeneration();advertised = Date.now();
    }
    toggle.checked = bridgeReady() && Boolean(store.transmissionStatus.data?.trajectory_enabled);
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
      entries = joints.map(joint => createJointRow(joint, {send, sendError}));
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
    clearInterval(ageTimer);for (const entry of entries) entry.dispose();
  }};
}
