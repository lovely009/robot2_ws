import { store, MODEL, selectGroups, selectJoint } from '../core/store.js';
import { fmt, ageText } from '../core/format.js';
import { getJointLimits, saveJointLimit } from '../core/joint_limits.js';
import { createLatestTargetSender } from '../core/joint_control.js';
import { getConnectionGeneration } from '../core/ros.js';

export const STALE_MS = 1500;

export function createJointRow(joint, {send, sendError}) {
  const model = MODEL;
  const unit = joint.type === 'prismatic' ? 'm' : 'rad';
  const element = document.createElement('li');
  element.className = 'joint-row';
  element.dataset.joint = joint.name;
  element.innerHTML = '<span class="jdot"></span><span class="jid"></span>'
    + '<span class="joint-slider-control">'
    + '<button type="button" class="joint-bound" data-bound="lower"></button>'
    + '<input type="number" step="any" class="joint-bound-editor" data-bound-editor="lower" hidden>'
    + '<span class="joint-range"><input type="range" step="0.001"><span class="joint-target-value"></span></span>'
    + '<button type="button" class="joint-bound" data-bound="upper"></button>'
    + '<input type="number" step="any" class="joint-bound-editor" data-bound-editor="upper" hidden></span>'
    + '<span class="jpos"></span><span class="jstate"></span><span class="joint-message"></span>';
  element.querySelector('.jid').textContent = joint.name;
  const slider = element.querySelector('input[type="range"]');
  slider.setAttribute('aria-label', `${joint.name} 目标 ${unit}`);
  const target = element.querySelector('.joint-target-value');
  const message = element.querySelector('.joint-message');
  let draft = null, editing = null, result = '', editError = '', disposed = false;
  const groups = () => store.modelTransmissions.groups.filter(g => g.joints.includes(joint.name));
  const select = () => {selectJoint(joint.name);selectGroups(groups().map(g => g.name));};
  element.addEventListener('click', event => {
    if (!event.target.closest('input,button')) select();
  });
  const sender = createLatestTargetSender({
    isCurrent: connection => !disposed && model===MODEL && connection===getConnectionGeneration()
      && !document.hidden && !sendError(joint.name),
    send: value => {result=send(joint,value);editError='';render();},
    onError: error => {editError=String(error.message || error);render();},
  });
  const submit = final => {
    if (disposed || model!==MODEL || slider.disabled) {sender.cancel();return;}
    draft=Number(slider.value);select();
    sender.update(draft,getConnectionGeneration(),final);render();
  };
  slider.addEventListener('input', () => submit(false));
  slider.addEventListener('change', () => submit(true));
  const visibility = () => {if (document.hidden) sender.cancel();};
  document.addEventListener('visibilitychange',visibility);
  for (const side of ['lower', 'upper']) {
    const button = element.querySelector(`[data-bound="${side}"]`);
    const editor = element.querySelector(`[data-bound-editor="${side}"]`);
    const label = `${joint.name} ${side === 'lower' ? '下限' : '上限'} ${unit}`;
    button.setAttribute('aria-label', label);
    editor.setAttribute('aria-label', `编辑 ${label}`);
    button.title = '双击编辑软限位；Enter 或离开输入框保存，Esc 取消';
    const begin = () => {
      if (editing) return;
      editing = side; editError = '';
      const bounds = getJointLimits(model, joint);
      editor.value = bounds[side] === null ? '' : String(bounds[side]);
      button.hidden = true; editor.hidden = false;editor.focus();editor.select();
    };
    const close = () => {editing = null;button.hidden = false;editor.hidden = true;};
    const commit = () => {
      if (disposed || model !== MODEL || editing !== side) return;
      try {
        if (!editor.value.trim()) throw new Error('软限位不能为空');
        saveJointLimit(model, joint, side, Number(editor.value));
        close(); editError = ''; result = '软限位已保存';
      } catch (error) {editError = String(error.message || error);}
      render();
    };
    button.addEventListener('dblclick', begin);
    button.addEventListener('keydown', event => {
      if (event.key === 'Enter' || event.key === 'F2') {event.preventDefault();begin();}
    });
    editor.addEventListener('blur', commit);
    editor.addEventListener('keydown', event => {
      if (event.key === 'Enter') {event.preventDefault();commit();}
      if (event.key === 'Escape') {event.preventDefault();close();editError = '';render();}
    });
  }
  function render(now = Date.now()) {
    const reading = store.jointStates.byName[joint.name];
    const age = reading ? now-reading.at : Infinity;
    const selected = groups().some(g => store.selectedGroup.split(',').includes(g.name));
    element.classList.toggle('on', selected);
    element.dataset.groups = groups().map(g => g.name).join(',');
    element.title = `关节 ${joint.name} · ${joint.parent} → ${joint.child} · 实际反馈 ${unit}`;
    element.querySelector('.jdot').style.background = !reading ? 'var(--dim-2)'
      : age >= STALE_MS ? 'var(--warn)' : 'var(--ok)';
    element.querySelector('.jpos').textContent = reading ? fmt(reading.position, 3) : '--';
    element.querySelector('.jpos').title = `实际位置 ${unit}`;
    element.querySelector('.jstate').textContent = reading ? ageText(age) : '无反馈';
    element.querySelector('.jstate').classList.toggle('warn', age >= STALE_MS);
    const bounds = getJointLimits(model, joint);
    const available = bounds.lower !== null && bounds.upper !== null && !bounds.error;
    for (const side of ['lower', 'upper']) {
      element.querySelector(`[data-bound="${side}"]`).textContent = bounds[side] === null ? '设置' : fmt(bounds[side], 3);
    }
    const blocked = bounds.error || (!available ? '双击左右端设置软限位' : sendError(joint.name));
    slider.disabled = Boolean(blocked);
    if (blocked) sender.cancel();
    slider.title = blocked || `拖动连续发送目标 ${unit}（最高 20 Hz），松手提交最终位置`;
    slider.min = available ? String(bounds.lower) : '0';
    slider.max = available ? String(bounds.upper) : '1';
    const value = draft ?? reading?.position;
    if (Number.isFinite(value) && available) {
      slider.value = String(Math.min(bounds.upper, Math.max(bounds.lower, value)));
    }
    target.textContent = `目标 ${Number.isFinite(value) ? fmt(value, 3) : '--'} ${unit}`;
    message.textContent = editError || result || (selected ? blocked : '');
    message.classList.toggle('warn', Boolean(editError || (!result && blocked)));
  }
  return {element, render, cancel:()=>sender.cancel(), dispose() {
    disposed=true;editing=null;sender.dispose();
    document.removeEventListener('visibilitychange',visibility);element.remove();
  }};
}
