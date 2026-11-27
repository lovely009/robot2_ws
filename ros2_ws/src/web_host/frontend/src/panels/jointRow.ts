import { store, MODEL, selectGroups, selectJoint } from '../core/store';
import { fmt, ageText } from '../core/format';
import { getJointLimits, saveJointLimits } from '../core/joint_limits';
import { createLatestTargetSender } from '../core/joint_control';

export const STALE_MS = 1500;

export function createJointRow(
  joint: any,
  { send, sendError, syncLimits, simulationBounds, commandContext, displayDeg = () => false }: {
    send: (joint: any, value: number) => string;
    sendError: (name: string) => string;
    syncLimits: (joint: any, side: 'lower' | 'upper', value: number) => Promise<string>;
    simulationBounds: (name: string) => any;
    commandContext: () => string;
    displayDeg?: () => boolean;
  },
) {
  const model = MODEL;
  const isPrismatic = joint.type === 'prismatic';
  const unitLabel = () => (isPrismatic ? 'm' : displayDeg() ? '°' : 'rad');
  const formatVal = (val: number, decimals = 3) => {
    if (isPrismatic || !displayDeg()) return fmt(val, decimals);
    return fmt((val * 180) / Math.PI, 1);
  };
  const element = document.createElement('li');
  element.className = 'joint-row';
  element.dataset.joint = joint.name;
  element.innerHTML = '<div class="joint-row-head">'
    + '<span class="jdot"></span><span class="jid"></span>'
    + '<span class="jpos"></span><span class="jstate"></span>'
    + '<span class="joint-message info-badge" tabindex="0" hidden>!</span>'
    + '</div>'
    + '<span class="joint-slider-control">'
    + '<button type="button" class="joint-bound" data-bound="lower"></button>'
    + '<input type="number" step="any" class="joint-bound-editor" data-bound-editor="lower" hidden>'
    + '<button type="button" class="tiny-btn secondary joint-nudge" data-nudge="-1" aria-label="微调减小">−</button>'
    + '<span class="joint-range"><input type="range" step="0.001"><span class="joint-target-value"></span></span>'
    + '<button type="button" class="tiny-btn secondary joint-nudge" data-nudge="1" aria-label="微调增大">+</button>'
    + '<button type="button" class="joint-bound" data-bound="upper"></button>'
    + '<input type="number" step="any" class="joint-bound-editor" data-bound-editor="upper" hidden></span>';
  element.querySelector('.jid')!.textContent = joint.name;
  const slider = element.querySelector<HTMLInputElement>('input[type="range"]')!;
  slider.setAttribute('aria-label', `${joint.name} 目标 ${unitLabel()}`);
  const nudgeButtons = [...element.querySelectorAll<HTMLButtonElement>('[data-nudge]')];
  const target = element.querySelector('.joint-target-value')!;
  const message = element.querySelector('.joint-message')!;
  let draft: number | null = null;
  let editing: 'lower' | 'upper' | null = null;
  let saving = false;
  let result = '';
  let editError = '';
  let disposed = false;
  let boundsKey = '';

  const groups = () => store.modelTransmissions.groups.filter((g: any) => g.joints.includes(joint.name));
  const select = () => { selectJoint(joint.name); selectGroups(groups().map((g: any) => g.name)); };
  element.addEventListener('click', event => {
    if (!(event.target as Element).closest('input,button')) select();
  });
  const sender = createLatestTargetSender({
    isCurrent: context => !disposed && model === MODEL && context === commandContext()
      && !document.hidden && !sendError(joint.name),
    send: value => { result = send(joint, value); editError = ''; render(); },
    onError: error => { editError = String((error as Error).message || error); render(); },
  });
  const submit = (final: boolean) => {
    if (disposed || model !== MODEL || slider.disabled) { sender.cancel(); return; }
    draft = Number(slider.value); select();
    sender.update(draft, commandContext(), final); render();
  };
  slider.addEventListener('input', () => submit(false));
  slider.addEventListener('change', () => submit(true));
  for (const btn of nudgeButtons) {
    btn.addEventListener('click', () => {
      if (disposed || model !== MODEL || slider.disabled) return;
      const dir = Number(btn.dataset.nudge || 0);
      const step = isPrismatic ? 0.005 : Math.PI / 180;
      const base = draft ?? Number(slider.value);
      const lo = Number(slider.min);
      const hi = Number(slider.max);
      const next = Math.min(hi, Math.max(lo, base + dir * step));
      slider.value = String(next);
      draft = next;
      select();
      sender.update(next, commandContext(), true);
      render();
    });
  }
  const visibility = () => { if (document.hidden) sender.cancel(); };
  document.addEventListener('visibilitychange', visibility);

  for (const side of ['lower', 'upper'] as const) {
    const button = element.querySelector<HTMLButtonElement>(`[data-bound="${side}"]`)!;
    const editor = element.querySelector<HTMLInputElement>(`[data-bound-editor="${side}"]`)!;
    const label = `${joint.name} ${side === 'lower' ? '下限' : '上限'} ${unitLabel()}`;
    button.setAttribute('aria-label', label);
    editor.setAttribute('aria-label', `编辑 ${label}`);
    button.title = '单击编辑软限位；Enter 或离开输入框保存，Esc 取消；仅约束后续目标';
    const begin = () => {
      if (button.disabled || editing || saving) return;
      editing = side; editError = ''; draft = null; result = ''; sender.cancel(); select();
      const bounds = getJointLimits(store, model, joint);
      editor.value = bounds[side] === null ? '' : String(bounds[side]);
      button.hidden = true; editor.hidden = false; editor.focus(); editor.select();
    };
    const close = () => { editing = null; button.hidden = false; editor.hidden = true; };
    const commit = async () => {
      if (disposed || model !== MODEL || editing !== side || saving) return;
      try {
        if (!editor.value.trim()) throw new Error('软限位不能为空');
        saving = true; editor.disabled = true; sender.cancel();
        const confirmation = await syncLimits(joint, side, Number(editor.value));
        if (disposed || model !== MODEL) return;
        const bounds = getJointLimits(store, model, joint);
        close(); editError = ''; result = confirmation;
        try { saveJointLimits(model, joint, bounds); } catch (error: any) {
          editError = `主机限位已生效，本地记录未保存：${error.message}`;
        }
      } catch (error: any) {
        if (!disposed && model === MODEL) editError = String(error.message || error);
      } finally {
        saving = false; editor.disabled = false;
        if (!disposed) render();
      }
    };
    button.addEventListener('click', begin);
    button.addEventListener('keydown', event => {
      if (event.key === 'Enter' || event.key === 'F2') { event.preventDefault(); begin(); }
    });
    editor.addEventListener('blur', commit);
    editor.addEventListener('keydown', event => {
      if (event.key === 'Enter') { event.preventDefault(); void commit(); }
      if (event.key === 'Escape') { event.preventDefault(); close(); editError = ''; render(); }
    });
  }

  function render(now = Date.now()) {
    const reading = store.jointStates.byName[joint.name];
    const age = reading ? now - reading.at : Infinity;
    const selected = groups().some((g: any) => store.selectedGroup.split(',').includes(g.name));
    element.classList.toggle('on', selected);
    element.dataset.groups = groups().map((g: any) => g.name).join(',');
    element.title = `关节 ${joint.name} · ${joint.parent} → ${joint.child} · 实际反馈 ${unitLabel()}`;
    element.querySelector<HTMLElement>('.jdot')!.style.background = !reading ? 'var(--dim-2)'
      : age >= STALE_MS ? 'var(--warn)' : 'var(--ok)';
    const jpos = element.querySelector<HTMLElement>('.jpos')!;
    jpos.textContent = reading ? formatVal(reading.position, 3) : '--';
    jpos.title = `实际位置 ${unitLabel()}`;
    const jstate = element.querySelector<HTMLElement>('.jstate')!;
    jstate.textContent = reading ? ageText(age) : '无反馈';
    jstate.classList.toggle('warn', age >= STALE_MS);
    const sim = simulationBounds(joint.name);
    const bounds = sim ?? getJointLimits(store, model, joint);
    const dynamic = bounds.dynamic === true;
    const nextBounds = JSON.stringify([sim?.session, bounds.lower, bounds.upper]);
    if (boundsKey !== nextBounds) {
      boundsKey = nextBounds;
      const keepDraft = dynamic && !bounds.error && Number.isFinite(draft)
        && bounds.lower <= draft! && draft! <= bounds.upper;
      if (!keepDraft) { sender.cancel(); draft = null; result = ''; }
    }
    const available = bounds.lower !== null && bounds.upper !== null && !bounds.error;
    for (const side of ['lower', 'upper'] as const) {
      const button = element.querySelector<HTMLButtonElement>(`[data-bound="${side}"]`)!;
      button.textContent = bounds[side] === null ? (sim || dynamic ? '—' : '设置') : formatVal(bounds[side], 3);
      button.disabled = Boolean(sim) || dynamic;
      button.title = dynamic ? '当前姿态下的耦合动态限位' : sim ? '当前 MuJoCo 模型的仿真范围' : '单击编辑软限位';
      if ((sim || dynamic) && editing === side) {
        editing = null;
        button.hidden = false;
        element.querySelector<HTMLInputElement>(`[data-bound-editor="${side}"]`)!.hidden = true;
      }
    }
    const blocked = (saving ? '正在同步关节限位…' : '') || bounds.error
      || (sim || available ? sendError(joint.name) : '单击左右端设置软限位');
    slider.disabled = Boolean(blocked);
    for (const btn of nudgeButtons) btn.disabled = slider.disabled;
    if (blocked) sender.cancel();
    slider.title = blocked || `拖动连续发送目标 ${unitLabel()}（最高 20 Hz），松手提交最终位置`;
    slider.min = available ? String(bounds.lower) : '0';
    slider.max = available ? String(bounds.upper) : '1';
    const value = draft ?? reading?.position;
    if (Number.isFinite(value) && available) {
      slider.value = String(Math.min(bounds.upper, Math.max(bounds.lower, value)));
      if (draft !== null) draft = Number(slider.value);
    }
    target.textContent = `目标 ${Number.isFinite(draft) ? formatVal(draft!, 3) : '--'} ${unitLabel()}`;
    const rejection = !sim && selected && store.commandStatus.data?.model === model
      ? store.commandStatus.data.reject : '';
    const msgText = editError || (rejection ? `主机拒绝：${rejection}` : '') || result
      || bounds.error || (selected ? blocked : '');
    const msgEl = message as HTMLElement;
    msgEl.hidden = !msgText;
    msgEl.title = msgText;
    msgEl.setAttribute('aria-label', msgText);
    msgEl.classList.toggle('warn', Boolean(editError || rejection || (!result && blocked)));
  }

  return {
    element,
    render,
    cancel: () => sender.cancel(),
    dispose() {
      disposed = true; editing = null; sender.dispose();
      document.removeEventListener('visibilitychange', visibility); element.remove();
    },
  };
}
