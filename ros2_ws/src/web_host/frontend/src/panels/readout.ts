// 电机状态：按电机编号显示，点击只选中。电机是固件执行器，与关节（TF 侧）分开显示。
import { store, DIRTY, orderedMotorIds, targetOf } from '../core/store';
import { describeMotorState } from '../core/motor_state';
import { $, fmt, ageText, busLabel, FAULT_NAMES } from '../core/format';
import type { MountedPanel } from '../contracts';

const cardMarkup = new WeakMap<Element, string>();
const escapeHtml = (value: any) => String(value).replace(/[&<>"']/g,
  (ch) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[ch]!));

function stateTextOf(id: number) {
  const view = describeMotorState(store, id);
  return { fault: view.fault, text: `${view.connection} · ${view.lifecycle} · ${view.execution} · ${view.health} · ${view.protocol}` };
}

function fullDetail(id: number): string {
  const r = store.motors.get(id) || {};
  const st = r.status || {}, mo = r.motion || {};
  const tgt = targetOf(id);
  const err = (mo.target_source === 2 || !Number.isFinite(tgt) || !Number.isFinite(mo.position))
    ? '--' : fmt(tgt - mo.position, 4) + ' rad';
  const { fault } = stateTextOf(id);
  return [
    `位置 ${fmt(mo.position, 3)} rad`,
    `速度 ${fmt(mo.velocity, 3)} rad/s`,
    `力矩 ${fmt(mo.effort, 3)} N·m`,
    `电流 ${fmt(st.current, 2)} A`,
    `温度 ${fmt(st.temperature, 1)} °C`,
    `跟随误差 ${err}`,
    `命令源 ${['无', '快照', '轨迹'][mo.target_source] || '--'}`,
    `故障 ${fault ? (FAULT_NAMES[fault] || fault) : '无'}`,
    `测量年龄 ${ageText(mo.state_age_ms)}`,
  ].join(' · ');
}

function cardHtml(id: number): string {
  const r = store.motors.get(id) || {};
  const t = r.topo || {}, st = r.status || {}, mo = r.motion || {};
  const view = describeMotorState(store, id);
  const online = view.online, fault = view.fault;
  const text = `${view.connection} · ${view.lifecycle} · ${view.execution} · ${view.health} · ${view.protocol}`;
  const state = view.lifecycle;
  const bus = Number.isInteger(st.bus_index) ? st.bus_index : t.bus_index;
  return `<button type="button" class="motor-card${id === store.selectedId ? ' on' : ''}"
      data-id="${id}" aria-pressed="${id === store.selectedId}" title="${escapeHtml(fullDetail(id))}">
    <span class="motor-card-heading"><strong>m${id}</strong>
      <span class="motor-card-state${fault ? ' fault' : online ? ' online' : ''}"
        title="${escapeHtml(text)}">${state}</span></span>
    <span class="motor-card-model">${escapeHtml(t.model_name || '未绑定型号')}</span>
    <span class="motor-card-bus"><i class="dot ${online ? 'green' : 'amber'}"></i>${busLabel(bus)} · ${view.connection}</span>
    <span class="motor-card-control-detail">${view.execution} · ${view.protocol}${fault ? ' · ' + view.health : ''}</span>
    <span class="motor-card-metrics">
      <span><span>位置 <small>rad</small></span><b>${fmt(mo.position, 3)}</b></span>
      <span><span>速度 <small>rad/s</small></span><b>${fmt(mo.velocity, 3)}</b></span>
      <span><span>电流 <small>A</small></span><b>${fmt(st.current, 2)}</b></span>
      <span><span>力矩 <small>N·m</small></span><b>${fmt(mo.effort, 3)}</b></span>
      <span><span>温度 <small>°C</small></span><b>${fmt(st.temperature, 1)}</b></span>
    </span>
  </button>`;
}

function renderRows(cards: HTMLElement | null) {
  const ids = orderedMotorIds();
  let online = 0;

  if (cards) {
    const existing = new Map([...cards.querySelectorAll<HTMLElement>('.motor-card')].map(el => [Number(el.dataset.id), el]));
    if (ids.length) cards.querySelector('.motor-card-empty')?.remove();
    for (const [index, id] of ids.entries()) {
      if (describeMotorState(store, id).online) online++;
      const html = cardHtml(id);
      let card = existing.get(id);
      if (!card || cardMarkup.get(card) !== html) {
        const template = document.createElement('template');
        template.innerHTML = html;
        const next = template.content.firstElementChild as HTMLElement;
        if (!card) card = next;
        else {
          card.className = next.className;
          card.title = next.title;
          card.setAttribute('aria-pressed', next.getAttribute('aria-pressed')!);
          card.replaceChildren(...next.childNodes);
        }
        cardMarkup.set(card, html);
      }
      if (cards.children[index] !== card) cards.insertBefore(card, cards.children[index] || null);
      existing.delete(id);
    }
    for (const old of existing.values()) old.remove();
    if (!ids.length && !cards.querySelector('.motor-card-empty')) {
      cards.innerHTML = '<div class="motor-card-empty">等待电机反馈</div>';
    }
  } else {
    // 仅在未启用卡片视图（表格可见）时构建表格 HTML，避免对隐藏 DOM 进行高频字符串拼接。
    const rows = ids.map((id) => {
      const r = store.motors.get(id) || {};
      const st = r.status || {}, mo = r.motion || {};
      if (describeMotorState(store, id).online) online++;
      const { fault, text } = stateTextOf(id);
      return `<tr class="${id === store.selectedId ? 'on' : ''}" data-id="${id}"
          title="${fullDetail(id)}">
        <td class="jid">m${id}</td>
        <td class="num">${fmt(mo.position, 3)}</td>
        <td class="num">${fmt(mo.effort, 3)}</td>
        <td class="num">${fmt(st.temperature, 1)}</td>
        <td class="jst${fault ? ' warn' : ''}">${text}</td>
      </tr>`;
    });
    $('selRows').innerHTML = rows.length
      ? rows.join('')
      : '<tr class="empty"><td colspan="5">等待 motor_topology…</td></tr>';
  }

  const id = store.selectedId;
  if (!id || cards) {
    $('selTitle').textContent = '电机状态';
  } else {
    const r = store.motors.get(id) || {};
    const t = r.topo || {}, st = r.status || {};
    const busIndex = Number.isInteger(st.bus_index) ? st.bus_index : t.bus_index;
    $('selTitle').textContent =
      `m${id} · ${busLabel(busIndex)} · ${t.model_name || '未绑定'} · ${describeMotorState(store, id).lifecycle}`;
  }
  $('selSum').textContent = ids.length ? `${online}/${ids.length} 在线` : '--';
}

export function mount(root: HTMLElement, ctx: any): MountedPanel {
  const cards = root.closest('.objects-area, .motor-inspector-panel') ? root.querySelector<HTMLElement>('#motorCards') : null;
  if (cards) {
    cards.hidden = false;
    root.classList.add('readout-cards');
    const wrap = root.querySelector<HTMLElement>('.table-wrap');
    if (wrap) wrap.hidden = true;
  }
  const select = (e: Event) => {
    const tr = (e.target as Element).closest<HTMLElement>('[data-id]');
    if (!tr) return;
    const id = parseInt(tr.dataset.id!, 10);
    ctx.ui.selectMotor(id);
  };
  $('selRows').addEventListener('click', select);
  cards?.addEventListener('click', select);

  const render = (dirty: Set<string>) => {
    if (!(dirty.has(DIRTY.SELECTED) || dirty.has(DIRTY.MOTION) || dirty.has(DIRTY.STATUS)
          || dirty.has(DIRTY.TOPOLOGY))) return;
    renderRows(cards);
  };
  render(new Set([DIRTY.TOPOLOGY]));
  return { render, intervalMs: 200, unmount() {} };
}
