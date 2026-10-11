// 选中关节读数：纯显示，不参与任何下发（搬 renderSelected，app.js:679-711）。
// 臂的选中态由 arm 面板自己在 DIRTY.SELECTED 响应，不由本面板顺带同步。
import { store, DIRTY, targetOf } from '../core/store.js';
import { $, fmt, ageText, busLabel, LIFECYCLE, FAULT_NAMES } from '../core/format.js';

function renderSelected() {
  const id = store.selectedId;
  if (!id) {
    $('selTitle').textContent = '未选中关节（点左栏或臂上的小球）';
    for (const k of ['selPosition', 'selVelocity', 'selEffort', 'selCurrent',
                     'selTemp', 'selErr', 'selSource', 'selFault', 'selAge']) {
      $(k).textContent = '--';
    }
    return;
  }
  const r = store.motors.get(id) || {};
  const t = r.topo || {}, mo = r.motion || {}, st = r.status || {};
  const lc = st.lifecycle !== undefined ? st.lifecycle : t.lifecycle;
  const busIndex = Number.isInteger(st.bus_index) ? st.bus_index : t.bus_index;
  $('selTitle').textContent =
    `m${id} · ${busLabel(busIndex)} · ${t.model_name || '未绑定'} · ${LIFECYCLE[lc] || '--'}`;
  $('selPosition').textContent = fmt(mo.position, 3) + ' rad';
  $('selVelocity').textContent = fmt(mo.velocity, 3) + ' rad/s';
  $('selEffort').textContent = fmt(mo.effort, 3) + ' N·m';
  $('selCurrent').textContent = fmt(st.current, 2) + ' A';
  $('selTemp').textContent = fmt(st.temperature, 1) + ' °C';
  const tgt = targetOf(id);
  $('selErr').textContent =
    (mo.target_source === 2 || !Number.isFinite(tgt) || !Number.isFinite(mo.position))
      ? '--' : fmt(tgt - mo.position, 4) + ' rad';
  $('selSource').textContent = ['无', '快照', '轨迹'][mo.target_source] || '--';
  const fault = st.unified_fault_code || 0;
  $('selFault').textContent = fault ? (FAULT_NAMES[fault] || fault) : '无';
  $('selAge').textContent = ageText(mo.state_age_ms);
}

export function mount(root, ctx) {
  const render = (dirty) => {
    if (!(dirty.has(DIRTY.SELECTED) || dirty.has(DIRTY.MOTION) || dirty.has(DIRTY.STATUS))) return;
    renderSelected();
  };
  render(new Set([DIRTY.SELECTED]));
  return {render, intervalMs: 200, unmount() {}};
}
