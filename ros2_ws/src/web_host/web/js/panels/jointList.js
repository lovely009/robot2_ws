// 关节索引：一行一台，点击选中（与臂上的小球双向联动）。只读 store。
import { store, DIRTY, devDetailTitle } from '../core/store.js';
import { $, fmt, busLabel, LIFECYCLE, FAULT_NAMES } from '../core/format.js';

export function mount(root, ctx) {
  const list = $('motorRows');
  const summary = $('sumJoints');

  // 事件委托：列表每次渲染都整体重建。
  list.addEventListener('click', (e) => {
    const li = e.target.closest('li[data-id]');
    if (li) ctx.ui.selectMotor(parseInt(li.dataset.id, 10));
  });

  const render = (dirty) => {
    if (!(dirty.has(DIRTY.TOPOLOGY) || dirty.has(DIRTY.STATUS) || dirty.has(DIRTY.MOTION)
          || dirty.has(DIRTY.SELECTED) || dirty.has(DIRTY.PARAMS))) return;
    // 行渲染搬自重构前的 renderMotors（app.js:522-554），只改数据来源。
    const rows = [...store.motors.entries()]
      .sort((a, b) => a[0] - b[0])
      .map(([id, r]) => {
        const t = r.topo || {}, st = r.status || {};
        const online = st.online !== undefined ? st.online : t.online;
        const lc = st.lifecycle !== undefined ? st.lifecycle : t.lifecycle;
        const fault = st.unified_fault_code || 0;
        // 旧 style.css 里这个色点写的是 var(--muted)；新 token 里叫 --dim-2（写错
        // 不会报错，色点会静默变透明）。
        const dot = !online ? 'var(--dim-2)'
          : ((fault || lc === 6 || lc === 7) ? 'var(--err)'
            : (lc === 4 ? 'var(--ok)' : 'var(--warn)'));
        const stateText = fault ? (FAULT_NAMES[fault] || fault)
          : (LIFECYCLE[lc] !== undefined ? LIFECYCLE[lc] : '--');
        const mo = r.motion || {};
        // 优先取状态流字段；字段缺失时回退到低频拓扑消息。
        const busIndex = Number.isInteger(st.bus_index) ? st.bus_index : t.bus_index;
        return `<li class="${id === store.selectedId ? 'on' : ''}" data-id="${id}"
            title="${devDetailTitle(id)}">
          <span class="jdot" style="background:${dot}"></span>
          <span><span class="jid">m${id}</span><span class="jbus">${busLabel(busIndex)}</span><span class="jmodel">${t.model_name || '未绑定'}</span></span>
          <span class="jpos">${fmt(mo.position, 3)}</span>
          <span class="jstate ${fault ? 'warn' : 'muted'}">${stateText}</span>
        </li>`;
      });
    list.innerHTML = rows.length
      ? rows.join('')
      : '<li class="empty">暂无数据（等待 motor_topology）</li>';

    // 摘要（搬 renderFoldSummaries 的 sumJoints 段，口径与顶栏 chips 一致）。
    const c = {online: 0, conflict: 0, fault: 0};
    for (const r of store.motors.values()) {
      const st = r.status || {}, t = r.topo || {};
      if (st.online !== undefined ? st.online : t.online) c.online++;
      if (t.conflict || st.lifecycle === 7) c.conflict++;
      if (st.unified_fault_code) c.fault++;
    }
    summary.textContent = store.motors.size
      ? `${c.online}/${store.motors.size} 在线`
        + (c.conflict ? ` · 冲突 ${c.conflict}` : '') + (c.fault ? ` · 故障 ${c.fault}` : '')
      : '--';
  };

  render(new Set([DIRTY.TOPOLOGY, DIRTY.STATUS, DIRTY.MOTION, DIRTY.SELECTED, DIRTY.PARAMS]));
  // 重表节流：motion 是 50 Hz，逐帧重建表格会比旧版 200 ms 定时刷新更糟。
  return {render, intervalMs: 200, unmount() {}};
}
