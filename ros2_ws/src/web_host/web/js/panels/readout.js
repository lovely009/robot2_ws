// 电机状态：全部电机同屏表（行点击 = 置顶 + 选中，行序与左栏关节列表共用 store 的
// 置顶序）。电机是固件的执行器，与左栏的关节（TF 侧）不是一类东西。
// 列只放常用 5 项；完整读数（速度/电流/跟随误差/命令源/测量年龄）在行的 title 里。
import { store, DIRTY, pinMotor, orderedMotorIds, targetOf } from '../core/store.js';
import { $, fmt, ageText, busLabel, LIFECYCLE, FAULT_NAMES } from '../core/format.js';

function stateTextOf(st, t) {
  const lc = st.lifecycle !== undefined ? st.lifecycle : t.lifecycle;
  const fault = st.unified_fault_code || 0;
  return {
    fault,
    text: fault ? (FAULT_NAMES[fault] || fault)
      : (LIFECYCLE[lc] !== undefined ? LIFECYCLE[lc] : '--'),
  };
}

// 行悬停的完整读数（9 项，与旧单台读数卡同口径）。
function fullDetail(id) {
  const r = store.motors.get(id) || {};
  const st = r.status || {}, mo = r.motion || {};
  const tgt = targetOf(id);
  const err = (mo.target_source === 2 || !Number.isFinite(tgt) || !Number.isFinite(mo.position))
    ? '--' : fmt(tgt - mo.position, 4) + ' rad';
  const { fault } = stateTextOf(st, r.topo || {});
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

function renderRows() {
  const ids = orderedMotorIds();
  let online = 0;
  const rows = ids.map((id) => {
    const r = store.motors.get(id) || {};
    const t = r.topo || {}, st = r.status || {}, mo = r.motion || {};
    if (st.online !== undefined ? st.online : t.online) online++;
    const { fault, text } = stateTextOf(st, t);
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

  // 标题：选中台明细（与旧读数卡一致）；未选中时是栏目名。
  const id = store.selectedId;
  if (!id) {
    $('selTitle').textContent = '电机状态';
  } else {
    const r = store.motors.get(id) || {};
    const t = r.topo || {}, st = r.status || {};
    const lc = st.lifecycle !== undefined ? st.lifecycle : t.lifecycle;
    const busIndex = Number.isInteger(st.bus_index) ? st.bus_index : t.bus_index;
    $('selTitle').textContent =
      `m${id} · ${busLabel(busIndex)} · ${t.model_name || '未绑定'} · ${LIFECYCLE[lc] || '--'}`;
  }
  $('selSum').textContent = ids.length ? `${online}/${ids.length} 在线` : '--';
}

export function mount(root, ctx) {
  // 点击行 = 置顶 + 选中（与左栏列表同行为）。
  $('selRows').addEventListener('click', (e) => {
    const tr = e.target.closest('tr[data-id]');
    if (!tr) return;
    const id = parseInt(tr.dataset.id, 10);
    pinMotor(id);
    ctx.ui.selectMotor(id);
  });

  const render = (dirty) => {
    if (!(dirty.has(DIRTY.SELECTED) || dirty.has(DIRTY.MOTION) || dirty.has(DIRTY.STATUS)
          || dirty.has(DIRTY.TOPOLOGY))) return;
    renderRows();
  };
  render(new Set([DIRTY.TOPOLOGY]));
  return {render, intervalMs: 200, unmount() {}};
}
