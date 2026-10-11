// 命令历史面板：只读 store.history（core/ros.js 与 command 面板写入），支持导出与清空。
import { store, markDirty, DIRTY } from '../core/store.js';
import { $, fmt, download } from '../core/format.js';

function renderHistory() {
  const tbody = $('histRows');
  tbody.innerHTML = store.history.slice(0, 20).map((h) =>
    `<tr>
      <td class="num">${new Date(h.t).toLocaleTimeString()}</td>
      <td class="num">${h.seq}</td>
      <td>${h.targets.map((x) => `${x.motor_id}:${fmt(x.position,1)},${fmt(x.velocity,1)},${fmt(x.effort,1)}`).join(' ')}</td>
      <td><span class="pill on">已发</span></td>
    </tr>`
  ).join('') || '<tr class="empty"><td colspan="4">暂无记录</td></tr>';
}

function exportHistory() {
  download('cmd-history-' + Date.now() + '.json',
    JSON.stringify(store.history.slice().reverse(), null, 2));
}

export function mount(root, ctx) {
  $('btnExportHist').onclick = exportHistory;
  $('btnClearHist').onclick = () => { store.history.length = 0; markDirty(DIRTY.HISTORY); };

  const render = (dirty) => {
    if (!dirty.has(DIRTY.HISTORY)) return;
    renderHistory();
    $('sumHist').textContent = store.history.length ? `${store.history.length} 条` : '空';
  };
  render(new Set([DIRTY.HISTORY]));
  return {render, unmount() {}};
}
