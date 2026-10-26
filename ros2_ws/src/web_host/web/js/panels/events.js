// 事件日志面板：只读 store.events（订阅在 core/ros.js），支持导出与清空。
import { store, markDirty, DIRTY } from '../core/store.js';
import { $, download } from '../core/format.js';

function renderEvents() {
  $('eventLog').innerHTML = store.events.slice(0, 50).map((e) =>
    `<li><span class="muted">${new Date(e.t).toLocaleTimeString()}</span> `
    + `<b>${e.name}</b> · id=${e.id}${e.reason ? ' · reason=' + e.reason : ''}`
    + (e.request_id ? ` · req#${e.request_id}` : '') + '</li>'
  ).join('') || '<li class="muted">暂无事件</li>';
}

function exportEvents(csv) {
  const rows = store.events.map((e) => csv
    ? [new Date(e.t).toISOString(), e.name, e.id, e.reason ?? '', e.request_id ?? ''].join(',')
    : JSON.stringify(e));
  download(
    'events-' + Date.now() + (csv ? '.csv' : '.json'),
    rows.join('\n'),
    csv ? 'text/csv' : 'application/json');
}

export function mount(root, ctx) {
  $('btnExportEvents').onclick = () => exportEvents(false);
  $('btnExportEventsCsv').onclick = () => exportEvents(true);
  $('btnClearEvents').onclick = () => { store.events.length = 0; markDirty(DIRTY.EVENTS); };

  const render = (dirty) => {
    if (!dirty.has(DIRTY.EVENTS)) return;
    renderEvents();
    $('sumEvents').textContent = store.events.length ? `${store.events.length} 条` : '空';
  };
  render(new Set([DIRTY.EVENTS]));
  return {render, unmount() {}};
}
