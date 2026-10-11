// 状态翼：任何屏都常驻的窄栏。它只读 store，不改任何状态（点告警会跳屏并选中该台）。
// 挂载点就是 index.html 里的 #wingBody 本身（外壳的一部分，不进屏模板）——不克隆
// 模板、不重建容器，只更新自己的子元素。
import { store, DIRTY, countMotors } from '../core/store.js';
import { $ } from '../core/format.js';

// 告警类型 → 目标屏：冲突/参数类去「设备」处理，离线去「运行」看。
const ALERT_SCREEN = {configConflict: 'device', offline: 'run', params: 'device'};

function collectAlerts() {
  const out = [];
  for (const [id, row] of store.motors) {
    const lc = row.status ? row.status.lifecycle : undefined;
    const dev = store.devInit.get(id);
    if (lc === 7 || (dev && dev.state === 3)) {
      out.push({id, kind: 'configConflict', text: `ID${id} 参数未核对`});
    } else if (dev && (dev.state === 4 || dev.state === 5)) {
      out.push({id, kind: 'params', text: `ID${id} ${dev.state === 4 ? '回读不成功' : '本机无记录'}`});
    } else if (lc === 8) {
      out.push({id, kind: 'offline', text: `ID${id} 离线`});
    }
  }
  return out;
}

export function mount(root, ctx) {
  const counts = $('wingCounts');
  const alertsEl = $('wingAlerts');
  const lastEl = $('wingLast');
  const railEl = $('wingRail');

  alertsEl.addEventListener('click', (e) => {
    const li = e.target.closest('li[data-id]');
    if (!li) return;
    ctx.ui.selectMotor(parseInt(li.dataset.id, 10));
    ctx.ui.gotoScreen(li.dataset.screen);
  });

  // 收起态只剩竖条：点竖条即展开。复用 #wingToggle 的翻转逻辑（含 localStorage 记忆；
  // <820 由 CSS 无条件压回竖条，记忆保留）。
  railEl.addEventListener('click', () => $('wingToggle').click());

  const render = (dirty) => {
    if (dirty.has(DIRTY.MOTION) || dirty.has(DIRTY.STATUS) || dirty.has(DIRTY.TOPOLOGY)
        || dirty.has(DIRTY.PARAMS)) {
      const {online, total, active, conflict, fault} = countMotors();
      counts.textContent = `● 在线 ${online}/${total}　运行 ${active}　冲突 ${conflict}　故障 ${fault}`;
      counts.classList.toggle('err', conflict + fault > 0);
    }
    if (dirty.has(DIRTY.ALERTS) || dirty.has(DIRTY.STATUS) || dirty.has(DIRTY.PARAMS)) {
      const list = collectAlerts();
      alertsEl.replaceChildren();
      if (!list.length) {
        const li = document.createElement('li');
        li.className = 'empty';
        li.textContent = '无告警';
        alertsEl.append(li);
      }
      for (const a of list) {
        const li = document.createElement('li');
        li.dataset.id = String(a.id);          // 动态行只用 data- 与 class，不用 id
        li.dataset.screen = ALERT_SCREEN[a.kind];
        li.className = 'alert';
        li.textContent = a.text;
        alertsEl.append(li);
      }
      railEl.textContent = list.length ? `⚠${list.length}` : '●';
      railEl.title = list.map((a) => a.text).join('\n') || '无告警';
    }
    if (dirty.has(DIRTY.EVENTS)) {
      const e = store.events[0];
      lastEl.textContent = e ? `最近：${e.name} ID${e.id}` : '最近：--';
    }
  };

  render(new Set([DIRTY.MOTION, DIRTY.STATUS, DIRTY.PARAMS, DIRTY.ALERTS, DIRTY.EVENTS]));
  return {render, unmount() { /* 外壳常驻，无定时器与 canvas 需要释放 */ }};
}
