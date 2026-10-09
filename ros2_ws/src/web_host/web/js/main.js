// 入口：装外壳、初始化路由、挂面板、建立全局订阅、起 rAF 渲染循环。
// 屏切换只换主区里克隆的模板与其面板；顶栏/左导航/状态翼不重建。
import { store, markDirty, onDirty, runRenderLoop, DIRTY } from './core/store.js';
import { $, FOLD_KEY, WING_KEY } from './core/format.js';
import { SCREENS, initRouter, currentScreen } from './core/router.js';

// 已挂载面板：panelId -> 面板实例
const mounted = new Map();

export function mountPanel(panelId, container) {
  // 动态 import：未使用的面板模块不会被下载
  return import(`./panels/${panelId}.js`).then((mod) => {
    const inst = mod.mount(container, ctx);
    mounted.set(panelId, inst);
    return inst;
  });
}

export function unmountAll() {
  for (const [, inst] of mounted) {
    try { inst.unmount(); } catch (e) { console.error('unmount 失败', e); }
  }
  mounted.clear();
}

export function unmountPanel(panelId) {
  const inst = mounted.get(panelId);
  if (!inst) return;
  try { inst.unmount(); } catch (e) { console.error('unmount 失败', e); }
  mounted.delete(panelId);
}

// 面板契约（权威）：
//   mount(root, ctx) -> { render(dirty), unmount(), intervalMs? }
//   - root：屏模板里的 [data-panel] 占位元素（wing 例外，见 Task 3）
//   - render(dirty)：dirty 是自上次渲染以来变脏的键集合（core/store.js 的 DIRTY）
//   - intervalMs：可选，>0 时该面板的 render 至多每 N 毫秒一次（重表用 200）
//   - unmount()：解绑事件、释放 canvas/three 资源；**不得清空 store**
// 面板上下文：所有面板共用的能力都在这里，面板不直接操作别的面板
export const ctx = {
  store,
  markDirty,
  ui: {
    selectMotor(id) {
      store.selectedId = id;
      markDirty(DIRTY.SELECTED);
    },
    gotoScreen(id) { gotoScreen(id); },
    toast(text, level = '') { /* Task 2 接事件日志；本任务只记 console */ console.log('[toast]', level, text); },
  },
};

// 面板渲染节流：motion 流是 50 Hz，若每帧都重建表格会比重构前的 200 ms 定时刷新
// 更糟。面板可在 mount 返回值里声明 intervalMs（默认 0 = 每脏帧都渲染）。
const lastRender = new Map();

function renderPanels(dirty, now) {
  for (const [id, inst] of mounted) {
    const every = inst.intervalMs || 0;
    if (every > 0) {
      const prev = lastRender.get(id) || 0;
      if (now - prev < every) continue;
      lastRender.set(id, now);
    }
    try { inst.render(dirty); } catch (e) { console.error(`面板 ${id} 渲染失败`, e); }
  }
}

// 外壳文案（顶栏与 chips）由 core/agent.js 与 core/ros.js 写 store 后标脏，
// 渲染在 Task 2 接上。本任务先只分发面板。
onDirty((dirty) => renderPanels(dirty, performance.now()));

const FOLD_OF_SCREEN = (screen) => `${FOLD_KEY}.${screen}`;

// 切屏：卸载旧屏的全部面板 → 克隆新屏模板 → 挂载其面板
async function showScreen(screen) {
  unmountAll();
  const root = $('screenRoot');
  root.replaceChildren();
  const tpl = document.querySelector(`template[data-screen="${screen}"]`);
  if (!tpl) { console.error('未知的屏', screen); return; }
  root.append(tpl.content.cloneNode(true));
  for (const btn of document.querySelectorAll('#nav button')) {
    btn.setAttribute('aria-current', btn.dataset.goto === screen ? 'page' : 'false');
  }
  // 屏内所有 [data-panel] 占位按出现顺序挂载
  for (const slot of root.querySelectorAll('[data-panel]')) {
    await mountPanel(slot.dataset.panel, slot);
  }
  markDirty(DIRTY.ALERTS);
}

function gotoScreen(id) {
  if (id === currentScreen()) return;
  location.hash = `#/${id}`;
}

// 状态翼的折叠：记忆优先，<820 由 CSS 强制
function initWing() {
  const collapsed = localStorage.getItem(WING_KEY) === '1';
  document.body.classList.toggle('wing-collapsed', collapsed);
  $('wingToggle').onclick = () => {
    const next = !document.body.classList.contains('wing-collapsed');
    document.body.classList.toggle('wing-collapsed', next);
    localStorage.setItem(WING_KEY, next ? '1' : '0');
    markDirty(DIRTY.ALERTS);
  };
}

function initHelp() {
  const overlay = $('helpOverlay');
  $('btnHelp').onclick = () => overlay.classList.add('on');
  $('helpClose').onclick = () => overlay.classList.remove('on');
  overlay.addEventListener('click', (e) => { if (e.target === overlay) overlay.classList.remove('on'); });
}

function initKeys() {
  document.addEventListener('keydown', (e) => {
    if (e.target.closest('input, select, textarea')) return;
    if (e.key === 'Escape') { $('helpOverlay').classList.remove('on'); return; }
    const idx = '12345'.indexOf(e.key);
    if (idx >= 0 && idx < SCREENS.length) gotoScreen(SCREENS[idx]);
  });
}

function initNav() {
  $('nav').addEventListener('click', (e) => {
    const btn = e.target.closest('button[data-goto]');
    if (btn) gotoScreen(btn.dataset.goto);
  });
}

function boot() {
  initNav();
  initWing();
  initHelp();
  initKeys();
  initRouter({onScreenChange: showScreen});
  runRenderLoop();
}

boot();
