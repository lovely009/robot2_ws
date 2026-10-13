// 入口：装外壳、初始化路由、挂面板、建立全局订阅、起 rAF 渲染循环。
// 屏切换只换主区里克隆的模板与其面板；顶栏/左导航/状态翼不重建。
import { store, markDirty, onDirty, runRenderLoop, DIRTY, countMotors, loadModelJoints } from './core/store.js';
import { $, FOLD_KEY, WING_KEY, ROS_URL_STORAGE_KEY, CONTROLLER_PORT_STORAGE_KEY }
  from './core/format.js';
import { SCREENS, initRouter, currentScreen } from './core/router.js';
import { connect, waitSysStatus } from './core/ros.js';
import { refreshAgentControls, refreshAgentStatus, startAgent, stopAgent }
  from './core/agent.js';
import { loadParams } from './core/device.js';

// 已挂载面板：panelId -> 面板实例
const mounted = new Map();

export function mountPanel(panelId, container) {
  // 动态 import：未使用的面板模块不会被下载
  return import(`./panels/${panelId}.js`).then((mod) => {
    // 面板标记的唯一来源是 index.html 的 <template data-panel-template>：克隆进
    // 屏模板里的 [data-panel] 占位后交给模块（JS 不生成带 id 的标记，只克隆）。
    // wing 例外：标记直接写在外壳 #wingBody 里，没有模板可克隆。
    const tpl = document.querySelector(`template[data-panel-template="${panelId}"]`);
    if (tpl && container) container.append(tpl.content.cloneNode(true));
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

// 外壳文案（顶栏与 chips）：core/agent.js 与 core/ros.js 只写 store 并标脏，
// 这里在脏帧里统一画——订阅层不碰 DOM，壳层不认识面板数据来源。
function renderShell(dirty) {
  if (dirty.has(DIRTY.CONN)) {
    $('connDot').classList.toggle('on', store.conn.connected);
    $('connText').textContent = store.conn.text || (store.conn.connected ? '已连接' : '未连接');
    const c = store.conn;
    const el = $('agentText');
    el.classList.remove('running', 'failed');
    // 「正在启动/断开」是瞬时态：文案与按钮禁用与重构前 startAgent/stopAgent 一致。
    $('btnConnect').disabled = c.agent === 'starting';
    if (c.agent === 'starting' || c.agent === 'stopping') {
      el.textContent = c.agent === 'starting' ? '正在启动 Agent…' : '正在断开 Agent…';
      el.title = '';
      $('btnDisconnect').disabled = true;
    } else if (c.agent === 'running') {
      el.textContent = `Agent 进程运行中${c.agentDevice ? ` · ${c.agentDevice}` : ''}`;
      el.classList.add('running');
      el.title = '由 web_host 管理的 micro-ROS Agent 进程';
      $('btnDisconnect').disabled = false;
    } else if (c.agent === 'failed') {
      el.textContent = `Agent 已退出${c.agentReturncode === null ? '' : ` · code ${c.agentReturncode}`}`;
      el.classList.add('failed');
      el.title = c.agentError || c.agentDevice || 'Agent 进程已退出';
      $('btnDisconnect').disabled = true;
    } else if (c.agent === 'error') {
      el.textContent = c.canStop
        ? `Agent 已运行${c.agentDevice ? ` · ${c.agentDevice}` : ''}`
        : (c.agentError ? `Agent 启动失败 · ${c.agentError}` : 'Agent 启动失败');
      el.classList.add('failed');
      el.title = c.agentError || '启动 Agent 失败';
      $('btnDisconnect').disabled = !c.canStop;
    } else {
      el.textContent = 'Agent 未启动';
      el.title = '';
      $('btnDisconnect').disabled = true;
    }
  }
  if (dirty.has(DIRTY.TOPOLOGY) || dirty.has(DIRTY.STATUS) || dirty.has(DIRTY.MOTION)) {
    renderChips();
  }
}

// 顶栏汇总 chips（在线/运行/冲突/故障）：计数口径与状态翼共用 store.js 的
// countMotors()；chipHz 是 1 s 窗口计数，在 boot 里起定时器（脏帧驱动做不到"每秒"）。
function renderChips() {
  const {online, total, active, conflict, fault} = countMotors();
  $('chipOnline').textContent = `在线 ${online}/${total}`;
  $('chipActive').textContent = `运行 ${active}`;
  $('chipActive').classList.toggle('ok', active > 0);
  $('chipConflict').textContent = `冲突 ${conflict}`;
  $('chipConflict').classList.toggle('warn', conflict > 0);
  $('chipFault').textContent = `故障 ${fault}`;
  $('chipFault').classList.toggle('err', fault > 0);
}

onDirty((dirty) => {
  const now = performance.now();
  // 连上后自动读取一次参数：使能面板的电机列表来自 paramCache，设备屏不挂载时也要有。
  // 先等首帧 system_status 确认固件侧已在总线上（订阅刚建时 DDS 发现可能还没完成，
  // REPORT 太早发出可能白等）；真没固件时由 loadParams 自己的超时兜底。
  // 掉线时 core/ros.js 会复位 autoLoaded，重连后自动再读一次。
  if (dirty.has(DIRTY.CONN) && store.conn.connected && !store.conn.autoLoaded) {
    store.conn.autoLoaded = true;
    waitSysStatus().then(() => loadParams());
  }
  renderShell(dirty);          // 顶栏连接文案与 Agent 状态
  renderPanels(dirty, now);
});

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
  // 屏内所有 [data-panel] 占位按出现顺序挂载（含抽屉里的，未展开也挂——展开即有形）
  for (const slot of root.querySelectorAll('[data-panel]')) {
    await mountPanel(slot.dataset.panel, slot);
  }
  initDrawers(screen);
  markDirty(DIRTY.ALERTS);
}

// 屏内抽屉：开合状态与当前 tab 按屏记忆（web_host.fold.<屏>）。
// tab 行的 .on、面板的 hidden 与标题栏的 .drawer-sum 都在这里统一管。
function initDrawers(screen) {
  const key = FOLD_OF_SCREEN(screen);
  let saved = {};
  try { saved = JSON.parse(localStorage.getItem(key) || '{}'); } catch (e) { saved = {}; }
  const save = () => localStorage.setItem(key, JSON.stringify(saved));

  const details = document.querySelector('#screenRoot details.drawer');
  if (!details) return;
  if (saved.drawerOpen !== undefined) details.open = !!saved.drawerOpen;
  details.addEventListener('toggle', () => { saved.drawerOpen = details.open; save(); });

  const tabs = [...document.querySelectorAll('#screenRoot [data-drawer-tab]')];
  const panels = [...document.querySelectorAll('#screenRoot [data-drawer-panel]')];
  const sum = document.querySelector('#screenRoot .drawer-sum');
  const activate = (name) => {
    for (const p of panels) p.hidden = p.dataset.drawerPanel !== name;
    for (const t of tabs) {
      const on = t.dataset.drawerTab === name;
      t.classList.toggle('on', on);
      if (on && sum) sum.textContent = t.textContent;   // 收起时标题栏仍有信息
    }
    saved.tab = name; save();
  };
  for (const t of tabs) t.onclick = () => activate(t.dataset.drawerTab);
  const want = tabs.some((t) => t.dataset.drawerTab === saved.tab)
    ? saved.tab : (tabs[0] && tabs[0].dataset.drawerTab);
  activate(want);
}

function gotoScreen(id) {
  if (id === currentScreen()) return;
  location.hash = `#/${id}`;
}

// 状态翼三态：用户显式选择（'1' 收起 / '0' 展开）优先；localStorage 无值 = 未选择，
// 默认展开；<820 由 CSS 强制收起。
function initWing() {
  const saved = localStorage.getItem(WING_KEY);
  document.body.classList.toggle('wing-collapsed', saved === '1');
  document.body.classList.toggle('wing-expanded', saved === '0');
  $('wingToggle').onclick = () => {
    // 从「当前生效」状态翻转。默认（无显式选择）是展开——<820 由 CSS 无条件压成
    // 竖条、与类无关，所以只有真正带 wing-collapsed 类时才算收起。
    const collapsed = document.body.classList.contains('wing-collapsed');
    const next = collapsed ? '0' : '1';   // 翻转：收起 → 展开，展开 → 收起
    document.body.classList.toggle('wing-collapsed', next === '1');
    document.body.classList.toggle('wing-expanded', next === '0');
    localStorage.setItem(WING_KEY, next);   // 始终写显式值，绝不删除键
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

// 顶栏控件接线：rosbridge 地址与控制板串口都记忆在浏览器本地；启动/断开按钮只
// 管理 micro-ROS Agent 进程，不影响 WebSocket 连接（搬迁自旧 init()）。
function initTopbar() {
  const defaultRosUrl = (location.protocol === 'https:' ? 'wss://' : 'ws://')
    + (location.hostname || 'localhost') + ':9090';
  $('rosUrl').value = localStorage.getItem(ROS_URL_STORAGE_KEY) || defaultRosUrl;
  $('rosUrl').addEventListener('input', () => {
    localStorage.setItem(ROS_URL_STORAGE_KEY, $('rosUrl').value.trim());
  });
  $('btnConnect').onclick = startAgent;
  $('btnDisconnect').onclick = stopAgent;
  $('btnPortRefresh').onclick = refreshAgentControls;
  $('serialPort').addEventListener('change', (e) => {
    if (e.target.value) localStorage.setItem(CONTROLLER_PORT_STORAGE_KEY, e.target.value);
  });
}

function boot() {
  initNav();
  initWing();
  initHelp();
  initKeys();
  initTopbar();
  initRouter({onScreenChange: showScreen});
  // 状态翼是外壳的一部分（不进屏模板），在这里挂一次后常驻。
  mountPanel('wing', $('wingBody'));
  // 臂视图的 motor_id → 关节名 映射按**当前模型**去服务器取（权威来源是模型自己的
  // config/motor_driver.yaml）。不 await：拿不到就退回内置表并在控制台留痕，
  // 不阻塞开页。成功时标脏 TOPOLOGY，已挂载的臂视图下一帧重算。
  loadModelJoints();
  runRenderLoop();
  // 运动流频率：1 s 窗口计数。
  let lastMotionCount = 0;
  setInterval(() => {
    const hz = store.motionCount - lastMotionCount;
    lastMotionCount = store.motionCount;
    $('chipHz').textContent = `运动流 ${store.conn.connected ? hz + ' Hz' : '-- Hz'}`;
  }, 1000);
  // rosbridge 开页即连（断线自动重连在 core/ros.js）；Agent 状态轮询 2 s。
  refreshAgentControls();
  setInterval(refreshAgentStatus, 2000);
  connect();
}

boot();
