// hash 路由。屏不是模块：屏的标记是 index.html 里的 <template data-screen="...">，
// 这里只持有屏 id 的权威集合与默认屏。
export const SCREENS = ['run', 'teach', 'device', 'sim', 'diag'];
export const DEFAULT_SCREEN = 'run';

let onChange = null;
let current = null;

const normalize = (hash) => {
  const id = String(hash || '').replace(/^#\/?/, '').split('?')[0];
  return SCREENS.includes(id) ? id : DEFAULT_SCREEN;
};

export function currentScreen() { return current; }

export function gotoScreen(id) {
  const next = SCREENS.includes(id) ? id : DEFAULT_SCREEN;
  if (`#/${next}` === location.hash) return;
  location.hash = `#/${next}`;   // 触发 hashchange → 统一在 handler 里切
}

export function initRouter({onScreenChange}) {
  onChange = onScreenChange;
  const apply = () => {
    const next = normalize(location.hash);
    if (next === current) return;
    current = next;
    onChange(next);
  };
  window.addEventListener('hashchange', apply);
  apply();     // 首次：空 hash 或未知 hash 都落 DEFAULT_SCREEN
}
