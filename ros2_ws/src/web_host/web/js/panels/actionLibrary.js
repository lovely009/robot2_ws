import { $ } from '../core/format.js';
import { playAction, stopPlayback, loadSeries, playbackSnapshot } from '../core/playback.js';
import { bvhModelText, bvhSelection } from '../core/bvh_playback.js';
import { MODEL } from '../core/store.js';

// 预览（列表行点击展开）：存**解析后**的轨迹（loadSeries 产物），不是原文——
// 预览要回答的是"这条动作是什么样"（时刻×各台位置 + 范围），原文留给需要的人
// 从文件拿。展开状态与解析缓存在模块级，列表重建后原样恢复。
const PREVIEW_MAX_ROWS = 40;
let expandedName = '';
const previewCache = new Map();

// 解析后的预览：摘要（目标/时刻数/时长/采样率/各台位置范围）+ 时刻×位置表
// （均匀抽样到 ≤40 行，跳行处标出省略量）。
function renderPreview(action) {
  const box = document.createElement('div');
  box.className = 'act-preview';
  box.dataset.name = action.name;
  const parsed = previewCache.get(action.name);
  if (parsed === undefined) {
    box.textContent = '加载中…';
    return box;
  }
  if (parsed.error) {
    box.textContent = parsed.error;
    return box;
  }
  const { ids, steps } = parsed;

  const duration = steps[steps.length - 1].t - steps[0].t;
  const stride = Math.max(1, Math.ceil(steps.length / PREVIEW_MAX_ROWS));
  const head = document.createElement('div');
  head.className = 'act-preview-sum';
  head.textContent = `${parsed.interpolation === 'linear' ? '原始线性' : '单调插值'} · 目标 m${ids.join(', m')} · ${steps.length} 时刻`
    + `（${steps.length * ids.length} 点）· ${(duration / 1000).toFixed(1)} s`
    + ` · ${duration > 0 ? ((steps.length - 1) / (duration / 1000)).toFixed(1) : '--'} Hz`
    + (stride > 1 ? ` · 表内每 ${stride} 时刻取 1 行` : '');
  box.append(head);

  const range = document.createElement('div');
  range.className = 'act-preview-sum';
  range.textContent = ids.map((id) => {
    let low = Infinity;
    let high = -Infinity;
    for (const step of steps) {
      const [position] = step.vals[id] || [0, 0, 0];
      if (position < low) low = position;
      if (position > high) high = position;
    }
    return `m${id} 位置 ${low.toFixed(2)}..${high.toFixed(2)}`;
  }).join(' · ');
  box.append(range);

  const table = document.createElement('table');
  table.className = 'act-preview-table';
  const headRow = document.createElement('tr');
  const timeHead = document.createElement('th');
  timeHead.textContent = 't/ms';
  headRow.append(timeHead);
  for (const id of ids) {
    const th = document.createElement('th');
    th.textContent = `m${id}`;
    headRow.append(th);
  }
  const thead = document.createElement('thead');
  thead.append(headRow);
  table.append(thead);

  const indices = [];
  for (let index = 0; index < steps.length; index += stride) indices.push(index);
  if (indices[indices.length - 1] !== steps.length - 1) {
    indices.push(steps.length - 1);
  }
  const tbody = document.createElement('tbody');
  for (const index of indices) {
    const step = steps[index];
    const row = document.createElement('tr');
    const time = document.createElement('td');
    time.textContent = String(step.t);
    row.append(time);
    for (const id of ids) {
      const cell = document.createElement('td');
      cell.textContent = (step.vals[id] || [0, 0, 0])[0].toFixed(3);
      row.append(cell);
    }
    tbody.append(row);
  }
  table.append(tbody);
  box.append(table);
  return box;
}

export function mount(root, ctx) {
  const local = {
    list: $('actList'),
    result: $('actResult'),
    hint: $('actHint'),
    summary: $('actionSummary'),
    stop: $('btnActStop'),
    enable: $('actEnable'),
    kp: $('actKp'),
    kd: $('actKd'),
    loop: $('actLoop'),
    file: $('actFile'),
    importButton: $('btnActImport'),
  };
  let actions = [];
  let bvhSources = [];
  let catalogKey;
  let catalogRequest;
  local.stop.textContent = '取消轨迹';
  local.stop.title = 'JTC cancel 并保持位置；全部软停请使用停止控件';

  function setResult(text, kind = '') {
    local.result.textContent = text;
    local.result.className = `result tiny${kind ? ` ${kind}` : ''}`;
  }

  function updatePlayingRow() {
    const player=playbackSnapshot();
    for (const row of local.list.querySelectorAll('.act-row')) {
      row.classList.toggle('playing',
        player.phase !== 'idle' && row.dataset.name === player.name);
    }
  }

  function renderList() {
    local.list.replaceChildren();
    if (!actions.length && !bvhSources.length) {
      const empty = document.createElement('div');
      empty.className = 'tiny muted';
      empty.textContent = '当前模型没有已保存的动作。可在动作工作台转换并保存 BVH。';
      local.list.append(empty);
      return;
    }
    for (const action of actions) {
      const row = document.createElement('div');
      row.className = 'act-row';
      row.dataset.name = action.name;
      const play = document.createElement('button');
      play.className = 'secondary';
      play.textContent = '播放';
      play.dataset.action = 'play';
      play.dataset.name = action.name;
      play.disabled = Boolean(action.error);
      const name = document.createElement('span');
      name.className = 'act-name';
      name.textContent = action.name;
      name.dataset.action = 'preview';
      name.dataset.name = action.name;
      name.title = '点击展开 / 收起文件内容';
      const meta = document.createElement('span');
      meta.className = 'tiny muted act-meta';
      meta.textContent = action.error
        ? `不可用：${action.error}`
        : `${action.target_count} 台 · ${action.point_count} 点 · `
          + `${(action.duration_ms / 1000).toFixed(1)} s`;
      row.append(play, name, meta);
      local.list.append(row);
      if (expandedName === action.name) {
        local.list.append(renderPreview(action));
      }
    }
    for (const source of bvhSources) {
      const row = document.createElement('div');
      row.className = 'act-row';
      row.dataset.name = source;
      const play = document.createElement('button');
      play.className = 'secondary';
      play.textContent = '播放';
      play.dataset.action = 'play';
      play.dataset.name = source;
      const name = document.createElement('span');
      name.className = 'act-name';
      name.textContent = source;
      const meta = document.createElement('span');
      meta.className = 'tiny muted act-meta';
      meta.textContent = 'BVH · 关节轨迹';
      row.append(play, name, meta);
      local.list.append(row);
    }
    updatePlayingRow();
  }

  // 展开/收起某条动作的内容：展开状态与文本缓存在模块级，列表重建后原样恢复
  async function togglePreview(name) {
    if (expandedName === name) {
      expandedName = '';
      renderList();
      return;
    }
    expandedName = name;
    renderList();
    if (previewCache.has(name)) return;
    try {
      const response = await fetch(
        `/api/actions/${encodeURIComponent(name)}`, { cache: 'no-store' });
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      previewCache.set(name, loadSeries(JSON.parse(await response.text())));
    } catch (error) {
      previewCache.set(name, { error: `解析失败：${error.message}` });
    }
    if (expandedName === name) renderList();   // 加载完成时仍处于展开态才重绘
  }

  async function refreshActions() {
    const selection = bvhSelection(MODEL);
    const key = JSON.stringify(selection);
    if (key !== catalogKey && bvhSources.length) {
      bvhSources = [];
      renderList();
    }
    catalogKey = key;
    catalogRequest?.abort();
    const request = new AbortController();
    catalogRequest = request;
    async function read(path, options) {
      const response = await fetch(path, { cache: 'no-store', signal: request.signal, ...options });
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      return response.json();
    }
    const results = await Promise.allSettled([
      read('/api/actions'),
      (async () => {
        const urdf_text = await bvhModelText(selection.model, request.signal);
        return read('/api/motion/sources/status', {
          method: 'POST', headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ ...selection, urdf_text }),
        });
      })(),
    ]);
    if (request.signal.aborted) return;
    const [json, bvh] = results;
    if (json.status === 'fulfilled') actions = json.value.actions || [];
    bvhSources = bvh.status === 'fulfilled' ? (bvh.value.sources || [])
      .filter(source => source.saved === true && typeof source.name === 'string')
      .map(source => source.name) : [];
    renderList();
    render();
    const failures = results.filter(result => result.status === 'rejected');
    if (failures.length) setResult(`读取部分动作库失败：${failures.map(result => result.reason.message).join('；')}`, 'err');
  }

  async function importAction(name, text, overwrite) {
    const response = await fetch('/api/actions', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ name, text, overwrite }),
    });
    const payload = await response.json().catch(() => ({}));
    return { response, payload };
  }

  async function onImport() {
    const file = local.file.files[0];
    if (!file) {
      setResult('先选择一个 JSON 文件（MotorTrajectory 载荷）', 'err');
      return;
    }
    let text;
    try {
      text = await file.text();
      JSON.parse(text);   // 本地先校验，避免把错误格式传给后端
    } catch (error) {
      setResult(`不是合法 JSON：${error.message}`, 'err');
      return;
    }
    let { response, payload } = await importAction(file.name, text, false);
    if (response.status === 409 && payload.exists) {
      if (!window.confirm(`动作库已有 ${file.name}，覆盖它吗？`)) {
        setResult('已取消导入');
        return;
      }
      ({ response, payload } = await importAction(file.name, text, true));
    }
    if (!response.ok) {
      setResult(`导入失败：${payload.error || `HTTP ${response.status}`}`, 'err');
      return;
    }
    setResult(`已导入 ${payload.action.name}（${payload.action.target_count} 台 · `
      + `${payload.action.point_count} 点）`, 'ok');
    local.file.value = '';
    await refreshActions();
  }

  local.list.addEventListener('click', (event) => {
    const play = event.target.closest('button[data-action="play"]');
    if (play) {
      playAction(play.dataset.name, {
        kp: readNumber(local.kp, 30),
        kd: readNumber(local.kd, 1),
        loop: local.loop.checked,
        enable: local.enable.checked,
      });
      return;
    }
    const preview = event.target.closest('[data-action="preview"]');
    if (preview) togglePreview(preview.dataset.name);
  });
  local.stop.addEventListener('click', stopPlayback);
  local.importButton.addEventListener('click', onImport);
  local.enable.addEventListener('change', () => render());

  function refreshOnRun() {
    if (location.hash === '#/run') void refreshActions();
  }
  globalThis.addEventListener?.('hashchange', refreshOnRun);

  const render = () => {
    if (catalogKey !== JSON.stringify(bvhSelection(MODEL))) void refreshActions();
    const player=playbackSnapshot();
    const running = player.phase === 'running';
    local.stop.disabled = !(running || player.phase === 'done');
    local.summary.textContent = running || player.phase === 'done'
      ? `${player.name}${player.loop ? ' · 循环' : ''}`
      : player.phase === 'error' ? '错误' : `${bvhSources.length} BVH · ${actions.length} JSON`;
    local.hint.textContent = '播放：就绪检查 → '
      + (local.enable.checked ? '武装 kp/kd（已勾选「使能」）' : '跳过武装')
      + ' → 已保存的 BVH 关节轨迹 / JSON 按文件插值 → JTC 播放（含起点引导，末点验收）；取消保持位置，全部软停请使用停止控件；点 JSON 文件名展开内容';
    if (player.phase === 'error') setResult(player.error, 'err');
    else if (player.detail) setResult(player.detail, running ? '' : 'ok');
    updatePlayingRow();
  };

  refreshActions();
  render();
  return {
    render,
    unmount() {
      catalogRequest?.abort();
      globalThis.removeEventListener?.('hashchange', refreshOnRun);
      // 播放调度在模块级：切屏不打断已下发的动作
    },
  };
}

function readNumber(input, fallback) {
  const value = parseFloat(input.value);
  return Number.isFinite(value) && value >= 0 ? value : fallback;
}
