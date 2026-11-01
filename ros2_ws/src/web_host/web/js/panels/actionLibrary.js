// MotorTrajectory 动作文件经低频客户端转换成 JTC action，不再上传 MCU。
// 任意长度动作按 ≤256 点串行分段，下一窗仅在收到 JTC 成功结果后提交。
// 播放调度在模块级，切屏不打断动作。
import { store, MODEL, markDirty, DIRTY } from '../core/store.js';
import { $, TRAJ_MAX_POINTS, TRAJ_MAX_MOTORS, TRAJ_STATE_TIMEOUT_PAD_MS } from '../core/format.js';
import { pub, isConnected, nextPolicyGeneration, getConnectionGeneration } from '../core/ros.js';
import { playbackReadinessError } from './command.js';
import { limitsError } from '../core/host_limits.js';
import { controlSessionError, rawCouplingError } from '../core/joint_control.js';
import { controlTargetsError, describeMotorState } from '../core/motor_state.js';
import { isUnprotected } from './policy.js';

const ALIGN_MS = 2500;        // 由 JTC 插值完成起点对齐
const ARM_SETTLE_MS = 300;    // 武装后等策略被受理
const CANCEL_SETTLE_MS = 800; // 等待 JTC cancel 完成
const WINDOW_STEPS_CAP = 60;  // 每窗最多 60 个未来时刻，点数另按电机数收窄

const player = {
  phase: 'idle',  // idle | running | done | error
  detail: '',     // 给 UI 的一句话
  error: '',
  name: '',
  ids: [],
  model: '',
  modelMetadata: null,
  firstStep: null,
  windows: [],
  winIndex: 0,
  round: 0,
  loop: false,
  enable: true,
  kp: 30,
  kd: 1,
  windowTid: 0,
  windowDurMs: 0,
  sessionId: Date.now(),  // 刷新后不复用网关已关闭的 (session_id, trajectory_id)。
  tidSeq: 90000,
  cancelRequested: false,
  connectionGeneration: 0,
};

function assertPlaybackSession() {
  if (player.cancelRequested) throw new Error('已停止');
  if (!isConnected() || player.connectionGeneration !== getConnectionGeneration()) {
    throw new Error('连接已更换或中断，请重新确认后播放');
  }
  if (MODEL !== player.model || store.modelTransmissions !== player.modelMetadata) {
    throw new Error('播放期间模型或传动配置已更换，请重新确认');
  }
  const error = controlSessionError(store,player.model) || rawCouplingError(store,player.model,player.ids);
  if (error) throw new Error(error);
}

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
  head.textContent = `目标 m${ids.join(', m')} · ${steps.length} 时刻`
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

function sleep(ms) { return new Promise((resolve) => setTimeout(resolve, ms)); }

function announce() { markDirty(DIRTY.TRAJ); }

// 空轨迹只取消 JTC 并保持位置；全部软停另走 /control/stop。
function cancelTrajectory() {
  pub('trajectory', {
    sync_time_ns: Date.now() * 1e6,
    sequence: 0,
    trajectory_id: 0,
    segment_index: 0,
    segment_count: 1,
    count: 0,
    target_count: 0,
    target_ids: [],
    points: [],
  });
}

// MotorTrajectory 载荷 → {ids, steps=[{t, vals:{id:[pos,vel,eff]}}]}。
// 同一时刻缺台用上一时刻补齐（固件要求每台在每个声明时刻都有路点，缺席整条拒收）。
export function loadSeries(document_) {
  if (!document_ || typeof document_ !== 'object') throw new Error('不是 JSON 对象');
  const ids = [...new Set((document_.target_ids || []).map(Number))];
  if (!ids.length) throw new Error('动作缺 target_ids');
  const buckets = new Map();
  for (const point of document_.points || []) {
    const id = Number(point.motor_id);
    if (!ids.includes(id)) continue;
    const t = Math.round(Number(point.time_from_start_ms));
    if (!Number.isFinite(t)) throw new Error('路点缺 time_from_start_ms');
    const effort = point.effort !== undefined ? point.effort : (point.torque || 0);
    if (!buckets.has(t)) buckets.set(t, {});
    buckets.get(t)[id] = [
      Number(point.position), Number(point.velocity || 0), Number(effort),
    ];
  }
  const zero = Object.fromEntries(ids.map((id) => [id, [0, 0, 0]]));
  const steps = [];
  for (const t of [...buckets.keys()].sort((a, b) => a - b)) {
    const vals = { ...(steps.length ? steps[steps.length - 1].vals : zero) };
    Object.assign(vals, buckets.get(t));
    steps.push({ t, vals });
  }
  if (!steps.length) throw new Error('动作没有路点');
  return { ids, steps };
}

// 每窗保留一个内部时间基准点；发布时只发送其后的未来路点。
export function makeWindows(steps, idCount) {
  if (!Number.isInteger(idCount) || idCount < 1 || idCount > TRAJ_MAX_MOTORS) {
    throw new Error(`动作电机数量须在 1～${TRAJ_MAX_MOTORS}`);
  }
  const maxSteps = Math.min(WINDOW_STEPS_CAP, Math.floor(TRAJ_MAX_POINTS / idCount));
  const windows = [];
  for (let i = 1; i < steps.length; i += maxSteps) {
    windows.push([steps[i-1], ...steps.slice(i,i+maxSteps)]);
  }
  return windows;
}

function sendWindow(chunk, t0 = chunk[0].t) {
  assertPlaybackSession();
  const authority=controlTargetsError(store,player.ids);
  if (authority) throw new Error(authority);
  player.tidSeq = (player.tidSeq + 1) >>> 0;
  const tid = player.tidSeq;
  const points = [];
  for (const step of chunk) {
    if (step.t === t0) continue;
    for (const id of player.ids) {
      const [position, velocity, effort] = step.vals[id] || [0, 0, 0];
      points.push({
        time_from_start_ms: step.t - t0,
        motor_id: id,
        position,
        velocity,
        effort,
      });
    }
  }
  if (!points.length || points.length > TRAJ_MAX_POINTS
      || points.some(p => !Number.isFinite(p.time_from_start_ms) || p.time_from_start_ms <= 0)) {
    throw new Error('轨迹窗口必须包含正时刻路点，且不超过 256 点');
  }
  const blocked = limitsError(points, player.model);
  if (blocked) throw new Error(blocked);
  pub('trajectory', {
    sync_time_ns: Date.now() * 1e6,
    sequence: tid,
    session_id: player.sessionId,
    trajectory_id: tid,
    segment_index: 0,
    segment_count: 1,
    count: points.length,
    target_count: player.ids.length,
    target_ids: player.ids,
    points,
  });
  store.traj.activeTrajectoryId = tid;
  player.windowTid = tid;
  player.windowDurMs = chunk[chunk.length - 1].t - t0;
  return tid;
}

// completed 由客户端根据 JTC 成功结果映射，发布成功和时间到达都不算完成。
export function trajectoryWindowResult(state, tid, durationMs) {
  if (!state || state.trajectory_id !== tid) return 'waiting';
  if (!state.active && state.completed && state.elapsed_ms >= durationMs) return 'done';
  return state.active ? 'waiting' : 'canceled';
}

// 等待当前 action 的成功终态；本地计时只用于回执超时。
async function waitWindow(tid) {
  const deadline = Date.now() + player.windowDurMs + TRAJ_STATE_TIMEOUT_PAD_MS;
  while (Date.now() < deadline) {
    if (player.cancelRequested) return 'stopped';
    assertPlaybackSession();
    const state = store.traj.state;
    const result = trajectoryWindowResult(state, tid, player.windowDurMs);
    if (result !== 'waiting') return result;
    await sleep(50);
  }
  return 'timeout';
}

async function armMotors() {
  assertPlaybackSession();
  const generation = nextPolicyGeneration();
  pub('policy', {
    sync_time_ns: Date.now() * 1e6,
    request_id: generation,
    policy_generation: generation,
    count: player.ids.length,
    targets: player.ids.map((id) => ({
      motor_id: id,
      enabled: true,
      kp: player.kp,
      kd: player.kd,
      confirm_unprotected: isUnprotected(id),
    })),
  });
  for (const id of player.ids) {
    store.policy.set(id, { enabled: true, kp: player.kp, kd: player.kd });
  }
  markDirty(DIRTY.POLICY);
  await sleep(ARM_SETTLE_MS);
  assertPlaybackSession();
  if (store.controlStates?.seen) {
    const deadline=Date.now()+3000;
    while (true) {
      assertPlaybackSession();
      if (player.cancelRequested) throw new Error('已停止');
      if (!controlTargetsError(store,player.ids) && player.ids.every(id =>
          describeMotorState(store,id).generation===generation)) break;
      if (Date.now()>=deadline) throw new Error('MCU 未确认本次使能策略，拒绝播放');
      await sleep(50);
    }
  }
}

// 对齐也使用 JTC，取消时才能统一保持位置，不留下 manual_controller 的旧目标。
async function alignToStart() {
  const first = player.firstStep;
  assertPlaybackSession();
  const target = Object.fromEntries(player.ids.map(id => [id, [first.vals[id][0], 0, 0]]));
  const tid = sendWindow([{t:ALIGN_MS,vals:target}], 0);
  const result = await waitWindow(tid);
  if (result !== 'done') throw new Error(result === 'stopped' ? '已停止' : `起点对齐未获 JTC 成功结果（${result}）`);
}

export async function playAction(name, options) {
  if (player.phase === 'running') return;
  if (!isConnected()) {
    player.phase = 'error';
    player.error = '未连接 rosbridge';
    announce();
    return;
  }
  player.phase = 'running';
  player.error = '';
  player.cancelRequested = false;
  player.connectionGeneration = getConnectionGeneration();
  player.model = MODEL;
  player.modelMetadata = store.modelTransmissions;
  player.ids = [];
  player.name = name;
  player.kp = options.kp;
  player.kd = options.kd;
  player.loop = options.loop;
  player.enable = options.enable;
  try {
    player.detail = `读取 ${name}…`;
    announce();
    const response = await fetch(
      `/api/actions/${encodeURIComponent(name)}`, { cache: 'no-store' });
    assertPlaybackSession();
    if (!response.ok) throw new Error(`读取动作失败：HTTP ${response.status}`);
    const document_ = await response.json();
    assertPlaybackSession();
    const { ids, steps } = loadSeries(document_);
    const coupled=rawCouplingError(store,player.model,ids);
    if (coupled) throw new Error(coupled);
    const limitError = limitsError(steps.flatMap(step => ids.map(id => ({
      motor_id: id, position: step.vals[id][0],
    }))), player.model);
    if (limitError) throw new Error(limitError);
    const blocked = playbackReadinessError(ids);
    if (blocked) throw new Error(blocked);
    const unprotected = ids.filter(isUnprotected);
    if (unprotected.length && !window.confirm(
        `m${unprotected.join('、m')} 的设备侧限位保护当前是关闭的：\n\n` +
        '这些关节没有任何限位在执行（含看门狗自锁）。确认继续播放吗？')) {
      throw new Error('已取消（限位保护未确认）');
    }
    player.ids = ids;
    player.firstStep = steps[0];
    player.windows = makeWindows(steps, ids.length);

    player.detail = '取消旧轨迹…';
    announce();
    cancelTrajectory();
    await sleep(CANCEL_SETTLE_MS);
    assertPlaybackSession();
    if (player.cancelRequested) throw new Error('已停止');

    if (player.enable) {
      player.detail = `武装 kp=${player.kp} kd=${player.kd}…`;
      announce();
      await armMotors();
      if (player.cancelRequested) throw new Error('已停止');
    } else {
      player.detail = '跳过武装（未勾选「使能」）';
      announce();
    }

    for (player.round = 0; ; player.round++) {
      player.detail = '对齐起点…';
      announce();
      await alignToStart();
      assertPlaybackSession();
      for (let index = 0; index < player.windows.length; index++) {
        if (player.cancelRequested) throw new Error('已停止');
        player.winIndex = index;
        const tid = sendWindow(player.windows[index]);
        player.detail = `${player.loop ? `第${player.round + 1}轮 ` : ''}`
          + `窗 ${index + 1}/${player.windows.length} · ${player.windowDurMs}ms`;
        announce();
        const verdict = await waitWindow(tid);
        if (verdict === 'done') continue;
        if (verdict === 'stopped') throw new Error('已停止');
        if (verdict === 'canceled') throw new Error(`JTC 未成功（取消/拒绝/执行失败）：${store.commandStatus.data?.reject || store.commandStatus.data?.reason || '请核对控制状态'}`);
        throw new Error(`窗 ${index + 1} 无 JTC 结果回执：${store.commandStatus.data?.reject || '请核对控制客户端状态'}`);
      }
      if (!player.loop) break;
    }
    player.phase = 'done';
    player.detail = '播放完成（JTC 成功）';
    announce();
  } catch (error) {
    player.phase = 'error';
    player.error = error.message;
    announce();
  }
}

export function stopPlayback() {
  // 运行中先置停止位（各 await 点会退出）；保持态（done）直接取消释放
  const wasRunning = player.phase === 'running';
  player.cancelRequested = true;
  cancelTrajectory();
  if (!wasRunning) {
    player.phase = 'idle';
    player.detail = '已请求 JTC 取消并保持位置；全部软停请使用停止控件';
    announce();
  }
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
  local.stop.textContent = '取消轨迹';
  local.stop.title = 'JTC cancel 并保持位置；全部软停请使用停止控件';

  function setResult(text, kind = '') {
    local.result.textContent = text;
    local.result.className = `result tiny${kind ? ` ${kind}` : ''}`;
  }

  function updatePlayingRow() {
    for (const row of local.list.querySelectorAll('.act-row')) {
      row.classList.toggle('playing',
        player.phase !== 'idle' && row.dataset.name === player.name);
    }
  }

  function renderList() {
    local.list.replaceChildren();
    if (!actions.length) {
      const empty = document.createElement('div');
      empty.className = 'tiny muted';
      empty.textContent = '动作库为空（action_library/ 下没有 JSON）';
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
    try {
      const response = await fetch('/api/actions', { cache: 'no-store' });
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      actions = (await response.json()).actions || [];
      renderList();
    } catch (error) {
      setResult(`读取动作库失败：${error.message}`, 'err');
    }
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

  const render = () => {
    const running = player.phase === 'running';
    local.stop.disabled = !(running || player.phase === 'done');
    local.summary.textContent = running || player.phase === 'done'
      ? `${player.name}${player.loop ? ' · 循环' : ''}`
      : player.phase === 'error' ? '错误' : '--';
    local.hint.textContent = '播放：就绪检查 → '
      + (local.enable.checked ? '武装 kp/kd（已勾选「使能」）' : '跳过武装')
      + ' → 对齐起点 → JTC 分段播放（≤256 点/窗，成功后交棒）；取消保持位置，全部软停请使用停止控件；点文件名展开内容';
    if (player.phase === 'error') setResult(player.error, 'err');
    else if (player.detail) setResult(player.detail, running ? '' : 'ok');
    updatePlayingRow();
  };

  refreshActions();
  render();
  return {
    render,
    unmount() {
      // 播放调度在模块级：切屏不打断已下发的动作
    },
  };
}

function readNumber(input, fallback) {
  const value = parseFloat(input.value);
  return Number.isFinite(value) && value >= 0 ? value : fallback;
}
