// 使能与增益（/micro_ros_do/motor_policy）：只写、固件不回读——表格里显示的是
// 页面最后一次发出去的值（store.policy）。搬自重构前 app.js 的策略段。
import { store, DIRTY, paramRowIds, writesAllowed } from '../core/store.js';
import { $ } from '../core/format.js';
import { pub, isConnected, nextPolicyGeneration } from '../core/ros.js';

// 每个下行消息一条关联 id。与代数同法用启动时间把起点垫高（见 ros.js 的
// nextPolicyGeneration 说明）；只用于跨系统日志对齐与回执关联。
let polSeq = (Date.now() & 0x7fffffff) >>> 0;

function writeResult(id, ok, text) {
  const el = $(id);
  if (!el) return;
  el.className = 'result tiny ' + (ok ? 'ok' : 'err');
  el.textContent = text;
}
function setPolResult(ok, text) { writeResult('polResult', ok, text); }

function polInputs(id) {
  const q = (f) => $('polRows').querySelector(
    `[data-pol-field="${f}"][data-id="${id}"]`);
  return {enable: q('enabled'), kp: q('kp'), kd: q('kd')};
}

// 设备侧限位保护被关掉的台：该关节没有任何限位在跑（CERB 是设备侧限位，固件没有
// 软限位兜底），所以每次武装都要显式确认。固件侧有同一道门禁（缺确认位直接拒），
// 这里只是把"为什么按不下去"提前说清楚。动作库播放前也会用它做同一确认。
export function isUnprotected(id) {
  const pc = store.paramCache.get(id);
  const protect = Number(pc ? pc.pick.limit_protect : NaN);
  return Number.isFinite(protect) && protect === 0;
}

// 逐台下发（表格每行一个按钮）：改哪台发哪台。
function sendPolicyRow(id) {
  if (!isConnected()) { setPolResult(false, '未连接 rosbridge'); return; }
  const el = polInputs(id);
  if (!el.enable || !el.kp || !el.kd) { return; }
  const kp = parseFloat(el.kp.value);
  const kd = parseFloat(el.kd.value);
  if (!Number.isFinite(kp) || !Number.isFinite(kd) || kp < 0 || kd < 0) {
    setPolResult(false, `m${id}：kp/kd 必须是非负有限值`);
    return;
  }
  const unprotected = isUnprotected(id);
  if (el.enable.checked && unprotected) {
    if (!window.confirm(`m${id} 的设备侧限位保护当前是关闭的：\n\n` +
        '该关节没有任何限位在执行（含看门狗自锁）。确认要武装它吗？')) {
      el.enable.checked = false;
      syncEnableAll();
      return;
    }
  }
  const generation = nextPolicyGeneration();
  polSeq = (polSeq + 1) >>> 0;
  pub('policy', {
    sync_time_ns: Date.now() * 1e6,
    request_id: polSeq,
    policy_generation: generation,
    count: 1,
    targets: [{motor_id: id, enabled: el.enable.checked, kp, kd,
               confirm_unprotected: el.enable.checked && unprotected}],
  });
  store.policy.set(id, {enabled: el.enable.checked, kp, kd});
  setPolResult(true, `已下发 seq=${polSeq} gen=${generation}：m${id} ` +
    (el.enable.checked ? '武装' : '失能') + ` kp=${kp} kd=${kd}` +
    '。受理/拒绝是异步的，看事件日志的 CONFIG_RESULT');
}

// 一键下发：把**当前列出的所有行**（含尚未下发的手改）合成**一条**多目标策略。
// targets 上限 64，且是稀疏语义（只影响列出的台），所以整条臂一条消息就够——与逐台
// 「下发」的唯一差别是 count/targets 的条数：代数只 +1，而不是 18 台各 +1 代。
// 与逐台一致，值取的是**DOM 里的当前值**，不是 store.policy 里的旧记录。
function sendPolicyAll() {
  if (!isConnected()) { setPolResult(false, '未连接 rosbridge'); return; }
  const targets = [];
  const unprotected = [];
  for (const box of $('polRows').querySelectorAll('[data-pol-field="enabled"]')) {
    const id = parseInt(box.dataset.id, 10);
    const el = polInputs(id);
    if (!el.kp || !el.kd) continue;
    const kp = parseFloat(el.kp.value);
    const kd = parseFloat(el.kd.value);
    if (!Number.isFinite(kp) || !Number.isFinite(kd) || kp < 0 || kd < 0) {
      setPolResult(false, `m${id}：kp/kd 必须是非负有限值`);
      return;
    }
    if (box.checked && isUnprotected(id)) unprotected.push(id);
    targets.push({motor_id: id, enabled: box.checked, kp, kd, confirm_unprotected: false});
  }
  if (!targets.length) {
    setPolResult(false, '没有可下发的行：先点「读取」载入电机列表');
    return;
  }
  if (unprotected.length && !window.confirm(
      `m${unprotected.join('、m')} 的设备侧限位保护当前是关闭的：\n\n` +
      '这些关节没有任何限位在执行（含看门狗自锁）。确认要武装它们吗？')) return;
  for (const t of targets) t.confirm_unprotected = t.enabled && unprotected.includes(t.motor_id);
  const generation = nextPolicyGeneration();
  polSeq = (polSeq + 1) >>> 0;
  pub('policy', {
    sync_time_ns: Date.now() * 1e6,
    request_id: polSeq,
    policy_generation: generation,
    count: targets.length,
    targets,
  });
  for (const t of targets) {
    store.policy.set(t.motor_id, {enabled: t.enabled, kp: t.kp, kd: t.kd});
  }
  const armed = targets.filter((t) => t.enabled).length;
  setPolResult(true, `已一键下发 seq=${polSeq} gen=${generation}：${targets.length} 台` +
    `（武装 ${armed} / 失能 ${targets.length - armed}）` +
    '。受理/拒绝是异步的，看事件日志的 CONFIG_RESULT');
}

// 「全选使能」与各行使能框保持一致：它只改勾选状态（不下发），单行手改后反推全选框的
// 勾上 / 半选 / 未勾三态。断连或没有行时不可点——那时行内控件本来就是禁用的。
function syncEnableAll() {
  const box = $('chkPolEnableAll');
  if (!box) return;
  const rows = [...$('polRows').querySelectorAll('[data-pol-field="enabled"]')];
  const on = rows.filter((c) => c.checked).length;
  box.checked = rows.length > 0 && on === rows.length;
  box.indeterminate = on > 0 && on < rows.length;
  box.disabled = rows.length === 0 || !writesAllowed();
}

function setAllEnable(checked) {
  for (const c of $('polRows').querySelectorAll('[data-pol-field="enabled"]')) {
    c.checked = checked;
  }
  syncEnableAll();
}

// 策略面板里有没有「改了还没按策略」的输入。判据 = 控件当前值 ≠ store.policy 里
// **最后一次下发**的记录；没记录过按 0 / 未勾对待，与 renderPolicyRows 的呈现口径
// 一致。
// 为什么需要它：device_param_report 是按需电平帧，每帧都重建两张表的 innerHTML，
// 而设备参数表只看自己的行——漏判的后果是操作员勾上「使能」，不到一秒就被下一帧
// 重建抹掉，看着像"勾不上/使能不了"。
function hasPendingPolicyEdit() {
  const tb = $('polRows');
  if (!tb) return false;
  for (const inp of tb.querySelectorAll('[data-pol-field]')) {
    const last = store.policy.get(parseInt(inp.dataset.id, 10)) || {};
    if (inp.dataset.polField === 'enabled') {
      if (inp.checked !== (last.enabled === true)) return true;
    } else if (parseFloat(inp.value) !== (last[inp.dataset.polField] !== undefined
        ? last[inp.dataset.polField] : 0)) {
      return true; // 清空输入框时 parseFloat 是 NaN，恒 != 记录 → 同样算未下发
    }
  }
  return false;
}

// 使能与增益（只写，固件不回读）：走策略话题、不在参数树里。每条消息可以只带一台
// （行内「下发」）也可以带上列出的全部（「一键下发」），都是稀疏语义。
function renderPolicyRows() {
  const tbody = $('polRows');
  const ids = paramRowIds($('chkAllPolicy').checked);
  if (ids.length === 0) {
    tbody.innerHTML = '<tr class="empty"><td colspan="6">连接后点「读取」载入电机列表</td></tr>';
    return;
  }
  tbody.innerHTML = ids.map((id) => {
    const row = store.motors.get(id) || {};
    const model = row.topo?.model_name || '';
    const pol = store.policy.get(id) || {};
    const polNum = (f) => `<input class="pnum" type="number" min="0" step="0.1"
      data-id="${id}" data-pol-field="${f}" value="${pol[f] !== undefined ? pol[f] : 0}"
      title="页面最后一次下发的值；固件不回读 kp/kd。设备参数未核对的台会被直接拒">`;
    const polEnable = `<input type="checkbox" data-id="${id}" data-pol-field="enabled"
      ${pol.enabled ? ' checked' : ''}
      title="页面最后一次下发的值。设备参数未核对的台会被直接拒（CONFIG_RESULT reason=2）">`;
    return `<tr>
      <td class="num">${id}</td>
      <td>${model || '<span class="muted">--</span>'}</td>
      <td>${polEnable}</td>
      <td>${polNum('kp')}</td><td>${polNum('kd')}</td>
      <td><button class="tiny-btn primary" data-pol="${id}" title="把这一行的 使能/kp/kd 作为一条策略下发（代数自动 +1）">下发</button></td>
    </tr>`;
  }).join('');
  tbody.querySelectorAll('[data-pol]').forEach((btn) => {
    btn.onclick = () => sendPolicyRow(parseInt(btn.dataset.pol, 10));
  });
  tbody.querySelectorAll('[data-pol-field="enabled"]').forEach((box) => {
    box.onchange = syncEnableAll;
  });
  syncEnableAll();
}

export function mount(root, ctx) {
  const render = (dirty) => {
    if (!(dirty.has(DIRTY.POLICY) || dirty.has(DIRTY.SELECTED) || dirty.has(DIRTY.TOPOLOGY)
          || dirty.has(DIRTY.PARAMS) || dirty.has(DIRTY.CONN))) return;
    if (hasPendingPolicyEdit()) return;    // 有未下发的手改就不重建（见函数注释）
    renderPolicyRows();
    const ids = paramRowIds($('chkAllPolicy').checked);
    $('sumPol').textContent = (ids.length ? `${ids.length} 台` : '--')
      + (store.policy.size ? ` · 已下发 ${store.policy.size}` : '');
    // 断连禁用写操作（Task 2 的统一规则）：策略按钮与 使能/kp/kd 输入框逐个设。
    // 行是每次渲染重建的，所以禁用态必须在重建之后设；「显示全部」只是筛选，不禁用。
    const allow = writesAllowed();
    for (const el of $('polRows').querySelectorAll('[data-pol], [data-pol-field]')) {
      el.disabled = !allow;
    }
    $('btnPolSendAll').disabled = !allow;
    syncEnableAll();
  };
  $('chkAllPolicy').onchange = () => render(new Set([DIRTY.POLICY]));
  $('chkPolEnableAll').onchange = (e) => setAllEnable(e.target.checked);
  $('btnPolSendAll').onclick = sendPolicyAll;
  render(new Set([DIRTY.POLICY, DIRTY.SELECTED, DIRTY.TOPOLOGY, DIRTY.PARAMS]));
  return {render, intervalMs: 200, unmount() {}};
}
