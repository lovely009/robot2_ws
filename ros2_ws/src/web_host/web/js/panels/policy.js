// 使能与增益（/micro_ros_do/motor_policy）：只写、固件不回读——表格里显示的是
// 页面最后一次发出去的值（store.policy）。搬自重构前 app.js 的策略段。
import { store, DIRTY, paramRowIds, writesAllowed } from '../core/store.js';
import { $ } from '../core/format.js';
import { pub, isConnected } from '../core/ros.js';

// 每个下行消息一条关联 id。与 seq 同法用启动时间把起点垫高（见下方代数记录的
// 说明）；只用于跨系统日志对齐与回执关联，从 0 起也能跑，但垫高后更易读。
let polSeq = (Date.now() & 0x7fffffff) >>> 0;

// 「代数严格前进」的防复权门。
// 稀疏语义：只影响下发的那些电机，其余保持原状。
// 固件**不回读** kp/kd，所以表格里显示的是页面最后一次发出去的值（store.policy）。
// 固件的代数门是**跨页面持久**的：表项进 FAULT/OFFLINE/CONFIG_CONFLICT 时把当时
// 的代数锁存下来，之后「enabled=true 且代数 <= 锁存值」一律拒绝（见
// MotorPolicyArray.msg）。所以这个计数器不能从 0 起——刷新页面/重开浏览器只是把
// 我们这一侧归零，固件那边记得的仍是上一轮的值，于是每一条「武装」都被静默拒
// （只有事件日志的 CONFIG_RESULT reason=2 能看出），而轨迹照旧受理、照旧
// completed：表现就是"播放完成但机构不动"。与 seq 同法，用启动时间把起点垫高。
// 代价：代数不再连续可读，判"是不是新意图"看的是它是否前进，而不是它等于几。
let policyGeneration = (Date.now() & 0x7fffffff) >>> 0;

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
  // 保护被关掉的台：该关节没有任何限位在跑（CERB 是设备侧限位，固件没有软限位
  // 兜底），所以每次武装都要显式确认。固件侧有同一道门禁（缺确认位直接拒），
  // 这里只是把"为什么按不下去"提前说清楚。
  const pc = store.paramCache.get(id);
  const protect = Number(pc ? pc.pick.limit_protect : NaN);
  const unprotected = Number.isFinite(protect) && protect === 0;
  if (el.enable.checked && unprotected) {
    if (!window.confirm(`m${id} 的设备侧限位保护当前是关闭的：\n\n` +
        '该关节没有任何限位在执行（含看门狗自锁）。确认要武装它吗？')) {
      el.enable.checked = false;
      return;
    }
  }
  policyGeneration = (policyGeneration + 1) >>> 0;
  polSeq = (polSeq + 1) >>> 0;
  pub('policy', {
    sync_time_ns: Date.now() * 1e6,
    request_id: polSeq,
    policy_generation: policyGeneration,
    count: 1,
    targets: [{motor_id: id, enabled: el.enable.checked, kp, kd,
               confirm_unprotected: el.enable.checked && unprotected}],
  });
  store.policy.set(id, {enabled: el.enable.checked, kp, kd});
  setPolResult(true, `策略已发 seq=${polSeq} gen=${policyGeneration}：m${id} ` +
    (el.enable.checked ? '武装' : '失能') + ` kp=${kp} kd=${kd}` +
    '。受理/拒绝是异步的，看事件日志的 CONFIG_RESULT');
}

// 策略面板里有没有「改了还没按策略」的输入。判据 = 控件当前值 ≠ store.policy 里
// **最后一次下发**的记录；没记录过按 0 / 未勾对待，与 renderPolicyRows 的呈现口径
// 一致。
// 为什么需要它：device_param_report 是按需电平帧，每帧都重建两张表的 innerHTML，
// 而设备参数表只看自己的行——漏判的后果是操作员勾上「启用」，不到一秒就被下一帧
// 重建抹掉，看着像"勾不上/启用不了"。
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

// 使能与增益（只写，固件不回读）：走策略话题、不在参数树里。每条策略只带一台
// （稀疏），改哪台发哪台。
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
      <td><button class="tiny-btn primary" data-pol="${id}" title="把这一行的 启用/kp/kd 作为一条策略下发（代数自动 +1）">策略</button></td>
    </tr>`;
  }).join('');
  tbody.querySelectorAll('[data-pol]').forEach((btn) => {
    btn.onclick = () => sendPolicyRow(parseInt(btn.dataset.pol, 10));
  });
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
    // 断连禁用写操作（Task 2 的统一规则）：策略按钮与 启用/kp/kd 输入框逐个设。
    // 行是每次渲染重建的，所以禁用态必须在重建之后设；「显示全部」只是筛选，不禁用。
    const allow = writesAllowed();
    for (const el of $('polRows').querySelectorAll('[data-pol], [data-pol-field]')) {
      el.disabled = !allow;
    }
  };
  $('chkAllPolicy').onchange = () => render(new Set([DIRTY.POLICY]));
  render(new Set([DIRTY.POLICY, DIRTY.SELECTED, DIRTY.TOPOLOGY, DIRTY.PARAMS]));
  return {render, intervalMs: 200, unmount() {}};
}
