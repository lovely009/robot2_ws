// 设备屏：只读电机参数、初始化核验诊断、失能态设零与故障清除。
// 事务函数在 core/device.js（连接后自动读取也走它）；本面板只做渲染与按钮接线。
import { describeMotorState } from '../core/motor_state.js';
import { store, DIRTY, markDirty, paramRowIds, paramReadState, writesAllowed,
  syncWriteButtons } from '../core/store.js';
import { $, fmt, fmtParam, lifecyclePill } from '../core/format.js';
import { refreshMotorParams, clearMotorFault, setMotorZero, motorZeroError } from '../core/device.js';

// 型号库（减速比等）——设备端 RPM 的折算系数。取一次缓存；拿到后重绘一次把
// 设备端原值填上。取不到就只显示 cnt / mA 两项，不报错。
let motorSpecs = null;
const escapeHtml = (value) => String(value).replace(/[&<>"']/g,
  (ch) => ({'&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;'}[ch]));
fetch('/api/motor-specs')
  .then((r) => r.json())
  .then((d) => { motorSpecs = d.motor_types || {}; markDirty(DIRTY.PARAMS); })
  .catch(() => { motorSpecs = {}; });

// ---------- 全部参数（以驱动板为准，可回读） ----------
function renderDevRows() {
  const tbody = $('paramRows');
  const ids = paramRowIds(true);
  const existing = new Map([...tbody.querySelectorAll('.motor-parameter-item')]
    .map((item) => [Number(item.dataset.motor), item.open]));
  if (ids.length === 0) {
    tbody.innerHTML = '';
    return;
  }
  tbody.innerHTML = ids.map((id, index) => {
    const v = store.paramCache.get(id);
    const pick = v.pick; // 逐字段：驱动板回读优先，没回读到才用固件记录
    const row = store.motors.get(id) || {};
    const model = row.topo?.model_name || '';
    const lc = row.status?.lifecycle ?? row.topo?.lifecycle;
    // 在线不代表参数已核对，必须同时有完整回读和设备核对终态。
    const availability = paramReadState(id);
    const src = `<span class="pill ${availability.verified ? 'mid' : 'off'}">${
      availability.verified ? '驱动板 · 已核对' : availability.device === 8 ? '驱动板 · 完整回读'
        : availability.device ? '部分设备回读 · 未核对'
        : availability.record ? '固件记录 · 未核对' : '无有效参数'}</span>`;
    // 设备端原值：页面已知"参数单位 → 设备原值"的换算（设备对象就是 cnt / mA /
    // 电机端 RPM），显示在输入框下，省去对着 CAN 日志换算。位置参数本身就是 rad
    // （与状态、URDF 同一套单位），cnt 用设备侧 65536 cnt/360° 折；转速是**电机端**
    // RPM，乘减速比——系数取自型号库（/api/motor-specs），取不到就不显示这一项。
    const isCerb = row.topo?.protocol === 1;
    const gear = motorSpecs && motorSpecs[model]
      ? Number(motorSpecs[model].gear_ratio) : NaN;
    const raw = (f, value) => {
      if (!isCerb || !Number.isFinite(value)) return '';
      if (f === 'pos_min' || f === 'pos_max') {
        return `${Math.round(value * 65536 / (2 * Math.PI))} cnt`;
      }
      if (f === 'torque_limit') return `${Math.round(value * 1000)} mA`;
      if (f === 'vel_limit' && Number.isFinite(gear) && gear > 0) {
        return `${Math.round(value * gear * 60 / (2 * Math.PI))} RPM`;
      }
      return '';
    };
    // 两行对照：显示的是 pick（驱动板优先），另一个值放 title。不一致时明说，
    // 免得操作员把"记录与设备对不上"看成页面读错。
    const rowsHint = (f) => (v.rec[f] === v.dev[f]) ? ''
      : `；记录 ${fmtParam(v.rec[f])} / 驱动板 ${fmtParam(v.dev[f])}`;
    const cell = (f, label = f) => {
      const dev = raw(f, pick[f]);
      const hint = (dev ? `设备端 ${dev}` : '') + rowsHint(f);
      return `<input aria-label="${label}" type="number" readonly placeholder="未读取" value="${fmtParam(pick[f])}"${hint ? ` title="${hint}"` : ''}>` +
        (dev ? `<span class="rawval muted tiny">${dev}</span>` : '');
    };
    const protectCell = Number.isFinite(pick.limit_protect)
      ? (pick.limit_protect === 1 ? '开启' : '关闭') : '未读取';
    const dirCell = Number.isFinite(pick.direction) ? (pick.direction === 1 ? 'CCW' : 'CW') : '未读取';
    const unprotected = pick.limit_protect === 0;
    // 位置按状态流就地更新，参数只在新报告到达时重建。
    const curPos = row.motion?.position;

    const field = (name, label, unit) => {
      const title = label + (unit ? `（${unit}）` : '');
      return `<div><dt>${title}</dt><dd>${cell(name, title)}</dd></div>`;
    };
    return `<details class="motor-parameter-item${unprotected ? ' unprotected' : ''}" data-motor="${id}"` +
      `${(existing.get(id) ?? index === 0) ? ' open' : ''}>
      <summary><strong>电机 ${id}</strong><span class="motor-parameter-model">${escapeHtml(model || '型号待识别')}</span>
        ${lifecyclePill(lc, describeMotorState(store,id))}${src}</summary>
      <div class="motor-parameter-body">
        ${!availability.usable ? `<p class="panel-error">电机 ${id} ${!model ? '型号尚未识别，固件尚未绑定型号；' : ''}未返回有效参数，不能视为读取成功。</p>` : ''}
        ${unprotected ? '<p class="panel-error">保护已关：该关节无任何限位。</p>' : ''}
        <h3>电机参数</h3>
        <p class="micro">参数值只读；设零须先失能，完成后核对实际位置与机械行程。</p><dl>
          ${field('node_id', '电机 ID', '')}${field('t_max', 'T_max', 'N·m')}
          <div><dt>当前通信 ID</dt><dd>${id}${Number.isFinite(pick.node_id) && pick.node_id !== id ? '（新 ID 重启电机后生效）' : ''}</dd></div>

          ${field('pos_min', '位置下限', 'rad')}${field('pos_max', '位置上限', 'rad')}
          <div><dt>当前位置（rad）</dt><dd class="curpos" data-cur="${id}">${fmt(curPos, 4)}</dd></div>

          ${field('vel_limit', '速度上限', 'rad/s')}${field('torque_limit', '电流上限', 'A')}

          <div><dt>限位保护</dt><dd>${protectCell}</dd></div>
          <div><dt>电机方向</dt><dd>${dirCell}</dd></div>
          <div><dt>维护操作</dt><dd>
            <button class="tiny-btn" data-zero="${id}" title="把该台当前位置设为设备零点（仅失能态受理）">设零</button>
            <button class="tiny-btn" data-clearfault="${id}" title="清除故障锁存（仅非运动态受理；身份冲突锁存不可解除）">清错</button>
          </dd></div>
        </dl>

      </div>
    </details>`;
  }).join('');
  tbody.querySelectorAll('[data-zero]').forEach(btn => {
    btn.onclick = () => setMotorZero(Number(btn.dataset.zero));
  });
  tbody.querySelectorAll('[data-clearfault]').forEach((btn) => {
    btn.onclick = () => clearMotorFault(parseInt(btn.dataset.clearfault, 10));
  });
}

// 位置更新不重建参数表，保留展开状态与焦点。
function updateDevCurPos() {
  const tb = $('paramRows');
  if (!tb) return;
  tb.querySelectorAll('.curpos').forEach((td) => {
    const row = store.motors.get(parseInt(td.dataset.cur, 10)) || {};
    const text = fmt(row.motion?.position, 4);
    if (td.textContent !== text) td.textContent = text;
  });
}

// 结果文本住在 store 里（core/device.js 的 writeResult 写它），因为同一批事务
// 的按钮分布在设备屏与运行屏两处。这里只负责把它画出来。
function renderResultFromStore() {
  const r = store.results.get('paramResult');
  const el = $('paramResult');
  if (!el) return;
  el.classList.toggle('ok', !!(r && r.ok));
  el.classList.toggle('err', !!(r && !r.ok));
  el.textContent = r ? r.text : '';
}

// 摘要（搬 renderFoldSummaries 的 sumDev 段）：已读台数与未初始化计数。
function updateSumDev() {
  const el = $('sumDev');
  if (!el) return;
  let conflict = 0;
  for (const r of store.motors.values()) {
    const st = r.status || {}, t = r.topo || {};
    if (t.conflict || st.lifecycle === 7) conflict++;
  }
  el.textContent = store.paramCache.size
    ? `${store.paramCache.size} 台已报告` + (conflict ? ` · 未初始化 ${conflict}` : '')
    : '未读取';
}

export function mount(root, ctx) {
  $('btnParamLoad').onclick = () => refreshMotorParams();

  const render = (dirty) => {
    const structural = dirty.has(DIRTY.PARAMS) || dirty.has(DIRTY.SELECTED)
      || dirty.has(DIRTY.TOPOLOGY) || dirty.has(DIRTY.CONN);
    if (!structural && !dirty.has(DIRTY.MOTION) && !dirty.has(DIRTY.STATUS)) return;
    if (structural) {
      renderDevRows();
      updateDevCurPos();
      updateSumDev();
      const ids = [...new Set([...store.motors.keys(), ...store.paramCache.keys()])].sort((a, b) => a - b);
      const selected = ids.includes(store.selectedId) ? store.selectedId : ids[0];
      const singleMotor = !!root.closest('.device-workspace');
      const hint = $('paramSelectionHint');
      hint.hidden = selected !== undefined && (singleMotor
        ? store.paramCache.has(selected) : store.paramCache.size > 0);
      hint.textContent = selected === undefined ? '等待电机反馈后读取参数。'
        : `电机 ${selected} 暂无参数，请读取电机参数。`;
      for (const item of $('paramRows').querySelectorAll('.motor-parameter-item')) {
        item.hidden = singleMotor && Number(item.dataset.motor) !== selected;
        if (singleMotor && !item.hidden) item.open = true;
      }
      // 断连时禁用查询与清错，维护请求仍由服务端检查运行状态。
      const allow = writesAllowed();
      syncWriteButtons(['btnParamLoad'], allow);
      if (!allow) {
        for (const el of $('paramRows').querySelectorAll(
            '[data-clearfault], [data-zero]')) {
          el.disabled = true;
        }
      }
      renderResultFromStore();
    } else {
      // motion 帧（50 Hz）经 intervalMs:200 节流后 ≈5 Hz，等价旧版的
      // setInterval(updateDevCurPos, 200)。
      updateDevCurPos();
    }
    for (const button of $('paramRows').querySelectorAll('[data-zero]')) {
      const error = motorZeroError(Number(button.dataset.zero));
      button.disabled = !writesAllowed() || Boolean(error);
      button.title = error || '把该台当前位置设为设备零点（仅失能态受理）';
    }
  };
  render(new Set([DIRTY.PARAMS, DIRTY.CONN]));
  return {render, intervalMs: 200, unmount() {}};
}
