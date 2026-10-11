// 设备屏：全部参数（驱动板真值 + 固件记录，可回读可写）与四个写事务的入口。
// 事务函数在 core/device.js（连接后自动读取也走它）；本面板只做渲染与按钮接线。
// 防抹除保护从订阅处理器挪进 render：订阅层已不认识面板，正在编辑的输入框仍不能被
// device_param_report 的下一帧重建抹掉。
import { store, DIRTY, paramRowIds, writesAllowed, syncWriteButtons } from '../core/store.js';
import { $, fmt, fmtParam, lifecyclePill, devSlotNames, devRecoveryText } from '../core/format.js';
import { loadParams, applyParams, saveParamsToFlash, setLimitFromPosition,
  setMotorZero, clearMotorFault, flipDirection, hasDirtyParamInput } from '../core/device.js';

// ---------- 全部参数（以驱动板为准，可回读） ----------
function renderDevRows() {
  const tbody = $('paramRows');
  const ids = paramRowIds($('chkAllParams').checked);
  if (ids.length === 0) {
    tbody.innerHTML = '<tr class="empty"><td colspan="13">'
      + (store.paramCache.size === 0
        ? '参数集为空（等待 device_param_report）'
        : `m${store.selectedId} 暂无参数记录（未发现或未回读）——勾「显示全部」看其余`)
      + '</td></tr>';
    return;
  }
  tbody.innerHTML = ids.map((id) => {
    const v = store.paramCache.get(id);
    const pick = v.pick; // 逐字段：驱动板回读优先，没回读到才用固件记录
    const row = store.motors.get(id) || {};
    const model = row.topo?.model_name || '';
    const online = row.status?.online ?? row.topo?.online;
    const lc = row.status?.lifecycle ?? row.topo?.lifecycle;
    // 来源：只按 online 判会把"型号占位/未核对"也标成驱动板——生命周期
    // CONFIG_CONFLICT(7) 表示设备参数未核对（未回读完 / 与记录不一致 / 回读
    // 失败），此时这行的数字不是设备真值，必须如实标出来。
    const lcVerified = online === true && lc !== 7;
    const src = online === undefined ? '<span class="muted">--</span>'
      : (!online ? '<span class="pill off">记录</span>'
                 : (lcVerified ? '<span class="pill mid">驱动板</span>'
                               : '<span class="pill off">未核对</span>'));
    // 设备端原值提示：页面已知"参数单位 → 设备原值"的换算（设备对象就是 cnt / mA），
    // 直接写在输入框 title 里，省去对着 CAN 日志换算。位置参数本身就是 rad
    // （与状态、URDF 同一套单位），cnt 用设备侧 65536 cnt/360° 折。速度的电机端
    // RPM 还要按减速比折算，页面拿不到该系数，故不给。
    const isCerb = row.topo?.protocol === 1;
    const raw = (f, value) => {
      if (!isCerb || !Number.isFinite(value)) return '';
      if (f === 'pos_min' || f === 'pos_max') {
        return `设备端 ${Math.round(value * 65536 / (2 * Math.PI))} cnt`;
      }
      if (f === 'torque_limit') return `设备端 ${Math.round(value * 1000)} mA`;
      return '';
    };
    // 两行对照：显示的是 pick（驱动板优先），另一个值放 title。不一致时明说，
    // 免得操作员把"记录与设备对不上"看成页面读错。
    const rowsHint = (f) => (v.rec[f] === v.dev[f]) ? ''
      : `；记录 ${fmtParam(v.rec[f])} / 驱动板 ${fmtParam(v.dev[f])}`;
    const cell = (f) => {
      const hint = raw(f, Number(pick[f])) + rowsHint(f);
      return `<input data-id="${id}" data-field="${f}" class="pnum" value="${fmtParam(pick[f])}"` +
        (hint ? ` title="${hint}"` : '') + `>`;
    };
    // 限位教学：把该台**当前关节位置**写进对应的限位槽。厂商手册要求设置限位时
    // 电机失能、人掰到限位再设——内核也只受理未运动状态（device_limit_write_allowed），
    // 所以运动中的行直接禁用按钮，别让操作员白按。
    const moving = lc === 4 || lc === 5; // ACTIVE / SAFE_STOP
    const limBtn = (side, label, full) =>
      `<button class="tiny-btn" data-setlim="${id}" data-lim="${side}"` +
      (moving ? ' disabled' : '') +
      ` title="${moving ? `该台正在运动中，限位只能在失能状态设置（先点「策略」失能）`
        : `把 m${id} 的**当前位置**设为${full}。设置时电机必须失能；超出 ±180° 会被设备侧钳制。`}">` +
      `${label}</button>`;
    const limCell = limBtn('min', '设负', '位置下限 pos_min（负向 / 左）') +
      limBtn('max', '设正', '位置上限 pos_max（正向 / 右）');
    // 保护开关：参数 1 = 保护生效（设备侧原值 0）。设备还没回读到该对象时
    // 固件给 NaN —— 这时勾选框禁用，不能让操作员以为自己看到了真值。
    const protect = Number(pick['limit_protect']);
    const protectCell = Number.isFinite(protect)
      ? `<input type="checkbox" data-id="${id}" data-field="limit_protect"` +
        (protect === 1 ? ' checked' : '') +
        ' title="限位保护：勾选 = 设备侧看门狗 + 限位保护生效；取消 = 关闭' +
        '（该关节将没有任何限位在执行）。仅未运动时受理。' +
        '上电默认开（固件在回读比对之前写回）；取消后该台每次武装都要二次确认，' +
        '且下次上电会自动恢复为开。">'
      : '<input type="checkbox" disabled title="设备还没回读到该对象">';
    // 方向：原值直通（0=CW / 1=CCW），点一下切换。翻转只改力矩方向，不改位置
    // 坐标系（实测）；风险是位置环反向，故仅在未运动时可改。
    const dir = Number(pick['direction']);
    const dirCell = Number.isFinite(dir)
      ? `<button class="tiny-btn" data-dir="${id}" data-dir-cur="${dir}"` +
        ' title="点击切换 CW / CCW。翻转只改力矩方向（位置坐标不变），' +
        '但位置环会反向：使能后会朝远离目标的方向冲。仅未运动时受理。">' +
        `${dir === 1 ? 'CCW' : 'CW'}</button>`
      : '<button class="tiny-btn" disabled title="设备还没回读到该对象">--</button>';
    // 保护确知为关的行标红：该关节此刻没有任何限位在跑。
    const unprotected = Number.isFinite(protect) && protect === 0;
    // 当前位置：与限位同单位（rad），取 50 Hz 状态流。初始值渲染一次，
    // 之后由 updateDevCurPos（render 的 200 ms 节流拍）就地改文本——整表 innerHTML
    // 重建会抹掉操作员正在敲的值，不能拿它来刷这个 50 Hz 量。
    const curPos = Number(row.motion?.position);
    return `<tr${unprotected ? ' class="unprotected"' : ''}` +
      `${unprotected ? ' title="保护已关：该关节无任何限位"' : ''}>
      <td class="num">${id}</td>
      <td>${model || '<span class="muted">--</span>'}</td>
      <td>${lifecyclePill(lc)}</td>
      <td>${src}</td>
      <td>${cell('pos_min')}</td><td>${cell('pos_max')}</td>
      <td class="num curpos" data-cur="${id}">${fmt(curPos, 4)}</td>
      <td>${limCell}</td>
      <td>${cell('vel_limit')}</td><td>${cell('torque_limit')}</td>
      <td>${protectCell}</td>
      <td>${dirCell}</td>
      <td><button class="tiny-btn" data-zero="${id}" title="把该台当前位置设为设备零点（仅失能态受理）">设零</button>
        <button class="tiny-btn" data-clearfault="${id}" title="清除故障锁存（仅非运动态受理；身份冲突锁存不可解除）">清错</button></td>
    </tr>`;
  }).join('');
  tbody.querySelectorAll('input').forEach((inp) => {
    inp.addEventListener('input', () => { inp.dataset.dirty = '1'; });
    inp.addEventListener('change', () => { inp.dataset.dirty = '1'; });
  });
  tbody.querySelectorAll('[data-dir]').forEach((btn) => {
    btn.onclick = () => flipDirection(parseInt(btn.dataset.dir, 10),
      parseInt(btn.dataset.dirCur, 10));
  });
  tbody.querySelectorAll('[data-zero]').forEach((btn) => {
    btn.onclick = () => setMotorZero(parseInt(btn.dataset.zero, 10));
  });
  tbody.querySelectorAll('[data-clearfault]').forEach((btn) => {
    btn.onclick = () => clearMotorFault(parseInt(btn.dataset.clearfault, 10));
  });
  tbody.querySelectorAll('[data-setlim]').forEach((btn) => {
    btn.onclick = () => setLimitFromPosition(parseInt(btn.dataset.setlim, 10),
      btn.dataset.lim);
  });
  renderParamSummary();
}

// 「当前位置」列的就地刷新：只改单元格文本，**不重建 innerHTML**——整表重建会把
// 操作员正在敲的限位值抹掉（同 hasDirtyParamInput 那条理由），所以这个 50 Hz 量
// 走独立的文本更新路径。
function updateDevCurPos() {
  const tb = $('paramRows');
  if (!tb) return;
  tb.querySelectorAll('td.curpos').forEach((td) => {
    const row = store.motors.get(parseInt(td.dataset.cur, 10)) || {};
    const text = fmt(Number(row.motion?.position), 4);
    if (td.textContent !== text) td.textContent = text;
  });
}

// 设备参数核对总览：把"上电核对"的结论放到最显眼处。判据是内核把未完成核对的台
// 置 CONFIG_CONFLICT(7)，具体是"与记录不一致(40)"还是"回读不成功(41)"看
// system.last_error；其余在线台按已核对显示（来源=驱动板）。
function renderParamSummary() {
  const el = $('paramSummary');
  if (!el) return;
  const ids = [...store.paramCache.keys()].sort((a, b) => a - b);
  if (ids.length === 0) {
    el.className = 'summary tiny';
    el.textContent = '设备参数核对：暂无电机';
    return;
  }
  const offline = [], blocked = [], ok = [];
  for (const id of ids) {
    const row = store.motors.get(id) || {};
    const online = row.status?.online ?? row.topo?.online;
    const lc = row.status?.lifecycle ?? row.topo?.lifecycle;
    if (online === false) offline.push(id);
    else if (lc === 7) blocked.push(id);
    else ok.push(id);
  }
  const parts = [`<b>设备参数核对：共 ${ids.length} 台</b>`,
    `<span class="pill mid">已核对 ${ok.length}</span>`];
  if (blocked.length) {
    const code = Number(store.lastError || 0);
    const why = (code === 40) ? '与固件记录不一致'
      : (code === 41 ? '回读不成功' : (code === 42 ? '本机无该台记录' : '未完成'));
    // 逐台指出"哪几项"：位图来自 DEV_INIT 事件（收到前只显示结论）。code 40 的
    // 位图是"对不上"的槽位、41 是"没读到"的槽位——两者都只是槽位名，靠 why 区分。
    let blockedMask = 0;
    const per = blocked.map((id) => {
      const info = store.devInit.get(id);
      const mask = info ? info.mask : 0;
      blockedMask |= mask;
      if (!mask) return `${id}`;
      return `${id}(${devSlotNames(mask).join('、')})`;
    });
    const suffix = (code === 41)
      ? '括号内是未收到的对象——重连或再点该行「回读」重试'
      : devRecoveryText(blockedMask);
    parts.push(`<span class="pill off">未初始化 ${blocked.length}</span>` +
      `<span class="muted">（${per.join(' ')}：${why}——${suffix}）</span>`);
  }
  if (offline.length) {
    parts.push(`<span class="pill off">离线 ${offline.length}</span>` +
      `<span class="muted">（${offline.join('/')}：显示固件记录回落值）</span>`);
  }
  el.className = 'summary tiny';
  el.innerHTML = parts.join(' ');
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
    ? `${store.paramCache.size} 台已读` + (conflict ? ` · 未初始化 ${conflict}` : '')
    : '未读取';
}

export function mount(root, ctx) {
  $('chkAllParams').onchange = () => render(new Set([DIRTY.PARAMS, DIRTY.SELECTED]));
  $('btnParamLoad').onclick = () => loadParams();
  $('btnParamApply').onclick = () => applyParams();
  $('btnParamSave').onclick = () => saveParamsToFlash();

  const render = (dirty) => {
    const structural = dirty.has(DIRTY.PARAMS) || dirty.has(DIRTY.SELECTED)
      || dirty.has(DIRTY.TOPOLOGY) || dirty.has(DIRTY.CONN);
    if (!structural && !dirty.has(DIRTY.MOTION)) return;
    if (structural) {
      if (hasDirtyParamInput()) {
        updateDevCurPos();          // 有未下发的手改：只就地刷位置，不重建
      } else {
        renderParamSummary();
        renderDevRows();
        updateDevCurPos();
        updateSumDev();
      }
      // 断连禁用写操作（Task 2 的统一规则）。三个按钮双向同步；行内控件只做
      // 「断连→禁用」这一层——它们各有独立的禁用理由（运动中的限位、没回读到
      // 的保护/方向），重连时由重建的行自然恢复，不能在这里反向 enable。
      const allow = writesAllowed();
      syncWriteButtons(['btnParamLoad', 'btnParamApply', 'btnParamSave'], allow);
      if (!allow) {
        for (const el of $('paramRows').querySelectorAll(
            'input, [data-dir], [data-zero], [data-clearfault], [data-setlim]')) {
          el.disabled = true;
        }
      }
      renderResultFromStore();
    } else {
      // motion 帧（50 Hz）经 intervalMs:200 节流后 ≈5 Hz，等价旧版的
      // setInterval(updateDevCurPos, 200)。
      updateDevCurPos();
    }
  };
  render(new Set([DIRTY.PARAMS, DIRTY.CONN]));
  return {render, intervalMs: 200, unmount() {}};
}
