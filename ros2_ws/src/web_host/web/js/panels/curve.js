// 曲线面板：canvas 2D 绘制 store.chartBuf 的通道曲线（搬自旧 app.js 的曲线段）。
// 缓冲上限在写入侧（store.chartPush 封顶 1500 样本/通道），旧的 chartTrim 已删。
import { store, DIRTY } from '../core/store.js';
import { $, fmt, CHART_COLORS } from '../core/format.js';

let lastIdsKey = '';

// 电机下拉：只在电机集合变化时重建（旧版 1s 一次无条件重建，会把正在展开的下拉打断）。
function fillChartMotorSelect() {
  const sel = $('chartMotor');
  const ids = [...store.motors.keys()].sort((a, b) => a - b);
  const key = ids.join(',');
  if (key === lastIdsKey) return;
  lastIdsKey = key;
  const cur = sel.value;
  sel.innerHTML = '<option value="0">全部</option>'
    + ids.map((id) => `<option value="${id}">电机 ${id}</option>`).join('');
  if (cur && ids.includes(parseInt(cur, 10))) sel.value = cur;
}

function drawChart() {
  const canvas = $('chart');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  const W = canvas.width, H = canvas.height;
  const padL = 52, padR = 10, padT = 12, padB = 22;
  const plotW = W - padL - padR, plotH = H - padT - padB;
  const now = Date.now();
  const start = now - store.chart.windowMs;
  const ch = $('chartCh').value;
  const selId = parseInt($('chartMotor').value, 10) || 0;

  ctx.fillStyle = '#0a0f1e';
  ctx.fillRect(0, 0, W, H);
  // 网格
  ctx.strokeStyle = '#1c2540';
  ctx.lineWidth = 1;
  for (let i = 0; i <= 4; i++) {
    const y = padT + plotH * i / 4;
    ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(W - padR, y); ctx.stroke();
  }
  for (let i = 0; i <= 6; i++) {
    const x = padL + plotW * i / 6;
    ctx.beginPath(); ctx.moveTo(x, padT); ctx.lineTo(x, padT + plotH); ctx.stroke();
  }

  // 收集序列
  const series = [];
  for (const [id, b] of store.chartBuf) {
    if (selId && id !== selId) continue;
    if (!b[ch] || !b.t.length) continue;
    series.push({id, b});
  }
  let min = Infinity, max = -Infinity;
  for (const {b} of series) {
    for (let i = 0; i < b.t.length; i++) {
      if (b.t[i] < start) continue;
      const v = b[ch][i];
      if (Number.isFinite(v)) { if (v < min) min = v; if (v > max) max = v; }
    }
  }
  ctx.fillStyle = '#7e8bab';
  ctx.font = '12px sans-serif';
  ctx.textAlign = 'left';
  if (!series.length || !Number.isFinite(min)) {
    ctx.fillText('暂无数据（等待运动/状态流）', padL + 8, padT + 18);
    $('chartHint').textContent = '等待数据…';
    return;
  }
  if (min === max) { min -= 1; max += 1; }
  const span = max - min;
  min -= span * 0.08; max += span * 0.08;
  // y 轴标签
  ctx.textAlign = 'right';
  for (let i = 0; i <= 4; i++) {
    const v = max - (max - min) * i / 4;
    ctx.fillText(fmt(v, 2), padL - 6, padT + plotH * i / 4 + 4);
  }
  // 时间标签
  ctx.textAlign = 'center';
  for (let i = 0; i <= 2; i++) {
    const t = start + store.chart.windowMs * i / 2;
    const x = padL + plotW * i / 2;
    ctx.fillText('-' + Math.round((now - t) / 1000) + 's', x, H - 6);
  }
  const px = (t) => padL + (t - start) / store.chart.windowMs * plotW;
  const py = (v) => padT + (max - v) / (max - min) * plotH;
  let idx = 0;
  for (const {id, b} of series) {
    const color = CHART_COLORS[idx++ % CHART_COLORS.length];
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.6;
    ctx.beginPath();
    let pen = false;
    for (let i = 0; i < b.t.length; i++) {
      if (b.t[i] < start) continue;
      const v = b[ch][i];
      if (!Number.isFinite(v)) { pen = false; continue; }
      const x = px(b.t[i]), y = py(v);
      if (pen) ctx.lineTo(x, y); else { ctx.moveTo(x, y); pen = true; }
    }
    ctx.stroke();
    ctx.fillStyle = color;
    ctx.textAlign = 'left';
    ctx.fillText('#' + id, padL + 8, padT + 12 + 14 * idx);
  }
  ctx.textAlign = 'left';
  ctx.fillStyle = '#7e8bab';
  ctx.font = '11px sans-serif';
  const legY = padT + 12 + 14 * idx;
  ctx.fillText($('chartCh').selectedOptions[0].text
    + ' · ' + (store.chart.windowMs / 1000) + 's 窗口' + (store.chart.running ? '' : '（已暂停）'),
    padL, idx ? legY + 2 : padT + 18);
  $('chartHint').textContent = series.length
    ? `绘制 ${series.length} 条 · 范围 [${fmt(min, 2)}, ${fmt(max, 2)}]`
    : '无数据';
}

export function mount(root, ctx) {
  lastIdsKey = '';   // 重新挂载后下拉是空的，必须允许重建
  $('chartCh').onchange = drawChart;
  $('chartMotor').onchange = drawChart;
  $('chartWindow').onchange = () => {
    store.chart.windowMs = parseInt($('chartWindow').value, 10);
    drawChart();
  };
  $('btnChartPause').onclick = () => {
    store.chart.running = !store.chart.running;
    $('btnChartPause').textContent = store.chart.running ? '暂停' : '继续';
  };
  $('btnChartClear').onclick = () => { store.chartBuf.clear(); drawChart(); };

  const render = (dirty) => {
    if (!(dirty.has(DIRTY.MOTION) || dirty.has(DIRTY.STATUS) || dirty.has(DIRTY.TOPOLOGY))) return;
    fillChartMotorSelect();
    drawChart();
  };
  render(new Set([DIRTY.MOTION, DIRTY.STATUS, DIRTY.TOPOLOGY]));
  // 与旧版 setInterval(drawChart, 200) 同拍；无 WebGL，unmount 无需释放 canvas。
  return {render, intervalMs: 200, unmount() {}};
}
