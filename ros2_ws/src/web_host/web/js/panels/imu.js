// IMU 面板（诊断屏）：Roll/Pitch/Yaw、温度、更新年龄、四元数 + 3D 姿态角与人工地平仪。
// 纯显示：数据由 core/ros.js 订阅 /imu/{data,temp} 写入 store.imu / store.temp，
// 本模块不订阅、不发布话题；CSS 控制显示尺寸，canvas 绘制缓冲使用构造时的像素尺寸。
import { store, DIRTY } from '../core/store.js';
import { $, fmt, rad2deg } from '../core/format.js';

// 3D 姿态：body→world 旋转矩阵（四元数）投影一个立方体与三轴（X 前红 / Y 上绿 / Z 右蓝）。
function drawImu3d() {
  const canvas = $('imu3d');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  const W = canvas.width, H = canvas.height;
  const cx = W / 2, cy = H / 2;
  ctx.fillStyle = '#0a0f1e';
  ctx.fillRect(0, 0, W, H);
  ctx.textAlign = 'center';
  const q = store.imu.q;
  if (!q) {
    ctx.fillStyle = '#7e8bab';
    ctx.font = '11px sans-serif';
    ctx.fillText('等待 IMU…', cx, cy);
    return;
  }
  const {x, y, z, w} = q;
  const R = [
    [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
  ];
  const r = 42;
  const verts = [];
  for (const sx of [-1, 1]) for (const sy of [-1, 1]) for (const sz of [-1, 1]) {
    verts.push([sx * r, sy * r, sz * r]);
  }
  const proj = (v) => {
    const xw = R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2];
    const yw = R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2];
    const zw = R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2];
    return [cx + xw, cy - yw, zw];
  };
  const p = verts.map(proj);
  const edges = [[0, 1], [0, 2], [0, 4], [1, 3], [1, 5], [2, 3], [2, 6],
    [3, 7], [4, 5], [4, 6], [5, 7], [6, 7]];
  const sorted = edges
    .map(([a, b]) => ({a, b, z: (p[a][2] + p[b][2]) / 2}))
    .sort((m, n) => m.z - n.z);
  ctx.strokeStyle = '#4da3ff';
  ctx.lineWidth = 1.5;
  for (const {a, b} of sorted) {
    ctx.beginPath(); ctx.moveTo(p[a][0], p[a][1]); ctx.lineTo(p[b][0], p[b][1]); ctx.stroke();
  }
  ctx.fillStyle = '#dce4f5';
  for (const pt of p) { ctx.beginPath(); ctx.arc(pt[0], pt[1], 1.6, 0, Math.PI * 2); ctx.fill(); }
  const axes = [[[1, 0, 0], [255, 107, 107]], [[0, 1, 0], [107, 255, 122]],
    [[0, 0, 1], [107, 168, 255]]];
  const o = proj([0, 0, 0]);
  for (const [v, col] of axes) {
    const tip = proj([v[0] * r * 1.25, v[1] * r * 1.25, v[2] * r * 1.25]);
    ctx.strokeStyle = `rgb(${col.join(',')})`;
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(o[0], o[1]); ctx.lineTo(tip[0], tip[1]); ctx.stroke();
  }
}

// 人工地平仪：按 roll 旋转、pitch 平移地平线。
function drawHorizon() {
  const canvas = $('attitudeHorizon');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  const W = canvas.width, H = canvas.height;
  const cx = W / 2, cy = H / 2;
  ctx.fillStyle = '#0a0f1e';
  ctx.fillRect(0, 0, W, H);
  if (!store.imu.q) {
    ctx.fillStyle = '#7e8bab';
    ctx.font = '11px sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('等待 IMU…', cx, cy);
    return;
  }
  const roll = store.imu.roll, pitch = store.imu.pitch;
  const pitchPx = pitch / Math.PI * H * 0.9;
  const grad = ctx.createLinearGradient(0, 0, 0, H);
  grad.addColorStop(0, '#12223f');
  grad.addColorStop(1, '#22303f');
  ctx.fillStyle = grad;
  ctx.fillRect(0, 0, W, H);
  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(roll);
  ctx.strokeStyle = '#f5b94a';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(-W, -pitchPx);
  ctx.lineTo(W, -pitchPx);
  ctx.stroke();
  ctx.strokeStyle = '#7e8bab';
  ctx.lineWidth = 1;
  for (let deg = -30; deg <= 30; deg += 10) {
    if (deg === 0) continue;
    const yy = -deg / 45 * H * 0.35 - pitchPx;
    ctx.beginPath(); ctx.moveTo(-46, yy); ctx.lineTo(46, yy); ctx.stroke();
  }
  ctx.restore();
  ctx.strokeStyle = '#ff6b6b';
  ctx.lineWidth = 2.5;
  ctx.beginPath(); ctx.moveTo(cx - 32, cy); ctx.lineTo(cx + 32, cy); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(cx, cy - 16); ctx.lineTo(cx, cy + 16); ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(cx, cy - 16); ctx.lineTo(cx - 5, cy - 8);
  ctx.moveTo(cx, cy - 16); ctx.lineTo(cx + 5, cy - 8);
  ctx.stroke();
  ctx.fillStyle = '#37d67a';
  ctx.beginPath(); ctx.arc(cx, cy, 3, 0, Math.PI * 2); ctx.fill();
}

function renderImu() {
  // 首帧前 store.imu 各角都是 0，直接格式化会显示成"0.0 °"（像读到了零位）——
  // 与两个 canvas 的"等待 IMU…"保持一致，收到数据前一律 --。
  const hasData = store.imu.count > 0;
  const deg = (v) => (hasData ? fmt(rad2deg(v), 1) + ' °' : '--');
  $('imuRoll').textContent = deg(store.imu.roll);
  $('imuPitch').textContent = deg(store.imu.pitch);
  $('imuYaw').textContent = deg(store.imu.yaw);
  $('imuTemp').textContent = store.temp !== undefined ? fmt(store.temp, 1) + ' °C' : '--';
  $('imuQuat').textContent = store.imu.q
    ? `${fmt(store.imu.q.x, 3)} ${fmt(store.imu.q.y, 3)} ${fmt(store.imu.q.z, 3)} ${fmt(store.imu.q.w, 3)}`
    : '--';
  const ageMs = Date.now() - store.imu.last;
  $('imuAge').textContent = store.imu.last
    ? (ageMs < 3000 ? ageMs + ' ms' : '超时')
    : '--';
  drawImu3d();
  drawHorizon();
}

export function mount(root, _ctx) {
  // 频率与"更新年龄"都靠这个 1 s 定时器：render 只由脏帧驱动，数据一停就没有脏帧，
  // 这两处会永久冻在最后一次的值上（年龄是新鲜度指标，冻结最误导）。
  let hzAnchor = 0, hzCount = 0;
  const tick = () => {
    const el = $('imuHz'), ageEl = $('imuAge');
    if (!el || !ageEl) return;
    if (!store.conn.connected) {
      el.textContent = '-- Hz';
      hzAnchor = 0;
      return;
    }
    const ageMs = Date.now() - store.imu.last;
    ageEl.textContent = store.imu.last ? (ageMs < 3000 ? ageMs + ' ms' : '超时') : '--';
    const now = performance.now();
    if (!hzAnchor) { hzAnchor = now; hzCount = store.imu.count; return; }
    const hz = (store.imu.count - hzCount) * 1000 / (now - hzAnchor);
    hzAnchor = now; hzCount = store.imu.count;
    el.textContent = `${hz.toFixed(0)} Hz`;
  };
  const hzTimer = setInterval(tick, 1000);

  const render = (dirty) => {
    if (dirty.has(DIRTY.IMU)) renderImu();
  };
  render(new Set([DIRTY.IMU]));
  return {render, intervalMs: 200, unmount() { clearInterval(hzTimer); }};
}
