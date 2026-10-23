// 设置、持久化与命令均使用 rad；旧版本度数配置保留实际行程换算。
const key = (model) => `web_host.host_limits.v2.${model}`;
function validate(rows) {
  if (!Array.isArray(rows)) throw new Error('主机限位配置必须是列表');
  const seen = new Set();
  for (const row of rows) {
    if (!Number.isInteger(row.id) || row.id < 1 || row.id > 65535 || seen.has(row.id)) {
      throw new Error('电机 ID 须为 1～65535 且不能重复');
    }
    seen.add(row.id);
    if (typeof row.enabled !== 'boolean' || typeof row.min !== 'number'
        || typeof row.max !== 'number' || !Number.isFinite(row.min)
        || !Number.isFinite(row.max) || row.min >= row.max) {
      throw new Error(`m${row.id}：上下限须为有限角度，且最小值小于最大值`);
    }
  }
  return rows;
}
export function readLimits(model) {
  const current = localStorage.getItem(key(model));
  if (current !== null) return validate(JSON.parse(current));
  const old = localStorage.getItem(`web_host.host_limits.v1.${model}`);
  if (old === null) return [];
  const rows = validate(JSON.parse(old)).map(row => ({
    ...row, min: row.min * Math.PI / 180, max: row.max * Math.PI / 180,
  }));
  saveLimits(model, rows);
  return rows;
}
export function saveLimits(model, rows) {
  localStorage.setItem(key(model), JSON.stringify(validate(rows)));
}
export function limitsError(targets, model) {
  let rows;
  try { rows = readLimits(model); }
  catch (_error) { return '主机限位配置无法读取，请在主机限位页检查并重新保存'; }
  const active = new Map(rows.filter(r => r.enabled).map(r => [r.id, r]));
  for (const target of targets) {
    const row = active.get(Number(target.motor_id));
    if (!row) continue;
    const position = Number(target.position);
    if (!Number.isFinite(position) || position < row.min - 1e-8 || position > row.max + 1e-8) {
      return `主机限位：m${row.id} 目标 ${position.toFixed(6)} rad 超出 [${row.min}, ${row.max}] rad，已拒绝下发`;
    }
  }
  return '';
}
