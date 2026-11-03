// 主机回读是生效范围；浏览器仅记住已经确认的设置，不自动恢复旧草稿。
const keyOf = (model) => `web_host.joint_limits.v1.${model}`;

function recordsOf(model, storage) {
  const raw = storage?.getItem(keyOf(model));
  if (raw === null || raw === undefined) return {};
  const records = JSON.parse(raw);
  if (!records || typeof records !== 'object' || Array.isArray(records)) {
    throw new Error('关节软限位存储格式无效');
  }
  return records;
}

export function getJointLimits(state, model, joint) {
  const runtime = state.transmissionStatus?.data;
  const fallback = {lower: Number.isFinite(joint.lower) ? joint.lower : null,
    upper: Number.isFinite(joint.upper) ? joint.upper : null, error: ''};
  if (runtime?.model !== model || !Array.isArray(runtime.joint_names) || !runtime.joint_names.includes(joint.name)) return fallback;
  const bounds = runtime.joint_limits?.[joint.name];
  if (!bounds || !['lower', 'upper'].every(side => bounds[side] === null || Number.isFinite(bounds[side]))
      || (bounds.lower !== null && bounds.upper !== null && bounds.lower >= bounds.upper)) {
    return {lower: null, upper: null, error: '主机未提供有效关节限位回读'};
  }
  return {lower: bounds.lower, upper: bounds.upper, error: ''};
}

export function saveJointLimits(model, joint, bounds, storage = globalThis.localStorage) {
  if (!storage) throw new Error('浏览器本地存储不可用');
  let records;
  try { records = recordsOf(model, storage); } catch (_) { records = {}; }
  storage.setItem(keyOf(model), JSON.stringify({
    ...records, [joint.name]: {lower: bounds.lower, upper: bounds.upper},
  }));
}

export function jointLimitsParameter(state, model, change) {
  const runtime = state.transmissionStatus?.data;
  if (runtime?.model !== model || !Array.isArray(runtime.joint_names)) {
    throw new Error('主机关节模型未就绪');
  }
  if (!change || !runtime.joint_names.includes(change.name)) throw new Error('该关节未接入当前控制会话，不能提交限位');
  if (!['lower', 'upper'].includes(change.side) || !Number.isFinite(change.value)) {
    throw new Error('关节软限位必须是有限数值');
  }
  const joint_limits = {};
  for (const name of runtime.joint_names) {
    const joint = state.modelJoints?.find(j => j.name === name);
    if (!joint) throw new Error(`关节 ${name} 的模型限位尚未加载`);
    const bounds = getJointLimits(state, model, joint);
    if (bounds.error) throw new Error(bounds.error);
    if (name === change.name) {
      bounds[change.side] = change.value;
      if (![bounds.lower, bounds.upper].every(Number.isFinite) || bounds.lower >= bounds.upper) {
        throw new Error('关节软限位必须是有限数值，且下限小于上限');
      }
    }
    if (bounds.lower !== null && bounds.upper !== null) {
      joint_limits[name] = {lower: bounds.lower, upper: bounds.upper};
    }
  }
  return {name: 'joint_limits_json', value: {type: 4,
    string_value: JSON.stringify({model, joint_limits})}};
}

export function jointLimitSyncError(state, model, name) {
  const joint = state.modelJoints?.find(j => j.name === name);
  if (!joint) return `关节 ${name} 的模型限位尚未加载`;
  return getJointLimits(state, model, joint).error;
}

export function jointTargetLimitError(state, model, name, target) {
  const joint = state.modelJoints?.find(j => j.name === name);
  if (!joint) return '关节软限位尚未加载';
  const bounds = getJointLimits(state, model, joint);
  if (bounds.error) return bounds.error;
  if (bounds.lower === null || bounds.upper === null) return '请先设置该关节的左右软限位';
  if (target < bounds.lower || target > bounds.upper) {
    return `${name} 目标超出关节软限位 [${bounds.lower}, ${bounds.upper}]`;
  }
  return '';
}
