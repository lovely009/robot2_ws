// 浏览器关节软限位：按模型与关节持久化，坐标与 URDF 一致。
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

export function getJointLimits(model, joint, storage) {
  try {
    storage ??= globalThis.localStorage;
    const records = recordsOf(model, storage);
    const saved = Object.hasOwn(records, joint.name) ? records[joint.name] : {};
    if (!saved || typeof saved !== 'object' || Array.isArray(saved)) {
      throw new Error('关节软限位记录无效');
    }
    const bounds = {};
    for (const side of ['lower', 'upper']) {
      if (Object.hasOwn(saved, side) && !Number.isFinite(saved[side])) {
        throw new Error('关节软限位必须是有限数值');
      }
      bounds[side] = Object.hasOwn(saved, side) ? saved[side]
        : Number.isFinite(joint[side]) ? joint[side] : null;
    }
    if (bounds.lower !== null && bounds.upper !== null && bounds.lower >= bounds.upper) {
      throw new Error('关节软限位下限必须小于上限');
    }
    return {...bounds, error: ''};
  } catch (error) {
    return {lower: null, upper: null, error: `关节软限位读取失败：${error.message}`};
  }
}

export function saveJointLimit(model, joint, side, value, storage = globalThis.localStorage) {
  if (!['lower', 'upper'].includes(side) || !Number.isFinite(value)) {
    throw new Error('关节软限位必须是有限数值');
  }
  const current = getJointLimits(model, joint, storage);
  if (current.error) throw new Error(current.error);
  const next = {...current, [side]: value};
  if (next.lower !== null && next.upper !== null && next.lower >= next.upper) {
    throw new Error('关节软限位下限必须小于上限');
  }
  if (!storage) throw new Error('浏览器本地存储不可用');
  const records = recordsOf(model, storage);
  const previous = Object.hasOwn(records, joint.name) ? records[joint.name] : {};
  storage.setItem(keyOf(model), JSON.stringify({
    ...records, [joint.name]: {...previous, [side]: value},
  }));
  return getJointLimits(model, joint, storage);
}

export function jointTargetLimitError(state, model, name, target) {
  const joint = state.modelJoints?.find(j => j.name === name);
  if (!joint) return '关节软限位尚未加载';
  const bounds = getJointLimits(model, joint);
  if (bounds.error) return bounds.error;
  if (bounds.lower === null || bounds.upper === null) return '请先设置该关节的左右软限位';
  if (target < bounds.lower || target > bounds.upper) {
    return `${name} 目标超出关节软限位 [${bounds.lower}, ${bounds.upper}]`;
  }
  return '';
}
