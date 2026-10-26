// TF 侧的关节：模型 URDF 的关节表（父子 link / 类型 / 轴 / 原点）× /tf 的帧变换。
// 「关节」与「电机」不是一类东西：本模块只认 URDF 与 TF，不读固件的电机流。
// store.tf 由 core/ros.js 的订阅写入，store.modelJoints 由臂视图加载 URDF 后装入。
import { store, markDirty, DIRTY } from './store.js';

function tuple(str, fallback) {
  if (!str) return fallback;
  const v = str.trim().split(/\s+/).map(Number);
  return v.length === 3 && v.every(Number.isFinite) ? v : fallback;
}

function childTag(node, name) {
  for (const n of node.children || []) {
    if (n.nodeName.toLowerCase() === name) return n;
  }
  return null;
}

// 从臂视图加载出的 URDF 机器人对象取关节元数据（原点/轴在反解里要用，不能只留名字）。
// 固定关节不是可动「关节」，不列。
export function collectModelJoints(robot) {
  const out = [];
  for (const j of Object.values((robot && robot.joints) || {})) {
    const node = j.urdfNode;
    if (!node || j.jointType === 'fixed') continue;
    const parent = childTag(node, 'parent');
    const child = childTag(node, 'child');
    const origin = childTag(node, 'origin');
    const axis = childTag(node, 'axis');
    const limit = childTag(node, 'limit');
    const bound = (side) => {
      const raw = j.jointType === 'continuous' ? null : limit?.getAttribute(side);
      const value = raw === null || raw === undefined || raw.trim() === '' ? NaN : Number(raw);
      return Number.isFinite(value) ? value : null;
    };
    out.push({
      name: j.name,
      type: j.jointType,
      parent: parent ? parent.getAttribute('link') : '',
      child: child ? child.getAttribute('link') : '',
      xyz: tuple(origin && origin.getAttribute('xyz'), [0, 0, 0]),
      rpy: tuple(origin && origin.getAttribute('rpy'), [0, 0, 0]),
      axis: tuple(axis && axis.getAttribute('xyz'), [1, 0, 0]),
      lower: bound('lower'),
      upper: bound('upper'),
      velocity: Number(limit?.getAttribute('velocity')) > 0
        ? Number(limit.getAttribute('velocity')) : null,
    });
  }
  store.modelJoints = out;
  markDirty(DIRTY.TF);
  return out;
}

// URDF rpy = Rz(yaw)·Ry(pitch)·Rx(roll)
function rpyMatrix([r, p, y]) {
  const cr = Math.cos(r), sr = Math.sin(r);
  const cp = Math.cos(p), sp = Math.sin(p);
  const cy = Math.cos(y), sy = Math.sin(y);
  return [
    [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
    [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
    [-sp, cp * sr, cp * cr],
  ];
}

function quatMatrix(q) {
  const {x, y, z, w} = q;
  const xx = x * x, yy = y * y, zz = z * z;
  return [
    [1 - 2 * (yy + zz), 2 * (x * y - w * z), 2 * (x * z + w * y)],
    [2 * (x * y + w * z), 1 - 2 * (xx + zz), 2 * (y * z - w * x)],
    [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (xx + yy)],
  ];
}

function unit([x, y, z]) {
  const n = Math.hypot(x, y, z) || 1;
  return [x / n, y / n, z / n];
}

// 绕轴旋转角：R - Rᵀ = 2sinθ·[a]ₓ，迹 = 1 + 2cosθ。
function twistAngle(D, axis) {
  const [ax, ay, az] = unit(axis);
  const s = ((D[2][1] - D[1][2]) * ax + (D[0][2] - D[2][0]) * ay
    + (D[1][0] - D[0][1]) * az) / 2;
  const c = (D[0][0] + D[1][1] + D[2][2] - 1) / 2;
  return Math.atan2(s, c);
}

// T(parent→child) = O·R(axis,θ)，故 D = Oᵀ·Q 应恰是绕轴的旋转/平移。
function jointValue(joint, tf) {
  const q = tf.quat, t = tf.xyz;
  if (!q || !t) return NaN;
  const O = rpyMatrix(joint.rpy);
  if (joint.type === 'prismatic') {
    // 轴在关节系，先转进父 link 系；位移 = 平移在轴上的投影。
    const [ax, ay, az] = unit(joint.axis);
    const pa = [0, 1, 2].map((i) => O[i][0] * ax + O[i][1] * ay + O[i][2] * az);
    return (t.x - joint.xyz[0]) * pa[0] + (t.y - joint.xyz[1]) * pa[1]
      + (t.z - joint.xyz[2]) * pa[2];
  }
  const Q = quatMatrix(q);
  const D = [0, 1, 2].map((i) => [0, 1, 2].map(
    (j) => O[0][i] * Q[0][j] + O[1][i] * Q[1][j] + O[2][i] * Q[2][j]));
  const v = twistAngle(D, joint.axis);
  return Number.isFinite(v) ? v : NaN;
}

// 关节读数：帧对（child，必要时 parent）对不上 → null（总线上没有这条 TF）。
export function jointReading(joint, now = Date.now()) {
  const tf = store.tf.get(joint.child);
  if (!tf || (tf.parent && joint.parent && tf.parent !== joint.parent)) return null;
  return {value: jointValue(joint, tf), ageMs: now - tf.at, dynamic: !!tf.dynamic};
}
