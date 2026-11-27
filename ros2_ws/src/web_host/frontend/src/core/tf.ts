// 从 URDF 提取活动关节元数据；关节反馈由 /joint_states 提供。
import { store, markDirty, DIRTY } from './store';

function tuple(str: string | null, fallback: number[]): number[] {
  if (!str) return fallback;
  const v = str.trim().split(/\s+/).map(Number);
  return v.length === 3 && v.every(Number.isFinite) ? v : fallback;
}

function childTag(node: any, name: string): any {
  for (const n of node.children || []) {
    if (n.nodeName.toLowerCase() === name) return n;
  }
  return null;
}

export function collectModelJoints(robot: any) {
  const out: any[] = [];
  for (const j of Object.values((robot && robot.joints) || {}) as any[]) {
    const node = j.urdfNode;
    if (!node || j.jointType === 'fixed' || node.getAttribute('simulation_auxiliary') === 'true') continue;
    const parent = childTag(node, 'parent');
    const child = childTag(node, 'child');
    const origin = childTag(node, 'origin');
    const axis = childTag(node, 'axis');
    const limit = childTag(node, 'limit');
    const bound = (side: string) => {
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
