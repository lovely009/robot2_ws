// BVH 转换保留全部关节帧，执行限位和传动换算由主机控制入口检查。
import { controlSessionError } from './joint_control';
import { jointLimitSyncError } from './joint_limits';

export function bvhSelection(model: string) {
  let selection: any;
  try { selection = JSON.parse(sessionStorage.getItem('robot2.motionSelection') || 'null'); }
  catch { selection = null; }
  const options = selection?.model === model ? selection : {};
  return {
    model, retargeting: 'generic', up_axis: options.up_axis || 'auto',
    scale: options.scale ?? .01,
  };
}

export async function bvhModelText(model: string, signal?: AbortSignal): Promise<string> {
  const encoded = encodeURIComponent(model);
  const response = await fetch(`/robot_sim/models/${encoded}/urdf/${encoded}.urdf`,
    { cache: 'no-store', signal });
  if (!response.ok) throw new Error(`模型 ${model} 的 URDF 加载失败`);
  return response.text();
}

export async function loadBvhMotion(
  name: string,
  model: string,
  { signal, check, progress }: { signal: AbortSignal; check: () => void; progress: (detail: string) => void },
) {
  async function request(path: string, body?: unknown) {
    const response = await fetch(path, {
      cache: 'no-store', signal,
      ...(body === undefined ? {} : {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body),
      }),
    });
    check();
    const result = await response.json();
    check();
    if (!response.ok) throw new Error(result.error || `动作转换失败：HTTP ${response.status}`);
    return result;
  }
  const selection = bvhSelection(model);
  const urdf = await bvhModelText(model, signal);
  check();
  let job = await request('/api/motion/jobs', {
    ...selection, bvh_source: name, urdf_text: urdf,
  });
  const base = `/api/motion/jobs/${encodeURIComponent(job.id)}`;
  while (job.state === 'running') {
    progress(`转换 ${name} · ${job.progress || 0}%`);
    await new Promise(resolve => setTimeout(resolve, 500));
    check();
    job = await request(base);
  }
  if (job.state === 'mapping') {
    throw new Error('该动作需要先在动作工作台确认人体与机器人映射，保存映射后可在这里播放');
  }
  if (job.state !== 'done') throw new Error(job.error || '动作转换未完成');
  return request(`${base}/trajectory`);
}

export function buildBvhPlan(motion: any, state: any, model: string, alignMs: number) {
  const error = controlSessionError(state, model);
  if (error) throw new Error(error);
  if (state.commandStatus.data.joint_trajectory_ids !== true) {
    throw new Error('主机控制程序需更新后才能接收 BVH 播放回执');
  }
  const { joint_names: sourceNames, positions, frame_time: frameTime } = motion;
  if (!Array.isArray(sourceNames) || !sourceNames.length
      || sourceNames.some(name => typeof name !== 'string' || !name)
      || new Set(sourceNames).size !== sourceNames.length
      || !Number.isFinite(frameTime) || frameTime <= 0
      || !Array.isArray(positions) || !positions.length
      || positions.some(row => !Array.isArray(row) || row.length !== sourceNames.length
        || row.some(value => !Number.isFinite(value)))) {
    throw new Error('BVH 关节轨迹格式无效');
  }
  const names: string[] = [...state.transmissionStatus.data.joint_names];
  const indices = names.map(name => {
    const index = sourceNames.indexOf(name);
    if (index < 0) throw new Error(`BVH 转换结果缺少当前控制关节 ${name}`);
    const limitError = jointLimitSyncError(state, model, name);
    if (limitError) throw new Error(limitError);
    return index;
  });
  const velocityLimits = names.map(name => {
    const limit = state.transmissionStatus.data.joint_velocity_limits?.[name];
    if (!Number.isFinite(limit) || limit <= 0) {
      throw new Error(`${name} 缺少主机速度限位回读，请更新主机控制程序`);
    }
    return limit as number;
  });
  const groups = state.modelTransmissions.hardware_ready === false
    ? state.transmissionStatus.data.control_groups : state.modelTransmissions.groups;
  const ids: number[] = [...new Set<number>(groups.filter((group: any) => group.joints.some((name: string) => names.includes(name)))
    .flatMap((group: any) => group.motors.map((motor: any) => Number(motor.id))))];
  const times = positions.map((_, index) => alignMs + Math.round(index * frameTime * 1000));
  if (times.at(-1)! > 0xffffffff || times.some((t, i) => i > 0 && t <= times[i - 1]!)) {
    throw new Error('BVH 采样时间须按毫秒严格递增且不超过 uint32');
  }
  const points = positions.map((row: number[], index: number) => ({
    positions: indices.map(column => row[column]!),
    velocities: names.map(() => 0),
    time_from_start: { sec: Math.floor(times[index]! / 1000), nanosec: times[index]! % 1000 * 1e6 },
  }));
  for (let i = 1; i < points.length - 1; i++) {
    const dt0 = (times[i]! - times[i - 1]!) / 1000;
    const dt1 = (times[i + 1]! - times[i]!) / 1000;
    const w1 = 2 * dt1 + dt0, w2 = dt1 + 2 * dt0;
    for (let joint = 0; joint < names.length; joint++) {
      const d0 = (points[i]!.positions[joint]! - points[i - 1]!.positions[joint]!) / dt0;
      const d1 = (points[i + 1]!.positions[joint]! - points[i]!.positions[joint]!) / dt1;
      if (!Number.isFinite(d0) || !Number.isFinite(d1)) {
        throw new Error('BVH 差分速度必须为有限值');
      }
      if (d0 * d1 > 0) {
        const limit = 3 * Math.min(Math.abs(d0), Math.abs(d1));
        points[i]!.velocities[joint] = Math.max(-limit, Math.min(limit, (w1 + w2) / (w1 / d0 + w2 / d1)));
      }
    }
  }
  let requiredScale = 1;
  for (let i = 1; i < points.length; i++) {
    const before = points[i - 1]!, after = points[i]!;
    const dt = (times[i]! - times[i - 1]!) / 1000;
    for (let joint = 0; joint < names.length; joint++) {
      const p = before.positions[joint]!, q = after.positions[joint]!;
      const v0 = before.velocities[joint]!, v1 = after.velocities[joint]!;
      const a = 2 * p - 2 * q + dt * (v0 + v1);
      const b = -3 * p + 3 * q - dt * (2 * v0 + v1);
      let peak = Math.max(Math.abs(v0), Math.abs(v1));
      const s = a === 0 ? -1 : -b / (3 * a);
      if (s > 0 && s < 1) {
        peak = Math.max(peak, Math.abs((3 * a * s * s + 2 * b * s + dt * v0) / dt));
      }
      if (!Number.isFinite(peak)) throw new Error('BVH 插值速度必须为有限值');
      requiredScale = Math.max(requiredScale, peak / velocityLimits[joint]!);
    }
  }
  const scalePermille = requiredScale > 1 ? Math.ceil(requiredScale * (1 + 1e-6) * 1000) : 1000;
  const timeScale = scalePermille / 1000;
  const nanoseconds = times.map(t => alignMs * 1e6 + (t - alignMs) * scalePermille * 1000);
  if (nanoseconds.some(t => !Number.isSafeInteger(t) || t > 0xffffffff * 1e6)) {
    throw new Error('BVH 降速后总时长超过 uint32 毫秒范围');
  }
  for (let i = 0; i < points.length; i++) {
    points[i]!.velocities = points[i]!.velocities.map(velocity => velocity / timeScale);
    points[i]!.time_from_start = {
      sec: Math.floor(nanoseconds[i]! / 1e9), nanosec: nanoseconds[i]! % 1e9,
    };
  }
  return { names, ids, points, timeScale, durationMs: Math.floor(nanoseconds.at(-1)! / 1e6) };
}
