// MotorTrajectory 动作文件经低频客户端转换成 JTC action，不再上传 MCU。
import { store, MODEL, JOINT_OF_MOTOR, markDirty, DIRTY, beginTrajectory } from './store';
import { LIFECYCLE, TRAJ_MAX_PLAN_POINTS, TRAJ_MAX_MOTORS, TRAJ_STATE_TIMEOUT_PAD_MS } from './format';
import { pub, publishTrajectory, isConnected, submitPolicy, getConnectionGeneration } from './ros';
import { controlGroups, controlSessionError, rawCouplingError } from './joint_control';
import { controlTargetsError, describeMotorState, policyRequestState, applyPolicySubmission, confirmPolicyIntent, stopPolicyIntents } from './motor_state';
import { isUnprotected } from './control_policy';
import { buildBvhPlan, loadBvhMotion } from './bvh_playback';

function playbackReadinessError(ids: number[]): string {
  const blocked: string[] = [];
  for (const id of [...new Set(ids)].sort((a, b) => a - b)) {
    const row = store.motors.get(id);
    if (!row) {
      blocked.push(`m${id}(未发现)`);
      continue;
    }
    if (store.controlStates?.seen) {
      const view = describeMotorState(store, id);
      const control = store.controlStates.byId.get(id);
      if (!view.online || view.connection !== '在线' || view.fault
          || ![2, 3].includes(control?.control_lifecycle)) blocked.push(`m${id}(${view.connection} · ${view.lifecycle} · ${view.health})`);
      continue;
    }
    const st = row.status || {}, topo = row.topo || {};
    const online = st.online !== undefined ? st.online : topo.online;
    const lc = st.lifecycle !== undefined ? st.lifecycle : topo.lifecycle;
    const fault = Number(st.unified_fault_code || 0);
    if (online !== true) blocked.push(`m${id}(离线)`);
    else if (fault) blocked.push(`m${id}(故障${fault})`);
    else if (lc !== 3 && lc !== 4) {
      const name = LIFECYCLE[lc] !== undefined ? LIFECYCLE[lc] : '未就绪';
      blocked.push(`m${id}(${name})`);
    }
  }
  return blocked.length
    ? `电机未就绪：${blocked.join('、')}；请先连接/启动仿真，等目标电机到 `
      + `READY_DISABLED（或 ACTIVE）后重试`
    : '';
}

const ALIGN_MS = 2500;
const CANCEL_SETTLE_MS = 800;
const RETURN_ZERO_SPEED_RAD_S = 0.4;
const RETURN_ZERO_MIN_MS = 2000;
const RETURN_ZERO_MAX_MS = 10000;
const ZERO_EPS_RAD = 0.005;

export function computeReturnZeroDurationMs(positions: number[]): number {
  let maxAbs = 0;
  for (const v of positions) {
    if (Number.isFinite(v)) maxAbs = Math.max(maxAbs, Math.abs(v));
  }
  if (maxAbs < ZERO_EPS_RAD) return 1000;
  return Math.min(RETURN_ZERO_MAX_MS, Math.max(RETURN_ZERO_MIN_MS, Math.ceil((maxAbs / RETURN_ZERO_SPEED_RAD_S) * 1000)));
}

function buildReturnZeroPlan(names: string[], ids: number[], fromPositions: number[]) {
  const durationMs = computeReturnZeroDurationMs(fromPositions);
  return {
    ids,
    names,
    points: [{
      positions: names.map(() => 0),
      velocities: names.map(() => 0),
      time_from_start: { sec: Math.floor(durationMs / 1000), nanosec: (durationMs % 1000) * 1e6 },
    }],
    durationMs,
  };
}

const player = {
  phase: 'idle',
  detail: '',
  error: '',
  name: '',
  ids: [] as number[],
  model: '',
  modelMetadata: null as any,
  points: [] as any[],
  raw: false,
  round: 0,
  loop: false,
  enable: true,
  kp: 30,
  kd: 1,
  durationMs: 0,
  sessionId: Date.now(),
  tidSeq: Math.floor(Math.random() * 0xfffffffe) + 1,
  cancelRequested: false,
  planStarted: false,
  connectionGeneration: 0,
  jointPlan: null as any,
  loading: null as AbortController | null,
};

function assertPlaybackSession() {
  if (player.cancelRequested) throw new Error('已停止');
  if (!isConnected() || player.connectionGeneration !== getConnectionGeneration()) {
    throw new Error('连接已更换或中断，请重新确认后播放');
  }
  if (MODEL !== player.model || store.modelTransmissions !== player.modelMetadata) {
    throw new Error('播放期间模型或传动配置已更换，请重新确认');
  }
  if (player.jointPlan && (store.commandStatus.data?.joint_trajectory_ids === false
      || !store.transmissionStatus.data?.joint_names?.length
      || player.jointPlan.names.some((name: string) => !store.transmissionStatus.data.joint_names.includes(name)))) {
    throw new Error('播放期间主机控制关节已更换，请重新确认');
  }
  const error = controlSessionError(store, player.model)
    || (!player.jointPlan && rawCouplingError(store, player.model, player.ids));
  if (error) throw new Error(error);
}

function sleep(ms: number) { return new Promise<void>((resolve) => setTimeout(resolve, ms)); }

function announce() { markDirty(DIRTY.TRAJ); }

function cancelTrajectory(): boolean {
  return pub('trajectory', {
    header: {},
    joint_names: [],
    points: [],
  });
}

export function loadSeries(document_: any) {
  if (!document_ || typeof document_ !== 'object') throw new Error('不是 JSON 对象');
  if (document_.interpolation !== undefined && !['monotone', 'linear'].includes(document_.interpolation)) {
    throw new Error('动作插值模式只能为 monotone 或 linear');
  }
  const ids: number[] = [...new Set<number>((document_.target_ids || []).map(Number))];
  if (!ids.length) throw new Error('动作缺 target_ids');
  const buckets = new Map<number, Record<number, [number, number, number]>>();
  for (const point of document_.points || []) {
    const id = Number(point.motor_id);
    if (!ids.includes(id)) continue;
    const t = Math.round(Number(point.time_from_start_ms));
    if (!Number.isFinite(t)) throw new Error('路点缺 time_from_start_ms');
    const effort = point.effort !== undefined ? point.effort : (point.torque || 0);
    if (!buckets.has(t)) buckets.set(t, {});
    buckets.get(t)![id] = [
      Number(point.position), Number(point.velocity || 0), Number(effort),
    ];
  }
  const zero = Object.fromEntries(ids.map((id) => [id, [0, 0, 0]])) as Record<number, [number, number, number]>;
  const steps: { t: number; vals: Record<number, [number, number, number]> }[] = [];
  for (const t of [...buckets.keys()].sort((a, b) => a - b)) {
    const vals = { ...(steps.length ? steps[steps.length - 1]!.vals : zero) };
    Object.assign(vals, buckets.get(t));
    steps.push({ t, vals });
  }
  if (!steps.length) throw new Error('动作没有路点');
  return { ids, steps, interpolation: (document_.interpolation ?? 'monotone') as string };
}

function makePlan(steps: { t: number; vals: Record<number, [number, number, number]> }[], ids: number[]) {
  if (!Number.isInteger(ids.length) || ids.length < 1 || ids.length > TRAJ_MAX_MOTORS) {
    throw new Error(`动作电机数量须在 1～${TRAJ_MAX_MOTORS}`);
  }
  if (steps.length * ids.length > TRAJ_MAX_PLAN_POINTS) {
    throw new Error(`动作最多 ${TRAJ_MAX_PLAN_POINTS} 个电机路点，请缩减采样后重试`);
  }
  const points = steps.flatMap((step, index) => ids.map(id => {
    const [position, velocity, effort] = step.vals[id]!;
    return {
      time_from_start_ms: ALIGN_MS + step.t - steps[0]!.t, motor_id: id,
      position, velocity: index === 0 ? 0 : velocity, effort: index === 0 ? 0 : effort,
    };
  }));
  if (!points.length || points.some(point =>
      !Number.isInteger(point.time_from_start_ms) || point.time_from_start_ms <= 0
      || point.time_from_start_ms > 0xFFFFFFFF
      || ![point.position, point.velocity, point.effort].every(Number.isFinite))) {
    throw new Error('轨迹路点须为有限值，时间须为正毫秒且不超过 uint32');
  }
  return points;
}

function motorJointMap(state: any): Map<number, string> {
  const map = new Map<number, string>();
  for (const group of controlGroups(state) || []) {
    if (group?.motors?.length === 1 && group?.joints?.length === 1) {
      map.set(Number(group.motors[0].id), group.joints[0]);
    }
  }
  const direct = state.modelTransmissions?.direct_mapping;
  if (direct && typeof direct === 'object') {
    for (const [id, name] of Object.entries(direct)) {
      if (!map.has(Number(id))) map.set(Number(id), String(name));
    }
  }
  for (const [id, name] of Object.entries(JOINT_OF_MOTOR)) {
    if (!map.has(Number(id))) map.set(Number(id), name);
  }
  return map;
}

function buildJsonJointPlan(series: ReturnType<typeof loadSeries>, state: any) {
  const planPoints = makePlan(series.steps, series.ids);
  const jointMap = motorJointMap(state);
  const names = series.ids.map(id => jointMap.get(id));
  if (names.some(name => !name)) throw new Error('缺少电机到关节的映射');
  const t0 = series.steps[0]!.t;
  const points = series.steps.map((step, index) => {
    const ms = ALIGN_MS + step.t - t0;
    return {
      positions: series.ids.map(id => step.vals[id]![0]),
      velocities: series.interpolation === 'linear'
        ? []
        : series.ids.map(id => (index === 0 ? 0 : step.vals[id]![1])),
      time_from_start: { sec: Math.floor(ms / 1000), nanosec: (ms % 1000) * 1e6 },
    };
  });
  return {
    ids: series.ids,
    names: names as string[],
    points,
    durationMs: planPoints[planPoints.length - 1]!.time_from_start_ms,
  };
}

async function sendPlan(): Promise<number> {
  assertPlaybackSession();
  const authority = controlTargetsError(store, player.ids);
  if (authority) throw new Error(authority);
  player.tidSeq = player.tidSeq % 0xffffffff + 1;
  const tid = player.tidSeq;
  const syncTime = store.controlStates.sourceTimeNs;
  if (!syncTime) throw new Error('尚未收到 MCU 同步源时间，拒绝发送动作');
  if (player.jointPlan) {
    player.planStarted = true;
    beginTrajectory(store.traj.activeTrajectoryId, true);
    if (!await publishTrajectory({
      header: {
        stamp: { sec: Math.floor(syncTime / 1e9), nanosec: Math.round(syncTime % 1e9) },
        frame_id: `${player.raw ? 'workbench-trajectory-linear' : 'workbench-trajectory'}/${tid}`,
      },
      joint_names: player.jointPlan.names, points: player.jointPlan.points,
    })) throw new Error('关节轨迹发送失败，请检查连接与控制模式');
    beginTrajectory(tid);
    return tid;
  }
  throw new Error('缺少关节轨迹数据，请通过关节规划路径发送');
}

export function trajectoryWindowResult(state: any, tid: number, durationMs: number): 'waiting' | 'done' | 'canceled' {
  if (!state || state.trajectory_id !== tid) return 'waiting';
  if (!state.active && state.completed && state.elapsed_ms >= durationMs) return 'done';
  return state.active ? 'waiting' : 'canceled';
}

async function waitPlan(tid: number) {
  const deadline = Date.now() + player.durationMs + TRAJ_STATE_TIMEOUT_PAD_MS;
  while (Date.now() < deadline) {
    if (player.cancelRequested) return 'stopped';
    assertPlaybackSession();
    const state = store.traj.state;
    const result = trajectoryWindowResult(state, tid, player.durationMs);
    if (result !== 'waiting') return result;
    await sleep(50);
  }
  return 'timeout';
}

async function armMotors() {
  assertPlaybackSession();
  const ticket = await submitPolicy(player.ids.map(motor_id => ({
    motor_id, enabled: true, kp: player.kp, kd: player.kd,
    confirm_unprotected: isUnprotected(motor_id),
  })));
  assertPlaybackSession();
  if (player.cancelRequested) throw new Error('已停止');
  applyPolicySubmission(store, player.ids.map(motor_id => ({ motor_id, enabled: true, kp: player.kp, kd: player.kd })), ticket);
  markDirty(DIRTY.POLICY);
  const deadline = Date.now() + 3000;
  while (true) {
    assertPlaybackSession();
    if (player.cancelRequested) throw new Error('已停止');
    const outcome = policyRequestState(store, ticket, player.ids, true);
    if (outcome.error) throw new Error(outcome.error);
    if (outcome.confirmed) {
      confirmPolicyIntent(store, player.ids, ticket);
      return;
    }
    if (Date.now() >= deadline) throw new Error('MCU 未确认本次使能策略，拒绝播放');
    await sleep(50);
  }
}

export async function playAction(
  name: string,
  options: { kp: number; kd: number; loop: boolean; enable: boolean; returnToZero?: boolean },
): Promise<void> {
  if (player.phase === 'running') return;
  if (!isConnected()) {
    player.phase = 'error';
    player.error = '未连接 rosbridge';
    announce();
    return;
  }
  player.phase = 'running';
  player.error = '';
  player.cancelRequested = false;
  player.connectionGeneration = getConnectionGeneration();
  player.model = MODEL;
  player.modelMetadata = store.modelTransmissions;
  player.ids = [];
  player.name = name;
  player.kp = options.kp;
  player.kd = options.kd;
  player.loop = options.loop;
  player.enable = options.enable;
  player.planStarted = false;
  player.jointPlan = null;
  player.loading = new AbortController();
  try {
    player.detail = `读取 ${name}…`;
    announce();
    let ids: number[];
    if (/\.bvh$/i.test(name)) {
      assertPlaybackSession();
      const motion = await loadBvhMotion(name, player.model, {
        signal: player.loading.signal, check: assertPlaybackSession,
        progress: detail => { player.detail = detail; announce(); },
      });
      player.jointPlan = buildBvhPlan(motion, store, player.model, ALIGN_MS);
      ids = player.jointPlan.ids;
      player.raw = false;
      player.durationMs = player.jointPlan.durationMs;
    } else {
      const response = await fetch(
        `/api/actions/${encodeURIComponent(name)}`, { cache: 'no-store', signal: player.loading.signal });
      assertPlaybackSession();
      if (!response.ok) throw new Error(`读取动作失败：HTTP ${response.status}`);
      const document_ = await response.json();
      assertPlaybackSession();
      const series = loadSeries(document_);
      ids = series.ids;
      player.raw = series.interpolation === 'linear';
      const coupled = rawCouplingError(store, player.model, ids);
      if (coupled) throw new Error(coupled);
      player.jointPlan = buildJsonJointPlan(series, store);
      player.durationMs = player.jointPlan.durationMs;
    }
    const blocked = playbackReadinessError(ids);
    if (blocked) throw new Error(blocked);
    const unprotected = ids.filter(isUnprotected);
    if (unprotected.length && !window.confirm(
        `m${unprotected.join('、m')} 的设备侧限位保护当前是关闭的：\n\n` +
        '这些关节没有任何限位在执行（含看门狗自锁）。确认继续播放吗？')) {
      throw new Error('已取消（限位保护未确认）');
    }
    player.ids = ids;

    player.detail = '取消旧轨迹…';
    announce();
    await publishTrajectory({ header: {}, joint_names: [], points: [] });
    await sleep(CANCEL_SETTLE_MS);
    assertPlaybackSession();
    if (player.cancelRequested) throw new Error('已停止');

    if (player.enable) {
      player.detail = `武装 kp=${player.kp} kd=${player.kd}…`;
      announce();
      await armMotors();
      if (player.cancelRequested) throw new Error('已停止');
    } else {
      player.detail = '跳过武装（未勾选「使能」）';
      announce();
    }

    for (player.round = 0; ; player.round++) {
      const tid = await sendPlan();
      player.detail = `${player.loop ? `第${player.round + 1}轮 · ` : ''}`
        + `JTC ${player.raw ? '原始线性' : '单调插值'}播放（含起点引导）· ${player.durationMs}ms`
        + (player.jointPlan?.timeScale > 1
          ? ` · 按速度限位降至 ${(100 / player.jointPlan.timeScale).toFixed(1)}%` : '');
      announce();
      const verdict = await waitPlan(tid);
      if (verdict === 'stopped') throw new Error('已停止');
      if (verdict === 'canceled') throw new Error(`JTC 未成功（取消/拒绝/执行失败）：${store.commandStatus.data?.reject || store.commandStatus.data?.reason || '请核对控制状态'}`);
      if (verdict !== 'done') throw new Error(`无 JTC 结果回执：${store.commandStatus.data?.reject || '请核对控制客户端状态'}`);
      if (!player.loop) break;
    }
    if (options.returnToZero && player.jointPlan && !player.cancelRequested) {
      const lastPos: number[] = player.jointPlan.points.at(-1)?.positions || [];
      if (lastPos.some((v) => Number.isFinite(v) && Math.abs(v) > 1e-4)) {
        const zeroPlan = buildReturnZeroPlan(player.jointPlan.names, player.jointPlan.ids, lastPos);
        player.raw = false;
        player.jointPlan = zeroPlan;
        player.durationMs = zeroPlan.durationMs;
        const ztid = await sendPlan();
        player.detail = `平滑回初始位并保持 · ${zeroPlan.durationMs}ms`;
        announce();
        const zverdict = await waitPlan(ztid);
        if (zverdict === 'stopped') throw new Error('已停止');
        if (zverdict === 'canceled') throw new Error(`回初始位未成功（取消/拒绝/执行失败）：${store.commandStatus.data?.reject || store.commandStatus.data?.reason || '请核对控制状态'}`);
        if (zverdict !== 'done') throw new Error(`回初始位无 JTC 结果回执：${store.commandStatus.data?.reject || '请核对控制客户端状态'}`);
      }
      player.phase = 'done';
      player.detail = '播放完成（已回初始位并保持）';
    } else {
      player.phase = 'done';
      player.detail = '播放完成（JTC 成功）';
    }
    announce();
  } catch (error: any) {
    player.phase = 'error';
    player.error = player.cancelRequested ? '已停止' : error.message;
    if (player.planStarted && !player.cancelRequested && isConnected()
        && player.connectionGeneration === getConnectionGeneration()) {
      let sent = false;
      try { sent = await publishTrajectory({ header: {}, joint_names: [], points: [] }); } catch { /* ignore */ }
      if (!sent) player.error += '；取消未发送，运动停止未确认，请使用全部软停';
    }
    announce();
  } finally {
    player.loading = null;
  }
}

export async function returnToZero(
  options: { kp: number; kd: number; enable: boolean },
): Promise<void> {
  if (player.phase === 'running') return;
  if (!isConnected()) {
    player.phase = 'error';
    player.error = '未连接 rosbridge';
    announce();
    return;
  }
  player.phase = 'running';
  player.error = '';
  player.cancelRequested = false;
  player.connectionGeneration = getConnectionGeneration();
  player.model = MODEL;
  player.modelMetadata = store.modelTransmissions;
  player.ids = [];
  player.name = '回初始位';
  player.kp = options.kp;
  player.kd = options.kd;
  player.loop = false;
  player.raw = false;
  player.enable = options.enable;
  player.planStarted = false;
  player.jointPlan = null;
  player.loading = new AbortController();
  try {
    const sessionErr = controlSessionError(store, player.model);
    if (sessionErr) throw new Error(sessionErr);
    const controlledNames: string[] = Array.isArray(store.transmissionStatus.data?.joint_names)
      ? [...store.transmissionStatus.data.joint_names]
      : [];
    if (!controlledNames.length) throw new Error('尚未收到主机控制关节列表，无法回初始位');
    const nameSet = new Set(controlledNames);
    const ids = [...new Set<number>(
      (controlGroups(store) || [])
        .filter((group: any) => (group?.joints || []).some((n: string) => nameSet.has(n)))
        .flatMap((group: any) => (group?.motors || []).map((m: any) => Number(m.id)))
        .filter((id: number) => Number.isInteger(id) && id > 0),
    )];
    if (!ids.length) throw new Error('缺少可控关节到电机的映射');
    const blocked = playbackReadinessError(ids);
    if (blocked) throw new Error(blocked);
    const unprotected = ids.filter(isUnprotected);
    if (unprotected.length && !window.confirm(
        `m${unprotected.join('、m')} 的设备侧限位保护当前是关闭的：\n\n` +
        '这些关节没有任何限位在执行（含看门狗自锁）。确认继续回初始位吗？')) {
      throw new Error('已取消（限位保护未确认）');
    }
    const currentPositions = controlledNames.map((n) => Number(store.jointStates?.byName?.[n]?.position ?? 0));
    const zeroPlan = buildReturnZeroPlan(controlledNames, ids, currentPositions);
    player.ids = ids;
    player.jointPlan = zeroPlan;
    player.durationMs = zeroPlan.durationMs;

    player.detail = '取消旧轨迹…';
    announce();
    await publishTrajectory({ header: {}, joint_names: [], points: [] });
    await sleep(CANCEL_SETTLE_MS);
    assertPlaybackSession();
    if (player.cancelRequested) throw new Error('已停止');

    if (player.enable) {
      player.detail = `武装 kp=${player.kp} kd=${player.kd}…`;
      announce();
      await armMotors();
      if (player.cancelRequested) throw new Error('已停止');
    } else {
      player.detail = '跳过武装（未勾选「使能」）';
      announce();
    }

    const tid = await sendPlan();
    player.detail = `平滑回初始位并保持 · ${zeroPlan.durationMs}ms`;
    announce();
    const verdict = await waitPlan(tid);
    if (verdict === 'stopped') throw new Error('已停止');
    if (verdict === 'canceled') throw new Error(`回初始位未成功（取消/拒绝/执行失败）：${store.commandStatus.data?.reject || store.commandStatus.data?.reason || '请核对控制状态'}`);
    if (verdict !== 'done') throw new Error(`回初始位无 JTC 结果回执：${store.commandStatus.data?.reject || '请核对控制客户端状态'}`);
    player.phase = 'done';
    player.detail = '已回初始位并保持';
    announce();
  } catch (error: any) {
    player.phase = 'error';
    player.error = player.cancelRequested ? '已停止' : error.message;
    if (player.planStarted && !player.cancelRequested && isConnected()
        && player.connectionGeneration === getConnectionGeneration()) {
      let sent = false;
      try { sent = await publishTrajectory({ header: {}, joint_names: [], points: [] }); } catch { /* ignore */ }
      if (!sent) player.error += '；取消未发送，运动停止未确认，请使用全部软停';
    }
    announce();
  } finally {
    player.loading = null;
  }
}

export function stopPlayback(): void {
  const wasRunning = player.phase === 'running';
  player.cancelRequested = true;
  player.loading?.abort();
  stopPolicyIntents(store);
  cancelTrajectory();
  if (!wasRunning) {
    player.phase = 'idle';
    player.detail = '已请求 JTC 取消并保持位置；全部软停请使用停止控件';
    announce();
  }
}

export function stopAll(): boolean {
  stopPlayback();
  return isConnected() && pub('stop', {});
}

export function playbackSnapshot() {
  const { phase, detail, error, name, loop } = player;
  return { phase, detail, error, name, loop };
}
