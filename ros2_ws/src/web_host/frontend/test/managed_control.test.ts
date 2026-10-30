import { webcrypto } from 'node:crypto';
import { afterEach, beforeEach, expect, it, vi } from 'vitest';
import { ManagedControl } from '../../web/js/core/managed_control.js';

const SESSION = 1_800_000_000_000;
const EPOCH = 17;
const policy = {count: 1, targets: [{motor_id: 1, enabled: true, kp: 1, kd: .1}]};
const command = {count: 1, targets: [{motor_id: 1, position: .2, velocity: 0, effort: 0}]};
const trajectory = {count: 2, target_count: 1, target_ids: [1], points: [
  {motor_id: 1, position: .1, time_from_start_ms: 0},
  {motor_id: 1, position: .2, time_from_start_ms: 1000},
]};
const jointTrajectory = {joint_names: ['j1'], points: [
  {positions: [.1], time_from_start: {sec: 0, nanosec: 0}},
  {positions: [.2], time_from_start: {sec: 1, nanosec: 0}},
]};

function deferred<T>() {
  let resolve!: (value: T) => void;
  const promise = new Promise<T>(done => { resolve = done; });
  return {promise, resolve};
}

function fixture() {
  let clock = 1000;
  const publish = vi.fn();
  const service = vi.fn(async (name: string) => {
    if (name === '/robot/acquire_control') return {
      success: true, lease_id: 'private-test-token', epoch: EPOCH,
      session_id: SESSION, lease_ms: 1000,
    };
    if (name === '/robot/renew_control') return {success: true, lease_remaining_ms: 1000};
    return {success: true};
  });
  const client = new ManagedControl({service, publish, now: () => clock,
    wall: () => SESSION + clock, changed: vi.fn()});
  const state = (changes = {}) => ({epoch: EPOCH, session_id: SESSION,
    ready: true, armed: false, mcu_authorized: false, fault_latched: false,
    owner: client.owner || '', ...changes});
  client.setConnected(true);
  client.update(state());
  return {client, service, publish, state,
    advance(ms: number) { clock += ms; },
    refresh(changes = {}) { client.update(state(changes)); },
    now: () => clock};
}

async function armedFixture(ack = true) {
  const fixtureState = fixture();
  await fixtureState.client.acquire();
  await fixtureState.client.arm(true);
  fixtureState.refresh({armed: true, mcu_authorized: ack});
  return fixtureState;
}

beforeEach(() => {
  vi.useFakeTimers();
  vi.stubGlobal('crypto', webcrypto);
});
afterEach(() => {
  vi.clearAllTimers();
  vi.useRealTimers();
  vi.unstubAllGlobals();
});

it('申请和武装不会自行发布使能；必须等 MCU 确认后才能提交运动', async () => {
  const f = await armedFixture(false);
  expect(f.publish).not.toHaveBeenCalled();
  expect(f.client.send('policy', policy)).toBe(false);
  expect(f.client.send('command', command)).toBe(false);
  expect(f.client.send('trajectory', trajectory)).toBe(false);
  await expect(f.client.sendJointTrajectory(jointTrajectory)).rejects.toThrow();
  expect(f.publish).not.toHaveBeenCalled();
  f.refresh({armed: true, mcu_authorized: true});
  expect(f.client.send('policy', policy)).toBe(true);
});

it('所有托管通道共享严格递增序号并保留完整会话和新鲜时间戳', async () => {
  const f = await armedFixture();
  expect(f.client.send('policy', policy)).toBe(true);
  expect(f.client.send('command', command)).toBe(true);
  expect(f.client.send('trajectory', trajectory)).toBe(true);
  expect(f.publish.mock.calls.map(call => call[0])).toEqual([
    '/robot/control/policy', '/robot/control/command', '/robot/control/trajectory',
  ]);
  expect(f.publish.mock.calls.map(call => call[2].sequence)).toEqual([1, 2, 3]);
  for (const [, type, message] of f.publish.mock.calls) {
    expect(type).toMatch(/^robot_system_msgs\/msg\/Leased/);
    expect(message).toMatchObject({lease_id: 'private-test-token', epoch: EPOCH, session_id: SESSION});
    expect(message.header.stamp).toEqual({sec: 1_800_000_001, nanosec: 0});
  }
});

it('续租使用 RenewControl 的 lease_remaining_ms 字段并按发送时刻保守计时', async () => {
  const f = await armedFixture();
  f.advance(350);
  f.refresh({armed: true, mcu_authorized: true});
  f.client.tick();
  await Promise.resolve();
  await Promise.resolve();
  expect(f.service).toHaveBeenCalledWith('/robot/renew_control',
    'robot_system_msgs/srv/RenewControl',
    {lease_id: 'private-test-token', epoch: EPOCH, lease_ms: 1000}, 1000);
  expect(f.client.lease.deadline).toBe(f.now() + 1000);
  expect(Number.isFinite(f.client.lease.deadline)).toBe(true);
});

it.each(['hidden', 'disconnect', 'session', 'fault', 'expired-state'])('%s 清除令牌和旧目标，恢复后需要重新申请', async reason => {
  const f = await armedFixture();
  f.client.send('command', command);
  if (reason === 'hidden') f.client.setVisible(false);
  if (reason === 'disconnect') f.client.setConnected(false);
  if (reason === 'session') f.refresh({session_id: SESSION + 1, armed: true, mcu_authorized: true});
  if (reason === 'fault') f.refresh({fault_latched: true, ready: false});
  if (reason === 'expired-state') { f.advance(501); f.client.tick(); }
  expect(f.client.lease).toBeNull();
  expect(f.client.command).toBeNull();
  expect(f.client.timer).toBeNull();
  f.client.setConnected(true);
  f.client.setVisible(true);
  f.refresh({armed: true, mcu_authorized: true});
  const before = f.publish.mock.calls.length;
  f.client.tick();
  expect(f.client.send('command', command)).toBe(false);
  expect(f.publish).toHaveBeenCalledTimes(before);
  expect(f.service.mock.calls.filter(call => call[0] === '/robot/acquire_control')).toHaveLength(1);
});

it('申请响应跨越页面隐藏时立即释放，不保留迟到的控制令牌', async () => {
  const f = fixture();
  const pending = deferred<any>();
  f.service.mockImplementationOnce(() => pending.promise);
  const operation = f.client.acquire();
  f.client.setVisible(false);
  pending.resolve({success: true, lease_id: 'late-token', epoch: EPOCH, session_id: SESSION, lease_ms: 1000});
  await expect(operation).rejects.toThrow();
  expect(f.client.lease).toBeNull();
  expect(f.service).toHaveBeenCalledWith('/robot/release_control',
    'robot_system_msgs/srv/ReleaseControl', {lease_id: 'late-token', epoch: EPOCH}, 500);
});

it('申请尚未返回时重复点击不会创建第二个控制会话', async () => {
  const f = fixture();
  const pending = deferred<any>();
  f.service.mockImplementationOnce(() => pending.promise);
  const operation = f.client.acquire();
  await expect(f.client.acquire()).rejects.toThrow();
  expect(f.service.mock.calls.filter(call => call[0] === '/robot/acquire_control')).toHaveLength(1);
  pending.resolve({success: true, lease_id: 'one-token', epoch: EPOCH, session_id: SESSION, lease_ms: 1000});
  await operation;
  expect(f.client.lease.lease_id).toBe('one-token');
});

it('一次手动意图仅短时重发，面板停止产生新意图后不会永久续播旧目标', async () => {
  const f = await armedFixture();
  f.client.send('command', command);
  f.advance(50);
  f.refresh({armed: true, mcu_authorized: true});
  f.client.tick();
  const shortStream = f.publish.mock.calls.length;
  expect(shortStream).toBeGreaterThan(1);
  f.advance(201);
  f.refresh({armed: true, mcu_authorized: true});
  f.client.tick();
  expect(f.publish).toHaveBeenCalledTimes(shortStream);
  expect(f.client.command).toBeNull();
});

it('标准 Action 正在执行时不接收手动目标，也不混入另一条轨迹', async () => {
  const f = await armedFixture();
  const result = deferred<any>();
  f.service.mockImplementation(async (name: string) => {
    if (name.endsWith('/get_result')) return result.promise;
    if (name.endsWith('/send_goal')) return {accepted: true};
    return {success: true};
  });
  const operation = f.client.sendJointTrajectory(jointTrajectory);
  await Promise.resolve();
  await Promise.resolve();
  await Promise.resolve();
  await Promise.resolve();
  const authorization = f.service.mock.calls.find(call => call[0] === '/robot/authorize_motion');
  const goal = f.service.mock.calls.find(call => call[0].endsWith('/send_goal'));
  expect(authorization).toBeDefined();
  expect(goal).toBeDefined();
  expect(authorization![2].goal_id).toHaveLength(16);
  expect(goal![2].goal_id.uuid).toEqual(authorization![2].goal_id);
  expect(authorization![2]).toMatchObject({lease_id: 'private-test-token', epoch: EPOCH,
    session_id: SESSION, sequence: 1});
  expect(f.client.send('command', command)).toBe(false);
  expect(f.client.send('trajectory', trajectory)).toBe(false);
  await expect(f.client.sendJointTrajectory(jointTrajectory)).rejects.toThrow();
  expect(f.publish).not.toHaveBeenCalled();
  result.resolve({status: 4, result: {error_code: 0}});
  await operation;
  expect(f.client.message).toContain('成功');
});

it('断连后迟到的续租回执不会恢复令牌或发送旧目标', async () => {
  const f = await armedFixture();
  const pending = deferred<any>();
  f.service.mockImplementationOnce(() => pending.promise);
  f.advance(350);
  f.refresh({armed: true, mcu_authorized: true});
  f.client.tick();
  f.client.setConnected(false);
  pending.resolve({success: true, lease_remaining_ms: 1000});
  await Promise.resolve();
  await Promise.resolve();
  expect(f.client.lease).toBeNull();
  expect(f.client.timer).toBeNull();
  f.client.setConnected(true);
  f.refresh({armed: true, mcu_authorized: true});
  f.client.tick();
  expect(f.publish).not.toHaveBeenCalled();
});
