import {expect, it, vi} from 'vitest';
vi.stubGlobal('location', {search: ''});
vi.stubGlobal('localStorage', {getItem: () => null});
const {describeAgentStatus} = await import('../../web/js/core/agent.js');
const {applyAgentWireStatus} = await import('../../web/js/core/store.js');
function state() {
  return {conn: {connected: true}, agentWireStatus: {at: 1000, data: {
    schema_version: 4, session_id: 100, sequence: 1, snapshot_stamp: 1000000000,
    clock_valid: true, clock_sync_age_ms: 100,
    clock_sync_failures: 0, snapshot_tick_ms: 100,
    lease_active: true, lease_valid_until_tick_ms: 300,
  }}};
}
it('separates link, clock and authority including lease expiry', () => {
  const s = state();
  expect(describeAgentStatus(s, 1100)).toMatchObject({link: '在线', clock: '正常', authority: '有效'});
  expect(describeAgentStatus(s, 1200).authority).toBe('未授权');
  s.agentWireStatus.data.clock_valid = false;
  expect(describeAgentStatus(s, 1100)).toMatchObject({link: '在线', clock: '异常', authority: '未授权'});
});
it('never shows stale, disconnected or incompatible data as ready', () => {
  const s = state();
  expect(describeAgentStatus(s, 1301)).toMatchObject({link: '状态陈旧', clock: '未知', authority: '未知'});
  s.conn.connected = false;
  expect(describeAgentStatus(s, 1100).link).toBe('未连接');
  s.conn.connected = true; s.agentWireStatus.data.schema_version = 3;
  expect(describeAgentStatus(s, 1100).link).toBe('版本不匹配');
});
it('expires clock mapping and handles wrapped MCU ticks', () => {
  const s = state();
  s.agentWireStatus.data.clock_sync_age_ms = 29950;
  expect(describeAgentStatus(s, 1100).clock).toBe('异常');
  s.agentWireStatus.data.clock_sync_age_ms = 100;
  s.agentWireStatus.data.snapshot_tick_ms = 0xfffffff0;
  s.agentWireStatus.data.lease_valid_until_tick_ms = 184;
  expect(describeAgentStatus(s, 1100).authority).toBe('有效');
});

it('rejects malformed, replayed and old-session status without refreshing receipt time', () => {
  const s = state();
  expect(applyAgentWireStatus(s, {...s.agentWireStatus.data}, 1200)).toBe(false);
  expect(s.agentWireStatus.at).toBe(1000);
  expect(applyAgentWireStatus(s, {...s.agentWireStatus.data, sequence: 2, session_id: 99}, 1200)).toBe(false);
  expect(applyAgentWireStatus(s, {...s.agentWireStatus.data, sequence: 2, clock_valid: 'true'}, 1200)).toBe(false);
  expect(applyAgentWireStatus(s, {...s.agentWireStatus.data, sequence: 2}, 1200)).toBe(true);
  expect(s.agentWireStatus.at).toBe(1200);
});

it('does not revive an expired lease when a new sequence was delayed in transit', () => {
  const s = state();
  expect(applyAgentWireStatus(s, {...s.agentWireStatus.data, sequence: 2}, 2000)).toBe(true);
  expect(describeAgentStatus(s, 2000)).toMatchObject({link: '状态陈旧', clock: '未知', authority: '未知'});
  s.agentWireStatus.data.snapshot_stamp = 2200000000;
  expect(describeAgentStatus(s, 2000)).toMatchObject({clock: '时间基准异常', authority: '未知'});
});
