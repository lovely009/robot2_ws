import { beforeEach, afterEach, expect, it, vi } from 'vitest';
beforeEach(() => {
  const values = new Map<string, string>();
  vi.stubGlobal('localStorage', { getItem: (k: string) => values.get(k) ?? null,
    setItem: (k: string, v: string) => values.set(k, v) });
});
afterEach(() => vi.unstubAllGlobals());
it('按模型保存，边界允许，超限及非有限位置拒绝', async () => {
  const api = await import(/* @vite-ignore */ new URL('../../web/js/core/host_limits.js', import.meta.url).href);
  api.saveLimits('a', [{id: 1, enabled: true, min: -0.417, max: 0.304}]);
  expect(api.limitsError([{motor_id: 1, position: 0.304}], 'a')).toBe('');
  expect(api.limitsError([{motor_id: 1, position: 0.305}], 'a')).toContain('m1');
  expect(api.limitsError([{motor_id: 1, position: NaN}], 'a')).toBeTruthy();
  expect(api.limitsError([{motor_id: 1, position: 1}], 'b')).toBe('');
});
it('拒绝非法设置，停用不拦截，损坏配置不能静默放行', async () => {
  const api = await import(/* @vite-ignore */ new URL('../../web/js/core/host_limits.js', import.meta.url).href);
  expect(() => api.saveLimits('a', [{id: 1, enabled: true, min: 20, max: 10}])).toThrow();
  expect(() => api.saveLimits('a', [{id: 1, enabled: true, min: '', max: 10}])).toThrow();
  api.saveLimits('a', [{id: 1, enabled: false, min: -0.417, max: 0.304}]);
  expect(api.limitsError([{motor_id: 1, position: 1}], 'a')).toBe('');
  vi.stubGlobal('localStorage', {getItem: () => '{bad'});
  expect(api.limitsError([{motor_id: 1, position: 0}], 'a')).toContain('配置');
});

it('rad 提示保留保存值，旧版度数配置按原行程换算', async () => {
  const api = await import(/* @vite-ignore */ new URL('../../web/js/core/host_limits.js', import.meta.url).href);
  api.saveLimits('a', [{id: 1, enabled: true, min: -0.417, max: 0.304}]);
  expect(api.limitsError([{motor_id: 1, position: 0.4}], 'a')).toContain('[-0.417, 0.304] rad');
  localStorage.setItem('web_host.host_limits.v1.old', JSON.stringify([{id: 1, enabled: true, min: -90, max: 90}]));
  expect(api.readLimits('old')[0].max).toBeCloseTo(Math.PI / 2);
});
