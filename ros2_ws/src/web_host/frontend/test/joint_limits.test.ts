import { afterEach, beforeEach, expect, it, vi } from 'vitest';

beforeEach(() => {
  const values = new Map<string, string>();
  vi.stubGlobal('localStorage', {
    getItem: (key: string) => values.get(key) ?? null,
    setItem: (key: string, value: string) => values.set(key, value),
  });
});
afterEach(() => vi.unstubAllGlobals());
const api = await import(/* @vite-ignore */ new URL('../../web/js/core/joint_limits.js', import.meta.url).href);
const joint = {name: 'elbow', type: 'revolute', lower: -0.17, upper: 0.52};

it('defaults to URDF bounds and saves each joint and model independently', () => {
  expect(api.getJointLimits('arm', joint)).toMatchObject({lower: -0.17, upper: 0.52});
  api.saveJointLimit('arm', joint, 'lower', -0.1);
  api.saveJointLimit('arm', joint, 'upper', 0.4);
  expect(api.getJointLimits('arm', joint)).toMatchObject({lower: -0.1, upper: 0.4});
  expect(api.getJointLimits('other', joint)).toMatchObject({lower: -0.17, upper: 0.52});
  expect(api.getJointLimits('arm', {...joint, name: 'wrist'})).toMatchObject({lower: -0.17, upper: 0.52});
});

it('rejects invalid edits without losing the saved bounds', () => {
  api.saveJointLimit('arm', joint, 'lower', -0.1);
  expect(() => api.saveJointLimit('arm', joint, 'upper', -0.2)).toThrow();
  expect(() => api.saveJointLimit('arm', joint, 'lower', NaN)).toThrow();
  expect(api.getJointLimits('arm', joint)).toMatchObject({lower: -0.1, upper: 0.52});
});

it('does not invent bounds for continuous joints and refuses corrupt storage', () => {
  expect(api.getJointLimits('arm', {name: 'spin', type: 'continuous'})).toMatchObject({lower: null, upper: null});
  localStorage.setItem('web_host.joint_limits.v1.arm', '{bad json');
  expect(api.getJointLimits('arm', joint).error).toBeTruthy();
});
