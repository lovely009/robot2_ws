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

function state() {
  return {modelJoints: [joint], transmissionStatus: {data: {model:'arm',joint_names:['elbow'],
    joint_limits:{elbow:{lower:-.1,upper:.4}}}}};
}

it('uses host readback and ignores old unconfirmed or corrupt browser drafts', () => {
  localStorage.setItem('web_host.joint_limits.v1.arm', JSON.stringify({elbow:{lower:-9,upper:9}}));
  expect(api.getJointLimits(state(),'arm',joint)).toEqual({lower:-.1,upper:.4,error:''});
  localStorage.setItem('web_host.joint_limits.v1.arm','{bad json');
  expect(api.jointLimitSyncError(state(),'arm','elbow')).toBe('');
  expect(api.getJointLimits({},'arm',joint)).toMatchObject({lower:-.17,upper:.52});
});

it('prepares explicitly widened overrides without saving or changing the effective range', () => {
  const s=state();
  const parameter=api.jointLimitsParameter(s,'arm',{name:'elbow',side:'lower',value:-.5});
  expect(JSON.parse(parameter.value.string_value)).toEqual({model:'arm',
    joint_limits:{elbow:{lower:-.5,upper:.4}}});
  expect(localStorage.getItem('web_host.joint_limits.v1.arm')).toBeNull();
  expect(api.getJointLimits(s,'arm',joint).lower).toBe(-.1);
  s.transmissionStatus.data.joint_limits.elbow.lower=-.5;
  api.saveJointLimits('arm',joint,api.getJointLimits(s,'arm',joint));
  expect(JSON.parse(localStorage.getItem('web_host.joint_limits.v1.arm')!)).toEqual({elbow:{lower:-.5,upper:.4}});
});

it('rejects invalid bounds and unconnected joints before any host submission', () => {
  const s=state();
  for (const value of [NaN,Infinity,.4,.5])
    expect(()=>api.jointLimitsParameter(s,'arm',{name:'elbow',side:'lower',value})).toThrow(/限位/);
  expect(()=>api.jointLimitsParameter(s,'arm',{name:'missing',side:'lower',value:-1})).toThrow(/未接入/);
  expect(()=>api.jointLimitsParameter(s,'other',{name:'elbow',side:'lower',value:-1})).toThrow(/模型/);
  expect(localStorage.getItem('web_host.joint_limits.v1.arm')).toBeNull();
});

it('requires valid host limits for controlled joints and keeps target validation', () => {
  const s=state();
  expect(api.jointTargetLimitError(s,'arm','elbow',.45)).toMatch(/超出/);
  expect(api.jointTargetLimitError(s,'arm','elbow',.3)).toBe('');
  delete (s.transmissionStatus.data as any).joint_limits;
  expect(api.jointLimitSyncError(s,'arm','elbow')).toMatch(/主机.*限位/);
});
