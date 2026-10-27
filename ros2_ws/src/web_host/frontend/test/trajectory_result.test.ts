import { expect, it, vi } from 'vitest';
vi.stubGlobal('location', { search: '' });
vi.stubGlobal('localStorage', { getItem: () => null });
const { trajectoryWindowResult: result } = await import(/* @vite-ignore */
  new URL('../../web/js/panels/actionLibrary.js', import.meta.url).href);
it('recognizes the V3 terminal soft stop as success and waits for completion', () => {
  expect(result({trajectory_id:7,active:false,completed:true,elapsed_ms:1000},7,1000)).toBe('done');
  expect(result({trajectory_id:7,active:true,completed:false,elapsed_ms:1000},7,1000)).toBe('waiting');
  expect(result({trajectory_id:7,active:true,completed:true,elapsed_ms:1000},7,1000)).toBe('done');
});
it('keeps cancellation and unrelated trajectories distinct from completion', () => {
  expect(result({trajectory_id:7,active:false,completed:false,elapsed_ms:500},7,1000)).toBe('canceled');
  expect(result({trajectory_id:8,active:false,completed:true,elapsed_ms:1000},7,1000)).toBe('waiting');
  expect(result(null,7,1000)).toBe('waiting');
});
