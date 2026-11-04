import { test } from 'node:test';
import assert from 'node:assert/strict';
import vm from 'node:vm';
import { readFileSync } from 'node:fs';

const source = readFileSync(new URL('./app.js', import.meta.url), 'utf8');
function fixture({ denied = false, httpError = false } = {}) {
  const elements = Object.fromEntries(['start', 'stop', 'mute', 'status', 'audio', 'events', 'api-key'].map(id => [id, { disabled: id !== 'start', textContent: '', value: '', play: async () => {} }]));
  const requests = [];
  const track = { enabled: true, stopped: false, stop() { this.stopped = true; } };
  const stream = { getTracks: () => [track], getAudioTracks: () => [track] };
  let peer;
  class Peer {
    constructor() { peer = this; this.iceGatheringState = 'complete'; }
    addTrack() {}
    createOffer() { return { sdp: 'v=0' }; }
    setLocalDescription(value) { this.localDescription = value; }
    setRemoteDescription(value) { this.answer = value; }
    createDataChannel(label) {
      assert.equal(label, 'oai-events');
      return this.channel = { readyState: 'open', sent: [], close() { this.onclose?.(); }, send(data) { this.sent.push(JSON.parse(data)); } };
    }
    close() { this.closed = true; }
  }
  let pagehide;
  const context = vm.createContext({
    document: { querySelector: selector => elements[selector.slice(1)] },
    navigator: { mediaDevices: { getUserMedia: async () => { if (denied) throw new Error('permission denied'); return stream; } } },
    RTCPeerConnection: Peer, AbortController, MediaStream: class {}, setTimeout, clearTimeout,
    window: { addEventListener: (_name, fn) => { pagehide = fn; } },
    fetch: async (_url, options) => {
      requests.push(JSON.parse(options.body));
      return { ok: !httpError, json: async () => httpError ? { error: 'HTTP 403' } : { session: { id: 'live_test' }, transport: { sdp: 'answer' } } };
    },
  });
  vm.runInContext(source, context);
  return { elements, track, requests, get peer() { return peer; },
    event: type => peer.channel.onmessage({ data: JSON.stringify({ type }) }),
    hide: () => pagehide() };
}

test('waits for session.started, mutes, and gracefully closes', async () => {
  const f = fixture();
  await f.elements.start.onclick();
  assert.equal(f.elements.stop.disabled, true);
  assert.equal(f.peer.channel.sent.length, 0);
  f.event('session.started');
  assert.equal(f.elements.stop.disabled, false);
  f.elements.mute.onclick();
  assert.equal(f.track.enabled, false);
  f.elements.mute.onclick();
  assert.equal(f.track.enabled, true);
  f.elements.stop.onclick();
  assert.deepEqual(f.peer.channel.sent, [{ type: 'session.close' }]);
  assert.equal(f.track.stopped, false);
  f.event('session.closed');
  assert.equal(f.track.stopped, true);
  assert.equal(f.peer.closed, true);
  assert.equal(f.elements.start.disabled, false);
  assert.equal(f.elements.status.textContent, '对话已结束。');
});

test('microphone permission rejection allows retry', async () => {
  const f = fixture({ denied: true });
  await f.elements.start.onclick();
  assert.equal(f.elements.start.disabled, false);
  assert.match(f.elements.status.textContent, /permission denied/);
  assert.equal(f.peer.closed, true);
});

test('HTTP failure releases microphone and preserves error', async () => {
  const f = fixture({ httpError: true });
  await f.elements.start.onclick();
  assert.equal(f.track.stopped, true);
  assert.equal(f.elements.status.textContent, 'HTTP 403');
  assert.equal(f.elements.start.disabled, false);
});

test('unexpected channel close releases microphone', async () => {
  const f = fixture();
  await f.elements.start.onclick();
  f.peer.channel.onclose();
  assert.equal(f.track.stopped, true);
  assert.match(f.elements.status.textContent, /最终用量/);
});

test('pagehide requests closure and releases tracks', async () => {
  const f = fixture();
  await f.elements.start.onclick();
  f.event('session.started');
  f.hide();
  assert.deepEqual(f.peer.channel.sent, [{ type: 'session.close' }]);
  assert.equal(f.track.stopped, true);
});

test('entered key is sent only with the connection request and field unlocks on failure', async () => {
  const f = fixture({ httpError: true });
  f.elements['api-key'].value = '  sk-page-test  ';
  await f.elements.start.onclick();
  assert.equal(f.requests[0].api_key, 'sk-page-test');
  assert.equal(f.elements['api-key'].disabled, false);
  assert.doesNotMatch(f.elements.events.textContent, /sk-page-test/);
});
