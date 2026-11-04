const start = document.querySelector('#start');
const stop = document.querySelector('#stop');
const mute = document.querySelector('#mute');
const status = document.querySelector('#status');
const audio = document.querySelector('#audio');
const log = document.querySelector('#events');
const apiKey = document.querySelector('#api-key');
let active;

function cleanup(session) {
  const owned = active === session;
  if (owned) active = undefined;
  clearTimeout(session.timer);
  session.controller.abort();
  session.stream?.getTracks().forEach(track => track.stop());
  session.channel?.close();
  session.peer.close();
  if (!owned) return;
  audio.srcObject = null;
  start.disabled = false;
  apiKey.disabled = false;
  stop.disabled = mute.disabled = true;
  mute.textContent = '麦克风静音';
}

function fail(session, message) {
  if (active !== session) return;
  cleanup(session);
  status.textContent = message;
}

start.onclick = async () => {
  start.disabled = true;
  apiKey.disabled = true;
  const enteredKey = apiKey.value.trim();
  log.textContent = '';
  status.textContent = '正在申请麦克风并连接…';
  let session;
  try {
    session = { peer: new RTCPeerConnection(), controller: new AbortController(), ready: false };
    active = session;
    session.stream = await navigator.mediaDevices.getUserMedia({ audio: true });
    if (active !== session) return cleanup(session);
    session.timer = setTimeout(() => fail(session, '建立会话超时，请重试。'), 45000);
    session.stream.getAudioTracks().forEach(track => session.peer.addTrack(track, session.stream));
    session.peer.ontrack = event => {
      if (active !== session) return;
      audio.srcObject = new MediaStream([event.track]);
      audio.play().catch(() => { status.textContent = '请点击音频控件播放助手声音。'; });
    };
    const channel = session.channel = session.peer.createDataChannel('oai-events');
    channel.onmessage = ({ data }) => {
      if (active !== session) return;
      try {
        const event = JSON.parse(data);
        log.textContent = (log.textContent + event.type + '\n').slice(-8000);
        if (event.type === 'session.started') {
          clearTimeout(session.timer);
          session.ready = true;
          stop.disabled = mute.disabled = false;
          status.textContent = '已连接，可以开始说话。';
        } else if (event.type === 'session.closed') {
          status.textContent = '对话已结束。';
          cleanup(session);
        } else if (event.type === 'error') {
          fail(session, '会话返回错误，请检查模型权限或稍后重试。');
        }
      } catch { fail(session, '收到无法解析的会话事件。'); }
    };
    channel.onclose = () => fail(session, '连接已断开，未确认最终用量。');
    session.peer.onconnectionstatechange = () => {
      if (session.peer.connectionState === 'failed') fail(session, '语音连接失败，请检查网络。');
    };
    await session.peer.setLocalDescription(await session.peer.createOffer());
    if (session.peer.iceGatheringState !== 'complete') {
      await new Promise((resolve, reject) => {
        const timeout = setTimeout(() => { session.peer.removeEventListener('icegatheringstatechange', changed); reject(new Error('收集网络候选地址超时。')); }, 10000);
        function changed() {
          if (session.peer.iceGatheringState !== 'complete') return;
          clearTimeout(timeout);
          session.peer.removeEventListener('icegatheringstatechange', changed);
          resolve();
        }
        session.peer.addEventListener('icegatheringstatechange', changed);
        changed();
      });
    }
    if (active !== session) return;
    const response = await fetch('/api/session', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ sdp: session.peer.localDescription.sdp, api_key: enteredKey }),
      signal: session.controller.signal,
    });
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || `HTTP ${response.status}`);
    if (active !== session) return;
    await session.peer.setRemoteDescription({ type: 'answer', sdp: result.transport.sdp });
    // HTTP already started the session; wait for session.started on the channel.
  } catch (error) {
    if (session) fail(session, error.message);
    else { status.textContent = error.message; start.disabled = apiKey.disabled = false; }
  }
};

mute.onclick = () => {
  if (!active?.ready) return;
  const tracks = active.stream.getAudioTracks();
  const enabled = !tracks[0].enabled;
  tracks.forEach(track => { track.enabled = enabled; });
  mute.textContent = enabled ? '麦克风静音' : '恢复麦克风';
};

stop.onclick = () => {
  const session = active;
  if (!session?.ready || session.channel.readyState !== 'open') return;
  stop.disabled = mute.disabled = true;
  session.stream.getTracks().forEach(track => { track.enabled = false; });
  status.textContent = '正在结束对话…';
  session.timer = setTimeout(() => fail(session, '结束确认超时，已断开；最终用量未确认。'), 15000);
  try { session.channel.send(JSON.stringify({ type: 'session.close' })); }
  catch { fail(session, '结束消息发送失败，已断开；最终用量未确认。'); }
};

window.addEventListener('pagehide', () => {
  if (!active) return;
  if (active.channel?.readyState === 'open') {
    try { active.channel.send(JSON.stringify({ type: 'session.close' })); } catch { /* Connection already closed. */ }
  }
  cleanup(active);
});
