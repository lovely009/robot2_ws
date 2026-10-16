// 重力补偿开关（ADR 0013）：把本页面的下行命令源在「直连固件」与「经重力前馈叠加」
// 之间切换，并托管叠加节点（bench-gravity-feedforward.service）。
//
// 适用面是「STM32 在环」——叠加节点输出的话题只有固件订阅。判据取 Agent 在线
// （页面唯一看得见的在环信号）：不在环时按钮置灰并写明原因，不做静默降级。
//
// 面板要同时说清三件**互相独立**的事实，缺一件就会出现「亮着但不生效」：
//   1. 本页面的接线（路由）：direct | overlay —— 页面自己的状态，别的命令源不受它约束；
//   2. 叠加节点的进程状态与配置：来自 /bench/status 的 gravity_feedforward 段；
//   3. τ_g 的输入流 /joint_states 是否新鲜 —— 它断了节点会**静默**整帧直通。
// 三者不一致时是告警而不是自愈：路由开着而节点不在、或节点在而路由是直连，都摆明说。
import { $, DOWNLINK_ROUTE } from '../core/format.js';
import {
  store, MODEL, availableModels, modelJointsFor, jointMapSpec,
} from '../core/store.js';
import { getDownlinkRoute, setDownlinkRoute } from '../core/ros.js';
import * as bench from '../core/agent.js';

const STORAGE_KEY = 'robot2.feedforward.options.v1';
// 与 gravity_feedforward_overlay 的 joint_states_timeout_s 同口径（默认 0.5 s）。
// 页面这一侧放宽一档：它只是给操作者看的提示，不该比节点本身更早喊。
const JOINT_STATES_STALE_MS = 1000;
const REFRESH_MS = 1000;

const stateLabels = {
  stopped: '未启动',
  running: '运行中',
  failed: '已退出',
};

export function mount(root, ctx) {
  const ui = {
    route: $('ffRoute'),
    model: $('ffModel'),
    gain: $('ffGain'),
    revert: $('btnFfRevert'),
    summary: $('ffSummary'),
    state: $('ffState'),
    alarm: $('ffAlarm'),
    result: $('ffResult'),
    hint: $('ffHint'),
  };

  let busy = false;
  let refreshTimer = null;

  function readSaved() {
    try { return JSON.parse(localStorage.getItem(STORAGE_KEY) || '{}') || {}; }
    catch (_error) { return {}; }
  }

  const saved = readSaved();

  function saveOptions() {
    try {
      localStorage.setItem(STORAGE_KEY, JSON.stringify({
        model: ui.model.value,
        effort_gain: ui.gain.value,
      }));
    } catch (_error) {
      // 浏览器禁用存储时面板照常可用，只是不记忆。
    }
  }

  function setResult(message, kind = '') {
    ui.result.textContent = message;
    ui.result.className = `result tiny${kind ? ` ${kind}` : ''}`;
  }

  // 允许增益留空/非数字 → 报错而不是把 NaN 发出去（服务端也会拒，但那里说不清是哪个框）。
  function readGain() {
    const value = Number(ui.gain.value);
    if (!Number.isFinite(value) || value < 0) {
      throw new Error('增益必须是不小于 0 的数字');
    }
    return value;
  }

  function unit() {
    return store.bench.gravity_feedforward || null;
  }

  function agentOnline() {
    return (store.bench.agent || {}).state === 'running';
  }

  function routeIsOverlay() {
    return getDownlinkRoute() === DOWNLINK_ROUTE.OVERLAY;
  }

  function jointStatesAgeMs() {
    return store.jointStates.last ? Date.now() - store.jointStates.last : null;
  }

  function jointStatesFresh() {
    const age = jointStatesAgeMs();
    return age !== null && age <= JOINT_STATES_STALE_MS;
  }

  // 断链：路由指着叠加节点，而它没在跑。ADR 0013 明确**不自动回退**——改接线必须由人
  // 按下，且事后要能在事件日志里查到那一刻。所以这里只把话说重。
  function brokenRoute() {
    return routeIsOverlay() && (unit() || {}).state !== 'running';
  }

  function renderGate() {
    if (agentOnline()) {
      ui.route.title = '';
      return;
    }
    ui.route.disabled = true;
    ui.route.title = '需要 STM32 在环：先接上控制板并在顶栏点「启动并连接」';
  }

  function renderAlarm() {
    const ff = unit() || {};
    const lines = [];
    if (brokenRoute()) {
      lines.push(ff.state === 'failed'
        ? `叠加节点已退出（exit ${ff.returncode}）——命令发的还是 /robot_dynamics/*，`
          + '固件收不到任何东西。先回退直连，再查日志。'
        : '叠加节点没在跑，命令发的还是 /robot_dynamics/*，固件收不到任何东西。');
    }
    if (routeIsOverlay() && ff.state === 'running' && !jointStatesFresh()) {
      const age = jointStatesAgeMs();
      lines.push('叠加节点在跑，但 /joint_states 不新鲜（'
        + (age === null ? '本次连接内一帧都没收到' : `${(age / 1000).toFixed(1)} s 前`)
        + '）：τ_g 会按 0 处理、整帧原样转发——开关亮着但没有补偿。'
        + 'HIL 下这条流来自 firmware_topic_bridge（motion_states → /joint_states），'
        + '确认它起着。');
    }
    if (routeIsOverlay() && ff.state === 'running' && ff.config
        && ff.config.model && ff.config.model !== ui.model.value) {
      lines.push(`叠加节点跑的是模型 ${ff.config.model}，面板选的是 ${ui.model.value}：`
        + 'τ_g 是按台上那台的 URDF 算的，对不上说明关节映射就是错的。'
        + '换模型要关掉开关再开（服务拒绝静默替换）。');
    }
    if (ff.gain_error) lines.push(`最近一次增益写入未生效：${ff.gain_error}`);
    ui.alarm.hidden = lines.length === 0;
    ui.alarm.textContent = lines.join(' ');
  }

  function renderStatus() {
    const ff = unit();
    const overlay = routeIsOverlay();
    const state = ff ? (ff.state || 'stopped') : 'stopped';
    const label = stateLabels[state] || state;
    const config = ff && ff.config ? ff.config : null;

    ui.route.checked = overlay;
    // 关着的时候节点不该在跑（关 = 切回直连 + 停节点）；开着而节点不在＝断链，由告警说。
    ui.revert.hidden = !brokenRoute();
    ui.summary.textContent = overlay
      ? (state === 'running' ? '经前馈' : '断链')
      : (state === 'running' ? '节点在跑·未接线' : '直连');
    ui.summary.className = `sum${brokenRoute() ? ' err' : (overlay ? ' ok' : '')}`;

    const parts = [overlay ? '下行经 /robot_dynamics/*' : '下行直连固件'];
    if (config) {
      parts.push(`${config.model} · 增益 ${config.effort_gain}`);
    }
    parts.push(`叠加节点${label}`);
    if (state === 'failed' && ff.returncode !== null && ff.returncode !== undefined) {
      parts.push(`exit ${ff.returncode}`);
    }
    if (overlay) {
      const age = jointStatesAgeMs();
      parts.push(age === null ? '/joint_states 一帧未收'
        : `/joint_states ${(age / 1000).toFixed(1)} s`);
    }
    ui.state.textContent = parts.join(' · ');

    ui.route.disabled = busy || !agentOnline();
    ui.gain.disabled = busy;
    ui.model.disabled = busy;
    renderGate();
    renderAlarm();
  }

  function renderHint() {
    if (!agentOnline()) {
      ui.hint.textContent = '重力补偿要 STM32 在环：叠加节点的输出只有固件订阅，'
        + '纯 Gazebo / 纯 MuJoCo / 主机驱动闭环三路没有接收者。';
      return;
    }
    if (brokenRoute()) {
      ui.hint.textContent = '节点不在时命令会全部落空：先点「立即回退」把下行切回直连，'
        + '再从日志（仿真屏）查它为什么退出。';
      return;
    }
    ui.hint.textContent = routeIsOverlay()
      ? '本页面发的命令与轨迹都经叠加节点加 τ_g 后转发给固件。'
        + '其它命令源（snapshot_stream.py / traj_play.py / command_streamer.py）'
        + '不经这里，同时跑就是混流。'
      : '填好模型与增益后打开开关：服务会先起叠加节点并确认它 running，再把本页面的'
        + '下行切到 /robot_dynamics/*。关闭时反过来先切回直连、再停节点。';
  }

  function renderAll() {
    renderStatus();
    renderHint();
  }

  // 起：先起节点、确认 running，再切话题。反过来会出现「话题已切、节点没起」的空窗
  // （命令直接落空）。任何一步失败都保持在直连——不把下游留在断链上。
  async function turnOn() {
    const model = ui.model.value;
    if (!model) {
      setResult('先选择台上那台的模型。', 'err');
      return;
    }
    let gain;
    try {
      gain = readGain();
    } catch (error) {
      setResult(error.message, 'err');
      return;
    }
    busy = true;
    renderStatus();
    setResult('正在启动叠加节点…');
    try {
      const joints = await modelJointsFor(model);
      await bench.startGravityFeedforward({
        model, joint_map: jointMapSpec(joints), effort_gain: gain,
      });
      setDownlinkRoute(DOWNLINK_ROUTE.OVERLAY);
      setResult(`下行已切到前馈（模型 ${model}，增益 ${gain}）。`, 'ok');
      saveOptions();
    } catch (error) {
      setResult(`开启失败：${error.message}`, 'err');
    } finally {
      busy = false;
      renderAll();
    }
  }

  // 关：先切回直连、再停节点。顺序与开启镜像——先停节点会让开关还亮着的那一瞬间命令
  // 全部落空。
  async function turnOff() {
    busy = true;
    renderStatus();
    const wasOverlay = routeIsOverlay();
    if (wasOverlay) setDownlinkRoute(DOWNLINK_ROUTE.DIRECT);
    setResult('正在停止叠加节点…');
    try {
      await bench.stopGravityFeedforward();
      setResult('下行已切回直连，叠加节点已停止。', 'ok');
    } catch (error) {
      setResult(`停止失败：${error.message}`, 'err');
    } finally {
      busy = false;
      renderAll();
    }
  }

  // 断链告警上的一键回退：只切回直连，**不动**已经在跑的节点（它可能只是另一个模型/
  // 刚被外部停掉），要停请在开关上关一次。
  function revertToDirect() {
    setDownlinkRoute(DOWNLINK_ROUTE.DIRECT);
    setResult('下行已切回直连固件。', 'ok');
    renderAll();
  }

  async function applyGain() {
    let gain;
    try {
      gain = readGain();
    } catch (error) {
      setResult(error.message, 'err');
      return;
    }
    if (!routeIsOverlay() || (unit() || {}).state !== 'running') {
      setResult('增益只在叠加节点运行中可改；现在先打开开关。', 'err');
      return;
    }
    busy = true;
    renderStatus();
    try {
      await bench.setGravityFeedforwardGain(gain);
      // 服务 ok 只代表写请求发出去了；真值以快照（gain_error / config.effort_gain）为准。
      const ff = unit() || {};
      if (ff.gain_error) {
        setResult(`增益写入未生效：${ff.gain_error}`, 'err');
      } else {
        setResult(`增益已改为 ${(ff.config || {}).effort_gain}。`, 'ok');
      }
      saveOptions();
    } catch (error) {
      setResult(`改增益失败：${error.message}`, 'err');
    } finally {
      busy = false;
      renderAll();
    }
  }

  function onRouteChange() {
    if (ui.route.checked) turnOn();
    else turnOff();
  }

  // 模型候选与臂视图同源（有 urdf/<名>.urdf 的目录）；默认取当前模型——HIL 会话是
  // 操作者手起的，页面没有别的途径知道台上是哪台，只能给个可改的默认值。
  async function loadModels() {
    try {
      const models = await availableModels();
      ui.model.replaceChildren();
      for (const name of models) {
        const option = document.createElement('option');
        option.value = name;
        option.textContent = name;
        ui.model.append(option);
      }
      const desired = saved.model && models.includes(saved.model) ? saved.model : MODEL;
      if (models.includes(desired)) ui.model.value = desired;
      else if (models.length) ui.model.value = models[0];
    } catch (error) {
      ui.model.replaceChildren();
      const option = document.createElement('option');
      option.value = '';
      option.textContent = '取不到模型列表';
      ui.model.append(option);
    }
    // 候选载入后模型框才有值，而告警里的模型比对要拿它当基准。
    renderAll();
  }

  ui.route.addEventListener('change', onRouteChange);
  ui.revert.addEventListener('click', revertToDirect);
  ui.gain.addEventListener('change', applyGain);
  ui.model.addEventListener('change', () => {
    saveOptions();
    renderAll();
  });

  ui.gain.value = saved.effort_gain !== undefined ? String(saved.effort_gain) : '1';
  renderAll();
  loadModels();
  // /joint_states 的新鲜度只能靠时间推进才看得出来：流断了就没有新帧、也就没有脏帧，
  // 于是「开关亮着但 τ_g 恒为 0」这条静默失效永远不会被重绘出来。只在开关亮着时有话说。
  refreshTimer = window.setInterval(() => {
    if (routeIsOverlay()) renderStatus();
  }, REFRESH_MS);

  return {
    render(dirty) {
      if (dirty.has(DIRTY.BENCH) || dirty.has(DIRTY.CONN)) renderAll();
    },
    unmount() {
      if (refreshTimer !== null) {
        clearInterval(refreshTimer);
        refreshTimer = null;
      }
    },
  };
}
