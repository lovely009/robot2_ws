// 重力补偿开关（ADR 0013，接线归属见 ADR 0014 决定 3）：托管叠加节点
// （bench-gravity-feedforward.service），并把下行切到「经重力前馈叠加」。
//
// 下行有**两条**通道，归属不同：
//   · 命令（设定值 → command_streamer）：改道由 bench_manager 改 streamer 的
//     output_topic 执行，页面切不了，只能读 /motor_command/status 的回执核对；
//   · 轨迹：仍由页面自己发，所以 setDownlinkRoute 切的是它。
// 开启/关闭的顺序因此由服务端与页面各管一半，见 turnOn / turnOff 的注释。
//
// 适用面是「STM32 在环」——叠加节点输出的话题只有固件订阅。判据取 Agent 在线
// （页面唯一看得见的在环信号）：不在环时按钮置灰并写明原因，不做静默降级。
//
// 面板要同时说清四件**互相独立**的事实，缺一件就会出现「亮着但不生效」：
//   1. 开关的意图（本页面的路由状态）——别的命令源不受它约束；
//   2. 命令流**实际**发往哪里（/motor_command/status 的 output_topic）——服务端改的，
//      与 1 不一致就是「亮着但没切过去」；
//   3. 叠加节点的进程状态与配置：来自 /bench/status 的 gravity_feedforward 段；
//   4. τ_g 的输入流 /joint_states 是否新鲜 —— 它断了节点会**静默**整帧直通。
// 不一致时是告警而不是自愈：摆明说，改接线必须由人按下。
import { $, DOWNLINK_ROUTE } from '../core/format.js';
import {
  store, DIRTY, MODEL, availableModels, modelJointsFor, jointMapSpec,
} from '../core/store.js';
import { getDownlinkRoute, setDownlinkRoute, getConnectionGeneration, isManagedMode } from '../core/ros.js';
import * as bench from '../core/agent.js';
import { DIRECT_COMMAND_TOPIC, OVERLAY_COMMAND_TOPIC } from '../core/agent.js';

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
  if (isManagedMode()) {
    root.textContent = '托管 MCU 的运动由整机入口统一授权；本模式不开放台架重力前馈旁路。';
    return {render() {}, unmount() {}};
  }
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
  // 操作者的**意图**（开关位置）。实际接线以 commandStatus.output_topic 为准——
  // 改道是服务端做的，勾选框不能自己宣布成功。
  let wantOverlay = false;
  // 是否已用服务端观测值校准过意图（见 syncIntentFromServer）。
  let intentSynced = false;
  let intentGeneration = -1;

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

  // 命令流**实际**发往哪里。null = 本次连接内还没收到 /motor_command/status
  // （command_streamer 没起或还没发过状态）——那是「不知道」，不是「直连」。
  function commandTopic() {
    const data = store.commandStatus.data;
    return data ? data.output_topic : null;
  }

  function commandRouteIsOverlay() {
    return commandTopic() === OVERLAY_COMMAND_TOPIC;
  }

  // 开关亮着、节点也在跑，命令却还发在直连话题上：改道没生效（streamer 没起、
  // 参数被拒，或它刚重启回落到了默认话题）。这时按钮亮着却没有补偿。
  function routeNotSwitched() {
    return wantOverlay && (unit() || {}).state === 'running'
      && commandTopic() !== null && !commandRouteIsOverlay();
  }

  // 反向：开关关了，接线却还指着叠加节点。
  function routeNotReverted() {
    return !wantOverlay && commandRouteIsOverlay();
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
    return wantOverlay && (unit() || {}).state !== 'running';
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
    if (routeNotSwitched()) {
      lines.push(`叠加节点在跑，但命令实际还发在 ${commandTopic()}——改道没生效，`
        + (ff.route_error ? `服务端报：${ff.route_error}。` : '')
        + '开关亮着但没有补偿。');
    }
    if (routeNotReverted()) {
      lines.push(`开关已关，但命令实际仍发往 ${commandTopic()}——改道没切回来。`
        + (ff.route_error ? `服务端报：${ff.route_error}。` : ''));
    }
    if (wantOverlay && ff.state === 'running' && !jointStatesFresh()) {
      const age = jointStatesAgeMs();
      lines.push('叠加节点在跑，但 /joint_states 不新鲜（'
        + (age === null ? '本次连接内一帧都没收到' : `${(age / 1000).toFixed(1)} s 前`)
        + '）：τ_g 会按 0 处理、整帧原样转发——开关亮着但没有补偿。'
        + 'HIL 下这条流来自 firmware_topic_bridge（motion_states → /joint_states），'
        + '确认它起着。');
    }
    if (wantOverlay && ff.state === 'running' && ff.config
        && ff.config.model && ff.config.model !== ui.model.value) {
      lines.push(`叠加节点跑的是模型 ${ff.config.model}，面板选的是 ${ui.model.value}：`
        + 'τ_g 是按台上那台的 URDF 算的，对不上说明关节映射就是错的。'
        + '换模型要关掉开关再开（服务拒绝静默替换）。');
    }
    if (ff.gain_error) lines.push(`最近一次增益写入未生效：${ff.gain_error}`);
    ui.alarm.hidden = lines.length === 0;
    ui.alarm.textContent = lines.join(' ');
  }

  // unit 与命令路由跨页面存活；首次拿到两侧状态后校准本地开关，避免刷新后误报改道。
  function syncIntentFromServer() {
    const generation = getConnectionGeneration();
    if (generation !== intentGeneration) {
      intentGeneration = generation;
      intentSynced = false;
    }
    if (intentSynced || busy || !store.conn.connected) return;
    const ff = unit();
    const topic = commandTopic();
    if (!ff || ![DIRECT_COMMAND_TOPIC, OVERLAY_COMMAND_TOPIC].includes(topic)) return;
    intentSynced = true;
    // core/ros 已在本次连接的首帧状态恢复实际页面接线；进程状态仅用于告警。
    wantOverlay = routeIsOverlay();
  }

  function renderStatus() {
    syncIntentFromServer();
    const ff = unit();
    const state = ff ? (ff.state || 'stopped') : 'stopped';
    const label = stateLabels[state] || state;
    const config = ff && ff.config ? ff.config : null;

    // 勾选框画的是**意图**，不是回执——回执对不上由告警说，不让勾选框自己宣布成功。
    ui.route.checked = wantOverlay;
    // 关着的时候节点不该在跑（关 = 切回直连 + 停节点）；开着而节点不在＝断链，由告警说。
    ui.revert.hidden = !(brokenRoute() || routeNotSwitched() || routeNotReverted());
    ui.summary.textContent = wantOverlay
      ? (state === 'running' ? (routeNotSwitched() ? '接线没切过去' : '经前馈') : '断链')
      : (commandRouteIsOverlay() ? '接线没切回来'
        : (state === 'running' ? '节点在跑·未接线' : '直连'));
    ui.summary.className = `sum${brokenRoute() || routeNotSwitched()
      || routeNotReverted() ? ' err' : (wantOverlay ? ' ok' : '')}`;

    const parts = [wantOverlay ? '下行经 /robot_dynamics/*' : '下行直连固件'];
    if (commandTopic() !== null) {
      parts.push(`命令实际发往 ${commandTopic()}`);
    } else {
      parts.push('命令落点未知（command_streamer 未上报）');
    }
    if (config) {
      parts.push(`${config.model} · 增益 ${config.effort_gain}`);
    }
    parts.push(`叠加节点${label}`);
    if (state === 'failed' && ff.returncode !== null && ff.returncode !== undefined) {
      parts.push(`exit ${ff.returncode}`);
    }
    if (wantOverlay) {
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
      ui.hint.textContent = '重力补偿要 STM32 在环：叠加节点的输出由 bridge 转发，'
        + '纯 Gazebo / 纯 MuJoCo / 主机驱动闭环三路 bridge 之后没有固件消费。';
      return;
    }
    if (brokenRoute()) {
      ui.hint.textContent = '节点不在时命令会全部落空：先点「立即回退」把下行切回直连，'
        + '再从日志（仿真屏）查它为什么退出。';
      return;
    }
    ui.hint.textContent = wantOverlay
      ? '本页面发的命令与轨迹都经叠加节点加 τ_g 后转发给固件。'
        + '其它命令源（snapshot_stream.py / traj_play.py）不经这里，同时跑就是混流。'
        + '（command_streamer 是页面下行的唯一通道，本就该经本节点。）'
      : '填好模型与增益后打开开关：服务会先起叠加节点并确认它 running，再把命令流改道到'
        + '/robot_dynamics/*（轨迹那一条由本页面自己切）。关闭时反过来先切回直连、再停节点。';
  }

  function renderAll() {
    renderStatus();
    renderHint();
  }

  // 起：服务端在一个服务里做完两件事——起 unit、确认 running，再把命令流改道到叠加
  // 节点（ADR 0013 决定 7 的顺序；ADR 0014 决定 3 把改道挪到了服务端）。
  // 页面只剩**轨迹**那一条要自己切。
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
    wantOverlay = true;
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
      // 服务失败＝unit 没起来，接线也就没动过（改道排在起 unit 之后）。把意图拨回去。
      wantOverlay = false;
      setResult(`开启失败：${error.message}`, 'err');
    } finally {
      busy = false;
      renderAll();
    }
  }

  // 关：页面自己那条（轨迹）先切回直连；命令那条由服务端在停 unit **之前**切。
  // 顺序反过来，命令会在「接线还指着叠加节点、节点已经没了」的窗口里全部落空。
  async function turnOff() {
    busy = true;
    wantOverlay = false;
    renderStatus();
    if (routeIsOverlay()) setDownlinkRoute(DOWNLINK_ROUTE.DIRECT);
    setResult('正在停止叠加节点…');
    try {
      await bench.stopGravityFeedforward();
      setResult('下行已切回直连，叠加节点已停止。', 'ok');
    } catch (error) {
      // 服务失败可能是「改道没切回来，所以没敢停 unit」（见 bench_manager）——
      // 那正是要摆明说的状态，别把开关画成关着。
      setResult(`停止失败：${error.message}`, 'err');
    } finally {
      busy = false;
      renderAll();
    }
  }

  // 告警上的一键回退。命令那一条只能经服务端切（页面发的是设定值，不碰命令话题），
  // 而「切回直连」的通道就是 stop 服务——它先改道、确认成功再停 unit，正是这里要的顺序。
  async function revertToDirect() {
    busy = true;
    wantOverlay = false;
    renderStatus();
    if (routeIsOverlay()) setDownlinkRoute(DOWNLINK_ROUTE.DIRECT);
    try {
      await bench.stopGravityFeedforward();
      setResult('命令已切回直连固件，叠加节点已停止。', 'ok');
    } catch (error) {
      setResult(`回退失败：${error.message}`, 'err');
    } finally {
      busy = false;
      renderAll();
    }
  }

  async function applyGain() {
    let gain;
    try {
      gain = readGain();
    } catch (error) {
      setResult(error.message, 'err');
      return;
    }
    if (!wantOverlay || (unit() || {}).state !== 'running') {
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
    if (wantOverlay) renderStatus();
  }, REFRESH_MS);

  return {
    render(dirty) {
      // COMMAND 也要：命令**实际发往哪里**来自 /motor_command/status，改道后它才变。
      // 漏了它，服务端切完接线页面不会重绘，只能等下一帧 BENCH（1 Hz）。
      if (dirty.has(DIRTY.BENCH) || dirty.has(DIRTY.CONN)
          || dirty.has(DIRTY.COMMAND)) {
        renderAll();
      }
    },
    unmount() {
      if (refreshTimer !== null) {
        clearInterval(refreshTimer);
        refreshTimer = null;
      }
    },
  };
}
