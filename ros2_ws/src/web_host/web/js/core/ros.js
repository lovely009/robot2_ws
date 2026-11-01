// rosbridge 层：连接、订阅、发布与「等一帧/等序号」工具。
// 数据写入 store 后 markDirty，由面板渲染；连接状态同步到 body.offline 以禁用写操作。
import {
  store, markDirty, DIRTY, chartPush, applyBenchStatus, applyCommandStatus,
  jointFeedbackMatchesModel,
} from './store.js';
import {
  $, TOPICS, EVENT_NAMES, MOTOR_ID_NONE, quatToEuler,
} from './format.js';

import { applyControlState, createControlStateCache } from './motor_state.js';
import { ManagedControl } from './managed_control.js';

let controlMode = 'managed';
export function isManagedMode() { return controlMode === 'managed'; }
const managedPublishers = {};
const managed = new ManagedControl({
  service: (...args) => callService(...args),
  publish: (name, messageType, message) => {
    if (!managedPublishers[name]) {
      managedPublishers[name] = new ROSLIB.Topic({ros, name, messageType});
      managedPublishers[name].advertise();
    }
    managedPublishers[name].publish(new ROSLIB.Message(message));
  },
  changed: () => {
    store.managed = {enabled: isManagedMode(), ...managed.view()};
    markDirty(DIRTY.CONN, DIRTY.STATUS);
  },
});
store.managed = {enabled: true, ...managed.view()};
let visibilityInstalled = false;
export function configureControl(configuration) {
  if (!['managed', 'readonly', 'bench'].includes(configuration?.control_mode)) throw new Error('工作台运行模式无效');
  controlMode = configuration.control_mode;
  managed.model = configuration.robot_model || '';
  store.managed = {enabled: isManagedMode(), ...managed.view(), robotId: configuration.robot_id || ''};
  document.body.classList.toggle('managed-mode', isManagedMode());
  if (!visibilityInstalled && document.addEventListener) {
    visibilityInstalled = true;
    document.addEventListener('visibilitychange', () => {
      if (document.hidden && isManagedMode()) connectionGeneration++;
      managed.setVisible(!document.hidden);
    });
    globalThis.addEventListener?.('pagehide', () => {
      if (isManagedMode()) connectionGeneration++;
      managed.suspend('页面已退出', true);
    });
  }
  managed.setVisible(!document.hidden);
}
export async function managedOperation(operation) {
  if (!isManagedMode()) return false;
  try {
    if (operation === 'arm') await managed.arm(true);
    else if (operation === 'disarm') await managed.arm(false);
    else if (['acquire', 'release', 'stop', 'reset'].includes(operation)) await managed[operation]();
    else throw new Error('未知整机操作');
    return true;
  } catch (error) {
    managed.message = String(error.message || error); managed.changed(); return false;
  }
}

let ros = null;
let connectionGeneration = 0;
export function getConnectionGeneration() { return connectionGeneration; }
let subs = [];
const publishers = {};
const CONFIG_RESULT_LIMIT = 64;
const configWaiters = new Map();
let configSessionId = null;

function settleConfigWaiters(requestId, value) {
  const waiters = configWaiters.get(requestId);
  if (!waiters) return;
  configWaiters.delete(requestId);
  for (const waiter of waiters) waiter.finish(value);
}

function clearConfigResults(outcome = 'disconnected') {
  store.configResults.clear();
  store.configSessionId = null;
  configSessionId = null;
  for (const requestId of [...configWaiters.keys()]) {
    settleConfigWaiters(requestId, {outcome, request_id: requestId});
  }
}

function ingestConfigResult(message) {
  const sessionId = String(message.session_id);
  if (configSessionId !== null && configSessionId !== sessionId) {
    clearConfigResults('session_changed');
  }
  configSessionId = sessionId;
  store.configSessionId = sessionId;
  const result = {...message};
  const requestId = Number(message.request_id);
  if (!Number.isInteger(requestId) || requestId <= 0) return;
  const previous = store.configResults.get(requestId);
  if (previous && Number(previous.status) !== 0 && Number(message.status) === 0) return;
  store.configResults.delete(requestId);
  store.configResults.set(requestId, result);
  while (store.configResults.size > CONFIG_RESULT_LIMIT) {
    store.configResults.delete(store.configResults.keys().next().value);
  }
  if (Number(message.status) !== 0) {
    settleConfigWaiters(requestId, {outcome: 'terminal', request_id: requestId, result});
  }
}

export function waitConfigResult(requestId, timeoutMs) {
  const id = Number(requestId);
  if (!Number.isInteger(id) || id <= 0) {
    return Promise.resolve({outcome: 'invalid_request', request_id: requestId});
  }
  const cached = store.configResults.get(id);
  if (cached && Number(cached.status) !== 0) {
    return Promise.resolve({outcome: 'terminal', request_id: id, result: cached});
  }
  if (!store.conn.connected) return Promise.resolve({outcome: 'disconnected', request_id: id});
  const generation = connectionGeneration;
  return new Promise((resolve) => {
    let settled = false;
    const finish = (value) => {
      if (settled) return;
      settled = true;
      clearTimeout(timer);
      const waiters = configWaiters.get(id);
      if (waiters) {
        waiters.delete(waiter);
        if (!waiters.size) configWaiters.delete(id);
      }
      resolve(value);
    };
    const waiter = {finish};
    const timer = setTimeout(() => finish({outcome: 'timeout', request_id: id}), timeoutMs);
    if (!configWaiters.has(id)) configWaiters.set(id, new Set());
    configWaiters.get(id).add(waiter);
    if (generation !== connectionGeneration || !store.conn.connected) {
      finish({outcome: 'disconnected', request_id: id});
    }
  });
}

export function setConn(ok, text) {
  store.conn.connected = ok;
  store.conn.text = text || (ok ? '已连接' : '未连接');
  document.body.classList.toggle('offline', !ok);
  markDirty(DIRTY.CONN);
}

export function isConnected() { return store.conn.connected; }

// 策略代数：跨面板共享的单调计数器（使能屏与动作库的「武装」共用同一序列）。
// 固件的代数门是**跨页面持久**的：表项进 FAULT/OFFLINE/CONFIG_CONFLICT 时把当时
// 的代数锁存下来，之后「enabled=true 且代数 <= 锁存值」一律拒绝（见
// MotorPolicyArray.msg）。所以计数器不能从 0 起——刷新页面只把这一侧归零，固件
// 那边记得的仍是上一轮的值，于是每一条「武装」都被静默拒（只有事件日志的
// CONFIG_RESULT reason=2 能看出），表现为"播放完成但机构不动"。用启动时间垫高
// 起点；判"是不是新意图"看的是它是否前进，而不是它等于几。
let policyGeneration = (Date.now() & 0x7fffffff) >>> 0;

export function nextPolicyGeneration() {
  policyGeneration = (policyGeneration + 1) >>> 0;
  return policyGeneration;
}

export function pub(key, message) {
  if (!store.conn.connected) return false;
  if (controlMode === 'readonly') return false;
  if (isManagedMode()) {
    if (key === 'stop') { void managedOperation('stop'); return true; }
    if (key === 'jointTrajectory') {
      if (!managed.canSubmitMotion()) {
        managed.message = '请先取得 MCU 授权，并等待当前轨迹结束或停止'; managed.changed();
        return false;
      }
      void managed.sendJointTrajectory(message).catch(error => {
        managed.message = String(error.message || error); managed.changed();
      });
      return true;
    }
    const channel = {setpoint: 'command', policy: 'policy', trajectory: 'trajectory'}[key];
    if (!channel) {
      managed.message = '托管模式不开放此维护或旁路写入口'; managed.changed(); return false;
    }
    return managed.send(channel, message);
  }
  ensurePublisher(key).publish(new ROSLIB.Message(message));
  return true;
}

// rosbridge 的 advertise 是异步 WebSocket 操作。若在第一次 advertise 后
// 立刻 publish，低频的一次性 command（尤其是动作前的授权快照）可能在
// micro-ROS 订阅端完成发现前丢掉；随后 trajectory_state 仍会回执，造成
// “轨迹已完成但没有运动帧”的假成功。连接建立时先把三个下行发布端注册好，
// 后续按钮/动作只发送已经存在的 publisher。
export function ensurePublisher(key) {
  if (controlMode === 'readonly') return null;
  if (isManagedMode()) {
    const channel = {setpoint: 'command', policy: 'policy', trajectory: 'trajectory'}[key];
    if (!channel) return null;
    const name = '/robot/control/' + channel;
    if (!managedPublishers[name]) {
      managedPublishers[name] = new ROSLIB.Topic({ros, name,
        messageType: 'robot_system_msgs/msg/Leased' + channel[0].toUpperCase() + channel.slice(1)});
      managedPublishers[name].advertise();
    }
    return managedPublishers[name];
  }
  if (!publishers[key]) {
    const t = TOPICS[key];
    publishers[key] = new ROSLIB.Topic({ros, name: t.name, messageType: t.type});
    publishers[key].advertise();
  }
  return publishers[key];
}

// roslibjs 不自动重连；连接中断后按 2 s 退避重试。
let reconnectTimer = null;

function resetSessionState() {
  clearConfigResults();
  store.motors.clear();
  store.controlStates = createControlStateCache();
  store.paramCache.clear();
  store.tf.clear();
  store.sysStatus.seen = false;
  store.firmwareReport = {seq: 0, at: 0, values: null, unsaved: false};
  store.conn.autoLoaded = false;
  store.conn.benchLoaded = false;
  for (const key of Object.keys(store.bench)) store.bench[key] = null;
  store.conn.agent = 'stopped';
  store.conn.agentDevice = '';
  store.conn.canStop = false;
  store.commandStatus = {last: 0, data: null};
  store.traj.state = null;
  store.traj.activeTrajectoryId = 0;
  store.jointStates.byName = {};
  store.jointStates.jointNames = [];
  store.transmissionStatus = {last: 0, data: null};
  markDirty(DIRTY.TOPOLOGY, DIRTY.STATUS, DIRTY.PARAMS, DIRTY.TF, DIRTY.BENCH,
    DIRTY.COMMAND, DIRTY.MOTION);
}

export function disconnect() {
  if (isManagedMode()) managed.suspend('已断开，控制权已释放', true);
  if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null; }
  closeSocket();
  resetSessionState();
  setConn(false, '已断开');
}

export function connect() {
  if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null; }
  closeSocket();
  resetSessionState();
  setConn(false, '正在连接…');
  const url = $('rosUrl').value.trim();
  try {
    ros = new ROSLIB.Ros({url});
  } catch (e) {
    setConn(false, 'URL 无效');
    return;
  }
  const connection = ros;
  ros.on('connection', () => {
    if (ros !== connection) return;
    setConn(true);
    managed.setConnected(true);
    subscribeAll();
    (isManagedMode() ? ['setpoint', 'trajectory', 'policy']
      : ['setpoint', 'trajectory', 'stop', 'policy', 'paramSet', 'sysCmd']).forEach(ensurePublisher);
    // 连接后自动读取参数表（REPORT → device_param_report）随设备屏接入——
    // loadParams 归 core/device.js，这里只负责建立连接与订阅。
  });
  ros.on('error', () => {
    if (ros === connection) {
      connectionGeneration++;
      clearConfigResults();
      managed.setConnected(false);
      setConn(false, '连接错误');
    }
  });
  ros.on('close', () => {
    if (ros !== connection) return;
    closeSocket();
    resetSessionState();
    // !reconnectTimer：同一时刻只允许一个待触发的重连（否则会叠出重连风暴，
    // 表现为页面一直连不上、看着"卡"）。
    if (!reconnectTimer) {
      setConn(false, '已断开，2s 后重连…');
      reconnectTimer = setTimeout(() => { reconnectTimer = null; connect(); }, 2000);
    }
  });
}

// 只关 socket，不动连接意图（自动重连要复用）。
// 连发/曲线等定时器由各自面板在连接态脏帧里自行停止；本层不反向调用面板。
export function closeSocket() {
  managed.setConnected(false);
  connectionGeneration++;
  clearConfigResults();
  if (ros) { const r = ros; ros = null; try { r.close(); } catch (e) {} }
  subs = [];
  Object.values(publishers).forEach((p) => { try { p.unadvertise(); } catch (e) {} });
  Object.keys(publishers).forEach((k) => delete publishers[k]);
  Object.values(managedPublishers).forEach(p => { try { p.unadvertise(); } catch (_e) {} });
  Object.keys(managedPublishers).forEach(k => delete managedPublishers[k]);
}

function sub(key, cb) {
  const connection = ros;
  const t = TOPICS[key];
  const topic = new ROSLIB.Topic({
    ros, name: t.name, messageType: t.type, throttle_rate: 0, queue_length: key === 'events' ? 64 : 1,
  });
  topic.subscribe((message) => { if (ros === connection && store.conn.connected) cb(message); });
  subs.push(topic);
}

function subscribeAll() {
  subs.forEach((s) => s.unsubscribe());
  subs = [];
  if (isManagedMode()) {
    const current = ros;
    const topic = new ROSLIB.Topic({ros, name: '/robot/state', messageType: 'robot_system_msgs/msg/RobotState',
      qos: {reliability: 'reliable', durability: 'transient_local', history: 'keep_last', depth: 1}});
    topic.subscribe(state => {
      if (ros === current && store.conn.connected) managed.update(state);
    });
    subs.push(topic);
  }

  // 桥心跳与普通周期消息不同：late subscriber 必须拿到最后一帧，随后按 1 Hz 检查新鲜度。
  const statusConnection = ros;
  const statusTopic = new ROSLIB.Topic({
    ros,
    name: TOPICS.robotTransmissionStatus.name,
    messageType: TOPICS.robotTransmissionStatus.type,
    qos: {reliability: 'reliable', durability: 'transient_local', history: 'keep_last', depth: 1},
  });
  statusTopic.subscribe((message) => {
    if (ros !== statusConnection || !store.conn.connected) return;
    try {
      const data = JSON.parse(String(message && message.data));
      if (!data || typeof data.model !== 'string' || !Array.isArray(data.joint_names)
          || typeof data.trajectory_enabled !== 'boolean') throw new Error('字段无效');
      store.transmissionStatus = {last: Date.now(), data};
    } catch (_e) {
      // 坏帧不覆盖最近一次有效的桥状态，也不刷新其新鲜度。
      store.transmissionStatus.last = 0;
      store.transmissionStatus.data = null;
    }
    markDirty(DIRTY.STATUS);
  });
  subs.push(statusTopic);

  sub('topology', (m) => {
    const seen = new Set();
    for (const e of (m.entries || [])) {
      seen.add(e.motor_id);
      const row = store.motors.get(e.motor_id) || {};
      row.topo = e;
      store.motors.set(e.motor_id, row);
    }
    // 拓扑消失的电机移出表（复位/换拓扑）。
    for (const id of [...store.motors.keys()]) {
      if (!seen.has(id)) store.motors.delete(id);
    }
    markDirty(DIRTY.TOPOLOGY);
  });

  sub('motion', (m) => {
    store.motionCount++;
    const t = Date.now();
    for (const s of (m.states || [])) {
      const row = store.motors.get(s.motor_id) || {};
      row.motion = s;
      store.motors.set(s.motor_id, row);
      chartPush(s.motor_id, 'position', s.position, t);
      chartPush(s.motor_id, 'velocity', s.velocity, t);
      chartPush(s.motor_id, 'effort', s.effort, t);
    }
    // 臂视图的关节角与曲线面板都从这里取数（经 store.motors）。
    markDirty(DIRTY.MOTION);
  });

  sub('controlStates', (m) => {
    const previousSession = store.controlStates.session;
    const accepted = applyControlState(store.controlStates,m);
    if (accepted) {
      const currentSession = String(store.controlStates.session);
      if (previousSession !== 0 && store.controlStates.session !== previousSession) {
        clearConfigResults('session_changed');
        configSessionId = currentSession;
        store.configSessionId = currentSession;
      } else if (previousSession === 0 && configSessionId === null) {
        configSessionId = currentSession;
        store.configSessionId = currentSession;
      }
    }
    markDirty(DIRTY.STATUS);
  });

  sub('status', (m) => {
    const t = Date.now();
    for (const s of (m.statuses || [])) {
      const row = store.motors.get(s.motor_id) || {};
      row.status = s;
      row.statusAt = t;
      store.motors.set(s.motor_id, row);
      chartPush(s.motor_id, 'current', s.current, t);
    }
    markDirty(DIRTY.STATUS);
  });

  // 客户端将 JTC 结果映射成轨迹状态；completed 只在 action 成功终态为真。
  sub('trajectoryState', (m) => {
    store.traj.activeTrajectoryId = m.trajectory_id;
    store.traj.state = {
      trajectory_id: m.trajectory_id,
      active: !!m.active,
      completed: !!m.completed,
      elapsed_ms: m.elapsed_ms,
      total_ms: m.total_ms,
      target_ids: m.target_ids || [],
    };
    markDirty(DIRTY.TRAJ);
  });

  sub('events', (m) => {
    if (m.event_type === 13) {
      const policy = store.policy.get(m.motor_id);
      if (policy && policy.requestId === m.request_id) {
        policy.receipt = m.reason;
        markDirty(DIRTY.POLICY, DIRTY.STATUS);
      }
    }
    store.events.unshift({
      t: Date.now(),
      name: EVENT_NAMES[m.event_type] !== undefined
        ? EVENT_NAMES[m.event_type] : 'EVENT_' + m.event_type,
      id: m.motor_id,
      reason: m.reason,
      request_id: m.request_id || 0,
    });
    if (store.events.length > 2000) store.events.pop();
    // DEV_INIT(type 15)：记下核对结论与"哪几项对不上"（request_id = 槽位位图）。
    if (m.event_type === 15) {
      store.devInit.set(m.motor_id, {
        state: m.reason,
        mask: m.request_id || 0,
      });
    }
    markDirty(DIRTY.EVENTS, DIRTY.PARAMS);
  });

  // TF（关节侧）：child frame → 帧对与变换；关节列表据此配对模型 URDF 的关节。
  // /tf_static 只在启动时发一次，收到即留在表里，不按新鲜度判它陈旧。
  const ingestTf = (m, dynamic) => {
    for (const tr of (m.transforms || [])) {
      if (!tr.child_frame_id || !tr.transform) continue;
      const prev = store.tf.get(tr.child_frame_id);
      store.tf.set(tr.child_frame_id, {
        parent: tr.header ? tr.header.frame_id : '',
        xyz: tr.transform.translation,
        quat: tr.transform.rotation,
        dynamic,
        at: Date.now(),
        count: (prev ? prev.count : 0) + 1,
      });
    }
    markDirty(DIRTY.TF);
  };
  sub('tf', (m) => ingestTf(m, true));
  sub('tfStatic', (m) => ingestTf(m, false));

  sub('imu', (m) => {
    const q = m.orientation;
    const e = quatToEuler(q.x, q.y, q.z, q.w);
    store.imu.roll = e.roll;
    store.imu.pitch = e.pitch;
    store.imu.yaw = e.yaw;
    store.imu.q = q;
    store.imu.count++;
    store.imu.last = Date.now();
    markDirty(DIRTY.IMU);
  });

  sub('temp', (m) => {
    store.temp = m.data;
    store.tempLast = Date.now();
    markDirty(DIRTY.IMU);
  });

  // 主机控制栈的实测关节反馈，按模型身份与新鲜度接收。
  sub('jointStates', (m) => {
    if (!jointFeedbackMatchesModel()) return;
    const now = Date.now();
    const names = Array.isArray(m && m.name) ? m.name : [];
    const positions = Array.isArray(m && m.position) ? m.position : [];
    for (let i = 0; i < names.length; i++) {
      const position = Number(positions[i]);
      if (typeof names[i] === 'string' && Number.isFinite(position)) {
        store.jointStates.byName[names[i]] = {
          position,
          velocity: Number(m.velocity?.[i]),
          effort: Number(m.effort?.[i]),
          at: now,
        };
      }
    }
    if (names.length) store.jointStates.jointNames = [
      ...new Set(names.filter((name) => typeof name === 'string'))];
    store.jointStates.count++;
    store.jointStates.last = now;
    markDirty(DIRTY.MOTION);
  });

  // 设备参数报告（reliable，按需：REPORT 命令 / 写事务收尾 / 设备初始化结论）：
  // 一帧带全部台的**双行**——rec_* 固件记录（权威）+ dev_* 驱动板回读。
  // 表里显示 dev_*（"优先显示驱动板"），该台没回读到（NaN，含电机不在线）时
  // 才退回 rec_*（即板内参数表）。写事务收尾后固件即时发一帧，所以
  // 「写 → 等报告变」就是这一侧的完成判据。
  sub('devParams', (m) => {
    store.devParamSeq++;
    // count 是 entries.size 的冗余声明（同 MotorPolicyArray.count）；不认它
    // 会在固件侧 count 与数组长度不一致时读到 0 条。
    const n = Math.min(m.count === undefined ? 0 : m.count, (m.entries || []).length);
    for (let i = 0; i < n; ++i) {
      const e = m.entries[i];
      // 保护开关要**反转**：报文字段 rec_safety/dev_safety 是设备原值
      // （0x2650:00 读回字节，**0 = 保护开**），而参数面 m<id>.limit_protect 与
      // 本页这一格 checkbox 用的都是正向语义（1 = 保护生效）——两者相反。
      // 契约见 DeviceParamEntry.msg；固件侧是原值透传（cerb_adapter.c 的
      // CERB_VALUE_SAFETY_RAW），不改。两行一起转，rec/dev 仍彼此可比
      // （rowsHint 拿它们对账），也仍与写回去的 field 同口径。
      // 未回读到该对象时保持 NaN——不能落 0，那会被读成"保护关"（危险方向）。
      const safety = (v) => (Number.isFinite(v) ? (v === 0 ? 1 : 0) : NaN);
      const rec = {pos_min: e.rec_pos_min, pos_max: e.rec_pos_max,
        vel_limit: e.rec_vel_limit, torque_limit: e.rec_torque_limit,
        limit_protect: safety(e.rec_safety), direction: e.rec_direction, t_max: e.rec_t_max, node_id: e.rec_node_id};
      const dev = {pos_min: e.dev_pos_min, pos_max: e.dev_pos_max,
        vel_limit: e.dev_vel_limit, torque_limit: e.dev_torque_limit,
        limit_protect: safety(e.dev_safety), direction: e.dev_direction, t_max: e.dev_t_max, node_id: e.dev_node_id};
      const pick = {};
      for (const f of Object.keys(rec)) {
        pick[f] = Number.isFinite(dev[f]) ? dev[f] : rec[f];
      }
      store.paramCache.set(e.motor_id, {rec, dev, pick,
        state: e.dev_init_state, dirty: e.record_dirty, mask: e.mismatch_mask});
    }
    // 重绘与否是面板的事：有未下发的手改时跳过重绘（否则会把操作员正在敲的
    // 值抹掉）——设备屏面板在收到 DIRTY.PARAMS 时自行判断。
    markDirty(DIRTY.PARAMS);
  });

  // 系统状态（best-effort，5 Hz + 变化即发）。这是**电平**不是事件：last_error
  // 保持到下次错误，*_seq 仅供兼容观察，不关联具体配置请求。
  sub('sysStatus', (m) => {
    if (m.firmware_values_valid && Array.isArray(m.firmware_values) && m.firmware_values.length === 6 &&
        m.firmware_values.every(v => Number.isInteger(v) && v > 0 && v <= 1000)) {
      store.firmwareReport = {seq: store.firmwareReport.seq + 1, at: Date.now(),
        values: [...m.firmware_values], unsaved: !!m.firmware_unsaved};
    }
    store.lastError = m.last_error;
    store.lastErrorMotor = m.last_error_motor === MOTOR_ID_NONE ? -1 : m.last_error_motor;
    store.sysStatus = {
      refresh_seq: m.refresh_seq, set_zero_seq: m.set_zero_seq,
      clear_fault_seq: m.clear_fault_seq, save_seq: m.save_seq,
      save_in_progress: !!m.save_in_progress, seen: true,
    };
  });

  sub('configResult', (m) => ingestConfigResult(m));

  // 台架状态（bench_manager，JSON 字符串）：仿真 / Agent / 电机后端三段的唯一
  // 数据入口（命令的服务响应也经 applyBenchStatus 写同一份）。
  if (!isManagedMode()) sub('bench', (m) => {
    try { applyBenchStatus(JSON.parse(m.data)); }
    catch (_e) { /* 非法帧不入 store；下一帧会重来 */ }
  });

  // ros2_control 模型、模式、路由与错误。
  sub('commandStatus', (m) => {
    try { applyCommandStatus(JSON.parse(m.data)); }
    catch (_e) { applyCommandStatus(null); }
  });
}

// 调 ROS 服务（bench_manager 的 /bench/*）。服务不存在/节点未起时，roslibjs 的
// 失败回调行为不统一，所以恒有超时兜底；resolve 出来的是响应对象（是否成功看
// 调用方——bench 服务的 ok/error 字段由 core/bench.js 统一裁决）。
export function callService(name, type, request, timeoutMs = 8000) {
  return new Promise((resolve, reject) => {
    if (controlMode === 'readonly') {
      reject(new Error('只读页面不开放控制或维护服务')); return;
    }
    if (isManagedMode() && !name.startsWith('/robot/')) {
      reject(new Error('托管模式不开放台架管理或维护写操作')); return;
    }
    if (!ros || !store.conn.connected) {
      reject(new Error('未连接 rosbridge'));
      return;
    }
    const connection = ros;
    const service = new ROSLIB.Service({ros, name, serviceType: type});
    let settled = false;
    const timer = setTimeout(() => {
      if (!settled) { settled = true; reject(new Error(`服务无响应：${name}`)); }
    }, timeoutMs);
    service.callService(new ROSLIB.Message(request || {}), (result) => {
      if (settled) return;
      settled = true;
      clearTimeout(timer);
      if (ros !== connection) { reject(new Error('连接已更换')); return; }
      resolve(result);
    }, (error) => {
      if (settled) return;
      settled = true;
      clearTimeout(timer);
      reject(new Error(typeof error === 'string'
        ? error : `服务调用失败：${name}`));
    });
  });
}

// 等首帧 system_status 到达（连接后参数自动读取的前置，Task 8 用）。
export function waitSysStatus(timeoutMs = 2000) {
  if (store.sysStatus.seen) return Promise.resolve(true);
  return new Promise((resolve) => {
    const deadline = Date.now() + timeoutMs;
    const timer = setInterval(() => {
      if (store.sysStatus.seen) { clearInterval(timer); resolve(true); }
      else if (Date.now() >= deadline) { clearInterval(timer); resolve(false); }
    }, 50);
  });
}

// 等下一帧 device_param_report（帧计数变大）。无完成序号可用，REPORT/写事务
// 都靠它确认帧到了。
export async function waitDevParamSeq(baseline, timeoutMs) {
  const deadline = Date.now() + timeoutMs;
  while (Date.now() < deadline) {
    if (store.devParamSeq > baseline) return 0;
    if (!store.conn.connected) return -2;
    await new Promise((r) => setTimeout(r, 100));
  }
  return -3;
}
