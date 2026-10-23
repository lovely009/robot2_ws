// rosbridge 层：连接、订阅、发布与「等一帧/等序号」工具。
// 搬迁自重构前的 app.js（rosbridge 段 127-505 + chartPush），四处适配：
//   state → store；写入 store 后 markDirty；撤掉全部 renderXxx() 调用
//   （渲染是面板的职责，本层不再认识任何 DOM）；setConn 改写为写 store +
//   切 body.offline（断连禁用写操作的样式钩子）。
import {
  store, markDirty, DIRTY, chartPush, applyBenchStatus, applyCommandStatus,
} from './store.js';
import {
  $, TOPICS, DOWNLINK_ROUTE, EVENT_NAMES, MOTOR_ID_NONE,
  OVERLAY_TRAJECTORY_TOPIC, quatToEuler,
} from './format.js';

let ros = null;
let connectionGeneration = 0;
export function getConnectionGeneration() { return connectionGeneration; }
let subs = [];
const publishers = {};

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
  ensurePublisher(key).publish(new ROSLIB.Message(message));
  return true;
}

// rosbridge 的 advertise 是异步 WebSocket 操作。若在第一次 advertise 后
// 立刻 publish，低频的一次性 command（尤其是动作前的授权快照）可能在
// micro-ROS 订阅端完成发现前丢掉；随后 trajectory_state 仍会回执，造成
// “轨迹已完成但没有运动帧”的假成功。连接建立时先把三个下行发布端注册好，
// 后续按钮/动作只发送已经存在的 publisher。
export function ensurePublisher(key) {
  if (!publishers[key]) {
    const t = TOPICS[key];
    publishers[key] = new ROSLIB.Topic({ros, name: t.name, messageType: t.type});
    publishers[key].advertise();
  }
  return publishers[key];
}

// ---- 下行接线：直连固件 ↔ 经重力前馈叠加（ADR 0013）------------------------
// 切换是「改话题名 + 换发布者」一件事的两半：TOPICS 里那两个 name 是运行期可变的，
// 而发布者在连接建立时就按旧名字 advertise 好了，只改 name 会让命令继续发到旧话题上
// （rosbridge 侧一切正常，micro-ROS 侧没人收）。所以唯一改写点在这里，且两半一起做。
//
// 直连名以 TOPICS 里声明的为准；叠加名是叠加节点的输入话题（robot_dynamics 契约）。
// 只列**页面自己发**的下行通道。命令不在其中：页面发的是设定值（固定话题），
// 命令从哪条路出去由 bench_manager 改 command_streamer 的 output_topic 决定
// （ADR 0014 决定 3），页面切不了、只能读 TOPICS.commandStatus 核对。
const DOWNLINK = {
  trajectory: {[DOWNLINK_ROUTE.DIRECT]: TOPICS.trajectory.name,
    [DOWNLINK_ROUTE.OVERLAY]: OVERLAY_TRAJECTORY_TOPIC},
};
let downlinkRoute = DOWNLINK_ROUTE.DIRECT;

export function getDownlinkRoute() { return downlinkRoute; }

export function setDownlinkRoute(next) {
  if (next !== DOWNLINK_ROUTE.DIRECT && next !== DOWNLINK_ROUTE.OVERLAY) {
    throw new Error(`未知的下行接线：${next}`);
  }
  if (next === downlinkRoute) return downlinkRoute;
  downlinkRoute = next;
  for (const key of Object.keys(DOWNLINK)) TOPICS[key].name = DOWNLINK[key][next];
  // 未连接时没有发布者要换（closeSocket 已清空），重连时 connect() 会按新名字建。
  if (store.conn.connected) {
    for (const key of Object.keys(DOWNLINK)) {
      const old = publishers[key];
      if (old) { try { old.unadvertise(); } catch (_e) { /* 连接已断，无需回收 */ } }
      delete publishers[key];
      // 立刻在新名字上 advertise，理由同上：等第一次 publish 才建会丢掉开头的命令。
      ensurePublisher(key);
    }
  }
  markDirty(DIRTY.CONN);
  return downlinkRoute;
}

// 自动重连：roslibjs 自身**不重连**。rosbridge 一重启（改 msg、改 launch 都会重启），
// 页面就静默变哑——表还在（ID 来自参数缓存），但型号/生命周期/位置全空，看起来像
// "固件没数据"。第一次踩这个坑花了不少时间，所以默认按 2s 退避自动重连；
let reconnectTimer = null;

function resetSessionState() {
  store.motors.clear();
  store.paramCache.clear();
  store.tf.clear();
  store.sysStatus.seen = false;
  store.conn.autoLoaded = false;
  store.conn.benchLoaded = false;
  for (const key of Object.keys(store.bench)) store.bench[key] = null;
  store.conn.agent = 'stopped';
  store.conn.agentDevice = '';
  store.conn.canStop = false;
  store.commandStatus = {last: 0, data: null};
  markDirty(DIRTY.TOPOLOGY, DIRTY.STATUS, DIRTY.PARAMS, DIRTY.TF, DIRTY.BENCH, DIRTY.COMMAND);
}

export function disconnect() {
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
    subscribeAll();
    // 重连后按**当前**接线重建发布者（setDownlinkRoute 只改了 TOPICS 与内存里的发布者，
    // 断线时那份已被 closeSocket 清掉）。开关的落点不带状态，重连不会悄悄切回直连。
    ['setpoint', 'trajectory', 'policy', 'paramSet', 'sysCmd'].forEach(ensurePublisher);
    // 连接后自动读取参数表（REPORT → device_param_report）随设备屏接入——
    // loadParams 归 core/device.js，这里只负责建立连接与订阅。
  });
  ros.on('error', () => {
    if (ros === connection) { connectionGeneration++; setConn(false, '连接错误'); }
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
  connectionGeneration++;
  if (ros) { const r = ros; ros = null; try { r.close(); } catch (e) {} }
  subs = [];
  Object.values(publishers).forEach((p) => { try { p.unadvertise(); } catch (e) {} });
  Object.keys(publishers).forEach((k) => delete publishers[k]);
}

function sub(key, cb) {
  const connection = ros;
  const t = TOPICS[key];
  const topic = new ROSLIB.Topic({
    ros, name: t.name, messageType: t.type, throttle_rate: 0, queue_length: 1,
  });
  topic.subscribe((message) => { if (ros === connection && store.conn.connected) cb(message); });
  subs.push(topic);
}

function subscribeAll() {
  subs.forEach((s) => s.unsubscribe());
  subs = [];

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

  sub('status', (m) => {
    const t = Date.now();
    for (const s of (m.statuses || [])) {
      const row = store.motors.get(s.motor_id) || {};
      row.status = s;
      store.motors.set(s.motor_id, row);
      chartPush(s.motor_id, 'current', s.current, t);
    }
    markDirty(DIRTY.STATUS);
  });

  // 轨迹执行状态（低频）：动作屏的窗口交棒（elapsed_ms 到达窗口时长）与
  // 进度显示都读 store.traj.state；不在条目上做裁决，原样存最近一帧。
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

  // 关节侧实测构型（HIL 下由 firmware_topic_bridge 从 motion_states 转出，
  // 仿真下由 joint_state_broadcaster / MuJoCo 桥发布）。页面只用它的**新鲜度**：
  // 叠加节点拿不到它就整帧直通，于是「开关亮着但没有补偿」不留任何痕迹。
  sub('jointStates', (m) => {
    store.jointStates.count++;
    store.jointStates.last = Date.now();
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
        limit_protect: safety(e.rec_safety), direction: e.rec_direction};
      const dev = {pos_min: e.dev_pos_min, pos_max: e.dev_pos_max,
        vel_limit: e.dev_vel_limit, torque_limit: e.dev_torque_limit,
        limit_protect: safety(e.dev_safety), direction: e.dev_direction};
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
  // 保持到下次错误，四个 *_seq 是单调递增的完成计数（判完成 = 记下"发命令时读到
  // 的值"再等它变大）。
  sub('sysStatus', (m) => {
    store.lastError = m.last_error;
    store.lastErrorMotor = m.last_error_motor === MOTOR_ID_NONE ? -1 : m.last_error_motor;
    store.sysStatus = {
      refresh_seq: m.refresh_seq, set_zero_seq: m.set_zero_seq,
      clear_fault_seq: m.clear_fault_seq, save_seq: m.save_seq,
      save_in_progress: !!m.save_in_progress, seen: true,
    };
  });

  // 台架状态（bench_manager，JSON 字符串）：仿真 / Agent / 电机后端三段的唯一
  // 数据入口（命令的服务响应也经 applyBenchStatus 写同一份）。
  sub('bench', (m) => {
    try { applyBenchStatus(JSON.parse(m.data)); }
    catch (_e) { /* 非法帧不入 store；下一帧会重来 */ }
  });

  // 命令节拍节点的状态：命令**当前实际发往**哪里（output_topic）。重力补偿开关的
  // 改道是服务端做的，页面只有读它才知道接线有没有真的切过去。
  sub('commandStatus', (m) => {
    try { applyCommandStatus(JSON.parse(m.data)); }
    catch (_e) { /* 非法帧不入 store；下一帧会重来 */ }
  });
}

// 调 ROS 服务（bench_manager 的 /bench/*）。服务不存在/节点未起时，roslibjs 的
// 失败回调行为不统一，所以恒有超时兜底；resolve 出来的是响应对象（是否成功看
// 调用方——bench 服务的 ok/error 字段由 core/bench.js 统一裁决）。
export function callService(name, type, request, timeoutMs = 8000) {
  return new Promise((resolve, reject) => {
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
