// rosbridge 层：连接、订阅、发布与「等一帧/等序号」工具。
// 搬迁自重构前的 app.js（rosbridge 段 127-505 + chartPush），四处适配：
//   state → store；写入 store 后 markDirty；撤掉全部 renderXxx() 调用
//   （渲染是面板的职责，本层不再认识任何 DOM）；setConn 改写为写 store +
//   切 body.offline（断连禁用写操作的样式钩子）。
import { store, markDirty, DIRTY, chartPush } from './store.js';
import { $, TOPICS, EVENT_NAMES, MOTOR_ID_NONE, quatToEuler } from './format.js';

let ros = null;
let subs = [];
const publishers = {};

export function setConn(ok, text) {
  store.conn.connected = ok;
  store.conn.text = text || (ok ? '已连接' : '未连接');
  document.body.classList.toggle('offline', !ok);
  markDirty(DIRTY.CONN);
}

export function isConnected() { return store.conn.connected; }

export function pub(key, message) {
  if (!store.conn.connected) return false;
  ensurePublisher(key).publish(new ROSLIB.Message(message));
  return true;
}

// rosbridge 的 advertise 是异步 WebSocket 操作。若在第一次 advertise 后
// 立刻 publish，低频的一次性 command（尤其是示教前的授权快照）可能在
// micro-ROS 订阅端完成发现前丢掉；随后 trajectory_state 仍会回执，造成
// “轨迹已完成但没有运动帧”的假成功。连接建立时先把三个下行发布端注册好，
// 后续按钮/示教只发送已经存在的 publisher。
export function ensurePublisher(key) {
  if (!publishers[key]) {
    const t = TOPICS[key];
    publishers[key] = new ROSLIB.Topic({ros, name: t.name, messageType: t.type});
    publishers[key].advertise();
  }
  return publishers[key];
}

// 自动重连：roslibjs 自身**不重连**。rosbridge 一重启（改 msg、改 launch 都会重启），
// 页面就静默变哑——表还在（ID 来自参数缓存），但型号/生命周期/位置全空，看起来像
// "固件没数据"。第一次踩这个坑花了不少时间，所以默认按 2s 退避自动重连；
let reconnectTimer = null;

export function connect() {
  if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null; }
  closeSocket();
  const url = $('rosUrl').value.trim();
  try {
    ros = new ROSLIB.Ros({url});
  } catch (e) {
    setConn(false, 'URL 无效');
    return;
  }
  ros.on('connection', () => {
    setConn(true);
    subscribeAll();
    ['command', 'trajectory', 'policy', 'paramSet', 'sysCmd'].forEach(ensurePublisher);
    // 连接后自动读取参数表（REPORT → device_param_report）随设备屏接入——
    // loadParams 归 core/device.js，这里只负责建立连接与订阅。
  });
  ros.on('error', () => setConn(false, '连接错误'));
  ros.on('close', () => {
    if (!ros) return; // 幂等：closeSocket 里已经收过尾，重复触发不再排重连
    closeSocket();
    // 掉线即清场：陈旧的表比空表更容易误导（曾把"页面掉线"看成"固件没数据"）。
    store.motors.clear();
    store.paramCache.clear();
    // 序号是固件侧单调递增的，重连后第一帧就比掉线前的基线大——不清 seen 的话
    // 一个还在飞的轮询会拿旧基线立刻"成功"。次帧到达前拒绝轮询。
    store.sysStatus.seen = false;
    markDirty(DIRTY.TOPOLOGY, DIRTY.STATUS, DIRTY.PARAMS, DIRTY.ALERTS);
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
  if (ros) { const r = ros; ros = null; try { r.close(); } catch (e) {} }
  subs = [];
  Object.values(publishers).forEach((p) => { try { p.unadvertise(); } catch (e) {} });
  Object.keys(publishers).forEach((k) => delete publishers[k]);
}

function sub(key, cb) {
  const t = TOPICS[key];
  const topic = new ROSLIB.Topic({
    ros, name: t.name, messageType: t.type, throttle_rate: 0, queue_length: 1,
  });
  topic.subscribe(cb);
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
    markDirty(DIRTY.TOPOLOGY, DIRTY.ALERTS);
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
    markDirty(DIRTY.STATUS, DIRTY.ALERTS);
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
    markDirty(DIRTY.EVENTS, DIRTY.PARAMS, DIRTY.ALERTS);
  });

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
    markDirty(DIRTY.ALERTS);
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
