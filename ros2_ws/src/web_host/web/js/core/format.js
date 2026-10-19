// 常量与格式化：从重构前的 app.js 常量区与工具区原样搬移。
// 与固件的隐式契约（DEV_SLOT_NAMES 位序、PARAM_FIELD 次序）在下方注释里保留原样。

export const TOPICS = {
  topology: {name: '/micro_ros_up/motor_topology', type: 'can_bridge_msgs/msg/MotorTopology'},
  motion: {name: '/micro_ros_up/motion_states', type: 'can_bridge_msgs/msg/MotorMotionStateArray'},
  status: {name: '/micro_ros_up/motor_status_array', type: 'can_bridge_msgs/msg/MotorStatusArray'},
  events: {name: '/micro_ros_up/motor_events', type: 'can_bridge_msgs/msg/MotorEvent'},
  imu: {name: '/micro_ros_up/imu/data', type: 'sensor_msgs/msg/Imu'},
  temp: {name: '/micro_ros_up/imu/temp', type: 'std_msgs/msg/Float32'},
  // 下·设定值（ADR 0014）：页面只下**设定值**，100 Hz 的完整快照流由主机侧
  // command_streamer 出。节拍不能留在页面里——浏览器把后台标签页的定时器钳到 1 Hz，
  // 而固件的快照新鲜度线是 30 ms 软停 / 100 ms 失能，页面一失焦台架就整臂掉力。
  // 它**不是**运行期可变的：命令从哪条路出去（直连固件 / 经叠加节点）由 bench_manager
  // 改 streamer 的 output_topic 决定，页面只读回执核对（见 TOPICS.commandStatus）。
  setpoint: {name: '/motor_command/setpoint',
    type: 'can_bridge_msgs/msg/MotorCommandArray'},
  // ⚠️ 轨迹的 name 是**运行期可变**的：重力补偿开关在「直连固件」与「经叠加节点」
  // 之间切它。唯一改写点是 core/ros.js 的 setDownlinkRoute（换名必须连带换发布者，
  // 散着改会让轨迹发到没人订阅的话题上）。
  trajectory: {name: '/micro_ros_do/motor_trajectory',
    type: 'can_bridge_msgs/msg/MotorTrajectory'},
  // 命令节拍节点的状态（JSON）：`output_topic` 是命令**当前实际发往**哪里。
  // 改道由服务端做，页面切不了，所以只能读它来核对开关与接线是否一致。
  commandStatus: {name: '/motor_command/status', type: 'std_msgs/msg/String'},
  trajectoryState: {name: '/micro_ros_up/trajectory_state', type: 'can_bridge_msgs/msg/MotorTrajectoryState'},
  policy: {name: '/micro_ros_do/motor_policy', type: 'can_bridge_msgs/msg/MotorPolicyArray'},
  devParams: {name: '/micro_ros_up/device_param_report', type: 'can_bridge_msgs/msg/DeviceParamReport'},
  sysStatus: {name: '/micro_ros_up/system_status', type: 'can_bridge_msgs/msg/SystemStatus'},
  paramSet: {name: '/micro_ros_do/param_set', type: 'can_bridge_msgs/msg/ParamSetArray'},
  sysCmd: {name: '/micro_ros_do/system_command', type: 'can_bridge_msgs/msg/SystemCommand'},
  // 关节侧实测构型：施加前馈时它是 τ_g 的**唯一**输入，而叠加节点在它超时/缺席时
  // 整帧原样转发（见 robot_dynamics/README.md 的安全不变式）——所以「开关亮着但
  // 没有补偿」是静默的，页面得自己看得见这条流的死活。
  jointStates: {name: '/joint_states', type: 'sensor_msgs/msg/JointState'},
  // TF（关节侧）：robot_state_publisher 发；页面只读，用来与电机流对照。
  tf: {name: '/tf', type: 'tf2_msgs/msg/TFMessage'},
  tfStatic: {name: '/tf_static', type: 'tf2_msgs/msg/TFMessage'},
  // 台架状态（bench_manager 节点，1 Hz + 命令后立即发）：仿真 / Agent / 电机
  // 后端 / 重力前馈四段快照的 JSON 字符串。
  bench: {name: '/bench/status', type: 'std_msgs/msg/String'},
};

// 下行接线的两个落点（core/ros.js 的 setDownlinkRoute 用，ADR 0013）。
// 叠加侧的 `/robot_dynamics/*` 是 gravity_feedforward_overlay 的**输入**话题，是节点契约。
// 页面自己只切**轨迹**那一条（命令那一条归服务端，见 TOPICS.setpoint 的注释）——
// 命令落点那两个名字的权威在 bench_manager/gravity_feedforward_manager.py。
export const DOWNLINK_ROUTE = {
  DIRECT: 'direct',
  OVERLAY: 'overlay',
};
export const OVERLAY_TRAJECTORY_TOPIC = '/robot_dynamics/trajectory';

export const LIFECYCLE = ['ABSENT', 'DISCOVERED', 'IDENTIFYING', 'READY_DISABLED',
  'ACTIVE', 'SAFE_STOP', 'FAULT', 'CONFIG_CONFLICT', 'OFFLINE'];
export const EVENT_NAMES = ['DISCOVERED', 'PROFILE_BOUND', 'PROFILE_REJECTED', 'READY',
  'ACTIVE', 'SOFT_STOP', 'DISABLE', 'FAULT', 'CONFIG_CONFLICT', 'OFFLINE',
  'RECONNECTED', 'AUTHORIZATION_LOST', 'SNAPSHOT_REJECTED', 'CONFIG_RESULT',
  'REGISTRY_FULL', 'DEV_INIT', 'TRAJECTORY_REJECTED'];
// 索引即故障码。8 是历史值（ZERO_STALE，2026-09-20 按真机实测删除）——
// 空着不复用，免得旧日志/事件里的 8 被认成别的意思。
export const FAULT_NAMES = ['无', '协议', '厂家', 'Profile', '冲突', '离线',
  '设备参数', '策略代数', undefined, '缺武装确认'];
// ⚠️ 与固件位图的隐式契约：槽位顺序必须逐位对齐 firmware 的 device_raw 槽索引，
// 权威定义在 micro_ros_agent/src/can_bridge_msgs/msg/MotorEvent.msg 的 EVENT_DEV_INIT 注释块。
export const DEV_SLOT_NAMES = ['位置下限', '位置上限', '最大转速', '峰值电流',
  '保护开关(0x2650:00)', '方向(0x2541)'];
export const DEV_INIT_STATE = ['无设备对象', '待回读', '已核对', '与记录不一致',
  '回读不成功', '本机无记录'];
// ⚠️ param_device_slot_t 的次序（= DEV_SLOT_NAMES 下标 = 行序 = mismatch_mask 位序），
// **不是** param_field_t。两套里 field=3 都合法 → 互用不会被拒绝，会静默写错槽位。
export const PARAM_FIELD = {POS_MIN: 0, POS_MAX: 1, VEL_LIMIT: 2,
  TORQUE_LIMIT: 3, LIMIT_PROTECT: 4, DIRECTION: 5};
export const SYS_CMD = {SAVE: 0, REFRESH: 1, SET_ZERO: 2, CLEAR_FAULT: 3, REPORT: 4};
export const MOTOR_ID_NONE = 65535;
export const CHART_COLORS = ['#4da3ff', '#37d67a', '#f5b94a', '#ff6b6b', '#c792ea',
  '#f78c6c', '#82aaff', '#ff9e64', '#7e8bab'];
export const TRAJ_MAX_POINTS = 256;
export const TRAJ_MAX_MOTORS = 64;
export const TRAJ_STATE_TIMEOUT_PAD_MS = 3000;

export const $ = (id) => document.getElementById(id);
export const fmt = (v, d = 2) => (v === null || v === undefined || !Number.isFinite(v))
  ? '--' : v.toFixed(d);
export const ageText = (ms) => {
  if (ms === null || ms === undefined) return '--';
  if (ms === 0xFFFFFFFF) return '从未';
  return ms < 1000 ? ms + ' ms' : (ms / 1000).toFixed(1) + ' s';
};
export function quatToEuler(x, y, z, w) {
  const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  const sinp = 2 * (w * y - z * x);
  const pitch = sinp >= 1 ? Math.PI / 2 : (sinp <= -1 ? -Math.PI / 2 : Math.asin(sinp));
  const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return {roll, pitch, yaw};
}
export const rad2deg = (r) => r * 180 / Math.PI;

export function lifecyclePill(lc) {
  if (lc === undefined || lc === null) return '--';
  const name = LIFECYCLE[lc] !== undefined ? LIFECYCLE[lc] : 'S' + lc;
  const cls = lc === 4 ? 'on' : (lc === 5 ? 'warn' : (lc === 6 || lc === 7 || lc === 8 ? 'off' : 'mid'));
  return `<span class="pill ${cls}">${name}</span>`;
}

export function busLabel(index) {
  return Number.isInteger(index) && index >= 0 && index < 3
    ? `FDCAN${index + 1}` : 'CAN --';
}

export function download(name, text, mime) {
  const blob = new Blob([text], {type: mime || 'application/json'});
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = name;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 3000);
}

// 位图 → 槽位名列表（槽位顺序即 DEV_SLOT_NAMES 顺序）。
export function devSlotNames(mask) {
  return DEV_SLOT_NAMES.filter((_, i) => (mask & (1 << i)) !== 0);
}

// 恢复口径（MISMATCH 用）：六个槽位都在参数面里了——槽 5（0x2541 方向）也能
// 通过参数表点一次方向建立基线并恢复。
export function devRecoveryText(mask) {
  if ((mask & (1 << 5)) !== 0) {
    return '方向(0x2541)无基线：在参数表点一次方向（CW/CCW）即建立基线并恢复';
  }
  return '写回对应项即可恢复';
}

// 参数值显示：内核把这些量存成 float32，拓宽成 double 后会出现
// 0.20000000298023224 这类尾巴——显示时收敛到有效位，别让人误以为是精度问题。
// 只影响显示；未编辑的字段不会下发。
export function fmtParam(value) {
  if (value === undefined || value === null || value === '') return '';
  const n = Number(value);
  if (!Number.isFinite(n)) return '';
  const s = Math.abs(n) >= 1e6 || (n !== 0 && Math.abs(n) < 1e-4)
    ? n.toExponential(6)
    : String(Number(n.toPrecision(7)));
  return s;
}

export const CONTROLLER_PORT_STORAGE_KEY = 'robot2.controllerSerialPort';
export const ROS_URL_STORAGE_KEY = 'robot2.rosbridgeUrl';
export const FOLD_KEY = 'web_host.fold';
