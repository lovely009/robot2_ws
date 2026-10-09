// 常量与格式化：从重构前的 app.js 常量区与工具区原样搬移。
// 与固件的隐式契约（DEV_SLOT_NAMES 位序、PARAM_FIELD 次序）在下方注释里保留原样。

export const TOPICS = {
  topology: {name: '/micro_ros_up/motor_topology', type: 'can_bridge_msgs/msg/MotorTopology'},
  motion: {name: '/micro_ros_up/motion_states', type: 'can_bridge_msgs/msg/MotorMotionStateArray'},
  status: {name: '/micro_ros_up/motor_status_array', type: 'can_bridge_msgs/msg/MotorStatusArray'},
  events: {name: '/micro_ros_up/motor_events', type: 'can_bridge_msgs/msg/MotorEvent'},
  imu: {name: '/micro_ros_up/imu/data', type: 'sensor_msgs/msg/Imu'},
  temp: {name: '/micro_ros_up/imu/temp', type: 'std_msgs/msg/Float32'},
  command: {name: '/micro_ros_do/motor_command_array', type: 'can_bridge_msgs/msg/MotorCommandArray'},
  trajectory: {name: '/micro_ros_do/motor_trajectory', type: 'can_bridge_msgs/msg/MotorTrajectory'},
  trajectoryState: {name: '/micro_ros_up/trajectory_state', type: 'can_bridge_msgs/msg/MotorTrajectoryState'},
  policy: {name: '/micro_ros_do/motor_policy', type: 'can_bridge_msgs/msg/MotorPolicyArray'},
  devParams: {name: '/micro_ros_up/device_param_report', type: 'can_bridge_msgs/msg/DeviceParamReport'},
  sysStatus: {name: '/micro_ros_up/system_status', type: 'can_bridge_msgs/msg/SystemStatus'},
  paramSet: {name: '/micro_ros_do/param_set', type: 'can_bridge_msgs/msg/ParamSetArray'},
  sysCmd: {name: '/micro_ros_do/system_command', type: 'can_bridge_msgs/msg/SystemCommand'},
};

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
export const STREAM_PERIOD_MS = 10;
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

export const CONTROLLER_PORT_STORAGE_KEY = 'robot2.controllerSerialPort';
export const ROS_URL_STORAGE_KEY = 'robot2.rosbridgeUrl';
export const FOLD_KEY = 'web_host.fold';
export const WING_KEY = 'web_host.wing';
