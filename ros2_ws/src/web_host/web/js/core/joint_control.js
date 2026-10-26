// 关节入口只校验身份与新鲜度；传动数值换算全部在主机共享库执行。
import { describeMotorState } from './motor_state.js';
import { jointTargetLimitError } from './joint_limits.js';

export function jointSliderDuration(joint, current, target) {
  return Number.isFinite(joint.velocity) && joint.velocity > 0
    ? Math.max(.15, 1.5*Math.abs(target-current)/joint.velocity) : .15;
}
// 连续拖动只保留最新目标；不把鼠标事件排成待执行轨迹队列。
export function createLatestTargetSender({send, isCurrent, onError=()=>{}, intervalMs=50}) {
  let pending=null, timer=null, last=null, sentAt=-Infinity, disposed=false;
  const cancel = () => {
    if (timer !== null) clearTimeout(timer);
    timer=null; pending=null; last=null; sentAt=-Infinity;
  };
  const flush = () => {
    if (timer !== null) clearTimeout(timer);
    timer=null;
    const target=pending; pending=null;
    if (disposed || !target) return;
    if (!isCurrent(target.context)) {cancel();return;}
    if (last && last.value===target.value && last.context===target.context) return;
    try {
      send(target.value); last=target; sentAt=Date.now();
    } catch (error) {cancel();onError(error);}
  };
  return {
    update(value, context, final=false) {
      if (disposed) return;
      if (!isCurrent(context)) {cancel();return;}
      if (last && last.context!==context) cancel();
      pending={value,context};
      const remaining=intervalMs-(Date.now()-sentAt);
      if (final || remaining<=0) flush();
      else if (timer===null) timer=setTimeout(flush,remaining);
    },
    cancel,
    dispose() {cancel();disposed=true;},
  };
}

export function jointControlError(state, model, joint, now = Date.now()) {
  if (!state.conn?.connected) return '未连接 rosbridge';
  const meta = state.modelTransmissions;
  if (meta?.model !== model || !Array.isArray(meta.groups)) return '模型传动配置未就绪';
  const runtime = state.transmissionStatus;
  if (!runtime?.last || now-runtime.last < 0 || now-runtime.last > 2500) return '主机传动桥状态已过期';
  if (runtime.data?.model !== model) return '页面模型与主机传动模型不一致';
  if (!runtime.data.trajectory_enabled) return '主机传动桥为只读模式';
  if (!runtime.data.joint_names?.includes(joint)) return '该关节不属于当前主机控制模型';
  const command = state.commandStatus;
  if (!command?.last || now-command.last < 0 || now-command.last > 2500
      || !['/motor/trajectory','/robot_dynamics/trajectory'].includes(command.data?.trajectory_output_topic)) {
    return '主机轨迹路由未就绪';
  }
  if (command.data.trajectory_output_topic !== '/motor/trajectory') return '关节目标暂需直连，请先关闭重力补偿';
  const group = meta.groups.find(g => g.joints.includes(joint));
  if (!group) return '关节未配置传动组';
  for (const name of group.joints) {
    const sample = state.jointStates?.byName?.[name];
    if (!Number.isFinite(sample?.position) || !Number.isFinite(sample?.at)
        || now-sample.at < 0 || now-sample.at > 500) return `同组关节 ${name} 缺少新鲜反馈`;
  }
  for (const member of group.motors) {
    const policy = state.policy?.get(Number(member.id));
    if (policy?.enabled === false) return `m${member.id} 已请求失能，请重新使能所选传动组`;
    if (policy?.receipt != null && policy.receipt !== 0) return `m${member.id} 的使能策略被拒绝（reason=${policy.receipt}），请重新下发`;
    if (state.controlStates?.seen) {
      if (!describeMotorState(state,Number(member.id),now).canCommand) {
        return `MCU 状态未确认 m${member.id} 控制已使能、在线且反馈新鲜`;
      }
      continue;
    }
    const status = state.motors.get(Number(member.id))?.status;
    if (!status?.online || ![3, 4].includes(status.lifecycle) || Number(status.unified_fault_code || 0)) {
      return `传动组 ${group.name} 的 m${member.id} 未就绪、离线或存在故障`;
    }
    // 策略武装不推进 ACTIVE；首个目标才启动运动，实际授权仍由固件裁决。
    if (status.lifecycle === 3 && !state.policy?.get(Number(member.id))?.enabled) {
      return `请先下发 m${member.id} 的使能策略，再提交首个关节目标`;
    }
  }
  return '';
}

export function buildJointTrajectory(state, model, joint, target, seconds, now = Date.now()) {
  const error = jointControlError(state, model, joint, now);
  if (error) throw new Error(error);
  if (!Number.isFinite(target) || !Number.isFinite(seconds) || seconds < .01 || seconds > 60) {
    throw new Error('目标必须为有限值，到达时间须在 0.01～60 秒');
  }
  const limitError = jointTargetLimitError(state, model, joint, target);
  if (limitError) throw new Error(limitError);
  const ms = Math.round(seconds*1000);
  return {header:{stamp:{sec:Math.floor(now/1000),nanosec:(now%1000)*1e6},frame_id:''},
    joint_names:[joint], points:[{positions:[target],velocities:[],accelerations:[],effort:[],
      time_from_start:{sec:Math.floor(ms/1000),nanosec:(ms%1000)*1e6}}]};
}

export function buildJointSliderTrajectory(state, model, joint, target, now=Date.now()) {
  const current=state.jointStates?.byName?.[joint.name]?.position;
  const message=buildJointTrajectory(state, model, joint.name, target,
    jointSliderDuration(joint,current,target),now);
  // 固件在首个路点前保持首点；必须显式带 t=0，单个未来路点会直接跳到终点。
  message.points.unshift({positions:[current],velocities:[],accelerations:[],effort:[],
    time_from_start:{sec:0,nanosec:0}});
  return message;
}

export function rawCouplingError(state, model, ids) {
  if (!ids.length) return '';
  const meta = state.modelTransmissions;
  if (meta?.model !== model || !Array.isArray(meta.groups)) return '当前模型传动配置未就绪，拒绝原始运动下发';
  const touched = meta.groups.find(g => (g.motors.length > 1 || g.joints.length > 1)
    && g.motors.some(m => ids.includes(Number(m.id))));
  return touched ? `传动组 ${touched.name} 必须使用关节目标；原始电机命令不执行混控` : '';
}
