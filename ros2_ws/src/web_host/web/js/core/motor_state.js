import { FAULT_NAMES } from './format.js';
// MCU 控制快照是授权事实；页面草稿与旧运行态保持独立。
export const CONTROL_LIFECYCLE = {
  0:'未知', 1:'未配置', 2:'就绪 · 未使能', 3:'活动 · 已使能', 4:'已终止',
  10:'配置中', 13:'使能中', 14:'失能中', 15:'错误处理中',
};
export const EXECUTION = ['未知','空闲','保持','快照控制','执行轨迹','软停','驱动使能中','驱动失能中'];
const uint = (v, max=0xFFFFFFFF) => Number.isSafeInteger(v) && v>=0 && v<=max;
export function createControlStateCache() {
  return {seen:false,valid:false,at:0,session:0,sequence:0,byId:new Map()};
}
export function applyControlState(cache, message, now=Date.now()) {
  cache.seen=true;
  const invalid=()=>{cache.valid=false;return false;};
  if (!message || message.schema_version!==2 || !uint(message.session_id,Number.MAX_SAFE_INTEGER)
      || !message.session_id || !uint(message.sequence) || !Array.isArray(message.states)
      || message.states.length>64 || !Number.isFinite(now)) return invalid();
  if (cache.session && message.session_id<cache.session) return false;
  if (cache.session===message.session_id) {
    const advance=(message.sequence-cache.sequence)>>>0;
    if (!advance || advance>=0x80000000) return false;
  }
  const byId=new Map();
  for (const s of message.states) {
    if (!s || !uint(s.motor_id,65535) || byId.has(s.motor_id)
        || !Number.isInteger(s.control_lifecycle) || !(s.control_lifecycle in CONTROL_LIFECYCLE)
        || !uint(s.execution_state,7) || typeof s.online!=='boolean'
        || typeof s.policy_enabled!=='boolean' || typeof s.motion_enabled!=='boolean'
        || !uint(s.state_age_ms) || !uint(s.unified_fault_code,65535)
        || !uint(s.vendor_fault_code) || !uint(s.last_failure_reason,65535)
        || !uint(s.policy_generation)) return invalid();
    byId.set(s.motor_id,{...s});
  }
  Object.assign(cache,{valid:true,at:now,session:message.session_id,sequence:message.sequence,byId});
  return true;
}
export function describeMotorState(state,id,now=Date.now()) {
  const cache=state.controlStates, row=state.motors.get(id)||{}, st=row.status||{}, topo=row.topo||{};
  if (cache?.seen) {
    const s=cache.byId.get(id), age=now-cache.at;
    const fresh=Boolean(state.conn.connected && cache.valid && s && age>=0 && age<=2500);
    const feedbackFresh=fresh && s.state_age_ms+age<=1500;
    const online=fresh && s.online;
    const fault=s?.unified_fault_code||0;
    const active=Boolean(state.conn.connected && online && s.control_lifecycle===3 && s.policy_enabled && !fault);
    return {protocol:'MCU 状态',online:Boolean(online),active,fault,
      connection:!state.conn.connected?'未连接':!fresh?'状态超时':!s.online?'离线':!feedbackFresh?'反馈超时':'在线',
      lifecycle:fresh?CONTROL_LIFECYCLE[s.control_lifecycle]:'状态待更新',
      execution:fresh?EXECUTION[s.execution_state]:'未知',
      health:fault?`故障：${FAULT_NAMES[fault] || fault}`:'正常',
      canCommand:active && s.state_age_ms+age<=500,
      policyEnabled:fresh ? s.policy_enabled : null,
      generation:fresh ? s.policy_generation : null};
  }
  const age=now-(row.statusAt ?? now), fresh=age>=0&&age<=2500;
  const online=Boolean(state.conn.connected && fresh && (st.online??topo.online));
  const raw=st.lifecycle??topo.lifecycle, fault=st.unified_fault_code||0;
  const names=['未发现','未配置','配置中','就绪 · 使能待确认','活动（旧版）','软停','故障','配置冲突','离线'];
  const motion=row.motion||{};
  return {protocol:'旧版状态',online,active:online&&raw===4&&!fault,fault,
    connection:!state.conn.connected?'未连接':!fresh?'状态超时':online?'在线':'离线',
    lifecycle:fresh?(names[raw]||'未知'):'状态待更新',
    execution:raw===5?'软停':raw===4?(motion.target_source===2?'轨迹归属（旧版）':'控制中（旧版）'):'未知',
    health:fault?`故障：${FAULT_NAMES[fault] || fault}`:'正常',canCommand:null,policyEnabled:null,generation:null};
}

export function controlTargetsError(state,ids,now=Date.now()) {
  if (!state.controlStates?.seen) return '';
  const blocked=ids.filter(id=>!describeMotorState(state,Number(id),now).canCommand);
  return blocked.length ? `MCU 未确认 m${blocked.join('、m')} 控制已使能、在线且反馈新鲜` : '';
}
