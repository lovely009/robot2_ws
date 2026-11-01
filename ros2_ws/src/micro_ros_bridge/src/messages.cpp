#include "micro_ros_bridge/messages.hpp"

#include <cmath>
#include <limits>
#include <algorithm>

namespace micro_ros_bridge
{
namespace
{
constexpr uint16_t kIdLimit = 65;  // V3 motor IDs are 1..64; index 0 remains for legacy helpers.
// 配置面目标数上界 = 内核 CONTROL_MAILBOX_CONFIG_QUEUE_CAPACITY（不是带宽约束）。
constexpr size_t kConfigTargetsMax = 16;
// 缺省哨兵：与 WireContract.SENTINEL_MISSING 同值，这里给个语义化别名。
constexpr int16_t kMissingI16 = contract::kSentinelMissing;   // -32768
constexpr uint16_t kMissingU16 = 0xFFFFu;
}

Result encode_low_command(
  const can_bridge_msgs::msg::MotorCommandArray & in,
  can_bridge_msgs::msg::LowCommand & out)
{
  Result r;
  if (in.targets.size() != in.count) {
    r.ok = false;
    r.error = "count 与 targets 长度不一致";
    return r;
  }
  // 防御性：经 ROS C++ API 不可达——targets 是 BoundedVector<...,64>（size 恒 ≤64），
  // 上面长度守卫又要求 count == size，count > 64 到不了这里。留着是因为它对应
  // MOTOR_COUNT_MAX；将来若放宽 .msg 上界，它是唯一拦截点。
  if (in.count > contract::kMotorCountMax) {
    r.ok = false;
    r.error = "目标数超过 MOTOR_COUNT_MAX";
    return r;
  }
  // 先全量校验再写输出：整条受理或整条拒绝，不留半条。
  bool seen[kIdLimit] = {false};
  for (const auto & t : in.targets) {
    if (t.motor_id >= kIdLimit) {
      r.ok = false;
      r.error = "motor_id 越界（最大 ID 64）";
      return r;
    }
    if (seen[t.motor_id]) {
      r.ok = false;
      r.error = "motor_id 重复";
      return r;
    }
    seen[t.motor_id] = true;
    if (!std::isfinite(t.position) || !std::isfinite(t.velocity) || !std::isfinite(t.effort)) {
      r.ok = false;
      r.error = "命令含 NaN/Inf：整条拒收";
      return r;
    }
  }

  out.sequence = in.sequence;
  out.lease_id = in.lease_id;
  out.count = in.count;
  out.motor_ids.clear();
  out.position_q.clear();
  out.velocity_q.clear();
  out.effort_q.clear();
  for (const auto & t : in.targets) {
    const auto p = quantize(t.position, contract::kScalePosition);
    const auto v = quantize(t.velocity, contract::kScaleVelocity);
    const auto e = quantize(t.effort, contract::kScaleEffort);
    r.saturated += (p.saturated ? 1u : 0u) + (v.saturated ? 1u : 0u) + (e.saturated ? 1u : 0u);
    out.motor_ids.push_back(static_cast<uint8_t>(t.motor_id));
    out.position_q.push_back(p.q);
    out.velocity_q.push_back(v.q);
    out.effort_q.push_back(e.q);
  }
  return r;
}

Result decode_low_state(
  const can_bridge_msgs::msg::LowState & in,
  can_bridge_msgs::msg::MotorMotionStateArray & out)
{
  Result r;
  const size_t n = in.motor_ids.size();
  if (in.position_q.size() != n || in.velocity_q.size() != n || in.effort_q.size() != n ||
    in.state_age_ms.size() != n || in.target_source.size() != n || in.count != n ||
    n > can_bridge_msgs::msg::WireContract::LOW_STATE_CHUNK_MAX ||
    in.schema_version != contract::kSchemaVersion || in.session_id == 0 ||
    in.chunk_count == 0 || in.chunk_count > contract::kMotorCountMax ||
    in.chunk_index >= in.chunk_count || in.total_count > contract::kMotorCountMax ||
    in.total_count < n || (in.total_count == 0 && (in.chunk_count != 1 || n != 0)) ||
    (in.total_count > 0 && n == 0) ||
    (in.total_count > 0 && in.chunk_count > in.total_count))
  {
    r.ok = false;
    r.error = "count 与各数组长度不一致";
    return r;
  }
  bool seen[kIdLimit] = {};
  for (size_t i = 0; i < n; ++i) {
    const auto id = in.motor_ids[i];
    if (id >= kIdLimit || seen[id] ||
      in.target_source[i] > can_bridge_msgs::msg::MotorMotionStateTarget::TARGET_SOURCE_TRAJECTORY)
    {
      r.ok = false;
      r.error = "motor_id 越界（最大 ID 64）";
      return r;
    }
    seen[id] = true;
  }
  can_bridge_msgs::msg::MotorMotionStateArray decoded;
  decoded.sync_time_ns = in.sync_time_ns;
  decoded.session_id = in.session_id;
  decoded.tick_ms = in.tick_ms;
  // seq 直通：主机侧唯一的丢帧判据（best-effort 无重传）。facade 的
  // MotorMotionStateArray 末尾为此追加了 seq 字段。
  decoded.seq = in.seq;
  for (size_t i = 0; i < n; ++i) {
    can_bridge_msgs::msg::MotorMotionStateTarget s;
    s.motor_id = in.motor_ids[i];
    s.position = dequantize(in.position_q[i], contract::kScalePosition);
    s.velocity = dequantize(in.velocity_q[i], contract::kScaleVelocity);
    s.effort = dequantize(in.effort_q[i], contract::kScaleEffort);
    s.state_age_ms = in.state_age_ms[i] == 0xFFFFu ? std::numeric_limits<uint32_t>::max() :
      static_cast<uint32_t>(in.state_age_ms[i]);
    s.target_source = in.target_source[i];
    decoded.states.push_back(s);
  }
  out = std::move(decoded);
  return r;
}

void LowStateAssembler::reset()
{
  session_ = 0;
  have_last_ = false;
  chunks_.clear();
  received_.clear();
  source_delays_ms_.clear();
}

bool LowStateAssembler::accept(const can_bridge_msgs::msg::LowState & in, uint64_t session,
  TimePoint now, can_bridge_msgs::msg::MotorMotionStateArray & out, uint32_t source_delay_ms)
{
  if (session == 0 || in.session_id != session) {return false;}
  if (session_ != session) {reset(); session_ = session;}
  if (have_last_ && static_cast<int32_t>(in.seq - last_seq_) <= 0) {return false;}
  can_bridge_msgs::msg::MotorMotionStateArray decoded;
  if (!decode_low_state(in, decoded).ok) {return false;}
  if (!chunks_.empty() && now - started_ > std::chrono::milliseconds(60)) {
    last_seq_ = pending_seq_;
    have_last_ = true;
    chunks_.clear();
    if (static_cast<int32_t>(in.seq - last_seq_) <= 0) {return false;}
  }
  if (!chunks_.empty() && in.seq != pending_seq_) {
    if (static_cast<int32_t>(in.seq - pending_seq_) <= 0) {return false;}
    chunks_.clear();
  }
  if (chunks_.empty()) {
    pending_seq_ = in.seq;
    pending_tick_ = in.tick_ms;
    pending_sync_ = in.sync_time_ns;
    pending_total_ = in.total_count;
    started_ = now;
    chunks_.resize(in.chunk_count);
    received_.resize(in.chunk_count);
    source_delays_ms_.resize(in.chunk_count);
  }
  if (in.chunk_count != chunks_.size() || in.total_count != pending_total_ ||
    in.tick_ms != pending_tick_ || in.sync_time_ns != pending_sync_ ||
    chunks_[in.chunk_index].has_value())
  {
    last_seq_ = pending_seq_;
    have_last_ = true;
    chunks_.clear();
    return false;
  }
  chunks_[in.chunk_index] = in;
  received_[in.chunk_index] = now;
  source_delays_ms_[in.chunk_index] = source_delay_ms;
  if (std::any_of(chunks_.begin(), chunks_.end(),
    [](const auto & item) {return !item.has_value();})) {return false;}
  can_bridge_msgs::msg::MotorMotionStateArray merged;
  merged.seq = in.seq;
  merged.sync_time_ns = pending_sync_;
  merged.session_id = in.session_id;
  uint64_t reference_delay_ms = 0;
  for (size_t j = 0; j < chunks_.size(); ++j) {
    const auto waited = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - received_[j]).count();
    reference_delay_ms = std::max(reference_delay_ms,
      static_cast<uint64_t>(source_delays_ms_[j]) + static_cast<uint64_t>(waited));
  }
  // Every chunk belongs to the same MCU tick. Use one common age reference,
  // so tick_ms - state_age_ms still denotes each physical measurement time.
  merged.tick_ms = pending_tick_ + static_cast<uint32_t>(reference_delay_ms);
  bool seen[kIdLimit] = {};
  for (size_t j = 0; j < chunks_.size(); ++j) {
    can_bridge_msgs::msg::MotorMotionStateArray part;
    if (!decode_low_state(*chunks_[j], part).ok) {
      last_seq_ = pending_seq_; have_last_ = true; chunks_.clear(); return false;
    }
    for (auto & state : part.states) {
      if (seen[state.motor_id]) {
        last_seq_ = pending_seq_; have_last_ = true; chunks_.clear(); return false;
      }
      seen[state.motor_id] = true;
      if (state.state_age_ms != std::numeric_limits<uint32_t>::max()) {
        state.state_age_ms = static_cast<uint32_t>(std::min<uint64_t>(
          static_cast<uint64_t>(state.state_age_ms) + reference_delay_ms,
          std::numeric_limits<uint32_t>::max() - 1u));
      }
      merged.states.push_back(state);
    }
  }
  chunks_.clear();
  if (merged.states.size() != pending_total_) {
    last_seq_ = pending_seq_; have_last_ = true; return false;
  }
  out = std::move(merged);
  last_seq_ = in.seq;
  have_last_ = true;
  return true;
}

Result encode_config(
  const can_bridge_msgs::msg::ParamSetArray & in,
  can_bridge_msgs::msg::ConfigRequest & out)
{
  (void)in;
  (void)out;
  return {false, "运行期电机参数设置已移除"};
}

Result encode_config(
  const can_bridge_msgs::msg::SystemCommand & in,
  can_bridge_msgs::msg::ConfigRequest & out)
{
  using Cmd = can_bridge_msgs::msg::SystemCommand;
  Result r;
  if (in.command != Cmd::COMMAND_REFRESH && in.command != Cmd::COMMAND_CLEAR_FAULT &&
      in.command != Cmd::COMMAND_REPORT) {
    r.ok = false;
    r.error = "运行期参数写入、设零与保存已移除";
    return r;
  }
  if (in.request_id >= 0x40000000u) {
    r.ok = false;
    r.error = "request_id 高两位保留，主机不得置位";
    return r;
  }
  if (in.command != Cmd::COMMAND_REPORT &&
    in.motor_id >= kIdLimit)
  {
    r.ok = false;
    r.error = "该命令必须给有效 motor_id（0..63）";
    return r;
  }

  out.sync_time_ns = in.sync_time_ns;
  out.request_id = in.request_id;
  out.kind = can_bridge_msgs::msg::ConfigRequest::CONFIG_KIND_SYSTEM_COMMAND;
  out.count = 0;
  out.targets.clear();  // 该组不适用：互斥由构造保证
  out.command = in.command;
  out.motor_id = in.motor_id;
  return r;
}

Result decode_motor_status(
  const can_bridge_msgs::msg::MotorStatus & in,
  can_bridge_msgs::msg::MotorStatusArray & statuses,
  can_bridge_msgs::msg::MotorTrajectoryState & traj,
  can_bridge_msgs::msg::SystemStatus & sys,
  can_bridge_msgs::msg::MotorDiagnostics & diag,
  std::vector<MotorFreshness> & freshness)
{
  Result r;
  // 整帧重来：本帧未列出的电机读作"从未收到"，不保留上一帧的陈旧值。
  std::vector<MotorFreshness> next_freshness(
    kIdLimit);
  const size_t n = in.motor_ids.size();
  const bool lengths_ok =
    in.count == n &&
    in.state_age_ms.size() == n && in.target_source.size() == n &&
    in.lifecycle.size() == n && in.unified_fault_code.size() == n &&
    in.vendor_fault_code.size() == n && in.protection_flags.size() == n &&
    in.limit_flags.size() == n && in.temperature_dc.size() == n &&
    in.current_ma.size() == n && in.voltage_mv.size() == n &&
    in.drive_mode.size() == n && in.status_flags.size() == n &&
    in.bus_index.size() == n;
  if (!lengths_ok) {
    r.ok = false;
    r.error = "逐台数组长度与 count 不一致";
    return r;
  }
  // motor_id 是 uint8（0..255），而 freshness 是定长 64 的缓存：越界 id 会越界写。
  // 这座桥是信任边界，任一条越界即整条拒收——不 clamp、不跳过（跳过会让上层拿到
  // 一条残缺的帧却以为它完整）。
  bool seen_status[kIdLimit] = {};
  for (size_t i = 0; i < n; ++i) {
    const auto id = in.motor_ids[i];
    if (id >= kIdLimit || seen_status[id] ||
      in.target_source[i] > can_bridge_msgs::msg::MotorMotionStateTarget::TARGET_SOURCE_TRAJECTORY ||
      in.lifecycle[i] > can_bridge_msgs::msg::MotorStatusTarget::LIFECYCLE_OFFLINE ||
      (in.status_flags[i] & ~0x1Fu) != 0)
    {
      r.ok = false;
      r.error = "motor_id 越界（最大 ID 64）";
      return r;
    }
    seen_status[id] = true;
  }
  if (in.traj_target_count != in.traj_target_ids.size()) {
    r.ok = false; r.error = "traj_target_count 不一致"; return r;
  }
  bool seen_traj[kIdLimit] = {};
  for (const auto id : in.traj_target_ids) {
    if (id >= kIdLimit || seen_traj[id]) {
      r.ok = false; r.error = "轨迹目标 ID 非法"; return r;
    }
    seen_traj[id] = true;
  }

  // ---- MotorStatusArray ----
  statuses.sync_time_ns = in.sync_time_ns;
  statuses.statuses.clear();
  for (size_t i = 0; i < n; ++i) {
    can_bridge_msgs::msg::MotorStatusTarget s;
    s.motor_id = in.motor_ids[i];
    // 新鲜度与目标来源存入缓存，填充 motion_states 时使用，不写入 MotorStatusTarget。
    // 0xFFFF = 从未收到 → 还原成 0xFFFFFFFF；其余毫秒直通（固件侧真实年龄
    // 饱和在 0xFFFE，不会与哨兵撞车）。
    next_freshness[in.motor_ids[i]].state_age_ms = (in.state_age_ms[i] == kMissingU16)
      ? 0xFFFFFFFFu : static_cast<uint32_t>(in.state_age_ms[i]);
    next_freshness[in.motor_ids[i]].target_source = in.target_source[i];
    s.lifecycle = in.lifecycle[i];
    s.unified_fault_code = in.unified_fault_code[i];
    s.vendor_fault_code = in.vendor_fault_code[i];
    s.protection_flags = in.protection_flags[i];
    s.limit_flags = in.limit_flags[i];
    // 电气量回 SI：facade 用 NaN 表示缺失（不是哨兵）。
    s.temperature = (in.temperature_dc[i] == kMissingI16)
      ? std::nan("") : static_cast<double>(in.temperature_dc[i]) * 0.1;
    s.current = (in.current_ma[i] == std::numeric_limits<int32_t>::min())
      ? std::nan("") : static_cast<double>(in.current_ma[i]) * 0.001;
    s.voltage = (in.voltage_mv[i] == std::numeric_limits<uint32_t>::max())
      ? std::nan("") : static_cast<double>(in.voltage_mv[i]) * 0.001;
    s.drive_mode = in.drive_mode[i];
    // status_flags 位序见 MotorStatus.msg 头注释；还原成**正向语义**布尔，
    // 0x2650 的反转不在这里做（它只发生在固件 param_bridge 一处）。
    const uint8_t f = in.status_flags[i];
    s.online = (f & 0x01u) != 0u;
    s.device_protect_known = (f & 0x02u) != 0u;
    s.device_protect_enabled = (f & 0x04u) != 0u;
    s.device_direction_known = (f & 0x08u) != 0u;
    s.device_direction = (f & 0x10u) != 0u ? 1u : 0u;
    s.bus_index = in.bus_index[i];
    statuses.statuses.push_back(s);
  }

  // ---- MotorTrajectoryState ----
  traj.sync_time_ns = in.sync_time_ns;
  traj.session_id = in.session_id;
  traj.sequence = in.sequence;
  traj.tick_ms = in.tick_ms;
  traj.trajectory_id = in.trajectory_id;
  traj.active = in.traj_active;
  traj.completed = in.traj_completed;
  traj.elapsed_ms = in.traj_elapsed_ms;
  traj.total_ms = in.traj_total_ms;
  traj.target_count = in.traj_target_count;
  traj.target_ids = in.traj_target_ids;  // 同类型序列

  // ---- SystemStatus ----
  sys.sync_time_ns = in.sync_time_ns;
  sys.last_error = in.last_error;
  sys.last_error_motor = in.last_error_motor;
  sys.refresh_seq = in.refresh_seq;
  sys.set_zero_seq = in.set_zero_seq;
  sys.clear_fault_seq = in.clear_fault_seq;
  sys.save_seq = in.save_seq;
  sys.save_in_progress = in.save_in_progress;

  // ---- MotorDiagnostics ----
  diag.sync_time_ns = in.sync_time_ns;
  diag.rx_frame_count = in.rx_frame_count;
  diag.unknown_frame_count = in.unknown_frame_count;
  diag.decode_error_count = in.decode_error_count;
  diag.snapshot_accepted = in.snapshot_accepted;
  diag.snapshot_rejected = in.snapshot_rejected;
  diag.config_rejected = in.config_rejected;
  diag.event_dropped = in.event_dropped;
  diag.tx_dropped = in.tx_dropped;
  diag.p0_retry = in.p0_retry;
  diag.trajectory_accepted = in.trajectory_accepted;
  diag.trajectory_rejected = in.trajectory_rejected;
  diag.policy_accepted = in.policy_accepted;
  diag.policy_rejected = in.policy_rejected;
  diag.mailbox_rejected_command = in.mailbox_rejected_command;
  diag.mailbox_rejected_trajectory = in.mailbox_rejected_trajectory;
  diag.mailbox_rejected_policy = in.mailbox_rejected_policy;
  diag.publish_fail = in.publish_fail;
  diag.ring_drop = in.ring_drop;            // uint32[<=3]，同类型
  diag.rx_hw_lost = in.rx_hw_lost;
  diag.tx_frame_count = in.tx_frame_count;
  diag.heap_free = in.heap_free;
  freshness = std::move(next_freshness);
  return r;
}
}  // namespace micro_ros_bridge
