#include "micro_ros_bridge/messages.hpp"

#include <cmath>
#include <limits>

namespace micro_ros_bridge
{
namespace
{
constexpr uint16_t kIdLimit = contract::kMotorCountMax;  // motor_id ∈ [0,63]
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
  if (in.count > kIdLimit) {
    r.ok = false;
    r.error = "目标数超过 MOTOR_COUNT_MAX";
    return r;
  }
  // 先全量校验再写输出：整条受理或整条拒绝，不留半条。
  bool seen[kIdLimit] = {false};
  for (const auto & t : in.targets) {
    if (t.motor_id >= kIdLimit) {
      r.ok = false;
      r.error = "motor_id 越界（注册表下标 0..63）";
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

  out.sync_time_ns = in.sync_time_ns;
  out.sequence = in.sequence;
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
    in.count != n)
  {
    r.ok = false;
    r.error = "count 与各数组长度不一致";
    return r;
  }
  out.sync_time_ns = in.sync_time_ns;
  // seq 直通：主机侧唯一的丢帧判据（best-effort 无重传）。facade 的
  // MotorMotionStateArray 末尾为此追加了 seq 字段。
  out.seq = in.seq;
  out.states.clear();
  for (size_t i = 0; i < n; ++i) {
    can_bridge_msgs::msg::MotorMotionStateTarget s;
    s.motor_id = in.motor_ids[i];
    s.position = dequantize(in.position_q[i], contract::kScalePosition);
    s.velocity = dequantize(in.velocity_q[i], contract::kScaleVelocity);
    s.effort = dequantize(in.effort_q[i], contract::kScaleEffort);
    // state_age_ms 与 target_source 不在这条消息里（循环面不分片）。本函数保持
    // 纯函数：留成"未知"，由 BridgeNode::on_low_state 用低频面缓存盖章。
    s.state_age_ms = std::numeric_limits<uint32_t>::max();
    s.target_source = can_bridge_msgs::msg::MotorMotionStateTarget::TARGET_SOURCE_NONE;
    out.states.push_back(s);
  }
  return r;
}

Result encode_config(
  const can_bridge_msgs::msg::ParamSetArray & in,
  can_bridge_msgs::msg::ConfigRequest & out)
{
  using Target = can_bridge_msgs::msg::ParamSetTarget;
  Result r;
  if (in.targets.size() != in.count) {
    r.ok = false;
    r.error = "count 与 targets 长度不一致";
    return r;
  }
  // 防御性：经 ROS C++ API 不可达——targets 是 BoundedVector<...,16>（size 恒 ≤16），
  // 上面长度守卫又要求 count == size，count > 16 到不了这里。留着是因为它对应
  // ConfigRequest.msg 的内核 FIFO 容量约束；将来若放宽 .msg 上界，它是唯一拦截点。
  if (in.count > kConfigTargetsMax) {
    r.ok = false;
    r.error = "目标数超过配置 FIFO 容量 16";
    return r;
  }
  // bit30 是固件自发脉冲的命名空间，主机置位会被本地拒绝；bit31 已被内核身份读取占用。
  if ((in.request_id & 0x40000000u) != 0u) {
    r.ok = false;
    r.error = "request_id bit30 保留给固件自发脉冲，主机不得置位";
    return r;
  }
  // 第二维用 FIELD_DIRECTION + 1 而不是字面量 6：将来加槽位时，上面那条
  // `t.field > Target::FIELD_DIRECTION` 会跟着放宽，而写死的 6 不会——于是
  // `seen[motor_id][field]` 静默越界写。
  bool seen[kIdLimit][Target::FIELD_DIRECTION + 1] = {};
  bool has_min[kIdLimit] = {};
  bool has_max[kIdLimit] = {};
  for (const auto & t : in.targets) {
    if (t.motor_id >= kIdLimit) {
      r.ok = false;
      r.error = "motor_id 越界（注册表下标 0..63）";
      return r;
    }
    if (t.field > Target::FIELD_DIRECTION) {
      r.ok = false;
      r.error = "未知参数槽位";
      return r;
    }
    if (seen[t.motor_id][t.field]) {
      r.ok = false;
      r.error = "同一 (motor_id, field) 重复";
      return r;
    }
    seen[t.motor_id][t.field] = true;
    if (t.field == Target::FIELD_POS_MIN) {
      has_min[t.motor_id] = true;
    }
    if (t.field == Target::FIELD_POS_MAX) {
      has_max[t.motor_id] = true;
    }
  }
  // 位置限位必须成对出现在同一条消息里（半边状态正是本设计要消掉的东西）。
  for (size_t id = 0; id < kIdLimit; ++id) {
    if (has_min[id] != has_max[id]) {
      r.ok = false;
      r.error = "位置限位必须成对：只给了一边";
      return r;
    }
  }

  out.sync_time_ns = in.sync_time_ns;
  out.request_id = in.request_id;
  out.kind = can_bridge_msgs::msg::ConfigRequest::CONFIG_KIND_PARAM_SET;
  out.count = in.count;
  out.targets = in.targets;  // 同类型序列，直接搬
  out.command = can_bridge_msgs::msg::SystemCommand::COMMAND_SAVE;  // 该组不适用
  out.motor_id = can_bridge_msgs::msg::SystemCommand::MOTOR_ID_NONE;
  return r;
}

Result encode_config(
  const can_bridge_msgs::msg::SystemCommand & in,
  can_bridge_msgs::msg::ConfigRequest & out)
{
  using Cmd = can_bridge_msgs::msg::SystemCommand;
  Result r;
  if (in.command > Cmd::COMMAND_REPORT) {
    r.ok = false;
    r.error = "未知系统命令码";
    return r;
  }
  if ((in.request_id & 0x40000000u) != 0u) {
    r.ok = false;
    r.error = "request_id bit30 保留给固件自发脉冲，主机不得置位";
    return r;
  }
  // 规则逐字沿用 SystemCommand.msg 的头注释：SAVE 只接受整机；REPORT 忽略 motor_id；
  // 其余三条必须给有效 id。
  if (in.command == Cmd::COMMAND_SAVE && in.motor_id != Cmd::MOTOR_ID_NONE) {
    r.ok = false;
    r.error = "SAVE 只接受整机保存（MOTOR_ID_NONE）";
    return r;
  }
  if (in.command != Cmd::COMMAND_SAVE && in.command != Cmd::COMMAND_REPORT &&
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
  freshness.assign(can_bridge_msgs::msg::WireContract::MOTOR_COUNT_MAX, MotorFreshness{});
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
  for (size_t i = 0; i < n; ++i) {
    if (in.motor_ids[i] >= kIdLimit) {
      r.ok = false;
      r.error = "motor_id 越界（注册表下标 0..63）";
      return r;
    }
  }

  // ---- MotorStatusArray ----
  statuses.sync_time_ns = in.sync_time_ns;
  statuses.statuses.clear();
  for (size_t i = 0; i < n; ++i) {
    can_bridge_msgs::msg::MotorStatusTarget s;
    s.motor_id = in.motor_ids[i];
    // 这两个量不进 MotorStatusTarget（spec §4.7：旧消息字段级沿用现定义）；
    // 它们进 freshness 缓存，由 Task 10 在填充 motion_states 时盖章。
    // 0xFFFF = 从未收到 → 还原成 0xFFFFFFFF；其余毫秒直通（固件侧真实年龄
    // 饱和在 0xFFFE，不会与哨兵撞车）。
    freshness[in.motor_ids[i]].state_age_ms = (in.state_age_ms[i] == kMissingU16)
      ? 0xFFFFFFFFu : static_cast<uint32_t>(in.state_age_ms[i]);
    freshness[in.motor_ids[i]].target_source = in.target_source[i];
    s.lifecycle = in.lifecycle[i];
    s.unified_fault_code = in.unified_fault_code[i];
    s.vendor_fault_code = in.vendor_fault_code[i];
    s.protection_flags = in.protection_flags[i];
    s.limit_flags = in.limit_flags[i];
    // 电气量回 SI：facade 用 NaN 表示缺失（不是哨兵）。
    s.temperature = (in.temperature_dc[i] == kMissingI16)
      ? std::nan("") : static_cast<double>(in.temperature_dc[i]) * 0.1;
    s.current = (in.current_ma[i] == kMissingI16)
      ? std::nan("") : static_cast<double>(in.current_ma[i]) * 0.001;
    s.voltage = (in.voltage_mv[i] == kMissingU16)
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
  return r;
}
}  // namespace micro_ros_bridge
