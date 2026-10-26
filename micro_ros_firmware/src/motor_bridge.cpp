// 固件私有消息桥：只编码与调度，电机裁决仍由 motor_rt 唯一持有。
#include "motor_bridge.h"
#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rmw_microros/time_sync.h>
#include <rmw_microros/timing.h>
#include <rosidl_runtime_c/string_functions.h>
#include <can_bridge_msgs/msg/wire_contract.h>
#include <can_bridge_msgs/msg/low_command.h>
#include <can_bridge_msgs/msg/low_state.h>
#include <can_bridge_msgs/msg/motor_status.h>
#include <can_bridge_msgs/msg/motor_policy_array.h>
#include <can_bridge_msgs/msg/motor_trajectory.h>
#include <can_bridge_msgs/msg/motor_control_state_array.h>
#include <can_bridge_msgs/msg/motor_topology.h>
#include <can_bridge_msgs/msg/motor_event.h>
#include <can_bridge_msgs/msg/device_param_report.h>
#include <can_bridge_msgs/msg/config_result.h>
#include "motor_rt_task.h"
#include "imu_bridge.h"
#include "serial_transport.h"
#include "param_bridge.h"
#include "param_units.h"
#include "ros_qos.h"
#include "wire_codec.h"
#include "publish_schedule.h"

static constexpr size_t kRosArrayCapacity = MOTOR_CONTROL_MAX_MOTORS;
static constexpr size_t kChunkCapacity = can_bridge_msgs__msg__WireContract__LOW_STATE_CHUNK_MAX;
static uint64_t g_session_id = 0U;
static uint64_t g_last_session_id = 0U;
static wire_command_guard_t g_command_guard;
static uint32_t g_publish_fail = 0U;
static uint32_t g_bridge_event_dropped = 0U;
static uint32_t g_max_publish_us = 0U;
static uint32_t g_status_seq = 0U;
static uint32_t g_motion_seq = 0U;
static uint32_t g_status_tick = 0U, g_control_tick = 0U, g_topology_tick = 0U;
static uint32_t g_motion_tick = 0U;
static bool g_device_param_pending = false;
static rcl_subscription_t g_command_sub, g_trajectory_sub, g_policy_sub;
static can_bridge_msgs__msg__LowCommand g_command_msg;
static can_bridge_msgs__msg__MotorTrajectory g_trajectory_msg;
static can_bridge_msgs__msg__MotorPolicyArray g_policy_msg;
static bool g_command_up = false, g_trajectory_up = false, g_policy_up = false;
static bool g_command_added = false, g_trajectory_added = false, g_policy_added = false;
static uint8_t g_command_ids_buf[kRosArrayCapacity];
static int16_t g_command_position_buf[kRosArrayCapacity], g_command_velocity_buf[kRosArrayCapacity], g_command_effort_buf[kRosArrayCapacity];
static can_bridge_msgs__msg__MotorTrajectoryPoint g_trajectory_points_buf[MOTOR_CONTROL_MAX_TRAJECTORY_POINTS];
static uint16_t g_trajectory_target_ids_buf[MOTOR_CONTROL_MAX_TRAJECTORY_MOTORS];
static can_bridge_msgs__msg__MotorPolicyTarget g_policy_targets_buf[kRosArrayCapacity];
static can_bridge_msgs__msg__MotorControlState g_control_states_buf[kRosArrayCapacity];
static can_bridge_msgs__msg__MotorTopologyEntry g_topology_buf[kRosArrayCapacity];
static can_bridge_msgs__msg__DeviceParamEntry g_device_param_entries[kRosArrayCapacity];
static rcl_publisher_t g_motion_states_pub;
static can_bridge_msgs__msg__LowState g_motion_states_msg;
static bool g_motion_states_up = false;
static rcl_publisher_t g_status_pub;
static can_bridge_msgs__msg__MotorStatus g_status_msg;
static bool g_status_up = false;
static rcl_publisher_t g_control_states_pub;
static can_bridge_msgs__msg__MotorControlStateArray g_control_states_msg;
static bool g_control_states_up = false;
static rcl_publisher_t g_topology_pub;
static can_bridge_msgs__msg__MotorTopology g_topology_msg;
static bool g_topology_up = false;
static rcl_publisher_t g_events_pub;
static can_bridge_msgs__msg__MotorEvent g_event_msg;
static bool g_events_up = false;
static rcl_publisher_t g_device_param_pub;
static can_bridge_msgs__msg__DeviceParamReport g_device_param_msg;
static bool g_device_param_up = false;
static rcl_publisher_t g_config_result_pub;
static can_bridge_msgs__msg__ConfigResult g_config_result_msg;
static bool g_config_result_up = false;
static uint8_t g_low_motor_ids[kChunkCapacity];
static int16_t g_low_position_q[kChunkCapacity];
static int16_t g_low_velocity_q[kChunkCapacity];
static int16_t g_low_effort_q[kChunkCapacity];
static uint16_t g_low_state_age_ms[kChunkCapacity];
static uint8_t g_low_target_source[kChunkCapacity];
static uint8_t g_status_motor_ids[kRosArrayCapacity];
static uint16_t g_status_state_age_ms[kRosArrayCapacity];
static uint8_t g_status_target_source[kRosArrayCapacity];
static uint8_t g_status_lifecycle[kRosArrayCapacity];
static uint16_t g_status_unified_fault_code[kRosArrayCapacity];
static uint32_t g_status_vendor_fault_code[kRosArrayCapacity];
static uint16_t g_status_protection_flags[kRosArrayCapacity];
static uint16_t g_status_limit_flags[kRosArrayCapacity];
static int16_t g_status_temperature_dc[kRosArrayCapacity];
static int32_t g_status_current_ma[kRosArrayCapacity];
static uint32_t g_status_voltage_mv[kRosArrayCapacity];
static uint8_t g_status_drive_mode[kRosArrayCapacity];
static uint8_t g_status_status_flags[kRosArrayCapacity];
static uint8_t g_status_bus_index[kRosArrayCapacity];
static uint16_t g_status_traj_ids[MOTOR_CONTROL_MAX_TRAJECTORY_MOTORS];
static uint32_t g_ring_drop[MOTOR_CONTROL_MAX_BUSES], g_rx_lost[MOTOR_CONTROL_MAX_BUSES], g_tx_count[MOTOR_CONTROL_MAX_BUSES];
static motor_control_event_t g_events[16];
static size_t g_event_count = 0U;

static void note_publish(rcl_ret_t rc) {
  if (rc != RCL_RET_OK) ++g_publish_fail;
}

static bool send_reliable(rcl_publisher_t *pub, const void *msg) {
  const uint32_t start = micros();
  serial_transport_send_begin();
  const rcl_ret_t rc = rcl_publish(pub, msg, nullptr);
  serial_transport_send_end();
  const uint32_t duration = micros() - start;
  if (duration > g_max_publish_us) g_max_publish_us = duration;
  note_publish(rc);
  return rc == RCL_RET_OK;
}

uint64_t motor_bridge_session_id(void) { return g_session_id; }

bool motor_bridge_begin_session(uint64_t epoch_ns) {
  const uint64_t session = wire_session_from_epoch_ns(epoch_ns, g_last_session_id);
  if (session == 0U) return false;
  // 连接边界前的事件不属于新会话；周期状态会重新给出当前电平。
  motor_control_event_t stale[16];
  for (size_t i = 0U; i < 8U; ++i) {
    if (motor_rt_drain_events(stale, 16U) == 0U) break;
  }
  g_session_id = session;
  g_last_session_id = session;
  wire_command_guard_reset(&g_command_guard, session);
  g_status_seq = g_motion_seq = 0U;
  g_event_count = 0U;
  g_device_param_pending = true;
  const uint32_t now = millis();
  g_status_tick = g_control_tick = g_topology_tick = now - 2000U;
  g_motion_tick = now - PUBLISH_SCHEDULE_MOTION_MS;
  return true;
}

size_t motor_bridge_executor_handle_count(void) { return 3U; }

static void low_command_cb(const void *msgin) {
  if (msgin == nullptr) {
    return;
  }
  const auto *msg = (const can_bridge_msgs__msg__LowCommand *)msgin;
  // 声明长度与四组数组必须自洽；超内核容量一律整条拒绝。NaN/Inf 在编码端已被
  // 拒收，这里解出来的 -32768 只会来自畸形帧——mailbox 的 isfinite 会整条拒。
  if (!wire_frame_shape_ok(msg->count, msg->motor_ids.size,
      msg->position_q.size, msg->velocity_q.size, msg->effort_q.size,
      CONTROL_COMMAND_MAILBOX_MAX_TARGETS)) {
    return;
  }
  if (msg->count != 0U && (!wire_command_guard_allows(
      &g_command_guard, msg->session_id, msg->sequence) ||
      !wire_command_deadline_valid(millis(), msg->valid_until_tick_ms))) return;
  control_command_snapshot_t snapshot = {0};
  snapshot.sequence = msg->sequence;
  snapshot.received_tick_ms = millis();
  snapshot.target_count = msg->count;
  for (uint8_t i = 0; i < msg->count; ++i) {
    snapshot.targets[i].motor_id = msg->motor_ids.data[i];
    snapshot.targets[i].position = (float)wire_dequantize(
        msg->position_q.data[i],
        can_bridge_msgs__msg__WireContract__SCALE_POSITION_RAD);
    snapshot.targets[i].velocity = (float)wire_dequantize(
        msg->velocity_q.data[i],
        can_bridge_msgs__msg__WireContract__SCALE_VELOCITY_RAD_S);
    snapshot.targets[i].effort = (float)wire_dequantize(
        msg->effort_q.data[i],
        can_bridge_msgs__msg__WireContract__SCALE_EFFORT_NM);
  }
  if (motor_rt_submit_command(&snapshot) && msg->count != 0U) {
    wire_command_guard_commit(&g_command_guard, msg->sequence);
  }
}

static void trajectory_cb(const void *msgin) {
  if (msgin == nullptr) {
    return;
  }
  const auto *msg = (const can_bridge_msgs__msg__MotorTrajectory *)msgin;
  const bool stop = msg->count == 0U && msg->target_count == 0U;
  if (!stop && (g_session_id == 0U || msg->session_id != g_session_id)) return;
  // 分段元数据和声明计数必须在有界范围内：超界或分段编号非法一律拒绝，
  // 不做静默截断（截断会把一条轨迹悄悄变成另一条）。
  if (msg->segment_count == 0U ||
      msg->segment_count > CONTROL_TRAJECTORY_MAILBOX_MAX_SEGMENTS ||
      msg->segment_index >= msg->segment_count ||
      msg->count != msg->points.size ||
      msg->target_count != msg->target_ids.size ||
      msg->count > CONTROL_TRAJECTORY_MAILBOX_MAX_POINTS ||
      msg->target_count > CONTROL_TRAJECTORY_MAILBOX_MAX_MOTORS) {
    return;
  }
  // 5KB 级别，静态而非栈上（回调在 micro_ros 任务栈上跑）。
  static control_trajectory_t trajectory;
  memset(&trajectory, 0, sizeof(trajectory));
  trajectory.sequence = msg->sequence;
  trajectory.trajectory_id = msg->trajectory_id;
  trajectory.received_tick_ms = millis(); // 本地接收 tick（ARD §12.2）
  trajectory.segment_index = msg->segment_index;
  trajectory.segment_count = msg->segment_count;
  trajectory.point_count = msg->count;
  trajectory.target_count = msg->target_count;
  for (uint16_t i = 0; i < msg->target_count; ++i) {
    trajectory.target_ids[i] = msg->target_ids.data[i];
  }
  for (uint16_t i = 0; i < msg->count; ++i) {
    trajectory.points[i].time_from_start_ms =
        msg->points.data[i].time_from_start_ms;
    trajectory.points[i].motor_id = msg->points.data[i].motor_id;
    trajectory.points[i].position = msg->points.data[i].position;
    trajectory.points[i].velocity = msg->points.data[i].velocity;
    trajectory.points[i].effort = msg->points.data[i].effort;
  }
  (void)motor_rt_submit_trajectory(&trajectory); // 非法轨迹由 mailbox 拒绝并计数
}

static void policy_cb(const void *msgin) {
  if (msgin == nullptr) {
    return;
  }
  const auto *msg = (const can_bridge_msgs__msg__MotorPolicyArray *)msgin;
  bool stop = msg->count == msg->targets.size;
  for (size_t i = 0; i < msg->targets.size; ++i) {
    if (msg->targets.data[i].enabled) stop = false;
  }
  if (!stop && (g_session_id == 0U || msg->session_id != g_session_id)) return;
  // 声明计数必须在有界数组之内：超界一律拒绝，不做静默截断。
  if (msg->count != msg->targets.size ||
      msg->count > CONTROL_POLICY_MAILBOX_MAX_TARGETS) {
    return;
  }
  // 约 0.8KB，静态而非栈上（回调在 micro_ros 任务栈上跑）。
  static control_policy_snapshot_t snapshot;
  snapshot.received_tick_ms = millis(); // 本地接收 tick
  snapshot.request_id = msg->request_id;
  snapshot.policy_generation = msg->policy_generation;
  snapshot.target_count = msg->count;
  for (uint8_t i = 0; i < msg->count; ++i) {
    snapshot.targets[i].motor_id = msg->targets.data[i].motor_id;
    snapshot.targets[i].enabled = msg->targets.data[i].enabled;
    snapshot.targets[i].kp = msg->targets.data[i].kp;
    snapshot.targets[i].kd = msg->targets.data[i].kd;
    snapshot.targets[i].confirm_unprotected =
        msg->targets.data[i].confirm_unprotected;
  }
  (void)motor_rt_submit_policy(&snapshot); // 非法策略由 mailbox 整条拒绝并计数
}

static void publish_motion_states(uint32_t now_ms) {
  if ((uint32_t)(now_ms - g_motion_tick) < PUBLISH_SCHEDULE_MOTION_MS) return;
  g_motion_tick = now_ms;
  motor_control_motion_state_t states[kRosArrayCapacity];
  const size_t count = motor_rt_copy_motion_states(states, kRosArrayCapacity);
  const size_t chunks = count == 0U ? 1U : (count + kChunkCapacity - 1U) / kChunkCapacity;
  auto &m = g_motion_states_msg;
  m.sync_time_ns = rmw_uros_epoch_nanos();
  m.session_id = g_session_id;
  m.schema_version = can_bridge_msgs__msg__WireContract__SCHEMA_VERSION;
  m.seq = ++g_motion_seq;
  m.tick_ms = now_ms;
  m.total_count = (uint8_t)count;
  m.chunk_count = (uint8_t)chunks;
  for (size_t chunk = 0U; chunk < chunks; ++chunk) {
    const size_t offset = chunk * kChunkCapacity;
    const size_t n = count - offset < kChunkCapacity ? count - offset : kChunkCapacity;
    m.chunk_index = (uint8_t)chunk;
    m.count = (uint8_t)n;
    m.motor_ids.size = n;
    m.position_q.size = n;
    m.velocity_q.size = n;
    m.effort_q.size = n;
    m.state_age_ms.size = n;
    m.target_source.size = n;
    for (size_t i = 0U; i < n; ++i) {
      const auto &s = states[offset + i];
      g_low_motor_ids[i] = (uint8_t)s.motor_id;
      g_low_position_q[i] = wire_quantize(s.position_rad, can_bridge_msgs__msg__WireContract__SCALE_POSITION_RAD).q;
      g_low_velocity_q[i] = wire_quantize(s.velocity_rad_s, can_bridge_msgs__msg__WireContract__SCALE_VELOCITY_RAD_S).q;
      g_low_effort_q[i] = wire_quantize(s.effort_nm, can_bridge_msgs__msg__WireContract__SCALE_EFFORT_NM).q;
      g_low_state_age_ms[i] = wire_encode_state_age(s.state_age_ms);
      g_low_target_source[i] = s.target_source;
    }
    serial_transport_send_begin();
    note_publish(rcl_publish(&g_motion_states_pub, &m, nullptr));
    serial_transport_send_end();
  }
}

static void publish_control_states(uint32_t now_ms) {
  auto &m = g_control_states_msg;
  motor_control_status_t g_control_status_snapshot[kRosArrayCapacity];
  const size_t n = motor_rt_copy_statuses(g_control_status_snapshot, kRosArrayCapacity);
  m.session_id = g_session_id;
  m.schema_version = can_bridge_msgs__msg__MotorControlStateArray__SCHEMA_VERSION;
  m.sync_time_ns = rmw_uros_epoch_nanos();
  m.tick_ms = now_ms;
  ++m.sequence;
  m.states.size = n;
  for (size_t i = 0U; i < n; ++i) {
    const auto &s = g_control_status_snapshot[i];
    auto &out = g_control_states_buf[i];
    out.motor_id = s.motor_id;
    out.online = s.online;
    out.control_lifecycle = s.control_lifecycle;
    out.execution_state = s.execution_state;
    out.policy_enabled = s.policy_enabled;
    out.motion_enabled = s.motion_enabled;
    out.state_age_ms = s.state_age_ms;
    out.unified_fault_code = s.unified_fault_code;
    out.vendor_fault_code = s.vendor_fault_code;
    out.last_failure_reason = s.last_failure_reason;
    out.policy_generation = s.policy_generation;
  }
  send_reliable(&g_control_states_pub, &m);
}

static void publish_statuses(uint32_t now_ms) {
  auto &m = g_status_msg;
  motor_control_status_t g_control_status_snapshot[kRosArrayCapacity];
  const size_t n = motor_rt_copy_statuses(g_control_status_snapshot, kRosArrayCapacity);
  m.sync_time_ns = rmw_uros_epoch_nanos();
  m.session_id = g_session_id;
  m.schema_version = can_bridge_msgs__msg__WireContract__SCHEMA_VERSION;
  m.tick_ms = now_ms;
  m.sequence = ++g_status_seq;
  m.command_sequence = g_command_guard.sequence;
  m.config_request_id = param_bridge_request_high_water();
  m.count = (uint8_t)n;
  m.motor_ids.size = n;
  m.state_age_ms.size = n;
  m.target_source.size = n;
  m.lifecycle.size = n;
  m.unified_fault_code.size = n;
  m.vendor_fault_code.size = n;
  m.protection_flags.size = n;
  m.limit_flags.size = n;
  m.temperature_dc.size = n;
  m.current_ma.size = n;
  m.voltage_mv.size = n;
  m.drive_mode.size = n;
  m.status_flags.size = n;
  m.bus_index.size = n;
  motor_control_motion_state_t motion[kRosArrayCapacity];
  const size_t motion_count = motor_rt_copy_motion_states(motion, kRosArrayCapacity);
  for (size_t i = 0U; i < n; ++i) {
    const auto &s = g_control_status_snapshot[i];
    g_status_motor_ids[i] = s.motor_id;
    g_status_state_age_ms[i] = wire_encode_state_age(s.state_age_ms);
    g_status_target_source[i] = MOTOR_CONTROL_TARGET_SOURCE_NONE;
    for (size_t j = 0U; j < motion_count; ++j) {
      if (motion[j].motor_id == s.motor_id) g_status_target_source[i] = motion[j].target_source;
    }
    g_status_lifecycle[i] = s.lifecycle;
    g_status_unified_fault_code[i] = s.unified_fault_code;
    g_status_vendor_fault_code[i] = s.vendor_fault_code;
    g_status_protection_flags[i] = s.protection_flags;
    g_status_limit_flags[i] = s.limit_flags;
    g_status_temperature_dc[i] = wire_encode_temperature_dc(s.temperature_c);
    g_status_current_ma[i] = wire_encode_current_ma_i32(s.current_a);
    g_status_voltage_mv[i] = wire_encode_voltage_mv_u32(s.voltage_v);
    g_status_drive_mode[i] = s.drive_mode;
    g_status_status_flags[i] = wire_encode_status_flags({s.online, s.device_protect_known, s.device_protect_enabled, s.device_direction_known, s.device_direction});
    g_status_bus_index[i] = s.bus_index;
  }
  motor_control_trajectory_state_t trajectory = {};
  motor_rt_copy_trajectory_state(&trajectory);
  m.trajectory_id = trajectory.trajectory_id;
  m.traj_active = trajectory.active;
  m.traj_completed = trajectory.completed;
  m.traj_elapsed_ms = trajectory.elapsed_ms;
  m.traj_total_ms = trajectory.total_ms;
  m.traj_target_count = trajectory.target_count;
  m.traj_target_ids.size = trajectory.target_count;
  for (size_t i = 0U; i < trajectory.target_count; ++i) g_status_traj_ids[i] = trajectory.target_ids[i];
  const auto *system = param_bridge_system_status();
  m.last_error = system->last_error;
  m.last_error_motor = system->last_error_motor;
  m.refresh_seq = system->refresh_seq;
  m.set_zero_seq = system->set_zero_seq;
  m.clear_fault_seq = system->clear_fault_seq;
  m.save_seq = system->save_seq;
  m.save_in_progress = system->save_in_progress;
  motor_rt_diagnostics_t d = {};
  motor_rt_copy_diagnostics(&d);
  m.rx_frame_count = d.kernel.rx_frame_count;
  m.unknown_frame_count = d.kernel.unknown_frame_count;
  m.decode_error_count = d.kernel.decode_error_count;
  m.snapshot_accepted = d.kernel.snapshot_accepted;
  m.snapshot_rejected = d.kernel.snapshot_rejected;
  m.config_rejected = d.kernel.config_rejected;
  m.event_dropped = d.kernel.event_dropped + g_bridge_event_dropped;
  m.tx_dropped = d.kernel.tx_dropped;
  m.p0_retry = d.kernel.p0_retry;
  m.trajectory_accepted = d.kernel.trajectory_accepted;
  m.trajectory_rejected = d.kernel.trajectory_rejected;
  m.policy_accepted = d.kernel.policy_accepted;
  m.policy_rejected = d.kernel.policy_rejected;
  m.mailbox_rejected_command = d.mailbox_rejected_command;
  m.mailbox_rejected_trajectory = d.mailbox_rejected_trajectory;
  m.mailbox_rejected_policy = d.mailbox_rejected_policy;
  m.heap_free = d.heap_free;
  m.publish_fail = g_publish_fail + imu_bridge_publish_fail_count();
  m.publish_max_us = g_max_publish_us;
  m.ring_drop.size = m.rx_hw_lost.size = m.tx_frame_count.size = MOTOR_CONTROL_MAX_BUSES;
  for (size_t i = 0U; i < MOTOR_CONTROL_MAX_BUSES; ++i) {
    g_ring_drop[i] = d.ring_drop[i];
    g_rx_lost[i] = d.rx_hw_lost[i];
    g_tx_count[i] = d.tx_frame_count[i];
  }
  send_reliable(&g_status_pub, &m);
}

static void publish_topology(uint32_t now_ms) {

  motor_rt_topology_entry_t entries[kRosArrayCapacity];
  const size_t count =
      motor_rt_copy_topology(entries, sizeof(entries) / sizeof(entries[0]));
  const uint8_t out_count = (uint8_t)(count < kRosArrayCapacity ? count : kRosArrayCapacity);
  g_topology_msg.entries.size = out_count;
  for (uint8_t i = 0; i < out_count; ++i) {
    auto &out = g_topology_msg.entries.data[i];
    out.motor_id = entries[i].motor_id;
    out.bus_index = entries[i].bus_index;
    out.protocol = entries[i].protocol;
    (void)rosidl_runtime_c__String__assignn(&out.model_name,
        entries[i].model_name,
        strlen(entries[i].model_name));
    out.capability_flags = entries[i].capability_flags;
    out.online = entries[i].online;
    out.conflict = entries[i].conflict;
  }
  g_topology_msg.schema_version = can_bridge_msgs__msg__WireContract__SCHEMA_VERSION;
  g_topology_msg.session_id = g_session_id;
  g_topology_msg.sync_time_ns = rmw_uros_epoch_nanos();
  if (send_reliable(&g_topology_pub, &g_topology_msg)) g_topology_tick = now_ms;
}

static void publish_device_param_report() {

  size_t view_count = 0U;
  const motor_control_param_view_t *views = param_bridge_views(&view_count);
  if (views == nullptr) {
    return;
  }
  // count 会在合并「未发现但有板内记录」行时增长，不能 const。
  uint16_t count = (uint16_t)(view_count < kRosArrayCapacity
                                        ? view_count
                                        : kRosArrayCapacity);
  g_device_param_msg.entries.size = count;
  for (uint16_t i = 0U; i < count; ++i) {
    auto &out = g_device_param_msg.entries.data[i];
    const motor_control_param_view_t &v = views[i];
    out.motor_id = v.motor_id;
    out.dev_init_state = v.dev_init;
    out.record_dirty = v.record_dirty_slots != 0U;
    out.mismatch_mask = v.dev_mismatch_mask;
    out.rec_pos_min = v.record_si[MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN];
    out.rec_pos_max = v.record_si[MOTOR_PROTOCOL_PARAM_SLOT_POS_MAX];
    out.rec_vel_limit = v.record_si[MOTOR_PROTOCOL_PARAM_SLOT_VEL_MAX];
    // 力矩列按 **A** 呈现（设备对象 0x2538 存的就是电流），与 param_set 的 field=3
    // 同口径；record_si/dev_si 是关节侧 N·m，这里过一次换算——系数只有内核有
    // （v.torque_per_amp），页面不碰 mA。系数缺失时回 NaN，守住"缺失量一律 NaN"
    // （绝不落 0）。位置/速度列与内核同为 rad / rad/s，直通不换算。
    out.rec_torque_limit = param_units_nm_to_amp(
        v.record_si[MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX], v.torque_per_amp);
    out.rec_safety = v.record_si[MOTOR_PROTOCOL_PARAM_SLOT_SAFETY];
    out.rec_direction = v.record_si[MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION];
    out.dev_pos_min = v.dev_si[MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN];
    out.dev_pos_max = v.dev_si[MOTOR_PROTOCOL_PARAM_SLOT_POS_MAX];
    out.dev_vel_limit = v.dev_si[MOTOR_PROTOCOL_PARAM_SLOT_VEL_MAX];
    out.dev_torque_limit =
        param_units_nm_to_amp(v.dev_si[MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX],
        v.torque_per_amp);
    out.dev_safety = v.dev_si[MOTOR_PROTOCOL_PARAM_SLOT_SAFETY];
    out.dev_direction = v.dev_si[MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION];
  }

  // 补上「有 W25 记录但本次上电未发现」的电机——内核视图按契约不含未发现电机
  // （motor_control.h：未发现电机不在结果中），不补这一行的话总线空载时
  // REPORT 上来就是空表。补行只有记录列、dev_* 全 NaN、状态 UNVERIFIED。
  size_t stored_count = 0U;
  const motor_control_stored_params_t *stored =
      param_bridge_stored_records(&stored_count);
  for (size_t s = 0U; s < stored_count; ++s) {
    const motor_control_stored_params_t &r = stored[s];
    bool already = false;
    for (uint16_t i = 0U; i < count; ++i) {
      if (g_device_param_msg.entries.data[i].motor_id == r.motor_id) {
        already = true;
        break;
      }
    }
    if (already || count >= kRosArrayCapacity) {
      continue;
    }
    auto &out = g_device_param_msg.entries.data[count++];
    out.motor_id = r.motor_id;
    out.dev_init_state = MOTOR_CONTROL_DEV_INIT_UNVERIFIED;
    out.record_dirty = false;
    out.mismatch_mask = 0U;
    out.rec_pos_min = r.has_position_limits ? r.pos_min_rad : NAN;
    out.rec_pos_max = r.has_position_limits ? r.pos_max_rad : NAN;
    out.rec_vel_limit = r.has_velocity_limit ? r.velocity_limit_rad_s : NAN;
    // 力矩列按 **A** 呈现：stored 记录里有设备写入时的 mA 原值，直接折 A，
    // 不需要未绑定 Profile 拿不到的 torque_per_amp；原值缺失则 NaN。
    out.rec_torque_limit = NAN;
    if ((r.device_raw_present &
        (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX)) != 0U) {
      out.rec_torque_limit =
          (float)r.device_raw[MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX] / 1000.0f;
    }
    out.rec_safety = NAN;
    if ((r.device_raw_present &
        (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_SAFETY)) != 0U) {
      out.rec_safety =
          (float)r.device_raw[MOTOR_PROTOCOL_PARAM_SLOT_SAFETY];
    }
    out.rec_direction = NAN;
    if ((r.device_raw_present &
        (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION)) != 0U) {
      out.rec_direction =
          (float)r.device_raw[MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION];
    }
    // 未发现 = 本上电没与设备通过话：驱动板侧整行 NaN（绝不落 0）。
    out.dev_pos_min = NAN;
    out.dev_pos_max = NAN;
    out.dev_vel_limit = NAN;
    out.dev_torque_limit = NAN;
    out.dev_safety = NAN;
    out.dev_direction = NAN;
  }
  // 内核视图顺序 + 追加的记录行，按 id 排一遍保持页面行序稳定。
  for (uint16_t i = 1U; i < count; ++i) {
    for (uint16_t j = i; j > 0U; --j) {
      auto &a = g_device_param_msg.entries.data[j - 1U];
      auto &b = g_device_param_msg.entries.data[j];
      if (a.motor_id <= b.motor_id) {
        break;
      }
      auto tmp = a;
      a = b;
      b = tmp;
    }
  }
  // .msg 同时声明 count 与 entries（两者都上线），口径同
  // MotorTrajectoryState.target_count：不设它消费者会读到 0 条。
  g_device_param_msg.entries.size = count;
  g_device_param_msg.count = count;
  g_device_param_msg.session_id = g_session_id;
  g_device_param_msg.sync_time_ns = rmw_uros_epoch_nanos();
  g_device_param_pending = !send_reliable(&g_device_param_pub, &g_device_param_msg);
}

static void collect_events(void) {
  motor_control_event_t incoming[16];
  const size_t capacity = sizeof(g_events) / sizeof(g_events[0]);
  const size_t n = motor_rt_drain_events(incoming, 16U);
  for (size_t i = 0U; i < n; ++i) {
    // 事务收尾不受出站诊断事件积压影响。
    param_bridge_on_config_result(&incoming[i]);
    if (incoming[i].type == MOTOR_CONTROL_EVENT_CONFIG_RESULT ||
        incoming[i].type == MOTOR_CONTROL_EVENT_DEV_INIT) g_device_param_pending = true;
    if (g_event_count < capacity) {
      g_events[g_event_count++] = incoming[i];
    } else {
      ++g_bridge_event_dropped;
    }
  }
}

static void publish_event(void) {
  const auto &event = g_events[0];
  auto &m = g_event_msg;
  m.session_id = g_session_id;
  m.tick_ms = event.tick_ms;
  m.event_type = (uint8_t)event.type;
  m.motor_id = event.motor_id;
  m.bus_index = event.bus_index;
  m.reason = event.reason;
  m.request_id = event.request_id;
  m.event_seq = event.seq;
  if (send_reliable(&g_events_pub, &m)) {
    --g_event_count;
    memmove(g_events, g_events + 1, g_event_count * sizeof(g_events[0]));
  }
}

bool motor_bridge_create(rcl_node_t *node, rclc_executor_t *executor) {
  if (node == nullptr || executor == nullptr) return false;
  g_command_msg.motor_ids.capacity = kRosArrayCapacity;
  g_command_msg.motor_ids.data = g_command_ids_buf;
  g_command_msg.position_q.capacity = kRosArrayCapacity;
  g_command_msg.position_q.data = g_command_position_buf;
  g_command_msg.velocity_q.capacity = kRosArrayCapacity;
  g_command_msg.velocity_q.data = g_command_velocity_buf;
  g_command_msg.effort_q.capacity = kRosArrayCapacity;
  g_command_msg.effort_q.data = g_command_effort_buf;
  g_trajectory_msg.points.capacity = MOTOR_CONTROL_MAX_TRAJECTORY_POINTS;
  g_trajectory_msg.points.data = g_trajectory_points_buf;
  g_trajectory_msg.target_ids.capacity = MOTOR_CONTROL_MAX_TRAJECTORY_MOTORS;
  g_trajectory_msg.target_ids.data = g_trajectory_target_ids_buf;
  g_policy_msg.targets.capacity = kRosArrayCapacity;
  g_policy_msg.targets.data = g_policy_targets_buf;
  g_motion_states_msg.motor_ids.capacity = kChunkCapacity;
  g_motion_states_msg.motor_ids.data = g_low_motor_ids;
  g_motion_states_msg.position_q.capacity = kChunkCapacity;
  g_motion_states_msg.position_q.data = g_low_position_q;
  g_motion_states_msg.velocity_q.capacity = kChunkCapacity;
  g_motion_states_msg.velocity_q.data = g_low_velocity_q;
  g_motion_states_msg.effort_q.capacity = kChunkCapacity;
  g_motion_states_msg.effort_q.data = g_low_effort_q;
  g_motion_states_msg.state_age_ms.capacity = kChunkCapacity;
  g_motion_states_msg.state_age_ms.data = g_low_state_age_ms;
  g_motion_states_msg.target_source.capacity = kChunkCapacity;
  g_motion_states_msg.target_source.data = g_low_target_source;
  g_status_msg.motor_ids.capacity = kRosArrayCapacity;
  g_status_msg.motor_ids.data = g_status_motor_ids;
  g_status_msg.state_age_ms.capacity = kRosArrayCapacity;
  g_status_msg.state_age_ms.data = g_status_state_age_ms;
  g_status_msg.target_source.capacity = kRosArrayCapacity;
  g_status_msg.target_source.data = g_status_target_source;
  g_status_msg.lifecycle.capacity = kRosArrayCapacity;
  g_status_msg.lifecycle.data = g_status_lifecycle;
  g_status_msg.unified_fault_code.capacity = kRosArrayCapacity;
  g_status_msg.unified_fault_code.data = g_status_unified_fault_code;
  g_status_msg.vendor_fault_code.capacity = kRosArrayCapacity;
  g_status_msg.vendor_fault_code.data = g_status_vendor_fault_code;
  g_status_msg.protection_flags.capacity = kRosArrayCapacity;
  g_status_msg.protection_flags.data = g_status_protection_flags;
  g_status_msg.limit_flags.capacity = kRosArrayCapacity;
  g_status_msg.limit_flags.data = g_status_limit_flags;
  g_status_msg.temperature_dc.capacity = kRosArrayCapacity;
  g_status_msg.temperature_dc.data = g_status_temperature_dc;
  g_status_msg.current_ma.capacity = kRosArrayCapacity;
  g_status_msg.current_ma.data = g_status_current_ma;
  g_status_msg.voltage_mv.capacity = kRosArrayCapacity;
  g_status_msg.voltage_mv.data = g_status_voltage_mv;
  g_status_msg.drive_mode.capacity = kRosArrayCapacity;
  g_status_msg.drive_mode.data = g_status_drive_mode;
  g_status_msg.status_flags.capacity = kRosArrayCapacity;
  g_status_msg.status_flags.data = g_status_status_flags;
  g_status_msg.bus_index.capacity = kRosArrayCapacity;
  g_status_msg.bus_index.data = g_status_bus_index;
  g_status_msg.traj_target_ids.data = g_status_traj_ids;
  g_status_msg.traj_target_ids.capacity = MOTOR_CONTROL_MAX_TRAJECTORY_MOTORS;
  g_status_msg.ring_drop.data = g_ring_drop;
  g_status_msg.ring_drop.capacity = MOTOR_CONTROL_MAX_BUSES;
  g_status_msg.rx_hw_lost.data = g_rx_lost;
  g_status_msg.rx_hw_lost.capacity = MOTOR_CONTROL_MAX_BUSES;
  g_status_msg.tx_frame_count.data = g_tx_count;
  g_status_msg.tx_frame_count.capacity = MOTOR_CONTROL_MAX_BUSES;
  g_control_states_msg.states.data = g_control_states_buf;
  g_control_states_msg.states.capacity = kRosArrayCapacity;
  g_control_states_msg.sequence = 0U;
  g_topology_msg.entries.data = g_topology_buf;
  g_topology_msg.entries.capacity = kRosArrayCapacity;
  g_device_param_msg.entries.data = g_device_param_entries;
  g_device_param_msg.entries.capacity = kRosArrayCapacity;
  static bool strings_ready = false;
  if (!strings_ready) {
    for (size_t i = 0U; i < kRosArrayCapacity; ++i) {
      if (!rosidl_runtime_c__String__init(&g_topology_buf[i].model_name)) return false;
    }
    strings_ready = true;
  }
  rcl_subscription_options_t sub_opts = rcl_subscription_get_default_options();
  sub_opts.qos = qos_stream();
  if (rcl_subscription_init(&g_command_sub, node, ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg, LowCommand),
      can_bridge_msgs__msg__WireContract__TOPIC_LOW_CMD, &sub_opts) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor); return false;
  }
  g_command_up = true;
  sub_opts.qos = qos_reliable(1);
  if (rcl_subscription_init(&g_trajectory_sub, node, ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg, MotorTrajectory),
      can_bridge_msgs__msg__WireContract__TOPIC_TRAJECTORY, &sub_opts) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor); return false;
  }
  g_trajectory_up = true;
  sub_opts.qos = qos_reliable(1);
  if (rcl_subscription_init(&g_policy_sub, node, ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg, MotorPolicyArray),
      can_bridge_msgs__msg__WireContract__TOPIC_POLICY, &sub_opts) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor); return false;
  }
  g_policy_up = true;
  rcl_publisher_options_t pub_opts = rcl_publisher_get_default_options();
  pub_opts.qos = qos_stream();
  if (rcl_publisher_init(&g_motion_states_pub, node, ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg, LowState),
      can_bridge_msgs__msg__WireContract__TOPIC_LOW_STATE, &pub_opts) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor); return false;
  }
  g_motion_states_up = true;
  pub_opts.qos = qos_reliable(1);
  if (rcl_publisher_init(&g_status_pub, node, ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg, MotorStatus),
      can_bridge_msgs__msg__WireContract__TOPIC_MOTOR_STATUS, &pub_opts) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor); return false;
  }
  g_status_up = true;
  if (rmw_uros_set_publisher_session_timeout(rcl_publisher_get_rmw_handle(&g_status_pub), 1) != RMW_RET_OK) {
    motor_bridge_destroy(node, executor); return false;
  }
  pub_opts.qos = qos_reliable(1);
  if (rcl_publisher_init(&g_control_states_pub, node, ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg, MotorControlStateArray),
      can_bridge_msgs__msg__WireContract__TOPIC_CONTROL_STATES, &pub_opts) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor); return false;
  }
  g_control_states_up = true;
  if (rmw_uros_set_publisher_session_timeout(rcl_publisher_get_rmw_handle(&g_control_states_pub), 1) != RMW_RET_OK) {
    motor_bridge_destroy(node, executor); return false;
  }
  pub_opts.qos = qos_reliable(1);
  if (rcl_publisher_init(&g_topology_pub, node, ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg, MotorTopology),
      can_bridge_msgs__msg__WireContract__TOPIC_TOPOLOGY, &pub_opts) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor); return false;
  }
  g_topology_up = true;
  if (rmw_uros_set_publisher_session_timeout(rcl_publisher_get_rmw_handle(&g_topology_pub), 1) != RMW_RET_OK) {
    motor_bridge_destroy(node, executor); return false;
  }
  pub_opts.qos = qos_reliable(8);
  if (rcl_publisher_init(&g_events_pub, node, ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg, MotorEvent),
      can_bridge_msgs__msg__WireContract__TOPIC_EVENTS, &pub_opts) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor); return false;
  }
  g_events_up = true;
  if (rmw_uros_set_publisher_session_timeout(rcl_publisher_get_rmw_handle(&g_events_pub), 1) != RMW_RET_OK) {
    motor_bridge_destroy(node, executor); return false;
  }
  pub_opts.qos = qos_reliable(1);
  if (rcl_publisher_init(&g_device_param_pub, node, ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg, DeviceParamReport),
      can_bridge_msgs__msg__WireContract__TOPIC_DEVICE_PARAM, &pub_opts) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor); return false;
  }
  g_device_param_up = true;
  if (rmw_uros_set_publisher_session_timeout(rcl_publisher_get_rmw_handle(&g_device_param_pub), 1) != RMW_RET_OK) {
    motor_bridge_destroy(node, executor); return false;
  }
  pub_opts.qos = qos_reliable(1);
  if (rcl_publisher_init(&g_config_result_pub, node, ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg, ConfigResult),
      can_bridge_msgs__msg__WireContract__TOPIC_CONFIG_RESULT, &pub_opts) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor); return false;
  }
  g_config_result_up = true;
  if (rmw_uros_set_publisher_session_timeout(rcl_publisher_get_rmw_handle(&g_config_result_pub), 1) != RMW_RET_OK) {
    motor_bridge_destroy(node, executor); return false;
  }
  if (rclc_executor_add_subscription(executor, &g_command_sub, &g_command_msg, low_command_cb, ON_NEW_DATA) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor); return false;
  }
  g_command_added = true;
  if (rclc_executor_add_subscription(executor, &g_trajectory_sub, &g_trajectory_msg, trajectory_cb, ON_NEW_DATA) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor); return false;
  }
  g_trajectory_added = true;
  if (rclc_executor_add_subscription(executor, &g_policy_sub, &g_policy_msg, policy_cb, ON_NEW_DATA) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor); return false;
  }
  g_policy_added = true;
  return true;
}

void motor_bridge_destroy(rcl_node_t *node, rclc_executor_t *executor) {
  g_session_id = 0U;
  wire_command_guard_reset(&g_command_guard, 0U);
  if (executor != nullptr && g_command_added) {
    rclc_executor_remove_subscription(executor, &g_command_sub); g_command_added = false;
  }
  if (g_command_up) {
    const rcl_ret_t rc = rcl_subscription_fini(&g_command_sub, node); (void)rc; g_command_up = false;
  }
  if (executor != nullptr && g_trajectory_added) {
    rclc_executor_remove_subscription(executor, &g_trajectory_sub); g_trajectory_added = false;
  }
  if (g_trajectory_up) {
    const rcl_ret_t rc = rcl_subscription_fini(&g_trajectory_sub, node); (void)rc; g_trajectory_up = false;
  }
  if (executor != nullptr && g_policy_added) {
    rclc_executor_remove_subscription(executor, &g_policy_sub); g_policy_added = false;
  }
  if (g_policy_up) {
    const rcl_ret_t rc = rcl_subscription_fini(&g_policy_sub, node); (void)rc; g_policy_up = false;
  }
  if (g_motion_states_up) {
    const rcl_ret_t rc = rcl_publisher_fini(&g_motion_states_pub, node); (void)rc; g_motion_states_up = false;
  }
  if (g_status_up) {
    const rcl_ret_t rc = rcl_publisher_fini(&g_status_pub, node); (void)rc; g_status_up = false;
  }
  if (g_control_states_up) {
    const rcl_ret_t rc = rcl_publisher_fini(&g_control_states_pub, node); (void)rc; g_control_states_up = false;
  }
  if (g_topology_up) {
    const rcl_ret_t rc = rcl_publisher_fini(&g_topology_pub, node); (void)rc; g_topology_up = false;
  }
  if (g_events_up) {
    const rcl_ret_t rc = rcl_publisher_fini(&g_events_pub, node); (void)rc; g_events_up = false;
  }
  if (g_device_param_up) {
    const rcl_ret_t rc = rcl_publisher_fini(&g_device_param_pub, node); (void)rc; g_device_param_up = false;
  }
  if (g_config_result_up) {
    const rcl_ret_t rc = rcl_publisher_fini(&g_config_result_pub, node); (void)rc; g_config_result_up = false;
  }
  g_event_count = 0U;
}

void motor_bridge_poll(void) {
  if (g_session_id == 0U) return;
  const uint32_t now = millis();
  collect_events();
  param_bridge_poll();
  if (param_bridge_take_report_request()) g_device_param_pending = true;
  publish_motion_states(now);
  // 每轮最多一条 reliable；状态有固定预算，稀疏流公平轮转。
  const bool result_pending = param_bridge_result_peek(&g_config_result_msg);
  static publish_schedule_state_t schedule_state = {};
  const auto schedule = publish_schedule_step(
      (uint32_t)(now - g_control_tick) >= PUBLISH_SCHEDULE_CONTROL_MS,
      (uint32_t)(now - g_status_tick) >= PUBLISH_SCHEDULE_STATUS_MS,
      result_pending, g_event_count != 0U,
      (uint32_t)(now - g_topology_tick) >= 1000U,
      g_device_param_pending, &schedule_state);
  switch (schedule.reliable) {
    case PUBLISH_RELIABLE_CONTROL_STATES:
      g_control_tick = now; publish_control_states(now); break;
    case PUBLISH_RELIABLE_MOTOR_STATUS:
      g_status_tick = now; publish_statuses(now); break;
    case PUBLISH_RELIABLE_CONFIG_RESULT:
      if (send_reliable(&g_config_result_pub, &g_config_result_msg)) param_bridge_result_ack();
      break;
    case PUBLISH_RELIABLE_EVENT: publish_event(); break;
    case PUBLISH_RELIABLE_TOPOLOGY: publish_topology(now); break;
    case PUBLISH_RELIABLE_DEVICE_PARAM: publish_device_param_report(); break;
    default: break;
  }
}
