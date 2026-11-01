// V3 在线桥只校验、编码和调度；电机、租约与轨迹裁决仍由实时任务唯一持有。
#include "motor_bridge.h"

#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rmw_microros/timing.h>
#include <can_bridge_msgs/msg/agent_wire_contract.h>
#include <can_bridge_msgs/msg/agent_command.h>
#include <can_bridge_msgs/msg/agent_trajectory_request.h>
#include <can_bridge_msgs/msg/agent_motion.h>
#include <can_bridge_msgs/msg/agent_status.h>
#include <can_bridge_msgs/msg/agent_info.h>

#include "agent_protocol.h"
#include "motor_rt_task.h"
#include "imu_bridge.h"
#include "param_bridge.h"
#include "firmware_config.h"
#include "ros_qos.h"
#include "serial_transport.h"
#include "time_utils.h"
#include "wire_codec.h"

static constexpr size_t kMotorCapacity = can_bridge_msgs__msg__AgentWireContract__MOTOR_COUNT_MAX;
static constexpr size_t kChunkCapacity = can_bridge_msgs__msg__AgentWireContract__MOTION_CHUNK_MAX;
static constexpr size_t kTrajectoryPointCapacity = can_bridge_msgs__msg__AgentWireContract__TRAJECTORY_CHUNK_MAX;
static constexpr size_t kResultCapacity = can_bridge_msgs__msg__AgentWireContract__CONFIG_RESULTS_MAX;
static constexpr uint32_t kMotionIntervalMs = 20U;
static constexpr uint32_t kReliableIntervalMs = 10U;
static constexpr uint32_t kStatusMinIntervalMs = 20U;
static constexpr uint32_t kStatusIntervalMs = 100U;
static_assert(kMotorCapacity == MOTOR_CONTROL_MAX_MOTORS, "V3 电机容量不一致");
static_assert(kChunkCapacity == 32U && kMotorCapacity <= 2U * kChunkCapacity, "motion 每代最多两组");
static_assert(kTrajectoryPointCapacity == AGENT_TRAJECTORY_SEGMENT_POINTS, "轨迹接收容量不一致");
static_assert(MOTOR_CONTROL_MAX_BUSES <= 3U, "诊断总线数组容量不足");

static uint64_t g_session_id = 0U, g_last_session_id = 0U;
static uint32_t g_publish_fail = 0U;
static uint32_t g_motion_sequence = 0U, g_status_sequence = 0U;
static uint32_t g_motion_tick = 0U, g_status_tick = 0U;
static uint32_t g_status_attempt_tick = 0U, g_reliable_tick = 0U;
static uint32_t g_sent_status_fingerprint = 0U;
static uint8_t g_consecutive_status = 0U;
static bool g_status_dirty = true;
static bool g_have_sent_status = false;
static rcl_subscription_t g_command_sub, g_trajectory_sub;
static rcl_publisher_t g_motion_pub, g_status_pub, g_info_pub;
static bool g_command_up = false, g_trajectory_up = false;
static bool g_command_added = false, g_trajectory_added = false;
static bool g_motion_up = false, g_status_up = false, g_info_up = false;

// 接收区只绑定静态数组，拒绝异常 size/capacity/指针，不在回调扩容。
static can_bridge_msgs__msg__AgentCommand g_command_msg;
static uint8_t g_command_ids[kMotorCapacity];
static int16_t g_command_position[kMotorCapacity], g_command_velocity[kMotorCapacity], g_command_effort[kMotorCapacity];
static can_bridge_msgs__msg__AgentTrajectoryRequest g_trajectory_msg;
static uint16_t g_trajectory_ids[kMotorCapacity];
static can_bridge_msgs__msg__MotorTrajectoryPoint g_trajectory_points[kTrajectoryPointCapacity];
static control_command_snapshot_t g_command_snapshot;
static agent_trajectory_request_t g_trajectory_request;

static can_bridge_msgs__msg__AgentMotion g_motion_msg;
static uint8_t g_motion_ids[kChunkCapacity], g_motion_source[kChunkCapacity];
static int16_t g_motion_position[kChunkCapacity], g_motion_velocity[kChunkCapacity], g_motion_effort[kChunkCapacity];
static uint16_t g_motion_age[kChunkCapacity];
static motor_control_motion_state_t g_motion_snapshot[kMotorCapacity];
static can_bridge_msgs__msg__AgentStatus g_status_msg;
static can_bridge_msgs__msg__AgentMotorStatus g_status_motors[kMotorCapacity];
static can_bridge_msgs__msg__AgentConfigResult g_status_results[kResultCapacity];
static uint16_t g_status_trajectory_ids[kMotorCapacity];
static uint32_t g_ring_drop[MOTOR_CONTROL_MAX_BUSES], g_rx_lost[MOTOR_CONTROL_MAX_BUSES], g_tx_count[MOTOR_CONTROL_MAX_BUSES];
static motor_control_status_t g_status_snapshot[kMotorCapacity];

// 验证实际静态接收区，而不只检查声明 count；空载荷也检查形状。
template <typename Sequence, typename T>
static bool sequence_ok(const Sequence &sequence, const T *buffer, size_t capacity) {
  return sequence.data == buffer && sequence.capacity == capacity &&
      sequence.size <= sequence.capacity;
}

template <typename Sequence, typename T, size_t N>
static void bind_sequence(Sequence &sequence, T (&buffer)[N]) {
  sequence.data = buffer;
  sequence.size = 0U;
  sequence.capacity = N;
}

static bool valid_motor_id(uint16_t id) {
  return id > 0U && id <= can_bridge_msgs__msg__AgentWireContract__MOTOR_ID_MAX;
}

static void note_publish(rcl_ret_t rc) {
  if (rc != RCL_RET_OK) ++g_publish_fail;
}

static bool send_reliable(rcl_publisher_t *publisher, const void *message) {
  serial_transport_send_begin();
  const rcl_ret_t rc = rcl_publish(publisher, message, nullptr);
  serial_transport_send_end();
  note_publish(rc);
  return rc == RCL_RET_OK;
}

uint64_t motor_bridge_session_id(void) { return g_session_id; }
size_t motor_bridge_executor_handle_count(void) { return 2U; }

bool motor_bridge_begin_session(uint64_t epoch_ns) {
  const uint64_t session = wire_session_from_epoch_ns(epoch_ns, g_last_session_id);
  if (session == 0U) return false;
  // 连接边界前的事件不归新会话；清除数量有界，当前事实仍由状态快照恢复。
  motor_control_event_t stale[16];
  for (size_t batch = 0U; batch < 8U; ++batch) {
    if (motor_rt_drain_events(stale, 16U) == 0U) break;
  }
  g_session_id = g_last_session_id = session;
  motor_rt_begin_agent_session(session);
  g_motion_sequence = g_status_sequence = 0U;
  g_consecutive_status = 0U;
  g_status_dirty = true;
  g_have_sent_status = false;
  const uint32_t now = millis();
  g_motion_tick = now - kMotionIntervalMs;
  g_status_tick = now - kStatusIntervalMs;
  g_status_attempt_tick = now - kStatusMinIntervalMs;
  g_reliable_tick = now - kReliableIntervalMs;
  return true;
}

static void command_cb(const void *msgin) {
  if (msgin == nullptr) return;
  const auto &m = *static_cast<const can_bridge_msgs__msg__AgentCommand *>(msgin);
  if (m.schema_version != can_bridge_msgs__msg__AgentWireContract__SCHEMA_VERSION ||
      !sequence_ok(m.motor_ids, g_command_ids, kMotorCapacity) ||
      !sequence_ok(m.position_q, g_command_position, kMotorCapacity) ||
      !sequence_ok(m.velocity_q, g_command_velocity, kMotorCapacity) ||
      !sequence_ok(m.effort_q, g_command_effort, kMotorCapacity) ||
      !wire_frame_shape_ok(m.count, m.motor_ids.size, m.position_q.size,
          m.velocity_q.size, m.effort_q.size, kMotorCapacity)) return;
  if (m.mode == can_bridge_msgs__msg__AgentCommand__STOP_ALL) {
    // 停止必须为空，且先于会话/时钟门禁，绝不排入配置 FIFO。
    if (m.count == 0U) motor_rt_request_stop_all();
    return;
  }
  if (m.mode == can_bridge_msgs__msg__AgentCommand__AUTHORITY) {
    if (m.count == 0U && m.sequence == 0U && m.valid_until_tick_ms == 0U)
      (void)motor_rt_submit_control_lease(m.session_id, m.lease_id,
          m.lease_sequence, m.lease_valid_until_tick_ms, m.lease_active);
    return;
  }
  if (m.mode != can_bridge_msgs__msg__AgentCommand__SNAPSHOT ||
      m.lease_sequence || m.lease_valid_until_tick_ms || m.lease_active) return;
  const uint32_t now = millis();
  if (g_session_id == 0U || m.session_id != g_session_id ||
      !time_clock_valid() || !agent_deadline_valid(now, m.valid_until_tick_ms)) return;
  for (size_t i = 0U; i < m.count; ++i) {
    if (!valid_motor_id(m.motor_ids.data[i]) ||
        m.position_q.data[i] == WIRE_SENTINEL_MISSING ||
        m.velocity_q.data[i] == WIRE_SENTINEL_MISSING ||
        m.effort_q.data[i] == WIRE_SENTINEL_MISSING) return;
    for (size_t j = 0U; j < i; ++j) {
      if (m.motor_ids.data[j] == m.motor_ids.data[i]) return;
    }
  }
  auto &snapshot = g_command_snapshot;
  memset(&snapshot, 0, sizeof(snapshot));
  snapshot.session_id = m.session_id;
  snapshot.lease_id = m.lease_id;
  snapshot.sequence = m.sequence;
  snapshot.received_tick_ms = now;
  snapshot.valid_until_tick_ms = m.valid_until_tick_ms;
  snapshot.target_count = m.count;
  for (size_t i = 0U; i < m.count; ++i) {
    auto &target = snapshot.targets[i];
    target.motor_id = m.motor_ids.data[i];
    target.position = (float)wire_dequantize(m.position_q.data[i], can_bridge_msgs__msg__AgentWireContract__SCALE_POSITION);
    target.velocity = (float)wire_dequantize(m.velocity_q.data[i], can_bridge_msgs__msg__AgentWireContract__SCALE_VELOCITY);
    target.effort = (float)wire_dequantize(m.effort_q.data[i], can_bridge_msgs__msg__AgentWireContract__SCALE_EFFORT);
  }
  // 空 SNAPSHOT 仍走相同 RT 序号门禁；续租不等于取消活动轨迹。
  (void)motor_rt_submit_command(&snapshot);
}

static void trajectory_cb(const void *msgin) {
  if (msgin == nullptr) return;
  const auto &m = *static_cast<const can_bridge_msgs__msg__AgentTrajectoryRequest *>(msgin);
  if (m.schema_version != can_bridge_msgs__msg__AgentWireContract__SCHEMA_VERSION ||
      g_session_id == 0U || m.session_id != g_session_id ||
      !sequence_ok(m.target_ids, g_trajectory_ids, kMotorCapacity) ||
      !sequence_ok(m.points, g_trajectory_points, kTrajectoryPointCapacity) ||
      m.operation > can_bridge_msgs__msg__AgentTrajectoryRequest__ACK) return;
  const bool load = m.operation == can_bridge_msgs__msg__AgentTrajectoryRequest__LOAD;
  const bool commit = m.operation == can_bridge_msgs__msg__AgentTrajectoryRequest__COMMIT;
  const bool ack = m.operation == can_bridge_msgs__msg__AgentTrajectoryRequest__ACK;
  if (!load && (m.target_ids.size != 0U || m.points.size != 0U ||
      m.segment_index != 0U || m.segment_count != 0U)) return;
  if (ack) {
    if (m.request_id != 0U || m.trajectory_id != 0U ||
        m.issued_stamp != 0U || m.ttl_ms != 0U ||
        m.ack_request_id > AGENT_TRAJECTORY_MAX_ID ||
        m.ack_terminal_trajectory_id > AGENT_TRAJECTORY_MAX_ID) return;
  } else {
    if (m.ack_request_id != 0U || m.ack_terminal_trajectory_id != 0U ||
        m.request_id == 0U || m.request_id > AGENT_TRAJECTORY_MAX_ID ||
        m.trajectory_id == 0U || m.trajectory_id > AGENT_TRAJECTORY_MAX_ID) return;
  }
  if (load && (m.target_ids.size == 0U || m.points.size == 0U ||
      m.segment_count == 0U || m.segment_count > CONTROL_TRAJECTORY_MAILBOX_MAX_SEGMENTS ||
      m.segment_index >= m.segment_count)) return;

  uint32_t deadline = 0U;
  if (load || commit) {
    const uint32_t now = millis();
    uint64_t epoch = 0U;
    if (!time_sample_epoch(now, &epoch) ||
        !agent_request_deadline(m.issued_stamp, m.ttl_ms, 1000U,
            epoch, now, &deadline)) return;
  }
  // 只保留一个 32 点纯 C 接收转换区，不复制 256 点整条轨迹。
  auto &request = g_trajectory_request;
  memset(&request, 0, sizeof(request));
  request.session_id = m.session_id;
  request.lease_id = m.lease_id;
  request.request_id = m.request_id;
  request.trajectory_id = m.trajectory_id;
  request.valid_until_tick_ms = deadline;
  request.ack_request_id = m.ack_request_id;
  request.ack_terminal_trajectory_id = m.ack_terminal_trajectory_id;
  request.operation = m.operation;
  request.segment_index = m.segment_index;
  request.segment_count = m.segment_count;
  request.target_count = (uint16_t)m.target_ids.size;
  request.point_count = (uint16_t)m.points.size;
  for (size_t i = 0U; i < m.target_ids.size; ++i) {
    if (!valid_motor_id(m.target_ids.data[i])) return;
    for (size_t j = 0U; j < i; ++j) {
      if (m.target_ids.data[j] == m.target_ids.data[i]) return;
    }
    request.target_ids[i] = m.target_ids.data[i];
  }
  for (size_t i = 0U; i < m.points.size; ++i) {
    const auto &in = m.points.data[i];
    if (!valid_motor_id(in.motor_id) || !isfinite(in.position) ||
        !isfinite(in.velocity) || !isfinite(in.effort)) return;
    auto &out = request.points[i];
    out.time_from_start_ms = in.time_from_start_ms;
    out.motor_id = in.motor_id;
    out.position = in.position;
    out.velocity = in.velocity;
    out.effort = in.effort;
  }
  // CANCEL/ACK 不依赖时钟；收据、水位、去重和最终执行均归 RT。
  (void)motor_rt_submit_agent_trajectory(&request);
}

static void publish_motion(uint32_t now) {
  if ((uint32_t)(now - g_motion_tick) < 1000U / FIRMWARE_MOTOR_REPORT_HZ) return;
  g_motion_tick = now;
  const uint32_t capture_tick = millis();
  const size_t count = motor_rt_copy_motion_states(g_motion_snapshot, kMotorCapacity);
  auto &m = g_motion_msg;
  m.session_id = g_session_id;
  m.schema_version = can_bridge_msgs__msg__AgentWireContract__SCHEMA_VERSION;
  m.snapshot_tick_ms = capture_tick;
  m.snapshot_stamp = 0U;
  (void)time_sample_epoch(capture_tick, &m.snapshot_stamp);
  m.sequence = ++g_motion_sequence;
  const size_t chunks = count == 0U ? 1U : (count + kChunkCapacity - 1U) / kChunkCapacity;
  m.group_count = (uint8_t)chunks;
  // 两组允许同轮发出；序号、时间和数据都取自同一个只读快照。
  for (size_t chunk = 0U; chunk < chunks; ++chunk) {
    const size_t offset = chunk * kChunkCapacity;
    const size_t n = count - offset < kChunkCapacity ? count - offset : kChunkCapacity;
    m.group_index = (uint8_t)chunk;
    m.count = (uint8_t)n;
    m.motor_ids.size = m.position_q.size = m.velocity_q.size = m.effort_q.size = n;
    m.state_age_ms.size = m.target_source.size = n;
    for (size_t i = 0U; i < n; ++i) {
      const auto &state = g_motion_snapshot[offset + i];
      g_motion_ids[i] = (uint8_t)state.motor_id;
      g_motion_position[i] = wire_quantize(state.position_rad, can_bridge_msgs__msg__AgentWireContract__SCALE_POSITION).q;
      g_motion_velocity[i] = wire_quantize(state.velocity_rad_s, can_bridge_msgs__msg__AgentWireContract__SCALE_VELOCITY).q;
      g_motion_effort[i] = wire_quantize(state.effort_nm, can_bridge_msgs__msg__AgentWireContract__SCALE_EFFORT).q;
      g_motion_age[i] = wire_encode_state_age(state.state_age_ms);
      g_motion_source[i] = state.target_source;
    }
    serial_transport_send_begin();
    note_publish(rcl_publish(&g_motion_pub, &m, nullptr));
    serial_transport_send_end();
  }
}

static void collect_events(void) {
  motor_control_event_t events[16];
  // 每轮有界排空；配置结果先送事务协调器，再生成聚合状态。
  for (size_t batch = 0U; batch < 4U; ++batch) {
    const size_t count = motor_rt_drain_events(events, 16U);
    for (size_t i = 0U; i < count; ++i) param_bridge_on_config_result(&events[i]);
    if (count < 16U) break;
  }
}

static void fill_status(void) {
  auto &m = g_status_msg;
  const uint32_t capture_tick = millis();
  clock_health_t clock = {};
  time_clock_snapshot(&clock);
  m.session_id = g_session_id;
  m.schema_version = can_bridge_msgs__msg__AgentWireContract__SCHEMA_VERSION;
  m.snapshot_tick_ms = capture_tick;
  m.snapshot_stamp = 0U;
  m.clock_valid = clock_health_map_sample(&clock, capture_tick, capture_tick, &m.snapshot_stamp);
  m.clock_sync_age_ms = clock.have_sync ? capture_tick - clock.sync_tick_ms : UINT32_MAX;
  m.clock_sync_failures = clock.failures;
  m.clock_offset_ns = clock.offset_ns;
  motor_rt_link_status_t link = {};
  motor_rt_copy_link_status(&link);
  control_lease_state_t authority{};
  motor_rt_copy_control_lease(&authority);
  m.lease_id = authority.lease_id;
  m.lease_sequence = authority.sequence;
  m.lease_valid_until_tick_ms = authority.valid_until_tick_ms;
  m.lease_active = authority.active;
  m.command_sequence = link.command_sequence;
  m.stop_generation = link.stop_generation;
  m.required_policy_generation = link.required_policy_generation;
  m.lease_valid = link.lease_valid;
  const size_t count = motor_rt_copy_statuses(g_status_snapshot, kMotorCapacity);
  m.motors.size = count;
  for (size_t i = 0U; i < count; ++i) {
    const auto &in = g_status_snapshot[i];
    auto &out = g_status_motors[i];
    out.motor_id = (uint8_t)in.motor_id;
    out.lifecycle = in.control_lifecycle;
    out.execution_state = in.execution_state;
    out.online = in.online;
    out.policy_enabled = in.policy_enabled;
    out.kernel_enabled = in.motion_enabled;
    out.policy_generation = in.policy_generation;
    out.unified_fault_code = in.unified_fault_code;
    out.vendor_fault_code = in.vendor_fault_code;
    out.protection_flags = (uint16_t)in.protection_flags;
    out.limit_flags = (uint16_t)in.limit_flags;
    out.temperature_dc = wire_encode_temperature_dc(in.temperature_c);
    out.current_ma = wire_encode_current_ma_i32(in.current_a);
    out.voltage_mv = wire_encode_voltage_mv_u32(in.voltage_v);
    out.drive_mode = in.drive_mode;
    out.status_flags = wire_encode_status_flags({in.online, in.device_protect_known,
        in.device_protect_enabled, in.device_direction_known, in.device_direction});
    out.bus_index = in.bus_index;
  }
  agent_trajectory_status_t trajectory = {};
  motor_rt_copy_agent_trajectory(&trajectory);
  m.trajectory_request_high_water = trajectory.request_high_water;
  auto &receipt = m.trajectory_request_receipt;
  receipt.request_id = trajectory.receipt.request_id;
  receipt.trajectory_id = trajectory.receipt.trajectory_id;
  receipt.operation = trajectory.receipt.operation;
  receipt.accepted = trajectory.receipt.request_id != 0U &&
      (trajectory.receipt.state == AGENT_TRAJECTORY_PENDING ||
       trajectory.receipt.state == AGENT_TRAJECTORY_ACCEPTED);
  receipt.error = trajectory.receipt.reason;
  receipt.received_segments = trajectory.receipt.segments_received;
  auto &state = m.trajectory;
  state.trajectory_id = trajectory.trajectory_id;
  state.state = trajectory.state;
  state.reason = trajectory.reason;
  state.received_segments = trajectory.segments_received;
  state.total_segments = trajectory.segment_count;
  state.elapsed_ms = trajectory.elapsed_ms;
  state.total_ms = trajectory.duration_ms;
  state.target_ids.size = trajectory.target_count <= kMotorCapacity ? trajectory.target_count : 0U;
  for (size_t i = 0U; i < state.target_ids.size; ++i) g_status_trajectory_ids[i] = trajectory.target_ids[i];
  param_bridge_fill_status(&m);

  motor_rt_diagnostics_t diagnostics = {};
  motor_rt_copy_diagnostics(&diagnostics);
  auto &d = m.diagnostics;
  d.sync_time_ns = m.snapshot_stamp;
  d.rx_frame_count = diagnostics.kernel.rx_frame_count;
  d.unknown_frame_count = diagnostics.kernel.unknown_frame_count;
  d.decode_error_count = diagnostics.kernel.decode_error_count;
  d.snapshot_accepted = diagnostics.kernel.snapshot_accepted;
  d.snapshot_rejected = diagnostics.kernel.snapshot_rejected;
  d.config_rejected = diagnostics.kernel.config_rejected;
  d.event_dropped = diagnostics.kernel.event_dropped;
  d.tx_dropped = diagnostics.kernel.tx_dropped;
  d.p0_retry = diagnostics.kernel.p0_retry;
  d.trajectory_accepted = diagnostics.kernel.trajectory_accepted;
  d.trajectory_rejected = diagnostics.kernel.trajectory_rejected;
  d.policy_accepted = diagnostics.kernel.policy_accepted;
  d.policy_rejected = diagnostics.kernel.policy_rejected;
  d.mailbox_rejected_command = diagnostics.mailbox_rejected_command;
  d.mailbox_rejected_trajectory = diagnostics.mailbox_rejected_trajectory;
  d.mailbox_rejected_policy = diagnostics.mailbox_rejected_policy;
  d.heap_free = diagnostics.heap_free;
  d.publish_fail = g_publish_fail + imu_bridge_publish_fail_count();
  d.ring_drop.size = d.rx_hw_lost.size = d.tx_frame_count.size = MOTOR_CONTROL_MAX_BUSES;
  for (size_t i = 0U; i < MOTOR_CONTROL_MAX_BUSES; ++i) {
    g_ring_drop[i] = diagnostics.ring_drop[i];
    g_rx_lost[i] = diagnostics.rx_hw_lost[i];
    g_tx_count[i] = diagnostics.tx_frame_count[i];
  }
}

// 只对重要电平/收据做摘要，不将持续变化的命令序号、年龄、轨迹 elapsed 当事件。
static void fingerprint_add(uint32_t &hash, uint64_t value) {
  for (size_t byte = 0U; byte < sizeof(value); ++byte) {
    hash = (hash ^ (uint8_t)value) * 16777619U;
    value >>= 8U;
  }
}
static void fingerprint_result(uint32_t &hash, const can_bridge_msgs__msg__AgentConfigResult &result) {
  fingerprint_add(hash, result.request_id);
  fingerprint_add(hash, result.operation);
  fingerprint_add(hash, result.state);
  fingerprint_add(hash, result.error);
  fingerprint_add(hash, result.completed_motor_mask);
  fingerprint_add(hash, result.failed_motor_mask);
}
static uint32_t status_fingerprint(void) {
  const auto &m = g_status_msg;
  uint32_t hash = 2166136261U;
  fingerprint_add(hash, m.info_revision);
  fingerprint_add(hash, m.stop_generation);
  fingerprint_add(hash, m.required_policy_generation);
  fingerprint_add(hash, m.lease_valid);
  fingerprint_add(hash, m.clock_valid);
  fingerprint_add(hash, m.clock_sync_failures);
  fingerprint_add(hash, m.save_in_progress);
  fingerprint_add(hash, m.last_error);
  fingerprint_add(hash, m.last_error_motor);
  fingerprint_add(hash, m.config_request_high_water);
  fingerprint_add(hash, m.results.size);
  for (size_t i = 0U; i < m.results.size; ++i) fingerprint_result(hash, m.results.data[i]);
  fingerprint_result(hash, m.rejection_receipt);
  fingerprint_add(hash, m.trajectory_request_high_water);
  const auto &receipt = m.trajectory_request_receipt;
  fingerprint_add(hash, receipt.request_id);
  fingerprint_add(hash, receipt.trajectory_id);
  fingerprint_add(hash, receipt.operation);
  fingerprint_add(hash, receipt.accepted);
  fingerprint_add(hash, receipt.error);
  fingerprint_add(hash, receipt.received_segments);
  fingerprint_add(hash, m.trajectory.trajectory_id);
  fingerprint_add(hash, m.trajectory.state);
  fingerprint_add(hash, m.trajectory.reason);
  fingerprint_add(hash, m.trajectory.received_segments);
  fingerprint_add(hash, m.motors.size);
  for (size_t i = 0U; i < m.motors.size; ++i) {
    const auto &motor = m.motors.data[i];
    fingerprint_add(hash, motor.motor_id);
    fingerprint_add(hash, motor.lifecycle);
    fingerprint_add(hash, motor.execution_state);
    fingerprint_add(hash, motor.online);
    fingerprint_add(hash, motor.policy_enabled);
    fingerprint_add(hash, motor.kernel_enabled);
    fingerprint_add(hash, motor.policy_generation);
    fingerprint_add(hash, motor.unified_fault_code);
    fingerprint_add(hash, motor.vendor_fault_code);
    fingerprint_add(hash, motor.protection_flags);
    fingerprint_add(hash, motor.limit_flags);
  }
  return hash;
}

bool motor_bridge_create(rcl_node_t *node, rclc_executor_t *executor) {
  if (node == nullptr || executor == nullptr) return false;
  // 所有 ROS 序列在实体注册前完成绑定；不调用会分配的消息 init/resize。
  bind_sequence(g_command_msg.motor_ids, g_command_ids);
  bind_sequence(g_command_msg.position_q, g_command_position);
  bind_sequence(g_command_msg.velocity_q, g_command_velocity);
  bind_sequence(g_command_msg.effort_q, g_command_effort);
  bind_sequence(g_trajectory_msg.target_ids, g_trajectory_ids);
  bind_sequence(g_trajectory_msg.points, g_trajectory_points);
  bind_sequence(g_motion_msg.motor_ids, g_motion_ids);
  bind_sequence(g_motion_msg.position_q, g_motion_position);
  bind_sequence(g_motion_msg.velocity_q, g_motion_velocity);
  bind_sequence(g_motion_msg.effort_q, g_motion_effort);
  bind_sequence(g_motion_msg.state_age_ms, g_motion_age);
  bind_sequence(g_motion_msg.target_source, g_motion_source);
  bind_sequence(g_status_msg.motors, g_status_motors);
  bind_sequence(g_status_msg.results, g_status_results);
  bind_sequence(g_status_msg.trajectory.target_ids, g_status_trajectory_ids);
  bind_sequence(g_status_msg.diagnostics.ring_drop, g_ring_drop);
  bind_sequence(g_status_msg.diagnostics.rx_hw_lost, g_rx_lost);
  bind_sequence(g_status_msg.diagnostics.tx_frame_count, g_tx_count);

  rcl_subscription_options_t sub_options = rcl_subscription_get_default_options();
  sub_options.qos = qos_stream();
  if (rcl_subscription_init(&g_command_sub, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg, AgentCommand),
      can_bridge_msgs__msg__AgentWireContract__TOPIC_COMMAND, &sub_options) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor); return false;
  }
  g_command_up = true;
  sub_options.qos = qos_reliable(1);
  if (rcl_subscription_init(&g_trajectory_sub, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg, AgentTrajectoryRequest),
      can_bridge_msgs__msg__AgentWireContract__TOPIC_TRAJECTORY, &sub_options) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor); return false;
  }
  g_trajectory_up = true;
  rcl_publisher_options_t pub_options = rcl_publisher_get_default_options();
  pub_options.qos = qos_stream();
  if (rcl_publisher_init(&g_motion_pub, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg, AgentMotion),
      can_bridge_msgs__msg__AgentWireContract__TOPIC_MOTION, &pub_options) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor); return false;
  }
  g_motion_up = true;
  pub_options.qos = qos_reliable(1);
  if (rcl_publisher_init(&g_status_pub, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg, AgentStatus),
      can_bridge_msgs__msg__AgentWireContract__TOPIC_STATUS, &pub_options) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor); return false;
  }
  g_status_up = true;
  if (rmw_uros_set_publisher_session_timeout(rcl_publisher_get_rmw_handle(&g_status_pub), 1) != RMW_RET_OK) {
    motor_bridge_destroy(node, executor); return false;
  }
  pub_options.qos = qos_reliable(1);
  if (rcl_publisher_init(&g_info_pub, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg, AgentInfo),
      can_bridge_msgs__msg__AgentWireContract__TOPIC_INFO, &pub_options) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor); return false;
  }
  g_info_up = true;
  if (rmw_uros_set_publisher_session_timeout(rcl_publisher_get_rmw_handle(&g_info_pub), 1) != RMW_RET_OK) {
    motor_bridge_destroy(node, executor); return false;
  }
  if (rclc_executor_add_subscription(executor, &g_command_sub, &g_command_msg,
      command_cb, ON_NEW_DATA) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor); return false;
  }
  g_command_added = true;
  if (rclc_executor_add_subscription(executor, &g_trajectory_sub, &g_trajectory_msg,
      trajectory_cb, ON_NEW_DATA) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor); return false;
  }
  g_trajectory_added = true;
  return true;
}

void motor_bridge_destroy(rcl_node_t *node, rclc_executor_t *executor) {
  g_session_id = 0U;
  if (executor != nullptr && g_command_added) {
    const rcl_ret_t rc = rclc_executor_remove_subscription(executor, &g_command_sub); (void)rc;
    g_command_added = false;
  }
  if (g_command_up) {
    const rcl_ret_t rc = rcl_subscription_fini(&g_command_sub, node); (void)rc; g_command_up = false;
  }
  if (executor != nullptr && g_trajectory_added) {
    const rcl_ret_t rc = rclc_executor_remove_subscription(executor, &g_trajectory_sub); (void)rc;
    g_trajectory_added = false;
  }
  if (g_trajectory_up) {
    const rcl_ret_t rc = rcl_subscription_fini(&g_trajectory_sub, node); (void)rc; g_trajectory_up = false;
  }
  if (g_motion_up) {
    const rcl_ret_t rc = rcl_publisher_fini(&g_motion_pub, node); (void)rc; g_motion_up = false;
  }
  if (g_status_up) {
    const rcl_ret_t rc = rcl_publisher_fini(&g_status_pub, node); (void)rc; g_status_up = false;
  }
  if (g_info_up) {
    const rcl_ret_t rc = rcl_publisher_fini(&g_info_pub, node); (void)rc; g_info_up = false;
  }
}

void motor_bridge_poll(void) {
  if (g_session_id == 0U) return;
  const uint32_t now = millis();
  collect_events();
  param_bridge_poll();
  publish_motion(now);
  fill_status();
  const uint32_t fingerprint = status_fingerprint();
  if (!g_have_sent_status || fingerprint != g_sent_status_fingerprint) g_status_dirty = true;
  const uint32_t send_tick = millis();
  const bool status_due = (uint32_t)(send_tick - g_status_attempt_tick) >= kStatusMinIntervalMs &&
      (g_status_dirty || (uint32_t)(send_tick - g_status_tick) >= 1000U / FIRMWARE_MOTOR_REPORT_HZ);
  const auto *info = param_bridge_info();
  if ((uint32_t)(send_tick - g_reliable_tick) < kReliableIntervalMs) return;
  // 每 10ms 最多一次可靠应用发布；连续两轮 status 后必须给待发 info 一轮。
  if (status_due && (info == nullptr || g_consecutive_status < 2U)) {
    g_reliable_tick = g_status_attempt_tick = send_tick;
    if (g_consecutive_status < 2U) ++g_consecutive_status;
    g_status_msg.sequence = ++g_status_sequence;
    if (send_reliable(&g_status_pub, &g_status_msg)) {
      g_sent_status_fingerprint = fingerprint;
      g_have_sent_status = true;
      g_status_dirty = false;
      g_status_tick = send_tick;
    } else {
      g_status_dirty = true;
    }
  } else if (info != nullptr) {
    g_reliable_tick = send_tick;
    g_consecutive_status = 0U;
    param_bridge_info_sent(send_reliable(&g_info_pub, info));
  }
}
