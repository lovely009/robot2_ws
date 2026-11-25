// 原生电机域 ROS 2 桥（Schema 6，37 电机单帧直发，标准 Header 时间戳）。
#include "motor_bridge.h"

#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rmw_microros/timing.h>
#include <std_msgs/msg/empty.h>
#include <std_msgs/msg/int32.h>
#include <can_bridge_msgs/msg/motor_command_array.h>
#include <can_bridge_msgs/msg/motor_motion_state_array.h>
#include <can_bridge_msgs/msg/motor_system_state.h>
#include <can_bridge_msgs/msg/motor_event.h>

#include "agent_protocol.h"
#include "motor_rt_task.h"
#include "imu_bridge.h"
#include "param_bridge.h"
#include "firmware_config.h"
#include "ros_bridge.h"
#include "ros_qos.h"
#include "micro_ros_task.h"
#include "serial_transport.h"
#include "time_utils.h"

static constexpr size_t kMotorCapacity = FIRMWARE_MAX_MOTORS;
static constexpr size_t kDiagnosticBusCount =
    can_bridge_msgs__msg__MotorDiagnostics__ring_drop__MAX_SIZE;
static constexpr uint32_t kLeaseTimeoutMs = 250U;   // <= CONTROL_LEASE_MAX_MS (300ms)
static constexpr uint32_t kCommandTimeoutMs = AGENT_COMMAND_SOFT_STOP_MS;
static constexpr uint32_t kStatusMinIntervalMs = 20U;

static uint64_t g_session_id = 0U, g_last_session_id = 0U;
static uint32_t g_publish_fail = 0U;
static uint32_t g_motion_sequence = 0U, g_status_sequence = 0U;
static uint32_t g_motion_tick = 0U, g_status_tick = 0U;
static uint32_t g_sent_status_fingerprint = 0U;
static bool g_status_dirty = true;
static bool g_have_sent_status = false;
static uint32_t g_lease_seq = 0U;
static uint64_t g_lease_id = 0U;
static uint32_t g_lease_epoch = 0U;

static rcl_subscription_t g_command_sub, g_stop_sub, g_client_sub;
static rcl_publisher_t g_motion_pub, g_state_pub, g_event_pub;
static bool g_command_up = false, g_stop_up = false, g_client_up = false;
static bool g_command_added = false, g_stop_added = false, g_client_added = false;
static bool g_motion_up = false, g_state_up = false, g_event_up = false;

static char kFrameId[] = "micro_ros_node";
static char g_cmd_frame_id[32];

static can_bridge_msgs__msg__MotorCommandArray g_command_msg;
static can_bridge_msgs__msg__MotorCommandTarget g_command_targets[kMotorCapacity];
static control_command_snapshot_t g_command_snapshot;
static std_msgs__msg__Empty g_stop_msg;
static std_msgs__msg__Int32 g_client_msg;

static can_bridge_msgs__msg__MotorMotionStateArray g_motion_msg;
static can_bridge_msgs__msg__MotorMotionStateTarget g_motion_states[kMotorCapacity];
static motor_control_motion_state_t g_motion_snapshot[kMotorCapacity];

static can_bridge_msgs__msg__MotorSystemState g_state_msg;
static can_bridge_msgs__msg__MotorControlState g_control_states[kMotorCapacity];
static can_bridge_msgs__msg__MotorStatusTarget g_statuses[kMotorCapacity];
static uint32_t g_ring_drop[kDiagnosticBusCount], g_rx_lost[kDiagnosticBusCount],
    g_tx_count[kDiagnosticBusCount];
static motor_control_status_t g_status_snapshot[kMotorCapacity];

static can_bridge_msgs__msg__MotorEvent g_event_msg;

template <typename Sequence, typename T, size_t N>
static void bind_sequence(Sequence &sequence, T (&buffer)[N]) {
  sequence.data = buffer;
  sequence.size = 0U;
  sequence.capacity = N;
}

static void bind_frame_id(std_msgs__msg__Header &header) {
  header.frame_id.data = kFrameId;
  header.frame_id.size = sizeof(kFrameId) - 1U;
  header.frame_id.capacity = sizeof(kFrameId);
}

static bool valid_motor_id(uint16_t id) {
  return id > 0U && id <= 64U;
}

static void note_publish(rcl_ret_t rc) {
  if (rc != RCL_RET_OK) ++g_publish_fail;
}

static uint64_t session_from_epoch_ns(uint64_t epoch_ns, uint64_t previous) {
  uint64_t session = epoch_ns / 1000000ULL;
  if (session == 0U) session = 1U;
  if (session <= previous) session = previous + 1U;
  return session;
}

uint64_t motor_bridge_session_id(void) { return g_session_id; }

uint64_t motor_bridge_ensure_lease(uint32_t now_ms) {
  if (g_session_id == 0U || g_session_id > UINT64_MAX / CONTROL_LEASE_SESSION_STRIDE) {
    return 0U;
  }
  control_lease_state_t auth{};
  motor_rt_copy_control_lease(&auth);
  if (!auth.active || g_lease_id == 0U || auth.lease_id != g_lease_id) {
    if (++g_lease_epoch == 0U || g_lease_epoch >= CONTROL_LEASE_SESSION_STRIDE) {
      g_lease_epoch = 1U;
    }
    g_lease_id = g_session_id * CONTROL_LEASE_SESSION_STRIDE + g_lease_epoch;
    g_lease_seq = 0U;
  }
  if (++g_lease_seq == 0U) g_lease_seq = 1U;
  if (!motor_rt_submit_control_lease(
      g_session_id, g_lease_id, g_lease_seq, now_ms + kLeaseTimeoutMs, true)) {
    return 0U;
  }
  return g_lease_id;
}

size_t motor_bridge_executor_handle_count(void) { return 3U; }

bool motor_bridge_begin_session(uint64_t epoch_ns) {
  const uint64_t session = session_from_epoch_ns(epoch_ns, g_last_session_id);
  motor_control_event_t stale[16];
  for (size_t batch = 0U; batch < 8U; ++batch) {
    if (motor_rt_drain_events(stale, 16U) == 0U) break;
  }
  g_session_id = g_last_session_id = session;
  motor_rt_begin_agent_session(session);
  g_motion_sequence = g_status_sequence = g_lease_seq = 0U;
  g_lease_id = 0U;
  g_lease_epoch = 0U;
  g_status_dirty = true;
  g_have_sent_status = false;
  const uint32_t now = millis();
  g_motion_tick = now - FIRMWARE_MOTOR_REPORT_PERIOD_MS;
  g_status_tick = now - FIRMWARE_STATUS_REPORT_PERIOD_MS;
  return true;
}

static void command_cb(const void *msgin) {
  if (msgin == nullptr || !ros_bridge_is_active()) return;
  const auto &m = *static_cast<const can_bridge_msgs__msg__MotorCommandArray *>(msgin);
  if (m.targets.data != g_command_targets || m.targets.capacity != kMotorCapacity ||
      m.targets.size > kMotorCapacity || m.count != m.targets.size) return;
  const uint32_t now = millis();
  if (g_session_id == 0U || m.session_id != g_session_id || !time_clock_valid()) return;
  for (size_t i = 0U; i < m.count; ++i) {
    const auto &t = m.targets.data[i];
    if (!valid_motor_id(t.motor_id) ||
        !isfinite(t.position) || !isfinite(t.velocity) || !isfinite(t.effort)) return;
    for (size_t j = 0U; j < i; ++j) {
      if (m.targets.data[j].motor_id == t.motor_id) return;
    }
  }
  const uint64_t lease_id = motor_bridge_ensure_lease(now);
  if (lease_id == 0U) return;
  auto &snapshot = g_command_snapshot;
  memset(&snapshot, 0, sizeof(snapshot));
  snapshot.session_id = m.session_id;
  snapshot.lease_id = lease_id;
  snapshot.lease_sequence = 0U;
  snapshot.lease_valid_until_tick_ms = 0U;
  snapshot.sequence = m.sequence;
  snapshot.received_tick_ms = now;
  snapshot.valid_until_tick_ms = now + kCommandTimeoutMs;
  snapshot.target_count = m.count;
  for (size_t i = 0U; i < m.count; ++i) {
    snapshot.targets[i].motor_id = m.targets.data[i].motor_id;
    snapshot.targets[i].position = m.targets.data[i].position;
    snapshot.targets[i].velocity = m.targets.data[i].velocity;
    snapshot.targets[i].effort = m.targets.data[i].effort;
  }
  if (motor_rt_submit_command(&snapshot)) micro_ros_note_agent_rx();
}

static void stop_cb(const void *msgin) {
  (void)msgin;
  motor_rt_request_stop_all();
  g_status_dirty = true;
}

static void client_count_cb(const void *msgin) {
  if (msgin == nullptr) return;
  const auto &m = *static_cast<const std_msgs__msg__Int32 *>(msgin);
  if (m.data <= 0) {
    motor_rt_request_stop_all();
    g_status_dirty = true;
  }
}

static void publish_motion(uint32_t now) {
  if ((uint32_t)(now - g_motion_tick) < FIRMWARE_MOTOR_REPORT_PERIOD_MS) return;
  g_motion_tick = now;
  const uint32_t capture_tick = millis();
  const size_t count = motor_rt_copy_motion_states(g_motion_snapshot, kMotorCapacity);
  auto &m = g_motion_msg;
  uint64_t epoch_ns = 0U;
  (void)time_sample_epoch(capture_tick, &epoch_ns);
  fill_epoch_stamp(&m.header.stamp, epoch_ns);
  bind_frame_id(m.header);
  m.session_id = g_session_id;
  m.tick_ms = capture_tick;
  m.seq = ++g_motion_sequence;
  m.states.size = count <= kMotorCapacity ? count : kMotorCapacity;
  for (size_t i = 0U; i < m.states.size; ++i) {
    const auto &in = g_motion_snapshot[i];
    auto &out = g_motion_states[i];
    out.motor_id = in.motor_id;
    out.position = in.position_rad;
    out.velocity = in.velocity_rad_s;
    out.effort = in.effort_nm;
    out.state_age_ms = in.state_age_ms;
    out.target_source = in.target_source;
  }
  serial_transport_send_begin();
  note_publish(rcl_publish(&g_motion_pub, &m, nullptr));
  serial_transport_send_end();
}

static void publish_events(void) {
  motor_control_event_t events[16];
  for (size_t batch = 0U; batch < 2U; ++batch) {
    const size_t count = motor_rt_drain_events(events, 16U);
    for (size_t i = 0U; i < count; ++i) {
      const auto &e = events[i];
      if (e.type == MOTOR_CONTROL_EVENT_CONFIG_RESULT) {
        param_bridge_on_config_result(&e);
      }
      uint64_t epoch_ns = 0U;
      (void)time_sample_epoch(e.tick_ms, &epoch_ns);
      fill_epoch_stamp(&g_event_msg.header.stamp, epoch_ns);
      bind_frame_id(g_event_msg.header);
      g_event_msg.tick_ms = e.tick_ms;
      g_event_msg.session_id = g_session_id;
      g_event_msg.event_type = e.type;
      g_event_msg.motor_id = e.motor_id;
      g_event_msg.bus_index = e.bus_index;
      g_event_msg.reason = e.reason;
      g_event_msg.request_id = e.request_id;
      g_event_msg.event_seq = e.seq;
      serial_transport_send_begin();
      note_publish(rcl_publish(&g_event_pub, &g_event_msg, nullptr));
      serial_transport_send_end();
    }
    if (count < 16U) break;
  }
}

static void fill_state(void) {
  auto &m = g_state_msg;
  const uint32_t capture_tick = millis();
  clock_health_t clock = {};
  time_clock_snapshot(&clock);
  uint64_t epoch_ns = 0U;
  m.clock_valid = clock_health_map_sample(&clock, capture_tick, capture_tick, &epoch_ns);
  fill_epoch_stamp(&m.header.stamp, epoch_ns);
  bind_frame_id(m.header);
  m.schema_version = can_bridge_msgs__msg__MotorSystemState__SCHEMA_VERSION;
  m.session_id = g_session_id;
  m.snapshot_tick_ms = capture_tick;
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
  m.control_states.size = count <= kMotorCapacity ? count : kMotorCapacity;
  m.statuses.size = m.control_states.size;
  for (size_t i = 0U; i < m.control_states.size; ++i) {
    const auto &in = g_status_snapshot[i];
    auto &cs = g_control_states[i];
    cs.motor_id = in.motor_id;
    cs.online = in.online;
    cs.control_lifecycle = in.control_lifecycle;
    cs.execution_state = in.execution_state;
    cs.policy_enabled = in.policy_enabled;
    cs.motion_enabled = in.motion_enabled;
    cs.state_age_ms = in.state_age_ms;
    cs.unified_fault_code = in.unified_fault_code;
    cs.vendor_fault_code = in.vendor_fault_code;
    cs.last_failure_reason = in.last_failure_reason;
    cs.policy_generation = in.policy_generation;

    auto &st = g_statuses[i];
    st.motor_id = in.motor_id;
    st.temperature = in.temperature_c;
    st.current = in.current_a;
    st.voltage = in.voltage_v;
    st.drive_mode = in.drive_mode;
    st.unified_fault_code = in.unified_fault_code;
    st.vendor_fault_code = in.vendor_fault_code;
    st.protection_flags = in.protection_flags;
    st.limit_flags = in.limit_flags;
    st.device_protect_known = in.device_protect_known;
    st.device_protect_enabled = in.device_protect_enabled;
    st.device_direction_known = in.device_direction_known;
    st.device_direction = in.device_direction;
    st.online = in.online;
    st.lifecycle = in.lifecycle;
    st.bus_index = in.bus_index;
  }

  param_bridge_fill_system_state(&m);

  motor_rt_diagnostics_t diagnostics = {};
  motor_rt_copy_diagnostics(&diagnostics);
  auto &d = m.diagnostics;
  d.rx_frame_count = diagnostics.kernel.rx_frame_count;
  d.unknown_frame_count = diagnostics.kernel.unknown_frame_count;
  d.decode_error_count = diagnostics.kernel.decode_error_count;
  d.snapshot_accepted = diagnostics.kernel.snapshot_accepted;
  d.snapshot_rejected = diagnostics.kernel.snapshot_rejected;
  d.config_rejected = diagnostics.kernel.config_rejected;
  d.event_dropped = diagnostics.kernel.event_dropped;
  d.tx_dropped = diagnostics.kernel.tx_dropped;
  d.p0_retry = diagnostics.kernel.p0_retry;
  d.trajectory_accepted = 0U;
  d.trajectory_rejected = 0U;
  d.policy_accepted = diagnostics.kernel.policy_accepted;
  d.policy_rejected = diagnostics.kernel.policy_rejected;
  d.mailbox_rejected_command = diagnostics.mailbox_rejected_command;
  d.mailbox_rejected_trajectory = 0U;
  d.mailbox_rejected_policy = diagnostics.mailbox_rejected_policy;
  d.heap_free = diagnostics.heap_free;
  d.publish_fail = g_publish_fail + imu_bridge_publish_fail_count();
  d.ring_drop.size = d.rx_hw_lost.size = d.tx_frame_count.size = kDiagnosticBusCount;
  for (size_t i = 0U; i < kDiagnosticBusCount; ++i) {
    g_ring_drop[i] = diagnostics.ring_drop[i];
    g_rx_lost[i] = diagnostics.rx_hw_lost[i];
    g_tx_count[i] = diagnostics.tx_frame_count[i];
  }
}

static void fingerprint_add(uint32_t &hash, uint64_t value) {
  for (size_t byte = 0U; byte < sizeof(value); ++byte) {
    hash = (hash ^ (uint8_t)value) * 16777619U;
    value >>= 8U;
  }
}

static uint32_t status_fingerprint(void) {
  const auto &m = g_state_msg;
  uint32_t hash = 2166136261U;
  fingerprint_add(hash, m.info_revision);
  fingerprint_add(hash, m.stop_generation);
  fingerprint_add(hash, m.required_policy_generation);
  fingerprint_add(hash, m.lease_valid);
  fingerprint_add(hash, m.lease_active);
  fingerprint_add(hash, m.clock_valid);
  fingerprint_add(hash, m.system_status.last_error);
  fingerprint_add(hash, m.system_status.last_error_motor);
  fingerprint_add(hash, m.control_states.size);
  for (size_t i = 0U; i < m.control_states.size; ++i) {
    const auto &cs = m.control_states.data[i];
    fingerprint_add(hash, cs.motor_id);
    fingerprint_add(hash, cs.control_lifecycle);
    fingerprint_add(hash, cs.execution_state);
    fingerprint_add(hash, cs.online);
    fingerprint_add(hash, cs.policy_enabled);
    fingerprint_add(hash, cs.motion_enabled);
    fingerprint_add(hash, cs.policy_generation);
    fingerprint_add(hash, cs.unified_fault_code);
    fingerprint_add(hash, cs.vendor_fault_code);
  }
  return hash;
}

bool motor_bridge_create(rcl_node_t *node, rclc_executor_t *executor) {
  if (node == nullptr || executor == nullptr) return false;
  g_command_msg.header.frame_id.data = g_cmd_frame_id;
  g_command_msg.header.frame_id.size = 0U;
  g_command_msg.header.frame_id.capacity = sizeof(g_cmd_frame_id);
  bind_sequence(g_command_msg.targets, g_command_targets);
  bind_sequence(g_motion_msg.states, g_motion_states);
  bind_sequence(g_state_msg.control_states, g_control_states);
  bind_sequence(g_state_msg.statuses, g_statuses);
  bind_sequence(g_state_msg.diagnostics.ring_drop, g_ring_drop);
  bind_sequence(g_state_msg.diagnostics.rx_hw_lost, g_rx_lost);
  bind_sequence(g_state_msg.diagnostics.tx_frame_count, g_tx_count);

  rcl_subscription_options_t sub_opts = rcl_subscription_get_default_options();
  sub_opts.qos = qos_stream();
  if (rcl_subscription_init(&g_command_sub, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg, MotorCommandArray),
      "/motor/command", &sub_opts) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor); return false;
  }
  g_command_up = true;

  sub_opts.qos = qos_reliable(4);
  if (rcl_subscription_init(&g_stop_sub, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
      "/control/stop", &sub_opts) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor); return false;
  }
  g_stop_up = true;

  if (rcl_subscription_init(&g_client_sub, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "/client_count", &sub_opts) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor); return false;
  }
  g_client_up = true;

  rcl_publisher_options_t pub_opts = rcl_publisher_get_default_options();
  pub_opts.qos = qos_stream();
  if (rcl_publisher_init(&g_motion_pub, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg, MotorMotionStateArray),
      "/motor/motion", &pub_opts) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor); return false;
  }
  g_motion_up = true;

  pub_opts.qos = qos_reliable(1);
  if (rcl_publisher_init(&g_state_pub, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg, MotorSystemState),
      "/motor/state", &pub_opts) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor); return false;
  }
  g_state_up = true;
  (void)rmw_uros_set_publisher_session_timeout(rcl_publisher_get_rmw_handle(&g_state_pub), 5);

  pub_opts.qos = qos_reliable(8);
  if (rcl_publisher_init(&g_event_pub, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg, MotorEvent),
      "/motor/events", &pub_opts) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor); return false;
  }
  g_event_up = true;
  (void)rmw_uros_set_publisher_session_timeout(rcl_publisher_get_rmw_handle(&g_event_pub), 5);

  if (rclc_executor_add_subscription(executor, &g_command_sub, &g_command_msg,
      command_cb, ON_NEW_DATA) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor); return false;
  }
  g_command_added = true;
  if (rclc_executor_add_subscription(executor, &g_stop_sub, &g_stop_msg,
      stop_cb, ON_NEW_DATA) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor); return false;
  }
  g_stop_added = true;
  if (rclc_executor_add_subscription(executor, &g_client_sub, &g_client_msg,
      client_count_cb, ON_NEW_DATA) != RCL_RET_OK) {
    motor_bridge_destroy(node, executor); return false;
  }
  g_client_added = true;
  return true;
}

void motor_bridge_destroy(rcl_node_t *node, rclc_executor_t *executor) {
  g_session_id = 0U;
  g_lease_id = 0U;
  g_lease_epoch = 0U;
  g_lease_seq = 0U;
  if (executor != nullptr) {
    if (g_command_added) (void)rclc_executor_remove_subscription(executor, &g_command_sub);
    if (g_stop_added) (void)rclc_executor_remove_subscription(executor, &g_stop_sub);
    if (g_client_added) (void)rclc_executor_remove_subscription(executor, &g_client_sub);
  }
  g_command_added = g_stop_added = g_client_added = false;
  if (g_command_up) (void)rcl_subscription_fini(&g_command_sub, node);
  if (g_stop_up) (void)rcl_subscription_fini(&g_stop_sub, node);
  if (g_client_up) (void)rcl_subscription_fini(&g_client_sub, node);
  g_command_up = g_stop_up = g_client_up = false;
  if (g_motion_up) (void)rcl_publisher_fini(&g_motion_pub, node);
  if (g_state_up) (void)rcl_publisher_fini(&g_state_pub, node);
  if (g_event_up) (void)rcl_publisher_fini(&g_event_pub, node);
  g_motion_up = g_state_up = g_event_up = false;
}

void motor_bridge_poll(void) {
  if (g_session_id == 0U || !ros_bridge_is_active()) return;
  const uint32_t now = millis();
  publish_events();
  param_bridge_poll();
  publish_motion(now);
  fill_state();
  const uint32_t fingerprint = status_fingerprint();
  if (!g_have_sent_status || fingerprint != g_sent_status_fingerprint) g_status_dirty = true;
  const bool periodic_due = (uint32_t)(now - g_status_tick) >= FIRMWARE_STATUS_REPORT_PERIOD_MS;
  const bool change_due = g_status_dirty && (uint32_t)(now - g_status_tick) >= kStatusMinIntervalMs;
  if (!periodic_due && !change_due) return;
  g_status_tick = now;
  g_state_msg.sequence = ++g_status_sequence;
  serial_transport_send_begin();
  const rcl_ret_t rc = rcl_publish(&g_state_pub, &g_state_msg, nullptr);
  serial_transport_send_end();
  note_publish(rc);
  if (rc == RCL_RET_OK) {
    g_sent_status_fingerprint = fingerprint;
    g_have_sent_status = true;
    g_status_dirty = false;
  }
}
