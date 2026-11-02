// 配置回调只提交 mailbox；终态保留到显式 ACK，查询只读内存快照。
#include "param_bridge.h"
#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <can_bridge_msgs/msg/agent_config_request.h>
#include <can_bridge_msgs/msg/detail/agent_wire_contract__struct.h>
#include <rcl/error_handling.h>
#include "agent_protocol.h"
#include "config_transaction.hpp"
#include "info_page.hpp"
#include "motor_bridge.h"
#include "motor_rt_task.h"
#include "param_units.h"
#include "firmware_config.h"
#include "ros_qos.h"
#include "time_utils.h"

using Config = can_bridge_msgs__msg__AgentConfigRequest;
static constexpr uint16_t kMotorIdNone = UINT16_MAX;
static constexpr size_t kParamSetMaxTargets = 16;
static constexpr uint16_t kInvalid = 10, kSubmitFailed = 11, kUnknownMotor = 12;
static_assert(kParamSetMaxTargets <= CONTROL_MAILBOX_CONFIG_QUEUE_CAPACITY,
    "参数批次不能超过配置 FIFO");
static_assert(config_transaction::kCapacity == 8, "线上结果容量必须为8");

static rcl_subscription_t g_config_sub;
static Config g_config_msg;
static can_bridge_msgs__msg__MotorPolicyTarget g_policies[48];
static can_bridge_msgs__msg__ParamSetTarget g_parameters[16];
static uint32_t g_ack_ids[8];
static bool g_created, g_sub_up, g_sub_added;
// CPU 快照放 DTCM，避免扩展参数槽耗尽 D1；create 显式清零此 NOLOAD 段。
static motor_control_param_view_t g_views[MOTOR_CONTROL_MAX_MOTORS]
    __attribute__((section(".dtcm_noinit")));
static size_t g_view_count;
static config_transaction::Coordinator g_transactions;
static int g_active_slot = -1;
static uint64_t g_observed_session;
static uint32_t g_next_operation_id = 1, g_active_deadline;
static uint16_t g_last_error, g_last_error_motor = kMotorIdNone;
static info_page::Pager g_pager;
static uint32_t g_info_revision = 1, g_info_hash;
static can_bridge_msgs__msg__AgentInfo g_info;
static can_bridge_msgs__msg__AgentDeviceInfo g_info_devices[16];
static can_bridge_msgs__msg__AgentOperationDetail g_info_details[16];
static char g_model_names[16][32];

static const motor_control_param_view_t *find_view(uint16_t motor_id) {
  for (size_t i = 0; i < g_view_count; ++i)
    if (g_views[i].motor_id == motor_id) return &g_views[i];
  return nullptr;
}
static void report_error(uint16_t error, uint16_t motor_id) {
  g_last_error = error; g_last_error_motor = motor_id;
}
static uint32_t next_internal_id() {
  g_next_operation_id = (g_next_operation_id + 1U) & 0x3FFFFFFFU;
  if (!g_next_operation_id) g_next_operation_id = 1;
  return 0x40000000U | g_next_operation_id;
}
static void sync_config_session() {
  const uint64_t session = motor_bridge_session_id();
  if (session == g_observed_session) return;
  g_transactions.set_session(session);
  g_pager.set_source(session, g_info_revision, g_view_count);
  g_observed_session = session;
}
static bool submit_config(control_mailbox_config_request_t &request, uint8_t field) {
  if (!g_transactions.can_start(g_active_slot, millis())) return false;
  request.request_id = next_internal_id();
  request.received_tick_ms = millis();
  request.valid_until_tick_ms = g_active_deadline;
  if (!motor_rt_submit_config(&request)) {
    report_error(kSubmitFailed, request.motor_id); return false;
  }
  return g_transactions.track(g_active_slot, request.request_id, request.motor_id, field);
}
// 有限操作使用互斥载荷；未用组必须为空/默认，避免含混或拼接请求。
static bool groups_valid(const Config &m) {
  if (!agent_config_operation_supported(m.operation) || m.policies.size > 48 ||
      m.parameters.size || m.ack_request_ids.size > 8) return false;
  if (m.operation != can_bridge_msgs__msg__AgentConfigRequest__SET_POLICY &&
      (m.policies.size || m.policy_generation || m.allow_no_protect)) return false;
  if (m.operation != can_bridge_msgs__msg__AgentConfigRequest__ACK_RESULTS && m.ack_request_ids.size) return false;
  if (m.operation != can_bridge_msgs__msg__AgentConfigRequest__QUERY_INFO &&
      (m.query_id || m.query_kind || m.query_revision || m.cursor || m.detail_request_id)) return false;
  const bool single = m.operation >= can_bridge_msgs__msg__AgentConfigRequest__REFRESH_PARAMETERS && m.operation <= can_bridge_msgs__msg__AgentConfigRequest__CLEAR_FAULT;
  if (single ? (m.motor_id == 0 || m.motor_id > 64) : m.motor_id != kMotorIdNone) return false;
  if (m.operation == can_bridge_msgs__msg__AgentConfigRequest__SET_POLICY && !m.policies.size) return false;
  if (m.operation == can_bridge_msgs__msg__AgentConfigRequest__ACK_RESULTS && !m.ack_request_ids.size) return false;
  if (m.operation == can_bridge_msgs__msg__AgentConfigRequest__QUERY_INFO &&
      (!m.query_id || m.query_kind > 1 || (m.query_kind == 0 && m.detail_request_id))) return false;
  return true;
}
static bool policies_valid(const Config &m, uint64_t &disabled, bool &enables) {
  disabled = 0; enables = false;
  for (size_t i = 0; i < m.policies.size; ++i) {
    const auto &p = m.policies.data[i];
    if (!p.motor_id || p.motor_id > 64 || !isfinite(p.kp) || !isfinite(p.kd) || p.kp < 0 || p.kd < 0)
      return false;
    const uint64_t bit = UINT64_C(1) << (p.motor_id - 1);
    if (disabled & bit) return false;
    disabled |= bit;
    enables |= p.enabled;
  }
  if (!enables) {
    if (m.allow_no_protect) return false;
    for (size_t i = 0; i < m.policies.size; ++i)
      if (m.policies.data[i].kp != 0 || m.policies.data[i].kd != 0 ||
          m.policies.data[i].confirm_unprotected) return false;
  }
  return true;
}
static bool trajectory_busy(const Config &m) {
  return (m.operation == can_bridge_msgs__msg__AgentConfigRequest__REFRESH_PARAMETERS ||
      m.operation == can_bridge_msgs__msg__AgentConfigRequest__SET_ZERO ||
      m.operation == can_bridge_msgs__msg__AgentConfigRequest__CLEAR_FAULT) &&
      motor_rt_agent_trajectory_owns(m.motor_id);
}

static void config_cb(const void *input) {
  const auto *msg = static_cast<const Config *>(input);
  if (!msg) return;
  sync_config_session();
  config_transaction::Request request{};
  request.session_id = msg->session_id; request.request_id = msg->request_id;
  request.command = msg->operation; request.motor_id = msg->motor_id;
  request.policy_generation = msg->policy_generation;
  request.sync_time_ns = msg->issued_stamp;
  if (msg->schema_version != can_bridge_msgs__msg__AgentWireContract__SCHEMA_VERSION || !groups_valid(*msg)) {
    g_transactions.reject(request, config_transaction::kFailed, kInvalid); return;
  }
  if (msg->operation == can_bridge_msgs__msg__AgentConfigRequest__STOP_ALL) {
    motor_rt_request_stop_all(); return;
  }
  if (!g_observed_session || msg->session_id != g_observed_session) {
    g_transactions.reject(request, config_transaction::kRetired, kInvalid); return;
  }
  if (msg->operation == can_bridge_msgs__msg__AgentConfigRequest__ACK_RESULTS) {
    for (size_t i = 0; i < msg->ack_request_ids.size; ++i)
      (void)g_transactions.ack_result(msg->session_id, msg->ack_request_ids.data[i]);
    return;
  }
  if (msg->operation == can_bridge_msgs__msg__AgentConfigRequest__QUERY_INFO) {
    g_pager.set_source(g_observed_session, g_info_revision, g_view_count);
    (void)g_pager.request(msg->session_id, msg->query_id, msg->query_revision,
        msg->cursor, msg->query_kind, msg->detail_request_id);
    return;
  }
  uint64_t disabled = 0;
  bool enables = false;
  if (msg->operation == can_bridge_msgs__msg__AgentConfigRequest__SET_POLICY) {
    if (!policies_valid(*msg, disabled, enables)) {
      g_transactions.reject(request, config_transaction::kFailed, kInvalid); return;
    }
    if (!enables) { motor_rt_request_disable(disabled); return; }
  }
  request.size = msg->parameters.size; request.count = request.size;
  for (size_t i = 0; i < request.size; ++i)
    request.targets[i] = {msg->parameters.data[i].motor_id,
        msg->parameters.data[i].field, msg->parameters.data[i].value};
  request.policy_count = msg->policies.size;
  for (size_t i = 0; i < request.policy_count; ++i) {
    const auto &p = msg->policies.data[i];
    request.policies[i] = {p.motor_id, p.enabled,
        p.confirm_unprotected || msg->allow_no_protect, p.kp, p.kd};
  }
  const uint32_t now = millis();
  const uint16_t ttl_limit = msg->operation == can_bridge_msgs__msg__AgentConfigRequest__SET_POLICY ? 100 : 1000;
  // begin 先检查相同 ID 的缓存，重试更改时间或 TTL 也绝不会续命/重执行。
  if (!agent_request_deadline(msg->issued_stamp, msg->ttl_ms, ttl_limit,
      time_epoch_now(), now, &request.valid_until_tick_ms)) request.valid_until_tick_ms = now;
  g_active_slot = g_transactions.begin(request, g_observed_session, now);
  if (g_active_slot < 0) return;
  g_active_deadline = request.valid_until_tick_ms;
  if (!time_clock_valid() || !g_transactions.can_start(g_active_slot, millis())) {
    g_transactions.complete(g_active_slot, 0, 0, config_transaction::kExpired); return;
  }
  if (trajectory_busy(*msg)) {
    g_transactions.busy(g_active_slot); return;
  }
  if (msg->operation == can_bridge_msgs__msg__AgentConfigRequest__SET_POLICY) {
    for (size_t i = 0; i < msg->policies.size; ++i) {
      const auto &target = msg->policies.data[i];
      if (target.enabled && g_transactions.pending_device_command(
          can_bridge_msgs__msg__AgentConfigRequest__SET_ZERO, target.motor_id)) {
        g_transactions.busy(g_active_slot); return;
      }
    }
    control_policy_snapshot_t policy{};
    policy.request_id = next_internal_id(); policy.received_tick_ms = now;
    policy.valid_until_tick_ms = g_active_deadline;
    policy.policy_generation = msg->policy_generation;
    policy.lease_id = msg->lease_id;
    policy.target_count = msg->policies.size;
    for (size_t i = 0; i < policy.target_count; ++i) {
      const auto &p = msg->policies.data[i];
      policy.targets[i] = {p.motor_id, p.enabled, p.kp, p.kd,
          p.confirm_unprotected || msg->allow_no_protect};
    }
    if (!motor_rt_submit_policy(&policy)) { g_transactions.busy(g_active_slot); return; }
    for (size_t i = 0; i < policy.target_count; ++i)
      (void)g_transactions.track(g_active_slot, policy.request_id, policy.targets[i].motor_id);
    g_transactions.submitted(g_active_slot, policy.target_count, 0);
  } else {
    const auto *v = find_view(msg->motor_id);
    if (!v || !v->bound) { g_transactions.complete(g_active_slot, 0, 1, kUnknownMotor); return; }
    // 首版保守地拒绝仍武装的 CLEAR_FAULT，不把故障清除当作运动命令。
    if ((msg->operation == can_bridge_msgs__msg__AgentConfigRequest__CLEAR_FAULT ||
        msg->operation == can_bridge_msgs__msg__AgentConfigRequest__SET_ZERO) && v->enable) {
      g_transactions.busy(g_active_slot); return;
    }
    control_mailbox_config_request_t item{};
    item.motor_id = msg->motor_id;
    item.kind = msg->operation == can_bridge_msgs__msg__AgentConfigRequest__REFRESH_PARAMETERS ? CONTROL_MAILBOX_CONFIG_REFRESH_PARAMS
        : msg->operation == can_bridge_msgs__msg__AgentConfigRequest__SET_ZERO ? CONTROL_MAILBOX_CONFIG_SET_ZERO
        : CONTROL_MAILBOX_CONFIG_CLEAR_FAULT;
    const bool ok = submit_config(item, UINT8_MAX);
    g_transactions.submitted(g_active_slot, 1, ok ? 0 : kSubmitFailed);
  }
}

void param_bridge_on_config_result(const motor_control_event_t *event) {
  if (!g_created || !event || event->type != MOTOR_CONTROL_EVENT_CONFIG_RESULT) return;
  const bool ok = event->reason == MOTOR_PROTOCOL_CONFIG_OK;
  (void)g_transactions.on_result(event->request_id, ok, event->reason, event->motor_id);
  if (!ok) report_error(event->reason, event->motor_id);
}
static uint32_t hash_bytes(uint32_t hash, const void *data, size_t size) {
  const auto *p = static_cast<const uint8_t *>(data);
  while (size--) hash = (hash ^ *p++) * 16777619U;
  return hash;
}
static void fill_info_cache() {
  if (!g_pager.needs_fill()) return;
  const auto &page = g_pager.page();
  const uint32_t tick = millis();
  const uint64_t epoch = time_epoch_now();
  if (page.kind == 1) {
    static config_transaction::Detail details[config_transaction::kMaxOperations];
    const size_t total = g_transactions.copy_details(g_observed_session,
        page.detail_request_id, details, config_transaction::kMaxOperations);
    const size_t begin = page.cursor < total ? page.cursor : total;
    const size_t count = total - begin < 16 ? total - begin : 16;
    g_pager.fill_details(details + begin, count, total, tick, epoch);
    return;
  }
  // 首版复用有界临时转换区；缓存仍只有16台，不触发设备回读。
  static info_page::Device devices[MOTOR_CONTROL_MAX_MOTORS];
  static motor_rt_topology_entry_t topology[MOTOR_CONTROL_MAX_MOTORS];
  const size_t count = motor_rt_copy_topology(topology, MOTOR_CONTROL_MAX_MOTORS);
  for (size_t i = 0; i < g_view_count; ++i) {
    auto &d = devices[i]; d = info_page::Device{};
    d.motor_id = g_views[i].motor_id; d.params = g_views[i]; d.online = g_views[i].online;
    for (size_t j = 0; j < count; ++j) if (topology[j].motor_id == d.motor_id) {
      d.bus = topology[j].bus_index; d.protocol = topology[j].protocol;
      memcpy(d.model, topology[j].model_name, sizeof(d.model)); d.model[31] = '\0';
      d.capabilities = topology[j].capability_flags; d.conflict = topology[j].conflict;
      d.lifecycle = topology[j].lifecycle;
    }
  }
  g_pager.fill(devices, g_view_count, tick, epoch);
}
void param_bridge_poll(void) {
  if (!g_created) return;
  sync_config_session();
  const uint32_t now = millis();
  g_view_count = motor_rt_copy_param_views(g_views, MOTOR_CONTROL_MAX_MOTORS);
  const uint32_t hash = hash_bytes(2166136261U, g_views, g_view_count * sizeof(g_views[0]));
  if (hash != g_info_hash) {
    g_info_hash = hash;
    if (++g_info_revision == 0) ++g_info_revision;
  }
  g_pager.set_source(g_observed_session, g_info_revision, g_view_count);
  g_transactions.poll(now);
  fill_info_cache();
}
static void encode_result(can_bridge_msgs__msg__AgentConfigResult &out,
    const config_transaction::Result &in) {
  out.request_id = in.request_id; out.operation = in.command;
  out.state = in.status; out.error = in.error_code;
  out.completed_motor_mask = in.completed_motor_mask; out.failed_motor_mask = in.failed_motor_mask;
}
void param_bridge_fill_status(can_bridge_msgs__msg__AgentStatus *out) {
  if (!out) return;
  out->info_revision = g_info_revision;
  out->config_request_high_water = g_transactions.request_high_water();
  out->save_in_progress = false;
  out->last_error = g_last_error; out->last_error_motor = g_last_error_motor;
  const uint16_t frequencies[] = {FIRMWARE_AGENT_PING_HZ, FIRMWARE_MOTOR_REPORT_HZ,
      FIRMWARE_MOTOR_REPORT_HZ, FIRMWARE_IMU_REPORT_HZ, FIRMWARE_TEMPERATURE_REPORT_HZ,
      FIRMWARE_MOTOR_COMMAND_HZ};
  out->firmware_values_valid = true;
  memcpy(out->firmware_values, frequencies, sizeof(frequencies));
  out->firmware_unsaved = false;
  config_transaction::Result results[8]{};
  const size_t count = g_transactions.copy_results(results, 8);
  out->results.size = count <= out->results.capacity ? count : out->results.capacity;
  for (size_t i = 0; i < out->results.size; ++i) encode_result(out->results.data[i], results[i]);
  config_transaction::Result rejection{};
  if (g_transactions.copy_rejection(rejection)) encode_result(out->rejection_receipt, rejection);
  else out->rejection_receipt = can_bridge_msgs__msg__AgentConfigResult{};
}
static void encode_parameters(can_bridge_msgs__msg__DeviceParamEntry &out,
    const motor_control_param_view_t &v) {
  out.motor_id = v.motor_id; out.dev_init_state = v.dev_init;
  out.record_dirty = v.record_dirty_slots != 0; out.mismatch_mask = v.dev_mismatch_mask;
  float rec[8], dev[8];
  for (size_t i = 0; i < 8; ++i) {
    rec[i] = (v.record_raw_present & (1U << i)) ? v.record_si[i] : NAN;
    dev[i] = (v.dev_raw_present & (1U << i)) ? v.dev_si[i] : NAN;
  }
  rec[3] = param_units_nm_to_amp(rec[3], v.torque_per_amp);
  dev[3] = param_units_nm_to_amp(dev[3], v.torque_per_amp);
  out.rec_pos_min = rec[0]; out.rec_pos_max = rec[1]; out.rec_vel_limit = rec[2];
  out.rec_torque_limit = rec[3]; out.rec_safety = rec[4]; out.rec_direction = rec[5];
  out.dev_pos_min = dev[0]; out.dev_pos_max = dev[1]; out.dev_vel_limit = dev[2];
  out.dev_torque_limit = dev[3]; out.dev_safety = dev[4]; out.dev_direction = dev[5];
  out.rec_t_max = rec[6]; out.dev_t_max = dev[6];
  out.rec_node_id = rec[7]; out.dev_node_id = dev[7];
}
const can_bridge_msgs__msg__AgentInfo *param_bridge_info(void) {
  if (!g_pager.pending() || g_pager.needs_fill()) return nullptr;
  const auto &page = g_pager.page();
  g_info.session_id = page.session; g_info.snapshot_stamp = page.epoch_ns;
  g_info.snapshot_tick_ms = page.tick_ms; g_info.clock_valid = page.epoch_ns != 0;
  g_info.schema_version = can_bridge_msgs__msg__AgentWireContract__SCHEMA_VERSION;
  g_info.query_id = page.query_id; g_info.kind = page.kind;
  g_info.result = page.result; g_info.revision = page.revision;
  g_info.cursor = page.cursor; g_info.next_cursor = page.next_cursor; g_info.complete = page.complete;
  g_info.motor_capacity = 48; g_info.trajectory_point_capacity = 256;
  g_info.detail_request_id = page.detail_request_id;
  g_info.devices.size = page.kind == 0 ? page.count : 0;
  g_info.details.size = page.kind == 1 ? page.count : 0;
  for (size_t i = 0; i < g_info.devices.size; ++i) {
    auto &out = g_info_devices[i]; const auto &in = page.devices[i];
    out.motor_id = in.motor_id; out.bus_index = in.bus; out.node_id = in.motor_id;
    out.protocol = in.protocol; out.capability_flags = in.capabilities;
    memcpy(g_model_names[i], in.model, 32); g_model_names[i][31] = '\0';
    out.model_name.size = strlen(g_model_names[i]);
    out.online = in.online; out.conflict = in.conflict; out.lifecycle = in.lifecycle;
    out.record_valid = in.params.record_raw_present != 0;
    out.device_valid = in.params.dev_raw_present == MOTOR_CONTROL_DEV_RAW_ALL;
    // 内核尚无参数采集 tick；显式未知，绝不把当前查询时间冒充回读时间。
    out.device_read_age_ms = UINT32_MAX;
    encode_parameters(out.parameters, in.params);
  }
  for (size_t i = 0; i < g_info.details.size; ++i) {
    const auto &in = page.details[i]; auto &out = g_info_details[i];
    out.motor_id = in.motor_id; out.field = in.field; out.error = in.error;
    out.completed = in.complete;
  }
  return &g_info;
}
void param_bridge_info_sent(bool success) { g_pager.sent(success); }
uint32_t param_bridge_request_high_water(void) { return g_transactions.request_high_water(); }
const motor_control_param_view_t *param_bridge_views(size_t *count) {
  if (count) *count = g_view_count;
  return g_views;
}
bool param_bridge_create(rcl_node_t *node, rclc_executor_t *executor) {
  if (g_created) return true;
  if (!node || !executor) return false;
  memset(g_views, 0, sizeof(g_views));
  g_config_msg.motor_id = kMotorIdNone;
  g_config_msg.policies.data = g_policies; g_config_msg.policies.capacity = 48;
  g_config_msg.parameters.data = g_parameters; g_config_msg.parameters.capacity = 16;
  g_config_msg.ack_request_ids.data = g_ack_ids; g_config_msg.ack_request_ids.capacity = 8;
  g_info.devices.data = g_info_devices; g_info.devices.capacity = 16;
  g_info.details.data = g_info_details; g_info.details.capacity = 16;
  for (size_t i = 0; i < 16; ++i) {
    g_info_devices[i].model_name.data = g_model_names[i];
    g_info_devices[i].model_name.capacity = 32;
  }
  rcl_subscription_options_t options = rcl_subscription_get_default_options();
  options.qos = qos_reliable(4);
  if (rcl_subscription_init(&g_config_sub, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg, AgentConfigRequest),
      can_bridge_msgs__msg__AgentWireContract__TOPIC_CONFIG, &options) != RCL_RET_OK) return false;
  g_sub_up = true;
  if (rclc_executor_add_subscription(executor, &g_config_sub, &g_config_msg,
      config_cb, ON_NEW_DATA) != RCL_RET_OK) {
    param_bridge_destroy(node, executor); return false;
  }
  g_sub_added = true; g_created = true;
  return true;
}
void param_bridge_destroy(rcl_node_t *node, rclc_executor_t *executor) {
  g_created = false;
  g_transactions.disconnect(); g_view_count = 0;
  if (g_sub_added && executor) (void)rclc_executor_remove_subscription(executor, &g_config_sub);
  g_sub_added = false;
  if (g_sub_up) { const rcl_ret_t result = rcl_subscription_fini(&g_config_sub, node); (void)result; }
  g_sub_up = false;
}
