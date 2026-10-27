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
#include "param_field.h"
#include "param_units.h"
#include "ros_qos.h"
#include "save_transaction.h"
#include "storage_task.h"
#include "time_utils.h"

using Config = can_bridge_msgs__msg__AgentConfigRequest;
static constexpr uint16_t kMotorIdNone = UINT16_MAX;
static constexpr size_t kParamSetMaxTargets = 16;
static constexpr uint16_t kInvalid = 10, kSubmitFailed = 11, kUnknownMotor = 12;
static constexpr uint16_t kSaveFailed = 20;
static_assert(kParamSetMaxTargets <= CONTROL_MAILBOX_CONFIG_QUEUE_CAPACITY,
    "参数批次不能超过配置 FIFO");
static_assert(config_transaction::kCapacity == 8, "线上结果容量必须为8");

static rcl_subscription_t g_config_sub;
static Config g_config_msg;
static can_bridge_msgs__msg__MotorPolicyTarget g_policies[48];
static can_bridge_msgs__msg__ParamSetTarget g_parameters[16];
static uint32_t g_ack_ids[8];
static bool g_created, g_sub_up, g_sub_added;
static motor_control_param_view_t g_views[MOTOR_CONTROL_MAX_MOTORS];
static size_t g_view_count;
static config_transaction::Coordinator g_transactions;
static int g_active_slot = -1, g_save_slot = -1;
static uint64_t g_observed_session;
static uint32_t g_next_operation_id = 1, g_active_deadline;
static uint16_t g_last_error, g_last_error_motor = kMotorIdNone;
static save_transaction_t g_save_transaction;
static uint32_t g_save_deadline;
static bool g_save_started;
static uint16_t g_save_total, g_save_completed, g_save_error;
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
  save_transaction_disconnect(&g_save_transaction);
  g_save_slot = -1;
  g_transactions.set_session(session);
  g_pager.set_source(session, g_info_revision, g_view_count);
  g_observed_session = session;
}
static bool deadline_valid(uint32_t deadline, uint32_t now) {
  const uint32_t remaining = deadline - now;
  return remaining > 0 && remaining <= 1000;
}

static uint32_t save_next_request_id(void *) { return next_internal_id(); }
static bool save_submit_device(void *, uint32_t id, uint16_t motor_id, uint8_t slots) {
  const uint32_t now = millis();
  if (!g_save_started && !deadline_valid(g_save_deadline, now)) return false;
  control_mailbox_config_request_t request{};
  request.request_id = id; request.received_tick_ms = now;
  // 后续设备属于已启动的 SAVE 事务；只约束它们在 FIFO 中的停留，不重启源 TTL。
  request.valid_until_tick_ms = g_save_started ? now + 1000U : g_save_deadline;
  request.motor_id = motor_id;
  request.kind = CONTROL_MAILBOX_CONFIG_SAVE_DEVICE_FLASH;
  request.value.save_slots = slots;
  if (!motor_rt_submit_config(&request)) return false;
  g_save_started = true;
  return g_transactions.track(g_save_slot, id, motor_id);
}
static bool save_request_storage(void *, uint32_t id) {
  if (!g_save_started && !deadline_valid(g_save_deadline, millis())) return false;
  g_save_started = true;
  storage_task_request_save(id);
  return true;
}
static save_transaction_storage_result_t save_take_storage(void *, uint32_t *id) {
  switch (storage_take_result(id)) {
  case STORAGE_SAVE_OK: return SAVE_TRANSACTION_STORAGE_OK;
  case STORAGE_SAVE_FAIL: return SAVE_TRANSACTION_STORAGE_FAIL;
  default: return SAVE_TRANSACTION_STORAGE_IDLE;
  }
}
static void save_report_error(void *, save_transaction_error_t,
    uint16_t motor_id, uint16_t code) {
  g_save_error = code ? code : kSaveFailed;
  report_error(g_save_error, motor_id);
}
static const save_transaction_ports_t kSavePorts = {
  .context = nullptr, .next_request_id = save_next_request_id,
  .submit_device_save = save_submit_device, .request_storage_save = save_request_storage,
  .take_storage_result = save_take_storage, .report_error = save_report_error,
};

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
static bool validate_parameters(const can_bridge_msgs__msg__ParamSetTarget *targets, size_t count) {
  static const param_field_t fields[] = {
    PARAM_FIELD_POS_MIN, PARAM_FIELD_POS_MAX, PARAM_FIELD_VEL_LIMIT,
    PARAM_FIELD_TORQUE_LIMIT, PARAM_FIELD_LIMIT_PROTECT, PARAM_FIELD_DIRECTION};
  if (!count || count > 16) return false;
  for (size_t i = 0; i < count; ++i) {
    const auto &t = targets[i];
    const auto *v = find_view(t.motor_id);
    if (t.field >= PARAM_DEVICE_SLOT_COUNT || !v || !v->bound ||
        !param_value_ok(fields[t.field], t.value, 0.0f, nullptr)) return false;
    if (t.field == 3 && !isfinite(param_units_amp_to_nm(t.value, v->torque_per_amp))) return false;
    for (size_t j = 0; j < i; ++j)
      if (targets[j].motor_id == t.motor_id && targets[j].field == t.field) return false;
    if (t.field <= 1) {
      bool found = false;
      for (size_t j = 0; j < count; ++j) {
        if (targets[j].motor_id != t.motor_id || targets[j].field != (t.field ^ 1)) continue;
        found = t.field == 0 ? param_position_limits_ok(t.value, targets[j].value)
                             : param_position_limits_ok(targets[j].value, t.value);
      }
      if (!found) return false;
    }
  }
  return true;
}
static void apply_parameters(const Config &msg) {
  uint16_t total = 0;
  for (size_t i = 0; i < msg.parameters.size; ++i) if (msg.parameters.data[i].field != 1) ++total;
  for (size_t i = 0; i < msg.parameters.size; ++i) {
    const auto &t = msg.parameters.data[i];
    if (t.field == 1) continue;
    const auto *v = find_view(t.motor_id);
    control_mailbox_config_request_t request{};
    request.motor_id = t.motor_id;
    switch (t.field) {
    case 0:
      request.kind = CONTROL_MAILBOX_CONFIG_SET_POSITION_LIMITS;
      request.value.position_limits.min_position_rad = t.value;
      for (size_t j = 0; j < msg.parameters.size; ++j)
        if (msg.parameters.data[j].motor_id == t.motor_id && msg.parameters.data[j].field == 1)
          request.value.position_limits.max_position_rad = msg.parameters.data[j].value;
      break;
    case 2:
      request.kind = CONTROL_MAILBOX_CONFIG_SET_VELOCITY_LIMIT;
      request.value.max_velocity_rad_s = t.value; break;
    case 3:
      request.kind = CONTROL_MAILBOX_CONFIG_SET_TORQUE_LIMIT;
      request.value.max_torque_nm = param_units_amp_to_nm(t.value, v->torque_per_amp); break;
    case 4:
      request.kind = CONTROL_MAILBOX_CONFIG_SET_SAFETY_PROTECT;
      request.value.safety_protect_raw = t.value == 1.0f ? 0U : 1U; break;
    case 5:
      request.kind = CONTROL_MAILBOX_CONFIG_SET_DIRECTION;
      request.value.direction_raw = t.value == 1.0f ? 1U : 0U; break;
    default: continue;
    }
    if (!submit_config(request, t.field)) {
      g_transactions.submitted(g_active_slot, total, kSubmitFailed); return;
    }
  }
  g_transactions.submitted(g_active_slot, total, 0);
}

// 有限操作使用互斥载荷；未用组必须为空/默认，避免含混或拼接请求。
static bool groups_valid(const Config &m) {
  if (m.operation > can_bridge_msgs__msg__AgentConfigRequest__ACK_RESULTS || m.policies.size > 48 ||
      m.parameters.size > 16 || m.ack_request_ids.size > 8) return false;
  if (m.operation != can_bridge_msgs__msg__AgentConfigRequest__SET_POLICY &&
      (m.policies.size || m.policy_generation || m.allow_no_protect)) return false;
  if (m.operation != can_bridge_msgs__msg__AgentConfigRequest__SET_PARAMETERS && m.parameters.size) return false;
  if (m.operation != can_bridge_msgs__msg__AgentConfigRequest__ACK_RESULTS && m.ack_request_ids.size) return false;
  if (m.operation != can_bridge_msgs__msg__AgentConfigRequest__QUERY_INFO &&
      (m.query_id || m.query_kind || m.query_revision || m.cursor || m.detail_request_id)) return false;
  const bool single = m.operation >= can_bridge_msgs__msg__AgentConfigRequest__REFRESH_PARAMETERS && m.operation <= can_bridge_msgs__msg__AgentConfigRequest__CLEAR_FAULT;
  if (single ? (m.motor_id == 0 || m.motor_id > 64) : m.motor_id != kMotorIdNone) return false;
  if (m.operation == can_bridge_msgs__msg__AgentConfigRequest__SET_POLICY && !m.policies.size) return false;
  if (m.operation == can_bridge_msgs__msg__AgentConfigRequest__SET_PARAMETERS && !m.parameters.size) return false;
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
  if (m.operation == can_bridge_msgs__msg__AgentConfigRequest__SAVE) {
    agent_trajectory_status_t status{};
    motor_rt_copy_agent_trajectory(&status);
    return status.state >= AGENT_TRAJECTORY_LOADING && status.state <= AGENT_TRAJECTORY_RUNNING;
  }
  if (m.operation == can_bridge_msgs__msg__AgentConfigRequest__SET_PARAMETERS) {
    for (size_t i = 0; i < m.parameters.size; ++i)
      if (motor_rt_agent_trajectory_owns(m.parameters.data[i].motor_id)) return true;
  }
  return m.operation >= can_bridge_msgs__msg__AgentConfigRequest__REFRESH_PARAMETERS && m.operation <= can_bridge_msgs__msg__AgentConfigRequest__CLEAR_FAULT &&
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
  if (msg->schema_version != 3 || !groups_valid(*msg)) {
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
  if (save_transaction_in_progress(&g_save_transaction) || trajectory_busy(*msg)) {
    g_transactions.busy(g_active_slot); return;
  }
  if (msg->operation == can_bridge_msgs__msg__AgentConfigRequest__SET_POLICY) {
    control_policy_snapshot_t policy{};
    policy.request_id = next_internal_id(); policy.received_tick_ms = now;
    policy.valid_until_tick_ms = g_active_deadline;
    policy.policy_generation = msg->policy_generation;
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
  } else if (msg->operation == can_bridge_msgs__msg__AgentConfigRequest__SET_PARAMETERS) {
    if (!validate_parameters(msg->parameters.data, msg->parameters.size)) {
      g_transactions.complete(g_active_slot, 0, msg->parameters.size, kInvalid); return;
    }
    apply_parameters(*msg);
  } else if (msg->operation == can_bridge_msgs__msg__AgentConfigRequest__SAVE) {
    if (g_transactions.pending_except(g_active_slot)) { g_transactions.busy(g_active_slot); return; }
    g_save_slot = g_active_slot; g_save_deadline = g_active_deadline;
    g_save_started = false; g_save_total = g_save_completed = g_save_error = 0;
    g_transactions.external(g_save_slot);
    save_transaction_request(&g_save_transaction);
  } else {
    const auto *v = find_view(msg->motor_id);
    if (!v || !v->bound) { g_transactions.complete(g_active_slot, 0, 1, kUnknownMotor); return; }
    // 首版保守地拒绝仍武装的 CLEAR_FAULT，不把故障清除当作运动命令。
    if (msg->operation == can_bridge_msgs__msg__AgentConfigRequest__CLEAR_FAULT && v->enable) {
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
  if (save_transaction_on_device_result(&g_save_transaction, event->request_id, ok,
      event->motor_id, event->reason, &kSavePorts) && ok) ++g_save_completed;
  if (!ok) report_error(event->reason, event->motor_id);
}
static void poll_save(uint32_t now) {
  if (g_save_slot < 0) return;
  const auto *result = g_transactions.result(g_save_slot);
  if (!result || result->status != config_transaction::kPending) {
    save_transaction_disconnect(&g_save_transaction); g_save_slot = -1; return;
  }
  if (!g_save_started && !g_transactions.can_start(g_save_slot, now)) {
    save_transaction_disconnect(&g_save_transaction); g_save_slot = -1; return;
  }
  static save_transaction_target_t targets[SAVE_TRANSACTION_MAX_TARGETS];
  g_save_total = 0;
  for (size_t i = 0; i < g_view_count; ++i) {
    targets[i] = {g_views[i].motor_id, g_views[i].record_dirty_slots};
    if (targets[i].dirty_slots) ++g_save_total;
  }
  const auto effects = save_transaction_poll(&g_save_transaction, now, targets,
      g_view_count, &kSavePorts);
  if (effects.completed) {
    g_transactions.complete(g_save_slot, g_save_completed,
        g_save_total > g_save_completed ? g_save_total - g_save_completed : 0, g_save_error);
    g_save_slot = -1;
  }
}
bool param_bridge_save_in_progress(void) {
  return save_transaction_in_progress(&g_save_transaction);
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
  poll_save(now);
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
  out->save_in_progress = save_transaction_in_progress(&g_save_transaction);
  out->last_error = g_last_error; out->last_error_motor = g_last_error_motor;
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
  float rec[6], dev[6];
  for (size_t i = 0; i < 6; ++i) {
    rec[i] = (v.record_raw_present & (1U << i)) ? v.record_si[i] : NAN;
    dev[i] = (v.dev_raw_present & (1U << i)) ? v.dev_si[i] : NAN;
  }
  rec[3] = param_units_nm_to_amp(rec[3], v.torque_per_amp);
  dev[3] = param_units_nm_to_amp(dev[3], v.torque_per_amp);
  out.rec_pos_min = rec[0]; out.rec_pos_max = rec[1]; out.rec_vel_limit = rec[2];
  out.rec_torque_limit = rec[3]; out.rec_safety = rec[4]; out.rec_direction = rec[5];
  out.dev_pos_min = dev[0]; out.dev_pos_max = dev[1]; out.dev_vel_limit = dev[2];
  out.dev_torque_limit = dev[3]; out.dev_safety = dev[4]; out.dev_direction = dev[5];
}
const can_bridge_msgs__msg__AgentInfo *param_bridge_info(void) {
  if (!g_pager.pending() || g_pager.needs_fill()) return nullptr;
  const auto &page = g_pager.page();
  g_info.session_id = page.session; g_info.snapshot_stamp = page.epoch_ns;
  g_info.snapshot_tick_ms = page.tick_ms; g_info.clock_valid = page.epoch_ns != 0;
  g_info.schema_version = 3; g_info.query_id = page.query_id; g_info.kind = page.kind;
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
const motor_control_stored_params_t *param_bridge_stored_records(size_t *count) {
  return storage_boot_params(count);
}

bool param_bridge_create(rcl_node_t *node, rclc_executor_t *executor) {
  if (g_created) return true;
  if (!node || !executor) return false;
  save_transaction_init(&g_save_transaction);
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
  save_transaction_disconnect(&g_save_transaction);
  g_save_slot = -1; g_transactions.disconnect(); g_view_count = 0;
  if (g_sub_added && executor) (void)rclc_executor_remove_subscription(executor, &g_config_sub);
  g_sub_added = false;
  if (g_sub_up) { const rcl_ret_t result = rcl_subscription_fini(&g_config_sub, node); (void)result; }
  g_sub_up = false;
}
