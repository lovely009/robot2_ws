// 原生 ROS 2 服务桥：/motor/submit_policy、/motor/execute_config、/motor/get_device_info。
#include "param_bridge.h"

#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <can_bridge_msgs/msg/system_command.h>
#include <can_bridge_msgs/srv/execute_config.h>
#include <can_bridge_msgs/srv/get_device_info.h>
#include <can_bridge_msgs/srv/submit_policy.h>
#include <rcl/error_handling.h>

#include "firmware_config.h"
#include "motor_bridge.h"
#include "motor_rt_task.h"
#include "param_units.h"
#include "ros_bridge.h"
#include "ros_qos.h"
#include "storage_boot.h"
#include "time_utils.h"

static constexpr size_t kMotorCapacity = FIRMWARE_MAX_MOTORS;
static constexpr uint16_t kMotorIdNone = UINT16_MAX;
static constexpr uint16_t kInvalid = 10U, kSubmitFailed = 11U, kUnknownMotor = 12U, kTimeout = 21U;
static constexpr uint8_t kDeviceSaveMask = MOTOR_PROTOCOL_PARAM_SAVE_SLOTS_MASK;

static rcl_service_t g_policy_srv, g_config_srv, g_info_srv;
static bool g_policy_up = false, g_config_up = false, g_info_up = false;
static bool g_policy_added = false, g_config_added = false, g_info_added = false;
static bool g_created = false;

static can_bridge_msgs__srv__SubmitPolicy_Request g_policy_req;
static can_bridge_msgs__srv__SubmitPolicy_Response g_policy_res;
static can_bridge_msgs__msg__MotorPolicyTarget g_policy_targets[kMotorCapacity];
static char g_policy_req_frame[32], g_policy_intent_id[32], g_policy_reason[64];

static can_bridge_msgs__srv__ExecuteConfig_Request g_config_req;
static can_bridge_msgs__srv__ExecuteConfig_Response g_config_res;
static can_bridge_msgs__msg__ParamSetTarget g_config_targets[16];
static char g_config_req_frame[32], g_config_res_intent[32];

static can_bridge_msgs__srv__GetDeviceInfo_Request g_info_req;
static can_bridge_msgs__srv__GetDeviceInfo_Response g_info_res;
static can_bridge_msgs__msg__MotorTopologyEntry g_info_topo[kMotorCapacity];
static can_bridge_msgs__msg__DeviceParamEntry g_info_params[kMotorCapacity];
static char g_info_frame[32], g_info_models[kMotorCapacity][32];
static char kNodeFrame[] = "micro_ros_node";

static motor_control_param_view_t g_views[kMotorCapacity]
    __attribute__((section(".dtcm_noinit")));
static size_t g_view_count = 0U;
static motor_rt_topology_entry_t g_topology[kMotorCapacity]
    __attribute__((section(".dtcm_noinit")));

static uint32_t g_info_revision = 1U, g_info_hash = 0U;
static uint32_t g_next_policy_gen = 1U, g_next_req_id = 1U;
static uint16_t g_last_error = 0U, g_last_error_motor = kMotorIdNone;

static volatile uint32_t g_pending_tx_id = 0U;
static volatile bool g_pending_tx_done = false;
static volatile uint16_t g_pending_tx_reason = 0U;

static uint32_t hash_bytes(uint32_t hash, const void *data, size_t size) {
  const auto *p = static_cast<const uint8_t *>(data);
  while (size--) hash = (hash ^ *p++) * 16777619U;
  return hash;
}

static const motor_control_param_view_t *find_view(uint16_t motor_id) {
  for (size_t i = 0U; i < g_view_count; ++i) {
    if (g_views[i].motor_id == motor_id) return &g_views[i];
  }
  return nullptr;
}

static void set_str(rosidl_runtime_c__String &dst, char *buf, size_t cap, const char *src) {
  const size_t len = strlen(src);
  const size_t n = len < (cap - 1U) ? len : (cap - 1U);
  memcpy(buf, src, n);
  buf[n] = '\0';
  dst.data = buf;
  dst.size = n;
  dst.capacity = cap;
}

static uint32_t next_internal_id(void) {
  g_next_req_id = (g_next_req_id + 1U) & 0x3FFFFFFFU;
  if (g_next_req_id == 0U) g_next_req_id = 1U;
  return 0x40000000U | g_next_req_id;
}

void param_bridge_on_config_result(const motor_control_event_t *event) {
  if (!g_created || event == nullptr || event->type != MOTOR_CONTROL_EVENT_CONFIG_RESULT) return;
  if (event->request_id == g_pending_tx_id) {
    g_pending_tx_reason = event->reason;
    g_pending_tx_done = true;
  }
  if (event->reason != MOTOR_PROTOCOL_CONFIG_OK) {
    g_last_error = event->reason;
    g_last_error_motor = event->motor_id;
  }
}

static bool wait_config_tx(uint32_t req_id, uint32_t timeout_ms, uint16_t &reason_out) {
  const uint32_t clamped = timeout_ms == 0U ? 500U : (timeout_ms > 500U ? 500U : timeout_ms);
  const uint32_t deadline = millis() + clamped;
  while ((int32_t)(millis() - deadline) < 0) {
    motor_control_event_t events[16];
    const size_t count = motor_rt_drain_events(events, 16U);
    for (size_t i = 0U; i < count; ++i) {
      if (events[i].type == MOTOR_CONTROL_EVENT_CONFIG_RESULT &&
          events[i].request_id == req_id) {
        reason_out = events[i].reason;
        g_pending_tx_id = 0U;
        return true;
      }
    }
    if (g_pending_tx_done && g_pending_tx_id == req_id) {
      reason_out = g_pending_tx_reason;
      g_pending_tx_id = 0U;
      return true;
    }
    vTaskDelay(pdMS_TO_TICKS(1U));
  }
  g_pending_tx_id = 0U;
  return false;
}

static void submit_policy_cb(const void *req_in, void *res_out) {
  const auto &req = *static_cast<const can_bridge_msgs__srv__SubmitPolicy_Request *>(req_in);
  auto &res = *static_cast<can_bridge_msgs__srv__SubmitPolicy_Response *>(res_out);
  const uint64_t session = motor_bridge_session_id();
  res.accepted = false;
  res.session_id = session;
  res.policy_generation = 0U;
  res.request_id = 0U;
  set_str(res.intent_id, g_policy_intent_id, sizeof(g_policy_intent_id), "");
  set_str(res.reason, g_policy_reason, sizeof(g_policy_reason), "");

  if (!ros_bridge_is_active() || session == 0U || req.session_id != session) {
    set_str(res.reason, g_policy_reason, sizeof(g_policy_reason), "session mismatch or inactive");
    return;
  }
  if (req.targets.size == 0U || req.targets.size > kMotorCapacity) {
    set_str(res.reason, g_policy_reason, sizeof(g_policy_reason), "invalid target count");
    return;
  }

  uint64_t disable_mask = 0U;
  uint64_t seen_mask = 0U;
  bool any_enable = false;
  for (size_t i = 0U; i < req.targets.size; ++i) {
    const auto &p = req.targets.data[i];
    if (p.motor_id == 0U || p.motor_id > 64U ||
        !isfinite(p.kp) || !isfinite(p.kd) || p.kp < 0.0f || p.kd < 0.0f) {
      set_str(res.reason, g_policy_reason, sizeof(g_policy_reason), "invalid policy target");
      return;
    }
    const uint64_t bit = UINT64_C(1) << (p.motor_id - 1U);
    if (seen_mask & bit) {
      set_str(res.reason, g_policy_reason, sizeof(g_policy_reason), "duplicate motor_id");
      return;
    }
    seen_mask |= bit;
    if (p.enabled) any_enable = true;
    else disable_mask |= bit;
  }

  if (!any_enable) {
    motor_rt_request_disable(disable_mask);
    res.accepted = true;
    snprintf(g_policy_intent_id, sizeof(g_policy_intent_id), "fw:disable");
    res.intent_id.data = g_policy_intent_id;
    res.intent_id.size = strlen(g_policy_intent_id);
    res.intent_id.capacity = sizeof(g_policy_intent_id);
    return;
  }

  const uint32_t now = millis();
  const uint64_t lease_id = motor_bridge_ensure_lease(now);
  if (lease_id == 0U) {
    set_str(res.reason, g_policy_reason, sizeof(g_policy_reason), "lease unavailable");
    return;
  }
  const uint32_t gen = ++g_next_policy_gen;
  const uint32_t req_id = next_internal_id();
  control_policy_snapshot_t policy{};
  policy.request_id = req_id;
  policy.received_tick_ms = now;
  policy.valid_until_tick_ms = now + 80U;
  policy.policy_generation = gen;
  policy.lease_id = lease_id;
  policy.target_count = req.targets.size;
  for (size_t i = 0U; i < req.targets.size; ++i) {
    const auto &p = req.targets.data[i];
    policy.targets[i] = {p.motor_id, p.enabled, p.kp, p.kd, p.confirm_unprotected};
  }
  if (!motor_rt_submit_policy(&policy)) {
    set_str(res.reason, g_policy_reason, sizeof(g_policy_reason), "mailbox rejected policy");
    return;
  }
  res.accepted = true;
  res.policy_generation = gen;
  res.request_id = req_id;
  snprintf(g_policy_intent_id, sizeof(g_policy_intent_id), "fw:%lu", (unsigned long)gen);
  res.intent_id.data = g_policy_intent_id;
  res.intent_id.size = strlen(g_policy_intent_id);
  res.intent_id.capacity = sizeof(g_policy_intent_id);
}

static bool build_param_item(const can_bridge_msgs__msg__ParamSetTarget &t,
    control_mailbox_config_request_t &item) {
  memset(&item, 0, sizeof(item));
  item.motor_id = t.motor_id;
  if (!isfinite(t.value)) return false;
  switch (t.field) {
  case can_bridge_msgs__msg__ParamSetTarget__FIELD_POS_MIN:
  case can_bridge_msgs__msg__ParamSetTarget__FIELD_POS_MAX: {
    const auto *v = find_view(t.motor_id);
    if (!v) return false;
    item.kind = CONTROL_MAILBOX_CONFIG_SET_POSITION_LIMITS;
    item.value.position_limits.min_position_rad =
        t.field == can_bridge_msgs__msg__ParamSetTarget__FIELD_POS_MIN ? t.value : v->pos_min_rad;
    item.value.position_limits.max_position_rad =
        t.field == can_bridge_msgs__msg__ParamSetTarget__FIELD_POS_MAX ? t.value : v->pos_max_rad;
    return true;
  }
  case can_bridge_msgs__msg__ParamSetTarget__FIELD_VEL_LIMIT:
    item.kind = CONTROL_MAILBOX_CONFIG_SET_VELOCITY_LIMIT;
    item.value.max_velocity_rad_s = t.value;
    return true;
  case can_bridge_msgs__msg__ParamSetTarget__FIELD_TORQUE_LIMIT: {
    const auto *v = find_view(t.motor_id);
    if (!v || !isfinite(v->torque_per_amp) || v->torque_per_amp <= 0.0f) return false;
    item.kind = CONTROL_MAILBOX_CONFIG_SET_TORQUE_LIMIT;
    item.value.max_torque_nm = t.value * v->torque_per_amp;
    return true;
  }
  case can_bridge_msgs__msg__ParamSetTarget__FIELD_LIMIT_PROTECT:
    if (t.value != 0.0f && t.value != 1.0f) return false;
    item.kind = CONTROL_MAILBOX_CONFIG_SET_SAFETY_PROTECT;
    item.value.safety_protect_raw = t.value == 1.0f ? 0U : 1U; // 0=保护开
    return true;
  case can_bridge_msgs__msg__ParamSetTarget__FIELD_DIRECTION:
    if (t.value != 0.0f && t.value != 1.0f) return false;
    item.kind = CONTROL_MAILBOX_CONFIG_SET_DIRECTION;
    item.value.direction_raw = static_cast<uint32_t>(t.value);
    return true;
  case can_bridge_msgs__msg__ParamSetTarget__FIELD_T_MAX:
    if (t.value <= 0.0f) return false;
    item.kind = CONTROL_MAILBOX_CONFIG_SET_T_MAX;
    item.value.t_max_raw = static_cast<uint32_t>(lroundf(t.value));
    return true;
  case can_bridge_msgs__msg__ParamSetTarget__FIELD_NODE_ID:
    if (t.value < 1.0f || t.value > 64.0f) return false;
    item.kind = CONTROL_MAILBOX_CONFIG_SET_NODE_ID;
    item.value.new_node_id = static_cast<uint16_t>(lroundf(t.value));
    return true;
  default:
    return false;
  }
}

static void execute_config_cb(const void *req_in, void *res_out) {
  const auto &req = *static_cast<const can_bridge_msgs__srv__ExecuteConfig_Request *>(req_in);
  auto &res = *static_cast<can_bridge_msgs__srv__ExecuteConfig_Response *>(res_out);
  const auto &r = req.request;
  const uint64_t session = motor_bridge_session_id();

  auto &out = res.result;
  out.session_id = session;
  out.request_id = r.request_id;
  out.kind = r.kind;
  out.command = r.command;
  out.status = can_bridge_msgs__msg__ConfigResult__STATUS_FAILED;
  out.error_code = kInvalid;
  out.completed_targets = 0U;
  out.failed_targets = 0U;
  set_str(out.intent_id, g_config_res_intent, sizeof(g_config_res_intent), "");

  if (!ros_bridge_is_active() || session == 0U) {
    return;
  }
  g_view_count = motor_rt_copy_param_views(g_views, kMotorCapacity);

  if (r.kind == can_bridge_msgs__msg__ConfigRequest__CONFIG_KIND_PARAM_SET) {
    if (r.count == 0U || r.count > 16U || r.targets.size != r.count) {
      out.failed_targets = 1U;
      return;
    }
    uint16_t completed = 0U, failed = 0U, last_err = 0U;
    for (size_t i = 0U; i < r.count; ++i) {
      const auto &t = r.targets.data[i];
      const auto *v = find_view(t.motor_id);
      if (!v || !v->bound || !v->device_side_limits || v->enable) {
        ++failed; last_err = kInvalid; continue;
      }
      control_mailbox_config_request_t item{};
      if (!build_param_item(t, item)) {
        ++failed; last_err = kInvalid; continue;
      }
      const uint32_t tx_id = next_internal_id();
      item.request_id = tx_id;
      item.received_tick_ms = millis();
      item.valid_until_tick_ms = item.received_tick_ms + 500U;
      g_pending_tx_done = false;
      g_pending_tx_id = tx_id;
      if (!motor_rt_submit_config(&item)) {
        g_pending_tx_id = 0U;
        ++failed; last_err = kSubmitFailed; continue;
      }
      uint16_t reason = 0U;
      if (!wait_config_tx(tx_id, req.timeout_ms, reason)) {
        ++failed; last_err = kTimeout; continue;
      }
      if (reason == MOTOR_PROTOCOL_CONFIG_OK) ++completed;
      else { ++failed; last_err = reason; }
    }
    out.completed_targets = completed;
    out.failed_targets = failed;
    out.error_code = last_err;
    out.status = failed == 0U ? can_bridge_msgs__msg__ConfigResult__STATUS_SUCCEEDED
                              : can_bridge_msgs__msg__ConfigResult__STATUS_FAILED;
    return;
  }

  if (r.kind != can_bridge_msgs__msg__ConfigRequest__CONFIG_KIND_SYSTEM_COMMAND) {
    out.failed_targets = 1U;
    return;
  }

  if (r.command == can_bridge_msgs__msg__SystemCommand__COMMAND_REPORT) {
    out.status = can_bridge_msgs__msg__ConfigResult__STATUS_SUCCEEDED;
    out.error_code = 0U;
    out.completed_targets = static_cast<uint16_t>(g_view_count);
    return;
  }

  if (r.command == can_bridge_msgs__msg__SystemCommand__COMMAND_SYNC_W25) {
    for (size_t i = 0U; i < g_view_count; ++i) {
      if (g_views[i].enable) {
        out.error_code = kInvalid;
        out.failed_targets = 1U;
        return;
      }
    }
    if (storage_save_records(g_views, g_view_count)) {
      out.status = can_bridge_msgs__msg__ConfigResult__STATUS_SUCCEEDED;
      out.error_code = 0U;
      out.completed_targets = static_cast<uint16_t>(g_view_count);
    } else {
      out.error_code = kSubmitFailed;
      out.failed_targets = 1U;
    }
    return;
  }

  const auto *v = find_view(r.motor_id);
  if (!v || !v->bound) {
    out.error_code = kUnknownMotor;
    out.failed_targets = 1U;
    return;
  }
  if (v->enable && r.command != can_bridge_msgs__msg__SystemCommand__COMMAND_REFRESH) {
    out.status = can_bridge_msgs__msg__ConfigResult__STATUS_BUSY;
    out.error_code = kInvalid;
    out.failed_targets = 1U;
    return;
  }
  control_mailbox_config_request_t item{};
  item.motor_id = r.motor_id;
  if (r.command == can_bridge_msgs__msg__SystemCommand__COMMAND_REFRESH)
    item.kind = CONTROL_MAILBOX_CONFIG_REFRESH_PARAMS;
  else if (r.command == can_bridge_msgs__msg__SystemCommand__COMMAND_SET_ZERO)
    item.kind = CONTROL_MAILBOX_CONFIG_SET_ZERO;
  else if (r.command == can_bridge_msgs__msg__SystemCommand__COMMAND_CLEAR_FAULT)
    item.kind = CONTROL_MAILBOX_CONFIG_CLEAR_FAULT;
  else if (r.command == can_bridge_msgs__msg__SystemCommand__COMMAND_SAVE_DEVICE) {
    if (!v->device_side_limits) { out.failed_targets = 1U; return; }
    item.kind = CONTROL_MAILBOX_CONFIG_SAVE_DEVICE_FLASH;
    item.value.save_slots = kDeviceSaveMask;
  } else {
    out.failed_targets = 1U;
    return;
  }

  const uint32_t tx_id = next_internal_id();
  item.request_id = tx_id;
  item.received_tick_ms = millis();
  item.valid_until_tick_ms = item.received_tick_ms + 500U;
  g_pending_tx_done = false;
  g_pending_tx_id = tx_id;
  if (!motor_rt_submit_config(&item)) {
    g_pending_tx_id = 0U;
    out.error_code = kSubmitFailed;
    out.failed_targets = 1U;
    return;
  }
  uint16_t reason = 0U;
  if (!wait_config_tx(tx_id, req.timeout_ms, reason)) {
    out.status = can_bridge_msgs__msg__ConfigResult__STATUS_PENDING;
    out.error_code = kTimeout;
    out.failed_targets = 1U;
    return;
  }
  out.error_code = reason;
  if (reason == MOTOR_PROTOCOL_CONFIG_OK) {
    out.status = can_bridge_msgs__msg__ConfigResult__STATUS_SUCCEEDED;
    out.completed_targets = 1U;
  } else {
    out.failed_targets = 1U;
  }
}

static void encode_parameters(can_bridge_msgs__msg__DeviceParamEntry &out,
    const motor_control_param_view_t &v) {
  out.motor_id = v.motor_id;
  out.dev_init_state = v.dev_init;
  out.record_dirty = v.record_dirty_slots != 0U;
  out.mismatch_mask = v.dev_mismatch_mask;
  float rec[8], dev[8];
  for (size_t i = 0U; i < 8U; ++i) {
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

static void get_device_info_cb(const void *req_in, void *res_out) {
  const auto &req = *static_cast<const can_bridge_msgs__srv__GetDeviceInfo_Request *>(req_in);
  auto &res = *static_cast<can_bridge_msgs__srv__GetDeviceInfo_Response *>(res_out);
  auto &info = res.info;
  uint64_t epoch_ns = 0U;
  (void)time_sample_epoch(millis(), &epoch_ns);
  fill_epoch_stamp(&info.header.stamp, epoch_ns);
  info.header.frame_id.data = kNodeFrame;
  info.header.frame_id.size = sizeof(kNodeFrame) - 1U;
  info.header.frame_id.capacity = sizeof(kNodeFrame);
  info.schema_version = can_bridge_msgs__msg__MotorDeviceInfo__SCHEMA_VERSION;
  info.session_id = motor_bridge_session_id();
  info.info_revision = g_info_revision;

  g_view_count = motor_rt_copy_param_views(g_views, kMotorCapacity);
  const size_t topo_count = motor_rt_copy_topology(g_topology, kMotorCapacity);

  size_t topo_out = 0U;
  for (size_t i = 0U; i < topo_count && topo_out < kMotorCapacity; ++i) {
    if (req.motor_id != 0U && g_topology[i].motor_id != req.motor_id) continue;
    const auto &in = g_topology[i];
    auto &out = g_info_topo[topo_out];
    out.motor_id = in.motor_id;
    out.bus_index = in.bus_index;
    out.protocol = in.protocol;
    set_str(out.model_name, g_info_models[topo_out], sizeof(g_info_models[topo_out]), in.model_name);
    out.capability_flags = in.capability_flags;
    out.online = in.online;
    out.conflict = in.conflict;
    ++topo_out;
  }
  info.topology.size = topo_out;

  size_t param_out = 0U;
  for (size_t i = 0U; i < g_view_count && param_out < kMotorCapacity; ++i) {
    if (req.motor_id != 0U && g_views[i].motor_id != req.motor_id) continue;
    encode_parameters(g_info_params[param_out++], g_views[i]);
  }
  info.params.size = param_out;
}

void param_bridge_poll(void) {
  if (!g_created) return;
  g_view_count = motor_rt_copy_param_views(g_views, kMotorCapacity);
  const uint32_t hash = hash_bytes(2166136261U, g_views, g_view_count * sizeof(g_views[0]));
  if (hash != g_info_hash) {
    g_info_hash = hash;
    if (++g_info_revision == 0U) ++g_info_revision;
  }
}

void param_bridge_fill_system_state(can_bridge_msgs__msg__MotorSystemState *out) {
  if (!out) return;
  out->info_revision = g_info_revision;
  auto &sys = out->system_status;
  sys.refresh_seq = 0U;
  sys.set_zero_seq = 0U;
  sys.clear_fault_seq = 0U;
  sys.save_seq = 0U;
  sys.save_in_progress = false;
  sys.last_error = g_last_error;
  sys.last_error_motor = g_last_error_motor;
  const uint16_t frequencies[] = {FIRMWARE_AGENT_PING_HZ, FIRMWARE_MOTOR_REPORT_HZ,
      FIRMWARE_STATUS_REPORT_HZ, FIRMWARE_IMU_REPORT_HZ, FIRMWARE_TEMPERATURE_REPORT_HZ,
      FIRMWARE_MOTOR_COMMAND_HZ};
  sys.firmware_values_valid = true;
  memcpy(sys.firmware_values, frequencies, sizeof(frequencies));
  sys.firmware_unsaved = false;
}

bool param_bridge_create(rcl_node_t *node, rclc_executor_t *executor) {
  if (g_created) return true;
  if (!node || !executor) return false;
  memset(g_views, 0, sizeof(g_views));

  g_policy_req.header.frame_id.data = g_policy_req_frame;
  g_policy_req.header.frame_id.capacity = sizeof(g_policy_req_frame);
  g_policy_req.targets.data = g_policy_targets;
  g_policy_req.targets.capacity = kMotorCapacity;

  g_config_req.request.header.frame_id.data = g_config_req_frame;
  g_config_req.request.header.frame_id.capacity = sizeof(g_config_req_frame);
  g_config_req.request.targets.data = g_config_targets;
  g_config_req.request.targets.capacity = 16U;

  g_info_res.info.header.frame_id.data = g_info_frame;
  g_info_res.info.header.frame_id.capacity = sizeof(g_info_frame);
  g_info_res.info.topology.data = g_info_topo;
  g_info_res.info.topology.capacity = kMotorCapacity;
  g_info_res.info.params.data = g_info_params;
  g_info_res.info.params.capacity = kMotorCapacity;
  for (size_t i = 0U; i < kMotorCapacity; ++i) {
    g_info_topo[i].model_name.data = g_info_models[i];
    g_info_topo[i].model_name.capacity = sizeof(g_info_models[i]);
  }

  rcl_service_options_t srv_opts = rcl_service_get_default_options();
  srv_opts.qos = qos_reliable(4);
  if (rcl_service_init(&g_policy_srv, node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(can_bridge_msgs, srv, SubmitPolicy),
      "/motor/submit_policy", &srv_opts) != RCL_RET_OK) {
    param_bridge_destroy(node, executor); return false;
  }
  g_policy_up = true;

  if (rcl_service_init(&g_config_srv, node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(can_bridge_msgs, srv, ExecuteConfig),
      "/motor/execute_config", &srv_opts) != RCL_RET_OK) {
    param_bridge_destroy(node, executor); return false;
  }
  g_config_up = true;

  if (rcl_service_init(&g_info_srv, node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(can_bridge_msgs, srv, GetDeviceInfo),
      "/motor/get_device_info", &srv_opts) != RCL_RET_OK) {
    param_bridge_destroy(node, executor); return false;
  }
  g_info_up = true;

  if (rclc_executor_add_service(executor, &g_policy_srv, &g_policy_req,
      &g_policy_res, submit_policy_cb) != RCL_RET_OK) {
    param_bridge_destroy(node, executor); return false;
  }
  g_policy_added = true;

  if (rclc_executor_add_service(executor, &g_config_srv, &g_config_req,
      &g_config_res, execute_config_cb) != RCL_RET_OK) {
    param_bridge_destroy(node, executor); return false;
  }
  g_config_added = true;

  if (rclc_executor_add_service(executor, &g_info_srv, &g_info_req,
      &g_info_res, get_device_info_cb) != RCL_RET_OK) {
    param_bridge_destroy(node, executor); return false;
  }
  g_info_added = true;

  g_created = true;
  return true;
}

void param_bridge_destroy(rcl_node_t *node, rclc_executor_t *executor) {
  g_created = false;
  g_view_count = 0U;
  g_pending_tx_id = 0U;
  if (executor != nullptr) {
    if (g_policy_added) (void)rclc_executor_remove_service(executor, &g_policy_srv);
    if (g_config_added) (void)rclc_executor_remove_service(executor, &g_config_srv);
    if (g_info_added) (void)rclc_executor_remove_service(executor, &g_info_srv);
  }
  g_policy_added = g_config_added = g_info_added = false;
  if (g_policy_up) (void)rcl_service_fini(&g_policy_srv, node);
  if (g_config_up) (void)rcl_service_fini(&g_config_srv, node);
  if (g_info_up) (void)rcl_service_fini(&g_info_srv, node);
  g_policy_up = g_config_up = g_info_up = false;
}
