// motor_protocol adapter 层 host test（Phase A）。
// 验证点：帧归属/节点范围、解码 SI 换算（Profile 驱动）、编码帧 ID/长度/类别、
// 使能/软停/失能/发现帧、配置事务阶段表、型号目录数值。
// 约定：只走公共 API，不触碰内部状态（白盒断言已在 Phase A 移除）。

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "motor_control_profile.h"
#include "motor_protocol_adapter.h"

// ---------------------------------------------------------------------------
// 帧构造助手
// ---------------------------------------------------------------------------

static motor_protocol_raw_frame_t make_frame(uint8_t bus, uint32_t id, size_t len,
                                             const uint8_t *data, uint32_t tick) {
  motor_protocol_raw_frame_t f = {};
  f.bus_index = bus;
  f.can_id = id;
  f.is_extended = false;
  f.is_fd = true;
  f.is_brs = true;
  f.dlc = (uint8_t)len;
  f.rx_tick_ms = tick;
  if (data != nullptr && len > 0) {
    memcpy(f.data, data, len > MOTOR_PROTOCOL_MAX_FRAME_BYTES
                             ? MOTOR_PROTOCOL_MAX_FRAME_BYTES
                             : len);
  }
  return f;
}

static bool find_profile(const char *name, const motor_protocol_model_profile_t **out) {
  for (size_t i = 0; i < motor_control_profile_count(); ++i) {
    const motor_protocol_model_profile_t *p = motor_control_profile_at(i);
    if (strcmp(p->model_name, name) == 0) {
      *out = p;
      return true;
    }
  }
  return false;
}

static bool nan_equal(float a, float b) {
  return std::isnan(a) && std::isnan(b);
}

// ---------------------------------------------------------------------------
// vaxor adapter
// ---------------------------------------------------------------------------

// 帧归属与节点范围：reply/status 段可匹配，控制/命令段与陌生 ID 不匹配。
static void test_vaxor_match_rx_frame() {
  uint16_t node = 0;
  uint8_t data[MOTOR_PROTOCOL_MAX_FRAME_BYTES] = {};
  motor_protocol_raw_frame_t reply =
      make_frame(0, motor_protocol_wrist_reply_id(2), 12, data, 100);
  assert(motor_protocol_vaxor_adapter.match_rx_frame(&reply, &node));
  assert(node == 2);

  motor_protocol_raw_frame_t status =
      make_frame(0, motor_protocol_wrist_status_id(7), 20, data, 100);
  assert(motor_protocol_vaxor_adapter.match_rx_frame(&status, &node));
  assert(node == 7);

  // 命令/控制段是固件 TX 方向，不属于 RX 归属。
  motor_protocol_raw_frame_t cmd =
      make_frame(0, motor_protocol_wrist_command_id(2), 12, data, 100);
  assert(!motor_protocol_vaxor_adapter.match_rx_frame(&cmd, &node));

  motor_protocol_raw_frame_t alien = make_frame(0, 0x123, 8, data, 100);
  assert(!motor_protocol_vaxor_adapter.match_rx_frame(&alien, &node));

  assert(motor_protocol_adapter_valid_node(&motor_protocol_vaxor_adapter, 1));
  assert(motor_protocol_adapter_valid_node(&motor_protocol_vaxor_adapter, 60));
  assert(!motor_protocol_adapter_valid_node(&motor_protocol_vaxor_adapter, 0));
  assert(!motor_protocol_adapter_valid_node(&motor_protocol_vaxor_adapter, 61));
}

// 解码 12B reply：SI 关节侧换算由 Profile 驱动（/减速比，Iq×Kt×减速比×效率）。
static void test_vaxor_decode_reply_converts_si() {
  const motor_protocol_model_profile_t *profile = nullptr;
  assert(find_profile("vaxor_2550", &profile));

  const double kMotorPos = 1.0;  // 电机轴 rad
  const double kMotorVel = 2.0;  // 电机轴 rad/s
  const double kCurrentA = 0.5;  // Iq (A)
  motor_protocol_joint_state_t wire = {kMotorPos, kMotorVel, kCurrentA};
  uint8_t data[12];
  motor_protocol_pack_wrist_reply(2, &wire, data);

  motor_protocol_raw_frame_t frame =
      make_frame(0, motor_protocol_wrist_reply_id(2), 12, data, 500);
  motor_protocol_measured_state_t measured = {};
  assert(motor_protocol_vaxor_adapter.decode_frame(profile, 2, &frame, &measured));

  assert(measured.has_motion_state);
  assert(!measured.is_status_frame);
  assert(!measured.fault_active);
  // 关节侧 = 电机侧 / 50
  assert(fabs(measured.position_rad - kMotorPos / 50.0) < 1e-4);
  assert(fabs(measured.velocity_rad_s - kMotorVel / 50.0) < 1e-3);
  // 关节扭矩 = Iq × Kt × 减速比 × 效率 = 0.5 × 0.042 × 50 × 0.65 = 0.6825
  assert(fabs(measured.effort_nm - 0.5 * 0.042 * 50.0 * 0.65) < 5e-3);
  assert(fabs(measured.current_a - kCurrentA) < 1e-3);
  assert(nan_equal(measured.temperature_c, NAN)); // 无温度来源
}

// 解码 20B 状态帧：运动量 + 身份/诊断字段（fault/motor_type 透传）。
static void test_vaxor_decode_status_frame() {
  const motor_protocol_model_profile_t *profile = nullptr;
  assert(find_profile("vaxor_2530", &profile));

  motor_protocol_joint_state_t wire = {0.0, 0.0, 0.25};
  uint8_t data[20];
  motor_protocol_pack_wrist_status(3, &wire, 48.6, 0x12345678, 4, 5, 2, 7, data);

  motor_protocol_raw_frame_t frame =
      make_frame(1, motor_protocol_wrist_status_id(3), 20, data, 600);
  motor_protocol_measured_state_t measured = {};
  assert(motor_protocol_vaxor_adapter.decode_frame(profile, 3, &frame, &measured));

  assert(measured.is_status_frame);
  assert(measured.has_motion_state);
  assert(fabs(measured.voltage_v - 48.6) < 1e-4); // float 存储精度
  assert(measured.vendor_fault_code == 0x12345678U);
  assert(measured.vendor_state == 4);
  assert(measured.calibration_state == 5);
  assert(measured.drive_mode == 2);
  assert(measured.motor_type == 7);
  // 故障字非零 → fault_active（内核负责降级，adapter 只标注）。
  assert(measured.fault_active);
  // 30 速比：扭矩 = 0.25 × 0.042 × 30 × 0.70
  assert(fabs(measured.effort_nm - 0.25 * 0.042 * 30.0 * 0.70) < 5e-3);
}

// 未绑定 Profile 时运动量为 NaN，但状态/身份字段照常（发现/识别依赖）。
static void test_vaxor_decode_without_profile_yields_nan_motion() {
  motor_protocol_joint_state_t wire = {1.0, 1.0, 0.1};
  uint8_t data[12];
  motor_protocol_pack_wrist_reply(4, &wire, data);
  motor_protocol_raw_frame_t frame =
      make_frame(0, motor_protocol_wrist_reply_id(4), 12, data, 100);
  motor_protocol_measured_state_t measured = {};
  assert(motor_protocol_vaxor_adapter.decode_frame(nullptr, 4, &frame, &measured));
  assert(nan_equal(measured.position_rad, NAN));
  assert(nan_equal(measured.velocity_rad_s, NAN));
  assert(nan_equal(measured.effort_nm, NAN));
  assert(fabs(measured.current_a - 0.1) < 1e-3);
}

// 编码 MIT 命令：关节侧 SI → 电机侧线协议（位置/速度 ×减速比；
// 前馈 N·m → Iq(A) = τ/(Kt·减速比·效率)，修正旧路径的单位混用）。
static void test_vaxor_encode_mit_command() {
  const motor_protocol_model_profile_t *profile = nullptr;
  assert(find_profile("vaxor_2550", &profile));

  // Kt×减速比×效率 = 0.042×50×0.65 = 1.365 → 1.365 N·m 恰为 1.0 A。
  motor_protocol_control_intent_t intent = {};
  intent.position_rad = 0.5f;  // 关节侧
  intent.velocity_rad_s = 1.0f;
  intent.effort_nm = 1.365f;
  intent.kp = 0.008f;
  intent.kd = 1.0f;
  intent.enable = true;

  motor_protocol_tx_frame_t out = {};
  assert(motor_protocol_vaxor_adapter.encode_mit_command(profile, 2, 1, &intent, &out));
  assert(out.bus_index == 1);
  assert(out.tx_class == MOTOR_PROTOCOL_TX_CLASS_P1);
  assert(out.can_id == motor_protocol_wrist_command_id(2));
  assert(out.is_fd);
  assert(!out.is_extended);
  assert(out.size == 12);

  // 用协议解码器反向验证线协议内容（线协议字段为电机侧/电流 A）。
  motor_protocol_joint_command_t wire = {};
  assert(motor_protocol_unpack_wrist_command(out.data, out.size, &wire));
  assert(fabs(wire.position - 0.5 * 50.0) < 1e-3);          // 电机轴 rad
  assert(fabs(wire.velocity - 1.0 * 50.0) < 0.02);          // 电机轴 rad/s
  assert(fabs(wire.feedforward - 1.0) < 0.005);             // 1.365 N·m → 1 A
  assert(fabs(wire.kp - 0.008) < 1e-4);
  assert(fabs(wire.kd - 1.0) < 0.02);
}

// 使能帧：control(run=1, MIT) 8B 非 FD，P1。
static void test_vaxor_encode_enable() {
  const motor_protocol_model_profile_t *profile = nullptr;
  assert(find_profile("vaxor_2550", &profile));
  motor_protocol_tx_frame_t out = {};
  assert(motor_protocol_vaxor_adapter.encode_enable(profile, 5, 2, &out));
  assert(out.can_id == motor_protocol_wrist_control_id(5));
  assert(out.size == 8);
  assert(!out.is_fd);
  assert(out.tx_class == MOTOR_PROTOCOL_TX_CLASS_P1);
  motor_protocol_wrist_control_t control = {};
  assert(motor_protocol_unpack_wrist_control(out.data, out.size, &control));
  assert(control.run);
  assert(!control.set_zero);
  assert(control.mode == MOTOR_PROTOCOL_WRIST_MODE_MIT);
}

// 软停帧：MIT 阻尼 {0,0,0,default_kd,0} 且保持使能（TBD-SAFETY-01 占位语义），
// 类别必须是 P0。
static void test_vaxor_encode_soft_stop() {
  const motor_protocol_model_profile_t *profile = nullptr;
  assert(find_profile("vaxor_2550", &profile));
  motor_protocol_tx_frame_t out = {};
  assert(motor_protocol_vaxor_adapter.encode_soft_stop(profile, 1, 0, &out));
  assert(out.tx_class == MOTOR_PROTOCOL_TX_CLASS_P0);
  assert(out.can_id == motor_protocol_wrist_command_id(1));
  assert(out.size == 12);
  motor_protocol_joint_command_t wire = {};
  assert(motor_protocol_unpack_wrist_command(out.data, out.size, &wire));
  assert(wire.position == 0.0);
  assert(wire.velocity == 0.0);
  assert(wire.kp == 0.0);
  assert(fabs(wire.kd - profile->default_kd) < 0.02);
  assert(wire.feedforward == 0.0);
}

// 失能帧：control(run=0) 8B，P0。
static void test_vaxor_encode_disable() {
  const motor_protocol_model_profile_t *profile = nullptr;
  assert(find_profile("vaxor_2550", &profile));
  motor_protocol_tx_frame_t out = {};
  assert(motor_protocol_vaxor_adapter.encode_disable(profile, 1, 0, &out));
  assert(out.tx_class == MOTOR_PROTOCOL_TX_CLASS_P0);
  assert(out.can_id == motor_protocol_wrist_control_id(1));
  assert(out.size == 8);
  motor_protocol_wrist_control_t control = {};
  assert(motor_protocol_unpack_wrist_control(out.data, out.size, &control));
  assert(!control.run);
}

// vaxor 无 SYNC：发现探测必须拒绝（0x080 只属于 CREB 发现路径）。
static void test_vaxor_no_discovery_sync() {
  motor_protocol_tx_frame_t out = {};
  assert(!motor_protocol_vaxor_adapter.encode_discovery_sync(0, &out));
}

// 配置事务：SET_MODE 是两阶段无响应事务（stop→start）；
// SET_GAINS 是本地策略，不是事务（阶段数 0 → 内核走本地路径）。
static void test_vaxor_config_mode_transaction_stages() {
  const motor_protocol_model_profile_t *profile = nullptr;
  assert(find_profile("vaxor_2550", &profile));

  motor_protocol_config_request_t request = {};
  request.request_id = 42U;
  request.kind = MOTOR_PROTOCOL_CONFIG_SET_MODE;
  request.mode = MOTOR_PROTOCOL_WRIST_MODE_MIT;

  assert(motor_protocol_vaxor_adapter.config_stage_count(profile, &request) == 2);

  motor_protocol_config_stage_info_t info = {};
  assert(motor_protocol_vaxor_adapter.config_stage_info(0, &request, &info));
  assert(!info.waits_response);

  motor_protocol_tx_frame_t out = {};
  assert(motor_protocol_vaxor_adapter.config_encode_stage(0, profile, 2, 0, &request, &out));
  assert(out.can_id == motor_protocol_wrist_control_id(2));
  motor_protocol_wrist_control_t control = {};
  assert(motor_protocol_unpack_wrist_control(out.data, out.size, &control));
  assert(!control.run);
  assert(control.mode == MOTOR_PROTOCOL_WRIST_MODE_MIT);

  assert(motor_protocol_vaxor_adapter.config_encode_stage(1, profile, 2, 0, &request, &out));
  assert(motor_protocol_unpack_wrist_control(out.data, out.size, &control));
  assert(control.run);
  assert(control.mode == MOTOR_PROTOCOL_WRIST_MODE_MIT);

  motor_protocol_config_request_t gains = {};
  gains.kind = MOTOR_PROTOCOL_CONFIG_SET_GAINS;
  assert(motor_protocol_vaxor_adapter.config_stage_count(profile, &gains) == 0);
}

// ---------------------------------------------------------------------------
// CREB adapter
// ---------------------------------------------------------------------------

static motor_protocol_raw_frame_t creb_feedback(uint16_t node, int16_t pos_deg_counts,
                                                int16_t vel_rpm, int16_t effort_ma,
                                                uint8_t tail_byte, uint32_t tick) {
  uint8_t data[12] = {};
  data[0] = (uint8_t)((uint16_t)pos_deg_counts >> 8);
  data[1] = (uint8_t)((uint16_t)pos_deg_counts & 0xff);
  data[2] = (uint8_t)((uint16_t)vel_rpm >> 8);
  data[3] = (uint8_t)((uint16_t)vel_rpm & 0xff);
  data[4] = (uint8_t)((uint16_t)effort_ma >> 8);
  data[5] = (uint8_t)((uint16_t)effort_ma & 0xff);
  data[11] = tail_byte;
  return make_frame(0, 0x300U + node, 12, data, tick);
}

// 解码：位置为关节侧原样（deg→rad）；速度为电机侧 rpm→rad/s 后 /减速比；
// 扭矩 = Iq(A)×Kt×减速比。故障位（data[11]&0x20）标注 fault_active 且不丢帧。
static void test_creb_decode_feedback_converts_si() {
  const motor_protocol_model_profile_t *profile = nullptr;
  assert(find_profile("creb_r58_101", &profile));

  motor_protocol_raw_frame_t frame = creb_feedback(3, 1820, 120, 2000, 0x00, 700);
  motor_protocol_measured_state_t measured = {};
  assert(motor_protocol_creb_adapter.decode_frame(profile, 3, &frame, &measured));

  assert(measured.has_motion_state);
  assert(!measured.fault_active);
  assert(fabs(measured.position_rad - 10.0 * M_PI / 180.0) < 1e-4); // 关节侧
  assert(fabs(measured.velocity_rad_s - (120.0 * 2.0 * M_PI / 60.0) / 101.0) < 1e-4);
  assert(fabs(measured.effort_nm - 2.0 * 0.086 * 101.0) < 1e-3);
  assert(fabs(measured.current_a - 2.0) < 1e-6);

  // 故障位帧：fault_active 置位，运动量缺失（NaN），帧本身有效。
  motor_protocol_raw_frame_t faulted = creb_feedback(3, 1820, 120, 2000, 0x20, 701);
  motor_protocol_measured_state_t fault_state = {};
  assert(motor_protocol_creb_adapter.decode_frame(profile, 3, &faulted, &fault_state));
  assert(fault_state.fault_active);
  assert(nan_equal(fault_state.position_rad, NAN));
}

// 编码 MIT：关节侧 SI 直通（CREB 线协议即关节侧），使能位进控制字。
static void test_creb_encode_mit_command() {
  const motor_protocol_model_profile_t *profile = nullptr;
  assert(find_profile("creb_r58_101", &profile));

  motor_protocol_control_intent_t intent = {};
  intent.position_rad = 1.0f;
  intent.velocity_rad_s = -2.0f;
  intent.effort_nm = 5.0f;
  intent.kp = 30.0f;
  intent.kd = 1.0f;
  intent.enable = true;

  motor_protocol_tx_frame_t out = {};
  assert(motor_protocol_creb_adapter.encode_mit_command(profile, 9, 2, &intent, &out));
  assert(out.bus_index == 2);
  assert(out.can_id == 0x110U + 9U);
  assert(out.is_fd);
  assert(out.size == 12);
  assert(out.tx_class == MOTOR_PROTOCOL_TX_CLASS_P1);
  assert(out.data[0] == 0xccU); // 使能位 + MIT 模式
  // 与 codec 直连打包逐字节一致（passthrough 语义）。
  motor_protocol_joint_command_t wire = {1.0, -2.0, 30.0, 1.0, 5.0};
  motor_protocol_creb_limits_t limits = {profile->mit_position_limit_rad,
                                         profile->mit_velocity_limit_rad_s,
                                         profile->mit_torque_limit_nm};
  uint8_t expected[12];
  motor_protocol_pack_creb_mit_frame(&wire, &limits, true, expected);
  assert(memcmp(out.data, expected, 12) == 0);
}

// 使能/失能帧：中性目标 + 控制字使能位（0xcc 使能 / 0x0c 失能）。
static void test_creb_enable_disable_frames() {
  const motor_protocol_model_profile_t *profile = nullptr;
  assert(find_profile("creb_r48_101", &profile));

  motor_protocol_tx_frame_t enable = {};
  assert(motor_protocol_creb_adapter.encode_enable(profile, 1, 0, &enable));
  assert(enable.can_id == 0x111U);
  assert(enable.size == 12);
  assert(enable.data[0] == 0xccU);

  motor_protocol_tx_frame_t disable = {};
  assert(motor_protocol_creb_adapter.encode_disable(profile, 1, 0, &disable));
  assert(disable.tx_class == MOTOR_PROTOCOL_TX_CLASS_P0);
  assert(disable.data[0] == 0x0cU);
}

// 软停帧：阻尼 kd=Profile 默认，保持使能，P0。
static void test_creb_encode_soft_stop() {
  const motor_protocol_model_profile_t *profile = nullptr;
  assert(find_profile("creb_r48_101", &profile));
  motor_protocol_tx_frame_t out = {};
  assert(motor_protocol_creb_adapter.encode_soft_stop(profile, 2, 1, &out));
  assert(out.tx_class == MOTOR_PROTOCOL_TX_CLASS_P0);
  assert(out.data[0] == 0xccU); // 保持使能
  // kd=1.0 量化到 12bit/[0,5] → 819 = 0x333，位于 kd 字段（byte6 高 4 位 + byte7 低 4 位前段）。
  // 与 codec 直连一致性校验：
  motor_protocol_joint_command_t wire = {0.0, 0.0, 0.0, profile->default_kd, 0.0};
  motor_protocol_creb_limits_t limits = {profile->mit_position_limit_rad,
                                         profile->mit_velocity_limit_rad_s,
                                         profile->mit_torque_limit_nm};
  uint8_t expected[12];
  motor_protocol_pack_creb_mit_frame(&wire, &limits, true, expected);
  assert(memcmp(out.data, expected, 12) == 0);
}

// 发现帧：0x080、零长度、经典帧、P2。
static void test_creb_encode_discovery_sync() {
  motor_protocol_tx_frame_t out = {};
  assert(motor_protocol_creb_adapter.encode_discovery_sync(2, &out));
  assert(out.can_id == 0x080U);
  assert(out.size == 0);
  assert(!out.is_fd);
  assert(out.tx_class == MOTOR_PROTOCOL_TX_CLASS_P2);
  assert(out.bus_index == 2);
}

// 配置事务：非 MIT 模式与限位写均不受支持（0 阶段 → 内核回 UNSUPPORTED；
// 限位厂家对象属 TBD-CONFIG-01，不猜）。
static void test_creb_config_unsupported_paths() {
  const motor_protocol_model_profile_t *profile = nullptr;
  assert(find_profile("creb_r58_101", &profile));

  motor_protocol_config_request_t mode = {};
  mode.kind = MOTOR_PROTOCOL_CONFIG_SET_MODE;
  mode.mode = MOTOR_PROTOCOL_WRIST_MODE_VELOCITY;
  assert(motor_protocol_creb_adapter.config_stage_count(profile, &mode) == 0);

  motor_protocol_config_request_t limits = {};
  limits.kind = MOTOR_PROTOCOL_CONFIG_SET_POSITION_LIMITS;
  limits.position_limits.min_position_rad = -1.0f;
  limits.position_limits.max_position_rad = 1.0f;
  assert(motor_protocol_creb_adapter.config_stage_count(profile, &limits) == 0);
}

// 节点范围与 ID 段不重叠（同一 CAN ID 不得同时归属两个协议）。
static void test_adapter_id_ranges_disjoint() {
  // vaxor reply 最低 0x701+4 = 0x705；CREB feedback 最高 0x300+127 = 0x37f。
  assert(0x300U + 127U < 0x701U + 4U);
  // vaxor command 最低 0x501+4 = 0x505 > 0x37f。
  assert(0x37fU < 0x505U);
}

// ---------------------------------------------------------------------------
// 型号目录（数值来源 = ROS 型号库，TBD-PROFILE-01 锚点）
// ---------------------------------------------------------------------------

static void test_profile_catalog_baselines() {
  assert(motor_control_profile_count() == 4U);

  const motor_protocol_model_profile_t *p = nullptr;
  assert(find_profile("vaxor_2550", &p));
  assert(p->protocol == &motor_protocol_vaxor_adapter);
  assert(p->gear_ratio == 50.0f);
  assert(p->gear_efficiency == 0.65f);
  assert(fabs(p->torque_constant_nm_per_amp - 0.042) < 1e-6);
  assert((p->capability_flags & MOTOR_PROTOCOL_CAP_LIMIT_SOFTWARE) != 0U);
  assert((p->capability_flags & MOTOR_PROTOCOL_CAP_AUTONOMOUS_FEEDBACK) != 0U);

  assert(find_profile("vaxor_2530", &p));
  assert(p->gear_ratio == 30.0f);
  assert(p->gear_efficiency == 0.70f);

  assert(find_profile("creb_r58_101", &p));
  assert(p->protocol == &motor_protocol_creb_adapter);
  assert(p->gear_ratio == 101.0f);
  assert(fabs(p->torque_constant_nm_per_amp - 0.086) < 1e-6);
  assert(p->rated_torque_nm == 20.0f);
  assert(p->peak_torque_nm == 45.0f);
  assert((p->capability_flags & MOTOR_PROTOCOL_CAP_LIMIT_DEVICE_SIDE) != 0U);

  assert(find_profile("creb_r48_101", &p));
  assert(fabs(p->torque_constant_nm_per_amp - 0.077) < 1e-6);
  assert(p->rated_torque_nm == 5.0f);
  assert(p->peak_torque_nm == 11.0f);

  // Vaxor 80 速比刻意缺席：按名字找不到（禁止项 15）。
  assert(!find_profile("vaxor_2580", &p));
  for (size_t i = 0; i < motor_control_profile_count(); ++i) {
    const motor_protocol_model_profile_t *row = motor_control_profile_at(i);
    assert(row->gear_ratio != 80.0f);
  }
}

int main() {
  test_vaxor_match_rx_frame();
  test_vaxor_decode_reply_converts_si();
  test_vaxor_decode_status_frame();
  test_vaxor_decode_without_profile_yields_nan_motion();
  test_vaxor_encode_mit_command();
  test_vaxor_encode_enable();
  test_vaxor_encode_soft_stop();
  test_vaxor_encode_disable();
  test_vaxor_no_discovery_sync();
  test_vaxor_config_mode_transaction_stages();
  test_creb_decode_feedback_converts_si();
  test_creb_encode_mit_command();
  test_creb_enable_disable_frames();
  test_creb_encode_soft_stop();
  test_creb_encode_discovery_sync();
  test_creb_config_unsupported_paths();
  test_adapter_id_ranges_disjoint();
  test_profile_catalog_baselines();
  printf("test_motor_protocol_adapter: all passed\n");
  return 0;
}
