// motor_protocol adapter 层 host test。
// 验证点：帧归属/节点范围、解码 SI 换算（Profile 驱动）、编码帧 ID/长度/类别、
// 使能/软停/失能/发现帧、配置事务阶段表、型号目录数值。
// 只走公共 API，不触碰内部状态。

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

static motor_protocol_tx_frame_t make_cerb_mit_tx(uint8_t bus, uint8_t node) {
  motor_protocol_tx_frame_t frame = {};
  frame.bus_index = bus;
  frame.tx_class = MOTOR_PROTOCOL_TX_CLASS_P1;
  frame.tx_slot = node;
  frame.can_id = MOTOR_PROTOCOL_CERB_COMMAND_ID_BASE + node;
  frame.is_fd = true;
  frame.is_brs = true;
  frame.size = 12U;
  frame.data[0] = 0x0CU;
  for (uint8_t i = 1U; i < 9U; ++i) {
    frame.data[i] = (uint8_t)(node + i);
  }
  return frame;
}

static void test_p1_batch_passes_through_unbatchable_frames() {
  assert(motor_protocol_p1_batch_max() == MOTOR_PROTOCOL_CERB_MULTI_MAX_AXES);
  assert(motor_protocol_aggregate_p1_frames(nullptr, 1U, nullptr) == 0U);

  motor_protocol_tx_frame_t input = make_cerb_mit_tx(1U, 3U);
  motor_protocol_tx_frame_t output = {};
  assert(motor_protocol_aggregate_p1_frames(&input, 1U, &output) == 1U);
  assert(memcmp(&input, &output, sizeof(input)) == 0);

  input.tx_class = MOTOR_PROTOCOL_TX_CLASS_P0;
  assert(motor_protocol_aggregate_p1_frames(&input, 1U, &output) == 1U);
  assert(memcmp(&input, &output, sizeof(input)) == 0);

  motor_protocol_tx_frame_t vaxor = {};
  vaxor.bus_index = 1U;
  vaxor.tx_class = MOTOR_PROTOCOL_TX_CLASS_P1;
  vaxor.can_id = 0x01U;
  vaxor.size = 8U;
  assert(motor_protocol_aggregate_p1_frames(&vaxor, 1U, &output) == 1U);
  assert(memcmp(&vaxor, &output, sizeof(vaxor)) == 0);
}

static void test_cerb_p1_batch_aggregates_two_to_six_axes() {
  motor_protocol_tx_frame_t inputs[MOTOR_PROTOCOL_CERB_MULTI_MAX_AXES] = {};
  for (uint8_t count = 2U; count <= MOTOR_PROTOCOL_CERB_MULTI_MAX_AXES; ++count) {
    for (uint8_t i = 0U; i < count; ++i) {
      inputs[i] = make_cerb_mit_tx(2U, (uint8_t)(i + 1U));
    }
    motor_protocol_tx_frame_t output = {};
    assert(motor_protocol_aggregate_p1_frames(inputs, count, &output) == count);
    assert(output.bus_index == 2U);
    assert(output.tx_class == MOTOR_PROTOCOL_TX_CLASS_P1);
    assert(output.tx_slot == UINT16_MAX);
    assert(output.can_id == MOTOR_PROTOCOL_CERB_MULTI_COMMAND_ID);
    assert(!output.is_extended && output.is_fd && output.is_brs);
    assert(output.size == 64U);
    for (uint8_t i = 0U; i < count; ++i) {
      assert(memcmp(&output.data[i * 9U], &inputs[i].data[0], 9U) == 0);
      assert(output.data[56U + i] == (uint8_t)(i + 1U));
    }
    for (size_t i = (size_t)count * 9U; i < 56U; ++i) {
      assert(output.data[i] == 0U);
    }
    for (uint8_t i = count; i < MOTOR_PROTOCOL_CERB_MULTI_MAX_AXES; ++i) {
      assert(output.data[56U + i] == 0U);
    }
  }
}

static void test_p1_batch_stops_at_incompatible_or_duplicate_frame() {
  motor_protocol_tx_frame_t inputs[3] = {
    make_cerb_mit_tx(0U, 1U), {}, make_cerb_mit_tx(0U, 2U)};
  inputs[1].bus_index = 0U;
  inputs[1].tx_class = MOTOR_PROTOCOL_TX_CLASS_P1;
  inputs[1].can_id = 0x01U;
  inputs[1].size = 8U;
  motor_protocol_tx_frame_t output = {};
  assert(motor_protocol_aggregate_p1_frames(inputs, 3U, &output) == 1U);
  assert(memcmp(&inputs[0], &output, sizeof(output)) == 0);

  inputs[1] = make_cerb_mit_tx(0U, 1U);
  assert(motor_protocol_aggregate_p1_frames(inputs, 3U, &output) == 1U);
  assert(memcmp(&inputs[0], &output, sizeof(output)) == 0);

  inputs[1] = make_cerb_mit_tx(1U, 2U);
  assert(motor_protocol_aggregate_p1_frames(inputs, 3U, &output) == 1U);
  assert(memcmp(&inputs[0], &output, sizeof(output)) == 0);
}

// ---------------------------------------------------------------------------
// vaxor adapter
// ---------------------------------------------------------------------------

// 帧归属与节点范围：reply/status 段可匹配，控制/命令段与陌生 ID 不匹配。
static void test_vaxor_match_rx_frame() {
  uint16_t node = 0;
  uint8_t data[MOTOR_PROTOCOL_MAX_FRAME_BYTES] = {};
  motor_protocol_raw_frame_t reply =
      make_frame(0, motor_protocol_vaxor_reply_id(2), 12, data, 100);
  assert(motor_protocol_vaxor_adapter.match_rx_frame(&reply, &node));
  assert(node == 2);

  motor_protocol_raw_frame_t status =
      make_frame(0, motor_protocol_vaxor_status_id(7), 20, data, 100);
  assert(motor_protocol_vaxor_adapter.match_rx_frame(&status, &node));
  assert(node == 7);

  // 命令/控制段是固件 TX 方向，不属于 RX 归属。
  motor_protocol_raw_frame_t cmd =
      make_frame(0, motor_protocol_vaxor_command_id(2), 12, data, 100);
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
  motor_protocol_pack_vaxor_reply(2, &wire, data);

  motor_protocol_raw_frame_t frame =
      make_frame(0, motor_protocol_vaxor_reply_id(2), 12, data, 500);
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
  assert(nan_equal(measured.temperature_c, NAN)); // vaxor 反馈无温度字段
}

// 解码 20B 状态帧：运动量 + 身份/诊断字段（fault/motor_type 透传）。
static void test_vaxor_decode_status_frame() {
  const motor_protocol_model_profile_t *profile = nullptr;
  assert(find_profile("vaxor_2550", &profile));

  motor_protocol_joint_state_t wire = {0.0, 0.0, 0.25};
  uint8_t data[20];
  motor_protocol_pack_vaxor_status(3, &wire, 48.6, 0x12345678, 4, 5, 2, 7, data);

  motor_protocol_raw_frame_t frame =
      make_frame(1, motor_protocol_vaxor_status_id(3), 20, data, 600);
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
  // 50 速比：扭矩 = 0.25 × 0.042 × 50 × 0.65
  assert(fabs(measured.effort_nm - 0.25 * 0.042 * 50.0 * 0.65) < 5e-3);
}

// 未绑定 Profile 时运动量为 NaN，但状态/身份字段照常（发现/识别依赖）。
static void test_vaxor_decode_without_profile_yields_nan_motion() {
  motor_protocol_joint_state_t wire = {1.0, 1.0, 0.1};
  uint8_t data[12];
  motor_protocol_pack_vaxor_reply(4, &wire, data);
  motor_protocol_raw_frame_t frame =
      make_frame(0, motor_protocol_vaxor_reply_id(4), 12, data, 100);
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
  assert(out.can_id == motor_protocol_vaxor_command_id(2));
  assert(out.is_fd);
  assert(!out.is_extended);
  assert(out.size == 12);

  // 用协议解码器反向验证线协议内容（线协议字段为电机侧/电流 A）。
  motor_protocol_joint_command_t wire = {};
  assert(motor_protocol_unpack_vaxor_command(out.data, out.size, &wire));
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
  assert(out.can_id == motor_protocol_vaxor_control_id(5));
  assert(out.size == 8);
  assert(!out.is_fd);
  assert(out.tx_class == MOTOR_PROTOCOL_TX_CLASS_P1);
  motor_protocol_vaxor_control_t control = {};
  assert(motor_protocol_unpack_vaxor_control(out.data, out.size, &control));
  assert(control.run);
  assert(!control.set_zero);
  assert(control.mode == MOTOR_PROTOCOL_MODE_MIT);
}

// 软停帧：MIT 阻尼 {0,0,0,default_kd,0} 且保持使能（TBD-SAFETY-01 占位语义），
// 类别必须是 P0。
static void test_vaxor_encode_soft_stop() {
  const motor_protocol_model_profile_t *profile = nullptr;
  assert(find_profile("vaxor_2550", &profile));
  motor_protocol_tx_frame_t out = {};
  assert(motor_protocol_vaxor_adapter.encode_soft_stop(profile, 1, 0, &out));
  assert(out.tx_class == MOTOR_PROTOCOL_TX_CLASS_P0);
  assert(out.can_id == motor_protocol_vaxor_command_id(1));
  assert(out.size == 12);
  motor_protocol_joint_command_t wire = {};
  assert(motor_protocol_unpack_vaxor_command(out.data, out.size, &wire));
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
  assert(out.can_id == motor_protocol_vaxor_control_id(1));
  assert(out.size == 8);
  motor_protocol_vaxor_control_t control = {};
  assert(motor_protocol_unpack_vaxor_control(out.data, out.size, &control));
  assert(!control.run);
}

// vaxor 无 SYNC：发现探测必须拒绝（0x080 只属于 CERB 发现路径）。
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
  request.mode = MOTOR_PROTOCOL_MODE_MIT;

  assert(motor_protocol_vaxor_adapter.config_stage_count(profile, &request) == 2);

  motor_protocol_config_stage_info_t info = {};
  assert(motor_protocol_vaxor_adapter.config_stage_info(0, profile, &request, &info));
  assert(!info.waits_response);

  motor_protocol_tx_frame_t out = {};
  assert(motor_protocol_vaxor_adapter.config_encode_stage(0, profile, 2, 0, &request, &out));
  assert(out.can_id == motor_protocol_vaxor_control_id(2));
  motor_protocol_vaxor_control_t control = {};
  assert(motor_protocol_unpack_vaxor_control(out.data, out.size, &control));
  assert(!control.run);
  assert(control.mode == MOTOR_PROTOCOL_MODE_MIT);

  assert(motor_protocol_vaxor_adapter.config_encode_stage(1, profile, 2, 0, &request, &out));
  assert(motor_protocol_unpack_vaxor_control(out.data, out.size, &control));
  assert(control.run);
  assert(control.mode == MOTOR_PROTOCOL_MODE_MIT);

  motor_protocol_config_request_t gains = {};
  gains.kind = MOTOR_PROTOCOL_CONFIG_SET_GAINS;
  assert(motor_protocol_vaxor_adapter.config_stage_count(profile, &gains) == 0);
}

// ---------------------------------------------------------------------------
// CERB adapter
// ---------------------------------------------------------------------------

static motor_protocol_raw_frame_t cerb_feedback(uint16_t node, int16_t pos_deg_counts,
    int16_t vel_rpm, int16_t effort_ma,
    uint8_t tail_byte, uint32_t tick,
    int16_t temp_deci_c = 240,
    uint16_t err_code = 0) {
  uint8_t data[12] = {};
  data[0] = (uint8_t)((uint16_t)pos_deg_counts >> 8);
  data[1] = (uint8_t)((uint16_t)pos_deg_counts & 0xff);
  data[2] = (uint8_t)((uint16_t)vel_rpm >> 8);
  data[3] = (uint8_t)((uint16_t)vel_rpm & 0xff);
  data[4] = (uint8_t)((uint16_t)effort_ma >> 8);
  data[5] = (uint8_t)((uint16_t)effort_ma & 0xff);
  data[6] = (uint8_t)(err_code >> 8); // 错误代码（厂家「报错代码表」）
  data[7] = (uint8_t)(err_code & 0xff);
  data[8] = (uint8_t)((uint16_t)temp_deci_c >> 8); // 线圈温度 0.1 °C
  data[9] = (uint8_t)((uint16_t)temp_deci_c & 0xff);
  data[11] = tail_byte;
  return make_frame(0, 0x300U + node, 12, data, tick);
}

// 解码：位置为关节侧原样（deg→rad）；速度为电机侧 rpm→rad/s 后 /减速比；
// 扭矩 = Iq(A)×Kt×减速比×效率（取 Profile 字段，不在测试里复刻型号常数）。
// 故障位（data[11]&0x20）标注 fault_active 且不丢帧。
static void test_cerb_decode_feedback_converts_si() {
  const motor_protocol_model_profile_t *profile = nullptr;
  assert(find_profile("cerb_r58_101", &profile));

  motor_protocol_raw_frame_t frame = cerb_feedback(3, 1820, 120, 2000, 0x00, 700);
  motor_protocol_measured_state_t measured = {};
  assert(motor_protocol_cerb_adapter.decode_frame(profile, 3, &frame, &measured));

  assert(measured.has_motion_state);
  assert(!measured.fault_active);
  assert(fabs(measured.position_rad - 10.0 * M_PI / 180.0) < 1e-4); // 关节侧
  assert(fabs(measured.velocity_rad_s - (120.0 * 2.0 * M_PI / 60.0) / 101.0) < 1e-4);
  // Iq 码 2000 × 20 mA/码 = 40 A → 40 × Kt × 速比 × 效率
  assert(fabs(measured.effort_nm -
              40.0 * profile->torque_constant_nm_per_amp * profile->gear_ratio *
                  profile->gear_efficiency) < 1e-3);
  assert(fabs(measured.current_a - 40.0) < 1e-6);
  assert(fabs(measured.temperature_c - 24.0) < 1e-6); // data[8..9] 240 → 24.0 °C
  assert(nan_equal(measured.voltage_v, NAN));         // 12B 反馈帧无电压来源
  assert(measured.vendor_fault_code == 0U);

  // 故障位帧：fault_active 置位，运动量缺失（NaN），帧本身有效；温度照常可读；
  // 厂家故障字取 data[6..7] 的错误码（原来是硬编码的 1）。
  motor_protocol_raw_frame_t faulted =
      cerb_feedback(3, 1820, 120, 2000, 0x20, 701, 855, 0x1234); // 85.5 °C
  motor_protocol_measured_state_t fault_state = {};
  assert(motor_protocol_cerb_adapter.decode_frame(profile, 3, &faulted, &fault_state));
  assert(fault_state.fault_active);
  assert(fault_state.vendor_fault_code == 0x1234U);
  assert(fabs(fault_state.temperature_c - 85.5) < 1e-6);
  assert(nan_equal(fault_state.position_rad, NAN));

  // data[6..7] 非零但故障位未置 → 不透传：上游「vendor_fault_code 非零 ⟺
  // 故障在册」的不变量靠这条守住（错误码可能是锁存的历史值）。
  motor_protocol_raw_frame_t latched =
      cerb_feedback(3, 1820, 120, 2000, 0x00, 702, 240, 0x1234);
  motor_protocol_measured_state_t latched_state = {};
  assert(motor_protocol_cerb_adapter.decode_frame(profile, 3, &latched, &latched_state));
  assert(!latched_state.fault_active);
  assert(latched_state.vendor_fault_code == 0U);
  assert(latched_state.has_motion_state);
}

// 编码 MIT：关节侧 SI 直通（CERB 线协议即关节侧），使能位进控制字。
static void test_cerb_encode_mit_command() {
  const motor_protocol_model_profile_t *profile = nullptr;
  assert(find_profile("cerb_r58_101", &profile));

  motor_protocol_control_intent_t intent = {};
  intent.position_rad = 1.0f;
  intent.velocity_rad_s = -2.0f;
  intent.effort_nm = 5.0f;
  intent.kp = 30.0f;
  intent.kd = 1.0f;
  intent.enable = true;

  motor_protocol_tx_frame_t out = {};
  assert(motor_protocol_cerb_adapter.encode_mit_command(profile, 9, 2, &intent, &out));
  assert(out.bus_index == 2);
  assert(out.can_id == (0x110U | 9U));
  assert(out.is_fd);
  assert(out.size == 12);
  assert(out.tx_class == MOTOR_PROTOCOL_TX_CLASS_P1);
  assert(out.data[0] == 0xccU); // 使能位 + MIT 模式
  // 与 codec 直连打包逐字节一致（passthrough 语义）。
  motor_protocol_joint_command_t wire = {1.0, -2.0, 30.0, 1.0, 5.0};
  motor_protocol_cerb_limits_t limits = {profile->mit_position_limit_rad,
                                         profile->mit_velocity_limit_rad_s,
                                         profile->mit_torque_limit_nm,
                                         profile->mit_kp_limit,
                                         profile->mit_kd_limit};
  uint8_t expected[12];
  motor_protocol_pack_cerb_mit_frame(&wire, &limits, true, expected);
  assert(memcmp(out.data, expected, 12) == 0);
}

// 五项线协议量程来自 Profile 的 mit_*_limit，与软件安全 kp_max/kd_max 分离。
static void test_cerb_profile_separates_wire_limits_from_software_caps() {
  const motor_protocol_model_profile_t *profile = nullptr;
  assert(find_profile("cerb_r48_101", &profile));
  assert(profile->mit_kp_limit == 2500.0f);
  assert(profile->mit_kd_limit == 500.0f);
  assert(profile->kp_max == 500.0f); // 软件安全上限不因线协议量程上移
  assert(profile->kd_max == 5.0f);

  motor_protocol_control_intent_t intent = {};
  intent.kp = 30.0f;
  intent.kd = 1.0f;
  motor_protocol_tx_frame_t out = {};
  assert(motor_protocol_cerb_adapter.encode_mit_command(profile, 1, 0, &intent,
      &out));
  // 带控制字节的帧：Kp 位于 byte4 低半字节 + byte5，Kd 位于 byte6 + byte7 高半字节。
  assert(((out.data[4] & 0x0fU) << 8U | out.data[5]) == 49U);
  assert(((uint32_t)out.data[6] << 4U | (out.data[7] >> 4U)) == 8U);
  assert(find_profile("cerb_r58_101", &profile));
  assert(profile->mit_kp_limit == 2500.0f);
  assert(profile->mit_kd_limit == 500.0f);
}

// NodeID>15 没有无歧义的单轴 ID：只能产出聚合成员帧，且必须可聚合成 0x210。
static void test_cerb_high_node_uses_multi_axis_only() {
  const motor_protocol_model_profile_t *profile = nullptr;
  assert(find_profile("cerb_r48_101", &profile));
  motor_protocol_control_intent_t intent = {};
  intent.position_rad = 1.0f;
  intent.kp = 30.0f;
  intent.kd = 1.0f;
  intent.enable = true;

  motor_protocol_tx_frame_t frame = {};
  assert(motor_protocol_cerb_adapter.encode_mit_command(profile, 16, 0, &intent,
      &frame));
  assert(frame.aggregate_only);
  assert(frame.node == 16U);
  assert(frame.can_id == MOTOR_PROTOCOL_CERB_MULTI_COMMAND_ID);
  assert(frame.size == 12U);
  assert(frame.data[0] == 0xccU);

  motor_protocol_tx_frame_t aggregated = {};
  assert(motor_protocol_aggregate_p1_frames(&frame, 1U, &aggregated) == 1U);
  assert(aggregated.can_id == MOTOR_PROTOCOL_CERB_MULTI_COMMAND_ID);
  assert(aggregated.size == 64U);
  assert(aggregated.data[56] == 16U);
  assert(memcmp(aggregated.data, frame.data, 9U) == 0);
  for (size_t i = 9U; i < 56U; ++i) assert(aggregated.data[i] == 0U);
  for (size_t i = 57U; i < 64U; ++i) assert(aggregated.data[i] == 0U);

  motor_protocol_config_request_t clear = {};
  clear.kind = MOTOR_PROTOCOL_CONFIG_CLEAR_FAULT;
  motor_protocol_tx_frame_t cleared = {};
  assert(motor_protocol_cerb_adapter.config_encode_stage(
      0U, profile, 16U, 0U, &clear, &cleared));
  assert(cleared.can_id == 0x210U);
  assert(cleared.size == 64U);
  assert(!cleared.aggregate_only);
  assert(cleared.data[56] == 16U);
  assert((cleared.data[0] & 0x20U) != 0U);
  assert((cleared.data[0] & 0x80U) == 0U);

  // 高低节点可同帧聚合，节点表用完整 1..127。
  motor_protocol_tx_frame_t pair[2] = {};
  pair[0] = frame;
  assert(motor_protocol_cerb_adapter.encode_mit_command(profile, 2, 0, &intent,
      &pair[1]));
  assert(motor_protocol_aggregate_p1_frames(pair, 2U, &aggregated) == 2U);
  assert(aggregated.data[56] == 16U && aggregated.data[57] == 2U);

  // 低节点仍走单轴帧，长/ID 不变。
  motor_protocol_tx_frame_t low = {};
  assert(motor_protocol_cerb_adapter.encode_mit_command(profile, 15, 0, &intent,
      &low));
  assert(!low.aggregate_only);
  assert(low.can_id == 0x11fU);
  assert(low.size == 12U);

  // 越界节点 fail-fast。
  assert(!motor_protocol_cerb_adapter.encode_mit_command(profile, 0, 0, &intent,
      &frame));
  assert(!motor_protocol_cerb_adapter.encode_mit_command(profile, 128, 0,
      &intent, &frame));
}

// 五个 raw 全零：适配层必须返回 false，不能假装已发送。
static void test_cerb_encode_rejects_all_zero_raw_payload() {
  const motor_protocol_model_profile_t *profile = nullptr;
  assert(find_profile("cerb_r48_101", &profile));
  motor_protocol_control_intent_t intent = {};
  intent.position_rad = -profile->mit_position_limit_rad;
  intent.velocity_rad_s = -profile->mit_velocity_limit_rad_s;
  intent.effort_nm = -profile->mit_torque_limit_nm;
  intent.kp = 0.0f;
  intent.kd = 0.0f;
  motor_protocol_tx_frame_t out = {};
  assert(!motor_protocol_cerb_adapter.encode_mit_command(profile, 1, 0, &intent,
      &out));
}

// 速度单位契约：命令侧关节 rad/s 直通 3.14 上限 12 位对称量化
// （不乘 gear），反馈侧电机侧 rpm ÷gear。两侧口径若混淆——命令误乘 gear=101
// 会满码饱和失真——由「非饱和」断言当场抓获。
static void test_cerb_velocity_unit_contract() {
  const motor_protocol_model_profile_t *profile = nullptr;
  assert(find_profile("cerb_r58_101", &profile));

  motor_protocol_tx_frame_t out = {};
  motor_protocol_control_intent_t intent = {};
  intent.kp = 0.0f;
  intent.kd = 0.0f;

  // ±2.0 rad/s 落在量化区间内部：非饱和即自证命令侧未乘 gear（×101 会双双
  // 饱和到满码/零码），正负码关于中点对称（12 位对称量化）。
  intent.velocity_rad_s = 2.0f;
  assert(motor_protocol_cerb_adapter.encode_mit_command(profile, 9, 2, &intent, &out));
  const uint32_t code_pos = ((uint32_t)out.data[3] << 4) | (out.data[4] >> 4);
  intent.velocity_rad_s = -2.0f;
  assert(motor_protocol_cerb_adapter.encode_mit_command(profile, 9, 2, &intent, &out));
  const uint32_t code_neg = ((uint32_t)out.data[3] << 4) | (out.data[4] >> 4);
  assert(code_pos > 0U && code_pos < 4095U);
  assert(code_neg > 0U && code_neg < 4095U);
  assert(code_pos + code_neg == 4095U || code_pos + code_neg == 4094U);

  // 上限语义：超出 Profile 3.14 即饱和（钉死为线协议量程，而非任何 ×gear 放大限）。
  intent.velocity_rad_s = 10.0f;
  assert(motor_protocol_cerb_adapter.encode_mit_command(profile, 9, 2, &intent, &out));
  assert(((uint32_t)out.data[3] << 4) | (out.data[4] >> 4) == 0xFFFU);
  intent.velocity_rad_s = -10.0f;
  assert(motor_protocol_cerb_adapter.encode_mit_command(profile, 9, 2, &intent, &out));
  assert(((uint32_t)out.data[3] << 4) | (out.data[4] >> 4) == 0x000U);

  // 反馈分向复断言：1200 rpm（电机侧）→ ÷101 → 关节侧 rad/s。
  motor_protocol_raw_frame_t frame =
      cerb_feedback(3, 1820, 1200, 2000, 0x00, 705);
  motor_protocol_measured_state_t measured = {};
  assert(motor_protocol_cerb_adapter.decode_frame(profile, 3, &frame, &measured));
  assert(fabs(measured.velocity_rad_s - (1200.0 * 2.0 * M_PI / 60.0) / 101.0) <
      1e-4);
}

// 使能/失能帧：中性目标 + 控制字使能位（0xcc 使能 / 0x0c 失能）。
static void test_cerb_enable_disable_frames() {
  const motor_protocol_model_profile_t *profile = nullptr;
  assert(find_profile("cerb_r48_101", &profile));

  motor_protocol_tx_frame_t enable = {};
  assert(motor_protocol_cerb_adapter.encode_enable(profile, 1, 0, &enable));
  assert(enable.can_id == 0x111U);
  assert(enable.size == 12);
  assert(enable.data[0] == 0xccU);

  motor_protocol_tx_frame_t disable = {};
  assert(motor_protocol_cerb_adapter.encode_disable(profile, 1, 0, &disable));
  assert(disable.tx_class == MOTOR_PROTOCOL_TX_CLASS_P0);
  assert(disable.data[0] == 0x0cU);
}

// 软停帧：阻尼 kd=Profile 默认，保持使能，P0。
static void test_cerb_encode_soft_stop() {
  const motor_protocol_model_profile_t *profile = nullptr;
  assert(find_profile("cerb_r48_101", &profile));
  motor_protocol_tx_frame_t out = {};
  assert(motor_protocol_cerb_adapter.encode_soft_stop(profile, 2, 1, &out));
  assert(out.tx_class == MOTOR_PROTOCOL_TX_CLASS_P0);
  assert(out.data[0] == 0xccU); // 保持使能
  // kd=1.0 量化到 12bit/[0,500] → 8 = 0x008，位于 kd 字段（byte6 高 4 位 + byte7 低 4 位前段）。
  // 与 codec 直连一致性校验：
  motor_protocol_joint_command_t wire = {0.0, 0.0, 0.0, profile->default_kd, 0.0};
  motor_protocol_cerb_limits_t limits = {profile->mit_position_limit_rad,
                                         profile->mit_velocity_limit_rad_s,
                                         profile->mit_torque_limit_nm,
                                         profile->mit_kp_limit,
                                         profile->mit_kd_limit};
  uint8_t expected[12];
  motor_protocol_pack_cerb_mit_frame(&wire, &limits, true, expected);
  assert(memcmp(out.data, expected, 12) == 0);
}

// 发现帧：0x080、零长度、FD+BRS、P2。
static void test_cerb_encode_discovery_sync() {
  motor_protocol_tx_frame_t out = {};
  assert(motor_protocol_cerb_adapter.encode_discovery_sync(2, &out));
  assert(out.can_id == 0x080U);
  assert(out.size == 0);
  assert(out.is_fd);
  assert(out.is_brs);
  assert(out.tx_class == MOTOR_PROTOCOL_TX_CLASS_P2);
  assert(out.bus_index == 2);
}

// 配置事务：非 MIT 模式不受支持；管理对象走 CANopen 快速 SDO（写→读回成对）。
static void test_cerb_config_unsupported_paths() {
  const motor_protocol_model_profile_t *profile = nullptr;
  assert(find_profile("cerb_r58_101", &profile));

  motor_protocol_config_request_t mode = {};
  mode.kind = MOTOR_PROTOCOL_CONFIG_SET_MODE;
  mode.mode = MOTOR_PROTOCOL_MODE_VELOCITY;
  assert(motor_protocol_cerb_adapter.config_stage_count(profile, &mode) == 0);

  // 增益/使能是纯内核本地项，不经设备事务。
  motor_protocol_config_request_t gains = {};
  gains.kind = MOTOR_PROTOCOL_CONFIG_SET_GAINS;
  assert(motor_protocol_cerb_adapter.config_stage_count(profile, &gains) == 0);
}

// 位置限位：4 阶段（写下限/读下限/写上限/读上限）。保存（0x2539）不在这里——
// 它是独立的 SAVE_DEVICE_FLASH 事务（见 test_cerb_save_flash_transaction）。
// 字节序与《CANopen-V0.9 字典表》示例一致（0x607D:01 下限、:02 上限，单位 cnt）。
// **本条同时锁死子索引方向**：说明书"读取正限位/负限位"两行写反了（笔误），
// 别照它把两张表对调——真机实测 :01 = -32768（见 cerb_adapter.c 顶部）。
static void test_cerb_sdo_position_limit_frames() {
  const motor_protocol_model_profile_t *profile = nullptr;
  assert(find_profile("cerb_r58_101", &profile));

  motor_protocol_config_request_t req = {};
  req.kind = MOTOR_PROTOCOL_CONFIG_SET_POSITION_LIMITS;
  req.position_limits.min_position_rad = -(float)MOTOR_PROTOCOL_PI / 2.0f; // 90°
  req.position_limits.max_position_rad = (float)MOTOR_PROTOCOL_PI / 2.0f;

  assert(motor_protocol_cerb_adapter.config_stage_count(profile, &req) == 4U);

  // 90° → cnt = 90 × 65536/360 = 16384 = 0x4000。
  const uint8_t kWriteLower[8] = {0x23, 0x7D, 0x60, 0x01, 0x00, 0xC0, 0xFF, 0xFF};
  const uint8_t kReadLower[8] = {0x40, 0x7D, 0x60, 0x01, 0x00, 0x00, 0x00, 0x00};
  const uint8_t kWriteUpper[8] = {0x23, 0x7D, 0x60, 0x02, 0x00, 0x40, 0x00, 0x00};
  const uint8_t kReadUpper[8] = {0x40, 0x7D, 0x60, 0x02, 0x00, 0x00, 0x00, 0x00};
  const uint8_t *expect[4] = {kWriteLower, kReadLower, kWriteUpper, kReadUpper};

  for (uint8_t stage = 0U; stage < 4U; ++stage) {
    motor_protocol_tx_frame_t frame = {};
    assert(motor_protocol_cerb_adapter.config_encode_stage(
        stage, profile, 1U, 0U, &req, &frame));
    assert(frame.size == 8U);
    assert(frame.is_fd && frame.is_brs); // CANFD 加速标准帧（说明书 §3.1）
    assert(frame.can_id == 0x600U + 1U);
    for (size_t i = 0; i < 8U; ++i) {
      assert(frame.data[i] == expect[stage][i]);
    }
    motor_protocol_config_stage_info_t info = {};
    assert(motor_protocol_cerb_adapter.config_stage_info(stage, profile, &req, &info));
    assert(info.waits_response); // 四个 SDO 阶段都等应答（写等 ack、读等回读）
    assert(info.timeout_ms > 0U);
  }
  // 越界阶段无效（4 阶段：0..3）。
  motor_protocol_tx_frame_t frame = {};
  assert(!motor_protocol_cerb_adapter.config_encode_stage(4U, profile, 1U, 0U, &req, &frame));
}

// 身份读取事务（READ_IDENTITY）与设备侧管理对象共用同一裁决入口
// （cerb_config_on_response），此处锁死两条分支互不串台。
static void test_cerb_identity_read_transaction() {
  const motor_protocol_model_profile_t *profile = nullptr;
  assert(find_profile("cerb_r58_101", &profile));

  motor_protocol_config_request_t req = {};
  req.kind = MOTOR_PROTOCOL_CONFIG_READ_IDENTITY;
  req.motor_id = 3U;
  assert(motor_protocol_cerb_adapter.config_stage_count(profile, &req) == 5U);

  // 阶段 0：上传发起 40 08 10 00（索引 0x1008 小端）。
  motor_protocol_tx_frame_t f = {};
  assert(motor_protocol_cerb_adapter.config_encode_stage(0U, profile, 3U, 0U, &req, &f));
  assert(f.is_fd && f.is_brs && !f.is_extended);
  assert(f.can_id == 0x600U + 3U && f.size == 8U);
  assert(f.data[0] == 0x40U && f.data[1] == 0x08U && f.data[2] == 0x10U &&
      f.data[3] == 0x00U);

  uint8_t storage[32] = {0};
  motor_protocol_read_buffer_t read = {storage, sizeof(storage), 0U, false};

  // 同节点 12B 周期上报：管理对象分支的合法帧型，身份事务里必须被忽略。
  motor_protocol_raw_frame_t r = {};
  r.can_id = 0x300U + 3U;
  r.dlc = 12U;
  assert(motor_protocol_cerb_adapter.config_on_response(0U, profile, &req, &r, &read) ==
      MOTOR_PROTOCOL_STAGE_IGNORE);

  // 设备拒绝该对象 → FAIL。
  r.dlc = 8U;
  r.can_id = 0x580U + 3U;
  const uint8_t kAbort[8] = {0x80, 0x08, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00};
  memcpy(r.data, kAbort, 8U);
  assert(motor_protocol_cerb_adapter.config_on_response(0U, profile, &req, &r, &read) ==
      MOTOR_PROTOCOL_STAGE_FAIL);

  // 加速上传应答（41）：数据在 byte4..7、未用字节数 0 → COMPLETE 且载荷入缓冲。
  const uint8_t kAccel[8] = {0x43, 0x08, 0x10, 0x00, 'R', '5', '8', 'L'};
  memcpy(r.data, kAccel, 8U);
  assert(motor_protocol_cerb_adapter.config_on_response(0U, profile, &req, &r, &read) ==
      MOTOR_PROTOCOL_STAGE_COMPLETE);
  assert(read.size == 4U && !read.overflow);
  assert(memcmp(storage, "R58L", 4U) == 0);

  // 同一入口喂管理对象请求：必须仍走写→读回，不得落进身份分支。
  motor_protocol_config_request_t lim = {};
  lim.kind = MOTOR_PROTOCOL_CONFIG_SET_VELOCITY_LIMIT;
  lim.max_velocity_rad_s = (float)MOTOR_PROTOCOL_PI;
  const uint8_t kWriteAck[8] = {0x60, 0x80, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
  memcpy(r.data, kWriteAck, 8U);
  assert(motor_protocol_cerb_adapter.config_on_response(0U, profile, &lim, &r, &read) ==
      MOTOR_PROTOCOL_STAGE_ADVANCE);
}


// REFRESH_PARAMS：依次上传 0x607D:01/02、0x6080、0x2538、0x2650:00、0x2541。
// 限值换算成关节侧 SI，保护/方向原值透传，结果追加到读缓冲。
// Vaxor 不支持此事务（stage_count=0）。
static void test_cerb_refresh_params_transaction() {
  const motor_protocol_model_profile_t *profile = nullptr;
  assert(find_profile("cerb_r58_101", &profile)); // 型号参数取自固件目录，不在此复述

  motor_protocol_config_request_t req = {};
  req.kind = MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS;
  req.motor_id = 4U;
  assert(motor_protocol_cerb_adapter.config_stage_count(profile, &req) ==
      MOTOR_PROTOCOL_PARAM_SLOT_COUNT);

  const uint8_t kReadLower[8] = {0x40, 0x7D, 0x60, 0x01, 0x00, 0x00, 0x00, 0x00};
  const uint8_t kReadUpper[8] = {0x40, 0x7D, 0x60, 0x02, 0x00, 0x00, 0x00, 0x00};
  const uint8_t kReadSpeed[8] = {0x40, 0x80, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
  const uint8_t kReadCurrent[8] = {0x40, 0x38, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00};
  const uint8_t kReadSafety[8] = {0x40, 0x50, 0x26, 0x00, 0x00, 0x00, 0x00, 0x00};
  const uint8_t kReadDirection[8] = {0x40, 0x41, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00};
  const uint8_t *expect[6] = {kReadLower, kReadUpper,     kReadSpeed,
                              kReadCurrent, kReadSafety,  kReadDirection};
  for (uint8_t stage = 0U; stage < 6U; ++stage) {
    motor_protocol_tx_frame_t frame = {};
    assert(motor_protocol_cerb_adapter.config_encode_stage(
        stage, profile, 4U, 0U, &req, &frame));
    assert(frame.size == 8U && frame.is_fd && frame.is_brs);
    assert(frame.can_id == 0x600U + 4U);
    assert(memcmp(frame.data, expect[stage], 8U) == 0);
    motor_protocol_config_stage_info_t info = {};
    assert(motor_protocol_cerb_adapter.config_stage_info(stage, profile, &req, &info));
    assert(info.waits_response && info.timeout_ms > 0U);
  }
  motor_protocol_tx_frame_t frame = {};
  assert(!motor_protocol_cerb_adapter.config_encode_stage(
      8U, profile, 4U, 0U, &req, &frame));

  uint8_t storage[64] = {0};
  motor_protocol_read_buffer_t read = {storage, sizeof(storage), 0U, false};
  motor_protocol_raw_frame_t r = {};
  r.can_id = 0x580U + 4U;
  r.dlc = 8U;

  // 同节点 12B 周期上报：不是本事务应答 → IGNORE，缓冲不动。
  r.can_id = 0x300U + 4U;
  r.dlc = 12U;
  assert(motor_protocol_cerb_adapter.config_on_response(0U, profile, &req, &r, &read) ==
      MOTOR_PROTOCOL_STAGE_IGNORE);
  assert(read.size == 0U);

  // 设备中止 → FAIL（且不写缓冲）。
  r.can_id = 0x580U + 4U;
  r.dlc = 8U;
  const uint8_t kAbort[8] = {0x80, 0x7D, 0x60, 0x01, 0x00, 0x00, 0x00, 0x00};
  memcpy(r.data, kAbort, 8U);
  assert(motor_protocol_cerb_adapter.config_on_response(0U, profile, &req, &r, &read) ==
      MOTOR_PROTOCOL_STAGE_FAIL);
  assert(read.size == 0U);

  // 索引不符（阶段 0 等 :01，回了 :02）→ RETRY，不写缓冲。
  const uint8_t kWrongSub[8] = {0x43, 0x7D, 0x60, 0x02, 0x00, 0x40, 0x00, 0x00};
  memcpy(r.data, kWrongSub, 8U);
  assert(motor_protocol_cerb_adapter.config_on_response(0U, profile, &req, &r, &read) ==
      MOTOR_PROTOCOL_STAGE_RETRY);
  assert(read.size == 0U);

  // 命令字非上传应答（0x60 写应答）→ RETRY。
  const uint8_t kWriteAck[8] = {0x60, 0x7D, 0x60, 0x01, 0x00, 0x00, 0x00, 0x00};
  memcpy(r.data, kWriteAck, 8U);
  assert(motor_protocol_cerb_adapter.config_on_response(0U, profile, &req, &r, &read) ==
      MOTOR_PROTOCOL_STAGE_RETRY);
  assert(read.size == 0U);

  // 六个正常应答：16384 cnt(+90°)、-16384 cnt(-90°)、600 RPM、1000 mA、
  // 保护开关 0（保护开）、方向 0（CW）。
  struct {
    uint8_t payload[8];
  } ok_frames[6] = {
    {{0x43, 0x7D, 0x60, 0x01, 0x00, 0x40, 0x00, 0x00}},
    {{0x43, 0x7D, 0x60, 0x02, 0x00, 0xC0, 0xFF, 0xFF}},
    {{0x43, 0x80, 0x60, 0x00, 0x58, 0x02, 0x00, 0x00}},   // 600 LE
    {{0x43, 0x38, 0x25, 0x00, 0xE8, 0x03, 0x00, 0x00}},   // 1000 LE
    {{0x43, 0x50, 0x26, 0x00, 0x00, 0x00, 0x00, 0x00}},   // 0x2650:00 = 0
    {{0x43, 0x41, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00}},   // 0x2541 = 0
  };
  for (uint8_t stage = 0U; stage < 6U; ++stage) {
    memcpy(r.data, ok_frames[stage].payload, 8U);
    const auto outcome =
        motor_protocol_cerb_adapter.config_on_response(stage, profile, &req, &r, &read);
    assert(outcome == MOTOR_PROTOCOL_STAGE_ADVANCE);
  }
  // 每阶段一个 motor_protocol_param_stage_t：设备端原值 + 关节侧换算值。
  assert(read.size == 6U * sizeof(motor_protocol_param_stage_t) && !read.overflow);
  const motor_protocol_param_stage_t *values =
      reinterpret_cast<const motor_protocol_param_stage_t *>(storage);
  assert(values[0].raw == 16384U);   // +90° 的 cnt（原值原样带出）
  assert(values[1].raw == 0xFFFFC000U); // -16384 的补码
  assert(values[2].raw == 600U);     // 电机端 RPM
  assert(values[3].raw == 1000U);    // 电机端 mA
  assert(fabs(values[0].value_si - MOTOR_PROTOCOL_PI / 2.0) < 1e-5);
  assert(fabs(values[1].value_si + MOTOR_PROTOCOL_PI / 2.0) < 1e-5);
  assert(fabs(values[2].value_si - 2.0 * MOTOR_PROTOCOL_PI * 10.0 / 101.0) < 1e-5);
  // 限值对象用规格书峰值对（45 N·m ÷ 10.8 A），不是反馈的 Kt×gear 链。
  assert(fabs(values[3].value_si - 45.0 / 10.8) < 1e-5);
  // 安全开关/方向：原值透传（无单位换算），raw=0 → SI=0。
  assert(values[4].raw == 0U && values[4].value_si == 0.0f);
  assert(values[5].raw == 0U && values[5].value_si == 0.0f);

  // 阶段 → 设备端参数槽位（内核按槽位落袋，不硬编码阶段号）。
  {
    const uint8_t expect_slots[MOTOR_PROTOCOL_PARAM_SLOT_COUNT] = {
      MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN, MOTOR_PROTOCOL_PARAM_SLOT_POS_MAX,
      MOTOR_PROTOCOL_PARAM_SLOT_VEL_MAX, MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX,
      MOTOR_PROTOCOL_PARAM_SLOT_SAFETY, MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION,
      MOTOR_PROTOCOL_PARAM_SLOT_T_MAX, MOTOR_PROTOCOL_PARAM_SLOT_NODE_ID};
    for (uint8_t stage = 0U; stage < MOTOR_PROTOCOL_PARAM_SLOT_COUNT; ++stage) {
      motor_protocol_config_stage_info_t info = {};
      assert(motor_protocol_cerb_adapter.config_stage_info(stage, profile, &req, &info));
      assert(info.param_raw_valid);
      assert(info.param_slot == expect_slots[stage]);
      assert(info.param_raw == 0U); // 读阶段：原值由应答带回
    }
  }

  // Vaxor：无设备参数回读能力（0 阶段 → UNSUPPORTED）。
  const motor_protocol_model_profile_t *vp = nullptr;
  assert(find_profile("vaxor_2550", &vp));
  assert(motor_protocol_vaxor_adapter.config_stage_count(vp, &req) == 0U);
}

// 速度限值按减速比换算为 RPM；力矩限值优先用规格书峰值对换算为 mA。
static void test_cerb_sdo_velocity_torque_frames() {
  const motor_protocol_model_profile_t *profile = nullptr;
  assert(find_profile("cerb_r58_101", &profile)); // 型号参数取自固件目录，不在此复述

  // 关节侧 π rad/s → 电机端 3030 RPM。
  motor_protocol_config_request_t vel = {};
  vel.kind = MOTOR_PROTOCOL_CONFIG_SET_VELOCITY_LIMIT;
  vel.max_velocity_rad_s = (float)MOTOR_PROTOCOL_PI;
  assert(motor_protocol_cerb_adapter.config_stage_count(profile, &vel) == 2U);
  motor_protocol_tx_frame_t f = {};
  assert(motor_protocol_cerb_adapter.config_encode_stage(0U, profile, 1U, 0U, &vel, &f));
  assert(f.data[0] == 0x23U && f.data[1] == 0x80U && f.data[2] == 0x60U && f.data[3] == 0x00U);
  const uint32_t rpm = (uint32_t)f.data[4] | ((uint32_t)f.data[5] << 8U) |
      ((uint32_t)f.data[6] << 16U) | ((uint32_t)f.data[7] << 24U);
  assert(rpm == 3030U);
  assert(motor_protocol_cerb_adapter.config_encode_stage(1U, profile, 1U, 0U, &vel, &f));
  assert(f.data[0] == 0x40U); // 读回请求

  // 关节侧 30 N·m → 电流 = 30/(45/10.8) = 7.2 A → 7200 mA（限值系数 = 峰值对）。
  motor_protocol_config_request_t trq = {};
  trq.kind = MOTOR_PROTOCOL_CONFIG_SET_TORQUE_LIMIT;
  trq.max_torque_nm = 30.0f;
  assert(motor_protocol_cerb_adapter.config_stage_count(profile, &trq) == 2U);
  assert(motor_protocol_cerb_adapter.config_encode_stage(0U, profile, 1U, 0U, &trq, &f));
  assert(f.data[0] == 0x23U && f.data[1] == 0x38U && f.data[2] == 0x25U && f.data[3] == 0x00U);
  const uint32_t ma = (uint32_t)f.data[4] | ((uint32_t)f.data[5] << 8U) |
      ((uint32_t)f.data[6] << 16U) | ((uint32_t)f.data[7] << 24U);
  assert(ma == 7200U);

  // 写阶段必须把写入的设备端原值带出（内核据此写固件记录）：与帧内字节同值。
  motor_protocol_config_stage_info_t write_info = {};
  assert(motor_protocol_cerb_adapter.config_stage_info(0U, profile, &trq, &write_info));
  assert(write_info.param_raw_valid);
  assert(write_info.param_slot == MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX);
  assert(write_info.param_raw == ma);
  // 读回核验阶段与写阶段同槽位（内核写记录时只看写阶段）。
  assert(motor_protocol_cerb_adapter.config_stage_info(1U, profile, &trq, &write_info));
  assert(!write_info.param_raw_valid);
  assert(write_info.param_slot == MOTOR_PROTOCOL_PARAM_SLOT_NONE);

  motor_protocol_config_stage_info_t vel_info = {};
  assert(motor_protocol_cerb_adapter.config_stage_info(0U, profile, &vel, &vel_info));
  assert(vel_info.param_raw_valid);
  assert(vel_info.param_slot == MOTOR_PROTOCOL_PARAM_SLOT_VEL_MAX);
  assert(vel_info.param_raw == 3030U);

  motor_protocol_config_request_t pos = {};
  pos.kind = MOTOR_PROTOCOL_CONFIG_SET_POSITION_LIMITS;
  pos.position_limits.min_position_rad = -(float)MOTOR_PROTOCOL_PI / 2.0f;
  pos.position_limits.max_position_rad = (float)MOTOR_PROTOCOL_PI / 2.0f;
  motor_protocol_config_stage_info_t pos_info = {};
  assert(motor_protocol_cerb_adapter.config_stage_info(0U, profile, &pos, &pos_info));
  assert(pos_info.param_slot == MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN);
  assert(motor_protocol_cerb_adapter.config_stage_info(2U, profile, &pos, &pos_info));
  assert(pos_info.param_slot == MOTOR_PROTOCOL_PARAM_SLOT_POS_MAX);

  // 身份读取不携带设备参数（不得混入记录）。
  motor_protocol_config_request_t ident = {};
  ident.kind = MOTOR_PROTOCOL_CONFIG_READ_IDENTITY;
  motor_protocol_config_stage_info_t ident_info = {};
  assert(motor_protocol_cerb_adapter.config_stage_info(0U, profile, &ident, &ident_info));
  assert(!ident_info.param_raw_valid);

  // Vaxor 的 SET_MODE 阶段同理不带设备参数。
  const motor_protocol_model_profile_t *vp = nullptr;
  assert(find_profile("vaxor_2550", &vp));
  motor_protocol_config_request_t mode = {};
  mode.kind = MOTOR_PROTOCOL_CONFIG_SET_MODE;
  motor_protocol_config_stage_info_t mode_info = {};
  assert(motor_protocol_vaxor_adapter.config_stage_info(0U, vp, &mode, &mode_info));
  assert(!mode_info.param_raw_valid);
  assert(mode_info.param_slot == MOTOR_PROTOCOL_PARAM_SLOT_NONE);
}


// 描述符表边界：所有 kind 的越界阶段统一拒绝；保存阶段不裁决任何应答。
static void test_cerb_descriptor_table_bounds() {
  const motor_protocol_model_profile_t *profile = nullptr;
  assert(find_profile("cerb_r58_101", &profile));

  motor_protocol_config_request_t kinds[] = {
    {}, {}, {}, {}, {},
  };
  kinds[0].kind = MOTOR_PROTOCOL_CONFIG_SET_ZERO;
  kinds[1].kind = MOTOR_PROTOCOL_CONFIG_SET_POSITION_LIMITS;
  kinds[2].kind = MOTOR_PROTOCOL_CONFIG_SET_VELOCITY_LIMIT;
  kinds[3].kind = MOTOR_PROTOCOL_CONFIG_SET_TORQUE_LIMIT;
  kinds[4].kind = MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS;
  const uint8_t counts[] = {1U, 4U, 2U, 2U, 8U};

  motor_protocol_tx_frame_t frame = {};
  motor_protocol_config_stage_info_t info = {};
  motor_protocol_raw_frame_t resp = {};
  resp.can_id = 0x581U;
  resp.dlc = 8U;
  for (size_t i = 0; i < sizeof(kinds) / sizeof(kinds[0]); ++i) {
    const uint8_t beyond = counts[i];
    assert(!motor_protocol_cerb_adapter.config_encode_stage(
        beyond, profile, 1U, 0U, &kinds[i], &frame));
    assert(!motor_protocol_cerb_adapter.config_stage_info(
        beyond, profile, &kinds[i], &info));
    assert(motor_protocol_cerb_adapter.config_on_response(
        beyond, profile, &kinds[i], &resp, nullptr) ==
        MOTOR_PROTOCOL_STAGE_FAIL);
  }

  // SET_MODE 空阶段的杂散帧：非 8B RETRY、8B FAIL（旧边界口径）。
  motor_protocol_config_request_t mode = {};
  mode.kind = MOTOR_PROTOCOL_CONFIG_SET_MODE;
  mode.mode = (uint8_t)MOTOR_PROTOCOL_MODE_MIT;
  motor_protocol_raw_frame_t mode_resp = {};
  mode_resp.dlc = 7U;
  assert(motor_protocol_cerb_adapter.config_on_response(
      0U, profile, &mode, &mode_resp, nullptr) ==
      MOTOR_PROTOCOL_STAGE_RETRY);
  mode_resp.dlc = 8U;
  mode_resp.data[0] = 0x80U;
  assert(motor_protocol_cerb_adapter.config_on_response(
      0U, profile, &mode, &mode_resp, nullptr) ==
      MOTOR_PROTOCOL_STAGE_FAIL);

  // 保存阶段（SAVE_DEVICE_FLASH）的杂散应答：IGNORE，不得触发 FLASH 重写/推进。
  motor_protocol_config_request_t save = {};
  save.kind = MOTOR_PROTOCOL_CONFIG_SAVE_DEVICE_FLASH;
  save.motor_id = 1U;
  save.save_slots = (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX);
  const uint8_t save_ack[8] = {0x60, 0x39, 0x25, 0x00, 0x01, 0x00, 0x00, 0x00};
  memcpy(resp.data, save_ack, 8U);
  assert(motor_protocol_cerb_adapter.config_on_response(
      0U, profile, &save, &resp, nullptr) ==
      MOTOR_PROTOCOL_STAGE_IGNORE);
}

// 设零点：单阶段写 0x2531=1（字典表「设置电机输出端零位」），等 SDO 写应答即
// 完成——命令式对象，不读回（读回值语义未定义）。失能态门禁在内核侧。
static void test_cerb_set_zero_frames() {
  const motor_protocol_model_profile_t *profile = nullptr;
  assert(find_profile("cerb_r58_101", &profile));
  motor_protocol_config_request_t req = {};
  req.kind = MOTOR_PROTOCOL_CONFIG_SET_ZERO;
  req.motor_id = 3U; // 与应答帧 0x580+3 同节点（内核按 can_id 预过滤归属）
  assert(motor_protocol_cerb_adapter.config_stage_count(profile, &req) == 1U);

  motor_protocol_tx_frame_t frame = {};
  assert(motor_protocol_cerb_adapter.config_encode_stage(0U, profile, 3U, 0U, &req,
      &frame));
  const uint8_t kExpect[8] = {0x23, 0x31, 0x25, 0x00, 0x01, 0x00, 0x00, 0x00};
  assert(frame.can_id == 0x600U + 3U && frame.size == 8U && frame.is_fd);
  assert(memcmp(frame.data, kExpect, 8U) == 0);

  motor_protocol_config_stage_info_t info = {};
  assert(motor_protocol_cerb_adapter.config_stage_info(0U, profile, &req, &info));
  assert(info.waits_response && info.timeout_ms > 0U);
  assert(!info.param_raw_valid); // 不是设备参数：不进记录/比对
  assert(!motor_protocol_cerb_adapter.config_encode_stage(1U, profile, 3U, 0U, &req,
      &frame));

  motor_protocol_raw_frame_t r = {};
  r.can_id = 0x580U + 3U;
  r.dlc = 8U;
  const uint8_t kAck[8] = {0x60, 0x31, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00};
  memcpy(r.data, kAck, 8U);
  assert(motor_protocol_cerb_adapter.config_on_response(0U, profile, &req, &r, nullptr) ==
      MOTOR_PROTOCOL_STAGE_ADVANCE);
  const uint8_t kAbort[8] = {0x80, 0x31, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00};
  memcpy(r.data, kAbort, 8U);
  assert(motor_protocol_cerb_adapter.config_on_response(0U, profile, &req, &r, nullptr) ==
      MOTOR_PROTOCOL_STAGE_FAIL);
  const uint8_t kWrong[8] = {0x60, 0x38, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00};
  memcpy(r.data, kWrong, 8U);
  assert(motor_protocol_cerb_adapter.config_on_response(0U, profile, &req, &r, nullptr) ==
      MOTOR_PROTOCOL_STAGE_RETRY);
}

// 响应裁决：写等 ack、读回必须一致；设备 abort → FAIL；索引不符 → RETRY。
static void test_cerb_sdo_response_judging() {
  const motor_protocol_model_profile_t *profile = nullptr;
  assert(find_profile("cerb_r58_101", &profile));
  motor_protocol_config_request_t req = {};
  req.kind = MOTOR_PROTOCOL_CONFIG_SET_POSITION_LIMITS;
  req.position_limits.min_position_rad = -(float)MOTOR_PROTOCOL_PI / 2.0f;
  req.position_limits.max_position_rad = (float)MOTOR_PROTOCOL_PI / 2.0f;

  motor_protocol_raw_frame_t r = {};
  r.dlc = 8U;
  r.is_fd = false;

  // 阶段 0（写下限）：ack = 0x60 → ADVANCE。
  const uint8_t kWriteAck[8] = {0x60, 0x7D, 0x60, 0x01, 0x00, 0x00, 0x00, 0x00};
  memcpy(r.data, kWriteAck, 8U);
  assert(motor_protocol_cerb_adapter.config_on_response(0U, profile, &req, &r, nullptr) ==
      MOTOR_PROTOCOL_STAGE_ADVANCE);

  // 阶段 1（读下限）：回读 = 0xC000（-16384）→ 与写入一致 → ADVANCE。
  const uint8_t kReadOk[8] = {0x43, 0x7D, 0x60, 0x01, 0x00, 0xC0, 0xFF, 0xFF};
  memcpy(r.data, kReadOk, 8U);
  assert(motor_protocol_cerb_adapter.config_on_response(1U, profile, &req, &r, nullptr) ==
      MOTOR_PROTOCOL_STAGE_ADVANCE);

  // 回读不一致 → RETRY（内核重试 ≤2 次后收敛 UNCONFIRMED，不报 OK）。
  const uint8_t kReadMismatch[8] = {0x43, 0x7D, 0x60, 0x01, 0x00, 0x40, 0x00, 0x00};
  memcpy(r.data, kReadMismatch, 8U);
  assert(motor_protocol_cerb_adapter.config_on_response(1U, profile, &req, &r, nullptr) ==
      MOTOR_PROTOCOL_STAGE_RETRY);

  // 设备中止（0x80）→ FAIL。
  const uint8_t kAbort[8] = {0x80, 0x7D, 0x60, 0x01, 0x00, 0x00, 0x00, 0x00};
  memcpy(r.data, kAbort, 8U);
  assert(motor_protocol_cerb_adapter.config_on_response(0U, profile, &req, &r, nullptr) ==
      MOTOR_PROTOCOL_STAGE_FAIL);

  // 索引不符（别的对象的迟到应答）→ RETRY。
  const uint8_t kWrongIndex[8] = {0x60, 0x80, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
  memcpy(r.data, kWrongIndex, 8U);
  assert(motor_protocol_cerb_adapter.config_on_response(0U, profile, &req, &r, nullptr) ==
      MOTOR_PROTOCOL_STAGE_RETRY);
}

// 节点范围与 ID 段不重叠（同一 CAN ID 不得同时归属两个协议）。
static void test_adapter_id_ranges_disjoint() {
  // vaxor reply 最低 0x701+4 = 0x705；CERB feedback 最高 0x300+127 = 0x37f。
  assert(0x300U + 127U < 0x701U + 4U);
  // vaxor command 最低 0x501+4 = 0x505 > 0x37f。
  assert(0x37fU < 0x505U);
}

// ---------------------------------------------------------------------------
// 型号目录（数值来源 = ROS 型号库，TBD-PROFILE-01 锚点）
// ---------------------------------------------------------------------------

static void test_profile_catalog_only_exposes_supported_models() {
  // 生产支持集只有三种；误加回退役型号会重新开放 MotorBind，必须由测试拦住。
  assert(motor_control_profile_count() == 3U);

  const motor_protocol_model_profile_t *p = nullptr;
  assert(find_profile("vaxor_2550", &p));
  assert(p->protocol == &motor_protocol_vaxor_adapter);
  assert(p->gear_ratio == 50.0f);
  assert(p->gear_efficiency == 0.65f);
  assert(fabs(p->torque_constant_nm_per_amp - 0.042) < 1e-6);
  assert((p->capability_flags & MOTOR_PROTOCOL_CAP_LIMIT_SOFTWARE) != 0U);
  assert((p->capability_flags & MOTOR_PROTOCOL_CAP_AUTONOMOUS_FEEDBACK) != 0U);

  assert(find_profile("cerb_r58_101", &p));
  assert(p->protocol == &motor_protocol_cerb_adapter);
  assert(p->gear_ratio == 101.0f);
  assert(fabs(p->torque_constant_nm_per_amp - 0.067) < 1e-6);
  assert(fabs(p->gear_efficiency - 0.75) < 1e-6);
  assert(p->rated_torque_nm == 20.0f);
  assert(p->peak_torque_nm == 45.0f);
  assert(p->mit_torque_limit_nm == 300.0f);
  assert((p->capability_flags & MOTOR_PROTOCOL_CAP_LIMIT_DEVICE_SIDE) != 0U);

  assert(find_profile("cerb_r48_101", &p));
  assert(fabs(p->torque_constant_nm_per_amp - 0.1) < 1e-6);
  assert(fabs(p->gear_efficiency - 0.5) < 1e-6);
  assert(p->rated_torque_nm == 5.0f);
  assert(p->peak_torque_nm == 11.0f);
  assert(p->mit_torque_limit_nm == 300.0f);

  // Vaxor 80 速比刻意缺席：按名字找不到（禁止项 15）。
  assert(!find_profile("vaxor_2580", &p));
  for (size_t i = 0; i < motor_control_profile_count(); ++i) {
    const motor_protocol_model_profile_t *row = motor_control_profile_at(i);
    assert(row->gear_ratio != 80.0f);
  }
}

// REFRESH 的保护/方向阶段只上传原值；value_si == (float)raw，不做单位换算。
static void test_cerb_refresh_covers_safety_slots() {
  const motor_protocol_model_profile_t *profile = nullptr;
  assert(find_profile("cerb_r58_101", &profile));

  motor_protocol_config_request_t req = {};
  req.kind = MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS;
  req.motor_id = 1U;
  assert(motor_protocol_cerb_adapter.config_stage_count(profile, &req) == 8U);

  const uint8_t kReadSafety[8] = {0x40, 0x50, 0x26, 0x00, 0x00, 0x00, 0x00, 0x00};
  const uint8_t kReadDirection[8] = {0x40, 0x41, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00};
  const uint8_t *expect[2] = {kReadSafety, kReadDirection};
  const uint8_t slots[2] = {MOTOR_PROTOCOL_PARAM_SLOT_SAFETY,
                            MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION};
  for (uint8_t i = 0U; i < 2U; ++i) {
    const uint8_t stage = (uint8_t)(4U + i);
    motor_protocol_config_stage_info_t info = {};
    assert(motor_protocol_cerb_adapter.config_stage_info(stage, profile, &req, &info));
    assert(info.param_slot == slots[i]);
    assert(info.waits_response); // 读回等应答
    motor_protocol_tx_frame_t frame = {};
    assert(motor_protocol_cerb_adapter.config_encode_stage(stage, profile, 1U, 0U,
        &req, &frame));
    assert(frame.data[0] == 0x40U); // 本阶段为上传请求
    assert(memcmp(frame.data, expect[i], 8U) == 0);
  }

  // 原值透传：应答 raw=1 → value_si=1.0（不做任何换算）。两槽都喂 1——若用
  // {1,0}，方向槽会落在 0，`*out = 0.0f` 这种写死实现也能蒙混通过。
  // 走真实的 on_response 路径（不是内部换算助手）。
  uint8_t storage[16] = {0};
  motor_protocol_raw_frame_t r = {};
  r.can_id = 0x580U + 1U;
  r.dlc = 8U;
  const uint32_t kRaws[2] = {1U, 1U};
  for (uint8_t i = 0U; i < 2U; ++i) {
    const uint8_t stage = (uint8_t)(4U + i);
    motor_protocol_read_buffer_t read = {storage, sizeof(storage), 0U, false};
    const uint32_t raw = kRaws[i];
    const uint8_t payload[8] = {0x43U, expect[i][1], expect[i][2], expect[i][3],
                                (uint8_t)(raw & 0xFFU),
                                (uint8_t)((raw >> 8U) & 0xFFU),
                                (uint8_t)((raw >> 16U) & 0xFFU),
                                (uint8_t)((raw >> 24U) & 0xFFU)};
    memcpy(r.data, payload, 8U);
    assert(motor_protocol_cerb_adapter.config_on_response(stage, profile, &req, &r,
        &read) ==
        MOTOR_PROTOCOL_STAGE_ADVANCE);
    assert(read.size == sizeof(motor_protocol_param_stage_t));
    const motor_protocol_param_stage_t *v =
        reinterpret_cast<const motor_protocol_param_stage_t *>(storage);
    assert(v->raw == raw);
    assert(v->value_si == (float)raw);
  }
}

// 保护开关易失：仅写与读回核验，无保存阶段。
// 关闭保护的写帧须为 23 50 26 00 01 00 00 00。
static void test_cerb_safety_protect_write_frames() {
  const motor_protocol_model_profile_t *profile = nullptr;
  assert(find_profile("cerb_r58_101", &profile));

  motor_protocol_config_request_t req = {};
  req.kind = MOTOR_PROTOCOL_CONFIG_SET_SAFETY_PROTECT;
  req.motor_id = 3U;
  req.safety_protect_raw = 1U;
  assert(motor_protocol_cerb_adapter.config_stage_count(profile, &req) == 2U);

  motor_protocol_tx_frame_t frame = {};
  assert(motor_protocol_cerb_adapter.config_encode_stage(0U, profile, 3U, 0U,
      &req, &frame));
  const uint8_t kWriteCmd[8] = {0x23, 0x50, 0x26, 0x00, 0x01, 0x00, 0x00, 0x00};
  assert(memcmp(frame.data, kWriteCmd, 8U) == 0);

  // 重开：数据字节回到 0，其余不变（手册只给了关闭命令，重开是同一条写）。
  req.safety_protect_raw = 0U;
  assert(motor_protocol_cerb_adapter.config_encode_stage(0U, profile, 3U, 0U,
      &req, &frame));
  assert(frame.data[0] == 0x23U && frame.data[4] == 0x00U);

  req.safety_protect_raw = 1U;
  assert(motor_protocol_cerb_adapter.config_encode_stage(1U, profile, 3U, 0U,
      &req, &frame));
  assert(frame.data[0] == 0x40U && frame.data[1] == 0x50U &&
      frame.data[2] == 0x26U && frame.data[3] == 0x00U);

  // 写阶段登记槽 4（内核据此把记录一起挪到新值）。
  motor_protocol_config_stage_info_t info = {};
  assert(motor_protocol_cerb_adapter.config_stage_info(0U, profile, &req, &info));
  assert(info.param_slot == MOTOR_PROTOCOL_PARAM_SLOT_SAFETY &&
      info.param_raw_valid && info.param_raw == 1U);

  motor_protocol_raw_frame_t r = {};
  r.can_id = 0x580U + 3U;
  r.dlc = 8U;
  uint8_t scratch[16] = {0};
  motor_protocol_read_buffer_t read = {scratch, sizeof(scratch), 0U, false};

  memcpy(r.data, kWriteCmd, 8U);
  r.data[0] = 0x60U; // 写 ack
  assert(motor_protocol_cerb_adapter.config_on_response(0U, profile, &req, &r,
      &read) ==
      MOTOR_PROTOCOL_STAGE_ADVANCE);

  const uint8_t kReadBackOk[8] = {0x43, 0x50, 0x26, 0x00, 0x01, 0, 0, 0};
  memcpy(r.data, kReadBackOk, 8U);
  assert(motor_protocol_cerb_adapter.config_on_response(1U, profile, &req, &r,
      &read) ==
      MOTOR_PROTOCOL_STAGE_ADVANCE);

  // 读回不一致（设备没接受）→ 重发本阶段，不得判成功。
  const uint8_t kReadBackBad[8] = {0x43, 0x50, 0x26, 0x00, 0x00, 0, 0, 0};
  memcpy(r.data, kReadBackBad, 8U);
  assert(motor_protocol_cerb_adapter.config_on_response(1U, profile, &req, &r,
      &read) ==
      MOTOR_PROTOCOL_STAGE_RETRY);
}

// 方向写事务（0x2541:00）：写 + 读回核验，**没有保存阶段**（易失，同 0x2650）。
// 写命令 `23 41 25 00 0X 00 00 00` 由 维护者 手工核实（手册未给这个对象的写命令），
// 逐字节锁死；阶段数恰好 2——哪天有人"顺手补一条保存"会在这里被拦住。
static void test_cerb_direction_write_frames() {
  const motor_protocol_model_profile_t *profile = nullptr;
  assert(find_profile("cerb_r58_101", &profile));

  motor_protocol_config_request_t req = {};
  req.kind = MOTOR_PROTOCOL_CONFIG_SET_DIRECTION;
  req.motor_id = 3U;
  req.direction_raw = 1U; // CCW
  assert(motor_protocol_cerb_adapter.config_stage_count(profile, &req) == 2U);

  motor_protocol_tx_frame_t frame = {};
  assert(motor_protocol_cerb_adapter.config_encode_stage(0U, profile, 3U, 0U,
      &req, &frame));
  const uint8_t kCcwCmd[8] = {0x23, 0x41, 0x25, 0x00, 0x01, 0x00, 0x00, 0x00};
  assert(memcmp(frame.data, kCcwCmd, 8U) == 0);

  req.direction_raw = 0U; // CW
  assert(motor_protocol_cerb_adapter.config_encode_stage(0U, profile, 3U, 0U,
      &req, &frame));
  const uint8_t kCwCmd[8] = {0x23, 0x41, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00};
  assert(memcmp(frame.data, kCwCmd, 8U) == 0);

  // 阶段 1 是读回核验（0x40 = 加速上传请求），不是写。
  assert(motor_protocol_cerb_adapter.config_encode_stage(1U, profile, 3U, 0U,
      &req, &frame));
  assert(frame.data[0] == 0x40U && frame.data[1] == 0x41U &&
      frame.data[2] == 0x25U && frame.data[3] == 0x00U);

  // 写阶段登记槽 5（内核据此把记录一起挪到新值）。
  motor_protocol_config_stage_info_t info = {};
  assert(motor_protocol_cerb_adapter.config_stage_info(0U, profile, &req, &info));
  assert(info.param_slot == MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION &&
      info.param_raw_valid && info.param_raw == 0U);

  // 读回核验语义（CCW 写 1，设备回 1）：期望值由本阶段的 value_kind 反算，
  // 故读阶段必须与写阶段同一取值来源——用 PASSTHROUGH 会让期望恒为 0，
  // CCW 永远收敛不到 ADVANCE。
  req.direction_raw = 1U;
  motor_protocol_raw_frame_t r = {};
  r.can_id = 0x580U + 3U;
  r.dlc = 8U;
  uint8_t scratch[16] = {0};
  motor_protocol_read_buffer_t read = {scratch, sizeof(scratch), 0U, false};
  const uint8_t kAck[8] = {0x60, 0x41, 0x25, 0x00, 0, 0, 0, 0};
  memcpy(r.data, kAck, 8U);
  assert(motor_protocol_cerb_adapter.config_on_response(0U, profile, &req, &r,
      &read) ==
      MOTOR_PROTOCOL_STAGE_ADVANCE);
  const uint8_t kReadCcw[8] = {0x43, 0x41, 0x25, 0x00, 0x01, 0, 0, 0};
  memcpy(r.data, kReadCcw, 8U);
  assert(motor_protocol_cerb_adapter.config_on_response(1U, profile, &req, &r,
      &read) ==
      MOTOR_PROTOCOL_STAGE_ADVANCE);

  // 读回不一致（设备没接受）→ 重发本阶段，不得判成功。
  const uint8_t kReadCw[8] = {0x43, 0x41, 0x25, 0x00, 0x00, 0, 0, 0};
  memcpy(r.data, kReadCw, 8U);
  assert(motor_protocol_cerb_adapter.config_on_response(1U, profile, &req, &r,
      &read) ==
      MOTOR_PROTOCOL_STAGE_RETRY);
}

// 保存事务（SAVE_DEVICE_FLASH）：0x2539 保存码按"哪些槽位脏"选表，全部无应答、
// 发完即推进。**写事务里不再跟保存阶段**——"写"与"保存"是两次独立动作
// （见 docs/plans/active/param-authority-reversal-design.md §3.2）。
static void test_cerb_save_flash_transaction() {
  const motor_protocol_model_profile_t *profile = nullptr;
  assert(find_profile("cerb_r58_101", &profile));

  // 保存码位图 = 1u << MOTOR_PROTOCOL_PARAM_SLOT_*
  // （POS_MIN=0, POS_MAX=1, VEL_MAX=2, TORQUE_MAX=3, SAFETY=4, DIRECTION=5）
  // 地址即语义，这里直接写位图比引枚举更贴测试意图。
  const uint8_t kPosBoth = 0x03U;       // 位置下限 | 位置上限
  const uint8_t kTorque = 0x08U;        // 力矩限值
  const uint8_t kParamsAndPos = 0x07U;  // 位置上下限 | 速度限值
  const uint8_t kVolatileOnly = 0x30U;  // 保护开关 | 方向

  // 0x2539 帧逐字节：23 39 25 00 <码> 00 00 00
  // （快速 SDO 写 4 字节到 0x2539:00；码 1=PI 参数+最大电流 / 2=位置上限 / 4=位置下限）
  auto expect_save = [&](const motor_protocol_config_request_t *req, uint8_t stage,
          uint8_t code) {
        motor_protocol_tx_frame_t frame = {};
        assert(motor_protocol_cerb_adapter.config_encode_stage(stage, profile, 1U, 0U,
            req, &frame));
        assert(frame.size == 8U && frame.is_fd && frame.is_brs);
        assert(frame.can_id == 0x600U + 1U);
        assert(frame.data[0] == 0x23U && frame.data[1] == 0x39U &&
            frame.data[2] == 0x25U && frame.data[3] == 0x00U);
        assert(frame.data[4] == code);
        motor_protocol_config_stage_info_t info = {};
        assert(motor_protocol_cerb_adapter.config_stage_info(stage, profile, req, &info));
        assert(!info.waits_response); // 无应答：驱动器写 FLASH 期间不拖总线
        assert(!info.param_raw_valid); // 保存码不是设备参数，不进记录/比对
        assert(info.timeout_ms == 0U);
      };

  motor_protocol_config_request_t pos = {};
  pos.kind = MOTOR_PROTOCOL_CONFIG_SAVE_DEVICE_FLASH;
  pos.save_slots = kPosBoth;
  assert(motor_protocol_cerb_adapter.config_stage_count(profile, &pos) == 2U);
  expect_save(&pos, 0U, 4U); // 下限
  expect_save(&pos, 1U, 2U); // 上限

  motor_protocol_config_request_t trq = {};
  trq.kind = MOTOR_PROTOCOL_CONFIG_SAVE_DEVICE_FLASH;
  trq.save_slots = kTorque;
  assert(motor_protocol_cerb_adapter.config_stage_count(profile, &trq) == 1U);
  expect_save(&trq, 0U, 1U);

  // 两组都脏 → 三条：先参数组，再位置两码（顺序只是确定性，不是协议要求）。
  motor_protocol_config_request_t both = {};
  both.kind = MOTOR_PROTOCOL_CONFIG_SAVE_DEVICE_FLASH;
  both.save_slots = kParamsAndPos;
  assert(motor_protocol_cerb_adapter.config_stage_count(profile, &both) == 3U);
  expect_save(&both, 0U, 1U);
  expect_save(&both, 1U, 4U);
  expect_save(&both, 2U, 2U);

  // 只脏易失槽（保护开关 / 方向）：厂家没给这两个对象保存码 → **0 阶段**。
  // 内核据此只清脏位 + 落 W25，不发任何 0x2539；两者靠上电归一从记录重建。
  motor_protocol_config_request_t volatile_only = {};
  volatile_only.kind = MOTOR_PROTOCOL_CONFIG_SAVE_DEVICE_FLASH;
  volatile_only.save_slots = kVolatileOnly;
  assert(motor_protocol_cerb_adapter.config_stage_count(profile, &volatile_only) == 0U);

  // Vaxor 无设备对象 → 0 阶段（内核回 UNSUPPORTED；其限值是本地软限位，
  // 记录落 W25 即持久，没有设备侧 flash 要写）。
  const motor_protocol_model_profile_t *vp = nullptr;
  assert(find_profile("vaxor_2550", &vp));
  assert(motor_protocol_vaxor_adapter.config_stage_count(vp, &both) == 0U);
}

static void test_adapter_registry_is_shared_by_runtime_and_batching() {
  size_t adapter_count = 0U;
  const motor_protocol_adapter_t *const *adapters =
      motor_protocol_adapter_registry(&adapter_count);
  assert(adapters != nullptr);
  assert(adapter_count == 2U);
  assert(adapters[0] == &motor_protocol_vaxor_adapter);
  assert(adapters[1] == &motor_protocol_cerb_adapter);
  assert(motor_protocol_adapter_registry(nullptr) == nullptr);
}

static void test_cerb_tmax_and_node_write_readback() {
  const motor_protocol_model_profile_t *profile = nullptr;
  assert(find_profile("cerb_r48_101", &profile));
  motor_protocol_config_request_t req{};
  req.kind = MOTOR_PROTOCOL_CONFIG_SET_T_MAX;
  req.motor_id = 1; req.t_max_raw = 150;
  assert(motor_protocol_cerb_adapter.config_stage_count(profile, &req) == 2);
  motor_protocol_tx_frame_t frame{};
  assert(motor_protocol_cerb_adapter.config_encode_stage(0, profile, 1, 0, &req, &frame));
  const uint8_t expected[] = {0x23, 0x46, 0x25, 0, 150, 0, 0, 0};
  assert(frame.can_id == 0x601 && memcmp(frame.data, expected, 8) == 0);
  assert(motor_protocol_cerb_adapter.config_encode_stage(1, profile, 1, 0, &req, &frame));
  assert(frame.data[0] == 0x40 && frame.data[1] == 0x46);
  req.kind = MOTOR_PROTOCOL_CONFIG_SET_NODE_ID;
  req.new_node_id = 7;
  assert(motor_protocol_cerb_adapter.config_stage_count(profile, &req) == 2);
  assert(motor_protocol_cerb_adapter.config_encode_stage(0, profile, 1, 0, &req, &frame));
  assert(frame.can_id == 0x601 && frame.data[1] == 0x30 && frame.data[2] == 0x25 && frame.data[4] == 7);
  assert(motor_protocol_cerb_adapter.config_encode_stage(1, profile, 1, 0, &req, &frame));
  assert(frame.can_id == 0x601 && frame.data[0] == 0x40);
}

int main() {
  test_cerb_tmax_and_node_write_readback();
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
  test_cerb_decode_feedback_converts_si();
  test_cerb_encode_mit_command();
  test_cerb_profile_separates_wire_limits_from_software_caps();
  test_cerb_high_node_uses_multi_axis_only();
  test_cerb_encode_rejects_all_zero_raw_payload();
  test_cerb_velocity_unit_contract();
  test_cerb_enable_disable_frames();
  test_cerb_encode_soft_stop();
  test_cerb_encode_discovery_sync();
  test_cerb_config_unsupported_paths();
  test_cerb_sdo_position_limit_frames();
  test_cerb_save_flash_transaction();
  test_cerb_set_zero_frames();
  test_cerb_sdo_velocity_torque_frames();
  test_cerb_sdo_response_judging();
  test_cerb_identity_read_transaction();
  test_cerb_descriptor_table_bounds();
  test_cerb_refresh_params_transaction();
  test_cerb_refresh_covers_safety_slots();
  test_cerb_safety_protect_write_frames();
  test_cerb_direction_write_frames();
  test_adapter_id_ranges_disjoint();
  test_profile_catalog_only_exposes_supported_models();
  test_p1_batch_passes_through_unbatchable_frames();
  test_cerb_p1_batch_aggregates_two_to_six_axes();
  test_p1_batch_stops_at_incompatible_or_duplicate_frame();
  test_adapter_registry_is_shared_by_runtime_and_batching();
  printf("test_motor_protocol_adapter: all passed\n");
  return 0;
}
