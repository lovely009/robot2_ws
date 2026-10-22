#include <cassert>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstring>

#include "motor_protocol.h"

namespace {

constexpr double kVaxorPositionTolerance = 0.00001;
constexpr double kVaxorVelocityTolerance = 0.011;
constexpr double kVaxorKpTolerance = 0.00001;
constexpr double kVaxorKdTolerance = 0.01;
constexpr double kVaxorCurrentTolerance = 0.003;

void test_vaxor_12_byte_command_round_trip() {
  const motor_protocol_joint_command_t input{1.5, -2.0, 0.12345, 0.67, -1.1};
  uint8_t bytes[12]{};
  motor_protocol_pack_vaxor_command(&input, bytes);

  motor_protocol_joint_command_t output{};
  assert(motor_protocol_unpack_vaxor_command(bytes, sizeof(bytes), &output));
  assert(std::abs(output.position - input.position) < kVaxorPositionTolerance);
  assert(std::abs(output.velocity - input.velocity) < kVaxorVelocityTolerance);
  assert(std::abs(output.kp - input.kp) < kVaxorKpTolerance);
  assert(std::abs(output.kd - input.kd) < kVaxorKdTolerance);
  assert(std::abs(output.feedforward - input.feedforward) < kVaxorCurrentTolerance);
}

void test_vaxor_status_and_feedback_round_trip() {
  const motor_protocol_joint_state_t input{5.0, -10.0, 2.0};
  uint8_t reply[12]{};
  motor_protocol_pack_vaxor_reply(2U, &input, reply);

  motor_protocol_joint_state_t output{};
  assert(motor_protocol_unpack_vaxor_feedback(reply, sizeof(reply), &output));
  assert(std::abs(output.position - input.position) < kVaxorPositionTolerance);
  assert(std::abs(output.velocity - input.velocity) < kVaxorVelocityTolerance);
  assert(std::abs(output.effort - input.effort) < kVaxorCurrentTolerance);
}

void test_vaxor_status_decodes_all_status_fields() {
  const uint8_t bytes[20]{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
                          0xe6U, 0x01U, 0x78U, 0x56U, 0x34U, 0x12U,
                          4U, 5U, 2U, 7U};

  motor_protocol_vaxor_status_t status{};
  assert(motor_protocol_unpack_vaxor_status(bytes, sizeof(bytes), &status));
  assert(std::abs(status.voltage_v - 48.6) < 0.051);
  assert(status.fault == 0x12345678U);
  assert(status.motor_state == 4U);
  assert(status.calibration_state == 5U);
  assert(status.mode == 2U);
  assert(status.motor_type == 7U);
}

void test_vaxor_status_encodes_voltage_as_unbiased_signed_tenths() {
  const motor_protocol_joint_state_t input{};
  uint8_t bytes[20]{};
  motor_protocol_pack_vaxor_status(2U, &input, 48.6, 0U,
      0U, 0U, 0U, 0U, bytes);
  assert(bytes[10] == 0xe6U);
  assert(bytes[11] == 0x01U);
}

void test_vaxor_status_rejects_short_feedback() {
  uint8_t bytes[12]{};
  motor_protocol_vaxor_status_t status{};
  assert(!motor_protocol_unpack_vaxor_status(bytes, sizeof(bytes), &status));
}

void test_unpack_rejects_invalid_lengths() {
  uint8_t bytes[20]{};
  motor_protocol_joint_command_t command{};
  motor_protocol_joint_state_t state{};
  motor_protocol_vaxor_control_t control{};

  assert(!motor_protocol_unpack_vaxor_command(bytes, 11U, &command));
  assert(!motor_protocol_unpack_vaxor_control(bytes, 7U, &control));
  assert(!motor_protocol_unpack_vaxor_feedback(bytes, 11U, &state));
  assert(!motor_protocol_unpack_vaxor_feedback(bytes, 13U, &state));
}

void test_cerb_mit_frame_matches_new_protocol_layout() {
  const motor_protocol_joint_command_t command{12.5, -30.0, 500.0, 5.0,
                                               -10.0};
  const motor_protocol_cerb_limits_t limits{12.5, 30.0, 10.0, 2500.0, 500.0};
  uint8_t bytes[12]{};
  assert(motor_protocol_pack_cerb_mit_frame(&command, &limits, true, bytes));

  // kp=500/Nm 满量程 2500 → round(819.0)；kd=5 满量程 500 → round(40.95)=41。
  const uint8_t expected[12]{0xcc, 0xff, 0xff, 0x00, 0x03, 0x33,
                             0x02, 0x90, 0x00, 0x00, 0x00, 0x00};
  for (size_t index = 0; index < sizeof(expected); ++index) {
    assert(bytes[index] == expected[index]);
  }
}

void test_cerb_disabled_neutral_frame_is_safe() {
  const motor_protocol_joint_command_t command{};
  const motor_protocol_cerb_limits_t limits{12.5, 30.0, 10.0, 2500.0, 500.0};
  uint8_t bytes[12]{};
  assert(motor_protocol_pack_cerb_mit_frame(&command, &limits, false, bytes));

  // 零命令必须编码到 raw 中点（round 后 32768 / 2048 / 2048），不是截断的 32767/2047。
  const uint8_t expected[12]{0x0c, 0x80, 0x00, 0x80, 0x00, 0x00,
                             0x00, 0x08, 0x00, 0x00, 0x00, 0x00};
  for (size_t index = 0; index < sizeof(expected); ++index) {
    assert(bytes[index] == expected[index]);
  }
}

void test_cerb_feedback_decodes_big_endian_units() {
  const uint8_t bytes[12]{0x07, 0x1c, 0xff, 0xc4, 0x05, 0xdc,
                          0x00, 0x00, 0x00, 0xf0, 0x06, 0xc0};
  motor_protocol_joint_state_t state{};
  assert(motor_protocol_unpack_cerb_feedback(bytes, sizeof(bytes), &state));
  assert(std::abs(state.position - 10.0 * MOTOR_PROTOCOL_PI / 180.0) < 1e-12);
  assert(std::abs(state.velocity + 2.0 * MOTOR_PROTOCOL_PI) < 1e-12);
  assert(state.effort == 30.0); // Iq 码 1500 × 20 mA/码
  // data[8..9] = 0x00f0 = 240 → 24.0 °C
  assert(motor_protocol_cerb_feedback_temperature_c(bytes, sizeof(bytes)) == 24.0f);
}

// 温度独占 data[8..9]：与运动量解码分开，故障帧同样可读（故障帧里运动量
// 不可信，但过温恰恰是故障成因），帧形不成立时返回 NaN。
void test_cerb_feedback_temperature_decoding() {
  uint8_t bytes[12]{};
  bytes[8] = 0xff;
  bytes[9] = 0x38; // -200 → -20.0 °C
  assert(motor_protocol_cerb_feedback_temperature_c(bytes, sizeof(bytes)) == -20.0f);
  bytes[11] = 0x20; // 故障位：温度不受影响
  assert(motor_protocol_cerb_feedback_temperature_c(bytes, sizeof(bytes)) == -20.0f);
  assert(std::isnan(motor_protocol_cerb_feedback_temperature_c(bytes, 11U)));
  assert(std::isnan(motor_protocol_cerb_feedback_temperature_c(NULL, 12U)));
}

void test_cerb_rejects_invalid_or_faulted_data() {
  motor_protocol_joint_command_t command{};
  command.position = NAN;
  const motor_protocol_cerb_limits_t limits{12.5, 30.0, 10.0, 2500.0, 500.0};
  uint8_t bytes[12]{};
  assert(!motor_protocol_pack_cerb_mit_frame(&command, &limits, true, bytes));

  const motor_protocol_joint_command_t finite_command{};
  const motor_protocol_cerb_limits_t invalid_limits{0.0, 30.0, 10.0, 2500.0, 500.0};
  assert(!motor_protocol_pack_cerb_mit_frame(
      &finite_command, &invalid_limits, true, bytes));
  const motor_protocol_cerb_limits_t invalid_kp{12.5, 30.0, 10.0, 0.0, 500.0};
  assert(!motor_protocol_pack_cerb_mit_frame(
      &finite_command, &invalid_kp, true, bytes));
  const motor_protocol_cerb_limits_t invalid_kd{12.5, 30.0, 10.0, 2500.0, NAN};
  assert(!motor_protocol_pack_cerb_mit_frame(
      &finite_command, &invalid_kd, true, bytes));
  const motor_protocol_cerb_limits_t overflowing_limits{DBL_MAX, 30.0, 10.0,
                                                        2500.0, 500.0};
  assert(!motor_protocol_pack_cerb_mit_frame(
      &finite_command, &overflowing_limits, true, bytes));

  bytes[11] = 0x20;
  motor_protocol_joint_state_t state{};
  assert(!motor_protocol_unpack_cerb_feedback(bytes, sizeof(bytes), &state));
  assert(!motor_protocol_unpack_cerb_feedback(bytes, 11U, &state));
}

void test_cerb_large_finite_limit_quantizes_without_overflow() {
  const motor_protocol_cerb_limits_t limits{DBL_MAX / 4.0, 30.0, 10.0,
                                            2500.0, 500.0};
  const motor_protocol_joint_command_t command{limits.position_rad, 0.0, 0.0,
                                               0.0, 0.0};
  uint8_t bytes[12]{};
  assert(motor_protocol_pack_cerb_mit_frame(&command, &limits, false, bytes));
  assert(bytes[1] == 0xff);
  assert(bytes[2] == 0xff);
}

void test_cerb_node_ids_cover_declared_range() {
  // 多轴列表/发现/反馈/SDO 仍允许 1..127。
  assert(!motor_protocol_cerb_node_valid(0U));
  assert(motor_protocol_cerb_node_valid(1U));
  assert(motor_protocol_cerb_node_valid(127U));
  assert(!motor_protocol_cerb_node_valid(128U));
  // 单轴入口 0x110|NodeID 只在 1..15 无歧义（bit4 起会与基址/其他 ID 重叠）。
  assert(!motor_protocol_cerb_single_node_valid(0U));
  assert(motor_protocol_cerb_single_node_valid(1U));
  assert(motor_protocol_cerb_single_node_valid(15U));
  assert(!motor_protocol_cerb_single_node_valid(16U));
  assert(!motor_protocol_cerb_single_node_valid(127U));
  uint32_t id = 0U;
  assert(motor_protocol_cerb_command_id(1U, &id));
  assert(id == (0x110U | 1U));
  assert(motor_protocol_cerb_command_id(15U, &id));
  assert(id == 0x11fU);
  assert(!motor_protocol_cerb_command_id(16U, &id));
  assert(!motor_protocol_cerb_command_id(127U, &id)); // 高节点只走 0x210
  assert(!motor_protocol_cerb_command_id(0U, &id));
  assert(!motor_protocol_cerb_command_id(128U, &id));
  assert(!motor_protocol_cerb_command_id(1U, nullptr));
  assert(motor_protocol_cerb_feedback_id(1U, &id));
  assert(id == 0x301U);
  assert(motor_protocol_cerb_feedback_id(127U, &id));
  assert(id == 0x37fU);
}

// 0x210 的槽位顺序、ID 表和保留字节必须与厂家 64B 示例一致。
void test_cerb_multi_mit_packs_two_axes_and_zeroes_unused_slots() {
  const motor_protocol_cerb_multi_axis_t axes[2] = {
    {1U, {0xcc, 0x80, 0x00, 0x80, 0x03, 0xe8, 0x14, 0x18, 0x00}},
    {2U, {0xcc, 0x80, 0x01, 0x80, 0x03, 0xe8, 0x14, 0x18, 0x01}},
  };
  uint8_t out[64];
  memset(out, 0xff, sizeof(out));
  assert(motor_protocol_pack_cerb_multi_mit(axes, 2U, out));
  assert(memcmp(out, axes[0].data, 9U) == 0);
  assert(memcmp(out + 9U, axes[1].data, 9U) == 0);
  for (size_t i = 18U; i < 56U; ++i) assert(out[i] == 0U);
  assert(out[56] == 1U && out[57] == 2U);
  for (size_t i = 58U; i < 64U; ++i) assert(out[i] == 0U);
}

void test_cerb_multi_mit_rejects_invalid_axes_without_partial_output() {
  motor_protocol_cerb_multi_axis_t axes[7] = {};
  axes[0].node = 1U;
  axes[1].node = 1U;
  uint8_t out[64];
  memset(out, 0x5a, sizeof(out));
  assert(!motor_protocol_pack_cerb_multi_mit(axes, 2U, out));
  for (uint8_t b : out) assert(b == 0x5a);
  axes[1].node = 2U;
  assert(!motor_protocol_pack_cerb_multi_mit(axes, 7U, out));
  assert(!motor_protocol_pack_cerb_multi_mit(axes, 0U, out));
  axes[1].node = 0U;
  assert(!motor_protocol_pack_cerb_multi_mit(axes, 2U, out));
  // 多轴列表使用完整 1..127 节点空间（高节点不能被单轴限制连带拒绝）。
  axes[0].node = 16U;
  axes[1].node = 127U;
  assert(motor_protocol_pack_cerb_multi_mit(axes, 2U, out));
  assert(out[56] == 16U && out[57] == 127U);
}

void test_cerb_multi_mit_recognizes_only_cerb_control_frames() {
  const uint8_t mit[12] = {0xcc, 0x80, 0, 0x80, 3, 0xe8, 0x14, 0x18, 0};
  uint8_t node = 0U;
  assert(motor_protocol_cerb_single_mit_node(0x111, false, true, true,
      mit, 12U, &node));
  assert(node == 1U);
  assert(!motor_protocol_cerb_single_mit_node(0x11f, false, true, true,
      mit, 12U, &node) || node == 15U);
  assert(motor_protocol_cerb_single_mit_node(0x11f, false, true, true,
      mit, 12U, &node));
  assert(node == 15U);
  assert(!motor_protocol_cerb_single_mit_node(0x120, false, true, true,
      mit, 12U, &node));
  assert(!motor_protocol_cerb_single_mit_node(0x17f, false, true, true,
      mit, 12U, &node));
  assert(!motor_protocol_cerb_single_mit_node(0x210, false, true, true,
      mit, 12U, &node));
  assert(!motor_protocol_cerb_single_mit_node(0x111, false, false, false,
      mit, 12U, &node));
  assert(!motor_protocol_cerb_single_mit_node(0x111, false, true, true,
      mit, 9U, &node));
  uint8_t clear_fault[12];
  memcpy(clear_fault, mit, sizeof(mit));
  clear_fault[0] |= 0x20U;
  assert(!motor_protocol_cerb_single_mit_node(0x111, false, true, true,
      clear_fault, 12U, &node));
}


// 指南默认五项量程：P/V=3.14、Kp=2500、Kd=500、T=300。
// 金字节来自指南 §2.2/§5/§6，不是从实现反推。
void test_cerb_guide_default_quantization() {
  const motor_protocol_cerb_limits_t limits{3.14, 3.14, 300.0, 2500.0, 500.0};
  uint8_t bytes[12]{};

  // kp=30 → raw 49；kd=1 → raw 8（旧实现用 500/5，会得到 245/819）。
  const motor_protocol_joint_command_t gains{0.0, 0.0, 30.0, 1.0, 0.0};
  assert(motor_protocol_pack_cerb_mit_frame(&gains, &limits, false, bytes));
  assert(bytes[1] == 0x80 && bytes[2] == 0x00); // P 中点
  assert(bytes[3] == 0x80 && bytes[4] == 0x00); // V 中点 | kp 高 4 位 0
  assert(bytes[5] == 49U);
  assert(bytes[6] == 0x00 && bytes[7] == 0x88); // kd=8，torque 中点 0x800
  assert(bytes[8] == 0x00);

  // q=1 rad → 43203 = 0xA8C3。
  const motor_protocol_joint_command_t position{1.0, 0.0, 0.0, 0.0, 0.0};
  assert(motor_protocol_pack_cerb_mit_frame(&position, &limits, false, bytes));
  const uint32_t raw = ((uint32_t)bytes[1] << 8U) | bytes[2];
  assert(raw == 43203U);

  // 满量程 3.14 rad → 65535；速度 ±3.14 → 4095/0。
  const motor_protocol_joint_command_t full{3.14, 3.14, 0.0, 0.0, 0.0};
  assert(motor_protocol_pack_cerb_mit_frame(&full, &limits, false, bytes));
  assert(bytes[1] == 0xffU && bytes[2] == 0xffU);
  assert(bytes[3] == 0xffU && (bytes[4] >> 4U) == 0x0fU);
}

// 指南 §5.1 解码公式与 §5.2 编码公式往返误差 ≤ 1 LSB（五项全部）。
void test_cerb_guide_default_round_trip_within_one_lsb() {
  const motor_protocol_cerb_limits_t limits{3.14, 3.14, 300.0, 2500.0, 500.0};
  const motor_protocol_joint_command_t command{1.0, -2.0, 30.0, 1.0, 5.0};
  uint8_t bytes[12]{};
  assert(motor_protocol_pack_cerb_mit_frame(&command, &limits, true, bytes));
  const uint32_t p_raw = ((uint32_t)bytes[1] << 8U) | bytes[2];
  const uint32_t v_raw = ((uint32_t)bytes[3] << 4U) | (bytes[4] >> 4U);
  const uint32_t kp_raw = ((uint32_t)(bytes[4] & 0x0fU) << 8U) | bytes[5];
  const uint32_t kd_raw = ((uint32_t)bytes[6] << 4U) | (bytes[7] >> 4U);
  const uint32_t t_raw = ((uint32_t)(bytes[7] & 0x0fU) << 8U) | bytes[8];
  const double p = p_raw / 65535.0 * 6.28 - 3.14;
  const double v = v_raw / 4095.0 * 6.28 - 3.14;
  const double kp = kp_raw / 4095.0 * 2500.0;
  const double kd = kd_raw / 4095.0 * 500.0;
  const double t = t_raw / 4095.0 * 600.0 - 300.0;
  assert(std::abs(p - 1.0) <= 0.0001);
  assert(std::abs(v + 2.0) <= 0.0016);
  assert(std::abs(kp - 30.0) <= 0.62);
  assert(std::abs(kd - 1.0) <= 0.13);
  assert(std::abs(t - 5.0) <= 0.15);
}

// 指南 §6：五个 raw 全部为零不是零命令，必须拒绝而不是静默下发。
void test_cerb_all_zero_raw_payload_is_rejected() {
  const motor_protocol_cerb_limits_t limits{3.14, 3.14, 300.0, 2500.0, 500.0};
  const motor_protocol_joint_command_t command{-3.14, -3.14, 0.0, 0.0, -300.0};
  uint8_t bytes[12];
  memset(bytes, 0xa5, sizeof(bytes));
  assert(!motor_protocol_pack_cerb_mit_frame(&command, &limits, true, bytes));
  // 拒绝时不得留下半帧（适配层应报告失败，不得假装已发送）。
  for (uint8_t b : bytes) assert(b == 0xa5U);
}

void test_cerb_sdo_limit_scales_invert_forward_encodings() {
  // 0x607D 位置限位：65536 cnt/360°（负载端，与关节同轴）。
  assert(motor_protocol_cerb_limit_cnt_to_rad(32768) == MOTOR_PROTOCOL_PI);
  assert(motor_protocol_cerb_limit_cnt_to_rad(-32768) == -MOTOR_PROTOCOL_PI);
  assert(motor_protocol_cerb_limit_cnt_to_rad(0) == 0.0);
  // 90° = 16384 cnt
  assert(std::abs(motor_protocol_cerb_limit_cnt_to_rad(16384) -
      MOTOR_PROTOCOL_PI / 2.0) < 1e-9);

  // 0x6080 电机端 RPM → 关节侧 rad/s（÷gear；正向换算的逆函数）。
  // 600 RPM @ gear=101：电机侧 20π rad/s，关节侧 20π/101。
  const double gear = 101.0;
  assert(std::abs(motor_protocol_cerb_motor_rpm_to_joint_rad_s(gear, 600U) -
      2.0 * MOTOR_PROTOCOL_PI * 10.0 / gear) < 1e-12);
  assert(motor_protocol_cerb_motor_rpm_to_joint_rad_s(gear, 0U) == 0.0);

  // 0x2538 电机端 mA → 关节侧 N·m：τ = Iq × Kt × gear × eff。
  // 1000 mA, Kt=0.077, gear=101, eff=1.0 → 7.777 N·m。
  assert(std::abs(motor_protocol_cerb_motor_ma_to_joint_nm(
      0.077, gear, 1.0, 1000U) -
      0.077 * gear) < 1e-12);
  assert(std::abs(motor_protocol_cerb_motor_ma_to_joint_nm(
      0.077, gear, 0.8, 2500U) -
      2.5 * 0.077 * gear * 0.8) < 1e-12);
}

}  // namespace

int main() {
  test_vaxor_12_byte_command_round_trip();
  test_vaxor_status_and_feedback_round_trip();
  test_vaxor_status_decodes_all_status_fields();
  test_vaxor_status_encodes_voltage_as_unbiased_signed_tenths();
  test_vaxor_status_rejects_short_feedback();
  test_unpack_rejects_invalid_lengths();
  test_cerb_mit_frame_matches_new_protocol_layout();
  test_cerb_disabled_neutral_frame_is_safe();
  test_cerb_feedback_decodes_big_endian_units();
  test_cerb_feedback_temperature_decoding();
  test_cerb_rejects_invalid_or_faulted_data();
  test_cerb_large_finite_limit_quantizes_without_overflow();
  test_cerb_node_ids_cover_declared_range();
  test_cerb_guide_default_quantization();
  test_cerb_guide_default_round_trip_within_one_lsb();
  test_cerb_all_zero_raw_payload_is_rejected();
  test_cerb_multi_mit_packs_two_axes_and_zeroes_unused_slots();
  test_cerb_multi_mit_rejects_invalid_axes_without_partial_output();
  test_cerb_multi_mit_recognizes_only_cerb_control_frames();
  test_cerb_sdo_limit_scales_invert_forward_encodings();
  return 0;
}
