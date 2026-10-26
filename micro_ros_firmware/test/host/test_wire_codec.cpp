// 定点编解码的 host 测试。golden 向量与主机侧 C++ 用例取同一组值
// （ros2_ws/src/micro_ros_bridge/test/test_codec.cpp），两边注释互指；
// 量程常量本身仍是单一来源（WireContract.msg），这里只钉算术。
//
// 文件是 .cpp 而库是 .c：仓库里 22 个 host 测试全是 .cpp + <cassert>，
// wire_codec.h 有 extern "C" 保护，直编无碍。
#include <cassert>
#include <cmath>
#include <cstdio>

#include "wire_codec.h"

namespace {
constexpr double kScalePos = 12.5;
constexpr double kScaleVel = 64.0;

void test_golden_vectors()
{
  assert(wire_quantize(0.0, kScalePos).q == 0);
  assert(wire_quantize(kScalePos, kScalePos).q == 32767);
  assert(wire_quantize(-kScalePos, kScalePos).q == -32767);
  // 步长 = 12.5/32767 ≈ 0.000381469；0.00625 落在第 16 格
  assert(wire_quantize(0.00625, kScalePos).q == 16);
  assert(wire_quantize(64.0, kScaleVel).q == 32767);
}

void test_saturation()
{
  wire_quantize_result_t r = wire_quantize(1000.0, kScalePos);
  assert(r.q == 32767 && r.saturated);
  r = wire_quantize(-1000.0, kScalePos);
  assert(r.q == -32767 && r.saturated);
  // 边界内不算饱和
  r = wire_quantize(kScalePos, kScalePos);
  assert(!r.saturated);
}

void test_round_trip_within_half_step()
{
  const double step = kScalePos / 32767.0;
  for (int i = -1000; i <= 1000; ++i) {
    const double x = i * step * 3.7;
    const double back = wire_dequantize(wire_quantize(x, kScalePos).q, kScalePos);
    assert(std::fabs(back - x) <= step / 2.0 + 1e-12);
  }
}

void test_sentinel_is_not_a_value()
{
  assert(std::isnan(wire_dequantize(WIRE_SENTINEL_MISSING, kScalePos)));
  // 负满量程不会产生哨兵
  assert(wire_quantize(-kScalePos, kScalePos).q == -32767);
}

void test_non_finite_yields_sentinel()
{
  // 前置条件：调用方先校验有限性（命令路径 NaN = 整条拒收）。
  // 本函数对非有限输入给哨兵，而不是未定义行为。
  assert(wire_quantize(NAN, kScalePos).q == WIRE_SENTINEL_MISSING);
  assert(wire_quantize(INFINITY, kScalePos).q == WIRE_SENTINEL_MISSING);
}

void test_frame_shape()
{
  // count 必须等于四个序列的声明长度，且不超过内核容量。
  // 0 = 空目标列表 = 全部软停，合法。
  assert(wire_frame_shape_ok(2, 2, 2, 2, 2, 64));
  assert(wire_frame_shape_ok(0, 0, 0, 0, 0, 64));
  assert(!wire_frame_shape_ok(2, 2, 2, 2, 1, 64));
  assert(!wire_frame_shape_ok(2, 1, 2, 2, 2, 64));
  assert(!wire_frame_shape_ok(2, 2, 2, 2, 2, 1));
  assert(!wire_frame_shape_ok(65, 65, 65, 65, 65, 64));
}

void test_config_groups_exclusive()
{
  // PARAM_SET：只带参数组。命令组不适用时，bridge 的编码端写的是
  // command = COMMAND_SAVE(0)、motor_id = MOTOR_ID_NONE(65535)——
  // 不是 0（0 是合法电机 id），照着 .msg 字面「0/空」写会拒掉每一条合法参数写。
  assert(wire_config_groups_exclusive(
      WIRE_CONFIG_KIND_PARAM_SET, 2, 2, 0, WIRE_MOTOR_ID_NONE));
  assert(!wire_config_groups_exclusive(
      WIRE_CONFIG_KIND_PARAM_SET, 2, 1, 0, WIRE_MOTOR_ID_NONE));
  assert(!wire_config_groups_exclusive(
      WIRE_CONFIG_KIND_PARAM_SET, 2, 2, 4, WIRE_MOTOR_ID_NONE));
  assert(!wire_config_groups_exclusive(WIRE_CONFIG_KIND_PARAM_SET, 2, 2, 0, 0));
  // SYSTEM_COMMAND：只带命令组；目标必须为空。motor_id 可以是 MOTOR_ID_NONE。
  assert(wire_config_groups_exclusive(
      WIRE_CONFIG_KIND_SYSTEM_COMMAND, 0, 0, 0, WIRE_MOTOR_ID_NONE));
  assert(!wire_config_groups_exclusive(WIRE_CONFIG_KIND_SYSTEM_COMMAND, 1, 1, 0, 0));
  // 未知 kind 整条拒。
  assert(!wire_config_groups_exclusive(7, 0, 0, 0, 0));
}
void test_state_age_never_vs_saturated()
{
  // 0xFFFF = 从未收到；0xFFFE = 真实年龄饱和。65535 ms 是真实年龄，
  // 不能被读成"从未收到"。
  assert(wire_encode_state_age(0xFFFFFFFFu) == WIRE_AGE_MS_NEVER);
  assert(wire_encode_state_age(0u) == 0);
  assert(wire_encode_state_age(1000u) == 1000);
  assert(wire_encode_state_age(0xFFFDu) == 0xFFFDu);
  assert(wire_encode_state_age(0xFFFEu) == WIRE_AGE_MS_SATURATED);
  assert(wire_encode_state_age(0xFFFFu) == WIRE_AGE_MS_SATURATED);
  assert(wire_encode_state_age(0xFFFFFFFEu) == WIRE_AGE_MS_SATURATED);
}

void test_temperature_dc()
{
  // 0.1 ℃ 定点；缺失用 int16 哨兵，非有限输入不得走浮点转整数。
  assert(wire_encode_temperature_dc(25.3) == 253);
  assert(wire_encode_temperature_dc(-12.3) == -123);
  assert(wire_encode_temperature_dc(0.0) == 0);
  assert(wire_encode_temperature_dc(NAN) == WIRE_SENTINEL_MISSING);
  assert(wire_encode_temperature_dc(INFINITY) == WIRE_SENTINEL_MISSING);
  assert(wire_encode_temperature_dc(-INFINITY) == WIRE_SENTINEL_MISSING);
  // 超量程先钳位再取整：±3276.7 ℃ 之外饱和到 int16 可用边界。
  assert(wire_encode_temperature_dc(3276.8) == 32767);
  assert(wire_encode_temperature_dc(-3276.8) == -32767);
  assert(wire_encode_temperature_dc(1e300) == 32767);
  assert(wire_encode_temperature_dc(-1e300) == -32767);
}

void test_current_ma()
{
  assert(wire_encode_current_ma(1.5) == 1500);
  assert(wire_encode_current_ma(-2.25) == -2250);
  assert(wire_encode_current_ma(0.0) == 0);
  assert(wire_encode_current_ma(NAN) == WIRE_SENTINEL_MISSING);
  assert(wire_encode_current_ma(32.768) == 32767);
  assert(wire_encode_current_ma(-32.768) == -32767);
  assert(wire_encode_current_ma(1e300) == 32767);
}

void test_voltage_mv()
{
  // 0xFFFF 是缺失哨兵，所以正向量程只到 0xFFFE mV。
  assert(wire_encode_voltage_mv(24.0) == 24000);
  assert(wire_encode_voltage_mv(0.0) == 0);
  assert(wire_encode_voltage_mv(0.0004) == 0);
  assert(wire_encode_voltage_mv(NAN) == WIRE_VOLTAGE_MV_MISSING);
  assert(wire_encode_voltage_mv(INFINITY) == WIRE_VOLTAGE_MV_MISSING);
  assert(wire_encode_voltage_mv(70.0) == 0xFFFEu);
  assert(wire_encode_voltage_mv(1e300) == 0xFFFEu);
  assert(wire_encode_voltage_mv(-1.0) == 0);
}

void test_status_flags()
{
  // 位序必须与 MotorStatus.msg 头注释、主机 decode_motor_status 一致。
  wire_status_flags_input_t f = {};
  assert(wire_encode_status_flags(f) == 0x00u);
  f.online = true;
  assert(wire_encode_status_flags(f) == 0x01u);
  f.device_protect_known = true;
  assert(wire_encode_status_flags(f) == 0x03u);
  f.device_protect_enabled = true;
  assert(wire_encode_status_flags(f) == 0x07u);
  f.device_direction_known = true;
  assert(wire_encode_status_flags(f) == 0x0Fu);
  f.device_direction = 1;
  assert(wire_encode_status_flags(f) == 0x1Fu);
  f.device_direction = 0;
  assert(wire_encode_status_flags(f) == 0x0Fu);
  f.device_direction = 2;  // 原值直通：非零即 CCW
  assert((wire_encode_status_flags(f) & 0x10u) != 0u);
}
}  // namespace

void test_wide_electrical_telemetry()
{
  assert(wire_encode_current_ma_i32(100.0) == 100000);
  assert(wire_encode_current_ma_i32(-100.0) == -100000);
  assert(wire_encode_current_ma_i32(NAN) == (-2147483647 - 1));
  assert(wire_encode_voltage_mv_u32(72.0) == 72000U);
  assert(wire_encode_voltage_mv_u32(NAN) == 0xFFFFFFFFu);
}

void test_command_session_and_sequence()
{
  wire_command_guard_t guard = {};
  wire_command_guard_reset(&guard, 77);
  assert(!wire_command_guard_allows(&guard, 0, 1));
  assert(!wire_command_guard_allows(&guard, 76, 1));
  assert(wire_command_guard_allows(&guard, 77, 1));
  wire_command_guard_commit(&guard, 1);
  assert(!wire_command_guard_allows(&guard, 77, 1));
  assert(!wire_command_guard_allows(&guard, 77, 0));
  assert(wire_command_guard_allows(&guard, 77, 2));
  wire_command_guard_commit(&guard, 0xFFFFFFFFu);
  assert(wire_command_guard_allows(&guard, 77, 0));
  wire_command_guard_reset(&guard, 88);
  assert(!wire_command_guard_allows(&guard, 77, 3));
  assert(wire_command_guard_allows(&guard, 88, 1));
}

int main()
{
  assert(wire_command_deadline_valid(100U, 101U));
  assert(!wire_command_deadline_valid(101U, 101U));
  assert(!wire_command_deadline_valid(102U, 101U));
  assert(wire_command_deadline_valid(0xFFFFFFF0U, 5U));
  assert(!wire_command_deadline_valid(5U, 0xFFFFFFF0U));
  assert(wire_session_from_epoch_ns(1792828800123456789ULL, 0U) == 1792828800123ULL);
  assert(wire_session_from_epoch_ns(1792828800123456789ULL, 1792828800123ULL) == 1792828800124ULL);
  assert(wire_session_from_epoch_ns(0U, 123U) == 0U);
  assert(wire_session_from_epoch_ns(UINT64_MAX, 0U) < 9007199254740991ULL);
  test_command_session_and_sequence();
  test_wide_electrical_telemetry();
  test_golden_vectors();
  test_saturation();
  test_round_trip_within_half_step();
  test_sentinel_is_not_a_value();
  test_non_finite_yields_sentinel();
  test_frame_shape();
  test_config_groups_exclusive();
  test_state_age_never_vs_saturated();
  test_temperature_dc();
  test_current_ma();
  test_voltage_mv();
  test_status_flags();
  std::printf("test_wire_codec: ok\n");
  return 0;
}
