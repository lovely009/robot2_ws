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
}  // namespace

int main()
{
  test_golden_vectors();
  test_saturation();
  test_round_trip_within_half_step();
  test_sentinel_is_not_a_value();
  test_non_finite_yields_sentinel();
  test_frame_shape();
  test_config_groups_exclusive();
  std::printf("test_wire_codec: ok\n");
  return 0;
}
