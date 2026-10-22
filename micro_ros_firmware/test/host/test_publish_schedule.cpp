// 发布调度纯逻辑的 host 测试：5 ms 基准 = 200 拍/秒。
// 拍号是唯一时间输入；期望值全部按拍号手算，不复用调度器的取模逻辑。
#include <cassert>
#include <cstdint>
#include <cstdio>

#include "publish_schedule.h"

namespace {

publish_schedule_output_t at(uint32_t tick, bool event_pending = false,
    bool topology_pending = false, bool device_param_pending = false,
    publish_reliable_t last_spare = PUBLISH_RELIABLE_NONE) {
  return publish_schedule_step(tick, event_pending, topology_pending,
      device_param_pending, last_spare);
}

void test_stream_rates_over_five_seconds() {
  // 1000 拍 = 5 s。手算：偶数拍 500 → LowState；%4==1 的 250 拍里
  // 温度拍 49/149/.../949 共 10 条 → IMU 240；%4==3 的 250 拍无 stream。
  int low = 0, imu = 0, temp = 0, none = 0;
  for (uint32_t tick = 0; tick < 1000; ++tick) {
    const auto o = at(tick);
    // 无 pending 时 reliable 只有固定 10 Hz 的 MotorStatus。
    if (tick % 20 == 0) {
      assert(o.reliable == PUBLISH_RELIABLE_MOTOR_STATUS);
    } else {
      assert(o.reliable == PUBLISH_RELIABLE_NONE);
    }
    assert(o.spare_served == PUBLISH_RELIABLE_NONE);
    switch (o.stream) {
      case PUBLISH_STREAM_LOW_STATE: ++low; break;
      case PUBLISH_STREAM_IMU: ++imu; break;
      case PUBLISH_STREAM_IMU_TEMP: ++temp; break;
      case PUBLISH_STREAM_NONE: ++none; break;
      default: assert(false); break;
    }
  }
  assert(low == 500);
  assert(imu == 240);
  assert(temp == 10);
  assert(none == 250);
}

void test_exact_stream_slots() {
  assert(at(0).stream == PUBLISH_STREAM_LOW_STATE);
  assert(at(1).stream == PUBLISH_STREAM_IMU);
  assert(at(2).stream == PUBLISH_STREAM_LOW_STATE);
  assert(at(3).stream == PUBLISH_STREAM_NONE);
  assert(at(4).stream == PUBLISH_STREAM_LOW_STATE);
  assert(at(5).stream == PUBLISH_STREAM_IMU);
  assert(at(48).stream == PUBLISH_STREAM_LOW_STATE);
  assert(at(49).stream == PUBLISH_STREAM_IMU_TEMP);
  assert(at(51).stream == PUBLISH_STREAM_NONE);
  assert(at(53).stream == PUBLISH_STREAM_IMU);
  assert(at(149).stream == PUBLISH_STREAM_IMU_TEMP);
  assert(at(151).stream == PUBLISH_STREAM_NONE);
  assert(at(153).stream == PUBLISH_STREAM_IMU);
  assert(at(199).stream == PUBLISH_STREAM_NONE);
  assert(at(200).stream == PUBLISH_STREAM_LOW_STATE);
  assert(at(249).stream == PUBLISH_STREAM_IMU_TEMP);
}

void test_motor_status_cadence_and_priority() {
  // 每 20 拍一条 = 10 Hz；status 拍不吃 spare，且与 stream 互不挤占。
  int status = 0;
  for (uint32_t tick = 0; tick < 1000; ++tick) {
    const auto o = at(tick, true, true, true, PUBLISH_RELIABLE_NONE);
    if (o.reliable == PUBLISH_RELIABLE_MOTOR_STATUS) {
      ++status;
      assert(tick % 20 == 0);
      assert(o.spare_served == PUBLISH_RELIABLE_NONE);
    } else {
      assert(tick % 20 != 0);
      assert(o.spare_served == o.reliable);
    }
  }
  assert(status == 50);

  const auto o = at(20, true, true, true, PUBLISH_RELIABLE_NONE);
  assert(o.stream == PUBLISH_STREAM_LOW_STATE);
  assert(o.reliable == PUBLISH_RELIABLE_MOTOR_STATUS);
  assert(o.spare_served == PUBLISH_RELIABLE_NONE);
}

void test_spare_requires_pending() {
  const auto o = at(21);
  assert(o.reliable == PUBLISH_RELIABLE_NONE);
  assert(o.spare_served == PUBLISH_RELIABLE_NONE);
}

void test_round_robin_order() {
  auto o = at(21, true, true, true, PUBLISH_RELIABLE_NONE);
  assert(o.reliable == PUBLISH_RELIABLE_EVENT);
  o = at(22, true, true, true, o.spare_served);
  assert(o.reliable == PUBLISH_RELIABLE_TOPOLOGY);
  o = at(23, true, true, true, o.spare_served);
  assert(o.reliable == PUBLISH_RELIABLE_DEVICE_PARAM);
  o = at(24, true, true, true, o.spare_served);
  assert(o.reliable == PUBLISH_RELIABLE_EVENT);
}

void test_single_pending_is_never_starved() {
  // 轮转只在 pending 集合里走：不因 last_spare 停在别处而空转。
  assert(at(21, true, false, false, PUBLISH_RELIABLE_DEVICE_PARAM).reliable ==
      PUBLISH_RELIABLE_EVENT);
  assert(at(21, false, true, false, PUBLISH_RELIABLE_EVENT).reliable ==
      PUBLISH_RELIABLE_TOPOLOGY);
  assert(at(21, false, false, true, PUBLISH_RELIABLE_EVENT).reliable ==
      PUBLISH_RELIABLE_DEVICE_PARAM);
  assert(at(21, false, true, false, PUBLISH_RELIABLE_TOPOLOGY).reliable ==
      PUBLISH_RELIABLE_TOPOLOGY);
}

void test_fair_rotation_over_900_ticks() {
  // 头文件契约：每拍无条件把 spare_served 存为下一拍 last_spare，调用者不自行
  // 保留状态。1..900 共 900 拍，其中 45 拍被 MotorStatus 占用且不服务 spare，
  // 剩 855 个 spare 拍 → 三类各 285 条。
  int ev = 0, top = 0, dev = 0;
  publish_reliable_t last = PUBLISH_RELIABLE_NONE;
  for (uint32_t tick = 1; tick <= 900; ++tick) {
    const auto o = at(tick, true, true, true, last);
    last = o.spare_served;  // 严格按契约回灌，不做跨 status 拍的特殊保留
    if (o.reliable == PUBLISH_RELIABLE_EVENT) {
      ++ev;
    } else if (o.reliable == PUBLISH_RELIABLE_TOPOLOGY) {
      ++top;
    } else if (o.reliable == PUBLISH_RELIABLE_DEVICE_PARAM) {
      ++dev;
    }
  }
  if (ev != 285 || top != 285 || dev != 285) {
    std::printf("fair rotation: ev=%d top=%d dev=%d\n", ev, top, dev);
  }
  assert(ev == 285);
  assert(top == 285);
  assert(dev == 285);
}

void test_status_tick_preserves_rotation_position() {
  // MotorStatus 拍不服务 spare，但必须把 last_spare 原样交回；否则调用者按
  // 契约无条件回灌后轮转位置被清零，下一拍从 EVENT 重启，轮转不再公平。
  const auto status = at(20, true, true, true, PUBLISH_RELIABLE_DEVICE_PARAM);
  assert(status.stream == PUBLISH_STREAM_LOW_STATE);
  assert(status.reliable == PUBLISH_RELIABLE_MOTOR_STATUS);
  assert(status.spare_served == PUBLISH_RELIABLE_DEVICE_PARAM);
  // DEVICE_PARAM 之后的轮转位置是 EVENT。
  const auto next = at(21, true, true, true, status.spare_served);
  assert(next.reliable == PUBLISH_RELIABLE_EVENT);
}

void test_pending_waits_at_most_one_status_tick() {
  // 20 拍被 status 占用：本拍不吃 spare，紧邻的 21 拍立即服务单类 pending。
  auto o = at(20, true, false, false, PUBLISH_RELIABLE_NONE);
  assert(o.reliable == PUBLISH_RELIABLE_MOTOR_STATUS);
  assert(o.spare_served == PUBLISH_RELIABLE_NONE);
  o = at(21, true, false, false, o.spare_served);
  assert(o.reliable == PUBLISH_RELIABLE_EVENT);
}

void test_stream_and_reliable_are_independent() {
  auto o = at(21, true, false, false, PUBLISH_RELIABLE_NONE);
  assert(o.stream == PUBLISH_STREAM_IMU);
  assert(o.reliable == PUBLISH_RELIABLE_EVENT);
  o = at(3, false, true, false, PUBLISH_RELIABLE_NONE);
  assert(o.stream == PUBLISH_STREAM_NONE);
  assert(o.reliable == PUBLISH_RELIABLE_TOPOLOGY);
  o = at(4, false, false, true, PUBLISH_RELIABLE_NONE);
  assert(o.stream == PUBLISH_STREAM_LOW_STATE);
  assert(o.reliable == PUBLISH_RELIABLE_DEVICE_PARAM);
}

void test_200_tick_period_and_uint32_wrap() {
  for (uint32_t tick = 0; tick < 200; ++tick) {
    const auto a = at(tick, true, true, true, PUBLISH_RELIABLE_NONE);
    const auto b = at(tick + 200, true, true, true, PUBLISH_RELIABLE_NONE);
    assert(a.stream == b.stream);
    assert(a.reliable == b.reliable);
  }
  // 0xFFFFFFFE 是偶数拍 → LowState；0xFFFFFFFF %4==3 无 stream、%20==15
  // → 可作 spare 拍；回绕到 0 → LowState + MotorStatus。
  assert(at(0xFFFFFFFEu).stream == PUBLISH_STREAM_LOW_STATE);
  auto o = at(0xFFFFFFFFu, true, false, false, PUBLISH_RELIABLE_NONE);
  assert(o.stream == PUBLISH_STREAM_NONE);
  assert(o.reliable == PUBLISH_RELIABLE_EVENT);
  o = at(0u, true, false, false, o.spare_served);
  assert(o.stream == PUBLISH_STREAM_LOW_STATE);
  assert(o.reliable == PUBLISH_RELIABLE_MOTOR_STATUS);
}
}  // namespace

int main() {
  test_stream_rates_over_five_seconds();
  test_exact_stream_slots();
  test_motor_status_cadence_and_priority();
  test_spare_requires_pending();
  test_round_robin_order();
  test_single_pending_is_never_starved();
  test_fair_rotation_over_900_ticks();
  test_status_tick_preserves_rotation_position();
  test_pending_waits_at_most_one_status_tick();
  test_stream_and_reliable_are_independent();
  test_200_tick_period_and_uint32_wrap();
  std::printf("test_publish_schedule: ok\n");
  return 0;
}
