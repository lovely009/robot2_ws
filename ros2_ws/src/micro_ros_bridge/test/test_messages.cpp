// 消息级编解码：整条受理或整条拒绝的口径必须与固件旧契约逐字一致。
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "micro_ros_bridge/messages.hpp"

using can_bridge_msgs::msg::LowCommand;
using can_bridge_msgs::msg::LowState;
using can_bridge_msgs::msg::MotorCommandArray;
using can_bridge_msgs::msg::MotorCommandTarget;
using can_bridge_msgs::msg::MotorMotionStateArray;

namespace {

MotorCommandTarget target(uint16_t id, double pos, double vel, double eff)
{
  MotorCommandTarget t;
  t.motor_id = id;
  t.position = pos;
  t.velocity = vel;
  t.effort = eff;
  return t;
}

}  // namespace

TEST(EncodeLowCommand, RoundTripThroughQuantization) {
  MotorCommandArray in;
  in.count = 2;
  in.lease_id = 1800000000000000001ULL;
  in.targets.push_back(target(0, 0.5, -1.0, 2.0));
  in.targets.push_back(target(3, -0.25, 0.0, 0.0));

  LowCommand out;
  const auto r = micro_ros_bridge::encode_low_command(in, out);
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_EQ(out.motor_ids.size(), 2u);
  EXPECT_EQ(out.motor_ids[0], 0);
  EXPECT_EQ(out.motor_ids[1], 3);
  EXPECT_NEAR(micro_ros_bridge::dequantize(out.position_q[0],
    micro_ros_bridge::contract::kScalePosition), 0.5, 1e-3);
  EXPECT_EQ(out.count, 2);
  EXPECT_EQ(out.lease_id, in.lease_id);
}

// 整条拒绝：任何一条不合法，一个字节都不出去（沿用旧口径）。
TEST(EncodeLowCommand, RejectsWholeMessage) {
  LowCommand out;

  MotorCommandArray bad_id;
  bad_id.count = 1;
  bad_id.targets.push_back(target(65, 0.0, 0.0, 0.0));
  EXPECT_FALSE(micro_ros_bridge::encode_low_command(bad_id, out).ok);

  MotorCommandArray dup;
  dup.count = 2;
  dup.targets.push_back(target(1, 0.0, 0.0, 0.0));
  dup.targets.push_back(target(1, 0.0, 0.0, 0.0));
  EXPECT_FALSE(micro_ros_bridge::encode_low_command(dup, out).ok);

  MotorCommandArray nan;
  nan.count = 1;
  nan.targets.push_back(target(0, std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0));
  EXPECT_FALSE(micro_ros_bridge::encode_low_command(nan, out).ok);

  MotorCommandArray mismatch;
  mismatch.count = 2;
  mismatch.targets.push_back(target(0, 0.0, 0.0, 0.0));
  EXPECT_FALSE(micro_ros_bridge::encode_low_command(mismatch, out).ok);
}

TEST(EncodeLowCommand, CountsSaturatedChannels) {
  MotorCommandArray in;
  in.count = 1;
  in.targets.push_back(target(0, 1000.0, 0.0, 0.0));
  LowCommand out;
  const auto r = micro_ros_bridge::encode_low_command(in, out);
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(r.saturated, 1u);
  EXPECT_EQ(out.position_q[0], 32767);
}

TEST(DecodeLowState, SentinelBecomesNaN) {
  LowState in;
  in.seq = 7;
  in.count = 1;
  in.motor_ids = {5};
  in.position_q = {100};
  in.velocity_q = {micro_ros_bridge::contract::kSentinelMissing};
  in.effort_q = {0};
  in.state_age_ms = {12};
  in.target_source = {1};
  in.schema_version = micro_ros_bridge::contract::kSchemaVersion;
  in.session_id = 1;
  in.tick_ms = 123;
  in.chunk_count = 1;
  in.total_count = 1;

  MotorMotionStateArray out;
  const auto r = micro_ros_bridge::decode_low_state(in, out);
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(out.seq, 7u);  // 丢帧判据必须透到 facade，不能丢
  EXPECT_EQ(out.session_id, in.session_id);
  EXPECT_EQ(out.tick_ms, in.tick_ms);
  ASSERT_EQ(out.states.size(), 1u);
  EXPECT_EQ(out.states[0].motor_id, 5);
  EXPECT_TRUE(std::isnan(out.states[0].velocity));
  EXPECT_FALSE(std::isnan(out.states[0].position));
  EXPECT_EQ(out.states[0].state_age_ms, 12u);
  EXPECT_EQ(out.states[0].target_source, 1u);
}

TEST(DecodeLowState, RejectsOutOfRangeMotorIdWithoutPartialOutput) {
  LowState in;
  in.count = 2;
  in.motor_ids = {5, 65};
  in.position_q = {100, 200};
  in.velocity_q = {0, 0};
  in.effort_q = {0, 0};
  in.state_age_ms = {1, 1};
  in.target_source = {0, 0};
  in.schema_version = micro_ros_bridge::contract::kSchemaVersion;
  in.session_id = 1;
  in.chunk_count = 1;
  in.total_count = 2;

  MotorMotionStateArray out;
  const auto r = micro_ros_bridge::decode_low_state(in, out);
  EXPECT_FALSE(r.ok);
  EXPECT_TRUE(out.states.empty());
}

TEST(EncodeConfig, RetiredParameterWritesAreRejected) {
  can_bridge_msgs::msg::ParamSetArray in;
  in.request_id = 7;
  in.count = 2;
  can_bridge_msgs::msg::ParamSetTarget lo;
  lo.motor_id = 1;
  lo.field = can_bridge_msgs::msg::ParamSetTarget::FIELD_POS_MIN;
  lo.value = -0.5;
  can_bridge_msgs::msg::ParamSetTarget hi;
  hi.motor_id = 1;
  hi.field = can_bridge_msgs::msg::ParamSetTarget::FIELD_POS_MAX;
  hi.value = 0.5;
  in.targets = {lo, hi};

  can_bridge_msgs::msg::ConfigRequest out;
  const auto r = micro_ros_bridge::encode_config(in, out);
  EXPECT_FALSE(r.ok);
  EXPECT_TRUE(out.targets.empty());
}

TEST(EncodeConfig, ParamSetRejectsUnpairedLimit) {
  can_bridge_msgs::msg::ParamSetArray in;
  in.count = 1;
  can_bridge_msgs::msg::ParamSetTarget lo;
  lo.motor_id = 1;
  lo.field = can_bridge_msgs::msg::ParamSetTarget::FIELD_POS_MIN;
  in.targets = {lo};

  can_bridge_msgs::msg::ConfigRequest out;
  EXPECT_FALSE(micro_ros_bridge::encode_config(in, out).ok);
}

TEST(EncodeConfig, ParamSetRejectsDuplicateAndCountMismatch) {
  can_bridge_msgs::msg::ConfigRequest out;

  can_bridge_msgs::msg::ParamSetArray dup;
  dup.count = 2;
  can_bridge_msgs::msg::ParamSetTarget a;
  a.motor_id = 2;
  a.field = can_bridge_msgs::msg::ParamSetTarget::FIELD_VEL_LIMIT;
  dup.targets = {a, a};
  EXPECT_FALSE(micro_ros_bridge::encode_config(dup, out).ok);

  // count 声明 17 但只填 16 条：命中长度守卫（序列上界 16，装不下第 17 条）。
  can_bridge_msgs::msg::ParamSetArray many;
  many.count = 17;
  for (int i = 0; i < 16; ++i) {
    can_bridge_msgs::msg::ParamSetTarget t;
    t.motor_id = static_cast<uint16_t>(i);
    t.field = can_bridge_msgs::msg::ParamSetTarget::FIELD_VEL_LIMIT;
    many.targets.push_back(t);
  }
  EXPECT_FALSE(micro_ros_bridge::encode_config(many, out).ok);
}

TEST(EncodeConfig, SystemCommandOverloadFillsCommandGroup) {
  can_bridge_msgs::msg::SystemCommand in;
  in.command = can_bridge_msgs::msg::SystemCommand::COMMAND_REFRESH;
  in.motor_id = 3;

  can_bridge_msgs::msg::ConfigRequest out;
  const auto r = micro_ros_bridge::encode_config(in, out);
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(out.kind, can_bridge_msgs::msg::ConfigRequest::CONFIG_KIND_SYSTEM_COMMAND);
  EXPECT_EQ(out.command, can_bridge_msgs::msg::SystemCommand::COMMAND_REFRESH);
  EXPECT_EQ(out.motor_id, 3);
  EXPECT_TRUE(out.targets.empty());
  EXPECT_EQ(out.count, 0);
}

TEST(EncodeConfig, RetiredSaveIsRejected) {
  can_bridge_msgs::msg::SystemCommand in;
  in.command = can_bridge_msgs::msg::SystemCommand::COMMAND_SAVE;
  in.motor_id = 3;
  can_bridge_msgs::msg::ConfigRequest out;
  in.motor_id = can_bridge_msgs::msg::SystemCommand::MOTOR_ID_NONE;
  EXPECT_FALSE(micro_ros_bridge::encode_config(in, out).ok);
}

TEST(EncodeConfig, SetZeroAcceptsOnlySingleMotorIdsOneThrough64) {
  can_bridge_msgs::msg::SystemCommand in;
  in.command = can_bridge_msgs::msg::SystemCommand::COMMAND_SET_ZERO;
  in.request_id = 7; in.sync_time_ns = 123;
  can_bridge_msgs::msg::ConfigRequest out;
  for (const uint16_t id : {1, 64}) {
    in.motor_id = id;
    ASSERT_TRUE(micro_ros_bridge::encode_config(in, out).ok);
    EXPECT_EQ(out.command, in.command); EXPECT_EQ(out.motor_id, id);
    EXPECT_EQ(out.sync_time_ns, 123u); EXPECT_EQ(out.request_id, 7u);
    EXPECT_TRUE(out.targets.empty()); EXPECT_EQ(out.count, 0u);
  }
  for (const uint16_t id : {0, 65, 65535}) {
    in.motor_id = id;
    EXPECT_FALSE(micro_ros_bridge::encode_config(in, out).ok);
  }
}

TEST(DecodeMotorStatus, RestoresFacadeFields) {
  can_bridge_msgs::msg::MotorStatus in;
  in.sync_time_ns = 42;
  in.session_id = 1800000000000ULL;
  in.sequence = 123;
  in.tick_ms = 456;
  in.count = 1;
  in.motor_ids = {9};
  in.state_age_ms = {0xFFFF};  // 从未收到
  in.target_source = {can_bridge_msgs::msg::MotorMotionStateTarget::TARGET_SOURCE_TRAJECTORY};
  in.lifecycle = {can_bridge_msgs::msg::MotorStatusTarget::LIFECYCLE_ACTIVE};
  in.unified_fault_code = {0};
  in.vendor_fault_code = {0};
  in.protection_flags = {0};
  in.limit_flags = {0};
  in.temperature_dc = {253};   // 25.3 °C
  in.current_ma = {100000};    // 100 A，超过旧 int16 量程
  in.voltage_mv = {100000};    // 100 V，超过旧 uint16 量程
  in.drive_mode = {1};
  in.status_flags = {0x1F};    // 五个位全置
  in.bus_index = {0};

  can_bridge_msgs::msg::MotorStatusArray statuses;
  can_bridge_msgs::msg::MotorTrajectoryState traj;
  can_bridge_msgs::msg::SystemStatus sys;
  can_bridge_msgs::msg::MotorDiagnostics diag;
  std::vector<micro_ros_bridge::MotorFreshness> fresh;
  const auto r = micro_ros_bridge::decode_motor_status(in, statuses, traj, sys, diag, fresh);
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_EQ(statuses.statuses.size(), 1u);
  const auto & s = statuses.statuses[0];
  EXPECT_EQ(s.motor_id, 9);
  EXPECT_EQ(s.lifecycle, can_bridge_msgs::msg::MotorStatusTarget::LIFECYCLE_ACTIVE);
  // 新鲜度走缓存，不进 MotorStatusTarget（spec §4.7：旧消息字段级沿用现定义）。
  ASSERT_EQ(fresh.size(), 65u);
  EXPECT_EQ(fresh[9].state_age_ms, 0xFFFFFFFFu);  // 哨兵还原成"从未收到"
  EXPECT_EQ(fresh[9].target_source,
    can_bridge_msgs::msg::MotorMotionStateTarget::TARGET_SOURCE_TRAJECTORY);
  EXPECT_EQ(fresh[0].state_age_ms, 0xFFFFFFFFu);  // 本帧未列出的电机读作从未收到
  EXPECT_NEAR(s.temperature, 25.3, 1e-6);
  EXPECT_NEAR(s.current, 100.0, 1e-9);
  EXPECT_NEAR(s.voltage, 100.0, 1e-9);
  EXPECT_TRUE(s.online);
  EXPECT_TRUE(s.device_protect_enabled);
  EXPECT_EQ(s.device_direction, 1u);
  EXPECT_EQ(statuses.sync_time_ns, 42u);
  EXPECT_EQ(traj.session_id, in.session_id);
  EXPECT_EQ(traj.sequence, in.sequence);
  EXPECT_EQ(traj.tick_ms, in.tick_ms);
  const auto before = fresh;
  in.count = 2;
  EXPECT_FALSE(micro_ros_bridge::decode_motor_status(in, statuses, traj, sys, diag, fresh).ok);
  ASSERT_EQ(fresh.size(), before.size());
  EXPECT_EQ(fresh[9].state_age_ms, before[9].state_age_ms);
  EXPECT_EQ(statuses.statuses.size(), 1u);
  in.count = 1;
  in.lifecycle = {can_bridge_msgs::msg::MotorStatusTarget::LIFECYCLE_FAULT};
  ASSERT_TRUE(micro_ros_bridge::decode_motor_status(in, statuses, traj, sys, diag, fresh).ok);
  EXPECT_EQ(statuses.statuses[0].lifecycle,
    can_bridge_msgs::msg::MotorStatusTarget::LIFECYCLE_FAULT);
  in.lifecycle = {can_bridge_msgs::msg::MotorStatusTarget::LIFECYCLE_OFFLINE};
  ASSERT_TRUE(micro_ros_bridge::decode_motor_status(in, statuses, traj, sys, diag, fresh).ok);
  EXPECT_EQ(statuses.statuses[0].lifecycle,
    can_bridge_msgs::msg::MotorStatusTarget::LIFECYCLE_OFFLINE);
}

TEST(DecodeMotorStatus, MissingElectricalReadsAsNaN) {
  can_bridge_msgs::msg::MotorStatus in;
  in.count = 1;
  in.motor_ids = {0};
  in.state_age_ms = {100};  // 真实年龄，不是哨兵
  in.target_source = {0};
  in.lifecycle = {0};
  in.unified_fault_code = {0};
  in.vendor_fault_code = {0};
  in.protection_flags = {0};
  in.limit_flags = {0};
  in.temperature_dc = {micro_ros_bridge::contract::kSentinelMissing};
  in.current_ma = {std::numeric_limits<int32_t>::min()};
  in.voltage_mv = {std::numeric_limits<uint32_t>::max()};
  in.drive_mode = {0};
  in.status_flags = {0};
  in.bus_index = {0};

  can_bridge_msgs::msg::MotorStatusArray statuses;
  can_bridge_msgs::msg::MotorTrajectoryState traj;
  can_bridge_msgs::msg::SystemStatus sys;
  can_bridge_msgs::msg::MotorDiagnostics diag;
  std::vector<micro_ros_bridge::MotorFreshness> fresh;
  const auto r = micro_ros_bridge::decode_motor_status(in, statuses, traj, sys, diag, fresh);
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_TRUE(std::isnan(statuses.statuses[0].temperature));
  EXPECT_TRUE(std::isnan(statuses.statuses[0].current));
  EXPECT_TRUE(std::isnan(statuses.statuses[0].voltage));
  EXPECT_EQ(fresh[0].state_age_ms, 100u);  // 真实年龄，不是哨兵
}

TEST(DecodeMotorStatus, RejectsRaggedArrays) {
  can_bridge_msgs::msg::MotorStatus in;
  in.count = 2;
  in.motor_ids = {0, 1};
  in.state_age_ms = {0};  // 长度与 count 不符

  can_bridge_msgs::msg::MotorStatusArray statuses;
  can_bridge_msgs::msg::MotorTrajectoryState traj;
  can_bridge_msgs::msg::SystemStatus sys;
  can_bridge_msgs::msg::MotorDiagnostics diag;
  std::vector<micro_ros_bridge::MotorFreshness> fresh;
  EXPECT_FALSE(micro_ros_bridge::decode_motor_status(in, statuses, traj, sys, diag, fresh).ok);
}

// motor_id 是 uint8（0..255），freshness 缓存只有 64：越界 id 若不拦会越界写。
TEST(DecodeMotorStatus, RejectsOutOfRangeMotorId) {
  can_bridge_msgs::msg::MotorStatus in;
  in.count = 1;
  in.motor_ids = {200};  // > 63：越界，必须整条拒收（不 clamp）
  in.state_age_ms = {0};
  in.target_source = {0};
  in.lifecycle = {0};
  in.unified_fault_code = {0};
  in.vendor_fault_code = {0};
  in.protection_flags = {0};
  in.limit_flags = {0};
  in.temperature_dc = {0};
  in.current_ma = {0};
  in.voltage_mv = {0};
  in.drive_mode = {0};
  in.status_flags = {0};
  in.bus_index = {0};

  can_bridge_msgs::msg::MotorStatusArray statuses;
  can_bridge_msgs::msg::MotorTrajectoryState traj;
  can_bridge_msgs::msg::SystemStatus sys;
  can_bridge_msgs::msg::MotorDiagnostics diag;
  std::vector<micro_ros_bridge::MotorFreshness> fresh;
  EXPECT_FALSE(micro_ros_bridge::decode_motor_status(in, statuses, traj, sys, diag, fresh).ok);
}
