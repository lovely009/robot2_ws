#include <gtest/gtest.h>
#include <chrono>
#include "micro_ros_bridge/messages.hpp"

using namespace std::chrono_literals;
using can_bridge_msgs::msg::LowState;
using can_bridge_msgs::msg::MotorMotionStateArray;
using micro_ros_bridge::LowStateAssembler;

namespace {
LowState chunk(uint64_t session, uint32_t seq, uint8_t index, uint8_t id)
{
  LowState m;
  m.schema_version = micro_ros_bridge::contract::kSchemaVersion;
  m.session_id = session;
  m.seq = seq;
  m.tick_ms = 100;
  m.chunk_index = index;
  m.chunk_count = 2;
  m.total_count = 2;
  m.count = 1;
  m.motor_ids = {id};
  m.position_q = {100};
  m.velocity_q = {0};
  m.effort_q = {0};
  m.state_age_ms = {7};
  m.target_source = {1};
  return m;
}
}

TEST(LowStateAssembler, MissingChunkTimesOutWithoutReusingOldData)
{
  LowStateAssembler assembler;
  MotorMotionStateArray out;
  const auto t = std::chrono::steady_clock::now();
  EXPECT_FALSE(assembler.accept(chunk(5, 1, 0, 1), 5, t, out));
  EXPECT_FALSE(assembler.accept(chunk(5, 1, 1, 2), 5, t + 61ms, out));
  EXPECT_TRUE(out.states.empty());
  EXPECT_FALSE(assembler.accept(chunk(5, 2, 0, 1), 5, t + 62ms, out));
  EXPECT_TRUE(assembler.accept(chunk(5, 2, 1, 2), 5, t + 72ms, out));
  ASSERT_EQ(out.states.size(), 2u);
  EXPECT_EQ(out.states[0].state_age_ms, 17u);
  EXPECT_EQ(out.states[1].state_age_ms, 7u);
}

TEST(LowStateAssembler, RejectsCrossSessionDuplicatesAndReplay)
{
  LowStateAssembler assembler;
  MotorMotionStateArray out;
  const auto t = std::chrono::steady_clock::now();
  EXPECT_FALSE(assembler.accept(chunk(5, 1, 0, 1), 5, t, out));
  EXPECT_FALSE(assembler.accept(chunk(4, 1, 1, 2), 5, t + 1ms, out));
  EXPECT_FALSE(assembler.accept(chunk(5, 1, 1, 1), 5, t + 2ms, out));
  EXPECT_TRUE(out.states.empty());
  EXPECT_FALSE(assembler.accept(chunk(5, 2, 0, 1), 5, t + 3ms, out));
  EXPECT_TRUE(assembler.accept(chunk(5, 2, 1, 2), 5, t + 4ms, out));
  EXPECT_FALSE(assembler.accept(chunk(5, 1, 0, 3), 5, t + 5ms, out));
}

TEST(LowStateAssembler, RejectsBadMetadataAndNeverPublishesPartialGroup)
{
  LowStateAssembler assembler;
  MotorMotionStateArray out;
  const auto t = std::chrono::steady_clock::now();
  auto bad = chunk(5, 1, 0, 1);
  bad.schema_version = 1;
  EXPECT_FALSE(assembler.accept(bad, 5, t, out));
  bad = chunk(5, 1, 0, 1);
  bad.target_source = {9};
  EXPECT_FALSE(assembler.accept(bad, 5, t, out));
  bad = chunk(5, 1, 0, 1);
  bad.chunk_count = 0;
  EXPECT_FALSE(assembler.accept(bad, 5, t, out));
  EXPECT_FALSE(assembler.accept(chunk(5, 1, 0, 1), 5, t, out));
  auto inconsistent = chunk(5, 1, 1, 2);
  inconsistent.tick_ms = 101;
  EXPECT_FALSE(assembler.accept(inconsistent, 5, t + 1ms, out));
  EXPECT_FALSE(assembler.accept(chunk(5, 1, 1, 2), 5, t + 2ms, out));
  EXPECT_TRUE(out.states.empty());
}

TEST(LowStateAssembler, AddsSourceDelayAndHostWaitToMeasurementAge)
{
  LowStateAssembler assembler;
  MotorMotionStateArray out;
  const auto t = std::chrono::steady_clock::now();
  EXPECT_FALSE(assembler.accept(chunk(5, 1, 0, 1), 5, t, out, 30));
  EXPECT_TRUE(assembler.accept(chunk(5, 1, 1, 2), 5, t + 10ms, out, 40));
  ASSERT_EQ(out.states.size(), 2u);
  EXPECT_EQ(out.states[0].state_age_ms, 47u);
  EXPECT_EQ(out.states[1].state_age_ms, 47u);
}
