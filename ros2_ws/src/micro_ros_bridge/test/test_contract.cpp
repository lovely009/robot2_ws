// 契约测试：常量值、枚举与旧 facade 消息的一致性。
// 话题名与量程的唯一来源是 WireContract.msg；这里只钉「取出来的值是什么」，
// 因为 bridge 与固件都靠这些值对齐。
#include <gtest/gtest.h>

#include <can_bridge_msgs/msg/motor_motion_state_target.hpp>
#include <can_bridge_msgs/msg/motor_status_target.hpp>
#include <can_bridge_msgs/msg/system_command.hpp>
#include <can_bridge_msgs/msg/wire_contract.hpp>

#include "micro_ros_bridge/codec.hpp"

// 前缀同时编码方向：do = 主机→固件，up = 固件→主机。按前缀逐条钉住，
// 防止某条话题被挪到反方向（两端都有实现时不会报错，只会永远收不到）。
TEST(Contract, TopicNamesCarryDirectionPrefix) {
  namespace topic = micro_ros_bridge::contract::topic;
  EXPECT_EQ(topic::kLowCmd, "/micro_ros_do/low_cmd");
  for (const std::string & name : {topic::kLowCmd, topic::kPolicy, topic::kTrajectory,
      topic::kConfig, topic::kControlLease}) {
    EXPECT_EQ(std::string(name).rfind("/micro_ros_do/", 0), 0u) << name;
  }
  for (const std::string & name : {topic::kLowState, topic::kMotorStatus, topic::kTopology,
      topic::kEvents, topic::kDeviceParam, topic::kImuData, topic::kImuTemp})
  {
    EXPECT_EQ(std::string(name).rfind("/micro_ros_up/", 0), 0u) << name;
  }
}

TEST(Contract, ScalesAndSentinel) {
  EXPECT_DOUBLE_EQ(micro_ros_bridge::contract::kScalePosition, 12.5);
  EXPECT_DOUBLE_EQ(micro_ros_bridge::contract::kScaleVelocity, 64.0);
  EXPECT_DOUBLE_EQ(micro_ros_bridge::contract::kScaleEffort, 64.0);
  EXPECT_EQ(micro_ros_bridge::contract::kSentinelMissing, -32768);
  EXPECT_EQ(micro_ros_bridge::contract::kMotorCountMax, 64);
}

TEST(Contract, CompactLeaseIdentityBelongsToOnlyOneSession) {
  constexpr uint64_t session = 1800000000000ULL;
  constexpr uint64_t base = session * 1000000ULL;
  EXPECT_TRUE(micro_ros_bridge::lease_belongs_to_session(base + 1, session));
  EXPECT_TRUE(micro_ros_bridge::lease_belongs_to_session(base + 999999, session));
  EXPECT_FALSE(micro_ros_bridge::lease_belongs_to_session(base, session));
  EXPECT_FALSE(micro_ros_bridge::lease_belongs_to_session(base + 1, session + 1));
  EXPECT_FALSE(micro_ros_bridge::lease_belongs_to_session(base + 1000000, session));
  EXPECT_FALSE(micro_ros_bridge::lease_belongs_to_session(1, 0));
}

// 旧 facade 消息保留着同一套枚举取值。两处若漂移，页面与固件对同一台电机的
// 解释就会不一致——钉死。
TEST(Contract, LegacyEnumsStillMatch) {
  EXPECT_EQ(can_bridge_msgs::msg::MotorMotionStateTarget::TARGET_SOURCE_TRAJECTORY, 2);
  EXPECT_EQ(can_bridge_msgs::msg::MotorStatusTarget::LIFECYCLE_ACTIVE, 4);
  EXPECT_EQ(can_bridge_msgs::msg::SystemCommand::MOTOR_ID_NONE, 65535);
}
