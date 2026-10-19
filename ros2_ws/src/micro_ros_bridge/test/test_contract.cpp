// 契约测试：常量值、枚举与旧 facade 消息的一致性。
// 话题名与量程的唯一来源是 WireContract.msg；这里只钉「取出来的值是什么」，
// 因为 bridge 与固件都靠这些值对齐。
#include <gtest/gtest.h>

#include <can_bridge_msgs/msg/motor_motion_state_target.hpp>
#include <can_bridge_msgs/msg/motor_status_target.hpp>
#include <can_bridge_msgs/msg/system_command.hpp>
#include <can_bridge_msgs/msg/wire_contract.hpp>

#include "micro_ros_bridge/codec.hpp"

TEST(Contract, TopicNamesAreAbsoluteAndUnderMicroRos) {
  using micro_ros_bridge::contract::topic::kLowCmd;
  EXPECT_EQ(kLowCmd, "/micro_ros/do/low_cmd");
  EXPECT_EQ(std::string(kLowCmd).rfind("/micro_ros/", 0), 0u);
}

TEST(Contract, ScalesAndSentinel) {
  EXPECT_DOUBLE_EQ(micro_ros_bridge::contract::kScalePosition, 12.5);
  EXPECT_DOUBLE_EQ(micro_ros_bridge::contract::kScaleVelocity, 64.0);
  EXPECT_DOUBLE_EQ(micro_ros_bridge::contract::kScaleEffort, 64.0);
  EXPECT_EQ(micro_ros_bridge::contract::kSentinelMissing, -32768);
  EXPECT_EQ(micro_ros_bridge::contract::kMotorCountMax, 64);
}

// 旧 facade 消息保留着同一套枚举取值。两处若漂移，页面与固件对同一台电机的
// 解释就会不一致——钉死。
TEST(Contract, LegacyEnumsStillMatch) {
  EXPECT_EQ(can_bridge_msgs::msg::MotorMotionStateTarget::TARGET_SOURCE_TRAJECTORY, 2);
  EXPECT_EQ(can_bridge_msgs::msg::MotorStatusTarget::LIFECYCLE_ACTIVE, 4);
  EXPECT_EQ(can_bridge_msgs::msg::SystemCommand::MOTOR_ID_NONE, 65535);
}
