#include "motor_hardware_interface/canfd_logic.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace motor_hardware_interface {
namespace {

using Clock = std::chrono::steady_clock;

// ---- map_motors_to_joints ----

TEST(MapMotorsToJoints, MapsByNameWhenMotorOrderDiffersFromJointOrder) {
  const std::vector<std::string> joint_names{"j0", "j1", "j2"};
  const std::vector<std::string> motor_names{"m0", "m1", "m2"};
  const std::vector<std::string> motor_joint_names{"j2", "j0", "j1"};
  std::vector<std::size_t> mapping;
  std::string error;

  ASSERT_TRUE(map_motors_to_joints(joint_names, motor_names, motor_joint_names,
                                   mapping, error));
  ASSERT_EQ(mapping.size(), 3U);
  EXPECT_EQ(mapping[0], 2U);
  EXPECT_EQ(mapping[1], 0U);
  EXPECT_EQ(mapping[2], 1U);
}

TEST(MapMotorsToJoints, RejectsUnknownJoint) {
  const std::vector<std::string> joint_names{"j0", "j1", "j2"};
  const std::vector<std::string> motor_names{"m0", "m1", "m2"};
  const std::vector<std::string> motor_joint_names{"j0", "missing", "j2"};
  std::vector<std::size_t> mapping;
  std::string error;

  EXPECT_FALSE(map_motors_to_joints(joint_names, motor_names, motor_joint_names,
                                    mapping, error));
  EXPECT_NE(error.find("missing"), std::string::npos);
}

TEST(MapMotorsToJoints, RejectsDuplicateJointNames) {
  const std::vector<std::string> joint_names{"j0", "j0", "j1"};
  const std::vector<std::string> motor_names{"m0", "m1", "m2"};
  const std::vector<std::string> motor_joint_names{"j0", "j1", "j2"};
  std::vector<std::size_t> mapping;
  std::string error;

  EXPECT_FALSE(map_motors_to_joints(joint_names, motor_names, motor_joint_names,
                                    mapping, error));
  EXPECT_NE(error.find("duplicate"), std::string::npos);
}

TEST(MapMotorsToJoints, RejectsCountMismatch) {
  const std::vector<std::string> joint_names{"j0", "j1", "j2"};
  const std::vector<std::string> motor_names{"m0", "m1"};
  const std::vector<std::string> motor_joint_names{"j0", "j1"};
  std::vector<std::size_t> mapping;
  std::string error;

  EXPECT_FALSE(map_motors_to_joints(joint_names, motor_names, motor_joint_names,
                                    mapping, error));
  EXPECT_NE(error.find("exposes"), std::string::npos);
}

TEST(MapMotorsToJoints, RejectsUnmappedJoint) {
  const std::vector<std::string> joint_names{"j0", "j1"};
  const std::vector<std::string> motor_names{"m0", "m1"};
  // 两台电机都指向 j0，j1 没有被任何电机覆盖。
  const std::vector<std::string> motor_joint_names{"j0", "j0"};
  std::vector<std::size_t> mapping;
  std::string error;

  EXPECT_FALSE(map_motors_to_joints(joint_names, motor_names, motor_joint_names,
                                    mapping, error));
  EXPECT_NE(error.find("exactly one"), std::string::npos);
}

TEST(MapMotorsToJoints, RejectsMotorListSizeMismatch) {
  const std::vector<std::string> joint_names{"j0", "j1"};
  const std::vector<std::string> motor_names{"m0", "m1"};
  const std::vector<std::string> motor_joint_names{"j0", "j1", "j2"};
  std::vector<std::size_t> mapping;
  std::string error;

  EXPECT_FALSE(map_motors_to_joints(joint_names, motor_names, motor_joint_names,
                                    mapping, error));
  EXPECT_NE(error.find("different sizes"), std::string::npos);
}

TEST(MapMotorsToJoints, EmptyInputsProduceEmptyMapping) {
  const std::vector<std::string> joint_names{};
  const std::vector<std::string> motor_names{};
  const std::vector<std::string> motor_joint_names{};
  std::vector<std::size_t> mapping;
  std::string error;

  EXPECT_TRUE(map_motors_to_joints(joint_names, motor_names, motor_joint_names,
                                   mapping, error));
  EXPECT_TRUE(mapping.empty());
}

// ---- first_stale_feedback ----

TEST(FirstStaleFeedback, ReturnsNposWhenAllFresh) {
  const auto now = Clock::now();
  const std::vector<Clock::time_point> last{
      now - std::chrono::milliseconds(50), now - std::chrono::milliseconds(10)};
  EXPECT_EQ(first_stale_feedback(last, now, 0.1), 2U);
}

TEST(FirstStaleFeedback, ReturnsFirstStaleIndex) {
  const auto now = Clock::now();
  const std::vector<Clock::time_point> last{
      now - std::chrono::milliseconds(50), now - std::chrono::milliseconds(200),
      now - std::chrono::milliseconds(300)};
  EXPECT_EQ(first_stale_feedback(last, now, 0.1), 1U);
}

TEST(FirstStaleFeedback, ExactlyAtTimeoutIsNotStale) {
  const auto now = Clock::now();
  const std::vector<Clock::time_point> last{now - std::chrono::milliseconds(100)};
  EXPECT_EQ(first_stale_feedback(last, now, 0.1), 1U);
}

TEST(FirstStaleFeedback, EmptyListReturnsNpos) {
  const std::vector<Clock::time_point> last{};
  EXPECT_EQ(first_stale_feedback(last, Clock::now(), 0.1), 0U);
}

// ---- should_continue_waiting ----

TEST(ShouldContinueWaiting, StopsWhenAllObserved) {
  const auto now = Clock::now();
  const auto deadline = now + std::chrono::milliseconds(100);
  const std::vector<bool> observed{true, true};
  EXPECT_FALSE(should_continue_waiting(observed, now, deadline));
}

TEST(ShouldContinueWaiting, StopsAfterDeadline) {
  const auto now = Clock::now();
  const auto deadline = now - std::chrono::milliseconds(1);
  const std::vector<bool> observed{false, true};
  EXPECT_FALSE(should_continue_waiting(observed, now, deadline));
}

TEST(ShouldContinueWaiting, ContinuesWhileMissingFeedbackBeforeDeadline) {
  const auto now = Clock::now();
  const auto deadline = now + std::chrono::milliseconds(100);
  const std::vector<bool> observed{true, false};
  EXPECT_TRUE(should_continue_waiting(observed, now, deadline));
}

TEST(ShouldContinueWaiting, EmptyObservationIsSatisfied) {
  const auto now = Clock::now();
  const auto deadline = now + std::chrono::milliseconds(100);
  const std::vector<bool> observed{};
  EXPECT_FALSE(should_continue_waiting(observed, now, deadline));
}

// ---- take_observed_positions_as_initial_targets ----

TEST(TakeObservedPositions, CopiesFinitePositionsIntoCommands) {
  const std::vector<std::string> joint_names{"a", "b"};
  const std::vector<bool> observed{true, true};
  const std::vector<double> positions{1.0, -2.5};
  std::vector<double> commands{0.0, 0.0};
  std::string error;

  ASSERT_TRUE(take_observed_positions_as_initial_targets(
      joint_names, observed, positions, commands, error));
  EXPECT_EQ(commands[0], 1.0);
  EXPECT_EQ(commands[1], -2.5);
}

TEST(TakeObservedPositions, RejectsUnobservedJoint) {
  const std::vector<std::string> joint_names{"a", "b"};
  const std::vector<bool> observed{true, false};
  const std::vector<double> positions{1.0, 2.0};
  std::vector<double> commands{0.0, 0.0};
  std::string error;

  EXPECT_FALSE(take_observed_positions_as_initial_targets(
      joint_names, observed, positions, commands, error));
  EXPECT_NE(error.find("b"), std::string::npos);
}

TEST(TakeObservedPositions, RejectsNonFinitePosition) {
  const std::vector<std::string> joint_names{"a", "b"};
  const std::vector<bool> observed{true, true};
  const std::vector<double> positions{1.0, std::nan("")};
  std::vector<double> commands{0.0, 0.0};
  std::string error;

  EXPECT_FALSE(take_observed_positions_as_initial_targets(
      joint_names, observed, positions, commands, error));
  EXPECT_NE(error.find("b"), std::string::npos);
}

TEST(TakeObservedPositions, RejectsSizeMismatch) {
  const std::vector<std::string> joint_names{"a", "b"};
  const std::vector<bool> observed{true};
  const std::vector<double> positions{1.0, 2.0};
  std::vector<double> commands{0.0, 0.0};
  std::string error;

  EXPECT_FALSE(take_observed_positions_as_initial_targets(
      joint_names, observed, positions, commands, error));
  EXPECT_NE(error.find("different sizes"), std::string::npos);
}

// ---- all_feedback_received ----

TEST(AllFeedbackReceived, TrueWhenEveryJointObserved) {
  const std::vector<bool> observed{true, true, true};
  EXPECT_TRUE(all_feedback_received(observed));
}

TEST(AllFeedbackReceived, FalseWhenAnyJointMissing) {
  const std::vector<bool> observed{true, false, true};
  EXPECT_FALSE(all_feedback_received(observed));
}

TEST(AllFeedbackReceived, EmptyObservationIsComplete) {
  const std::vector<bool> observed{};
  EXPECT_TRUE(all_feedback_received(observed));
}

// ---- missing_joint_names ----

TEST(MissingJointNames, ReturnsNamesOfUnobservedJoints) {
  const std::vector<std::string> joint_names{"a", "b", "c"};
  const std::vector<bool> observed{true, false, true};
  const auto missing = missing_joint_names(joint_names, observed);
  ASSERT_EQ(missing.size(), 1U);
  EXPECT_EQ(missing[0], "b");
}

TEST(MissingJointNames, ReturnsAllWhenNoneObserved) {
  const std::vector<std::string> joint_names{"a", "b"};
  const std::vector<bool> observed{false, false};
  EXPECT_EQ(missing_joint_names(joint_names, observed),
            (std::vector<std::string>{"a", "b"}));
}

TEST(MissingJointNames, EmptyListsProduceEmptyResult) {
  const std::vector<std::string> joint_names{};
  const std::vector<bool> observed{};
  EXPECT_TRUE(missing_joint_names(joint_names, observed).empty());
}

TEST(MissingJointNames, SizeMismatchUsesShorterList) {
  const std::vector<std::string> joint_names{"a", "b", "c"};
  const std::vector<bool> observed{true, false};
  const auto missing = missing_joint_names(joint_names, observed);
  ASSERT_EQ(missing.size(), 1U);
  EXPECT_EQ(missing[0], "b");
}

} // namespace
} // namespace motor_hardware_interface
