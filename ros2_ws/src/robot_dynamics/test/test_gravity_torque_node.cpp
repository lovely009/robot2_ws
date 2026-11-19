#include "robot_dynamics/ros/gravity_torque_node.hpp"

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace {

using namespace std::chrono_literals;

// 与 test_arm_model.cpp 同一份 fixture 闭式解（planar_arm.urdf 顶部有推导）。
constexpr double kGravity = 9.81;

std::string fixture_path() {
  return std::string(TEST_SOURCE_DIR) + "/test/fixtures/planar_arm.urdf";
}

std::vector<double> closed_form(double q1, double q2, double q3) {
  const double c1 = std::cos(q1);
  const double c12 = std::cos(q1 + q2);
  const double c123 = std::cos(q1 + q2 + q3);
  return {-kGravity *
              (1.0 * 0.2 * c1 + 0.5 * (0.4 * c1 + 0.15 * c12) +
               0.25 * (0.4 * c1 + 0.3 * c12 + 0.1 * c123)),
          -kGravity * (0.5 * 0.15 * c12 + 0.25 * (0.3 * c12 + 0.1 * c123)),
          -kGravity * 0.25 * 0.1 * c123};
}

void ensure_rclcpp_initialized() {
  if (!rclcpp::ok()) {
    int argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);
  }
}

sensor_msgs::msg::JointState joint_state(
    const std::vector<std::string> &names, const std::vector<double> &positions) {
  sensor_msgs::msg::JointState message;
  message.name = names;
  message.position = positions;
  return message;
}

void expect_torque_near(const std::vector<double> &actual,
                        const std::vector<double> &expected) {
  ASSERT_EQ(actual.size(), expected.size());
  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_NEAR(actual[i], expected[i], 1e-12) << "关节下标 " << i;
  }
}

// 等到发现订阅者后再发，避免把消息丢在发现完成之前（否则要重发、队列里会留旧帧）。
void wait_for_subscription(rclcpp::executors::SingleThreadedExecutor &executor,
                           const rclcpp::Publisher<sensor_msgs::msg::JointState>
                               ::SharedPtr &publisher) {
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (publisher->get_subscription_count() == 0 &&
         std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_GT(publisher->get_subscription_count(), 0U);
}

void spin_until_received(
    rclcpp::executors::SingleThreadedExecutor &executor,
    const std::vector<sensor_msgs::msg::JointState> &received) {
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (received.empty() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(1ms);
  }
}

}  // namespace

// README 里对外的参数契约：名字与覆盖行为。
TEST(GravityTorqueNode, ReadsDocumentedParametersAndFailsFast) {
  ensure_rclcpp_initialized();
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("urdf_path", fixture_path()),
       rclcpp::Parameter("joint_states_topic", "override_states"),
       rclcpp::Parameter("gravity_torque_topic", "override_torque")});
  {
    auto node =
        std::make_shared<robot_dynamics::GravityTorqueNode>(options);
    EXPECT_EQ(node->get_parameter("urdf_path").as_string(), fixture_path());
    EXPECT_EQ(node->get_parameter("joint_states_topic").as_string(),
              "override_states");
    EXPECT_EQ(node->get_parameter("gravity_torque_topic").as_string(),
              "override_torque");
  }
  // urdf_path 未给（默认空）即构造失败，不静默起一个算不了的节点。
  EXPECT_THROW(robot_dynamics::GravityTorqueNode(rclcpp::NodeOptions{}),
               std::invalid_argument);
  rclcpp::shutdown();
}

TEST(GravityTorqueNode, PublishesNameIndexedTorqueAndSurvivesMalformedFrame) {
  ensure_rclcpp_initialized();
  robot_dynamics::GravityTorqueNodeParameters parameters;
  parameters.urdf_path = fixture_path();
  parameters.joint_states_topic = "test_joint_states";
  parameters.gravity_torque_topic = "test_gravity_torque";

  auto node = std::make_shared<robot_dynamics::GravityTorqueNode>(
      rclcpp::NodeOptions{}, parameters);
  auto observer = std::make_shared<rclcpp::Node>("gravity_torque_node_test");

  std::vector<sensor_msgs::msg::JointState> received;
  auto subscription =
      observer->create_subscription<sensor_msgs::msg::JointState>(
          "test_gravity_torque", 10,
          [&received](const sensor_msgs::msg::JointState &message) {
            received.push_back(message);
          });
  auto publisher = observer->create_publisher<sensor_msgs::msg::JointState>(
      "test_joint_states", 10);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(observer);

  // 子集 + 乱序 + 模型外名字：joint_2 保持 neutral，gripper_finger 只被忽略。
  const sensor_msgs::msg::JointState subset = joint_state(
      {"gripper_finger", "joint_3", "joint_1"}, {9.9, 0.4, -0.2});
  wait_for_subscription(executor, publisher);
  publisher->publish(subset);
  spin_until_received(executor, received);
  ASSERT_FALSE(received.empty());

  const sensor_msgs::msg::JointState &first = received.back();
  EXPECT_EQ(first.name,
            (std::vector<std::string>{"joint_1", "joint_2", "joint_3"}));
  // ADR 0009：只填 name 与 effort。
  EXPECT_TRUE(first.position.empty());
  EXPECT_TRUE(first.velocity.empty());
  expect_torque_near(first.effort, closed_form(-0.2, 0.0, 0.4));

  // name/position 长度不符的帧：只告警不发布，也不让节点退出。
  received.clear();
  publisher->publish(joint_state({"joint_1", "joint_2"}, {0.1}));
  const auto malformed_deadline = std::chrono::steady_clock::now() + 200ms;
  while (std::chrono::steady_clock::now() < malformed_deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_TRUE(received.empty());
  EXPECT_TRUE(rclcpp::ok());

  // 畸形帧之后仍能正常出帧。
  const sensor_msgs::msg::JointState full =
      joint_state({"joint_1", "joint_2", "joint_3"}, {0.5, 0.0, 0.0});
  publisher->publish(full);
  spin_until_received(executor, received);
  ASSERT_FALSE(received.empty());
  expect_torque_near(received.back().effort, closed_form(0.5, 0.0, 0.0));

  static_cast<void>(subscription);
  executor.remove_node(observer);
  executor.remove_node(node);
  rclcpp::shutdown();
}
