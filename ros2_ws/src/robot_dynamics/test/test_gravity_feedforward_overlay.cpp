#include "robot_dynamics/ros/gravity_feedforward_overlay_node.hpp"

#include <gtest/gtest.h>

#include <can_bridge_msgs/msg/motor_command_array.hpp>
#include <can_bridge_msgs/msg/motor_command_target.hpp>
#include <can_bridge_msgs/msg/motor_trajectory.hpp>
#include <can_bridge_msgs/msg/motor_trajectory_point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>
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
    const std::vector<std::string> & names, const std::vector<double> & positions) {
  sensor_msgs::msg::JointState message;
  message.name = names;
  message.position = positions;
  return message;
}

can_bridge_msgs::msg::MotorCommandTarget target(std::uint16_t motor_id,
                                                float position, float velocity,
                                                float effort) {
  can_bridge_msgs::msg::MotorCommandTarget message;
  message.motor_id = motor_id;
  message.position = position;
  message.velocity = velocity;
  message.effort = effort;
  return message;
}

can_bridge_msgs::msg::MotorCommandArray command_array(
    std::uint32_t sequence,
    const std::vector<can_bridge_msgs::msg::MotorCommandTarget> & targets) {
  can_bridge_msgs::msg::MotorCommandArray message;
  message.sync_time_ns = 111222333444ULL;
  message.sequence = sequence;
  message.count = static_cast<std::uint8_t>(targets.size());
  message.targets.assign(targets.begin(), targets.end());
  return message;
}

can_bridge_msgs::msg::MotorTrajectoryPoint trajectory_point(
    std::uint32_t time_from_start_ms, std::uint16_t motor_id, float position,
    float effort) {
  can_bridge_msgs::msg::MotorTrajectoryPoint message;
  message.time_from_start_ms = time_from_start_ms;
  message.motor_id = motor_id;
  message.position = position;
  message.velocity = 0.0f;
  message.effort = effort;
  return message;
}

can_bridge_msgs::msg::MotorTrajectory trajectory(
    std::uint32_t trajectory_id, const std::vector<std::uint16_t> & target_ids,
    const std::vector<can_bridge_msgs::msg::MotorTrajectoryPoint> & points) {
  can_bridge_msgs::msg::MotorTrajectory message;
  message.sync_time_ns = 999888777666ULL;
  message.sequence = 42;
  message.trajectory_id = trajectory_id;
  message.segment_index = 0;
  message.segment_count = 1;
  message.count = static_cast<std::uint16_t>(points.size());
  message.target_count = static_cast<std::uint16_t>(target_ids.size());
  message.target_ids.assign(target_ids.begin(), target_ids.end());
  message.points.assign(points.begin(), points.end());
  return message;
}

// 收到的帧可能短于一帧（少一帧/空帧本身就是被测行为）：越界读是 UB，先查大小。
float effort_or_nan(const can_bridge_msgs::msg::MotorCommandArray & frame,
                    std::size_t index) {
  return index < frame.targets.size()
             ? frame.targets[index].effort
             : std::numeric_limits<float>::quiet_NaN();
}

bool command_frames_identical(
    const can_bridge_msgs::msg::MotorCommandArray & left,
    const can_bridge_msgs::msg::MotorCommandArray & right) {
  if (left.sync_time_ns != right.sync_time_ns ||
      left.sequence != right.sequence || left.count != right.count ||
      left.targets.size() != right.targets.size()) {
    return false;
  }
  for (std::size_t i = 0; i < left.targets.size(); ++i) {
    if (left.targets[i].motor_id != right.targets[i].motor_id ||
        left.targets[i].position != right.targets[i].position ||
        left.targets[i].velocity != right.targets[i].velocity ||
        left.targets[i].effort != right.targets[i].effort) {
      return false;
    }
  }
  return true;
}

bool trajectory_frames_identical(
    const can_bridge_msgs::msg::MotorTrajectory & left,
    const can_bridge_msgs::msg::MotorTrajectory & right) {
  if (left.sync_time_ns != right.sync_time_ns ||
      left.sequence != right.sequence ||
      left.trajectory_id != right.trajectory_id ||
      left.segment_index != right.segment_index ||
      left.segment_count != right.segment_count || left.count != right.count ||
      left.target_count != right.target_count ||
      left.target_ids != right.target_ids || left.points.size() != right.points.size()) {
    return false;
  }
  for (std::size_t i = 0; i < left.points.size(); ++i) {
    if (left.points[i].time_from_start_ms != right.points[i].time_from_start_ms ||
        left.points[i].motor_id != right.points[i].motor_id ||
        left.points[i].position != right.points[i].position ||
        left.points[i].velocity != right.points[i].velocity ||
        left.points[i].effort != right.points[i].effort) {
      return false;
    }
  }
  return true;
}

// fixture URDF 的 joint_1..3，话题名按测试前缀隔离（绝不碰现场的 /motor/*）。
robot_dynamics::GravityFeedforwardOverlayParameters test_parameters(
    const std::string & prefix) {
  robot_dynamics::GravityFeedforwardOverlayParameters parameters;
  parameters.urdf_path = fixture_path();
  parameters.joint_map = "1:joint_1,2:joint_2,3:joint_3";
  parameters.command_input_topic = prefix + "_command_in";
  parameters.trajectory_input_topic = prefix + "_trajectory_in";
  parameters.command_output_topic = prefix + "_command_out";
  parameters.trajectory_output_topic = prefix + "_trajectory_out";
  parameters.joint_states_topic = prefix + "_joint_states";
  return parameters;
}

// in-process 装置：被测节点 + 观察节点 + 单线程执行器 + 收件箱。
// 观察侧的 QoS 必须与节点输出端兼容：命令输出是 reliable（bridge 订阅端），轨迹输出 reliable。
struct Harness {
  Harness(std::shared_ptr<robot_dynamics::GravityFeedforwardOverlayNode> node_in,
          const robot_dynamics::GravityFeedforwardOverlayParameters & topics_in)
      : node(std::move(node_in)),
        topics(topics_in),
        observer(std::make_shared<rclcpp::Node>("overlay_test_observer")) {
    command_subscription =
        observer->create_subscription<can_bridge_msgs::msg::MotorCommandArray>(
            topics.command_output_topic,
            rclcpp::QoS(1).reliable().durability_volatile(),
            [this](const can_bridge_msgs::msg::MotorCommandArray & message) {
              commands.push_back(message);
            });
    trajectory_subscription =
        observer->create_subscription<can_bridge_msgs::msg::MotorTrajectory>(
            topics.trajectory_output_topic,
            rclcpp::QoS(1).reliable().durability_volatile(),
            [this](const can_bridge_msgs::msg::MotorTrajectory & message) {
              trajectories.push_back(message);
            });
    command_publisher =
        observer->create_publisher<can_bridge_msgs::msg::MotorCommandArray>(
            topics.command_input_topic,
            rclcpp::QoS(1).reliable().durability_volatile());
    trajectory_publisher =
        observer->create_publisher<can_bridge_msgs::msg::MotorTrajectory>(
            topics.trajectory_input_topic,
            rclcpp::QoS(1).reliable().durability_volatile());
    joint_states_publisher =
        observer->create_publisher<sensor_msgs::msg::JointState>(
            topics.joint_states_topic,
            rclcpp::QoS(1).reliable().durability_volatile());
    executor.add_node(node);
    executor.add_node(observer);
  }

  ~Harness() {
    executor.remove_node(node);
    executor.remove_node(observer);
  }

  void spin_for(std::chrono::milliseconds duration) {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(5ms);
    }
  }

  // 换帧之前先让在途帧落地再清空收件箱：否则上一帧可能在下一次 publish 之后才到，
  // back() 会指到旧帧。
  void drain() {
    spin_for(100ms);
    commands.clear();
    trajectories.clear();
  }

  void wait_for_links() {
    const auto linked = [this] {
      return command_publisher->get_subscription_count() > 0 &&
             trajectory_publisher->get_subscription_count() > 0 &&
             joint_states_publisher->get_subscription_count() > 0 &&
             command_subscription->get_publisher_count() > 0 &&
             trajectory_subscription->get_publisher_count() > 0;
    };
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (!linked() && std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(5ms);
    }
    ASSERT_TRUE(linked());
  }

  // 反复重发同一帧直到收到的最后一帧满足 predicate：命令输出是 best_effort 且深度 1，
  // /joint_states 的构型更新也不保证先于命令帧到达——重发让这两个时序竞态都消失，
  // 断言仍打在帧内容上。
  template <typename Predicate>
  can_bridge_msgs::msg::MotorCommandArray publish_command_until(
      const can_bridge_msgs::msg::MotorCommandArray & frame,
      Predicate predicate) {
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (std::chrono::steady_clock::now() < deadline) {
      command_publisher->publish(frame);
      executor.spin_some();
      if (!commands.empty() && predicate(commands.back())) {
        return commands.back();
      }
      std::this_thread::sleep_for(5ms);
    }
    ADD_FAILURE() << "等待命令输出帧超时";
    return commands.empty() ? can_bridge_msgs::msg::MotorCommandArray{}
                            : commands.back();
  }

  template <typename Predicate>
  can_bridge_msgs::msg::MotorTrajectory publish_trajectory_until(
      const can_bridge_msgs::msg::MotorTrajectory & frame, Predicate predicate) {
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (std::chrono::steady_clock::now() < deadline) {
      trajectory_publisher->publish(frame);
      executor.spin_some();
      if (!trajectories.empty() && predicate(trajectories.back())) {
        return trajectories.back();
      }
      std::this_thread::sleep_for(5ms);
    }
    ADD_FAILURE() << "等待轨迹输出帧超时";
    return trajectories.empty() ? can_bridge_msgs::msg::MotorTrajectory{}
                                : trajectories.back();
  }

  std::shared_ptr<robot_dynamics::GravityFeedforwardOverlayNode> node;
  robot_dynamics::GravityFeedforwardOverlayParameters topics;
  rclcpp::Node::SharedPtr observer;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::vector<can_bridge_msgs::msg::MotorCommandArray> commands;
  std::vector<can_bridge_msgs::msg::MotorTrajectory> trajectories;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_states_publisher;
  rclcpp::Publisher<can_bridge_msgs::msg::MotorCommandArray>::SharedPtr
      command_publisher;
  rclcpp::Publisher<can_bridge_msgs::msg::MotorTrajectory>::SharedPtr
      trajectory_publisher;
  rclcpp::Subscription<can_bridge_msgs::msg::MotorCommandArray>::SharedPtr
      command_subscription;
  rclcpp::Subscription<can_bridge_msgs::msg::MotorTrajectory>::SharedPtr
      trajectory_subscription;
};

std::shared_ptr<robot_dynamics::GravityFeedforwardOverlayNode> test_node(
    const robot_dynamics::GravityFeedforwardOverlayParameters & parameters) {
  return std::make_shared<robot_dynamics::GravityFeedforwardOverlayNode>(
      rclcpp::NodeOptions{}, parameters);
}

}  // namespace

// README 里对外的参数契约：名字与 fail-fast 行为。
TEST(GravityFeedforwardOverlayNode, ReadsDocumentedParametersAndFailsFast) {
  ensure_rclcpp_initialized();
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("urdf_path", fixture_path()),
       rclcpp::Parameter("joint_map", "1:joint_1"),
       rclcpp::Parameter("command_input_topic", "p_command_in"),
       rclcpp::Parameter("trajectory_input_topic", "p_trajectory_in"),
       rclcpp::Parameter("command_output_topic", "p_command_out"),
       rclcpp::Parameter("trajectory_output_topic", "p_trajectory_out"),
       rclcpp::Parameter("joint_states_topic", "p_joint_states"),
       rclcpp::Parameter("joint_states_timeout_s", 0.25),
       rclcpp::Parameter("effort_gain", 0.5)});
  {
    auto node =
        std::make_shared<robot_dynamics::GravityFeedforwardOverlayNode>(options);
    EXPECT_EQ(node->get_parameter("urdf_path").as_string(), fixture_path());
    EXPECT_EQ(node->get_parameter("joint_map").as_string(), "1:joint_1");
    EXPECT_EQ(node->get_parameter("command_input_topic").as_string(),
              "p_command_in");
    EXPECT_EQ(node->get_parameter("trajectory_input_topic").as_string(),
              "p_trajectory_in");
    EXPECT_EQ(node->get_parameter("command_output_topic").as_string(),
              "p_command_out");
    EXPECT_EQ(node->get_parameter("trajectory_output_topic").as_string(),
              "p_trajectory_out");
    EXPECT_EQ(node->get_parameter("joint_states_topic").as_string(),
              "p_joint_states");
    EXPECT_DOUBLE_EQ(node->get_parameter("joint_states_timeout_s").as_double(),
                     0.25);
    EXPECT_DOUBLE_EQ(node->get_parameter("effort_gain").as_double(), 0.5);
  }
  // urdf_path 未给（默认空）即构造失败，不静默起一个算不了前馈的节点。
  EXPECT_THROW(robot_dynamics::GravityFeedforwardOverlayNode(rclcpp::NodeOptions{}),
               std::invalid_argument);
  // joint_map 畸形 / motor_id 重复同样 fail-fast：静默跳过会让某台电机永远没有前馈。
  for (const char * joint_map : {"1:joint_1,2joint_2", "1:joint_1,1:joint_2",
                                 "joint_1", "1:"}) {
    rclcpp::NodeOptions bad;
    bad.parameter_overrides({rclcpp::Parameter("urdf_path", fixture_path()),
                             rclcpp::Parameter("joint_map", joint_map)});
    EXPECT_THROW((robot_dynamics::GravityFeedforwardOverlayNode{bad}),
                 std::invalid_argument)
        << "joint_map=" << joint_map;
  }
  rclcpp::shutdown();
}

// 快照：用 /joint_states 的实测构型算 τ_g，逐字段保留其余内容。
TEST(GravityFeedforwardOverlayNode, SnapshotAddsGravityAndPreservesEveryOtherField) {
  ensure_rclcpp_initialized();
  const robot_dynamics::GravityFeedforwardOverlayParameters parameters =
      test_parameters("snap_add");
  Harness harness(test_node(parameters), parameters);
  harness.wait_for_links();

  ASSERT_GT(harness.joint_states_publisher->get_subscription_count(), 0U);
  harness.joint_states_publisher->publish(
      joint_state({"joint_1", "joint_2", "joint_3"}, {0.3, -0.4, 0.7}));

  const can_bridge_msgs::msg::MotorCommandArray frame =
      command_array(7, {target(1, 1.0f, 0.5f, 2.0f), target(2, -1.0f, 0.0f, -3.0f),
                        target(3, 0.0f, 0.2f, 0.5f)});
  const std::vector<double> torque = closed_form(0.3, -0.4, 0.7);
  const can_bridge_msgs::msg::MotorCommandArray output =
      harness.publish_command_until(frame, [&torque](const auto & received) {
        return std::abs(static_cast<double>(effort_or_nan(received, 0)) -
                        (2.0 + torque[0])) < 1e-6;
      });

  // 除 effort 外逐字段等于输入。
  EXPECT_EQ(output.sync_time_ns, frame.sync_time_ns);
  EXPECT_EQ(output.sequence, frame.sequence);
  EXPECT_EQ(output.count, frame.count);
  ASSERT_EQ(output.targets.size(), frame.targets.size());
  for (std::size_t i = 0; i < frame.targets.size(); ++i) {
    EXPECT_EQ(output.targets[i].motor_id, frame.targets[i].motor_id);
    EXPECT_EQ(output.targets[i].position, frame.targets[i].position);
    EXPECT_EQ(output.targets[i].velocity, frame.targets[i].velocity);
    EXPECT_NEAR(output.targets[i].effort,
                static_cast<double>(frame.targets[i].effort) + torque[i], 1e-6)
        << "targets[" << i << "]";
  }
  rclcpp::shutdown();
}

// 安全不变式：/joint_states 从未收到 → 原样转发，不丢帧、不发空帧。
TEST(GravityFeedforwardOverlayNode, SnapshotPassesThroughWhenJointStatesNeverArrive) {
  ensure_rclcpp_initialized();
  const robot_dynamics::GravityFeedforwardOverlayParameters parameters =
      test_parameters("snap_never");
  Harness harness(test_node(parameters), parameters);
  harness.wait_for_links();

  const can_bridge_msgs::msg::MotorCommandArray frame =
      command_array(9, {target(1, 0.4f, 0.0f, 1.5f), target(2, -0.2f, 0.1f, -0.75f)});
  const can_bridge_msgs::msg::MotorCommandArray output =
      harness.publish_command_until(frame, [](const auto &) { return true; });

  EXPECT_TRUE(command_frames_identical(output, frame));
  rclcpp::shutdown();
}

// 安全不变式：joint_states_timeout_s = 0（构型视为永远过期）→ 原样转发，
// 即使刚喂过一帧新鲜 /joint_states。这条同时是 A/B 对照的"直通"档。
TEST(GravityFeedforwardOverlayNode, SnapshotPassesThroughWhenTimeoutIsZero) {
  ensure_rclcpp_initialized();
  robot_dynamics::GravityFeedforwardOverlayParameters parameters =
      test_parameters("snap_timeout");
  parameters.joint_states_timeout_s = 0.0;
  Harness harness(test_node(parameters), parameters);
  harness.wait_for_links();

  harness.joint_states_publisher->publish(
      joint_state({"joint_1", "joint_2", "joint_3"}, {0.3, -0.4, 0.7}));
  harness.spin_for(100ms);

  const can_bridge_msgs::msg::MotorCommandArray frame =
      command_array(11, {target(1, 0.4f, 0.0f, 1.5f), target(2, -0.2f, 0.1f, -0.75f)});
  const can_bridge_msgs::msg::MotorCommandArray output =
      harness.publish_command_until(frame, [](const auto &) { return true; });

  EXPECT_TRUE(command_frames_identical(output, frame));
  rclcpp::shutdown();
}

TEST(GravityFeedforwardOverlayNode, InitialPartialFeedbackDoesNotAuthorizeNeutralJoints) {
  ensure_rclcpp_initialized();
  auto parameters = test_parameters("snap_initial_partial");
  parameters.joint_states_timeout_s = 2.0;
  Harness harness(test_node(parameters), parameters);
  harness.wait_for_links();
  const auto frame = command_array(12, {target(1, 0.4f, 0.0f, 1.5f)});

  harness.joint_states_publisher->publish(joint_state({"joint_1"}, {0.3}));
  harness.spin_for(100ms);
  const auto partial = harness.publish_command_until(
      frame, [](const auto &) { return true; });
  EXPECT_TRUE(command_frames_identical(partial, frame));

  // 构型可以来自不同帧；只要每个依赖都已测量且仍新鲜即可恢复补偿。
  harness.joint_states_publisher->publish(
      joint_state({"joint_2", "joint_3"}, {-0.4, 0.7}));
  const double expected = 1.5 + closed_form(0.3, -0.4, 0.7)[0];
  const auto complete = harness.publish_command_until(frame, [expected](const auto & received) {
    return std::abs(static_cast<double>(effort_or_nan(received, 0)) - expected) < 1e-6;
  });
  EXPECT_NEAR(effort_or_nan(complete, 0), expected, 1e-6);
  rclcpp::shutdown();
}

TEST(GravityFeedforwardOverlayNode, PartialFeedbackCannotRenewMissingJointAfterTimeout) {
  ensure_rclcpp_initialized();
  auto parameters = test_parameters("snap_partial_timeout");
  parameters.joint_states_timeout_s = 0.3;
  Harness harness(test_node(parameters), parameters);
  harness.wait_for_links();
  const auto frame = command_array(14, {target(1, 0.4f, 0.0f, 1.5f)});
  harness.joint_states_publisher->publish(
      joint_state({"joint_1", "joint_2", "joint_3"}, {0.3, -0.4, 0.7}));
  const double expected = 1.5 + closed_form(0.3, -0.4, 0.7)[0];
  const auto complete = harness.publish_command_until(frame, [expected](const auto & received) {
    return std::abs(static_cast<double>(effort_or_nan(received, 0)) - expected) < 1e-6;
  });
  EXPECT_NEAR(effort_or_nan(complete, 0), expected, 1e-6);

  harness.spin_for(400ms);
  harness.joint_states_publisher->publish(
      joint_state({"joint_1", "joint_2"}, {0.3, -0.4}));
  harness.spin_for(100ms);
  harness.commands.clear();
  const auto partial = harness.publish_command_until(
      frame, [](const auto &) { return true; });
  EXPECT_TRUE(command_frames_identical(partial, frame));
  rclcpp::shutdown();
}

TEST(GravityFeedforwardOverlayNode, MeasurementStampControlsAgeInsteadOfArrivalTime) {
  ensure_rclcpp_initialized();
  const auto parameters = test_parameters("snap_measured_age");
  Harness harness(test_node(parameters), parameters);
  harness.wait_for_links();
  auto old = joint_state({"joint_1", "joint_2", "joint_3"}, {0.3, -0.4, 0.7});
  old.header.stamp = harness.node->now() - rclcpp::Duration::from_seconds(5.0);
  harness.joint_states_publisher->publish(old);
  harness.spin_for(100ms);
  const auto frame = command_array(15, {target(1, 0.4f, 0.0f, 1.5f)});
  const auto output = harness.publish_command_until(
      frame, [](const auto &) { return true; });
  EXPECT_TRUE(command_frames_identical(output, frame));
  rclcpp::shutdown();
}

TEST(GravityFeedforwardOverlayNode, UnrelatedFeedbackDoesNotRefreshExpiredConfiguration) {
  ensure_rclcpp_initialized();
  const auto parameters = test_parameters("snap_unrelated");
  Harness harness(test_node(parameters), parameters);
  harness.wait_for_links();
  auto old = joint_state({"joint_1", "joint_2", "joint_3"}, {0.3, -0.4, 0.7});
  old.header.stamp = harness.node->now() - rclcpp::Duration::from_seconds(5.0);
  harness.joint_states_publisher->publish(old);
  harness.spin_for(100ms);
  harness.joint_states_publisher->publish(
      joint_state({"gripper_finger", "joint_aux"}, {0.2, 0.4}));
  harness.spin_for(100ms);
  const auto frame = command_array(16, {target(1, 0.4f, 0.0f, 1.5f)});
  const auto output = harness.publish_command_until(
      frame, [](const auto &) { return true; });
  EXPECT_TRUE(command_frames_identical(output, frame));
  rclcpp::shutdown();
}

// 未知 motor_id：该电机原值不动，其余照加，帧不丢、count/targets 不改。
TEST(GravityFeedforwardOverlayNode, UnknownMotorIdLeavesOnlyItsEffortUntouched) {
  ensure_rclcpp_initialized();
  const robot_dynamics::GravityFeedforwardOverlayParameters parameters =
      test_parameters("snap_unknown");
  Harness harness(test_node(parameters), parameters);
  harness.wait_for_links();

  harness.joint_states_publisher->publish(
      joint_state({"joint_1", "joint_2", "joint_3"}, {0.3, -0.4, 0.7}));
  const std::vector<double> torque = closed_form(0.3, -0.4, 0.7);

  const can_bridge_msgs::msg::MotorCommandArray frame =
      command_array(13, {target(1, 0.4f, 0.0f, 1.0f), target(42, 0.4f, 0.0f, 1.0f)});
  const can_bridge_msgs::msg::MotorCommandArray output =
      harness.publish_command_until(frame, [&torque](const auto & received) {
        return std::abs(static_cast<double>(effort_or_nan(received, 0)) -
                        (1.0 + torque[0])) < 1e-6;
      });

  ASSERT_GE(output.targets.size(), 2U);
  EXPECT_NEAR(effort_or_nan(output, 0), 1.0 + torque[0], 1e-6);
  EXPECT_FLOAT_EQ(effort_or_nan(output, 1), 1.0f);  // 未知 id：原值
  EXPECT_EQ(output.targets[1].motor_id, 42);
  EXPECT_EQ(output.count, 2);
  EXPECT_EQ(output.targets.size(), 2U);
  rclcpp::shutdown();
}

// 轨迹：某台电机缺一个时刻 → 用插值构型算该时刻的前馈，其余字段原样。
TEST(GravityFeedforwardOverlayNode, TrajectoryInterpolatesMissingMotorTick) {
  ensure_rclcpp_initialized();
  const robot_dynamics::GravityFeedforwardOverlayParameters parameters =
      test_parameters("traj_interp");
  Harness harness(test_node(parameters), parameters);
  harness.wait_for_links();

  // 本分段只预测 joint_1/2，未覆盖的 joint_3 必须有新鲜测量。
  harness.joint_states_publisher->publish(joint_state({"joint_3"}, {0.0}));

  const can_bridge_msgs::msg::MotorTrajectory frame = trajectory(
      123, {1, 2},
      {trajectory_point(0, 1, 0.2f, 1.0f), trajectory_point(0, 2, -0.4f, 2.0f),
       trajectory_point(100, 1, 0.5f, 1.5f) /* motor 2 在 t=100 缺拍 */,
       trajectory_point(200, 1, 0.8f, 2.5f), trajectory_point(200, 2, 0.6f, 3.0f)});
  const std::vector<double> at_0 = closed_form(0.2, -0.4, 0.0);
  const std::vector<double> at_100 = closed_form(0.5, 0.1, 0.0);  // 插值 q2 = 0.1
  const std::vector<double> at_200 = closed_form(0.8, 0.6, 0.0);
  const can_bridge_msgs::msg::MotorTrajectory output =
      harness.publish_trajectory_until(frame, [&at_100](const auto & received) {
        return received.points.size() == 5 &&
               std::abs(static_cast<double>(received.points[2].effort) -
                        (1.5 + at_100[0])) < 1e-6;
      });

  EXPECT_EQ(output.sync_time_ns, frame.sync_time_ns);
  EXPECT_EQ(output.sequence, frame.sequence);
  EXPECT_EQ(output.trajectory_id, frame.trajectory_id);
  EXPECT_EQ(output.segment_index, frame.segment_index);
  EXPECT_EQ(output.segment_count, frame.segment_count);
  EXPECT_EQ(output.count, frame.count);
  EXPECT_EQ(output.target_count, frame.target_count);
  EXPECT_EQ(output.target_ids, frame.target_ids);
  ASSERT_EQ(output.points.size(), frame.points.size());
  const std::vector<double> expected_effort = {
      1.0 + at_0[0], 2.0 + at_0[1], 1.5 + at_100[0], 2.5 + at_200[0],
      3.0 + at_200[1]};
  for (std::size_t i = 0; i < frame.points.size(); ++i) {
    EXPECT_EQ(output.points[i].time_from_start_ms,
              frame.points[i].time_from_start_ms);
    EXPECT_EQ(output.points[i].motor_id, frame.points[i].motor_id);
    EXPECT_EQ(output.points[i].position, frame.points[i].position);
    EXPECT_EQ(output.points[i].velocity, frame.points[i].velocity);
    EXPECT_NEAR(output.points[i].effort, expected_effort[i], 1e-6)
        << "points[" << i << "]";
  }
  rclcpp::shutdown();
}

TEST(GravityFeedforwardOverlayNode, TrajectoryCannotUseStaleUnpredictedJoint) {
  ensure_rclcpp_initialized();
  auto parameters = test_parameters("traj_stale_dependency");
  parameters.joint_states_timeout_s = 2.0;
  Harness harness(test_node(parameters), parameters);
  harness.wait_for_links();
  auto old = joint_state({"joint_1", "joint_2", "joint_3"}, {0.3, -0.4, 0.7});
  old.header.stamp = harness.node->now() - rclcpp::Duration::from_seconds(5.0);
  harness.joint_states_publisher->publish(old);
  harness.spin_for(100ms);

  // joint_3 只列在 target_ids 中但没有路点，不算预测到了该关节。
  const auto frame = trajectory(124, {1, 2, 3},
      {trajectory_point(0, 1, 0.2f, 1.0f), trajectory_point(0, 2, -0.4f, 2.0f)});
  const auto stale = harness.publish_trajectory_until(
      frame, [](const auto &) { return true; });
  EXPECT_TRUE(trajectory_frames_identical(stale, frame));

  harness.joint_states_publisher->publish(joint_state({"joint_3"}, {0.7}));
  const double expected = 1.0 + closed_form(0.2, -0.4, 0.7)[0];
  const auto fresh = harness.publish_trajectory_until(frame, [expected](const auto & received) {
    return !received.points.empty() &&
           std::abs(static_cast<double>(received.points[0].effort) - expected) < 1e-6;
  });
  ASSERT_FALSE(fresh.points.empty());
  EXPECT_NEAR(fresh.points[0].effort, expected, 1e-6);
  rclcpp::shutdown();
}

TEST(GravityFeedforwardOverlayNode, FullyPredictedTrajectoryNeedsNoMeasuredConfiguration) {
  ensure_rclcpp_initialized();
  const auto parameters = test_parameters("traj_fully_predicted");
  Harness harness(test_node(parameters), parameters);
  harness.wait_for_links();
  const auto frame = trajectory(125, {1, 2, 3},
      {trajectory_point(0, 1, 0.2f, 1.0f), trajectory_point(0, 2, -0.4f, 2.0f),
       trajectory_point(0, 3, 0.7f, 3.0f)});
  const auto torque = closed_form(0.2, -0.4, 0.7);
  const auto output = harness.publish_trajectory_until(frame, [&torque](const auto & received) {
    return received.points.size() == 3 &&
           std::abs(static_cast<double>(received.points[0].effort) - (1.0 + torque[0])) < 1e-6;
  });
  ASSERT_EQ(output.points.size(), 3U);
  for (std::size_t i = 0; i < output.points.size(); ++i) {
    EXPECT_NEAR(output.points[i].effort, frame.points[i].effort + torque[i], 1e-6);
  }
  rclcpp::shutdown();
}

// 契约外的帧（count 超界 / 空轨迹取消）原样转发：不叠加、不改写、不丢帧。
TEST(GravityFeedforwardOverlayNode, MalformedAndCancelFramesPassThroughUnchanged) {
  ensure_rclcpp_initialized();
  const robot_dynamics::GravityFeedforwardOverlayParameters parameters =
      test_parameters("frames_odd");
  Harness harness(test_node(parameters), parameters);
  harness.wait_for_links();

  // 快照 count 超出 targets。
  can_bridge_msgs::msg::MotorCommandArray bad_command =
      command_array(21, {target(1, 0.4f, 0.0f, 1.5f)});
  bad_command.count = 5;
  const can_bridge_msgs::msg::MotorCommandArray command_output =
      harness.publish_command_until(bad_command, [](const auto &) { return true; });
  EXPECT_TRUE(command_frames_identical(command_output, bad_command));

  // 轨迹 count 超出 points。
  harness.drain();
  can_bridge_msgs::msg::MotorTrajectory bad_trajectory = trajectory(
      7, {1}, {trajectory_point(0, 1, 0.4f, 1.5f)});
  bad_trajectory.count = 9;
  const can_bridge_msgs::msg::MotorTrajectory trajectory_output =
      harness.publish_trajectory_until(bad_trajectory,
                                       [](const auto &) { return true; });
  EXPECT_TRUE(trajectory_frames_identical(trajectory_output, bad_trajectory));

  // 空轨迹（count==0 && target_count==0）= 取消：原样转发。
  harness.drain();
  const can_bridge_msgs::msg::MotorTrajectory cancel = trajectory(8, {}, {});
  const can_bridge_msgs::msg::MotorTrajectory cancel_output =
      harness.publish_trajectory_until(cancel, [](const auto &) { return true; });
  EXPECT_TRUE(trajectory_frames_identical(cancel_output, cancel));
  rclcpp::shutdown();
}

// effort_gain 运行期可改：1 → 有前馈；set 成 0 后两个通道都逐字段等于输入
//（这条同时是验收用的 A/B 直通档）。
TEST(GravityFeedforwardOverlayNode, EffortGainIsRuntimeChangeableAndZeroIsIdentity) {
  ensure_rclcpp_initialized();
  const robot_dynamics::GravityFeedforwardOverlayParameters parameters =
      test_parameters("gain");
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("urdf_path", fixture_path()),
       rclcpp::Parameter("joint_map", "1:joint_1,2:joint_2,3:joint_3"),
       rclcpp::Parameter("command_input_topic", parameters.command_input_topic),
       rclcpp::Parameter("trajectory_input_topic",
                         parameters.trajectory_input_topic),
       rclcpp::Parameter("command_output_topic",
                         parameters.command_output_topic),
       rclcpp::Parameter("trajectory_output_topic",
                         parameters.trajectory_output_topic),
       rclcpp::Parameter("joint_states_topic", parameters.joint_states_topic),
       rclcpp::Parameter("effort_gain", 1.0)});
  Harness harness(
      std::make_shared<robot_dynamics::GravityFeedforwardOverlayNode>(options),
      parameters);
  harness.wait_for_links();

  harness.joint_states_publisher->publish(
      joint_state({"joint_1", "joint_2", "joint_3"}, {0.3, -0.4, 0.7}));
  const std::vector<double> torque = closed_form(0.3, -0.4, 0.7);
  const can_bridge_msgs::msg::MotorCommandArray command =
      command_array(31, {target(1, 0.4f, 0.0f, 1.0f), target(2, -0.2f, 0.1f, -0.5f)});
  const can_bridge_msgs::msg::MotorCommandArray fed =
      harness.publish_command_until(command, [&torque](const auto & received) {
        return std::abs(static_cast<double>(effort_or_nan(received, 0)) -
                        (1.0 + torque[0])) < 1e-6;
      });
  EXPECT_FALSE(command_frames_identical(fed, command));  // 增益 1：确实加了前馈

  ASSERT_TRUE(harness.node
                  ->set_parameter(rclcpp::Parameter("effort_gain", 0.0))
                  .successful);

  const can_bridge_msgs::msg::MotorTrajectory trajectory_frame = trajectory(
      55, {1, 2},
      {trajectory_point(0, 1, 0.2f, 1.0f), trajectory_point(0, 2, -0.4f, 2.0f),
       trajectory_point(100, 1, 0.5f, 1.5f)});
  const can_bridge_msgs::msg::MotorTrajectory trajectory_output =
      harness.publish_trajectory_until(
          trajectory_frame, [&trajectory_frame](const auto & received) {
            return trajectory_frames_identical(received, trajectory_frame);
          });
  EXPECT_TRUE(trajectory_frames_identical(trajectory_output, trajectory_frame));

  const can_bridge_msgs::msg::MotorCommandArray passthrough =
      harness.publish_command_until(command, [&command](const auto & received) {
        return command_frames_identical(received, command);
      });
  EXPECT_TRUE(command_frames_identical(passthrough, command));
  rclcpp::shutdown();
}

TEST(GravityFeedforwardOverlayDefaults, OutputsGoThroughBridge) {
  // 叠加节点是主机侧命令源之一：输出必须落在 bridge 的主机入口，
  // 直发线上话题会让命令绕过 schema 门禁与定点编码。
  robot_dynamics::GravityFeedforwardOverlayParameters parameters;
  EXPECT_EQ(parameters.command_output_topic, "/motor/command_array");
  EXPECT_EQ(parameters.trajectory_output_topic, "/motor/trajectory");
}
