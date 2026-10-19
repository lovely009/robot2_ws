// 节点图接线测试：话题名与 QoS 是**契约**，不是实现细节。
// 话题名取自 WireContract 常量，QoS 按消息性质选（高频流 vs 低频状态声明）。
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include "micro_ros_bridge/bridge_node.hpp"
#include "micro_ros_bridge/codec.hpp"

// 线上话题名是契约：少一个、拼错一个，症状都是"页面某块永远没数"。
TEST(BridgeNode, WireTopicsAreDeclared) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<micro_ros_bridge::BridgeNode>();
  const auto topics = node->get_topic_names_and_types();
  namespace t = micro_ros_bridge::contract::topic;
  for (const auto * name : {&t::kLowCmd, &t::kPolicy, &t::kTrajectory, &t::kConfig,
      &t::kLowState, &t::kMotorStatus, &t::kTopology, &t::kEvents, &t::kDeviceParam,
      &t::kImuData, &t::kImuTemp})
  {
    EXPECT_EQ(topics.count(*name), 1u) << "线上话题缺失：" << *name;
  }
  rclcpp::shutdown();
}

// 主机侧话题名是页面与动作库的接口。
TEST(BridgeNode, HostTopicsAreDeclared) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<micro_ros_bridge::BridgeNode>();
  const auto topics = node->get_topic_names_and_types();
  for (const char * name : {"/motor/command_array", "/motor/motion_states",
      "/motor/status_array", "/motor/trajectory_state", "/motor/system_status",
      "/motor/diagnostics", "/motor/param_set", "/motor/system_command",
      "/motor/policy_array", "/motor/trajectory", "/motor/topology", "/motor/events",
      "/motor/device_param_report", "/imu/data", "/imu/temp"})
  {
    EXPECT_EQ(topics.count(name), 1u) << "主机侧话题缺失：" << name;
  }
  rclcpp::shutdown();
}

// 端到端（同进程）：主机侧命令进 → 线上 low_cmd 出，且已被定点化。
TEST(BridgeNode, CommandArrayBecomesLowCommand) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<micro_ros_bridge::BridgeNode>();
  auto pub = node->create_publisher<can_bridge_msgs::msg::MotorCommandArray>(
    "/motor/command_array", rclcpp::QoS(1).reliable());
  can_bridge_msgs::msg::LowCommand got;
  std::atomic<bool> received{false};
  auto sub = node->create_subscription<can_bridge_msgs::msg::LowCommand>(
    micro_ros_bridge::contract::topic::kLowCmd, rclcpp::QoS(1).best_effort(),
    [&](can_bridge_msgs::msg::LowCommand::SharedPtr m) { got = *m; received = true; });

  can_bridge_msgs::msg::MotorCommandArray in;
  in.count = 1;
  can_bridge_msgs::msg::MotorCommandTarget t;
  t.motor_id = 0;
  t.position = 0.5;
  in.targets.push_back(t);
  pub->publish(in);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!received && std::chrono::steady_clock::now() < deadline) {
    exec.spin_some();
  }
  ASSERT_TRUE(received) << "2 s 内没有收到 low_cmd";
  EXPECT_EQ(got.count, 1);
  EXPECT_EQ(got.motor_ids[0], 0);
  rclcpp::shutdown();
}

// ADR 0015 决定 8：两端镜像不同步时拒绝发命令并报错，不静默按旧布局解析。
TEST(BridgeNode, RefusesCommandsOnSchemaMismatch) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<micro_ros_bridge::BridgeNode>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  auto status_pub = node->create_publisher<can_bridge_msgs::msg::MotorStatus>(
    micro_ros_bridge::contract::topic::kMotorStatus, rclcpp::QoS(10).reliable());
  auto cmd_pub = node->create_publisher<can_bridge_msgs::msg::MotorCommandArray>(
    "/motor/command_array", rclcpp::QoS(1).reliable());
  std::atomic<int> low_cmd_count{0};
  auto sub = node->create_subscription<can_bridge_msgs::msg::LowCommand>(
    micro_ros_bridge::contract::topic::kLowCmd, rclcpp::QoS(1).best_effort(),
    [&low_cmd_count](can_bridge_msgs::msg::LowCommand::SharedPtr) {++low_cmd_count;});

  auto spin_for = [&exec](std::chrono::milliseconds d) {
      const auto deadline = std::chrono::steady_clock::now() + d;
      while (std::chrono::steady_clock::now() < deadline) {
        exec.spin_some();
      }
    };
  auto send_command = [&]() {
      can_bridge_msgs::msg::MotorCommandArray in;
      in.count = 1;
      can_bridge_msgs::msg::MotorCommandTarget t;
      t.motor_id = 0;
      in.targets.push_back(t);
      cmd_pub->publish(in);
      spin_for(std::chrono::milliseconds(200));
    };
  auto announce_schema = [&](uint16_t v) {
      can_bridge_msgs::msg::MotorStatus st;
      st.schema_version = v;
      st.count = 0;
      status_pub->publish(st);
      spin_for(std::chrono::milliseconds(200));
    };

  announce_schema(99);  // 对端版本不匹配
  const int before = low_cmd_count.load();
  send_command();
  EXPECT_EQ(low_cmd_count.load(), before) << "版本不一致时仍在发命令";

  announce_schema(1);  // 对端回到本机版本 → 恢复放行
  send_command();
  EXPECT_GT(low_cmd_count.load(), before) << "版本一致后仍未放行";
  rclcpp::shutdown();
}
