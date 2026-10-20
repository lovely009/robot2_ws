#include "micro_ros_bridge/bridge_node.hpp"

#include "micro_ros_bridge/codec.hpp"
#include "micro_ros_bridge/messages.hpp"

namespace micro_ros_bridge
{
namespace
{
// QoS 判据是**消息性质**不是业务域：
//   stream   高频流，丢了就丢了——下一拍就是新的（循环面）
//   reliable 低频状态声明，漏一拍会让页面停在旧值上
// 两端 QoS 必须**相容**：best-effort 发布者配 reliable 订阅者是静默失效
// （不报错、永远收不到）。所以下行链路上"订阅端要 reliable"就必须让上游也 reliable。
rclcpp::QoS stream_qos()
{
  return rclcpp::QoS(1).best_effort();
}
rclcpp::QoS reliable_qos()
{
  return rclcpp::QoS(10).reliable();
}
// 主机侧入口：语义是"最新意图"，depth 1 就够；reliable 因为主机内不该丢。
rclcpp::QoS host_intent_qos()
{
  return rclcpp::QoS(1).reliable();
}
}  // namespace

template<typename T>
void BridgeNode::make_pass_through(
  const std::string & wire_topic, const std::string & host_topic, const rclcpp::QoS & qos)
{
  auto pub = create_publisher<T>(host_topic, qos);
  auto sub = create_subscription<T>(
    wire_topic, qos, [pub](typename T::SharedPtr m) {pub->publish(*m);});
  pass_pubs_.push_back(pub);
  pass_subs_.push_back(sub);
}

BridgeNode::BridgeNode()
: rclcpp::Node("micro_ros_bridge")
{
  using namespace can_bridge_msgs::msg;  // NOLINT(build/namespaces)
  // 命名空间别名，不是对象引用：`contract::topic` 是命名空间，绑不到 `auto &`。
  // 块作用域的 namespace 别名是合法的（`namespace fs = std::filesystem;` 同理）。
  namespace T = contract::topic;

  // ---- 发布者先建齐，再建订阅者（回调里引用它们，虽在构造期不会触发） ----
  pub_low_cmd_ = create_publisher<LowCommand>(T::kLowCmd, stream_qos());
  pub_config_ = create_publisher<ConfigRequest>(T::kConfig, reliable_qos());
  pub_motion_states_ = create_publisher<MotorMotionStateArray>("/motor/motion_states", stream_qos());
  pub_status_array_ = create_publisher<MotorStatusArray>("/motor/status_array", reliable_qos());
  pub_traj_state_ = create_publisher<MotorTrajectoryState>("/motor/trajectory_state", reliable_qos());
  pub_system_status_ = create_publisher<SystemStatus>("/motor/system_status", reliable_qos());
  pub_diagnostics_ = create_publisher<MotorDiagnostics>("/motor/diagnostics", reliable_qos());

  // ---- 主机侧 → 线上 ----
  sub_command_ = create_subscription<MotorCommandArray>(
    "/motor/command_array", host_intent_qos(),
    [this](MotorCommandArray::SharedPtr m) {on_command(m);});
  sub_param_set_ = create_subscription<ParamSetArray>(
    "/motor/param_set", reliable_qos(),
    [this](ParamSetArray::SharedPtr m) {
      if (peer_schema_mismatch()) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000, "schema_version 不一致：拒绝下发参数写");
        return;
      }
      ConfigRequest out;
      const auto r = encode_config(*m, out);
      if (!r.ok) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "param_set 整条拒收：%s", r.error);
        return;
      }
      pub_config_->publish(out);
    });
  sub_system_command_ = create_subscription<SystemCommand>(
    "/motor/system_command", reliable_qos(),
    [this](SystemCommand::SharedPtr m) {
      if (peer_schema_mismatch()) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000, "schema_version 不一致：拒绝下发系统命令");
        return;
      }
      ConfigRequest out;
      const auto r = encode_config(*m, out);
      if (!r.ok) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "system_command 整条拒收：%s", r.error);
        return;
      }
      pub_config_->publish(out);
    });

  // ---- 线上 → 主机侧 ----
  sub_low_state_ = create_subscription<LowState>(
    T::kLowState, stream_qos(),
    [this](LowState::SharedPtr m) {on_low_state(m);});
  sub_motor_status_ = create_subscription<MotorStatus>(
    T::kMotorStatus, reliable_qos(),
    [this](MotorStatus::SharedPtr m) {on_motor_status(m);});

  // ---- 纯改名透传（消息类型不变） ----
  make_pass_through<MotorPolicyArray>(T::kPolicy, "/motor/policy_array", reliable_qos());
  make_pass_through<MotorTrajectory>(T::kTrajectory, "/motor/trajectory", reliable_qos());
  make_pass_through<MotorTopology>(T::kTopology, "/motor/topology", reliable_qos());
  make_pass_through<MotorEvent>(T::kEvents, "/motor/events", reliable_qos());
  make_pass_through<DeviceParamReport>(
    T::kDeviceParam, "/motor/device_param_report", reliable_qos());
  make_pass_through<sensor_msgs::msg::Imu>(T::kImuData, "/imu/data", stream_qos());
  make_pass_through<std_msgs::msg::Float32>(T::kImuTemp, "/imu/temp", stream_qos());
}

bool BridgeNode::peer_schema_mismatch() const
{
  const uint16_t known = peer_schema_.load();
  return known != 0 && known != contract::kSchemaVersion;
}

void BridgeNode::on_command(const can_bridge_msgs::msg::MotorCommandArray::SharedPtr msg)
{
  // ADR 0015 决定 8：镜像不同步时**拒绝下发**，不静默按旧布局解析。
  if (peer_schema_mismatch()) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "固件 schema_version=%u 与本机 %u 不一致：拒绝下发命令",
      peer_schema_.load(), contract::kSchemaVersion);
    return;
  }
  can_bridge_msgs::msg::LowCommand out;
  const auto r = encode_low_command(*msg, out);
  if (!r.ok) {
    // 整条拒绝：一个字节都不下发（与固件侧同口径）。
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "命令整条拒收：%s", r.error);
    return;
  }
  if (r.saturated != 0) {
    // 饱和说明量程常量选错了，必须在台架上看得见，而不是让命令静默变形。
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "命令有 %u 个通道被量程钳位", r.saturated);
  }
  pub_low_cmd_->publish(out);
}

void BridgeNode::on_low_state(const can_bridge_msgs::msg::LowState::SharedPtr msg)
{
  can_bridge_msgs::msg::MotorMotionStateArray out;
  const auto r = decode_low_state(*msg, out);
  if (!r.ok) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "low_state 丢弃：%s", r.error);
    return;
  }
  // 新鲜度不在这条消息里（循环面不分片），用低频面 10 Hz 缓存的值盖章。
  // 页面读的正是 motion_states 元素上的这两个字段（readout.js:34 的「测量年龄」、
  // arm.js:270 的 target_source），spec §4.7 承诺 JS 一个字段都不用改。
  // 代价：显示值最多滞后一个 motor_status 周期（100 ms）——年龄本身是陈旧度度量。
  for (auto & s : out.states) {
    s.state_age_ms = freshness_[s.motor_id].state_age_ms;
    s.target_source = freshness_[s.motor_id].target_source;
  }
  pub_motion_states_->publish(out);
}

void BridgeNode::on_motor_status(const can_bridge_msgs::msg::MotorStatus::SharedPtr msg)
{
  // ADR 0015 决定 8：schema_version 由低频状态面带回，这里 latch 一次。
  // 只在**变化**时报，避免 10 Hz 刷屏。
  const uint16_t peer = msg->schema_version;
  if (peer != peer_schema_.load()) {
    peer_schema_.store(peer);
    if (peer == contract::kSchemaVersion) {
      RCLCPP_INFO(get_logger(), "固件 schema_version=%u 与本机一致，放行命令", peer);
    } else {
      RCLCPP_ERROR(
        get_logger(), "固件 schema_version=%u 与本机 %u 不一致：已停止下发命令",
        peer, contract::kSchemaVersion);
    }
  }

  can_bridge_msgs::msg::MotorStatusArray statuses;
  can_bridge_msgs::msg::MotorTrajectoryState traj;
  can_bridge_msgs::msg::SystemStatus sys;
  can_bridge_msgs::msg::MotorDiagnostics diag;
  const auto r = decode_motor_status(*msg, statuses, traj, sys, diag, freshness_);
  if (!r.ok) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "motor_status 丢弃：%s", r.error);
    return;
  }
  pub_status_array_->publish(statuses);
  pub_traj_state_->publish(traj);
  pub_system_status_->publish(sys);
  pub_diagnostics_->publish(diag);
}
}  // namespace micro_ros_bridge
