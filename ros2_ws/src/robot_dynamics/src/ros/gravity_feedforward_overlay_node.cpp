#include "robot_dynamics/ros/gravity_feedforward_overlay_node.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <system_error>
#include <utility>
#include <vector>

namespace robot_dynamics {
namespace {

constexpr int kLogThrottleMs = 5000;

std::string trim(const std::string & text) {
  const std::size_t first = text.find_first_not_of(" \t");
  if (first == std::string::npos) {
    return {};
  }
  return text.substr(first, text.find_last_not_of(" \t") - first + 1);
}

// "1:joint_link1,2:joint_link2" → {1,"joint_link1"},{2,"joint_link2"}；空串 = 兜底表。
// 条目畸形或 motor_id 重复时构造失败，避免静默丢失前馈。
MotorJointMap parse_joint_map(const std::string & spec) {
  const std::string effective =
      spec.empty() ? std::string(kDefaultJointMap) : spec;
  MotorJointMap map;
  std::size_t begin = 0;
  while (begin <= effective.size()) {
    const std::size_t end = effective.find(',', begin);
    const std::string item = trim(effective.substr(begin, end - begin));
    if (!item.empty()) {
      const std::size_t colon = item.find(':');
      if (colon == std::string::npos) {
        throw std::invalid_argument("robot_dynamics: joint_map 条目缺少冒号：" +
                                    item);
      }
      const std::string id_text = trim(item.substr(0, colon));
      const std::string joint_name = trim(item.substr(colon + 1));
      unsigned long motor_id = 0;
      const auto parsed =
          std::from_chars(id_text.data(), id_text.data() + id_text.size(),
                          motor_id);
      if (id_text.empty() || parsed.ec != std::errc{} ||
          parsed.ptr != id_text.data() + id_text.size() || motor_id > 65535UL) {
        throw std::invalid_argument("robot_dynamics: joint_map 的 motor_id 不是 "
                                    "0..65535 的整数：" + id_text);
      }
      if (joint_name.empty()) {
        throw std::invalid_argument("robot_dynamics: joint_map 条目关节名为空：" +
                                    item);
      }
      if (!map.emplace(static_cast<std::uint16_t>(motor_id), joint_name)
               .second) {
        throw std::invalid_argument("robot_dynamics: joint_map 里 motor_id 重复：" +
                                    id_text);
      }
    }
    if (end == std::string::npos) {
      break;
    }
    begin = end + 1;
  }
  return map;
}

// 消息注释里的校验项里与本节点相关的最小集。不合规的帧不叠加、原样放行，让它自己
// 到固件那一步再被拒——本节点只做叠加，不做规整。
bool trajectory_frame_well_formed(
    const can_bridge_msgs::msg::MotorTrajectory & message) {
  if (message.segment_count == 0 ||
      message.segment_index >= message.segment_count) {
    return false;
  }
  if (message.count > message.points.size() ||
      message.target_count > message.target_ids.size()) {
    return false;
  }
  const auto ids_end =
      message.target_ids.begin() +
      static_cast<std::ptrdiff_t>(message.target_count);
  for (std::size_t i = 0; i < message.count; ++i) {
    if (std::find(message.target_ids.begin(), ids_end,
                  message.points[i].motor_id) == ids_end) {
      return false;  // 路点的 motor_id 不在 target_ids 里
    }
  }
  return true;
}

std::string join_ids(const std::vector<std::uint16_t> & ids) {
  std::string joined;
  for (const std::uint16_t id : ids) {
    if (!joined.empty()) {
      joined += ", ";
    }
    joined += std::to_string(id);
  }
  return joined;
}

}  // namespace

GravityFeedforwardOverlayNode::GravityFeedforwardOverlayNode(
    const rclcpp::NodeOptions & options)
    : rclcpp::Node("gravity_feedforward_overlay", options) {
  GravityFeedforwardOverlayParameters parameters;
  parameters.urdf_path = declare_parameter<std::string>("urdf_path", "");
  parameters.joint_map =
      declare_parameter<std::string>("joint_map", kDefaultJointMap);
  parameters.command_input_topic = declare_parameter<std::string>(
      "command_input_topic", "robot_dynamics/command_array");
  parameters.trajectory_input_topic = declare_parameter<std::string>(
      "trajectory_input_topic", "robot_dynamics/trajectory");
  parameters.command_output_topic = declare_parameter<std::string>(
      "command_output_topic", "/motor/command_array");
  parameters.trajectory_output_topic = declare_parameter<std::string>(
      "trajectory_output_topic", "/motor/trajectory");
  parameters.joint_states_topic =
      declare_parameter<std::string>("joint_states_topic", "joint_states");
  parameters.joint_states_timeout_s =
      declare_parameter<double>("joint_states_timeout_s", 0.5);
  parameters.effort_gain = declare_parameter<double>("effort_gain", 1.0);
  initialize(std::move(parameters));
}

GravityFeedforwardOverlayNode::GravityFeedforwardOverlayNode(
    const rclcpp::NodeOptions & options,
    GravityFeedforwardOverlayParameters parameters)
    : rclcpp::Node("gravity_feedforward_overlay", options) {
  initialize(std::move(parameters));
}

void GravityFeedforwardOverlayNode::initialize(
    GravityFeedforwardOverlayParameters parameters) {
  model_ = std::make_unique<ArmModel>(parameters.urdf_path);  // 空/坏 URDF 即抛
  joint_map_ = parse_joint_map(parameters.joint_map);
  joint_states_timeout_s_ = parameters.joint_states_timeout_s;
  effort_gain_ = parameters.effort_gain;

  const std::vector<std::string> & joint_names = model_->joint_names();
  for (const auto & [motor_id, joint_name] : joint_map_) {
    mentioned_motor_ids_.insert(motor_id);
    const auto slot =
        std::find(joint_names.begin(), joint_names.end(), joint_name);
    if (slot == joint_names.end()) {
      RCLCPP_WARN(get_logger(),
                  "joint_map 的 %u:%s 落不到 URDF 的可控关节上，该电机不会有前馈",
                  motor_id, joint_name.c_str());
      continue;
    }
    torque_index_.emplace(motor_id,
                          static_cast<std::size_t>(slot - joint_names.begin()));
  }

  // effort_gain 运行期可改：只声明过的参数才会走这个回调（测试入口不声明参数）。
  parameters_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const rclcpp::Parameter & parameter : parameters) {
          if (parameter.get_name() == "effort_gain") {
            effort_gain_ = parameter.as_double();
          }
        }
        return result;
      });

  // 输出落到 bridge 的主机入口：那条订阅端是 RELIABLE，发 BEST_EFFORT
  // 在 ROS 2 里不建连接（静默失效）。轨迹仍是 reliable。
  // 深度 1：快照/分段都是覆盖语义。
  command_publisher_ =
      create_publisher<can_bridge_msgs::msg::MotorCommandArray>(
          parameters.command_output_topic,
          rclcpp::QoS(1).reliable().durability_volatile());
  trajectory_publisher_ =
      create_publisher<can_bridge_msgs::msg::MotorTrajectory>(
          parameters.trajectory_output_topic,
          rclcpp::QoS(1).reliable().durability_volatile());

  // 输入：三个命令源都是 RELIABLE，reliable/volatile 订阅与它们全部兼容。
  // 命令深度 1（只要最新快照）；轨迹深度 10（分段是离散命令，宁可排队也不丢）。
  command_subscription_ =
      create_subscription<can_bridge_msgs::msg::MotorCommandArray>(
          parameters.command_input_topic,
          rclcpp::QoS(1).reliable().durability_volatile(),
          [this](can_bridge_msgs::msg::MotorCommandArray::ConstSharedPtr
                     message) { command_callback(*message); });
  trajectory_subscription_ =
      create_subscription<can_bridge_msgs::msg::MotorTrajectory>(
          parameters.trajectory_input_topic,
          rclcpp::QoS(10).reliable().durability_volatile(),
          [this](can_bridge_msgs::msg::MotorTrajectory::ConstSharedPtr message) {
            trajectory_callback(*message);
          });
  // /joint_states：现场 MuJoCo 桥与 joint_state_broadcaster 都是 RELIABLE。
  joint_state_subscription_ =
      create_subscription<sensor_msgs::msg::JointState>(
          parameters.joint_states_topic,
          rclcpp::QoS(1).reliable().durability_volatile(),
          [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
            joint_state_callback(*message);
          });

  RCLCPP_INFO(get_logger(),
              "τ_g 叠加：%s → %s，%s → %s；%zu/%zu 台电机接上模型，超时 %.3fs，增益 %.3f",
              parameters.command_input_topic.c_str(),
              parameters.command_output_topic.c_str(),
              parameters.trajectory_input_topic.c_str(),
              parameters.trajectory_output_topic.c_str(), torque_index_.size(),
              mentioned_motor_ids_.size(), joint_states_timeout_s_,
              effort_gain_);
}

bool GravityFeedforwardOverlayNode::joint_states_fresh() const {
  const auto & names = model_->joint_names();
  if (names.empty()) {
    return false;
  }
  const int64_t now_ns = now().nanoseconds();
  for (const std::string & name : names) {
    if (!joint_state_fresh(name, now_ns)) {
      return false;
    }
  }
  return true;
}

bool GravityFeedforwardOverlayNode::joint_state_fresh(
    const std::string & name, int64_t now_ns) const {
  if (joint_states_timeout_s_ <= 0.0) {
    return false;  // 0 = 测量永远视为过期（快照 A/B 直通用）。
  }
  const auto measured = joint_state_stamps_ns_.find(name);
  if (measured == joint_state_stamps_ns_.end()) {
    return false;
  }
  const double age_s =
      static_cast<double>(now_ns - measured->second) * 1e-9;
  return age_s >= 0.0 && age_s <= joint_states_timeout_s_;
}

const std::size_t * GravityFeedforwardOverlayNode::torque_index_for(
    std::uint16_t motor_id, const char * channel) {
  const auto mapped = torque_index_.find(motor_id);
  if (mapped != torque_index_.end()) {
    return &mapped->second;
  }
  if (mentioned_motor_ids_.count(motor_id) == 0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs,
                         "%s：motor_id %u 不在 joint_map 里，该电机没有前馈",
                         channel, motor_id);
  } else {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), kLogThrottleMs,
        "%s：motor_id %u 的关节名落不到 URDF 的可控关节上，该电机没有前馈", channel,
        motor_id);
  }
  return nullptr;
}

void GravityFeedforwardOverlayNode::joint_state_callback(
    const sensor_msgs::msg::JointState & message) {
  // 畸形帧只丢这一帧的构型更新（命令帧照常转发，用上一次构型或直通）。
  if (message.name.size() != message.position.size()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs,
                         "joint_states 的 name/position 长度不符（%zu/%zu），"
                         "本帧不参与构型更新",
                         message.name.size(), message.position.size());
    return;
  }
  const int64_t now_ns = now().nanoseconds();
  if (message.header.stamp.sec < 0 ||
      message.header.stamp.nanosec >= 1000000000U) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs,
                         "joint_states 的测量时间无效，本帧不参与构型更新");
    return;
  }
  const int64_t stamped_ns =
      static_cast<int64_t>(message.header.stamp.sec) * 1000000000LL +
      message.header.stamp.nanosec;
  // 零时间戳表示发布者未提供测量时刻，兼容此类消息时以接收时刻计龄。
  const int64_t measured_ns = stamped_ns == 0 ? now_ns : stamped_ns;
  if (measured_ns > now_ns) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs,
                         "joint_states 的测量时间在未来，本帧不参与构型更新");
    return;
  }

  std::vector<std::string> names;
  std::vector<double> positions;
  std::size_t ignored = 0;
  for (std::size_t i = 0; i < message.name.size(); ++i) {
    if (!model_->has_joint(message.name[i])) {
      ++ignored;
      continue;
    }
    if (!std::isfinite(message.position[i])) {
      continue;  // 坏测量不污染模型，也不能续期该关节。
    }
    const auto previous = joint_state_stamps_ns_.find(message.name[i]);
    if (previous != joint_state_stamps_ns_.end() &&
        previous->second <= now_ns && measured_ns < previous->second) {
      continue;  // 延迟到达的旧帧不能覆盖更新的构型。
    }
    names.push_back(message.name[i]);
    positions.push_back(message.position[i]);
    joint_state_stamps_ns_[message.name[i]] = measured_ns;
  }
  model_->update_configuration(names, positions);
  if (ignored != 0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs,
                         "joint_states 含模型外或非单自由度关节，已忽略 %zu 个",
                         ignored);
  }
}

void GravityFeedforwardOverlayNode::command_callback(
    const can_bridge_msgs::msg::MotorCommandArray & message) {
  // output 是 message 的逐字段副本：sync_time_ns/sequence/count/targets 全原样，
  // 只改 effort。任何分支都发布它（不丢帧）。
  can_bridge_msgs::msg::MotorCommandArray output = message;
  if (message.count > message.targets.size()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs,
                         "命令快照 count=%u 超出 targets=%zu，原样转发",
                         message.count, message.targets.size());
  } else if (joint_states_fresh()) {
    const std::vector<double> torque = model_->gravity_torque();
    for (std::size_t i = 0; i < message.count; ++i) {
      const std::size_t * index =
          torque_index_for(message.targets[i].motor_id, "命令快照");
      if (index == nullptr) {
        continue;
      }
      output.targets[i].effort = static_cast<float>(
          static_cast<double>(message.targets[i].effort) + effort_gain_ * torque[*index]);
    }
  }
  command_publisher_->publish(output);
}

void GravityFeedforwardOverlayNode::trajectory_callback(
    const can_bridge_msgs::msg::MotorTrajectory & message) {
  can_bridge_msgs::msg::MotorTrajectory output = message;  // 逐字段副本
  if (!trajectory_frame_well_formed(message)) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), kLogThrottleMs,
        "轨迹帧不符合契约（segment %u/%u，count %u/%zu，target_count %u/%zu，"
        "或路点 motor_id 不在 target_ids 里），原样转发",
        message.segment_index, message.segment_count, message.count,
        message.points.size(), message.target_count, message.target_ids.size());
    trajectory_publisher_->publish(output);
    return;
  }

  std::vector<std::uint16_t> target_ids(
      message.target_ids.begin(),
      message.target_ids.begin() + message.target_count);
  std::vector<TrajectoryPoint> points;
  points.reserve(message.count);
  for (std::size_t i = 0; i < message.count; ++i) {
    points.push_back({message.points[i].time_from_start_ms,
                      message.points[i].motor_id,
                      message.points[i].position});
  }
  // 轨迹覆盖的关节由路点构型（含插值）预测，不要求实测反馈；其余关节仍要有
  // 新鲜实测值，不能从 q_ 的旧值或初始 neutral 猜测。只有 target_id 而没有
  // 有效路点的关节不会进入预测集合。
  std::unordered_set<std::string> predicted_joints;
  for (const TrajectoryPoint & point : points) {
    const auto mapped = joint_map_.find(point.motor_id);
    if (mapped != joint_map_.end() && model_->has_joint(mapped->second) &&
        std::isfinite(point.position_rad)) {
      predicted_joints.insert(mapped->second);
    }
  }
  const int64_t now_ns = now().nanoseconds();
  for (const std::string & name : model_->joint_names()) {
    if (predicted_joints.count(name) == 0 &&
        !joint_state_fresh(name, now_ns)) {
      trajectory_publisher_->publish(output);
      return;
    }
  }
  const TrajectoryFeedforward feedforward =
      trajectory_gravity_feedforward(*model_, joint_map_, target_ids, points);
  if (!feedforward.unmapped_motor_ids.empty()) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), kLogThrottleMs,
        "轨迹的 target_ids 里这些电机不在 joint_map 中，本分段没有前馈：%s",
        join_ids(feedforward.unmapped_motor_ids).c_str());
  }
  if (!feedforward.ignored_joint_names.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs,
                         "轨迹的 target_ids 里这些关节落不到 URDF 的可控关节上，"
                         "对应电机本分段没有前馈（%zu 个）",
                         feedforward.ignored_joint_names.size());
  }
  for (std::size_t i = 0; i < message.count; ++i) {
    output.points[i].effort = static_cast<float>(
        static_cast<double>(message.points[i].effort) +
        effort_gain_ * feedforward.effort_delta_nm[i]);
  }
  trajectory_publisher_->publish(output);
}

}  // namespace robot_dynamics
