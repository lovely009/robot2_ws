#include "micro_ros_bridge/bridge_node.hpp"

#include "micro_ros_bridge/codec.hpp"
#include "micro_ros_bridge/messages.hpp"
#include <algorithm>
#include <cstdlib>
#include <limits>

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
rclcpp::QoS low_state_reader_qos()
{
  constexpr auto motor_count = can_bridge_msgs::msg::WireContract::MOTOR_COUNT_MAX;
  constexpr auto chunk_max = can_bridge_msgs::msg::WireContract::LOW_STATE_CHUNK_MAX;
  static_assert(chunk_max > 0);
  constexpr size_t depth = (motor_count + chunk_max - 1u) / chunk_max;
  return rclcpp::QoS(depth).best_effort();
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
  namespace T = contract::topic;

  // ---- 发布者先建齐，再建订阅者（回调里引用它们，虽在构造期不会触发） ----
  pub_low_cmd_ = create_publisher<LowCommand>(T::kLowCmd, stream_qos());
  pub_config_ = create_publisher<ConfigRequest>(T::kConfig, reliable_qos());
  service_config_ = create_service<can_bridge_msgs::srv::ExecuteConfig>(
    "/motor/execute_config",
    [this](std::shared_ptr<rmw_request_id_t> header,
      std::shared_ptr<can_bridge_msgs::srv::ExecuteConfig::Request> request) {
      using Service = can_bridge_msgs::srv::ExecuteConfig;
      auto reply = [&](uint8_t status, uint16_t error) {
          Service::Response response;
          response.result.session_id = session_id_;
          response.result.request_id = request->request.request_id;
          response.result.kind = request->request.kind;
          response.result.command = request->request.command;
          response.result.status = status;
          response.result.error_code = error;
          service_config_->send_response(*header, response);
        };
      if (pending_configs_.size() >= 8) {
        reply(ConfigResult::STATUS_BUSY, 0); return;
      }
      const uint32_t timeout = request->timeout_ms == 0 ? 5000 : request->timeout_ms;
      if (timeout < 100 || timeout > 10000 || !motion_ready()) {
        reply(ConfigResult::STATUS_FAILED, 22); return;
      }
      ConfigRequest outgoing;
      Result valid{false, "未知配置种类", 0};
      if (request->request.kind == ConfigRequest::CONFIG_KIND_PARAM_SET &&
        request->request.command == SystemCommand::COMMAND_SAVE &&
        request->request.motor_id == SystemCommand::MOTOR_ID_NONE)
      {
        ParamSetArray source;
        source.sync_time_ns = request->request.sync_time_ns;
        source.request_id = request->request.request_id;
        source.count = request->request.count;
        source.targets = request->request.targets;
        valid = encode_config(source, outgoing);
      } else if (request->request.kind == ConfigRequest::CONFIG_KIND_SYSTEM_COMMAND &&
        request->request.count == 0 && request->request.targets.empty())
      {
        SystemCommand source;
        source.sync_time_ns = request->request.sync_time_ns;
        source.request_id = request->request.request_id;
        source.command = request->request.command;
        source.motor_id = request->request.motor_id;
        valid = encode_config(source, outgoing);
      }
      if (!valid.ok) {reply(ConfigResult::STATUS_FAILED, 22); return;}
      outgoing.session_id = session_id_;
      if (!assign_request_id(outgoing)) {reply(ConfigResult::STATUS_CONFLICT, 0); return;}
      pending_configs_.push_back({header, session_id_, outgoing.request_id,
        request->request.request_id, outgoing.kind, outgoing.command,
        std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout)});
      pub_config_->publish(outgoing);
    });
  config_timer_ = create_wall_timer(std::chrono::milliseconds(20),
    [this]() {expire_configs();});
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
      if (!motion_ready()) {
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
      out.session_id = session_id_;
      if (!assign_request_id(out)) {
        RCLCPP_ERROR(get_logger(), "config request_id 非法、已退休或内容冲突，拒绝下发"); return;
      }
      pub_config_->publish(out);
    });
  sub_system_command_ = create_subscription<SystemCommand>(
    "/motor/system_command", reliable_qos(),
    [this](SystemCommand::SharedPtr m) {
      if (!motion_ready()) {
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
      out.session_id = session_id_;
      if (!assign_request_id(out)) {
        RCLCPP_ERROR(get_logger(), "config request_id 非法、已退休或内容冲突，拒绝下发"); return;
      }
      pub_config_->publish(out);
    });

  // ---- 线上 → 主机侧 ----
  sub_low_state_ = create_subscription<LowState>(
    T::kLowState, low_state_reader_qos(),
    [this](LowState::SharedPtr m) {on_low_state(m);});
  sub_motor_status_ = create_subscription<MotorStatus>(
    T::kMotorStatus, reliable_qos(),
    [this](MotorStatus::SharedPtr m) {on_motor_status(m);});

  // ---- 纯改名透传（消息类型不变） ----
  auto policy_pub = create_publisher<MotorPolicyArray>(T::kPolicy, reliable_qos());
  pass_pubs_.push_back(policy_pub);
  pass_subs_.push_back(create_subscription<MotorPolicyArray>(
    "/motor/policy_array", reliable_qos(), [this, policy_pub](MotorPolicyArray::SharedPtr m) {
      if (m->count != m->targets.size()) {return;}
      const bool stopping = std::all_of(m->targets.begin(), m->targets.end(),
        [](const auto & t) {return !t.enabled;});
      if (!stopping && !motion_ready()) {return;}
      auto out = *m;
      out.session_id = session_id_;
      policy_pub->publish(out);
    }));
  auto traj_pub = create_publisher<MotorTrajectory>(T::kTrajectory, reliable_qos());
  pass_pubs_.push_back(traj_pub);
  pass_subs_.push_back(create_subscription<MotorTrajectory>(
    "/motor/trajectory", reliable_qos(), [this, traj_pub](MotorTrajectory::SharedPtr m) {
      const bool stopping = m->count == 0 && m->target_count == 0 && m->points.empty() &&
        m->target_ids.empty();
      if (!stopping && !motion_ready()) {return;}
      auto out = *m;
      out.session_id = session_id_;
      traj_pub->publish(out);
    }));
  auto control_pub = create_publisher<MotorControlStateArray>(
    "/motor/control_states", reliable_qos());
  pass_pubs_.push_back(control_pub);
  pass_subs_.push_back(create_subscription<MotorControlStateArray>(
    T::kControlStates, reliable_qos(), [this, control_pub](MotorControlStateArray::SharedPtr m) {
      if (session_id_ != 0 && m->session_id == session_id_ &&
        m->schema_version == MotorControlStateArray::SCHEMA_VERSION) {control_pub->publish(*m);}
    }));
  auto topology_pub = create_publisher<MotorTopology>("/motor/topology", reliable_qos());
  pass_pubs_.push_back(topology_pub);
  pass_subs_.push_back(create_subscription<MotorTopology>(T::kTopology, reliable_qos(),
    [this, topology_pub](MotorTopology::SharedPtr m) {on_topology(m, topology_pub);}));
  auto events_pub = create_publisher<MotorEvent>("/motor/events", reliable_qos());
  pass_pubs_.push_back(events_pub);
  pass_subs_.push_back(create_subscription<MotorEvent>(T::kEvents, reliable_qos(),
    [this, events_pub](MotorEvent::SharedPtr m) {
      if (session_id_ != 0 && m->session_id == session_id_) {events_pub->publish(*m);}
    }));
  auto device_pub = create_publisher<DeviceParamReport>(
    "/motor/device_param_report", reliable_qos());
  pass_pubs_.push_back(device_pub);
  pass_subs_.push_back(create_subscription<DeviceParamReport>(T::kDeviceParam, reliable_qos(),
    [this, device_pub](DeviceParamReport::SharedPtr m) {
      if (session_id_ != 0 && m->session_id == session_id_) {device_pub->publish(*m);}
    }));
  auto result_pub = create_publisher<ConfigResult>("/motor/config_result", reliable_qos());
  pass_pubs_.push_back(result_pub);
  pass_subs_.push_back(create_subscription<ConfigResult>(T::kConfigResult, reliable_qos(),
    [this, result_pub](ConfigResult::SharedPtr m) {
      on_config_result(m, result_pub);
    }));
  make_pass_through<sensor_msgs::msg::Imu>(T::kImuData, "/imu/data", stream_qos());
  make_pass_through<std_msgs::msg::Float32>(T::kImuTemp, "/imu/temp", stream_qos());
}

void BridgeNode::clear_cache()
{
  have_status_ = false;
  have_topology_ = false;
  freshness_.assign(contract::kMotorCountMax, MotorFreshness{});
  assembler_.reset();
}

bool BridgeNode::motion_ready()
{
  if (!clock_stable()) {return false;}
  const auto now = std::chrono::steady_clock::now();
  if (have_status_ && now - status_at_ > std::chrono::milliseconds(300)) {clear_cache();}
  if (have_topology_ && now - topology_at_ > std::chrono::milliseconds(1500)) {clear_cache();}
  return session_id_ != 0 && have_status_ && have_topology_;
}

bool BridgeNode::clock_stable()
{
  const auto steady = std::chrono::steady_clock::now();
  const auto wall = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  if (have_clock_sample_) {
    const auto steady_delta = std::chrono::duration_cast<std::chrono::nanoseconds>(
      steady - clock_sample_at_).count();
    const auto drift = (wall - clock_sample_ns_) - steady_delta;
    if (std::llabs(drift) > 50000000LL) {
      clear_cache();
      clock_sample_at_ = steady;
      clock_sample_ns_ = wall;
      return false;
    }
  }
  clock_sample_at_ = steady;
  clock_sample_ns_ = wall;
  have_clock_sample_ = true;
  return true;
}

bool BridgeNode::source_time_valid(uint64_t source_ns, std::chrono::milliseconds max_age)
{
  if (!clock_stable() || source_ns == 0 ||
    source_ns > static_cast<uint64_t>(std::numeric_limits<int64_t>::max())) {return false;}
  const int64_t age = clock_sample_ns_ - static_cast<int64_t>(source_ns);
  return age >= -20000000LL && age <=
    std::chrono::duration_cast<std::chrono::nanoseconds>(max_age).count();
}

bool BridgeNode::assign_request_id(can_bridge_msgs::msg::ConfigRequest & request)
{
  if (request.request_id != 0 && request.request_id <= last_request_id_) {
    const auto it = std::find_if(recent_requests_.begin(), recent_requests_.end(),
      [&request](const auto & previous) {return previous.request_id == request.request_id;});
    if (it == recent_requests_.end()) {return false;}
    auto prior = *it;
    prior.sync_time_ns = request.sync_time_ns;
    return prior == request;
  }
  const uint32_t id = request.request_id == 0 ? next_request_id_ : request.request_id;
  if (id == 0 || id >= 0x40000000u || id <= last_request_id_) {return false;}
  request.request_id = id;
  last_request_id_ = id;
  next_request_id_ = id + 1;
  recent_requests_.push_back(request);
  if (recent_requests_.size() > 8) {recent_requests_.pop_front();}
  return true;
}

void BridgeNode::finish_config(size_t index, const can_bridge_msgs::msg::ConfigResult & result)
{
  using Service = can_bridge_msgs::srv::ExecuteConfig;
  auto pending = pending_configs_[index];
  pending_configs_.erase(pending_configs_.begin() + index);
  Service::Response response;
  response.result = result;
  response.result.request_id = pending.caller_id == 0 ? pending.wire_id : pending.caller_id;
  service_config_->send_response(*pending.header, response);
}

void BridgeNode::on_config_result(const can_bridge_msgs::msg::ConfigResult::SharedPtr & result,
  const rclcpp::Publisher<can_bridge_msgs::msg::ConfigResult>::SharedPtr & pub)
{
  using ResultMsg = can_bridge_msgs::msg::ConfigResult;
  if (session_id_ == 0 || result->session_id != session_id_ ||
    result->status > ResultMsg::STATUS_CONFLICT) {return;}
  pub->publish(*result);
  if (result->status == ResultMsg::STATUS_PENDING) {return;}
  for (size_t i = pending_configs_.size(); i > 0; --i) {
    const auto & pending = pending_configs_[i - 1];
    if (pending.session_id == result->session_id && pending.wire_id == result->request_id &&
      pending.kind == result->kind && pending.command == result->command)
    {
      finish_config(i - 1, *result);
    }
  }
}

void BridgeNode::expire_configs()
{
  const auto now = std::chrono::steady_clock::now();
  for (size_t i = pending_configs_.size(); i > 0; --i) {
    const auto & pending = pending_configs_[i - 1];
    if (now < pending.deadline) {continue;}
    can_bridge_msgs::msg::ConfigResult timed_out;
    timed_out.session_id = pending.session_id;
    timed_out.request_id = pending.wire_id;
    timed_out.kind = pending.kind;
    timed_out.command = pending.command;
    timed_out.status = can_bridge_msgs::msg::ConfigResult::STATUS_FAILED;
    timed_out.error_code = 21;
    finish_config(i - 1, timed_out);
  }
}

void BridgeNode::on_topology(const can_bridge_msgs::msg::MotorTopology::SharedPtr msg,
  const rclcpp::Publisher<can_bridge_msgs::msg::MotorTopology>::SharedPtr & pub)
{
  if (msg->session_id == 0 || msg->session_id != session_id_ ||
    msg->schema_version != contract::kSchemaVersion ||
    !source_time_valid(msg->sync_time_ns, std::chrono::milliseconds(1500))) {return;}
  bool seen[contract::kMotorCountMax] = {};
  for (const auto & entry : msg->entries) {
    if (entry.motor_id >= contract::kMotorCountMax || seen[entry.motor_id]) {return;}
    seen[entry.motor_id] = true;
  }
  have_topology_ = true;
  const auto age = std::max<int64_t>(0, clock_sample_ns_ - static_cast<int64_t>(msg->sync_time_ns));
  topology_at_ = std::chrono::steady_clock::now() - std::chrono::nanoseconds(age);
  pub->publish(*msg);
}

void BridgeNode::on_command(const can_bridge_msgs::msg::MotorCommandArray::SharedPtr msg)
{
  if (msg->count != 0 &&
    (!motion_ready() || sub_command_->get_publisher_count() > 1 ||
    !source_time_valid(msg->sync_time_ns, std::chrono::milliseconds(30))))
  {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
      "会话、拓扑、单一命令源或源时间门禁未通过：拒绝下发运动命令");
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
  if (msg->count != 0) {
    const auto now = std::chrono::steady_clock::now();
    const auto elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      now - status_at_).count();
    const auto source_age_ns = clock_sample_ns_ - static_cast<int64_t>(msg->sync_time_ns) +
      std::chrono::duration_cast<std::chrono::nanoseconds>(now - clock_sample_at_).count();
    const auto deadline = command_deadline_tick(
      status_tick_ms_, elapsed_ns, source_age_ns);
    if (!deadline) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "命令源意图剩余寿命不足 1 ms：拒绝下发");
      return;
    }
    out.valid_until_tick_ms = *deadline;
  }
  out.session_id = session_id_;
  out.sequence = ++command_sequence_;
  pub_low_cmd_->publish(out);
}

void BridgeNode::on_low_state(const can_bridge_msgs::msg::LowState::SharedPtr msg)
{
  if (!motion_ready() || !source_time_valid(msg->sync_time_ns, std::chrono::milliseconds(100))) {
    return;
  }
  can_bridge_msgs::msg::MotorMotionStateArray out;
  const auto delay_ns = std::max<int64_t>(0,
    clock_sample_ns_ - static_cast<int64_t>(msg->sync_time_ns));
  const auto delay_ms = static_cast<uint32_t>(delay_ns / 1000000);
  if (!assembler_.accept(*msg, session_id_, std::chrono::steady_clock::now(), out, delay_ms)) {
    return;
  }
  pub_motion_states_->publish(out);
}

void BridgeNode::on_motor_status(const can_bridge_msgs::msg::MotorStatus::SharedPtr msg)
{
  if (!source_time_valid(msg->sync_time_ns, std::chrono::milliseconds(300))) {return;}
  can_bridge_msgs::msg::MotorStatusArray statuses;
  can_bridge_msgs::msg::MotorTrajectoryState traj;
  can_bridge_msgs::msg::SystemStatus sys;
  can_bridge_msgs::msg::MotorDiagnostics diag;
  std::vector<MotorFreshness> next_freshness = freshness_;
  const auto r = decode_motor_status(*msg, statuses, traj, sys, diag, next_freshness);
  if (!r.ok) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "motor_status 丢弃：%s", r.error);
    return;
  }
  if (msg->session_id == 0 || msg->schema_version != contract::kSchemaVersion ||
    (session_id_ != 0 && msg->session_id < session_id_)) {return;}
  if (msg->session_id != session_id_) {
    for (size_t i = pending_configs_.size(); i > 0; --i) {
      can_bridge_msgs::msg::ConfigResult interrupted;
      interrupted.session_id = pending_configs_[i - 1].session_id;
      interrupted.request_id = pending_configs_[i - 1].wire_id;
      interrupted.kind = pending_configs_[i - 1].kind;
      interrupted.command = pending_configs_[i - 1].command;
      interrupted.status = can_bridge_msgs::msg::ConfigResult::STATUS_FAILED;
      interrupted.error_code = 21;
      finish_config(i - 1, interrupted);
    }
    clear_cache();
    session_id_ = msg->session_id;
    command_sequence_ = 0;
    last_request_id_ = msg->config_request_id;
    next_request_id_ = last_request_id_ + 1;
    recent_requests_.clear();
    have_status_sequence_ = false;
  } else if (have_status_sequence_ && static_cast<int32_t>(msg->sequence - status_sequence_) <= 0) {
    return;
  }
  if (!have_status_sequence_) {
    command_sequence_ = msg->command_sequence;
  }
  if (msg->config_request_id > last_request_id_) {
    last_request_id_ = msg->config_request_id;
    next_request_id_ = last_request_id_ + 1;
  }
  freshness_ = std::move(next_freshness);
  status_sequence_ = msg->sequence;
  status_tick_ms_ = msg->tick_ms;
  have_status_sequence_ = true;
  const auto age = std::max<int64_t>(0, clock_sample_ns_ - static_cast<int64_t>(msg->sync_time_ns));
  status_at_ = std::chrono::steady_clock::now() - std::chrono::nanoseconds(age);
  have_status_ = true;
  pub_status_array_->publish(statuses);
  pub_traj_state_->publish(traj);
  pub_system_status_->publish(sys);
  pub_diagnostics_->publish(diag);
}
}  // namespace micro_ros_bridge
