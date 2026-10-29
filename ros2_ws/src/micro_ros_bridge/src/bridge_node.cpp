#include "micro_ros_bridge/bridge_node.hpp"
#include "micro_ros_bridge/agent_messages.hpp"
#include <can_bridge_msgs/msg/agent_wire_contract.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/float32.hpp>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <stdexcept>

namespace micro_ros_bridge
{
using namespace can_bridge_msgs::msg;  // NOLINT(build/namespaces)
namespace {
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
uint64_t epoch_ns() {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count());
}
rclcpp::QoS reliable() {return rclcpp::QoS(10).reliable();}
rclcpp::QoS stream() {return rclcpp::QoS(1).best_effort();}
size_t bits(uint64_t mask) {
  size_t n = 0; while (mask) {mask &= mask - 1; ++n;} return n;
}
bool valid_control_time(const AgentStatus & m) {
  return m.clock_valid ? m.snapshot_stamp != 0 : m.snapshot_stamp == 0;
}
}  // namespace

BridgeNode::BridgeNode(const rclcpp::NodeOptions & options) : rclcpp::Node("micro_ros_bridge", options)
{
  using W = AgentWireContract;
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  const auto execution = declare_parameter("trajectory_execution", "host", descriptor);
  if (execution != "firmware" && execution != "host") {
    throw std::invalid_argument("trajectory_execution must be firmware or host");
  }
  host_execution_ = execution == "host";
  command_pub_ = create_publisher<AgentCommand>(W::TOPIC_COMMAND, stream());
  config_pub_ = create_publisher<Config>(W::TOPIC_CONFIG, rclcpp::QoS(4).reliable());
  trajectory_pub_ = create_publisher<Trajectory>(W::TOPIC_TRAJECTORY, reliable());
  motion_pub_ = create_publisher<MotorMotionStateArray>("/motor/motion_states", stream());
  status_pub_ = create_publisher<MotorStatusArray>("/motor/status_array", reliable());
  control_pub_ = create_publisher<MotorControlStateArray>("/motor/control_states", reliable());
  topology_pub_ = create_publisher<MotorTopology>("/motor/topology", reliable());
  params_pub_ = create_publisher<DeviceParamReport>("/motor/device_param_report", reliable());
  traj_state_pub_ = create_publisher<MotorTrajectoryState>("/motor/trajectory_state", reliable());
  system_pub_ = create_publisher<SystemStatus>("/motor/system_status", reliable());
  diagnostics_pub_ = create_publisher<MotorDiagnostics>("/motor/diagnostics", reliable());
  result_pub_ = create_publisher<Reply>("/motor/config_result", reliable());
  event_pub_ = create_publisher<MotorEvent>("/motor/events", reliable());
  command_sub_ = create_subscription<MotorCommandArray>(
    host_execution_ ? "/motor/host_command_array" : "/motor/command_array",
    rclcpp::QoS(1).reliable(), [this](MotorCommandArray::SharedPtr m) {on_command(*m);});
  subscriptions_.push_back(create_subscription<MotorPolicyArray>("/motor/policy_array", reliable(),
    [this](MotorPolicyArray::SharedPtr m) {on_policy(*m);}));
  subscriptions_.push_back(create_subscription<MotorTrajectory>("/motor/trajectory", reliable(),
    [this](MotorTrajectory::SharedPtr m) {on_trajectory(*m);}));
  subscriptions_.push_back(create_subscription<ConfigRequest>("/motor/config_request", reliable(),
    [this](ConfigRequest::SharedPtr m) {submit_config(*m);}));
  subscriptions_.push_back(create_subscription<ParamSetArray>("/motor/param_set", reliable(),
    [this](ParamSetArray::SharedPtr m) {
      ConfigRequest request;
      if (encode_config(*m, request).ok) {submit_config(request);}
    }));
  subscriptions_.push_back(create_subscription<SystemCommand>("/motor/system_command", reliable(),
    [this](SystemCommand::SharedPtr m) {
      ConfigRequest request;
      if (encode_config(*m, request).ok) {submit_config(request);}
    }));
  subscriptions_.push_back(create_subscription<Status>(W::TOPIC_STATUS, reliable(),
    [this](Status::SharedPtr m) {on_status(*m);}));
  subscriptions_.push_back(create_subscription<AgentInfo>(W::TOPIC_INFO, reliable(),
    [this](AgentInfo::SharedPtr m) {on_info(*m);}));
  subscriptions_.push_back(create_subscription<AgentMotion>(W::TOPIC_MOTION,
    rclcpp::QoS(2).best_effort(), [this](AgentMotion::SharedPtr m) {on_motion(*m);}));
  auto imu = create_publisher<sensor_msgs::msg::Imu>("/imu/data", stream());
  publishers_.push_back(imu);
  subscriptions_.push_back(create_subscription<sensor_msgs::msg::Imu>(W::TOPIC_IMU, stream(),
    [imu](sensor_msgs::msg::Imu::SharedPtr m) {imu->publish(*m);}));
  auto temp = create_publisher<std_msgs::msg::Float32>("/imu/temp", stream());
  publishers_.push_back(temp);
  subscriptions_.push_back(create_subscription<sensor_msgs::msg::Temperature>(
    W::TOPIC_TEMPERATURE, stream(), [temp](sensor_msgs::msg::Temperature::SharedPtr m) {
      std_msgs::msg::Float32 out; out.data = static_cast<float>(m->temperature); temp->publish(out);
    }));
  config_service_ = create_service<can_bridge_msgs::srv::ExecuteConfig>("/motor/execute_config",
    [this](Header header, std::shared_ptr<can_bridge_msgs::srv::ExecuteConfig::Request> request) {
      submit_config(request->request, header, request->timeout_ms ? request->timeout_ms : 5000);
    });
  timer_ = create_wall_timer(20ms, [this] {tick();});
}

bool BridgeNode::clock_stable()
{
  const auto now = Clock::now(); const auto wall = static_cast<int64_t>(epoch_ns());
  bool stable = true;
  if (have_clock_) {
    const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(now - clock_at_).count();
    stable = std::llabs(wall - clock_ns_ - elapsed) <= 50000000LL;
  }
  clock_at_ = now; clock_ns_ = wall; have_clock_ = true;
  if (!stable) {have_status_ = false; assembler_.reset();}
  return stable;
}
bool BridgeNode::source_time_valid(uint64_t stamp, std::chrono::milliseconds age)
{
  if (!clock_stable() || !stamp || stamp > static_cast<uint64_t>(INT64_MAX)) {return false;}
  const auto delta = clock_ns_ - static_cast<int64_t>(stamp);
  return delta >= -20000000LL && delta <=
    std::chrono::duration_cast<std::chrono::nanoseconds>(age).count();
}
bool BridgeNode::connected()
{return have_status_ && session_id_ != 0 && Clock::now() - status_at_ <= 300ms;}
bool BridgeNode::motion_ready()
{return clock_stable() && connected() && status_.clock_valid && have_info_;}

void BridgeNode::clear_session(uint64_t session)
{
  for (auto & job : configs_) {
    publish_reply(job, job.sent ? Reply::STATUS_PENDING : Reply::STATUS_FAILED, 21);
  }
  for (auto & job : reports_) {publish_reply(job, Reply::STATUS_FAILED, 21);}
  reports_.clear(); configs_.clear(); finished_configs_.clear();
  std::fill(std::begin(last_caller_ids_), std::end(last_caller_ids_), 0u);
  if (upload_) {trajectory_event(upload_->caller_id, 21);}
  upload_.reset(); assembler_.reset();
  session_id_ = session; have_status_ = have_info_ = querying_ = false; info_revision_ = 0;
  topology_ = MotorTopology{}; info_topology_ = MotorTopology{}; info_params_ = DeviceParamReport{};
  system_ = SystemStatus{}; freshness_.assign(65, MotorFreshness{});
  next_config_id_ = next_trajectory_request_ = next_trajectory_id_ = 1;
  active_wire_trajectory_ = active_caller_trajectory_ = 0;
  std::fill(std::begin(trajectory_targets_), std::end(trajectory_targets_), false);
  for (auto & target : last_targets_) {target.reset();}
  host_takeover_pending_ = host_execution_; takeover_stop_sent_ = false;
}

void BridgeNode::on_status(const Status & msg)
{
  if (msg.schema_version != AgentWireContract::SCHEMA_VERSION || !msg.session_id ||
    (session_id_ && msg.session_id < session_id_) || !valid_control_time(msg) ||
    (msg.clock_valid && !source_time_valid(msg.snapshot_stamp, 300ms))) {return;}
  if (msg.session_id == session_id_ && have_status_ &&
    static_cast<int32_t>(msg.sequence - status_.sequence) <= 0) {return;}
  MotorStatusArray statuses; MotorControlStateArray controls;
  MotorTrajectoryState trajectory;
  SystemStatus system = msg.session_id == session_id_ ? system_ : SystemStatus{};
  MotorDiagnostics diagnostics;
  const auto result = decode_agent_status(msg, statuses, controls, trajectory, system, diagnostics);
  if (!result.ok) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "V3状态整帧拒收：%s", result.error);
    return;
  }
  if (msg.session_id != session_id_) {clear_session(msg.session_id);}
  if (!have_status_) {command_sequence_ = msg.command_sequence;}
  if (have_status_ && (msg.motors.size() != status_.motors.size() ||
    !std::equal(msg.motors.begin(), msg.motors.end(), status_.motors.begin(),
      [](const auto & a, const auto & b) {return a.motor_id == b.motor_id;})))
  {assembler_.reset(); freshness_.assign(65, MotorFreshness{}); have_info_ = false;}
  status_ = msg; have_status_ = true; system_ = system;
  const auto age = msg.clock_valid ? std::max<int64_t>(0, clock_ns_ - static_cast<int64_t>(msg.snapshot_stamp)) : 0;
  status_at_ = Clock::now() - std::chrono::nanoseconds(age);
  if (host_execution_) {
    const bool trajectory_live = msg.trajectory.state == AgentTrajectoryState::LOADING ||
      msg.trajectory.state == AgentTrajectoryState::READY ||
      msg.trajectory.state == AgentTrajectoryState::RUNNING;
    if (trajectory_live) {host_takeover_pending_ = true;}
    if (host_takeover_pending_) {
      const bool disarmed = std::all_of(msg.motors.begin(), msg.motors.end(),
        [](const auto & motor) {return !motor.policy_enabled;});
      // 新来源先撤掉旧授权，后续完整状态确认后才允许重新武装。
      if (takeover_stop_sent_ && disarmed && !trajectory_live) {
        host_takeover_pending_ = false;
      } else {
        Config stop; stop.operation = Config::STOP_ALL; config_pub_->publish(stop);
        takeover_stop_sent_ = true;
      }
    }
  }
  next_config_id_ = std::max(next_config_id_, msg.config_request_high_water + 1u);
  next_trajectory_request_ = std::max(next_trajectory_request_, msg.trajectory_request_high_water + 1u);
  if (msg.trajectory.trajectory_id >= next_trajectory_id_) {
    next_trajectory_id_ = msg.trajectory.trajectory_id == UINT32_MAX ? 0 : msg.trajectory.trajectory_id + 1;
  }
  for (auto & c : controls.states) {
    const auto & f = freshness_[c.motor_id];
    if (f.state_age_ms != UINT32_MAX) {
      const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        Clock::now() - motion_at_).count();
      c.state_age_ms = static_cast<uint32_t>(std::min<uint64_t>(UINT32_MAX - 1u,
        static_cast<uint64_t>(f.state_age_ms) + static_cast<uint64_t>(elapsed)));
    }
  }
  if (active_wire_trajectory_ && trajectory.trajectory_id == active_wire_trajectory_) {
    trajectory.trajectory_id = active_caller_trajectory_;
  }
  status_pub_->publish(statuses); control_pub_->publish(controls);
  if (msg.trajectory.state != AgentTrajectoryState::LOADING &&
    msg.trajectory.state != AgentTrajectoryState::READY) {traj_state_pub_->publish(trajectory);}
  diagnostics_pub_->publish(diagnostics);
  config_results(msg); system_pub_->publish(system_);
  trajectory_receipt(msg);
  if (!have_info_ || msg.info_revision != info_revision_) {
    if (!querying_) {query_info(true);}
  }
  dispatch_config(); dispatch_trajectory();
}

void BridgeNode::query_info(bool restart)
{
  if (!connected()) {return;}
  if (restart || !querying_) {
    querying_ = true;
    info_topology_ = MotorTopology{}; info_params_ = DeviceParamReport{};
    query_ = Config{}; query_.operation = Config::QUERY_INFO;
    query_.session_id = session_id_; query_.query_id = ++query_id_;
    if (!query_id_) {query_.query_id = ++query_id_;}
  }
  info_sent_at_ = Clock::now(); config_pub_->publish(query_);
}
void BridgeNode::on_info(const AgentInfo & msg)
{
  if (!connected() || !querying_ || msg.session_id != session_id_ ||
    msg.schema_version != 3 || msg.query_id != query_.query_id ||
    msg.kind != AgentInfo::DEVICE_PAGE) {return;}
  if (msg.result == AgentInfo::RETRY) {query_info(true); return;}
  if (msg.cursor != query_.cursor || msg.result != AgentInfo::OK || (query_.query_revision &&
    msg.revision != query_.query_revision) ||
    (msg.clock_valid && !source_time_valid(msg.snapshot_stamp, 1500ms)) ||
    (!msg.clock_valid && msg.snapshot_stamp != 0)) {return;}
  if (!append_agent_info(msg, info_topology_, info_params_).ok) {return;}
  if (!msg.complete) {
    query_.query_revision = msg.revision; query_.cursor = msg.next_cursor;
    query_info(); return;
  }
  // Revision changes retire the entire table; never expose a mixed set of pages.
  if (msg.revision != status_.info_revision) {query_info(true); return;}
  querying_ = false; have_info_ = true; info_revision_ = msg.revision;
  topology_ = info_topology_; topology_pub_->publish(topology_); params_pub_->publish(info_params_);
  for (auto & report : reports_) {publish_reply(report, Reply::STATUS_SUCCEEDED, 0);}
  reports_.clear();
}
void BridgeNode::on_motion(const AgentMotion & msg)
{
  if (!motion_ready() || !source_time_valid(msg.snapshot_stamp, 100ms)) {return;}
  LowState frame;
  if (!agent_motion_frame(msg, status_.motors.size(), frame).ok) {return;}
  for (auto id : msg.motor_ids) {
    if (std::none_of(status_.motors.begin(), status_.motors.end(),
      [id](const auto & m) {return m.motor_id == id;})) {return;}
  }
  MotorMotionStateArray out;
  const auto delay = static_cast<uint32_t>(std::max<int64_t>(0,
    clock_ns_ - static_cast<int64_t>(msg.snapshot_stamp)) / 1000000);
  if (!assembler_.accept(frame, session_id_, Clock::now(), out, delay)) {return;}
  freshness_.assign(65, MotorFreshness{}); motion_at_ = Clock::now();
  for (const auto & s : out.states) {freshness_[s.motor_id] = {s.state_age_ms, s.target_source};}
  motion_pub_->publish(out);
}
void BridgeNode::on_command(const MotorCommandArray & msg)
{
  if (count_publishers(AgentWireContract::TOPIC_COMMAND) > 1) {
    Config stop; stop.operation = Config::STOP_ALL; config_pub_->publish(stop); return;
  }
  if (host_takeover_pending_ && msg.count) {return;}
  if (!motion_ready() || command_sub_->get_publisher_count() > 1 ||
    !source_time_valid(msg.sync_time_ns, 30ms)) {return;}
  AgentCommand out;
  auto valid = encode_agent_command(msg, out);
  if (!valid.ok) {return;}
  if (valid.saturated) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "命令有 %u 个通道被量程钳位", valid.saturated);
  }
  for (const auto & target : msg.targets) {
    if (std::none_of(topology_.entries.begin(), topology_.entries.end(),
      [&](const auto & e) {return e.motor_id == target.motor_id;})) {return;}
  }
  auto effective = msg;
  effective.targets.clear();
  for (const auto & target : msg.targets) {
    const auto id = target.motor_id;
    // Repeated pre-trajectory setpoints must not restart motion after V3's terminal soft stop.
    if (trajectory_targets_[id] && (!last_targets_[id] || *last_targets_[id] == target)) {continue;}
    trajectory_targets_[id] = false; last_targets_[id] = target;
    effective.targets.push_back(target);
  }
  effective.count = static_cast<uint16_t>(effective.targets.size());
  if (!encode_agent_command(effective, out).ok) {return;}
  const auto now = Clock::now();
  const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(now - status_at_).count();
  const auto age = clock_ns_ - static_cast<int64_t>(msg.sync_time_ns) +
    std::chrono::duration_cast<std::chrono::nanoseconds>(now - clock_at_).count();
  const auto deadline = command_deadline_tick(status_.snapshot_tick_ms, elapsed, age);
  if (!deadline) {return;}
  out.valid_until_tick_ms = *deadline; out.session_id = session_id_;
  out.sequence = ++command_sequence_; command_pub_->publish(out);
}

void BridgeNode::publish_reply(ConfigJob & job, uint8_t state, uint16_t error,
  uint64_t completed, uint64_t failed)
{
  Reply reply;
  reply.session_id = job.wire.session_id;
  reply.request_id = job.caller_id ? job.caller_id : job.wire.request_id;
  reply.kind = job.kind; reply.command = job.command;
  reply.status = state; reply.error_code = error;
  reply.completed_targets = static_cast<uint16_t>(bits(completed));
  reply.failed_targets = static_cast<uint16_t>(bits(failed));
  result_pub_->publish(reply);
  if (state == Reply::STATUS_PENDING) {
    if (job.header && error == 21) {
      can_bridge_msgs::srv::ExecuteConfig::Response response; response.result = reply;
      config_service_->send_response(*job.header, response); job.header.reset();
    }
    return;
  }
  if (job.header) {
    can_bridge_msgs::srv::ExecuteConfig::Response response; response.result = reply;
    config_service_->send_response(*job.header, response); job.header.reset();
  }
  if (job.wire.request_id && job.sent) {
    finished_configs_.push_back({job.original, reply, job.wire});
    if (finished_configs_.size() > 32) {finished_configs_.pop_front();}
  }
  MotorEvent event;
  event.session_id = reply.session_id; event.tick_ms = status_.snapshot_tick_ms;
  event.event_type = MotorEvent::EVENT_CONFIG_RESULT;
  event.motor_id = job.wire.motor_id;
  event.reason = error; event.request_id = reply.request_id;
  if (job.wire.operation == Config::SET_POLICY) {
    for (const auto & policy : job.wire.policies) {
      event.motor_id = policy.motor_id;
      const auto bit = UINT64_C(1) << (policy.motor_id - 1);
      event.reason = (completed & bit) && !(failed & bit) ? 0 : error;
      event.event_seq = ++event_sequence_; event_pub_->publish(event);
    }
  } else {event.event_seq = ++event_sequence_; event_pub_->publish(event);}
}
void BridgeNode::submit_config(const ConfigRequest & request, Header header, uint32_t timeout_ms)
{
  ConfigJob job; job.original = request; job.header = header;
  job.caller_id = request.request_id; job.kind = request.kind; job.command = request.command;
  job.deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
  auto valid = encode_agent_config(request, job.wire);
  job.wire.session_id = session_id_;
  if (!valid.ok || timeout_ms < 100 || timeout_ms > 10000) {
    publish_reply(job, Reply::STATUS_FAILED, 10); return;
  }
  if (job.wire.operation == Config::QUERY_INFO) {
    if (!connected()) {publish_reply(job, Reply::STATUS_FAILED, 21); return;}
    if (reports_.size() >= 8) {publish_reply(job, Reply::STATUS_BUSY, 0); return;}
    reports_.push_back(job); query_info(true); return;
  }
  if (!motion_ready() || !source_time_valid(request.sync_time_ns, 1000ms)) {
    publish_reply(job, Reply::STATUS_FAILED, 10); return;
  }
  job.wire.ttl_ms = 1000;
  enqueue_config(std::move(job));
}
void BridgeNode::on_policy(const MotorPolicyArray & msg)
{
  ConfigJob job;
  job.caller_id = msg.request_id;
  // Policy has no ConfigRequest kind in the legacy facade; events use the original request ID.
  job.kind = 2; job.command = Config::SET_POLICY;
  job.deadline = Clock::now() + 5s;
  if (!encode_agent_policy(msg, job.wire).ok) {return;}
  job.wire.session_id = session_id_;
  const bool disable = std::all_of(msg.targets.begin(), msg.targets.end(),
    [](const auto & p) {return !p.enabled;});
  if (disable) {
    // This bypasses the transaction slots and source-clock gate in the firmware.
    config_pub_->publish(job.wire); return;
  }
  if (host_takeover_pending_) {return;}
  if (!motion_ready() || !source_time_valid(msg.sync_time_ns, 100ms)) {return;}
  job.wire.ttl_ms = 100;
  enqueue_config(std::move(job));
}
void BridgeNode::enqueue_config(ConfigJob job)
{
  auto same = [&](Config a, Config b) {
      a.issued_stamp = b.issued_stamp; a.request_id = b.request_id;
      return a == b;
    };
  if (job.caller_id) {
    for (auto & previous : configs_) {
      if (previous.caller_id != job.caller_id || previous.kind != job.kind) {continue;}
      if (!same(previous.wire, job.wire)) {publish_reply(job, Reply::STATUS_CONFLICT, 10); return;}
      if (job.header) {publish_reply(job, Reply::STATUS_BUSY, 0);}
      return;
    }
    for (const auto & previous : finished_configs_) {
      if (previous.reply.request_id != job.caller_id || previous.reply.kind != job.kind) {continue;}
      if (!same(previous.wire, job.wire)) {publish_reply(job, Reply::STATUS_CONFLICT, 10); return;}
      result_pub_->publish(previous.reply);
      if (job.header) {
        can_bridge_msgs::srv::ExecuteConfig::Response response; response.result = previous.reply;
        config_service_->send_response(*job.header, response);
      }
      return;
    }
    if (job.caller_id >= 0x40000000u || job.kind > 2 ||
      job.caller_id <= last_caller_ids_[job.kind])
    {publish_reply(job, Reply::STATUS_CONFLICT, 10); return;}
  }
  if (configs_.size() >= 8) {publish_reply(job, Reply::STATUS_BUSY, 0); return;}
  if (!next_config_id_ || next_config_id_ > AgentWireContract::REQUEST_ID_MAX) {
    publish_reply(job, Reply::STATUS_CONFLICT, 10); return;
  }
  if (job.caller_id) {last_caller_ids_[job.kind] = job.caller_id;}
  job.wire.request_id = next_config_id_++;
  configs_.push_back(std::move(job)); dispatch_config();
}
void BridgeNode::dispatch_config()
{
  if (!connected()) {return;}
  for (auto & job : configs_) {
    if (job.accepted || job.wait_expired) {continue;}
    if (!job.sent || Clock::now() - job.sent_at >= 200ms) {
      if (!source_time_valid(job.wire.issued_stamp, std::chrono::milliseconds(job.wire.ttl_ms))) {
        return;
      }
      job.sent = true; job.sent_at = Clock::now(); config_pub_->publish(job.wire);
    }
    return;  // At most one request may await its first receipt.
  }
}
void BridgeNode::config_results(const Status & msg)
{
  Config ack; ack.session_id = session_id_; ack.operation = Config::ACK_RESULTS;
  auto consume = [&](const AgentConfigResult & result, bool receipt) {
      if (!result.request_id) {return;}
      auto found = std::find_if(configs_.begin(), configs_.end(), [&](const auto & j) {
          return j.wire.request_id == result.request_id && j.wire.operation == result.operation;
        });
      if (found != configs_.end()) {
        found->accepted = true;
        if (result.state == AgentConfigResult::PENDING) {
          return;
        }
        uint8_t mapped = Reply::STATUS_FAILED;
        if (result.state == AgentConfigResult::SUCCEEDED) {mapped = Reply::STATUS_SUCCEEDED;}
        if (result.state == AgentConfigResult::BUSY) {mapped = Reply::STATUS_BUSY;}
        if (result.state == AgentConfigResult::CONFLICT || result.state == AgentConfigResult::RETIRED) {
          mapped = Reply::STATUS_CONFLICT;
        }
        if (mapped == Reply::STATUS_SUCCEEDED) {
          switch (found->wire.operation) {
            case Config::SAVE: ++system_.save_seq; break;
            case Config::REFRESH_PARAMETERS: ++system_.refresh_seq; break;
            case Config::SET_ZERO: ++system_.set_zero_seq; break;
            case Config::CLEAR_FAULT: ++system_.clear_fault_seq; break;
            default: break;
          }
        }
        publish_reply(*found, mapped, result.error,
          result.completed_motor_mask, result.failed_motor_mask);
        configs_.erase(found);
      }
      if (!receipt && result.state != AgentConfigResult::PENDING) {
        ack.ack_request_ids.push_back(result.request_id);
      }
    };
  for (const auto & result : msg.results) {consume(result, false);}
  if (std::none_of(msg.results.begin(), msg.results.end(), [&](const auto & result) {
      return result.request_id == msg.rejection_receipt.request_id;
    })) {consume(msg.rejection_receipt, true);}
  if (!ack.ack_request_ids.empty()) {config_pub_->publish(ack);}
}
void BridgeNode::tick()
{
  const auto now = Clock::now();
  for (auto it = configs_.begin(); it != configs_.end();) {
    const bool expired_unsent = !it->sent &&
      !source_time_valid(it->wire.issued_stamp, std::chrono::milliseconds(it->wire.ttl_ms));
    if (expired_unsent || (now >= it->deadline && !it->sent)) {
      publish_reply(*it, Reply::STATUS_FAILED, 21); it = configs_.erase(it);
    } else {
      if (now >= it->deadline && !it->wait_expired) {
        // Waiting expiry does not cancel a device write; retain its bounded correlation until terminal.
        publish_reply(*it, Reply::STATUS_PENDING, 21); it->wait_expired = true;
      }
      ++it;
    }
  }
  for (auto it = reports_.begin(); it != reports_.end();) {
    if (now >= it->deadline) {
      publish_reply(*it, Reply::STATUS_FAILED, 21); it = reports_.erase(it);
    } else {++it;}
  }
  if (querying_ && connected() && now - info_sent_at_ >= 250ms) {query_info();}
  dispatch_config(); dispatch_trajectory();
}

void BridgeNode::trajectory_event(uint32_t caller_id, uint16_t error)
{
  MotorEvent event; event.session_id = session_id_; event.tick_ms = status_.snapshot_tick_ms;
  event.event_type = MotorEvent::EVENT_TRAJECTORY_REJECTED;
  // The legacy reason is a point index; a V3 transaction error has no such index.
  event.request_id = caller_id; event.reason = UINT16_MAX; event.event_seq = ++event_sequence_;
  RCLCPP_WARN(get_logger(), "V3轨迹 %u 拒绝/中断，原因码 %u", caller_id, error);
  event_pub_->publish(event);
}
void BridgeNode::on_trajectory(const MotorTrajectory & msg)
{
  const bool cancel = !msg.count && !msg.target_count && msg.points.empty() && msg.target_ids.empty();
  if (host_execution_) {
    if (cancel) {
      Config stop; stop.operation = Config::STOP_ALL; config_pub_->publish(stop);
    } else {trajectory_event(msg.trajectory_id, 2);}
    return;
  }
  if (cancel) {
    std::fill(std::begin(trajectory_targets_), std::end(trajectory_targets_), false);
    uint32_t id = upload_ ? upload_->wire_id : status_.trajectory.trajectory_id;
    if (upload_) {upload_.reset();}
    if (!session_id_ || !id || !connected()) {
      Config stop; stop.operation = Config::STOP_ALL; config_pub_->publish(stop); return;
    }
    if (next_trajectory_request_ > AgentWireContract::REQUEST_ID_MAX) {
      Config stop; stop.operation = Config::STOP_ALL; config_pub_->publish(stop); return;
    }
    Upload upload; upload.caller_id = active_caller_trajectory_ ? active_caller_trajectory_ : msg.trajectory_id;
    upload.wire_id = id; upload.complete = true;
    Trajectory request; request.session_id = session_id_; request.trajectory_id = id;
    request.request_id = next_trajectory_request_++; request.operation = Trajectory::CANCEL;
    upload.pending = request; upload_ = std::move(upload); dispatch_trajectory(); return;
  }
  auto reject = [&](uint16_t reason) {trajectory_event(msg.trajectory_id, reason);};
  if (!motion_ready() || !source_time_valid(msg.sync_time_ns, 1000ms) ||
    msg.count != msg.points.size() || msg.target_count != msg.target_ids.size() ||
    msg.count == 0 || msg.count > 256 || !msg.target_count || msg.target_count > 48 ||
    !msg.segment_count || msg.segment_count > 256 || msg.segment_index >= msg.segment_count)
  {reject(2); return;}
  if (status_.trajectory.state == AgentTrajectoryState::RUNNING ||
    (upload_ && upload_->complete)) {reject(4); return;}
  bool ids[65] = {};
  for (auto id : msg.target_ids) {
    if (!id || id > 64 || ids[id] || std::none_of(topology_.entries.begin(), topology_.entries.end(),
      [id](const auto & entry) {return entry.motor_id == id;})) {reject(14); return;}
    ids[id] = true;
  }
  for (const auto & p : msg.points) {
    if (!p.motor_id || p.motor_id > 64 || !ids[p.motor_id] || p.time_from_start_ms > INT32_MAX ||
      !std::isfinite(p.position) || !std::isfinite(p.velocity) || !std::isfinite(p.effort))
    {reject(8); return;}
  }
  if (!upload_) {
    if (msg.segment_index || !next_trajectory_id_) {reject(7); return;}
    Upload upload; upload.caller_id = msg.trajectory_id;
    upload.wire_id = std::max(next_trajectory_id_, next_trajectory_request_);
    next_trajectory_id_ = upload.wire_id + 1;
    upload.issued = msg.sync_time_ns; upload.host_segments = msg.segment_count;
    upload.ids.assign(msg.target_ids.begin(), msg.target_ids.end()); upload_ = std::move(upload);
  }
  auto & upload = *upload_;
  if (upload.caller_id != msg.trajectory_id || upload.host_segments != msg.segment_count ||
    upload.next_host_segment != msg.segment_index ||
    !std::equal(upload.ids.begin(), upload.ids.end(), msg.target_ids.begin(), msg.target_ids.end()) ||
    upload.points.size() + msg.points.size() > 256)
  {reject(7); return;}
  std::vector<MotorTrajectoryPoint> points = upload.points;
  points.insert(points.end(), msg.points.begin(), msg.points.end());
  uint32_t last_time[65] = {}; bool seen[65] = {};
  uint32_t global_time = 0;
  for (const auto & p : points) {
    if (p.time_from_start_ms < global_time || (seen[p.motor_id] &&
      p.time_from_start_ms <= last_time[p.motor_id])) {reject(8); upload_.reset(); return;}
    global_time = last_time[p.motor_id] = p.time_from_start_ms; seen[p.motor_id] = true;
  }
  upload.points = std::move(points); ++upload.next_host_segment;
  if (upload.next_host_segment != upload.host_segments) {return;}
  for (auto id : upload.ids) {if (!seen[id]) {reject(14); upload_.reset(); return;}}
  upload.complete = true;
  for (auto id : upload.ids) {trajectory_targets_[id] = true;}
  upload.segment_count = static_cast<uint16_t>((upload.points.size() + 31) / 32);
  active_wire_trajectory_ = upload.wire_id; active_caller_trajectory_ = upload.caller_id;
  dispatch_trajectory();
}
void BridgeNode::dispatch_trajectory()
{
  if (!upload_) {return;}
  auto & upload = *upload_;
  if (upload.pending && upload.pending->operation == Trajectory::CANCEL) {
    if (Clock::now() - upload.began >= 2s) {
      Config stop; stop.operation = Config::STOP_ALL; config_pub_->publish(stop);
      trajectory_event(upload.caller_id, 10); upload_.reset(); return;
    }
    if (Clock::now() - upload.last_sent >= 200ms) {
      trajectory_pub_->publish(*upload.pending); upload.last_sent = Clock::now();
    }
    return;
  }
  if (!motion_ready() || !source_time_valid(upload.issued, 1000ms)) {
    trajectory_event(upload.caller_id, 3); upload_.reset(); return;
  }
  if (!upload.complete) {return;}
  if (!upload.pending) {
    if (next_trajectory_request_ > AgentWireContract::REQUEST_ID_MAX) {
      trajectory_event(upload.caller_id, 5); upload_.reset(); return;
    }
    Trajectory request;
    request.session_id = session_id_; request.trajectory_id = upload.wire_id;
    request.request_id = next_trajectory_request_++; request.issued_stamp = upload.issued;
    request.ttl_ms = 1000;
    if (upload.next_point < upload.points.size()) {
      request.operation = Trajectory::LOAD;
      request.segment_index = static_cast<uint16_t>(upload.next_point / 32);
      request.segment_count = upload.segment_count;
      request.target_ids.assign(upload.ids.begin(), upload.ids.end());
      const auto end = std::min(upload.next_point + 32, upload.points.size());
      request.points.assign(upload.points.begin() + upload.next_point, upload.points.begin() + end);
    } else {request.operation = Trajectory::COMMIT;}
    upload.pending = std::move(request); upload.last_sent = TimePoint{};
  }
  if (Clock::now() - upload.last_sent >= 200ms) {
    trajectory_pub_->publish(*upload.pending); upload.last_sent = Clock::now();
  }
}
void BridgeNode::trajectory_receipt(const Status & msg)
{
  if (!upload_ || !upload_->pending) {return;}
  auto & upload = *upload_; const auto request = *upload.pending;
  const auto & receipt = msg.trajectory_request_receipt;
  if (receipt.request_id != request.request_id || receipt.trajectory_id != request.trajectory_id ||
    receipt.operation != request.operation) {return;}
  if (!receipt.accepted || receipt.error) {
    trajectory_event(upload.caller_id, receipt.error ? receipt.error : 2);
    upload_.reset();
  } else if (request.operation == Trajectory::LOAD) {
    if (receipt.received_segments != request.segment_index + 1) {return;}
    upload.next_point += request.points.size(); upload.pending.reset();
  } else {
    const auto state = msg.trajectory.state;
    const bool terminal = state == AgentTrajectoryState::SUCCEEDED ||
      state == AgentTrajectoryState::CANCELED || state == AgentTrajectoryState::ABORTED;
    if (state != AgentTrajectoryState::RUNNING && !terminal &&
      !(request.operation == Trajectory::CANCEL && state == AgentTrajectoryState::IDLE)) {return;}
    upload_.reset();
  }
  Trajectory ack; ack.operation = Trajectory::ACK; ack.session_id = session_id_;
  ack.ack_request_id = request.request_id;
  trajectory_pub_->publish(ack);
}
}  // namespace micro_ros_bridge
