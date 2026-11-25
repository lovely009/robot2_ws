#include "motor_sim/ros/motor_sim_system.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace motor_sim::ros {

namespace {
constexpr double kDefaultInertia = 0.02;
constexpr double kDefaultDamping = 0.2;
}  // namespace

hardware_interface::CallbackReturn MotorSimSystem::on_init(
    const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto it = info_.hardware_parameters.find("config_file");
  if (it == info_.hardware_parameters.end() || it->second.empty()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    model_ = std::make_unique<robot_transmission::Model>(
        robot_transmission::Model::load(it->second));
  } catch (const std::exception &) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(it->second);
  } catch (const std::exception &) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  const YAML::Node motors_node = root["motor_hardware"]["motors"];

  const std::size_t n_joints = info_.joints.size();
  position_.assign(n_joints, 0.0);
  velocity_.assign(n_joints, 0.0);
  effort_.assign(n_joints, 0.0);
  cmd_position_.assign(n_joints, 0.0);
  cmd_velocity_.assign(n_joints, 0.0);
  cmd_effort_.assign(n_joints, 0.0);
  joint_indices_.clear();
  for (std::size_t i = 0; i < n_joints; ++i) {
    joint_indices_[info_.joints[i].name] = i;
  }
  for (const auto &jname : model_->joint_names()) {
    if (joint_indices_.find(jname) == joint_indices_.end()) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  actuators_.clear();
  for (const auto &act_name : model_->actuator_names()) {
    ActuatorSim sim;
    sim.name = act_name;
    const YAML::Node mnode = motors_node[act_name];
    if (mnode && mnode["command"]) {
      if (mnode["command"]["kp"]) sim.kp = mnode["command"]["kp"].as<double>();
      if (mnode["command"]["kd"]) sim.kd = mnode["command"]["kd"].as<double>();
    }
    if (mnode && mnode["type"] &&
        mnode["type"].as<std::string>() == "cerb_r48_101") {
      sim.effort_limit = 12.0;
    } else if (mnode && mnode["type"] &&
               mnode["type"].as<std::string>() == "vaxor_2550") {
      sim.effort_limit = 12.0;
      if (!mnode["command"] || !mnode["command"]["kp"]) sim.kp = 0.008;
    }
    core::PlantConfig cfg;
    cfg.kp = sim.kp;
    cfg.kd = sim.kd;
    cfg.effort_limit_nm = sim.effort_limit;
    cfg.torque_constant_nm_per_a = 0.1;
    cfg.current_limit_a = 30.0;
    sim.plant = std::make_unique<core::MotorPlantModel>(cfg);
    actuators_.push_back(std::move(sim));
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MotorSimSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(info_.joints.size() * 3U);
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    const auto &name = info_.joints[i].name;
    interfaces.emplace_back(name, hardware_interface::HW_IF_POSITION, &position_[i]);
    interfaces.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &velocity_[i]);
    interfaces.emplace_back(name, hardware_interface::HW_IF_EFFORT, &effort_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
MotorSimSystem::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(info_.joints.size() * 3U);
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    const auto &name = info_.joints[i].name;
    interfaces.emplace_back(name, hardware_interface::HW_IF_POSITION, &cmd_position_[i]);
    interfaces.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &cmd_velocity_[i]);
    interfaces.emplace_back(name, hardware_interface::HW_IF_EFFORT, &cmd_effort_[i]);
  }
  return interfaces;
}

hardware_interface::CallbackReturn MotorSimSystem::on_activate(
    const rclcpp_lifecycle::State &) {
  cmd_position_ = position_;
  std::fill(cmd_velocity_.begin(), cmd_velocity_.end(), 0.0);
  std::fill(cmd_effort_.begin(), cmd_effort_.end(), 0.0);
  active_ = true;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MotorSimSystem::on_deactivate(
    const rclcpp_lifecycle::State &) {
  active_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MotorSimSystem::read(
    const rclcpp::Time &, const rclcpp::Duration &) {
  robot_transmission::AxisMap act_map;
  for (const auto &act : actuators_) {
    act_map[act.name] = robot_transmission::Axis(act.position, act.velocity, act.effort);
  }
  const auto joint_map = model_->actuator_to_joint(act_map);
  for (const auto &[jname, axis] : joint_map) {
    const auto it = joint_indices_.find(jname);
    if (it != joint_indices_.end()) {
      position_[it->second] = axis.position;
      velocity_[it->second] = axis.velocity;
      effort_[it->second] = axis.effort;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MotorSimSystem::write(
    const rclcpp::Time &, const rclcpp::Duration &period) {
  if (!active_) {
    return hardware_interface::return_type::OK;
  }
  const double dt = std::clamp(period.seconds(), 1e-4, 0.05);

  robot_transmission::AxisMap joint_cmds;
  for (const auto &jname : model_->joint_names()) {
    const std::size_t idx = joint_indices_.at(jname);
    const double p = std::isfinite(cmd_position_[idx]) ? cmd_position_[idx] : position_[idx];
    const double v = std::isfinite(cmd_velocity_[idx]) ? cmd_velocity_[idx] : 0.0;
    const double e = std::isfinite(cmd_effort_[idx]) ? cmd_effort_[idx] : 0.0;
    joint_cmds[jname] = robot_transmission::Axis(p, v, e);
  }
  const auto act_cmds = model_->joint_to_actuator(joint_cmds);

  for (auto &act : actuators_) {
    const auto it = act_cmds.find(act.name);
    if (it == act_cmds.end()) continue;
    protocol::MotorCommand cmd;
    cmd.enabled = true;
    cmd.mode = protocol::CommandMode::Mit;
    cmd.position = it->second.position;
    cmd.velocity = it->second.velocity;
    cmd.feedforward = it->second.effort;
    cmd.kp = act.kp;
    cmd.kd = act.kd;
    const auto out = act.plant->step(cmd, {act.position, act.velocity}, dt);
    act.effort = out.effort_nm;
    const double accel = (act.effort - kDefaultDamping * act.velocity) / kDefaultInertia;
    act.velocity += accel * dt;
    act.position += act.velocity * dt;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace motor_sim::ros

PLUGINLIB_EXPORT_CLASS(motor_sim::ros::MotorSimSystem, hardware_interface::SystemInterface)
