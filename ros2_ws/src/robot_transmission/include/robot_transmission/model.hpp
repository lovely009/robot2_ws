#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace robot_transmission {
struct Axis {
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
  Axis() = default;
  Axis(double p, double v = 0.0, double e = 0.0) : position(p), velocity(v), effort(e) {}
};
using AxisMap = std::map<std::string, Axis>;
struct Group {
  std::string name;
  std::string plugin;
  std::vector<std::string> actuators;
  std::vector<std::string> joints;
  std::vector<double> actuator_reduction;
  std::vector<double> joint_reduction;
  std::vector<double> joint_offset;
  std::map<std::string, std::string> parameters;
};
// 单线程使用；调用方过滤过期反馈，命令必须完整覆盖被触及的传动组。
class Model {
public:
  static Model load(const std::string &driver_file, const std::string &transmissions_file = "");
  ~Model();
  Model(Model &&) noexcept;
  Model &operator=(Model &&) noexcept;
  Model(const Model &) = delete;
  Model &operator=(const Model &) = delete;
  const std::vector<std::string> &actuator_names() const;
  const std::vector<std::string> &joint_names() const;
  const std::map<std::string, int> &actuator_ids() const;
  const std::vector<Group> &groups() const;
  AxisMap actuator_to_joint(const AxisMap &state) const;
  AxisMap joint_to_actuator(const AxisMap &state) const;
  std::map<std::string, double> joint_effort_to_actuator(const std::map<std::string, double> &effort) const;
  std::map<std::string, double> actuator_effort_to_joint(const std::map<std::string, double> &effort) const;
  std::vector<std::string> actuators_for_joint(const std::string &joint) const;
private:
  struct Impl;
  explicit Model(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};
}  // namespace robot_transmission
