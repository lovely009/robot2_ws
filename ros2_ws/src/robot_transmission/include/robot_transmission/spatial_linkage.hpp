#pragma once

#include <array>
#include <hardware_interface/hardware_info.hpp>
#include <transmission_interface/transmission_loader.hpp>
#include "robot_transmission/model.hpp"

namespace robot_transmission {
class SpatialLinkageTransmission : public transmission_interface::Transmission {
public:
  explicit SpatialLinkageTransmission(const hardware_interface::TransmissionInfo &info);
  void configure(const std::vector<transmission_interface::JointHandle> &joints,
                 const std::vector<transmission_interface::ActuatorHandle> &actuators) override;
  void actuator_to_joint() override;
  void joint_to_actuator() override;
  std::size_t num_actuators() const override { return 2; }
  std::size_t num_joints() const override { return 2; }
private:
  using Vec = std::array<double,3>;
  Vec motor_origin_, motor_axis_, motor_ball_, origin1_, axis1_, origin2_, axis2_, joint_ball_;
  double length_, scale_, phase_, pitch_derivative_sign_, motor_derivative_sign_;
  std::array<double,2> signs_;
  std::array<std::string,2> actuator_names_, joint_names_;
  std::vector<transmission_interface::JointHandle> joints_;
  std::vector<transmission_interface::ActuatorHandle> actuators_;
  bool configured_{false};
  Vec motor_ball(double t) const;
  Vec joint_ball(double q1, double q2) const;
  void check_motor_branch(double t) const;
  std::array<double,3> derivatives(double q1, double q2, double t) const;
  double forward(double q1, double t) const;
  double inverse(double q1, double q2) const;
  void convert(bool to_joint);
};

class SpatialLinkageTransmissionLoader : public transmission_interface::TransmissionLoader {
public:
  std::shared_ptr<transmission_interface::Transmission> load(
    const hardware_interface::TransmissionInfo &info) override;
};
}  // namespace robot_transmission
