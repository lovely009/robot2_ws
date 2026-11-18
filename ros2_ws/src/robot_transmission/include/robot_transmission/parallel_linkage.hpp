#pragma once
#include <array>
#include <hardware_interface/hardware_info.hpp>
#include <transmission_interface/transmission_loader.hpp>

namespace robot_transmission {
// Two serial output hinges driven by two fixed-length rods, or one direct
// first hinge and one rod. Geometry is expressed in the same zero-pose frame.
class ParallelLinkageTransmission : public transmission_interface::Transmission {
public:
  explicit ParallelLinkageTransmission(const hardware_interface::TransmissionInfo &info);
  void configure(const std::vector<transmission_interface::JointHandle> &joints,
                 const std::vector<transmission_interface::ActuatorHandle> &actuators) override;
  void actuator_to_joint() override;
  void joint_to_actuator() override;
  std::size_t num_actuators() const override { return 2; }
  std::size_t num_joints() const override { return 2; }
private:
  using V = std::array<double,3>;
  using Pair = std::array<double,2>;
  using Matrix = std::array<Pair,2>;
  struct Rod { V origin, axis, ball, platform; double length, sign, lower, upper; };
  std::array<Rod,2> rods_;
  std::array<V,2> origins_, axes_;
  Pair signs_;
  std::array<std::string,2> joint_names_, actuator_names_;
  std::vector<transmission_interface::JointHandle> joints_;
  std::vector<transmission_interface::ActuatorHandle> actuators_;
  bool direct_first_{false}, configured_{false};
  V platform(std::size_t i, Pair q) const;
  std::array<double,4> constraint(std::size_t i, Pair q, double t) const;
  Pair inverse(Pair q) const;
  Pair forward(Pair t) const;
  Matrix jacobian(Pair q, Pair t) const;
  void convert(bool to_joint);
};
class ParallelLinkageTransmissionLoader : public transmission_interface::TransmissionLoader {
public:
  std::shared_ptr<transmission_interface::Transmission> load(
    const hardware_interface::TransmissionInfo &info) override;
};
}
