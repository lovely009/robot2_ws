#include "humanoid_gazebo/motor_sim/motor_sim_node.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

namespace motor_sim = humanoid_gazebo::motor_sim;
namespace transport = humanoid_gazebo::transport;

namespace {

motor_sim::NodeParameters valid_parameters() {
  motor_sim::NodeParameters parameters;
  parameters.transport_mode = transport::TransportMode::UsbSerial;
  parameters.ports = {
      {"left_arm", "/dev/serial/by-id/usb-left"},
      {"right_arm", "/dev/serial/by-id/usb-right"},
      {"legs", "/dev/serial/by-id/usb-legs"},
  };
  parameters.baud_rate = 3000000;
  parameters.config_file = "/tmp/motors.yaml";
  return parameters;
}

}  // namespace

TEST(MotorSimNodeParameters, AcceptsThreeStableUniqueUsbPorts) {
  EXPECT_NO_THROW(motor_sim::validate_node_parameters(valid_parameters()));
}

TEST(MotorSimNodeParameters, RejectsUnstableOrDuplicatePorts) {
  auto unstable = valid_parameters();
  unstable.ports[0].second = "/dev/ttyACM0";
  EXPECT_THROW(motor_sim::validate_node_parameters(unstable),
               std::invalid_argument);

  auto duplicate = valid_parameters();
  duplicate.ports[2].second = duplicate.ports[0].second;
  EXPECT_THROW(motor_sim::validate_node_parameters(duplicate),
               std::invalid_argument);
}

TEST(MotorSimNodeParameters, SerialSlcanRequiresExplicitOptIn) {
  auto parameters = valid_parameters();
  parameters.transport_mode = transport::TransportMode::SerialSlcan;
  EXPECT_THROW(motor_sim::validate_node_parameters(parameters),
               std::invalid_argument);
  parameters.allow_external_canfd = true;
  EXPECT_NO_THROW(motor_sim::validate_node_parameters(parameters));
}

TEST(MotorSimNodeParameters, RejectsMissingBusAndConfig) {
  auto missing_bus = valid_parameters();
  missing_bus.ports.pop_back();
  EXPECT_THROW(motor_sim::validate_node_parameters(missing_bus),
               std::invalid_argument);

  auto missing_config = valid_parameters();
  missing_config.config_file.clear();
  EXPECT_THROW(motor_sim::validate_node_parameters(missing_config),
               std::invalid_argument);
}
