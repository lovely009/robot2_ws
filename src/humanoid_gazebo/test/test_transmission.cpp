#include "humanoid_gazebo/motor_sim/transmission.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

namespace motor_sim = humanoid_gazebo::motor_sim;

TEST(Transmission, RoundTripsMotorAndJointPositionAndVelocity) {
  const motor_sim::Transmission transmission({36.0, 0.80, -1.0, 0.1});
  const double motor_position =
      transmission.joint_to_motor_position(0.4);
  EXPECT_NEAR(motor_position, -10.8, 1e-12);
  EXPECT_NEAR(transmission.motor_to_joint_position(motor_position), 0.4,
              1e-12);
  const double motor_velocity =
      transmission.joint_to_motor_velocity(-0.5);
  EXPECT_NEAR(motor_velocity, 18.0, 1e-12);
  EXPECT_NEAR(transmission.motor_to_joint_velocity(motor_velocity), -0.5,
              1e-12);
}

TEST(Transmission, ConvertsTorqueWithRatioEfficiencyAndDirection) {
  const motor_sim::Transmission transmission({36.0, 0.80, -1.0, 0.0});
  EXPECT_NEAR(transmission.motor_torque_to_joint_effort(0.1), -2.88, 1e-12);
  EXPECT_NEAR(transmission.joint_effort_to_motor_torque(-2.88), 0.1, 1e-12);
}

TEST(Transmission, RejectsInvalidMechanicalParameters) {
  EXPECT_THROW(motor_sim::Transmission({0.0, 0.80, 1.0, 0.0}),
               std::invalid_argument);
  EXPECT_THROW(motor_sim::Transmission({36.0, 1.20, 1.0, 0.0}),
               std::invalid_argument);
  EXPECT_THROW(motor_sim::Transmission({36.0, 0.80, 0.0, 0.0}),
               std::invalid_argument);
}
