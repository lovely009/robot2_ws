#include "humanoid_gazebo/motor_sim/plant_model.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

namespace motor_sim = humanoid_gazebo::motor_sim;
namespace protocol = humanoid_gazebo::protocol;

namespace {

protocol::MotorCommand position_command(const double position) {
  protocol::MotorCommand command;
  command.enabled = true;
  command.mode = protocol::CommandMode::Mit;
  command.position = position;
  command.use_nominal_gains = true;
  return command;
}

}  // namespace

TEST(PlantModel, SaturatesEffortAtJointLimit) {
  motor_sim::MotorPlantModel plant({30.0, 1.0, 8.0, 0.0, 0.0});
  const auto output =
      plant.step(position_command(1.0), {0.0, 0.0}, 0.01);
  EXPECT_NEAR(output.effort_nm, 8.0, 1e-12);
}

TEST(PlantModel, UsesVelocityDampingAndFeedforward) {
  motor_sim::MotorPlantModel plant({20.0, 2.0, 20.0, 0.0, 0.0});
  auto command = position_command(0.5);
  command.velocity = 0.2;
  command.feedforward = 1.0;
  const auto output = plant.step(command, {0.1, -0.3}, 0.01);
  EXPECT_NEAR(output.effort_nm, 10.0, 1e-12);
}

TEST(PlantModel, DisabledCommandProducesZeroEffort) {
  motor_sim::MotorPlantModel plant({30.0, 1.0, 8.0, 0.0, 0.0});
  auto command = position_command(1.0);
  command.enabled = false;
  EXPECT_DOUBLE_EQ(plant.step(command, {0.0, 0.0}, 0.01).effort_nm, 0.0);
}

TEST(PlantModel, RejectsNonPositiveOrNonFiniteTimeStep) {
  motor_sim::MotorPlantModel plant({30.0, 1.0, 8.0, 0.0, 0.0});
  EXPECT_THROW(plant.step(position_command(1.0), {0.0, 0.0}, 0.0),
               std::invalid_argument);
  EXPECT_THROW(plant.step(position_command(1.0), {0.0, 0.0}, -0.01),
               std::invalid_argument);
}
