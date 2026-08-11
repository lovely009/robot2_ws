#include <gtest/gtest.h>

#include <chrono>
#include <stdexcept>
#include <string>

#include "motor_canfd_sim/plant_model.hpp"

namespace {

namespace drive = motor_canfd_drive;
using motor_canfd_sim::JointSample;
using motor_canfd_sim::MotorPlantModel;
using motor_canfd_sim::SimMotorState;
using drive::protocol::JointCommand;
using drive::protocol::WristControl;
using drive::protocol::WristMode;

using Clock = std::chrono::steady_clock;

MotorPlantModel make_plant() {
  return MotorPlantModel(0.15, 60.0, 11.0, 5.0, 5.0, 0.08, 0.45);
}

drive::MotorConfig make_motor(const std::string &name, const std::string &type) {
  drive::MotorConfig motor;
  motor.name = name;
  motor.joint_name = name + "_joint";
  motor.bus = "left_arm";
  motor.type = type;
  motor.id = 1U;
  motor.parameters["torque_constant_nm_per_amp"] = "0.35";
  return motor;
}

SimMotorState fresh_state(const JointCommand &command) {
  SimMotorState state;
  state.enabled = true;
  state.command = command;
  state.last_command = Clock::now();
  return state;
}

TEST(PlantModel, RpComputesMitTorque) {
  auto plant = make_plant();
  auto state = fresh_state(JointCommand{0.5, 0.0, 30.0, 1.0, 0.0});
  JointSample sample;
  EXPECT_DOUBLE_EQ(plant.compute_effort(make_motor("rp", drive::kRpMotorType), state,
                                        sample, Clock::now()),
                   15.0);
}

TEST(PlantModel, RpClampsTorqueToLimit) {
  auto plant = make_plant();
  auto state = fresh_state(JointCommand{10.0, 0.0, 30.0, 0.0, 0.0});
  JointSample sample;
  EXPECT_DOUBLE_EQ(plant.compute_effort(make_motor("rp", drive::kRpMotorType), state,
                                        sample, Clock::now()),
                   60.0);
}

TEST(PlantModel, RpDisabledYieldsZeroTorque) {
  auto plant = make_plant();
  auto state = fresh_state(JointCommand{0.5, 0.0, 30.0, 0.0, 0.0});
  state.enabled = false;
  JointSample sample;
  EXPECT_DOUBLE_EQ(plant.compute_effort(make_motor("rp", drive::kRpMotorType), state,
                                        sample, Clock::now()),
                   0.0);
}

TEST(PlantModel, RpStaleCommandYieldsZeroTorque) {
  auto plant = make_plant();
  auto state = fresh_state(JointCommand{0.5, 0.0, 30.0, 0.0, 0.0});
  state.last_command = Clock::now() - std::chrono::seconds(1);
  JointSample sample;
  EXPECT_DOUBLE_EQ(plant.compute_effort(make_motor("rp", drive::kRpMotorType), state,
                                        sample, Clock::now()),
                   0.0);
}

TEST(PlantModel, RpIncludesFeedforwardTerm) {
  auto plant = make_plant();
  auto state = fresh_state(JointCommand{0.5, 0.0, 30.0, 0.0, 10.0});
  JointSample sample;
  sample.position = 0.5;
  EXPECT_DOUBLE_EQ(plant.compute_effort(make_motor("rp", drive::kRpMotorType), state,
                                        sample, Clock::now()),
                   10.0);
}

TEST(PlantModel, WristMitConvertsCurrentToTorqueViaKt) {
  auto plant = make_plant();
  auto state = fresh_state(JointCommand{0.0, 0.0, 0.0, 0.0, 2.0});
  JointSample sample;
  const auto torque =
      plant.compute_effort(make_motor("wrist", drive::kMotor2530Type), state, sample,
                           Clock::now());
  EXPECT_DOUBLE_EQ(torque, 2.0 * 0.35);
  EXPECT_DOUBLE_EQ(state.current_a, 2.0);
}

TEST(PlantModel, WristMitPositionErrorDrivesPositiveTorque) {
  auto plant = make_plant();
  auto state = fresh_state(JointCommand{0.5, 0.0, 0.008, 0.0, 0.0});
  JointSample sample;
  EXPECT_GT(plant.compute_effort(make_motor("wrist", drive::kMotor2530Type), state,
                                 sample, Clock::now()),
            0.0);
}

TEST(PlantModel, WristCurrentClampedBeforeTorqueConversion) {
  auto plant = make_plant();
  auto state = fresh_state(JointCommand{0.0, 0.0, 0.0, 0.0, 100.0});
  JointSample sample;
  const auto torque =
      plant.compute_effort(make_motor("wrist", drive::kMotor2530Type), state, sample,
                           Clock::now());
  EXPECT_DOUBLE_EQ(state.current_a, 11.0);
  EXPECT_DOUBLE_EQ(torque, 11.0 * 0.35);
}

TEST(PlantModel, WristTorqueClampedToLimit) {
  auto plant = MotorPlantModel(0.15, 60.0, 100.0, 5.0, 5.0, 0.08, 0.45);
  auto state = fresh_state(JointCommand{0.0, 0.0, 0.0, 0.0, 100.0});
  JointSample sample;
  EXPECT_DOUBLE_EQ(plant.compute_effort(make_motor("wrist", drive::kMotor2530Type),
                                        state, sample, Clock::now()),
                   5.0);
}

TEST(PlantModel, WristPositionModeUsesRelativeError) {
  auto plant = make_plant();
  auto state = fresh_state(JointCommand{0.5, 0.0, 0.0, 0.0, 0.0});
  state.mode = WristMode::Position;
  JointSample sample;
  const auto torque =
      plant.compute_effort(make_motor("wrist", drive::kMotor2530Type), state, sample,
                           Clock::now());
  EXPECT_DOUBLE_EQ(torque, 2.5 * 0.35);
}

TEST(PlantModel, WristPositionModeIncludesDamping) {
  auto plant = make_plant();
  auto state = fresh_state(JointCommand{0.5, 0.0, 0.0, 0.0, 0.0});
  state.mode = WristMode::Position;
  JointSample sample;
  sample.velocity = 2.0;
  const auto torque =
      plant.compute_effort(make_motor("wrist", drive::kMotor2530Type), state, sample,
                           Clock::now());
  EXPECT_DOUBLE_EQ(torque, (2.5 - 0.08 * 2.0) * 0.35);
}

TEST(PlantModel, WristVelocityModeUsesVelocityError) {
  auto plant = make_plant();
  auto state = fresh_state(JointCommand{0.0, 1.0, 0.0, 0.0, 0.0});
  state.mode = WristMode::Velocity;
  JointSample sample;
  const auto torque =
      plant.compute_effort(make_motor("wrist", drive::kMotor2530Type), state, sample,
                           Clock::now());
  EXPECT_DOUBLE_EQ(torque, 0.45 * 0.35);
}

TEST(PlantModel, WristSetZeroRebasesRelativePosition) {
  auto plant = make_plant();
  auto state = fresh_state(JointCommand{0.0, 0.0, 0.0, 0.0, 0.0});
  JointSample sample;
  sample.position = 0.7;
  plant.handle_motor_2530_control(
      state, WristControl{false, true, WristMode::Mit}, sample);
  ASSERT_FALSE(state.enabled);
  EXPECT_DOUBLE_EQ(state.zero_offset, 0.7);

  state.enabled = true;
  state.mode = WristMode::Position;
  // 置零后当前位置成为坐标零点：命令 0 表示保持当前姿态。
  state.command = JointCommand{0.0, 0.0, 0.0, 0.0, 0.0};
  state.last_command = Clock::now();
  EXPECT_DOUBLE_EQ(plant.compute_effort(make_motor("wrist", drive::kMotor2530Type),
                                        state, sample, Clock::now()),
                   0.0);
  // 偏离置零点 0.2 rad 且命令为 0：误差 -0.2，输出 -0.35 Nm（反向纠偏）。
  sample.position = 0.9;
  EXPECT_DOUBLE_EQ(plant.compute_effort(make_motor("wrist", drive::kMotor2530Type),
                                        state, sample, Clock::now()),
                   -0.35);
}

TEST(PlantModel, WristDisableClearsCurrent) {
  auto plant = make_plant();
  auto state = fresh_state(JointCommand{0.0, 0.0, 0.0, 0.0, 2.0});
  JointSample sample;
  static_cast<void>(plant.compute_effort(make_motor("wrist", drive::kMotor2530Type),
                                         state, sample, Clock::now()));
  EXPECT_DOUBLE_EQ(state.current_a, 2.0);

  plant.handle_motor_2530_control(state, WristControl{false, false, WristMode::Mit},
                                  sample);
  EXPECT_FALSE(state.enabled);
  EXPECT_DOUBLE_EQ(state.current_a, 0.0);
}

TEST(PlantModel, MissingTorqueConstantThrows) {
  auto plant = make_plant();
  auto state = fresh_state(JointCommand{});
  JointSample sample;
  auto motor = make_motor("wrist", drive::kMotor2530Type);
  motor.parameters.clear();
  EXPECT_THROW(plant.compute_effort(motor, state, sample, Clock::now()),
               std::runtime_error);
}

TEST(PlantModel, InvalidTorqueConstantThrows) {
  auto plant = make_plant();
  auto state = fresh_state(JointCommand{});
  JointSample sample;
  auto motor = make_motor("wrist", drive::kMotor2530Type);
  motor.parameters["torque_constant_nm_per_amp"] = "not-a-number";
  EXPECT_THROW(plant.compute_effort(motor, state, sample, Clock::now()),
               std::invalid_argument);
}

} // namespace
