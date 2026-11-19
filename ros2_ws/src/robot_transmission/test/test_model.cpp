#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <unistd.h>
#include "robot_transmission/model.hpp"
#include "spatial_fixture.hpp"

namespace rt = robot_transmission;
class ModelTest : public ::testing::Test {
protected:
  std::filesystem::path root;
  void SetUp() override {
    root = std::filesystem::temp_directory_path() / ("transmission_test_" + std::to_string(getpid()));
    std::filesystem::create_directories(root);
    std::ofstream(root / "driver.yaml") << "motor_hardware:\n  motors:\n    a: {id: 8, joint: yaw}\n    b: {id: 9, joint: pitch}\n    c: {id: 1, joint: shoulder}\n";
  }
  void TearDown() override { std::filesystem::remove_all(root); }
  rt::Model model(const std::string &extra = "") {
    std::ofstream(root / "transmissions.yaml") << "transmissions:\n  wrist:\n    plugin: transmission_interface/DifferentialTransmission\n    actuators: [a,b]\n    joints: [yaw,pitch]\n    actuator_reduction: [1,1]\n    joint_reduction: [1,1]\n    joint_offset: [0,0]\n" << extra;
    return rt::Model::load((root / "driver.yaml").string());
  }
};
TEST_F(ModelTest, SameAndOppositeMotionDecodeBothJoints) {
  auto m = model();
  auto same = m.actuator_to_joint({{"a",rt::Axis(0.2)}, {"b",rt::Axis(0.2)}});
  EXPECT_NEAR(same.at("yaw").position,0.2,1e-12);
  EXPECT_NEAR(same.at("pitch").position,0,1e-12);
  auto opposite = m.actuator_to_joint({{"a",rt::Axis(0.2)}, {"b",rt::Axis(-0.2)}});
  EXPECT_NEAR(opposite.at("yaw").position,0,1e-12);
  EXPECT_NEAR(opposite.at("pitch").position,0.2,1e-12);
}
TEST_F(ModelTest, MixedCommandAndPowerArePreserved) {
  auto m = model();
  auto a = m.joint_to_actuator({{"yaw",rt::Axis(0.2,0.3,2)}, {"pitch",rt::Axis(0.1,0.2,4)}});
  EXPECT_NEAR(a.at("a").position,0.3,1e-12);
  EXPECT_NEAR(a.at("b").position,0.1,1e-12);
  EXPECT_NEAR(a.at("a").effort,3,1e-12);
  EXPECT_NEAR(a.at("b").effort,-1,1e-12);
  EXPECT_NEAR(a.at("a").effort*a.at("a").velocity + a.at("b").effort*a.at("b").velocity,1.4,1e-12);
  auto j=m.actuator_to_joint(a);
  EXPECT_NEAR(j.at("pitch").effort,4,1e-12);
  EXPECT_EQ(m.actuator_ids().at("a"),8);
  EXPECT_EQ(m.actuators_for_joint("yaw"),(std::vector<std::string>{"a","b"}));
}
TEST_F(ModelTest, MissingActuatorDoesNotInventEitherJoint) {
  auto m=model();
  auto j=m.actuator_to_joint({{"a",rt::Axis(0.1)}, {"c",rt::Axis(0.4)}});
  ASSERT_EQ(j.size(),1U);
  EXPECT_NEAR(j.at("shoulder").position,0.4,1e-12);
  EXPECT_THROW(m.joint_to_actuator({{"yaw",rt::Axis(0.1)}}),std::invalid_argument);
}
TEST_F(ModelTest, DuplicateOwnershipAndInvalidPluginFailClosed) {
  EXPECT_THROW(model("  other:\n    plugin: transmission_interface/SimpleTransmission\n    actuators: [a]\n    joints: [other]\n"),std::invalid_argument);
  EXPECT_THROW(model("  other:\n    plugin: does_not_exist\n    actuators: [c]\n    joints: [shoulder]\n"),std::exception);
}
TEST_F(ModelTest, DeclaredMissingFileCannotFallBackToDirectMapping) {
  std::ofstream(root/"driver.yaml") << "motor_hardware:\n  transmissions_file: missing.yaml\n  motors:\n    a: {id: 8, joint: yaw}\n";
  EXPECT_THROW(rt::Model::load((root/"driver.yaml").string()),std::exception);
}
TEST_F(ModelTest, LegacyMappingWithoutTransmissionFileIsIdentity) {
  auto m=rt::Model::load((root/"driver.yaml").string());
  auto j=m.actuator_to_joint({{"a",rt::Axis(0.4,0.2,0.3)}});
  EXPECT_DOUBLE_EQ(j.at("yaw").position,0.4);
  EXPECT_DOUBLE_EQ(j.at("yaw").effort,0.3);
}
TEST_F(ModelTest, SignedRatioAndOffsetAreBidirectional) {
  std::ofstream(root/"transmissions.yaml") << "transmissions:\n  ratio:\n    plugin: transmission_interface/SimpleTransmission\n    actuators: [a]\n    joints: [yaw]\n    actuator_reduction: [-2]\n    joint_offset: [0.1]\n";
  auto m=rt::Model::load((root/"driver.yaml").string());
  auto j=m.actuator_to_joint({{"a",rt::Axis(0.4,0.2,0.3)}});
  EXPECT_NEAR(j.at("yaw").position,-0.1,1e-12);
  EXPECT_NEAR(j.at("yaw").effort,-0.6,1e-12);
  auto a=m.joint_to_actuator(j);
  EXPECT_NEAR(a.at("a").position,0.4,1e-12);
}
TEST_F(ModelTest, ZeroRatioAndNonFiniteSampleAreRejected) {
  std::ofstream(root/"transmissions.yaml") << "transmissions:\n  ratio:\n    plugin: transmission_interface/SimpleTransmission\n    actuators: [a]\n    joints: [yaw]\n    actuator_reduction: [0]\n";
  EXPECT_THROW(rt::Model::load((root/"driver.yaml").string()),std::invalid_argument);
  auto m=model();
  EXPECT_THROW(m.actuator_to_joint({{"a",rt::Axis(std::numeric_limits<double>::quiet_NaN())}}),std::invalid_argument);
}

TEST_F(ModelTest, MoreDifferentialGroupsNeedOnlyConfiguration) {
  std::ofstream(root / "driver.yaml") << "motor_hardware:\n  motors:\n    a: {id: 8, joint: yaw}\n    b: {id: 9, joint: pitch}\n    c: {id: 17, joint: yaw_b}\n    d: {id: 18, joint: pitch_b}\n";
  auto m=model("  second:\n    plugin: transmission_interface/DifferentialTransmission\n    actuators: [d,c]\n    joints: [yaw_b,pitch_b]\n    actuator_reduction: [2,4]\n");
  auto j=m.actuator_to_joint({{"a",rt::Axis(.2)}, {"b",rt::Axis(.2)}, {"c",rt::Axis(-1.2)}, {"d",rt::Axis(.6)}});
  EXPECT_NEAR(j.at("yaw").position,.2,1e-12);
  EXPECT_NEAR(j.at("pitch").position,0,1e-12);
  EXPECT_NEAR(j.at("yaw_b").position,0,1e-12);
  EXPECT_NEAR(j.at("pitch_b").position,.3,1e-12);
}

TEST_F(ModelTest, ConfigurationTyposAndRepeatedNamesDoNotSilentlyChangeMechanics) {
  EXPECT_THROW(model("  second:\n    plugin: transmission_interface/SimpleTransmission\n    actuators: [c]\n    joints: [shoulder]\n    joint_offet: [0.2]\n"),std::invalid_argument);
  EXPECT_THROW(model("  wrist:\n    plugin: transmission_interface/SimpleTransmission\n    actuators: [c]\n    joints: [shoulder]\n"),std::invalid_argument);
}

TEST_F(ModelTest, ExplicitRelativeTransmissionPathUsesDriverDirectory) {
  std::ofstream(root/"custom.yaml") << "transmissions:\n  ratio:\n    plugin: transmission_interface/SimpleTransmission\n    actuators: [a]\n    joints: [yaw]\n    joint_reduction: [2]\n";
  auto m=rt::Model::load((root/"driver.yaml").string(),"custom.yaml");
  auto j=m.actuator_to_joint({{"a",rt::Axis(.4)}});
  EXPECT_NEAR(j.at("yaw").position,.2,1e-12);
}
TEST_F(ModelTest, NonlinearEffortNeedsCompleteInputSidePose) {
  auto m=rt::Model::load(write_spatial_driver(root,{1,1}).string());
  EXPECT_TRUE(m.groups().front().position_dependent);
  const std::map<std::string,double> effort{{"Joint_Link1",2},{"Joint_Link2",4}};
  const std::map<std::string,double> pose{{"Joint_Link1",.17},{"Joint_Link2",.08}};
  auto expected=m.joint_to_actuator({{"Joint_Link1",rt::Axis(.17,0,2)},{"Joint_Link2",rt::Axis(.08,0,4)}});
  auto a=m.joint_effort_to_actuator(effort,pose);
  for(const auto &[name,axis]:expected) EXPECT_NEAR(a.at(name),axis.effort,1e-12);
  auto zero=m.joint_effort_to_actuator(effort,{{"Joint_Link1",0},{"Joint_Link2",0}});
  EXPECT_GT(std::abs(a.at("arm_joint2")-zero.at("arm_joint2")),1e-3);
  std::map<std::string,double> actuator_pose;
  for(const auto &[name,axis]:expected) actuator_pose[name]=axis.position;
  auto back=m.actuator_effort_to_joint(a,actuator_pose);
  EXPECT_NEAR(back.at("Joint_Link1"),2,1e-10);
  EXPECT_NEAR(back.at("Joint_Link2"),4,1e-10);
  EXPECT_THROW(m.joint_effort_to_actuator(effort),std::invalid_argument);
  EXPECT_THROW(m.joint_effort_to_actuator({{"Joint_Link1",0},{"Joint_Link2",0}}),std::invalid_argument);
  EXPECT_THROW(m.actuator_effort_to_joint(a),std::invalid_argument);
  EXPECT_THROW(m.joint_effort_to_actuator({{"Joint_Link1",2}},pose),std::invalid_argument);
  EXPECT_THROW(m.actuator_effort_to_joint({{"arm_joint2",0}},actuator_pose),std::invalid_argument);
  EXPECT_THROW(m.joint_effort_to_actuator(effort,{{"Joint_Link1",0}}),std::invalid_argument);
  EXPECT_THROW(m.actuator_effort_to_joint(a,{{"arm_joint2",0}}),std::invalid_argument);
  for (auto invalid: {std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}) {
    EXPECT_THROW(m.joint_effort_to_actuator(effort,{{"Joint_Link1",invalid},{"Joint_Link2",0}}),std::invalid_argument);
    EXPECT_THROW(m.actuator_effort_to_joint(a,{{"arm_joint1",invalid},{"arm_joint2",0}}),std::invalid_argument);
  }
  EXPECT_THROW(m.joint_effort_to_actuator(effort,{{"unknown",0}}),std::invalid_argument);
  EXPECT_THROW(m.actuator_effort_to_joint(a,{{"unknown",0}}),std::invalid_argument);
}
TEST_F(ModelTest, LinearEffortRemainsBackwardCompatible) {
  auto m=model();
  EXPECT_FALSE(m.groups().front().position_dependent);
  auto a=m.joint_effort_to_actuator({{"yaw",2},{"pitch",4}});
  EXPECT_NEAR(a.at("a"),3,1e-12); EXPECT_NEAR(a.at("b"),-1,1e-12);
  EXPECT_EQ(a,m.joint_effort_to_actuator({{"yaw",2},{"pitch",4}},{{"yaw",.2},{"pitch",.1},{"shoulder",.8}}));
  EXPECT_EQ(a.size(),2U);
  auto j=m.actuator_effort_to_joint(a);
  EXPECT_NEAR(j.at("yaw"),2,1e-12); EXPECT_NEAR(j.at("pitch"),4,1e-12);
}
