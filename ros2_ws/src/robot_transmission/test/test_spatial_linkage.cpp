#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <filesystem>
#include <unistd.h>
#include "robot_transmission/model.hpp"
#include "spatial_fixture.hpp"
#include "robot_transmission/spatial_linkage.hpp"
#include <yaml-cpp/yaml.h>
#include <limits>
#include <utility>

namespace rt = robot_transmission;
namespace {
const double pi = std::acos(-1.0);
const double rad = pi / 180.0;
// 独立按 CAD 的两条旋转轴计算球心，捕捉只做往返却未闭环的实现。
double rod_length(double t2, double q1, double q2) {
  const double ay = .04475 + std::cos(t2)*(.061556509578-.04475) - std::sin(t2)*(.389158356406-.4);
  const double az = .4 + std::sin(t2)*(.061556509578-.04475) + std::cos(t2)*(.389158356406-.4);
  const double x = .005 + std::cos(q2)*(.035840440737-.0025) - std::sin(q2)*(.494319840327-.5025);
  const double y = .0005;
  const double z = .5025 + std::sin(q2)*(.035840440737-.0025) + std::cos(q2)*(.494319840327-.5025);
  const double bx = -.0025 + std::cos(q1)*x - std::sin(q1)*y;
  const double by = .06725 + std::sin(q1)*x + std::cos(q1)*y;
  return std::sqrt(std::pow(bx-.033699999952,2) + std::pow(by-ay,2) + std::pow(z-az,2));
}
}
class SpatialLinkageTest : public ::testing::Test {
protected:
  std::filesystem::path root;
  void SetUp() override {
    char directory[] = "/tmp/spatial_transmission_XXXXXX";
    root = mkdtemp(directory);
  }
  void TearDown() override { std::filesystem::remove_all(root); }
  rt::Model model(std::array<double,2> signs = {1,1}) {
    return rt::Model::load(write_spatial_driver(root,signs).string());
  }
  hardware_interface::TransmissionInfo info() {
    write_spatial_driver(root,{1,1});
    hardware_interface::TransmissionInfo result;
    const auto parameters=YAML::LoadFile((root/"transmissions.yaml").string())["transmissions"]["shoulder_linkage"]["parameters"].as<std::map<std::string,std::string>>();
    result.parameters.insert(parameters.begin(),parameters.end());
    for (int i=0;i<2;++i) {
      hardware_interface::JointInfo j; j.name="j"+std::to_string(i); j.mechanical_reduction=1; j.offset=0;
      hardware_interface::ActuatorInfo a; a.name="a"+std::to_string(i); a.mechanical_reduction=1;
      result.joints.push_back(j); result.actuators.push_back(a);
    }
    return result;
  }
};
TEST_F(SpatialLinkageTest, CadSamplesRoundTrip) {
  auto m=model();
  for (const auto &v : std::vector<std::array<double,3>>{
      {0,0,0}, {0,10,5.362931449174114},
      {10,0,-1.0109202367883747}, {10,10,4.530462691371301}}) {
    auto j=m.actuator_to_joint({{"arm_joint1",rt::Axis(v[0]*rad)}, {"arm_joint2",rt::Axis(v[1]*rad)}});
    EXPECT_NEAR(j.at("Joint_Link1").position,v[0]*rad,1e-9);
    EXPECT_NEAR(j.at("Joint_Link2").position,v[2]*rad,1e-9);
    EXPECT_NEAR(rod_length(v[1]*rad,j.at("Joint_Link1").position,j.at("Joint_Link2").position),.10536545216923923,1e-12);
    auto a=m.joint_to_actuator(j);
    EXPECT_NEAR(a.at("arm_joint1").position,v[0]*rad,1e-9);
    EXPECT_NEAR(a.at("arm_joint2").position,v[1]*rad,1e-9);
  }
}
TEST_F(SpatialLinkageTest, JacobianMatchesFiniteDifference) {
  auto m=model();
  for (double t : {0.0,10*rad}) {
    for (int col=0;col<2;++col) {
      const double h=1e-6;
      auto state=[&](double offset, bool velocity) {
        return m.actuator_to_joint({
          {"arm_joint1",rt::Axis(t+(col==0?offset:0),velocity&&col==0?1:0)},
          {"arm_joint2",rt::Axis(t+(col==1?offset:0),velocity&&col==1?1:0)}});
      };
      auto mid=state(0,true), plus=state(h,false), minus=state(-h,false);
      for (const auto &name : {"Joint_Link1","Joint_Link2"})
        EXPECT_NEAR(mid.at(name).velocity,(plus.at(name).position-minus.at(name).position)/(2*h),1e-7);
      if(t==0) EXPECT_NEAR(mid.at("Joint_Link2").velocity,col==0?-.06708715259239355:.5206392907848132,1e-7);
    }
  }
}
TEST_F(SpatialLinkageTest, VelocityEffortRoundTripPreservesPower) {
  auto m=model();
  for (double t : {0.0,10*rad}) {
    rt::AxisMap input{{"arm_joint1",rt::Axis(t,.3,2)}, {"arm_joint2",rt::Axis(t,-.2,4)}};
    auto j=m.actuator_to_joint(input);
    EXPECT_NEAR(j.at("Joint_Link1").velocity*j.at("Joint_Link1").effort+
      j.at("Joint_Link2").velocity*j.at("Joint_Link2").effort, -.2,1e-10);
    auto a=m.joint_to_actuator(j);
    for(const auto &[name,axis]:input) {
      EXPECT_NEAR(a.at(name).position,axis.position,1e-9);
      EXPECT_NEAR(a.at(name).velocity,axis.velocity,1e-10);
      EXPECT_NEAR(a.at(name).effort,axis.effort,1e-10);
    }
  }
}
TEST_F(SpatialLinkageTest, DirectionSignsRoundTrip) {
  for(double s1 : {-1.,1.}) for(double s2 : {-1.,1.}) {
    auto m=model({s1,s2});
    auto j=m.actuator_to_joint({{"arm_joint1",rt::Axis(s1*10*rad,.3,2)}, {"arm_joint2",rt::Axis(s2*10*rad,-.2,4)}});
    EXPECT_NEAR(j.at("Joint_Link1").position,10*rad,1e-9);
    EXPECT_NEAR(j.at("Joint_Link2").position,4.530462691371301*rad,1e-9);
    auto a=m.joint_to_actuator(j);
    EXPECT_NEAR(a.at("arm_joint1").position,s1*10*rad,1e-9);
    EXPECT_NEAR(a.at("arm_joint2").position,s2*10*rad,1e-9);
    EXPECT_NEAR(a.at("arm_joint1").effort,2,1e-10);
    EXPECT_NEAR(a.at("arm_joint2").velocity,-.2,1e-10);
  }
}
TEST_F(SpatialLinkageTest, RejectsNumericallyUnresolvableNonzeroTangencies) {
  auto m=model();
  for(const auto &v:std::vector<std::array<double,2>>{
      {.17,1.9132826696561025},{.17,1.9132826706561026},{0,1.9590990014033134}}) {
    EXPECT_THROW(m.actuator_to_joint({{"arm_joint1",rt::Axis(v[0],.3,2)},
      {"arm_joint2",rt::Axis(v[1],-.2,4)}}),std::invalid_argument);
  }
  auto j=m.actuator_to_joint({{"arm_joint1",rt::Axis(.17,.3,2)},{"arm_joint2",rt::Axis(1.90,-.2,4)}});
  auto a=m.joint_to_actuator(j);
  EXPECT_NEAR(a.at("arm_joint2").position,1.90,1e-10);
  EXPECT_NEAR(a.at("arm_joint2").velocity,-.2,1e-8);
  EXPECT_NEAR(a.at("arm_joint2").effort,4,1e-8);
}
TEST_F(SpatialLinkageTest, RejectsUnreachablePitch) {
  auto m=model();
  EXPECT_THROW(m.joint_to_actuator({{"Joint_Link1",rt::Axis()}, {"Joint_Link2",rt::Axis(pi/2)}}),std::invalid_argument);
}
TEST_F(SpatialLinkageTest, RejectsInputHalfCircleBoundary) {
  auto m=model();
  const double phi=std::atan2(.061556509578-.04475,-(.389158356406-.4));
  for(double angle:{-phi,pi-phi,-phi-1e-6,pi-phi+1e-6,-2*pi,2*pi}) {
    EXPECT_THROW(m.actuator_to_joint({{"arm_joint1",rt::Axis()}, {"arm_joint2",rt::Axis(angle)}}),std::invalid_argument);
  }
}
TEST_F(SpatialLinkageTest, BranchDoesNotDependOnCallOrder) {
  auto m=model();
  for(int i=0;i<10;++i) {
    auto moved=std::move(m);
    auto j=moved.actuator_to_joint({{"arm_joint1",rt::Axis(10*rad)}, {"arm_joint2",rt::Axis(10*rad)}});
    EXPECT_NEAR(j.at("Joint_Link2").position,4.530462691371301*rad,1e-9);
    EXPECT_THROW(moved.joint_to_actuator({{"Joint_Link1",rt::Axis()}, {"Joint_Link2",rt::Axis(pi/2)}}),std::invalid_argument);
    auto zero=moved.actuator_to_joint({{"arm_joint1",rt::Axis()}, {"arm_joint2",rt::Axis()}});
    EXPECT_NEAR(zero.at("Joint_Link2").position,0,1e-9);
    auto back=moved.joint_to_actuator(j);
    EXPECT_NEAR(back.at("arm_joint2").position,10*rad,1e-9);
    m=model();
  }
}
TEST_F(SpatialLinkageTest, RejectsBadGeometry) {
  auto base=info();
  for (const auto &[key,value]:std::vector<std::pair<std::string,std::string>>{
      {"motor_axis_direction","0 0 0"}, {"joint2_axis_direction","0 -2 0"},
      {"motor_ball_zero","0.033699999952 0.04475 0.400"},
      {"joint_ball_zero","0.0025 0.06775 0.5025"}, {"rod_length","1"},
      {"rod_length","nan"}, {"rod_length","inf"}, {"rod_length","0"},
      {"rod_length","1e308"}, {"joint1_axis_origin","0 0 0 garbage"},
      {"joint1_axis_direction","0 1"}}) {
    auto bad=base; bad.parameters[key]=value;
    EXPECT_THROW(rt::SpatialLinkageTransmission{bad},std::invalid_argument) << key << ": " << value;
  }
  auto bad=base; bad.parameters.erase("rod_length");
  EXPECT_THROW(rt::SpatialLinkageTransmission{bad},std::invalid_argument);
  bad=base; bad.parameters["unknown"]="0";
  EXPECT_THROW(rt::SpatialLinkageTransmission{bad},std::invalid_argument);
  bad=base; bad.actuators.pop_back();
  EXPECT_THROW(rt::SpatialLinkageTransmission{bad},std::invalid_argument);
  for(double ratio:{0.,2.,std::numeric_limits<double>::quiet_NaN()}) {
    bad=base; bad.actuators[0].mechanical_reduction=ratio;
    EXPECT_THROW(rt::SpatialLinkageTransmission{bad},std::invalid_argument);
    bad=base; bad.joints[0].mechanical_reduction=ratio;
    EXPECT_THROW(rt::SpatialLinkageTransmission{bad},std::invalid_argument);
  }
  bad=base; bad.joints[1].offset=.1;
  EXPECT_THROW(rt::SpatialLinkageTransmission{bad},std::invalid_argument);
}
TEST_F(SpatialLinkageTest, RejectsBadHandlesAndDoesNotPartiallyWrite) {
  rt::SpatialLinkageTransmission plugin(info());
  std::array<double,6> av{},jv{7,7,7,7,7,7};
  std::vector<transmission_interface::JointHandle> j;
  std::vector<transmission_interface::ActuatorHandle> a;
  const std::array<std::string,3> interfaces{"position","velocity","effort"};
  for(int i=0;i<2;++i) for(int k=0;k<3;++k) {
    j.emplace_back("j"+std::to_string(i),interfaces[k],&jv[3*i+k]);
    a.emplace_back("a"+std::to_string(i),interfaces[k],&av[3*i+k]);
  }
  EXPECT_THROW(plugin.actuator_to_joint(),std::invalid_argument);
  auto missing=j; missing.pop_back();
  EXPECT_THROW(plugin.configure(missing,a),std::invalid_argument);
  auto duplicate=j; duplicate.back()=duplicate.front();
  EXPECT_THROW(plugin.configure(duplicate,a),std::invalid_argument);
  auto null=a; null[0]=transmission_interface::ActuatorHandle("a0","position");
  EXPECT_THROW(plugin.configure(j,null),std::invalid_argument);
  std::reverse(a.begin(),a.end()); std::reverse(j.begin(),j.end());
  plugin.configure(j,a);
  for(double invalid:{std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity(),10.}) {
    av[3]=invalid;
    EXPECT_THROW(plugin.actuator_to_joint(),std::invalid_argument);
    EXPECT_EQ(jv,(std::array<double,6>{7,7,7,7,7,7}));
  }
}
TEST_F(SpatialLinkageTest, RejectsTangentAndSingularZeroGeometry) {
  // 两个圆在零位外切：杆方向垂直于转动速度，F_q2=F_theta2=0。
  auto bad=info();
  bad.parameters["motor_axis_origin"]="0 0 0";
  bad.parameters["motor_axis_direction"]="1 0 0";
  bad.parameters["motor_ball_zero"]="0 1 0";
  bad.parameters["joint1_axis_origin"]="0 0 0";
  bad.parameters["joint2_axis_origin"]="0 4 0";
  bad.parameters["joint2_axis_direction"]="1 0 0";
  bad.parameters["joint_ball_zero"]="0 3 0";
  bad.parameters["rod_length"]="2";
  EXPECT_THROW(rt::SpatialLinkageTransmission{bad},std::invalid_argument);
}
TEST_F(SpatialLinkageTest, NonAlphabeticAxisOrderingSurvivesRealPluginReload) {
  auto path=write_spatial_driver(root,{1,1});
  auto document=YAML::LoadFile((root/"transmissions.yaml").string());
  document["transmissions"]["shoulder_linkage"]["actuators"]=std::vector<std::string>{"arm_joint2","arm_joint1"};
  document["transmissions"]["shoulder_linkage"]["joints"]=std::vector<std::string>{"Joint_Link2","Joint_Link1"};
  std::ofstream(root/"transmissions.yaml") << document;
  for(int i=0;i<5;++i) {
    auto m=rt::Model::load(path.string());
    auto j=m.actuator_to_joint({{"arm_joint2",rt::Axis(0)}, {"arm_joint1",rt::Axis(10*rad)}});
    EXPECT_NEAR(j.at("Joint_Link2").position,0,1e-9);
    EXPECT_NEAR(j.at("Joint_Link1").position,5.362931449174114*rad,1e-9);
  }
}
