#include "motor_sim/core/config.hpp"

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

namespace core = motor_sim::core;

namespace {

std::string fixture_path(const std::string &name) {
  return std::string(TEST_SOURCE_DIR) + "/test/fixtures/" + name;
}

}  // namespace

TEST(SystemConfig, AcceptsContractOwnedByCaller) {
  const core::TopologyContract contract{
      {"arm"},
      {{"joint_a", "arm", "vaxor", 1U},
       {"joint_b", "arm", "vaxor", 4U}}};
  const auto config =
      core::load_system_config(fixture_path("two_axis.yaml"), contract);
  ASSERT_EQ(config.motors.size(), 2U);
  EXPECT_EQ(config.motors[1].joint, "joint_b");
}

TEST(SystemConfig, RejectsOrderDifferentFromContract) {
  const core::TopologyContract contract{
      {"arm"}, {{"joint_b", "arm", "vaxor", 4U}}};
  EXPECT_THROW(core::load_system_config(fixture_path("two_axis.yaml"), contract),
               std::runtime_error);
}

// MIT τ_ff 的 12 位量化满量程是型号属性（写在 profile 里），必须落到逐台 MotorConfig，
// 否则协议对象只能拿到兼容默认值（R48 的 11.0）。
TEST(SystemConfig, ReadsPerModelMitTorqueLimitIntoMotorConfig) {
  const auto r58 = core::load_system_config(fixture_path("cerb_r58_axis.yaml"));
  ASSERT_EQ(r58.motors.size(), 1U);
  EXPECT_DOUBLE_EQ(r58.motors[0].mit_torque_limit_nm, 45.0);
  // 与 plant 的力矩饱和（本 fixture 照拓扑取 10.0）是两回事，不能互相顶替。
  EXPECT_DOUBLE_EQ(r58.motors[0].plant.effort_limit_nm, 10.0);
}

TEST(SystemConfig, DefaultsMitTorqueLimitForConfigsWithoutTheField) {
  const auto legacy = core::load_system_config(fixture_path("cerb_axis.yaml"));
  ASSERT_EQ(legacy.motors.size(), 1U);
  EXPECT_DOUBLE_EQ(legacy.motors[0].mit_torque_limit_nm, 11.0);
}

TEST(SystemConfig, ReadsBusTransportEndpointAndSerialFields) {
  const auto config = core::load_system_config(fixture_path("bus_transport.yaml"));
  ASSERT_EQ(config.buses.size(), 1U);
  EXPECT_EQ(config.buses[0].name, "arm");
  EXPECT_TRUE(config.buses[0].transport == core::BusTransport::SerialSlcan);
  EXPECT_EQ(config.buses[0].endpoint, "/dev/ttyACM1");
  EXPECT_EQ(config.buses[0].baud_rate, 3000000);
  EXPECT_EQ(config.buses[0].data_bitrate_index, 5);
  EXPECT_TRUE(config.buses[0].probe_adapter);
  EXPECT_TRUE(config.buses[0].allow_external_canfd);
}

// socket_can 收接口名，串口字段一律缺省；缺省值必须让既有夹具行为不变。
TEST(SystemConfig, DefaultsSocketCanBusWithoutSerialFields) {
  const auto config = core::load_system_config(fixture_path("cerb_axis.yaml"));
  ASSERT_EQ(config.buses.size(), 1U);
  EXPECT_TRUE(config.buses[0].transport == core::BusTransport::SocketCan);
  EXPECT_EQ(config.buses[0].endpoint, "robot_vcan0");
  EXPECT_EQ(config.buses[0].data_bitrate_index, -1);
  EXPECT_FALSE(config.buses[0].probe_adapter);
}

// 旧的 buses: [arm] 形状必须明确报错，不能静默当成空总线——否则节点会带着
// 零条总线起来，诊断里只说"没有总线"，与真因（配置形状不对）隔了一层。
TEST(SystemConfig, RejectsLegacyBusNameSequence) {
  EXPECT_THROW(core::load_system_config(fixture_path("legacy_bus_sequence.yaml")),
               std::runtime_error);
}

// socket_can 的 endpoint 是接口名；写设备路径必须被拒，否则下游会拿
// "/dev/..." 去 if_nametoindex，报出的却是"接口不存在"这种与真因无关的错。
TEST(SystemConfig, RejectsSocketCanEndpointWithSlash) {
  EXPECT_THROW(
      core::load_system_config(fixture_path("socket_can_path_endpoint.yaml")),
      std::runtime_error);
}

TEST(SystemConfig, RejectsOutOfRangeDataBitrateIndex) {
  EXPECT_THROW(core::load_system_config(fixture_path("bad_bitrate_index.yaml")),
               std::runtime_error);
}

// --no-brs 是取证用的对照开关：默认开 BRS（与固件冻结位时序一致），
// 关掉用于取"数据相位不切换"的对照样本。
TEST(SystemConfig, DefaultsBrsOnAndReadsOverride) {
  const auto with_brs =
      core::load_system_config(fixture_path("bus_transport.yaml"));
  EXPECT_TRUE(with_brs.buses[0].brs);
  const auto without = core::load_system_config(fixture_path("brs_off.yaml"));
  EXPECT_FALSE(without.buses[0].brs);
}

// serial_slcan 会把帧打到真实 CANable 所在的物理总线上，所以必须由配置显式声明
// 才允许——这是从 ROS 层 validate_port_mapping_common 搬过来的守卫，搬丢就等于
// 让一个笔误的 transport 值静默接上真总线。
TEST(SystemConfig, RejectsSerialSlcanWithoutExplicitOptIn) {
  EXPECT_THROW(core::load_system_config(fixture_path("slcan_without_optin.yaml")),
               std::runtime_error);
}

TEST(SystemConfig, ReadsFeedbackSchedulingFields) {
  const auto config = core::load_system_config(fixture_path("bus_transport.yaml"));
  EXPECT_EQ(config.timing.feedback_period_ms, 20U);
  EXPECT_DOUBLE_EQ(config.timing.stagger_ms, 2.2);
  EXPECT_DOUBLE_EQ(config.timing.effort_rate_hz, 200.0);
  EXPECT_EQ(config.timing.can_period_ms, 20U);
}

// 新字段的缺省必须等于旧行为，否则既有台架配置一加载就换了节拍。
TEST(SystemConfig, DefaultsSchedulingFieldsToLegacyBehaviour) {
  const auto config = core::load_system_config(fixture_path("cerb_axis.yaml"));
  EXPECT_EQ(config.timing.feedback_period_ms, 0U);
  EXPECT_DOUBLE_EQ(config.timing.stagger_ms, 0.0);
  EXPECT_DOUBLE_EQ(config.timing.effort_rate_hz, 0.0);
  EXPECT_EQ(config.timing.can_period_ms, 0U);
}

TEST(SystemConfig, RejectsNegativeStagger) {
  EXPECT_THROW(core::load_system_config(fixture_path("bad_stagger.yaml")),
               std::runtime_error);
}
