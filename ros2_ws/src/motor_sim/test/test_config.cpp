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
// 否则协议对象只能拿到兼容默认值（kCerbDefaultMitTorqueLimitNm = 300.0）。
TEST(SystemConfig, ReadsPerModelMitTorqueLimitIntoMotorConfig) {
  const auto r58 = core::load_system_config(fixture_path("cerb_r58_axis.yaml"));
  ASSERT_EQ(r58.motors.size(), 1U);
  EXPECT_DOUBLE_EQ(r58.motors[0].mit_torque_limit_nm, 300.0);
  // 与 plant 的力矩饱和（本 fixture 照拓扑取 10.0）是两回事，不能互相顶替。
  EXPECT_DOUBLE_EQ(r58.motors[0].plant.effort_limit_nm, 10.0);
}

TEST(SystemConfig, ReadsAllFiveCustomMitScales) {
  const auto config = core::load_system_config(fixture_path("cerb_custom_scales.yaml"));
  ASSERT_EQ(config.motors.size(), 1U);
  const auto &motor = config.motors.front();
  EXPECT_EQ(motor.id, 16U);
  EXPECT_DOUBLE_EQ(motor.mit_position_limit_rad, 6.28);
  EXPECT_DOUBLE_EQ(motor.mit_velocity_limit_rad_s, 8.0);
  EXPECT_DOUBLE_EQ(motor.mit_torque_limit_nm, 150.0);
  EXPECT_DOUBLE_EQ(motor.mit_kp_limit, 1200.0);
  EXPECT_DOUBLE_EQ(motor.mit_kd_limit, 200.0);
  EXPECT_DOUBLE_EQ(motor.plant.effort_limit_nm, 10.0);
}

TEST(SystemConfig, DefaultsMitTorqueLimitForConfigsWithoutTheField) {
  const auto legacy = core::load_system_config(fixture_path("cerb_axis.yaml"));
  ASSERT_EQ(legacy.motors.size(), 1U);
  EXPECT_DOUBLE_EQ(legacy.motors[0].mit_torque_limit_nm, 300.0);
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

TEST(SystemConfig, ReadsSdoOverrides) {
  const auto config = core::load_system_config(fixture_path("bus_transport.yaml"));
  ASSERT_EQ(config.motors.size(), 1U);
  EXPECT_EQ(config.motors[0].sdo_overrides.protection_enabled, 0U);
  EXPECT_EQ(config.motors[0].sdo_overrides.direction, 1U);
}

// 缺省 = 真机上电初值（保护开关 1，方向 0=CW）。这两个对象不落盘，"进程重启 =
// 掉电复现"，初值给错，台架就永远复现不出"上电后保护是关的"这个前提。
TEST(SystemConfig, DefaultsSdoOverridesToRealHardwarePowerOnValues) {
  const auto config = core::load_system_config(fixture_path("cerb_axis.yaml"));
  ASSERT_EQ(config.motors.size(), 1U);
  EXPECT_EQ(config.motors[0].sdo_overrides.protection_enabled, 1U);
  EXPECT_EQ(config.motors[0].sdo_overrides.direction, 0U);
}

TEST(SystemConfig, RejectsSdoOverrideOutsideBooleanDomain) {
  EXPECT_THROW(core::load_system_config(fixture_path("bad_sdo_overrides.yaml")),
               std::runtime_error);
}

// Review Focus #1：buses 某项缺 endpoint 必须在加载期抛错并指名第几条总线，
// 而不是留空串到 open()/if_nametoindex 才失败。
TEST(SystemConfig, RejectsBusWithoutEndpoint) {
  EXPECT_THROW(core::load_system_config(fixture_path("bus_without_endpoint.yaml")),
               std::runtime_error);
}

// 串口位率白名单：非白名单值必须在加载期拒绝（原 ROS 层用例随搬家一度失去覆盖）。
TEST(SystemConfig, RejectsUnsupportedBaudRate) {
  EXPECT_THROW(core::load_system_config(fixture_path("bad_baud_rate.yaml")),
               std::runtime_error);
}

// 设备身份串与 SDO 出厂值随 profile 落到逐台 MotorConfig：固件靠 0x1008 绑型号、
// 靠 0x6080/0x2538 回读核验，两者必须同源。
TEST(SystemConfig, ReadsCerbDeviceIdentityAndSdoDefaults) {
  const auto r58 = core::load_system_config(fixture_path("cerb_r58_axis.yaml"));
  ASSERT_EQ(r58.motors.size(), 1U);
  EXPECT_EQ(r58.motors[0].device_identity, "R58LITE_V2");
  EXPECT_EQ(r58.motors[0].device_max_speed_rpm, 3325);
  EXPECT_EQ(r58.motors[0].device_max_current_ma, 13300);
}

// 0x6080 是模块侧共用值，两台相同；逐型号的是 0x2538。
TEST(SystemConfig, CerbMaxSpeedIsSharedAcrossModelsButCurrentIsNot) {
  const auto r48 = core::load_system_config(fixture_path("cerb_axis.yaml"));
  ASSERT_EQ(r48.motors.size(), 1U);
  EXPECT_EQ(r48.motors[0].device_max_speed_rpm, 3325);
  EXPECT_EQ(r48.motors[0].device_max_current_ma, 5000);
}
