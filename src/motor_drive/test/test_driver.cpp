#include "motor_drive/motor_driver.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace drive = motor_drive;

namespace {

drive::DriverConfig make_large_config() {
  // 回归用例故意配置 13 台电机（8 RP + 5 个 25/30 mm），防止未来代码再次
  // 把“当前机械臂 7 关节”误写成 MotorDriver 的固定容量。
  drive::DriverConfig config;
  config.buses.push_back({"main_bus", "vcan0"});
  for (unsigned id = 1U; id <= 8U; ++id) {
    config.motors.push_back({"rp_" + std::to_string(id),
                             "joint_rp_" + std::to_string(id), "main_bus",
                             drive::kRpMotorType, id, {}});
  }
  for (unsigned id = 1U; id <= 5U; ++id) {
    config.motors.push_back(
        {"small_" + std::to_string(id), "joint_small_" + std::to_string(id),
         "main_bus", drive::kVaxorType, id,
         {{"torque_constant_nm_per_amp", "0.35"}}});
  }
  return config;
}

class DummyProtocol final : public drive::MotorProtocol {
public:
  void validate(const std::vector<drive::MotorConfig> &,
                const drive::MotorIndexList &) const override {}
  std::vector<drive::CanFrame>
  make_enable_frames(const std::vector<drive::MotorConfig> &,
                     const drive::MotorIndexList &, bool) const override {
    return {};
  }
  std::vector<drive::CanFrame>
  make_command_frames(const std::vector<drive::MotorConfig> &,
                      const drive::MotorIndexList &,
                      const std::vector<drive::protocol::JointCommand> &) const override {
    return {};
  }
  bool decode_feedback(const std::vector<drive::MotorConfig> &,
                       const drive::MotorIndexList &, const drive::CanFrame &,
                       std::size_t &, drive::protocol::JointState &) const override {
    return false;
  }
};

class DummyTransport final : public drive::CanTransport {
public:
  bool send(uint32_t, const uint8_t *, std::size_t, bool, bool,
            bool) const override {
    return true;
  }
  bool receive(drive::CanFrame &) const override { return false; }
};

} // namespace

TEST(MotorDriver, MotorCountIsConfigurationDrivenNotSeven) {
  const drive::MotorDriver driver(make_large_config());
  EXPECT_EQ(driver.motor_count(), 13U);
}

TEST(MotorDriver, RejectsDuplicateJointMapping) {
  auto config = make_large_config();
  config.motors.back().joint_name = config.motors.front().joint_name;
  EXPECT_THROW(drive::MotorDriver driver(std::move(config)),
               std::invalid_argument);
}

TEST(MotorDriver, CustomMotorTypeCanBeRegisteredWithoutChangingCoreDriver) {
  // 用一个完全虚构的 future_type 证明：扩展新电机家族只需注册协议策略，
  // 不需要给 MotorDriver 增加新的 type 分支。
  drive::DriverConfig config;
  config.buses.push_back({"aux", "vcan9"});
  config.motors.push_back({"future_motor", "future_joint", "aux", "future_type",
                           42U, {}});

  auto registry = drive::MotorProtocolRegistry::with_builtin_protocols();
  registry.register_factory("future_type",
                            [] { return std::make_unique<DummyProtocol>(); });
  EXPECT_NO_THROW(drive::MotorDriver driver(std::move(config),
                                            std::move(registry)));
}

TEST(CanTransportRegistry, BuiltinTransportsAreRegistered) {
  const auto registry = drive::CanTransportRegistry::with_builtin_transports();
  EXPECT_TRUE(registry.contains(drive::kSocketCanTransport));
  EXPECT_TRUE(registry.contains(drive::kSerialSlcanTransport));
  EXPECT_FALSE(registry.contains("unknown_transport"));
}

TEST(CanTransportRegistry, UnknownTransportThrowsOnCreate) {
  const auto registry = drive::CanTransportRegistry::with_builtin_transports();
  drive::BusConfig bus;
  bus.name = "bad";
  bus.transport = "not_a_transport";
  EXPECT_THROW(static_cast<void>(registry.create(bus)), std::invalid_argument);
}

TEST(MotorDriver, OpensTransportFromBusConfig) {
  // 传输类型由 BusConfig.transport 决定：注册 "fake" 后 open() 应调用对应工厂，
  // 而不是在驱动里写死 SocketCan。
  drive::DriverConfig config;
  config.buses.push_back({"bench", "", "fake", "/dev/null"});
  config.motors.push_back({"rp_1", "joint_rp_1", "bench",
                           drive::kRpMotorType, 1U, {}});

  auto transports = drive::CanTransportRegistry::with_builtin_transports();
  bool factory_called = false;
  transports.register_factory(
      "fake", [&factory_called](const drive::BusConfig &) {
        factory_called = true;
        return std::make_unique<DummyTransport>();
      });

  drive::MotorDriver driver(std::move(config),
                            drive::MotorProtocolRegistry::with_builtin_protocols(),
                            std::move(transports));
  driver.open();
  EXPECT_TRUE(driver.is_open());
  EXPECT_TRUE(factory_called);
  driver.close();
  EXPECT_FALSE(driver.is_open());
}

TEST(MotorDriver, RejectsUnregisteredTransport) {
  drive::DriverConfig config;
  config.buses.push_back({"bench", "", "not_a_transport", ""});
  config.motors.push_back({"rp_1", "joint_rp_1", "bench",
                           drive::kRpMotorType, 1U, {}});
  EXPECT_THROW(drive::MotorDriver driver(std::move(config)),
               std::invalid_argument);
}

TEST(MotorDriver, SerialSlcanBusRequiresTty) {
  drive::DriverConfig config;
  config.buses.push_back({"bench", "", drive::kSerialSlcanTransport, ""});
  config.motors.push_back({"rp_1", "joint_rp_1", "bench",
                           drive::kRpMotorType, 1U, {}});
  EXPECT_THROW(drive::MotorDriver driver(std::move(config)),
               std::invalid_argument);
}
