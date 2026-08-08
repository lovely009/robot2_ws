#include "motor_canfd_drive/motor_canfd_driver.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace drive = motor_canfd_drive;

namespace {

drive::DriverConfig make_large_config() {
  // 回归用例故意配置 13 台电机（8 RP + 5 个 25/30 mm），防止未来代码再次
  // 把“当前机械臂 7 关节”误写成 MotorCanfdDriver 的固定容量。
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
         "main_bus", drive::kMotor2530Type, id,
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

} // namespace

TEST(MotorCanfdDriver, MotorCountIsConfigurationDrivenNotSeven) {
  const drive::MotorCanfdDriver driver(make_large_config());
  EXPECT_EQ(driver.motor_count(), 13U);
}

TEST(MotorCanfdDriver, RejectsDuplicateJointMapping) {
  auto config = make_large_config();
  config.motors.back().joint_name = config.motors.front().joint_name;
  EXPECT_THROW(drive::MotorCanfdDriver driver(std::move(config)),
               std::invalid_argument);
}

TEST(MotorCanfdDriver, CustomMotorTypeCanBeRegisteredWithoutChangingCoreDriver) {
  // 用一个完全虚构的 future_type 证明：扩展新电机家族只需注册协议策略，
  // 不需要给 MotorCanfdDriver 增加新的 type 分支。
  drive::DriverConfig config;
  config.buses.push_back({"aux", "vcan9"});
  config.motors.push_back({"future_motor", "future_joint", "aux", "future_type",
                           42U, {}});

  auto registry = drive::MotorProtocolRegistry::with_builtin_protocols();
  registry.register_factory("future_type",
                            [] { return std::make_unique<DummyProtocol>(); });
  EXPECT_NO_THROW(drive::MotorCanfdDriver driver(std::move(config),
                                                  std::move(registry)));
}
