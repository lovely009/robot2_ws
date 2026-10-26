#include "motor_sim/core/sdo_server.hpp"

#include <gtest/gtest.h>

#include <string>

namespace core = motor_sim::core;

namespace {

core::MotorConfig cerb_motor(const std::string &identity, const int rpm,
                             const int ma, const unsigned protection,
                             const unsigned direction) {
  core::MotorConfig motor;
  motor.joint = "joint_cerb";
  motor.bus = "arm";
  motor.protocol = "cerb";
  motor.id = 1U;
  motor.device_identity = identity;
  motor.device_max_speed_rpm = rpm;
  motor.device_max_current_ma = ma;
  motor.sdo_overrides.protection_enabled = protection;
  motor.sdo_overrides.direction = direction;
  return motor;
}

// 快速读写的值在应答字节 4..7（小端有符号 32 位）。
int32_t sdo_value(const core::SdoFrame &frame) {
  return static_cast<int32_t>(static_cast<uint32_t>(frame[4]) |
                              (static_cast<uint32_t>(frame[5]) << 8) |
                              (static_cast<uint32_t>(frame[6]) << 16) |
                              (static_cast<uint32_t>(frame[7]) << 24));
}

core::CerbSdoServer server_with(const core::MotorConfig &motor) {
  core::CerbSdoServer server;
  server.add_node(motor.id, motor.device_identity,
                  core::cerb_sdo_initial_objects(motor));
  return server;
}

}  // namespace

// 写后读回：固件的参数写回核验就靠这个（写 0x23 → 0x60 ack，读 0x40 → 0x43 + 值）。
TEST(CerbSdoServer, WritesThenReadsBackAnObject) {
  auto server = server_with(cerb_motor("R48LITE_V2", 3325, 5000, 1U, 0U));
  core::SdoFrame response{};

  EXPECT_TRUE(server.handle(1U, core::SdoFrame{0x23, 0x7D, 0x60, 0x01, 0x42,
                                               0xD7, 0xFF, 0xFF},
                            response));
  EXPECT_EQ(response, (core::SdoFrame{0x60, 0x7D, 0x60, 0x01, 0, 0, 0, 0}));

  EXPECT_TRUE(server.handle(1U, core::SdoFrame{0x40, 0x7D, 0x60, 0x01, 0, 0, 0, 0},
                            response));
  EXPECT_EQ(response, (core::SdoFrame{0x43, 0x7D, 0x60, 0x01, 0x42, 0xD7, 0xFF,
                                      0xFF}));
}

// 中止码须区分索引不存在与子索引不存在，与真机应答一致。
TEST(CerbSdoServer, AbortsAreLayeredByIndexAndSubindex) {
  auto server = server_with(cerb_motor("R48LITE_V2", 3325, 5000, 1U, 0U));
  core::SdoFrame response{};

  EXPECT_TRUE(server.handle(1U, core::SdoFrame{0x40, 0x99, 0x99, 0x00, 0, 0, 0, 0},
                            response));
  EXPECT_EQ(response, (core::SdoFrame{0x80, 0x99, 0x99, 0x00, 0x00, 0x00, 0x02,
                                      0x06}));

  EXPECT_TRUE(server.handle(1U, core::SdoFrame{0x40, 0x80, 0x60, 0x01, 0, 0, 0, 0},
                            response));
  EXPECT_EQ(response, (core::SdoFrame{0x80, 0x80, 0x60, 0x01, 0x11, 0x00, 0x09,
                                      0x06}));
}

// 不是本从站的节点：**完全不回**。回一条 abort 会让固件以为该节点存在。
TEST(CerbSdoServer, StaysSilentForUnknownNodes) {
  auto server = server_with(cerb_motor("R48LITE_V2", 3325, 5000, 1U, 0U));
  core::SdoFrame response{};
  EXPECT_FALSE(
      server.handle(9U, core::SdoFrame{0x40, 0x08, 0x10, 0x00, 0, 0, 0, 0},
                    response));
}

// 设备端限值：0x6080 是**模块侧共用值**（两台同为 3325），0x2538 逐型号。
// 做成逐型号会让 R48 的速度天花板被整体缩放。
TEST(CerbSdoServer, ServesDeviceLimitsWithSharedMaxSpeed) {
  auto r58 = server_with(cerb_motor("R58LITE_V2", 3325, 13300, 1U, 0U));
  core::SdoFrame response{};
  ASSERT_TRUE(r58.handle(1U, core::SdoFrame{0x40, 0x80, 0x60, 0x00, 0, 0, 0, 0},
                         response));
  EXPECT_EQ(sdo_value(response), 3325);
  ASSERT_TRUE(r58.handle(1U, core::SdoFrame{0x40, 0x38, 0x25, 0x00, 0, 0, 0, 0},
                         response));
  EXPECT_EQ(sdo_value(response), 13300);

  auto r48 = server_with(cerb_motor("R48LITE_V2", 3325, 5000, 1U, 0U));
  ASSERT_TRUE(r48.handle(1U, core::SdoFrame{0x40, 0x38, 0x25, 0x00, 0, 0, 0, 0},
                         response));
  EXPECT_EQ(sdo_value(response), 5000);
}

// 位置限位出厂值是 ±32768 cnt（±180°）；固件的写后读回会拿它对账，
// 缺了或被写窄，台架表现为"限位与真机不符"而没有任何报错。
TEST(CerbSdoServer, FactoryPositionLimitsArePlusMinus32768) {
  auto server = server_with(cerb_motor("R48LITE_V2", 3325, 5000, 1U, 0U));
  core::SdoFrame response{};
  ASSERT_TRUE(server.handle(1U, core::SdoFrame{0x40, 0x7D, 0x60, 0x01, 0, 0, 0, 0},
                            response));
  EXPECT_EQ(sdo_value(response), -32768);
  ASSERT_TRUE(server.handle(1U, core::SdoFrame{0x40, 0x7D, 0x60, 0x02, 0, 0, 0, 0},
                            response));
  EXPECT_EQ(sdo_value(response), 32768);
}

// 易失对象的上电初值来自 sdo_overrides：真机这两个对象不落盘，"进程重启 =
// 掉电复现"。初值给错，台架就永远复现不出「保护被关掉」这个前提。
TEST(CerbSdoServer, PowerOnObjectsComeFromSdoOverrides) {
  auto defaults = server_with(cerb_motor("R48LITE_V2", 3325, 5000, 1U, 0U));
  core::SdoFrame response{};
  ASSERT_TRUE(defaults.handle(1U, core::SdoFrame{0x40, 0x50, 0x26, 0x00, 0, 0, 0, 0},
                              response));
  EXPECT_EQ(response[4], 1);  // 0x2650:00 保护默认开在 1
  ASSERT_TRUE(defaults.handle(1U, core::SdoFrame{0x40, 0x41, 0x25, 0x00, 0, 0, 0, 0},
                              response));
  EXPECT_EQ(response[4], 0);  // 0x2541:00 方向 CW

  auto overridden = server_with(cerb_motor("R48LITE_V2", 3325, 5000, 0U, 1U));
  ASSERT_TRUE(overridden.handle(1U,
                                core::SdoFrame{0x40, 0x50, 0x26, 0x00, 0, 0, 0, 0},
                                response));
  EXPECT_EQ(response[4], 0);  // 保护被关掉
  ASSERT_TRUE(overridden.handle(
      1U, core::SdoFrame{0x40, 0x41, 0x25, 0x00, 0, 0, 0, 0}, response));
  EXPECT_EQ(response[4], 1);  // 方向 CCW
}

// 身份读取应答（SDO 0x1008 分段上传）：字节格式必须与固件 cerb_adapter 对向，
// 否则台架上固件永远读不到身份串、全部电机停在未绑定。
TEST(CerbSdoServer, SegmentedUploadServesTheDeviceIdentity) {
  auto server = server_with(cerb_motor("R48LITE_V2", 3325, 5000, 1U, 0U));
  core::SdoFrame response{};
  const core::SdoFrame initiate{0x40, 0x08, 0x10, 0x00, 0, 0, 0, 0};

  ASSERT_TRUE(server.handle(1U, initiate, response));
  // 字节 4-5 = 总长（10），命令字 0x41 = 分段上传且长度已知。
  EXPECT_EQ(response, (core::SdoFrame{0x41, 0x08, 0x10, 0x00, 10, 0, 0, 0}));

  ASSERT_TRUE(server.handle(1U, core::SdoFrame{0x60, 0, 0, 0, 0, 0, 0, 0},
                            response));
  // 非末段：head = toggle(0)<<4 | unused(0)<<1 | last(0) = 0x00。
  EXPECT_EQ(response[0], 0x00);
  EXPECT_EQ(std::string(response.begin() + 1, response.begin() + 8), "R48LITE");

  ASSERT_TRUE(server.handle(1U, core::SdoFrame{0x70, 0, 0, 0, 0, 0, 0, 0},
                            response));
  // 末段：head = toggle(1)<<4 | unused(4)<<1 | last(1) = 0x19，数据 "_V2" + 4 字节填充。
  EXPECT_EQ(response[0], 0x19);
  EXPECT_EQ(std::string(response.begin() + 1, response.begin() + 4), "_V2");
}

// 身份串只挂在 :00；真机 :01/:02 回"子索引不存在"，这里不能回身份串。
TEST(CerbSdoServer, IdentitySubindicesOtherThanZeroAbort) {
  auto server = server_with(cerb_motor("R48LITE_V2", 3325, 5000, 1U, 0U));
  core::SdoFrame response{};
  ASSERT_TRUE(server.handle(1U, core::SdoFrame{0x40, 0x08, 0x10, 0x01, 0, 0, 0, 0},
                            response));
  EXPECT_EQ(response, (core::SdoFrame{0x80, 0x08, 0x10, 0x01, 0x11, 0x00, 0x09,
                                      0x06}));
}

// 取段游标按节点保存：两次上传之间会被 initiate 重置（否则第二次读到空串）。
TEST(CerbSdoServer, IdentityCursorResetsOnEachInitiate) {
  auto server = server_with(cerb_motor("R58LITE_V2", 3325, 13300, 1U, 0U));
  core::SdoFrame response{};
  const core::SdoFrame initiate{0x40, 0x08, 0x10, 0x00, 0, 0, 0, 0};
  ASSERT_TRUE(server.handle(1U, initiate, response));
  ASSERT_TRUE(server.handle(1U, core::SdoFrame{0x60, 0, 0, 0, 0, 0, 0, 0},
                            response));
  ASSERT_TRUE(server.handle(1U, initiate, response));
  ASSERT_TRUE(server.handle(1U, core::SdoFrame{0x60, 0, 0, 0, 0, 0, 0, 0},
                            response));
  EXPECT_EQ(std::string(response.begin() + 1, response.begin() + 8), "R58LITE");
}
