// 主机侧 codec：线上定点消息 ↔ 主机 facade 消息。
//
// 与固件 lib/wire_codec 是**同一条算术的两侧实现**：golden 向量在
// micro_ros_firmware/test/host/test_wire_codec.cpp 与 test/test_codec.cpp 里同值互钉。
// 本文件不含 rclcpp 依赖，将来 ros2_control 的 SystemInterface 直接复用。
#ifndef MICRO_ROS_BRIDGE__CODEC_HPP_
#define MICRO_ROS_BRIDGE__CODEC_HPP_

#include <cmath>
#include <cstdint>
#include <string>

#include <can_bridge_msgs/msg/wire_contract.hpp>

namespace micro_ros_bridge
{
namespace contract
{
// 常量别名：值全部来自 WireContract.msg，本文件不写第二份数值。
inline constexpr uint16_t kSchemaVersion =
  can_bridge_msgs::msg::WireContract::SCHEMA_VERSION;
inline constexpr uint8_t kMotorCountMax =
  can_bridge_msgs::msg::WireContract::MOTOR_COUNT_MAX;
inline constexpr int16_t kSentinelMissing =
  can_bridge_msgs::msg::WireContract::SENTINEL_MISSING;
inline constexpr double kScalePosition =
  can_bridge_msgs::msg::WireContract::SCALE_POSITION_RAD;
inline constexpr double kScaleVelocity =
  can_bridge_msgs::msg::WireContract::SCALE_VELOCITY_RAD_S;
inline constexpr double kScaleEffort =
  can_bridge_msgs::msg::WireContract::SCALE_EFFORT_NM;

namespace topic
{
// 11 个线上话题名，值全部来自 WireContract.msg。缺一个就意味着某处得自己拼字符串。
inline const std::string & kLowCmd = can_bridge_msgs::msg::WireContract::TOPIC_LOW_CMD;
inline const std::string & kPolicy = can_bridge_msgs::msg::WireContract::TOPIC_POLICY;
inline const std::string & kTrajectory = can_bridge_msgs::msg::WireContract::TOPIC_TRAJECTORY;
inline const std::string & kConfig = can_bridge_msgs::msg::WireContract::TOPIC_CONFIG;
inline const std::string & kLowState = can_bridge_msgs::msg::WireContract::TOPIC_LOW_STATE;
inline const std::string & kMotorStatus = can_bridge_msgs::msg::WireContract::TOPIC_MOTOR_STATUS;
inline const std::string & kTopology = can_bridge_msgs::msg::WireContract::TOPIC_TOPOLOGY;
inline const std::string & kEvents = can_bridge_msgs::msg::WireContract::TOPIC_EVENTS;
inline const std::string & kDeviceParam = can_bridge_msgs::msg::WireContract::TOPIC_DEVICE_PARAM;
inline const std::string & kImuData = can_bridge_msgs::msg::WireContract::TOPIC_IMU_DATA;
inline const std::string & kImuTemp = can_bridge_msgs::msg::WireContract::TOPIC_IMU_TEMP;
}  // namespace topic
}  // namespace contract
}  // namespace micro_ros_bridge

namespace micro_ros_bridge
{
struct Quantized
{
  int16_t q;
  bool saturated;
};

// 定点：q = clamp(round(value * 32767 / scale), -32767, 32767)。
// 与固件 wire_quantize 同式；非有限输入返回哨兵（命令路径的 NaN 由消息级
// 编码函数拒收，不在这里兜）。
inline Quantized quantize(double value, double scale)
{
  constexpr double kQMax = 32767.0;
  if (!std::isfinite(value)) {
    return {contract::kSentinelMissing, false};
  }
  const double raw = value * kQMax / scale;
  // 严格不等，与固件 wire_quantize 逐字同式：**恰好**±full-scale 可精确表示，不算越界、
  // 不置饱和位（Codec.SaturatesInsteadOfRejecting 的 EXPECT_FALSE 钉这一点）。
  if (raw > kQMax) {
    return {static_cast<int16_t>(32767), true};
  }
  if (raw < -kQMax) {
    return {static_cast<int16_t>(-32767), true};
  }
  return {static_cast<int16_t>(std::lround(raw)), false};
}

// 反定点。q == SENTINEL_MISSING → NaN。
inline double dequantize(int16_t q, double scale)
{
  if (q == contract::kSentinelMissing) {
    return std::nan("");
  }
  return static_cast<double>(q) * scale / 32767.0;
}
}  // namespace micro_ros_bridge

#endif  // MICRO_ROS_BRIDGE__CODEC_HPP_
