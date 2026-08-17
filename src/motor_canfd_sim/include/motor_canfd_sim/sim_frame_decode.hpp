#pragma once

#include <chrono>
#include <cstddef>
#include <vector>

#include "motor_drive/motor_driver.hpp"
#include "motor_drive/motor_protocol.hpp"
#include "motor_drive/protocol.hpp"

namespace motor_canfd_sim {

namespace drive = motor_drive;
namespace protocol = motor_drive::protocol;
using Clock = std::chrono::steady_clock;

// 一次帧解析对单台电机产生的状态更新（纯数据，不接触 socket/ROS）。
struct SimFrameUpdate {
  std::size_t motor{0U};
  bool set_enabled{false};
  bool enabled{false};
  bool set_command{false};
  protocol::JointCommand command{};
  bool set_control{false};
  protocol::WristControl control{};
};

// 一帧的解析结果：consumed 表示该帧属于本家族并被消费；
// needs_feedback 表示需要立即回反馈帧；updates 按电机列出状态更新。
struct SimFrameResult {
  bool consumed{false};
  bool needs_feedback{false};
  std::vector<SimFrameUpdate> updates;
};

// RP 家族帧解析（纯逻辑）：识别使能广播（0x00）与 64 Byte 槽位命令（0x80），
// 按 config.id 计算槽位偏移。bus_motors 是挂在该总线上的电机下标。
SimFrameResult decode_rp_frame(const drive::DriverConfig &config,
                               const std::vector<std::size_t> &bus_motors,
                               const drive::CanFrame &frame);

// 25/30 mm 家族帧解析（纯逻辑）：识别按节点 ID 寻址的控制/命令帧。
SimFrameResult decode_motor_2530_frame(const drive::DriverConfig &config,
                                       const std::vector<std::size_t> &bus_motors,
                                       const drive::CanFrame &frame);

} // namespace motor_canfd_sim
