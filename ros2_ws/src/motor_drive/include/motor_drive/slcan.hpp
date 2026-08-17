#pragma once

#include <cstddef>
#include <string>

#include "motor_drive/can_transport.hpp"

namespace motor_drive {

// CANable2 slcan 固件方言编解码（不依赖串口/内核）。
// 与 src/motor_drive/docs/16-30mm外置驱动器使用说明手册_20260720.pdf 及
// normaldotcom/canable2-fw 固件命令集保持一致：
//   t/T  classic CAN 标准/扩展数据帧
//   d/D  CAN-FD 无 BRS 标准/扩展
//   b/B  CAN-FD + BRS 标准/扩展
// FD 长度码：0-8 原样，9=12, A=16, B=20, C=24, D=32, E=48, F=64。

// 编码为一帧 slcan 行（不含结尾 '\r'）；非法帧返回空串。
std::string encode_slcan_frame(const CanFrame &frame);

// 解码一行完整的 slcan 帧（不含 '\r' 与首尾空白）。
// 状态帧/错误帧/乱码返回 false；成功返回 true 并填充 frame。
bool decode_slcan_line(const std::string &line, CanFrame &frame);

} // namespace motor_drive
