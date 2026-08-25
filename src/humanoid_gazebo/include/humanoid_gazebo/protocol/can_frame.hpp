#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace humanoid_gazebo::protocol {

// 协议层自己的 CAN/CAN-FD 帧表示，不依赖 Linux 内核网络类型。
struct CanFrame {
  uint32_t id{0U};
  bool extended{false};
  bool fd{false};
  bool brs{false};
  std::array<uint8_t, 64> data{};
  std::size_t size{0U};
};

}  // namespace humanoid_gazebo::protocol
