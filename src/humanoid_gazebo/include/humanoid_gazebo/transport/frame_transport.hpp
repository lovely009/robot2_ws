#pragma once

#include <chrono>

#include "humanoid_gazebo/protocol/can_frame.hpp"

namespace humanoid_gazebo::transport {

enum class ReceiveResult { Frame, NoData, Disconnected };

struct ReceivedFrame {
  protocol::CanFrame frame;
  std::chrono::steady_clock::time_point received_at;
};

class FrameTransport {
public:
  virtual ~FrameTransport() = default;
  virtual void open() = 0;
  virtual void close() noexcept = 0;
  virtual bool send(const protocol::CanFrame &frame) = 0;
  virtual ReceiveResult receive(ReceivedFrame &frame) = 0;
};

}  // namespace humanoid_gazebo::transport
