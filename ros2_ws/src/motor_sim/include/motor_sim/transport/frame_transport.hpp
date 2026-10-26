#pragma once

#include <chrono>

#include "motor_sim/protocol/can_frame.hpp"

namespace motor_sim::transport {

// 一个进程只能选择一种传输模式；socket_can 走 Linux SocketCAN（如 vcan0），
// 其余两种走串口承载的 ASCII 帧记录。
enum class TransportMode { UsbSerial, SerialSlcan, SocketCan };

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

}  // namespace motor_sim::transport
