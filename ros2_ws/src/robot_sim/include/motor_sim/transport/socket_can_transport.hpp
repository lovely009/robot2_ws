#pragma once

#include <atomic>
#include <chrono>
#include <string>

#include "motor_sim/transport/frame_transport.hpp"

namespace motor_sim::transport {

// 通过 Linux SocketCAN 原始套接字（例如 vcan0 虚拟接口）收发帧，
// 供本机 vcan 人工联调使用；不经串口，也没有适配器初始化序列。
class SocketCanTransport final : public FrameTransport {
public:
  explicit SocketCanTransport(std::string interface_name);
  ~SocketCanTransport() override;

  SocketCanTransport(const SocketCanTransport &) = delete;
  SocketCanTransport &operator=(const SocketCanTransport &) = delete;

  void open() override;
  void close() noexcept override;
  bool send(const protocol::CanFrame &frame) override;
  ReceiveResult receive(ReceivedFrame &frame) override;

private:
  void mark_disconnected() noexcept;

  std::string interface_name_;
  int fd_{-1};
  std::atomic<bool> disconnected_{true};
  // 内核 RX 时间戳是 CLOCK_REALTIME；打开时记录与 steady_clock 的偏移，
  // 使 duplicate-controller 窗口测量与串口路径一样基于单调时钟。
  std::chrono::steady_clock::duration realtime_to_steady_{
      std::chrono::steady_clock::duration::zero()};
};

}  // namespace motor_sim::transport
