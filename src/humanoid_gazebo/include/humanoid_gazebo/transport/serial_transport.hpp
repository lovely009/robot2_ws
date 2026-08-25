#pragma once

#include <atomic>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#include "humanoid_gazebo/transport/frame_record.hpp"
#include "humanoid_gazebo/transport/frame_transport.hpp"

namespace humanoid_gazebo::transport {

enum class TransportMode { UsbSerial, SerialSlcan };

class SerialTransport final : public FrameTransport {
public:
  SerialTransport(std::string device, TransportMode mode, int baud_rate);
  ~SerialTransport() override;

  SerialTransport(const SerialTransport &) = delete;
  SerialTransport &operator=(const SerialTransport &) = delete;

  void open() override;
  void close() noexcept override;
  bool send(const protocol::CanFrame &frame) override;
  ReceiveResult receive(ReceivedFrame &frame) override;

private:
  bool write_bytes(const std::string &bytes) noexcept;
  void reader_loop() noexcept;
  void mark_disconnected() noexcept;

  std::string device_;
  TransportMode mode_;
  int baud_rate_;
  int fd_{-1};
  std::atomic<bool> disconnected_{true};
  std::atomic<bool> stop_reader_{true};
  std::thread reader_thread_;
  std::mutex write_mutex_;
  std::mutex receive_mutex_;
  FrameRecordParser parser_;
  std::deque<ReceivedFrame> received_frames_;
};

}  // namespace humanoid_gazebo::transport
