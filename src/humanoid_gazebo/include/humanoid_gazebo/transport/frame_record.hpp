#pragma once

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

#include "humanoid_gazebo/protocol/can_frame.hpp"

namespace humanoid_gazebo::transport {

std::string encode_frame_record(const protocol::CanFrame &frame);

class FrameRecordParser {
public:
  std::vector<protocol::CanFrame> push(std::string_view bytes);
  std::size_t rejected_records() const noexcept { return rejected_records_; }

private:
  static constexpr std::size_t kMaximumRecordSize = 256U;
  std::string pending_;
  std::size_t rejected_records_{0U};
  bool dropping_overlong_record_{false};
};

}  // namespace humanoid_gazebo::transport
