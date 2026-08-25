#include "humanoid_gazebo/protocol/rp_protocol.hpp"

#include <algorithm>
#include <stdexcept>

namespace humanoid_gazebo::protocol {
namespace {

CanFrame make_frame(const uint32_t id, const uint8_t *data,
                    const std::size_t size, const bool fd) {
  if (data == nullptr || size > CanFrame{}.data.size()) {
    throw std::invalid_argument("invalid CAN frame payload");
  }
  CanFrame frame;
  frame.id = id;
  frame.fd = fd;
  frame.brs = fd;
  frame.size = size;
  std::copy(data, data + size, frame.data.begin());
  return frame;
}

}  // namespace

void RpProtocol::validate_motor_id(const unsigned motor_id) const {
  if (motor_id < 1U || motor_id > kRpBroadcastSlots) {
    throw std::out_of_range("RP motor ID must be in [1, 8]");
  }
}

std::optional<MotorCommand>
RpProtocol::decode_command(const CanFrame &frame,
                           const unsigned motor_id) const {
  validate_motor_id(motor_id);
  if (frame.extended) {
    return std::nullopt;
  }
  if (frame.id == kRpEnableId && !frame.fd && frame.size == 8U) {
    MotorCommand command;
    command.motor_id = motor_id;
    command.enabled = frame.data[0] != 0U;
    command.mode = command.enabled ? CommandMode::Enable : CommandMode::Disable;
    return command;
  }
  if (frame.id != kRpCommandId || !frame.fd || frame.size != 64U) {
    return std::nullopt;
  }
  const auto offset = (motor_id - 1U) * 8U;
  auto command = unpack_rp_slot(frame.data.data() + offset, 8U);
  command.motor_id = motor_id;
  return command;
}

CanFrame RpProtocol::encode_feedback(const MotorFeedback &feedback,
                                     const FeedbackKind) const {
  validate_motor_id(feedback.motor_id);
  const auto payload = pack_rp_state(feedback);
  return make_frame(feedback.motor_id, payload.data(), payload.size(), true);
}

}  // namespace humanoid_gazebo::protocol
