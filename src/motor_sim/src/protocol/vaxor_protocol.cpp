#include "motor_sim/protocol/vaxor_protocol.hpp"

#include <algorithm>
#include <stdexcept>

namespace motor_sim::protocol {
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

void VaxorProtocol::validate_motor_id(const unsigned motor_id) const {
  validate_vaxor_node(motor_id);
}

std::optional<MotorCommand>
VaxorProtocol::decode_command(const CanFrame &frame,
                              const unsigned motor_id) const {
  validate_motor_id(motor_id);
  if (frame.extended) {
    return std::nullopt;
  }
  if (frame.id == vaxor_control_id(motor_id) && !frame.fd &&
      frame.size == 8U) {
    auto command = unpack_vaxor_control(frame.data.data(), frame.size);
    command.motor_id = motor_id;
    return command;
  }
  if (frame.id == vaxor_command_id(motor_id) && frame.fd &&
      frame.size == 12U) {
    auto command = unpack_vaxor_command(frame.data.data(), frame.size);
    command.motor_id = motor_id;
    return command;
  }
  return std::nullopt;
}

CanFrame VaxorProtocol::encode_feedback(const MotorFeedback &feedback,
                                        const FeedbackKind kind) const {
  validate_motor_id(feedback.motor_id);
  if (kind == FeedbackKind::Status) {
    const auto payload = pack_vaxor_status(feedback);
    return make_frame(vaxor_status_id(feedback.motor_id), payload.data(),
                      payload.size(), true);
  }
  const auto payload = pack_vaxor_reply(feedback);
  return make_frame(vaxor_reply_id(feedback.motor_id), payload.data(),
                    payload.size(), true);
}

}  // namespace motor_sim::protocol
