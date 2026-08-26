#pragma once

#include "motor_sim/protocol/motor_protocol.hpp"

namespace motor_sim::protocol {

class VaxorProtocol final : public MotorProtocol {
public:
  void validate_motor_id(unsigned motor_id) const override;
  std::optional<MotorCommand>
  decode_command(const CanFrame &frame, unsigned motor_id) const override;
  CanFrame encode_feedback(const MotorFeedback &feedback,
                           FeedbackKind kind) const override;
};

}  // namespace motor_sim::protocol
