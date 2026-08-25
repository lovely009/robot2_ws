#pragma once

#include "humanoid_gazebo/protocol/motor_protocol.hpp"

namespace humanoid_gazebo::protocol {

class RpProtocol final : public MotorProtocol {
public:
  void validate_motor_id(unsigned motor_id) const override;
  std::optional<MotorCommand>
  decode_command(const CanFrame &frame, unsigned motor_id) const override;
  CanFrame encode_feedback(const MotorFeedback &feedback,
                           FeedbackKind kind) const override;
};

}  // namespace humanoid_gazebo::protocol
