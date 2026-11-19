#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace motor_hardware_interface {

// Retarget a trapezoidal profile from its commanded P/V, preserving both across input updates.
class PositionProfile {
public:
  PositionProfile(double velocity, double acceleration) : vmax_(velocity), amax_(acceleration) {
    if (!std::isfinite(vmax_) || !std::isfinite(amax_) || vmax_ <= 0 || amax_ <= 0) {
      throw std::invalid_argument("profile limits must be finite and positive");
    }
  }
  void reset(double position) {
    position_ = target_ = position;
    velocity_ = 0;
    remaining_.fill(0);
  }
  bool target(double position) {
    if (!std::isfinite(position)) return false;
    target_ = position;
    const double stopping = position_ + velocity_ * std::abs(velocity_) / (2 * amax_);
    const double direction = target_ >= stopping ? 1 : -1;
    const double distance = direction * (target_ - position_);
    const double speed = direction * velocity_;
    const double peak = std::min(vmax_, std::sqrt(std::max(0.0,
      amax_ * distance + speed * speed / 2)));
    remaining_ = {std::max(0.0, (peak - speed) / amax_),
      std::max(0.0, (distance - (2 * peak * peak - speed * speed) / (2 * amax_)) / vmax_),
      peak / amax_};
    acceleration_ = {direction * amax_, 0, -direction * amax_};
    return true;
  }
  void advance(double seconds) {
    for (std::size_t i = 0; i < remaining_.size(); ++i) {
      const double dt = std::min(seconds, remaining_[i]);
      position_ += velocity_ * dt + acceleration_[i] * dt * dt / 2;
      velocity_ += acceleration_[i] * dt;
      remaining_[i] -= dt;
      seconds -= dt;
      if (remaining_[i] > 0) return;
    }
    position_ = target_;
    velocity_ = 0;
  }
  double position() const { return position_; }
  double velocity() const { return velocity_; }

private:
  double vmax_, amax_, position_{0}, velocity_{0}, target_{0};
  std::array<double, 3> remaining_{}, acceleration_{};
};

}  // namespace motor_hardware_interface
