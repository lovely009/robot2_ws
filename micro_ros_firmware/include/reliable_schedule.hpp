#ifndef RELIABLE_SCHEDULE_HPP
#define RELIABLE_SCHEDULE_HPP

#include <cstdint>
#include "firmware_config.h"

enum class ReliablePublication { None, Status, Info };

struct ReliableSchedule {
  uint32_t last_publish = 0, status_attempt = 0, status_success = 0;
  uint8_t status_streak = 0;

  ReliablePublication next(uint32_t now, bool dirty, bool info_pending) const {
    if (static_cast<uint32_t>(now - last_publish) < 10U) return ReliablePublication::None;
    const bool status_due = static_cast<uint32_t>(now - status_attempt) >= 20U &&
        (dirty || static_cast<uint32_t>(now - status_success) >= FIRMWARE_MOTOR_REPORT_PERIOD_MS);
    if (status_due && (!info_pending || status_streak < 2U)) return ReliablePublication::Status;
    return info_pending ? ReliablePublication::Info : ReliablePublication::None;
  }

  void record(uint32_t now, ReliablePublication publication, bool success) {
    last_publish = now;
    if (publication == ReliablePublication::Status) {
      status_attempt = now;
      if (status_streak < 2U) ++status_streak;
      if (success) status_success = now;
    } else {
      status_streak = 0;
    }
  }
};
#endif
