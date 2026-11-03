#ifndef CONFIG_TRANSACTION_H
#define CONFIG_TRANSACTION_H

#include <cstddef>
#include <cstdint>
#include "firmware_config.h"

namespace config_transaction {
constexpr size_t kCapacity = 8, kMaxTargets = 16, kMaxOperations = FIRMWARE_MAX_MOTORS;
constexpr uint32_t kTimeoutMs = 5000;
constexpr uint8_t kPending = 0, kSucceeded = 1, kFailed = 2, kPartial = 3,
    kBusy = 4, kConflict = 5, kRetired = 6;
constexpr uint16_t kInvalid = 10, kTimeout = 21, kExpired = 22;

struct Target { uint16_t motor_id; uint8_t field; float value; };
struct Policy {
  uint16_t motor_id;
  bool enabled, confirm_unprotected;
  float kp, kd;
};
struct Request {
  uint64_t session_id = 0, sync_time_ns = 0;
  uint32_t request_id = 0, valid_until_tick_ms = 0, policy_generation = 0;
  uint8_t kind = 0, command = 0;
  uint16_t motor_id = 0, count = 0;
  size_t size = 0, policy_count = 0;
  Target targets[kMaxTargets]{};
  Policy policies[kMaxOperations]{};
};
struct Result {
  uint64_t session_id = 0;
  uint32_t request_id = 0;
  uint8_t kind = 0, command = 0, status = kPending;
  uint16_t error_code = 0, completed_targets = 0, failed_targets = 0;
  uint64_t completed_motor_mask = 0, failed_motor_mask = 0;
};
struct Detail {
  uint16_t motor_id = UINT16_MAX;
  uint8_t field = UINT8_MAX;
  uint16_t error = 0;
  bool complete = false;
};

// 固定八槽含 PENDING 与未确认终态；发布只能清 dirty，不能回收记录。
class Coordinator {
public:
  int begin(const Request &, uint64_t current_session, uint32_t now);
  void set_session(uint64_t session);
  uint32_t request_high_water() const { return high_water_; }
  bool track(int slot, uint32_t operation_id, uint16_t motor_id = UINT16_MAX,
      uint8_t field = UINT8_MAX);
  void submitted(int slot, uint16_t total, uint16_t error);
  void external(int slot);
  void busy(int slot);
  const Result *result(int slot) const;
  bool pending_except(int slot) const;
  bool pending_device_command(uint8_t command, uint16_t motor_id) const;
  bool can_start(int slot, uint32_t now);
  void complete(int slot, uint16_t completed, uint16_t failed, uint16_t error);
  bool on_result(uint32_t operation_id, bool success, uint16_t error,
      uint16_t motor_id = UINT16_MAX);
  void poll(uint32_t now);
  void disconnect();
  bool ack_result(uint64_t session_id, uint32_t request_id);
  size_t copy_results(Result *out, size_t capacity) const;
  void reject(const Request &, uint8_t status, uint16_t error);
  bool copy_rejection(Result &out) const;
  size_t copy_details(uint64_t session_id, uint32_t request_id,
      Detail *out, size_t capacity, size_t cursor = 0) const;
  // 过渡发布接口：不改变槽寿命，最终 status 读取 copy_results。
  bool peek(Result &out);
  void ack();
private:
  struct Entry {
    Request request{};
    Result result{};
    uint32_t ids[kMaxOperations]{};
    Detail details[kMaxOperations]{};
    uint32_t started = 0, generation = 0, previous_high_water = 0;
    uint16_t outstanding = 0, detail_count = 0;
    bool used = false, dirty = false, external = false, sealed = false;
  };
  Entry entries_[kCapacity]{};
  Result rejection_{};
  bool have_rejection_ = false, rejection_dirty_ = false;
  uint64_t session_ = 0;
  uint32_t high_water_ = 0;
  int peek_slot_ = -1;
  uint32_t peek_generation_ = 0;
  bool peek_rejected_ = false;
  bool valid_slot(int slot) const;
  void notify(Entry &);
  void settle(Entry &);
};
}
#endif
