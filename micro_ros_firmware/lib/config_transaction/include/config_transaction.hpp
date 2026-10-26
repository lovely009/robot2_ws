#ifndef CONFIG_TRANSACTION_H
#define CONFIG_TRANSACTION_H

#include <cstddef>
#include <cstdint>

namespace config_transaction {
constexpr size_t kCapacity = 8;
constexpr size_t kMaxTargets = 16;
constexpr uint32_t kTimeoutMs = 5000;
constexpr uint8_t kPending = 0, kSucceeded = 1, kFailed = 2, kBusy = 3, kConflict = 4;
constexpr uint16_t kInvalid = 10, kTimeout = 21, kExpired = 22;

struct Target {
  uint16_t motor_id;
  uint8_t field;
  float value;
};
struct Request {
  uint64_t session_id = 0;
  uint64_t sync_time_ns = 0;
  uint32_t request_id = 0;
  uint8_t kind = 0, command = 0;
  uint16_t motor_id = 0, count = 0;
  size_t size = 0;
  Target targets[kMaxTargets]{};
};
struct Result {
  uint64_t session_id = 0;
  uint32_t request_id = 0;
  uint8_t kind = 0, command = 0, status = kPending;
  uint16_t error_code = 0, completed_targets = 0, failed_targets = 0;
};

// 无 ROS/RTOS 依赖；每个实际设备操作有独立关联 ID。
class Coordinator {
public:
int begin(const Request &request, uint64_t current_session, uint32_t now);
void set_session(uint64_t session);
uint32_t request_high_water() const {
  return high_water_;
}
bool track(int slot, uint32_t operation_id);
void submitted(int slot, uint16_t total, uint16_t error);
void external(int slot);
void busy(int slot);
const Result *result(int slot) const;
bool pending_except(int slot) const;
void complete(int slot, uint16_t completed, uint16_t failed, uint16_t error);
bool on_result(uint32_t operation_id, bool success, uint16_t error);
void poll(uint32_t now);
void disconnect();
bool peek(Result &out);
void ack();
private:
struct Entry {
  Request request{};
  Result result{};
  uint32_t ids[kMaxTargets]{};
  uint32_t started = 0, generation = 0;
  uint16_t outstanding = 0;
  bool used = false, dirty = false, external = false, sealed = false;
};
Entry entries_[kCapacity]{};
Result rejected_[kCapacity]{};
size_t reject_head_ = 0, reject_count_ = 0, next_evict_ = 0;
uint64_t session_ = 0;
uint32_t high_water_ = 0;
int peek_slot_ = -1;
uint32_t peek_generation_ = 0;
bool peek_rejected_ = false;
void notify(Entry &entry);
void settle(Entry &entry);
void reject(const Request &request, uint8_t status, uint16_t error);
};
}
#endif
