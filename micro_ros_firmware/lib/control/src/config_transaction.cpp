#include "config_transaction.hpp"
#include <cstring>

namespace config_transaction {
static Result result_for(const Request &r) {
  Result out{};
  out.session_id = r.session_id; out.request_id = r.request_id;
  out.command = r.command;
  return out;
}
static uint64_t motor_bit(uint16_t id) {
  return id >= 1 && id <= 64 ? UINT64_C(1) << (id - 1) : 0;
}
static bool equal(const Request &a, const Request &b) {
  if (a.session_id != b.session_id || a.request_id != b.request_id ||
      a.command != b.command || a.motor_id != b.motor_id ||
      a.policy_count != b.policy_count ||
      a.policy_generation != b.policy_generation) return false;
  for (size_t i = 0; i < a.policy_count; ++i) {
    const auto &x = a.policies[i]; const auto &y = b.policies[i];
    if (x.motor_id != y.motor_id || x.enabled != y.enabled ||
        x.confirm_unprotected != y.confirm_unprotected ||
        std::memcmp(&x.kp, &y.kp, sizeof(float)) ||
        std::memcmp(&x.kd, &y.kd, sizeof(float))) return false;
  }
  return true;
}
bool Coordinator::valid_slot(int slot) const {
  return slot >= 0 && slot < static_cast<int>(kCapacity) && entries_[slot].used;
}
void Coordinator::reject(const Request &r, uint8_t status, uint16_t error) {
  rejection_ = result_for(r); rejection_.status = status; rejection_.error_code = error;
  have_rejection_ = true;
}
void Coordinator::set_session(uint64_t session) {
  if (session_ == session) return;
  for (auto &e : entries_) e = Entry{};
  have_rejection_ = false; session_ = session; high_water_ = 0;
}
int Coordinator::begin(const Request &r, uint64_t session, uint32_t now) {
  set_session(session);
  if (!session || r.session_id != session || !r.request_id ||
      r.request_id >= 0x40000000U || r.policy_count > kMaxOperations) {
    reject(r, kFailed, kInvalid); return -1;
  }
  for (auto &e : entries_) {
    if (!e.used || e.request.request_id != r.request_id) continue;
    if (!equal(e.request, r)) reject(r, kConflict, kInvalid);
    return -1;
  }
  if (r.request_id <= high_water_) { reject(r, kRetired, kExpired); return -1; }
  const uint32_t remaining = r.valid_until_tick_ms - now;
  if (!remaining || remaining > 1000U) { reject(r, kFailed, kExpired); return -1; }
  for (size_t i = 0; i < kCapacity; ++i) {
    auto &e = entries_[i];
    if (e.used) continue;
    e = Entry{}; e.used = true; e.request = r; e.result = result_for(r); e.started = now;
    e.previous_high_water = high_water_;
    high_water_ = r.request_id;
    return static_cast<int>(i);
  }
  reject(r, kBusy, 0); return -1;
}
bool Coordinator::track(int slot, uint32_t id, uint16_t motor_id, uint8_t field) {
  if (!valid_slot(slot) || !id) return false;
  auto &e = entries_[slot];
  if (e.result.status != kPending || e.sealed || e.detail_count == kMaxOperations) return false;
  for (const auto &entry : entries_) {
    for (size_t i = 0; i < entry.detail_count; ++i) {
      if (entry.ids[i] == id && entry.details[i].motor_id == motor_id) return false;
    }
  }
  const size_t i = e.detail_count++;
  e.ids[i] = id; e.details[i].motor_id = motor_id; e.details[i].field = field;
  ++e.outstanding; return true;
}
void Coordinator::settle(Entry &e) {
  if (e.sealed && !e.outstanding) {
    e.result.status = e.result.failed_targets || e.result.error_code
        ? (e.result.completed_targets ? kPartial : kFailed) : kSucceeded;
  }
}
void Coordinator::submitted(int slot, uint16_t total, uint16_t error) {
  if (!valid_slot(slot)) return;
  auto &e = entries_[slot];
  e.sealed = true; e.result.error_code = error;
  e.result.failed_targets = total > e.outstanding ? total - e.outstanding : 0;
  settle(e);
}
const Result *Coordinator::result(int slot) const {
  return valid_slot(slot) ? &entries_[slot].result : nullptr;
}
bool Coordinator::pending_device_command(uint8_t command, uint16_t motor_id) const {
  for (const auto &entry : entries_)
    if (entry.used && entry.result.status == kPending &&
        entry.request.command == command && entry.request.motor_id == motor_id) return true;
  return false;
}
bool Coordinator::can_start(int slot, uint32_t now) {
  if (!valid_slot(slot) || entries_[slot].result.status != kPending) return false;
  const uint32_t remaining = entries_[slot].request.valid_until_tick_ms - now;
  if (remaining && remaining <= 1000U) return true;
  complete(slot, 0, 0, kExpired); return false;
}
void Coordinator::busy(int slot) {
  if (!valid_slot(slot)) return;
  // 运行前互斥拒绝不占槽、不提升水位，允许原 ID 重试。
  const auto &request = entries_[slot].request;
  reject(request, kBusy, 0);
  if (high_water_ == request.request_id) high_water_ = entries_[slot].previous_high_water;
  entries_[slot] = Entry{};
}
void Coordinator::complete(int slot, uint16_t completed, uint16_t failed, uint16_t error) {
  if (!valid_slot(slot) || entries_[slot].result.status != kPending) return;
  auto &e = entries_[slot];
  e.result.completed_targets = completed; e.result.failed_targets = failed; e.result.error_code = error;
  e.result.status = failed || error ? (completed ? kPartial : kFailed) : kSucceeded;
  for (size_t i = 0; i < e.detail_count; ++i) {
    if (!e.details[i].complete) {
      e.details[i].complete = true; e.details[i].error = error ? error : kInvalid;
      e.result.failed_motor_mask |= motor_bit(e.details[i].motor_id);
    }
  }
  e.outstanding = 0; std::memset(e.ids, 0, sizeof(e.ids));
}
bool Coordinator::on_result(uint32_t id, bool success, uint16_t error, uint16_t motor_id) {
  if (!id) return false;
  for (auto &e : entries_) {
    if (!e.used || e.result.status != kPending) continue;
    for (size_t i = 0; i < e.detail_count; ++i) {
      auto &d = e.details[i];
      if (e.ids[i] != id || (motor_id != UINT16_MAX && motor_id != d.motor_id)) continue;
      e.ids[i] = 0; --e.outstanding; d.complete = true; d.error = success ? 0 : (error ? error : kInvalid);
      if (success) { ++e.result.completed_targets; e.result.completed_motor_mask |= motor_bit(d.motor_id); }
      else { ++e.result.failed_targets; e.result.failed_motor_mask |= motor_bit(d.motor_id); if (!e.result.error_code) e.result.error_code = d.error; }
      settle(e); return true;
    }
  }
  return false;
}
void Coordinator::poll(uint32_t now) {
  for (auto &e : entries_) {
    if (e.used && e.result.status == kPending && static_cast<uint32_t>(now - e.started) >= kTimeoutMs)
      complete(static_cast<int>(&e - entries_), e.result.completed_targets, e.result.failed_targets + e.outstanding, kTimeout);
  }
}
void Coordinator::disconnect() {
  for (auto &e : entries_) if (e.used && e.result.status == kPending)
    complete(static_cast<int>(&e - entries_), e.result.completed_targets, e.result.failed_targets + e.outstanding, kInvalid);
}
bool Coordinator::ack_result(uint64_t session, uint32_t request_id) {
  if (!session || session != session_) return false;
  for (size_t i = 0; i < kCapacity; ++i) {
    auto &e = entries_[i];
    if (e.used && e.request.request_id == request_id && e.result.status != kPending) {
      e = Entry{};
      return true;
    }
  }
  return false;
}
size_t Coordinator::copy_results(Result *out, size_t capacity) const {
  if (!out) return 0;
  size_t n = 0;
  for (const auto &e : entries_) if (e.used && n < capacity) out[n++] = e.result;
  return n;
}
bool Coordinator::copy_rejection(Result &out) const {
  if (!have_rejection_) return false;
  out = rejection_; return true;
}
size_t Coordinator::copy_details(uint64_t session, uint32_t id, Detail *out, size_t capacity, size_t cursor) const {
  if (!out || session != session_) return 0;
  for (const auto &e : entries_) {
    if (!e.used || e.request.request_id != id) continue;
    size_t n = 0;
    for (size_t i = cursor; i < e.detail_count && n < capacity; ++i) out[n++] = e.details[i];
    return n;
  }
  return 0;
}
}
