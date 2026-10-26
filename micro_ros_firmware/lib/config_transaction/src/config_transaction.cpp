#include "config_transaction.hpp"

#include <cstring>

namespace config_transaction {
static Result result_for(const Request &r) {
  Result out{};
  out.session_id = r.session_id;
  out.request_id = r.request_id;
  out.kind = r.kind;
  out.command = r.command;
  return out;
}

static bool equal(const Request &a, const Request &b) {
  if (a.session_id != b.session_id || a.request_id != b.request_id ||
      a.kind != b.kind ||
      a.command != b.command || a.motor_id != b.motor_id ||
      a.count != b.count || a.size != b.size) {
    return false;
  }
  for (size_t i = 0; i < a.size; ++i) {
    if (a.targets[i].motor_id != b.targets[i].motor_id ||
        a.targets[i].field != b.targets[i].field ||
        std::memcmp(&a.targets[i].value, &b.targets[i].value, sizeof(float)) != 0) {
      return false;
    }
  }
  return true;
}

void Coordinator::notify(Entry &e) {
  e.dirty = true;
  ++e.generation;
}

void Coordinator::reject(const Request &r, uint8_t status, uint16_t error) {
  for (size_t i = 0; i < reject_count_; ++i) {
    const auto &pending = rejected_[(reject_head_ + i) % kCapacity];
    if (pending.session_id == r.session_id && pending.request_id == r.request_id &&
        pending.kind == r.kind && pending.command == r.command &&
        pending.status == status && pending.error_code == error) {
      return;
    }
  }
  // 拒绝队列满时不执行动作；已排队结果绝不被覆盖。
  if (reject_count_ == kCapacity) {
    return;
  }
  Result out = result_for(r);
  out.status = status;
  out.error_code = error;
  rejected_[(reject_head_ + reject_count_) % kCapacity] = out;
  ++reject_count_;
}

void Coordinator::set_session(uint64_t session) {
  if (session_ == session) {
    return;
  }
  for (auto &e : entries_) { e = Entry{}; }
  reject_head_ = reject_count_ = next_evict_ = 0;
  peek_slot_ = -1;
  peek_rejected_ = false;
  session_ = session;
  high_water_ = 0;
}

int Coordinator::begin(const Request &r, uint64_t current_session, uint32_t now) {
  set_session(current_session);
  if (current_session == 0 || r.session_id != current_session ||
      r.request_id == 0 || r.request_id >= 0x40000000U || r.size > kMaxTargets) {
    reject(r, kFailed, kInvalid);
    return -1;
  }
  for (auto &e : entries_) {
    if (!e.used || e.request.session_id != r.session_id ||
        e.request.request_id != r.request_id) {
      continue;
    }
    if (!equal(e.request, r)) {
      reject(r, kConflict, kInvalid);
    } else {
      notify(e);
    }
    return -1;
  }
  // 终态逐出后仍拒绝旧 ID；桥在同一会话内只分配单调递增的新 ID。
  if (r.request_id <= high_water_) {
    reject(r, kFailed, kExpired);
    return -1;
  }
  for (size_t i = 0; i < kCapacity; ++i) {
    size_t n = (next_evict_ + i) % kCapacity;
    auto &e = entries_[n];
    if (e.used && (e.result.status == kPending || e.dirty ||
        peek_slot_ == static_cast<int>(n))) {
      continue;
    }
    e = Entry{};
    e.used = true;
    e.request = r;
    e.result = result_for(r);
    e.started = now;
    notify(e);
    next_evict_ = (n + 1) % kCapacity;
    high_water_ = r.request_id;
    return static_cast<int>(n);
  }
  reject(r, kBusy, 0);
  return -1;
}

bool Coordinator::track(int slot, uint32_t id) {
  if (slot < 0 || slot >= static_cast<int>(kCapacity) || id == 0) {
    return false;
  }
  auto &e = entries_[slot];
  if (!e.used || e.result.status != kPending || e.sealed) {
    return false;
  }
  for (auto &entry : entries_) {
    for (uint32_t pending : entry.ids) {
      if (pending == id) {
        return false;
      }
    }
  }
  for (auto &pending : e.ids) {
    if (pending == 0) {
      pending = id;
      ++e.outstanding;
      return true;
    }
  }
  return false;
}

void Coordinator::settle(Entry &e) {
  if (e.sealed && !e.external && e.outstanding == 0) {
    e.result.status = e.result.failed_targets || e.result.error_code ? kFailed : kSucceeded;
  }
  notify(e);
}

void Coordinator::submitted(int slot, uint16_t total, uint16_t error) {
  auto &e = entries_[slot];
  e.sealed = true;
  e.result.error_code = error;
  e.result.failed_targets = total > e.outstanding ? total - e.outstanding : 0;
  settle(e);
}

const Result *Coordinator::result(int slot) const {
  return slot >= 0 && slot < static_cast<int>(kCapacity) && entries_[slot].used
      ? &entries_[slot].result : nullptr;
}

bool Coordinator::pending_except(int slot) const {
  for (size_t i = 0; i < kCapacity; ++i) {
    if (static_cast<int>(i) != slot && entries_[i].used &&
        entries_[i].result.status == kPending) {
      return true;
    }
  }
  return false;
}

void Coordinator::busy(int slot) {
  entries_[slot].result.status = kBusy;
  notify(entries_[slot]);
}

void Coordinator::external(int slot) {
  entries_[slot].external = true;
}

void Coordinator::complete(int slot, uint16_t completed, uint16_t failed, uint16_t error) {
  auto &e = entries_[slot];
  e.result.completed_targets = completed;
  e.result.failed_targets = failed;
  e.result.error_code = error;
  e.result.status = failed || error ? kFailed : kSucceeded;
  e.outstanding = 0;
  std::memset(e.ids, 0, sizeof(e.ids));
  notify(e);
}

bool Coordinator::on_result(uint32_t id, bool success, uint16_t error) {
  if (id == 0) {
    return false;
  }
  for (auto &e : entries_) {
    if (!e.used || e.result.status != kPending) {
      continue;
    }
    for (auto &pending : e.ids) {
      if (pending != id) {
        continue;
      }
      pending = 0;
      --e.outstanding;
      if (success) {
        ++e.result.completed_targets;
      } else {
        ++e.result.failed_targets;
        if (!e.result.error_code) {
          e.result.error_code = error ? error : kInvalid;
        }
      }
      settle(e);
      return true;
    }
  }
  return false;
}

void Coordinator::poll(uint32_t now) {
  for (auto &e : entries_) {
    if (e.used && e.result.status == kPending && !e.external &&
        static_cast<uint32_t>(now - e.started) >= kTimeoutMs) {
      complete(static_cast<int>(&e - entries_), e.result.completed_targets,
          e.result.failed_targets + e.outstanding, kTimeout);
    }
  }
}

void Coordinator::disconnect() {
  for (auto &e : entries_) {
    if (e.used && e.result.status == kPending) {
      complete(static_cast<int>(&e - entries_), e.result.completed_targets,
          e.result.failed_targets + e.outstanding, kInvalid);
    }
  }
}

bool Coordinator::peek(Result &out) {
  // 同一 peek/ack 对保持快照，更新由 generation 保留到下一次发送。
  if (peek_rejected_) {
    out = rejected_[reject_head_];
    return true;
  }
  if (peek_slot_ >= 0) {
    out = entries_[peek_slot_].result;
    peek_generation_ = entries_[peek_slot_].generation;
    return true;
  }
  if (reject_count_) {
    peek_rejected_ = true;
    out = rejected_[reject_head_];
    return true;
  }
  for (size_t i = 0; i < kCapacity; ++i) {
    if (entries_[i].dirty) {
      peek_slot_ = static_cast<int>(i);
      peek_generation_ = entries_[i].generation;
      out = entries_[i].result;
      return true;
    }
  }
  return false;
}

void Coordinator::ack() {
  if (peek_rejected_) {
    reject_head_ = (reject_head_ + 1) % kCapacity;
    --reject_count_;
    peek_rejected_ = false;
  } else if (peek_slot_ >= 0) {
    auto &e = entries_[peek_slot_];
    if (e.generation == peek_generation_) {
      e.dirty = false;
    }
    peek_slot_ = -1;
  }
}
}
