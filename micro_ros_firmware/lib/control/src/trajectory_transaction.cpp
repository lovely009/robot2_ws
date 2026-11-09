#include "trajectory_transaction.hpp"

#include <cmath>
#include <cstring>

namespace trajectory_transaction {

bool Coordinator::deadline_valid(uint32_t now_ms, uint32_t deadline_ms) {
  const uint32_t remaining = deadline_ms - now_ms;
  return remaining > 0U && remaining <= 1000U;
}

void Coordinator::begin_session(uint64_t session_id) {
  // 不在栈上构造整个 Coordinator 临时副本。
  std::memset(&assembly_, 0, sizeof(assembly_));
  status_ = {};
  cached_receipt_ = {};
  session_id_ = session_id;
  cached_digest_ = 0U;
  trajectory_high_water_ = 0U;
  phase_tick_ms_ = commit_deadline_ms_ = 0U;
  cached_ = commit_pending_ = cancel_pending_ = finish_pending_ = false;
}

bool Coordinator::live() const {
  return status_.state == AGENT_TRAJECTORY_LOADING ||
      status_.state == AGENT_TRAJECTORY_READY ||
      status_.state == AGENT_TRAJECTORY_RUNNING;
}

bool Coordinator::owns(uint16_t motor_id) const {
  if (!live()) return false;
  for (uint16_t i = 0U; i < assembly_.target_count; ++i) {
    if (assembly_.target_ids[i] == motor_id) return true;
  }
  return false;
}

uint64_t Coordinator::digest(const Request &r) {
  // 逐字段摘要，避免 padding/未使用数组影响幂等；TTL 不属于业务内容。
  uint64_t h = UINT64_C(14695981039346656037);
  const auto add = [&h](const auto &value) {
    const auto *p = reinterpret_cast<const uint8_t *>(&value);
    for (size_t i = 0U; i < sizeof(value); ++i) {
      h = (h ^ p[i]) * UINT64_C(1099511628211);
    }
  };
  add(r.trajectory_id); add(r.operation); add(r.segment_index);
  add(r.segment_count); add(r.target_count); add(r.point_count);
  for (uint16_t i = 0U; i < r.target_count; ++i) add(r.target_ids[i]);
  for (uint16_t i = 0U; i < r.point_count; ++i) {
    add(r.points[i].time_from_start_ms); add(r.points[i].motor_id);
    add(r.points[i].position); add(r.points[i].velocity); add(r.points[i].effort);
  }
  return h;
}

bool Coordinator::request_valid(const Request &r, uint8_t operation) const {
  if (r.operation != operation || r.request_id == 0U ||
      r.request_id > AGENT_TRAJECTORY_MAX_ID || r.trajectory_id == 0U ||
      r.trajectory_id > AGENT_TRAJECTORY_MAX_ID || r.ack_request_id != 0U ||
      r.ack_terminal_trajectory_id != 0U || r.target_count > 48U ||
      r.point_count > AGENT_TRAJECTORY_SEGMENT_POINTS) return false;
  if (operation != AGENT_TRAJECTORY_LOAD) {
    return r.target_count == 0U && r.point_count == 0U &&
        r.segment_index == 0U && r.segment_count == 0U;
  }
  return r.target_count > 0U && r.point_count > 0U &&
      r.segment_count > 0U && r.segment_count <= 256U &&
      r.segment_index < r.segment_count;
}

Receipt Coordinator::reject(const Request &r, Reason reason) {
  Receipt receipt{};
  receipt.request_id = r.request_id;
  receipt.trajectory_id = r.trajectory_id;
  receipt.operation = r.operation;
  receipt.state = reason == REASON_RETIRED ? AGENT_TRAJECTORY_RETIRED :
      AGENT_TRAJECTORY_REJECTED;
  receipt.reason = reason;
  receipt.segments_received = status_.segments_received;
  receipt.segment_count = status_.segment_count;
  status_.receipt = receipt;
  return receipt;
}

bool Coordinator::replay(const Request &r, Receipt *receipt) {
  if (cached_ && r.request_id == cached_receipt_.request_id) {
    if (digest(r) != cached_digest_) *receipt = reject(r, REASON_CONFLICT);
    else {
      *receipt = cached_receipt_;
      status_.receipt = *receipt;
    }
    return true;
  }
  if (r.request_id <= status_.request_high_water) {
    *receipt = reject(r, REASON_RETIRED);
    return true;
  }
  return false;
}

Receipt Coordinator::accept(const Request &r, uint8_t state) {
  cached_receipt_ = {};
  cached_receipt_.request_id = r.request_id;
  cached_receipt_.trajectory_id = r.trajectory_id;
  cached_receipt_.operation = r.operation;
  cached_receipt_.state = state;
  cached_receipt_.segments_received = status_.segments_received;
  cached_receipt_.segment_count = status_.segment_count;
  cached_digest_ = digest(r);
  cached_ = true;
  status_.request_high_water = r.request_id;
  status_.receipt = cached_receipt_;
  return cached_receipt_;
}

void Coordinator::settle(uint8_t state, Reason reason) {
  if (!cached_) return;
  cached_receipt_.state = state;
  cached_receipt_.reason = reason;
  status_.receipt = cached_receipt_;
}

Receipt Coordinator::load(const Request &r, uint32_t now_ms) {
  poll(now_ms);
  if (r.session_id != session_id_ || session_id_ == 0U) return reject(r, REASON_SESSION);
  if (!request_valid(r, AGENT_TRAJECTORY_LOAD)) return reject(r, REASON_INVALID);
  Receipt previous{};
  if (replay(r, &previous)) return previous;
  if (commit_pending_ || cancel_pending_ || status_.state == AGENT_TRAJECTORY_RUNNING ||
      status_.state == AGENT_TRAJECTORY_READY) return reject(r, REASON_BUSY);
  if (!deadline_valid(now_ms, r.valid_until_tick_ms)) return reject(r, REASON_EXPIRED);
  const bool first = status_.state != AGENT_TRAJECTORY_LOADING;
  if (first) {
    if (r.trajectory_id <= trajectory_high_water_) return reject(r, REASON_RETIRED);
    if (r.segment_index != 0U) return reject(r, REASON_ORDER);
  } else {
    if (r.trajectory_id != status_.trajectory_id ||
        r.segment_count != status_.segment_count ||
        r.target_count != assembly_.target_count) return reject(r, REASON_CONFLICT);
    if (r.segment_index != status_.segments_received) return reject(r, REASON_ORDER);
    for (uint16_t i = 0U; i < r.target_count; ++i) {
      if (r.target_ids[i] != assembly_.target_ids[i]) return reject(r, REASON_CONFLICT);
    }
  }
  const uint16_t existing = first ? 0U : assembly_.point_count;
  if (existing + r.point_count > MOTOR_CONTROL_MAX_TRAJECTORY_POINTS)
    return reject(r, REASON_CAPACITY);
  for (uint16_t i = 0U; i < r.target_count; ++i) {
    if (r.target_ids[i] == 0U || r.target_ids[i] > 64U) return reject(r, REASON_TARGET);
    for (uint16_t j = 0U; j < i; ++j) {
      if (r.target_ids[j] == r.target_ids[i]) return reject(r, REASON_TARGET);
    }
  }
  for (uint16_t i = 0U; i < r.point_count; ++i) {
    const auto &p = r.points[i];
    if (!std::isfinite(p.position) || !std::isfinite(p.velocity) ||
        !std::isfinite(p.effort) || p.time_from_start_ms > INT32_MAX)
      return reject(r, REASON_TIMELINE);
    bool target = false;
    for (uint16_t j = 0U; j < r.target_count; ++j) target |= r.target_ids[j] == p.motor_id;
    if (!target) return reject(r, REASON_TARGET);
    if ((i > 0U && p.time_from_start_ms < r.points[i - 1U].time_from_start_ms) ||
        (i == 0U && existing > 0U && p.time_from_start_ms <
            assembly_.points[existing - 1U].time_from_start_ms))
      return reject(r, REASON_TIMELINE);
    // 同时刻不同电机可交错，不能只检查相邻路点。
    for (uint16_t j = 0U; j < existing; ++j) {
      if (assembly_.points[j].motor_id == p.motor_id &&
          assembly_.points[j].time_from_start_ms >= p.time_from_start_ms)
        return reject(r, REASON_TIMELINE);
    }
    for (uint16_t j = 0U; j < i; ++j) {
      if (r.points[j].motor_id == p.motor_id &&
          r.points[j].time_from_start_ms >= p.time_from_start_ms)
        return reject(r, REASON_TIMELINE);
    }
  }
  if (r.segment_index + 1U == r.segment_count) {
    for (uint16_t t = 0U; t < r.target_count; ++t) {
      bool found = false;
      for (uint16_t i = 0U; i < existing; ++i)
        found |= assembly_.points[i].motor_id == r.target_ids[t];
      for (uint16_t i = 0U; i < r.point_count; ++i)
        found |= r.points[i].motor_id == r.target_ids[t];
      if (!found) return reject(r, REASON_TARGET);
    }
  }
  // 所有校验先完成，失败不得污染现有组装区或提升水位。
  if (first) {
    assembly_.point_count = 0U;
    assembly_.target_count = r.target_count;
    for (uint16_t i = 0U; i < r.target_count; ++i) assembly_.target_ids[i] = r.target_ids[i];
    assembly_.trajectory_id = r.trajectory_id;
    assembly_.received_tick_ms = now_ms;
    trajectory_high_water_ = r.trajectory_id;
    status_.trajectory_id = r.trajectory_id;
    status_.target_count = r.target_count;
    for (uint16_t i = 0U; i < 48U; ++i)
      status_.target_ids[i] = i < r.target_count ? r.target_ids[i] : 0U;
    status_.elapsed_ms = status_.duration_ms = 0U;
    status_.reason = REASON_NONE;
    status_.segments_received = 0U;
    status_.segment_count = r.segment_count;
    phase_tick_ms_ = now_ms;
  }
  for (uint16_t i = 0U; i < r.point_count; ++i) {
    auto &p = assembly_.points[assembly_.point_count++];
    p.time_from_start_ms = r.points[i].time_from_start_ms;
    p.motor_id = r.points[i].motor_id;
    p.position_rad = r.points[i].position;
    p.velocity_rad_s = r.points[i].velocity;
    p.effort_nm = r.points[i].effort;
  }
  assembly_.sequence = r.request_id;
  ++status_.segments_received;
  status_.duration_ms = assembly_.points[assembly_.point_count - 1U].time_from_start_ms;
  status_.state = status_.segments_received == status_.segment_count ?
      AGENT_TRAJECTORY_READY : AGENT_TRAJECTORY_LOADING;
  if (status_.state == AGENT_TRAJECTORY_READY) phase_tick_ms_ = now_ms;
  return accept(r, AGENT_TRAJECTORY_ACCEPTED);
}

Receipt Coordinator::commit(const Request &r, uint32_t now_ms) {
  poll(now_ms);
  if (r.session_id != session_id_ || session_id_ == 0U) return reject(r, REASON_SESSION);
  if (!request_valid(r, AGENT_TRAJECTORY_COMMIT)) return reject(r, REASON_INVALID);
  Receipt previous{};
  if (replay(r, &previous)) return previous;
  if (commit_pending_ || cancel_pending_ || status_.state == AGENT_TRAJECTORY_RUNNING)
    return reject(r, REASON_BUSY);
  if (status_.state != AGENT_TRAJECTORY_READY || r.trajectory_id != status_.trajectory_id)
    return reject(r, REASON_TARGET);
  if (!deadline_valid(now_ms, r.valid_until_tick_ms)) return reject(r, REASON_EXPIRED);
  commit_deadline_ms_ = r.valid_until_tick_ms;
  commit_pending_ = true;
  return accept(r, AGENT_TRAJECTORY_PENDING);
}

Receipt Coordinator::cancel(const Request &r) {
  if (r.session_id != session_id_ || session_id_ == 0U) return reject(r, REASON_SESSION);
  if (!request_valid(r, AGENT_TRAJECTORY_CANCEL)) return reject(r, REASON_INVALID);
  Receipt previous{};
  if (replay(r, &previous)) return previous;
  if (r.trajectory_id != status_.trajectory_id || status_.state == AGENT_TRAJECTORY_IDLE)
    return reject(r, REASON_TARGET);
  if (!live()) return accept(r, AGENT_TRAJECTORY_ACCEPTED);
  // 独立抢占未执行的 COMMIT；与时钟/租约/结果槽无关。
  commit_pending_ = false;
  cancel_pending_ = true;
  return accept(r, AGENT_TRAJECTORY_PENDING);
}

bool Coordinator::ack(uint64_t session_id, uint32_t request_id,
    uint32_t terminal_trajectory_id) {
  if (session_id != session_id_ || session_id_ == 0U) return false;
  bool accepted = request_id == 0U && terminal_trajectory_id == 0U;
  if (request_id != 0U && cached_ && request_id == cached_receipt_.request_id &&
      cached_receipt_.state != AGENT_TRAJECTORY_PENDING) {
    cached_ = false;
    if (status_.receipt.request_id == request_id) status_.receipt.state = AGENT_TRAJECTORY_RETIRED;
    accepted = true;
  }
  if (terminal_trajectory_id != 0U && terminal_trajectory_id == status_.trajectory_id &&
      !live() && status_.state != AGENT_TRAJECTORY_IDLE) {
    status_.state = AGENT_TRAJECTORY_IDLE;
    status_.trajectory_id = status_.elapsed_ms = status_.duration_ms = 0U;
    status_.reason = REASON_NONE;
    status_.segments_received = status_.segment_count = 0U;
    status_.target_count = 0U;
    std::memset(status_.target_ids, 0, sizeof(status_.target_ids));
    accepted = true;
  }
  return accepted;
}

void Coordinator::abort(Reason reason) {
  if (!live()) return;
  status_.state = AGENT_TRAJECTORY_ABORTED;
  status_.reason = reason;
  if (commit_pending_ || cancel_pending_) settle(AGENT_TRAJECTORY_REJECTED, reason);
  commit_pending_ = cancel_pending_ = finish_pending_ = false;
}

void Coordinator::poll(uint32_t now_ms) {
  if ((status_.state == AGENT_TRAJECTORY_LOADING || status_.state == AGENT_TRAJECTORY_READY) &&
      !cancel_pending_ && now_ms - phase_tick_ms_ >= 1000U) abort(REASON_TIMEOUT);
}

void Coordinator::boundary(motor_control_context_t *kernel, uint32_t now_ms, bool allowed) {
  poll(now_ms);
  if (cancel_pending_) {
    (void)motor_control_cancel_trajectory_id(kernel, status_.trajectory_id, now_ms);
    if (!motor_control_trajectory_safety_pending(kernel, assembly_.target_ids, assembly_.target_count)) {
      status_.state = AGENT_TRAJECTORY_CANCELED;
      status_.reason = REASON_NONE;
      cancel_pending_ = finish_pending_ = false;
      settle(AGENT_TRAJECTORY_ACCEPTED, REASON_NONE);
    }
    return;
  }
  if (!allowed) {
    if (status_.state == AGENT_TRAJECTORY_RUNNING)
      (void)motor_control_cancel_trajectory_id(kernel, status_.trajectory_id, now_ms);
    abort(REASON_GATE);
    return;
  }
  if (commit_pending_) {
    commit_pending_ = false;
    if (!deadline_valid(now_ms, commit_deadline_ms_)) {
      settle(AGENT_TRAJECTORY_REJECTED, REASON_EXPIRED);
      return;
    }
    if (!motor_control_apply_agent_trajectory(kernel, &assembly_, now_ms)) {
      settle(AGENT_TRAJECTORY_REJECTED, REASON_MEASUREMENT);
      abort(REASON_MEASUREMENT);
      return;
    }
    status_.state = AGENT_TRAJECTORY_RUNNING;
    phase_tick_ms_ = now_ms;
    settle(AGENT_TRAJECTORY_ACCEPTED, REASON_NONE);
  }
  if (status_.state != AGENT_TRAJECTORY_RUNNING) return;
  if (!finish_pending_) motor_control_trajectory_step(kernel, now_ms);
  motor_control_trajectory_state_t state{};
  motor_control_copy_trajectory_state(kernel, &state, now_ms);
  status_.elapsed_ms = now_ms - phase_tick_ms_;
  if (finish_pending_) {
    if (!motor_control_trajectory_safety_pending(kernel, assembly_.target_ids, assembly_.target_count)) {
      status_.state = AGENT_TRAJECTORY_SUCCEEDED;
      finish_pending_ = false;
    }
  } else if (!state.active || state.trajectory_id != status_.trajectory_id) {
    abort(REASON_MEASUREMENT);
  } else if (state.completed) {
    // 末点已经在本边界求值；终态释放归属并提交 P0，避免完成后永久占用执行区。
    (void)motor_control_cancel_trajectory_id(kernel, status_.trajectory_id, now_ms);
    finish_pending_ = true;
    if (!motor_control_trajectory_safety_pending(kernel, assembly_.target_ids, assembly_.target_count)) {
      status_.state = AGENT_TRAJECTORY_SUCCEEDED;
      finish_pending_ = false;
    }
  }
}

}  // namespace trajectory_transaction
