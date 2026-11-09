#ifndef TRAJECTORY_TRANSACTION_HPP
#define TRAJECTORY_TRANSACTION_HPP

#include "agent_trajectory.h"
#include "motor_control.h"

namespace trajectory_transaction {

// 原因独立于厂家故障码；线上原样保留 uint16_t 数值。
enum Reason : uint16_t {
  REASON_NONE = 0, REASON_SESSION = 1, REASON_INVALID = 2,
  REASON_EXPIRED = 3, REASON_BUSY = 4, REASON_RETIRED = 5,
  REASON_CONFLICT = 6, REASON_ORDER = 7, REASON_TIMELINE = 8,
  REASON_CAPACITY = 9, REASON_TIMEOUT = 10, REASON_GATE = 11,
  REASON_MEASUREMENT = 12, REASON_STOPPED = 13, REASON_TARGET = 14
};

using Request = agent_trajectory_request_t;
using Receipt = agent_trajectory_receipt_t;

// 调用方串行化；不依赖 ROS/RTOS、不分配。唯一大对象是组装区，
// boundary 直接借用该区交给内核，不经过另一份完整转换缓冲。
class Coordinator {
 public:
  void begin_session(uint64_t session_id);
  Receipt load(const Request &request, uint32_t now_ms);
  Receipt commit(const Request &request, uint32_t now_ms);
  Receipt cancel(const Request &request);
  bool ack(uint64_t session_id, uint32_t request_id,
      uint32_t terminal_trajectory_id);
  Receipt reject(const Request &request, Reason reason);
  void poll(uint32_t now_ms);
  void abort(Reason reason);
  // 仅实时唯一写者调用；allowed 含当前会话/时钟/租约/全局授权。
  void boundary(motor_control_context_t *kernel, uint32_t now_ms, bool allowed);
  const agent_trajectory_status_t &status() const { return status_; }
  const motor_control_trajectory_t &assembly() const { return assembly_; }
  bool owns(uint16_t motor_id) const;

 private:
  bool replay(const Request &request, Receipt *receipt);
  Receipt accept(const Request &request, uint8_t state);
  bool request_valid(const Request &request, uint8_t operation) const;
  static uint64_t digest(const Request &request);
  static bool deadline_valid(uint32_t now_ms, uint32_t deadline_ms);
  void settle(uint8_t receipt_state, Reason reason);
  bool live() const;

  motor_control_trajectory_t assembly_{};
  agent_trajectory_status_t status_{};
  Receipt cached_receipt_{};
  uint64_t session_id_ = 0U;
  uint64_t cached_digest_ = 0U;
  uint32_t trajectory_high_water_ = 0U;
  uint32_t phase_tick_ms_ = 0U;
  uint32_t commit_deadline_ms_ = 0U;
  bool cached_ = false;
  bool commit_pending_ = false;
  bool cancel_pending_ = false;
  bool finish_pending_ = false;
};

}  // namespace trajectory_transaction
#endif
