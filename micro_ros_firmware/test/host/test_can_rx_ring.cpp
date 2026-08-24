#include "can_rx_queue.h"

#include <array>
#include <cassert>
#include <cstddef>

// 这是 host 侧策略替身，只验证容量与 drop-oldest 语义；FreeRTOS Queue 集成
// 由目标板 clean build 校验，不把本替身当作 RTOS 集成测试。
template <size_t Capacity>
class FixedRxRing {
 public:
  void push(const can_rx_frame_t &frame) {
    ++diagnostics_.rx_count;
    if (size_ == Capacity) {
      tail_ = (tail_ + 1U) % Capacity;
      --size_;
      ++diagnostics_.overflow_count;
    }
    storage_[head_] = frame;
    head_ = (head_ + 1U) % Capacity;
    ++size_;
  }

  bool pop(can_rx_frame_t *out) {
    if (out == nullptr || size_ == 0U) {
      return false;
    }
    *out = storage_[tail_];
    tail_ = (tail_ + 1U) % Capacity;
    --size_;
    return true;
  }

  const can_rx_diagnostics_t &diagnostics() const {
    return diagnostics_;
  }

 private:
  std::array<can_rx_frame_t, Capacity> storage_{};
  size_t head_ = 0U;
  size_t tail_ = 0U;
  size_t size_ = 0U;
  can_rx_diagnostics_t diagnostics_{};
};

template <size_t Capacity>
static void test_drop_oldest_policy(uint8_t bus_index) {
  FixedRxRing<Capacity> ring;
  for (size_t sequence = 1U; sequence <= Capacity + 1U; ++sequence) {
    can_rx_frame_t frame{};
    frame.bus_index = bus_index;
    frame.id = static_cast<uint32_t>(sequence);
    ring.push(frame);
  }

  assert(ring.diagnostics().rx_count == Capacity + 1U);
  assert(ring.diagnostics().overflow_count == 1U);
  assert(ring.diagnostics().hal_receive_error_count == 0U);
  for (size_t expected = 2U; expected <= Capacity + 1U; ++expected) {
    can_rx_frame_t frame{};
    assert(ring.pop(&frame));
    assert(frame.bus_index == bus_index);
    assert(frame.id == expected);
  }
  can_rx_frame_t empty{};
  assert(!ring.pop(&empty));
}

int main() {
  test_drop_oldest_policy<CAN1_RX_QUEUE_CAPACITY>(0U);
  test_drop_oldest_policy<CAN_OTHER_RX_QUEUE_CAPACITY>(1U);
  test_drop_oldest_policy<CAN_OTHER_RX_QUEUE_CAPACITY>(2U);
  return 0;
}
