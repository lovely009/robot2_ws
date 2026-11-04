#include "reliable_schedule.hpp"
#include <cassert>
#include <cstdint>

int main() {
  ReliableSchedule schedule{};
  assert(schedule.next(19, false, false) == ReliablePublication::None);
  assert(schedule.next(20, false, true) == ReliablePublication::Status);
  schedule.record(20, ReliablePublication::Status, true);
  assert(schedule.next(29, true, true) == ReliablePublication::None);
  assert(schedule.next(30, true, false) == ReliablePublication::None);
  assert(schedule.next(40, false, true) == ReliablePublication::Status);
  schedule.record(40, ReliablePublication::Status, true);
  assert(schedule.next(60, false, true) == ReliablePublication::Info);
  schedule.record(60, ReliablePublication::Info, false);
  assert(schedule.next(69, true, true) == ReliablePublication::None);
  assert(schedule.next(70, true, true) == ReliablePublication::Status);

  ReliableSchedule failed{};
  assert(failed.next(20, true, false) == ReliablePublication::Status);
  failed.record(20, ReliablePublication::Status, false);
  assert(failed.next(30, true, false) == ReliablePublication::None);
  assert(failed.next(40, true, false) == ReliablePublication::Status);
  assert(failed.status_success == 0);
  failed.record(40, ReliablePublication::Status, true);
  assert(failed.next(50, false, true) == ReliablePublication::Info);
  failed.record(50, ReliablePublication::Info, true);
  assert(failed.next(59, false, true) == ReliablePublication::None);
  assert(failed.next(60, false, false) == ReliablePublication::Status);

  ReliableSchedule wrapping{UINT32_MAX - 5, UINT32_MAX - 5, UINT32_MAX - 5, 0};
  assert(wrapping.next(3, true, true) == ReliablePublication::None);
  assert(wrapping.next(4, true, true) == ReliablePublication::Info);
  assert(wrapping.next(14, false, false) == ReliablePublication::Status);
}
