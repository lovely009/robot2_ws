#ifndef INFO_PAGE_HPP
#define INFO_PAGE_HPP
#include <cstddef>
#include <cstdint>
#include "motor_control.h"
#include "config_transaction.hpp"

namespace info_page {
constexpr size_t kPageSize = 16;
enum Result : uint8_t { Accepted = 0, Retry = 1, Busy = 2, Retired = 3 };
struct Device {
  uint16_t motor_id = 0;
  uint8_t bus = 0, protocol = 0;
  char model[32]{};
  uint32_t capabilities = 0;
  bool online = false, conflict = false;
  uint8_t lifecycle = 0;
  motor_control_param_view_t params{};
};
struct Page {
  uint64_t session = 0, epoch_ns = 0;
  uint32_t query_id = 0, revision = 0, tick_ms = 0, detail_request_id = 0;
  uint16_t cursor = 0, next_cursor = 0, count = 0;
  uint8_t kind = 0;
  Result result = Accepted;
  bool complete = false;
  Device devices[kPageSize]{};
  config_transaction::Detail details[kPageSize]{};
};
// 只保留当前页；输入必须来自内存快照，不调用设备、存储或 ROS。
class Pager {
public:
  void set_source(uint64_t session, uint32_t revision, size_t total);
  Result request(uint64_t session, uint32_t query_id, uint32_t revision,
      uint16_t cursor, uint8_t kind = 0, uint32_t detail_request_id = 0);
  void fill(const Device *devices, size_t count, uint32_t tick_ms, uint64_t epoch_ns);
  void fill_details(const config_transaction::Detail *details, size_t count,
      size_t total, uint32_t tick_ms, uint64_t epoch_ns);
  const Page &page() const { return page_; }
  bool needs_fill() const { return needs_fill_; }
  bool pending() const { return pending_; }
  void sent(bool success) { if (success) pending_ = false; }
private:
  uint64_t session_ = 0;
  uint32_t revision_ = 0;
  size_t total_ = 0;
  Page page_{};
  bool cached_ = false, needs_fill_ = false, pending_ = false;
};
}
#endif
