#include "info_page.hpp"
#include <algorithm>

namespace info_page {
void Pager::set_source(uint64_t session, uint32_t revision, size_t total) {
  if (session != session_) {
    page_ = Page{}; cached_ = needs_fill_ = pending_ = false;
  }
  session_ = session; revision_ = revision; total_ = total;
}
Result Pager::request(uint64_t session, uint32_t query, uint32_t revision,
    uint16_t cursor, uint8_t kind, uint32_t detail_id) {
  if (!session || session != session_) return Retired;
  const bool same = cached_ && page_.query_id == query && page_.cursor == cursor &&
      page_.kind == kind && page_.detail_request_id == detail_id;
  if (pending_ && !same) return Busy;
  if (!query || kind > 1) return Retired;
  if ((revision && revision != revision_) || (cursor && !revision)) {
    page_ = Page{}; page_.session = session; page_.query_id = query;
    page_.revision = revision_; page_.kind = kind; page_.result = Retry;
    pending_ = cached_ = true; needs_fill_ = false; return Retry;
  }
  if (same && page_.revision == revision_) { pending_ = true; return page_.result; }
  page_ = Page{}; page_.session = session; page_.query_id = query;
  page_.revision = revision_; page_.cursor = cursor; page_.kind = kind;
  page_.detail_request_id = detail_id;
  cached_ = pending_ = needs_fill_ = true;
  return Accepted;
}
void Pager::fill(const Device *devices, size_t count, uint32_t tick, uint64_t epoch) {
  if (!needs_fill_ || page_.kind != 0) return;
  page_.tick_ms = tick; page_.epoch_ns = epoch;
  const size_t total = std::min(count, total_);
  const size_t begin = std::min<size_t>(page_.cursor, total);
  page_.count = static_cast<uint16_t>(std::min(kPageSize, total - begin));
  if (!devices && page_.count) { page_.result = Retired; page_.count = 0; }
  for (size_t i = 0; i < page_.count; ++i) page_.devices[i] = devices[begin + i];
  page_.next_cursor = static_cast<uint16_t>(begin + page_.count);
  page_.complete = page_.next_cursor >= total;
  needs_fill_ = false;
}
void Pager::fill_details(const config_transaction::Detail *details, size_t count,
    size_t total, uint32_t tick, uint64_t epoch) {
  if (!needs_fill_ || page_.kind != 1) return;
  page_.tick_ms = tick; page_.epoch_ns = epoch;
  page_.count = static_cast<uint16_t>(std::min(count, kPageSize));
  if (!details || !total) { page_.count = 0; page_.result = Retired; }
  for (size_t i = 0; i < page_.count; ++i) page_.details[i] = details[i];
  page_.next_cursor = static_cast<uint16_t>(page_.cursor + page_.count);
  page_.complete = page_.next_cursor >= total;
  needs_fill_ = false;
}
}
