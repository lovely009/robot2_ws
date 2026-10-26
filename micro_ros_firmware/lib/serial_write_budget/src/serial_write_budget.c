#include "serial_write_budget.h"
size_t serial_write_until(const serial_write_ports_t *ports,
    const uint8_t *data, size_t size, uint32_t deadline_ms) {
  size_t sent = 0U;
  while (sent < size) {
    const uint32_t remaining = deadline_ms - ports->now_ms(ports->context);
    if (remaining == 0U || remaining >= 0x80000000U) break;
    const int space = ports->available(ports->context);
    if (space <= 0) { ports->wait(ports->context); continue; }
    const size_t part = size - sent < (size_t)space ? size - sent : (size_t)space;
    const size_t n = ports->write(ports->context, data + sent, part);
    if (n == 0U || n > part) break;
    sent += n;
  }
  return sent;
}
