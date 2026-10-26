#include <cassert>
#include <cstdint>
#include <cstdio>
#include "serial_write_budget.h"

struct Port {
  uint32_t now = 0;
  int free = 0;
  size_t sent = 0;
  serial_write_ports_t ports() {
    return {this, [](void *p) { return static_cast<Port *>(p)->now; },
      [](void *p) { return static_cast<Port *>(p)->free; },
      [](void *p, const uint8_t *, size_t n) {
        auto &s = *static_cast<Port *>(p);
        assert(n <= static_cast<size_t>(s.free));
        s.free -= static_cast<int>(n); s.sent += n; return n;
      }, [](void *p) { ++static_cast<Port *>(p)->now; }};
  }
};
int main() {
  uint8_t data[16] = {};
  Port p; auto ports = p.ports();
  assert(serial_write_until(&ports, data, 16, 2) == 0);
  assert(p.now == 2);
  p = {UINT32_MAX - 1, 4, 0}; ports = p.ports();
  assert(serial_write_until(&ports, data, 16, 1) == 4);
  assert(p.now == 1 && p.sent == 4);
  p = {100, 16, 0}; ports = p.ports();
  assert(serial_write_until(&ports, data, 8, 102) == 8);
  p.now = 102;
  assert(serial_write_until(&ports, data, 8, 102) == 0);
  p.now = 101;
  assert(serial_write_until(&ports, data, 8, 102) == 8);
  std::puts("test_serial_write_budget: ok");
}
