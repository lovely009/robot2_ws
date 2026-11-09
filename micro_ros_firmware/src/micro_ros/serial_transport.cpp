#include "serial_transport.h"
#include <micro_ros_platformio.h>
#include "serial_write_budget.h"

static bool g_budget_active = false;
static uint32_t g_write_deadline = 0U;
static constexpr uint32_t kWriteBudgetMs = 2U;
void serial_transport_send_begin(void) {
  g_write_deadline = millis() + kWriteBudgetMs;
  g_budget_active = true;
}
void serial_transport_send_end(void) { g_budget_active = false; }

static size_t bounded_write(uxrCustomTransport *transport,
    const uint8_t *data, size_t size, uint8_t *error) {
  // micro_ros 是唯一写者；IRQ 只释放队列，按可写空间切片避免进入 USBSerial 的等待循环。
  const serial_write_ports_t ports = {
    transport->args,
    [](void *) -> uint32_t { return millis(); },
    [](void *p) -> int { return static_cast<Stream *>(p)->availableForWrite(); },
    [](void *p, const uint8_t *bytes, size_t n) -> size_t {
      return static_cast<Stream *>(p)->write(bytes, n);
    },
    [](void *) { delay(1U); },
  };
  const uint32_t deadline = g_budget_active ? g_write_deadline : millis() + kWriteBudgetMs;
  const size_t sent = serial_write_until(&ports, data, size, deadline);
  *error = sent == size ? 0U : 1U;
  return sent;
}

void serial_transport_setup(Stream &serial) {
  rmw_uros_set_custom_transport(true, &serial,
      platformio_transport_open, platformio_transport_close,
      bounded_write, platformio_transport_read);
}
