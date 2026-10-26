#ifndef SERIAL_WRITE_BUDGET_H
#define SERIAL_WRITE_BUDGET_H
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
  void *context;
  uint32_t (*now_ms)(void *context);
  int (*available)(void *context);
  size_t (*write)(void *context, const uint8_t *data, size_t size);
  void (*wait)(void *context);
} serial_write_ports_t;
// write 端口只接收不大于 available 的片段；截止点可由多次 framing 回调共享。
size_t serial_write_until(const serial_write_ports_t *ports,
    const uint8_t *data, size_t size, uint32_t deadline_ms);
#ifdef __cplusplus
}
#endif
#endif
