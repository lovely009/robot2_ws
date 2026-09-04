// 与电机协议和运行时策略无关的完整命令快照 mailbox。
// 调用方负责在并发访问时提供短 critical section。

#ifndef CONTROL_COMMAND_MAILBOX_H
#define CONTROL_COMMAND_MAILBOX_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CONTROL_COMMAND_MAILBOX_BUFFER_COUNT 2U
#define CONTROL_COMMAND_MAILBOX_MAX_TARGETS 64U

typedef struct {
  uint16_t motor_id;
  float position;
  float velocity;
  float effort;
} control_command_target_t;

typedef struct {
  uint32_t sequence;
  uint32_t received_tick_ms;
  uint8_t target_count;
  control_command_target_t targets[CONTROL_COMMAND_MAILBOX_MAX_TARGETS];
} control_command_snapshot_t;

typedef struct {
  control_command_snapshot_t buffers[CONTROL_COMMAND_MAILBOX_BUFFER_COUNT];
  uint8_t published_buffer;
  bool pending;
  uint32_t overwrite_count;
  uint32_t invalid_snapshot_count;
} control_command_mailbox_t;

void control_command_mailbox_init(control_command_mailbox_t *mailbox);
bool control_command_mailbox_publish(
    control_command_mailbox_t *mailbox,
    const control_command_snapshot_t *snapshot);
bool control_command_mailbox_take(control_command_mailbox_t *mailbox,
                                  control_command_snapshot_t *snapshot);
void control_command_mailbox_clear(control_command_mailbox_t *mailbox);

#ifdef __cplusplus
}
#endif

#endif  // CONTROL_COMMAND_MAILBOX_H
