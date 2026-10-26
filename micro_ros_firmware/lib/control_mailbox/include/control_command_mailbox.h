// 与电机协议和运行时策略无关的完整命令快照 mailbox。
// 并发契约：无内部锁、无原子字段——publish/take/clear 的每次调用都必须在调用方临界区内，写侧与
// 读侧共用同一互斥域（单核下关中断即满足；两侧各自加锁不构成互斥）。仅支持任务上下文调用，
// ISR 内关中断非法且本层不做检测。
// 语义：双缓冲 latest-wins——publish 写后台并翻转，未取走时再次 publish 只覆盖「后台」并计入
// overwrite_count；take 取走并清 pending；clear 只撤销未取走的输入，诊断计数保留为生命周期统计。

#ifndef CONTROL_COMMAND_MAILBOX_H
#define CONTROL_COMMAND_MAILBOX_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CONTROL_COMMAND_MAILBOX_BUFFER_COUNT 2U
#include "firmware_config.h"
#define CONTROL_COMMAND_MAILBOX_MAX_TARGETS FIRMWARE_MAX_MOTORS

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
