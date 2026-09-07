#include "control_command_mailbox.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

// publish 的缓冲翻转写入 published 的补集，按双缓冲硬编码；
// 变更容量须同步修改翻转逻辑，勿单独调整宏。
_Static_assert(CONTROL_COMMAND_MAILBOX_BUFFER_COUNT == 2U,
               "publish buffer flip assumes exactly 2 buffers");

static bool snapshot_is_valid(const control_command_snapshot_t *snapshot) {
  if (snapshot == NULL ||
      snapshot->target_count > CONTROL_COMMAND_MAILBOX_MAX_TARGETS) {
    return false;
  }

  for (size_t index = 0U; index < snapshot->target_count; ++index) {
    const control_command_target_t *target = &snapshot->targets[index];
    if (!isfinite(target->position) || !isfinite(target->velocity) ||
        !isfinite(target->effort)) {
      return false;
    }
    for (size_t previous = 0U; previous < index; ++previous) {
      if (snapshot->targets[previous].motor_id == target->motor_id) {
        return false;
      }
    }
  }
  return true;
}

void control_command_mailbox_init(control_command_mailbox_t *mailbox) {
  if (mailbox == NULL) {
    return;
  }
  memset(mailbox, 0, sizeof(*mailbox));
}

bool control_command_mailbox_publish(
    control_command_mailbox_t *mailbox,
    const control_command_snapshot_t *snapshot) {
  if (mailbox == NULL || !snapshot_is_valid(snapshot)) {
    if (mailbox != NULL) {
      ++mailbox->invalid_snapshot_count;
    }
    return false;
  }

  const uint8_t next_buffer =
      mailbox->published_buffer == 0U ? 1U : 0U;
  if (mailbox->pending) {
    ++mailbox->overwrite_count;
  }
  mailbox->buffers[next_buffer] = *snapshot;
  mailbox->published_buffer = next_buffer;
  mailbox->pending = true;
  return true;
}

bool control_command_mailbox_take(control_command_mailbox_t *mailbox,
                                  control_command_snapshot_t *snapshot) {
  if (mailbox == NULL || snapshot == NULL || !mailbox->pending) {
    return false;
  }
  *snapshot = mailbox->buffers[mailbox->published_buffer];
  mailbox->pending = false;
  return true;
}

void control_command_mailbox_clear(control_command_mailbox_t *mailbox) {
  if (mailbox != NULL) {
    mailbox->pending = false;
  }
}
