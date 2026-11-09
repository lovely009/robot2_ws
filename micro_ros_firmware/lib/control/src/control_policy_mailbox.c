#include "control_policy_mailbox.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

// publish 的缓冲翻转写入 published 的补集，按双缓冲硬编码；
// 变更容量须同步修改翻转逻辑，勿单独调整宏。
_Static_assert(CONTROL_POLICY_MAILBOX_BUFFER_COUNT == 2U,
    "publish buffer flip assumes exactly 2 buffers");

// 整条策略的合法性：非法即整条拒绝，绝不部分采纳（与命令快照同口径）。
// 空策略是合法载荷（无操作），不算非法。
static bool policy_is_valid(const control_policy_snapshot_t *snapshot) {
  if (snapshot == NULL ||
      snapshot->target_count > CONTROL_POLICY_MAILBOX_MAX_TARGETS) {
    return false;
  }
  for (size_t index = 0U; index < snapshot->target_count; ++index) {
    const control_policy_target_t *target = &snapshot->targets[index];
    if (!isfinite(target->kp) || !isfinite(target->kd) || target->kp < 0.0F ||
        target->kd < 0.0F) {
      return false;
    }
    for (size_t previous = 0U; previous < index; ++previous) {
      if (snapshot->targets[previous].motor_id == target->motor_id) {
        return false; // 同一台出现两次：哪一条算数没有定义
      }
    }
  }
  return true;
}

void control_policy_mailbox_init(control_policy_mailbox_t *mailbox) {
  if (mailbox == NULL) {
    return;
  }
  memset(mailbox, 0, sizeof(*mailbox));
}

bool control_policy_mailbox_publish(control_policy_mailbox_t *mailbox,
    const control_policy_snapshot_t *snapshot) {
  if (mailbox == NULL || !policy_is_valid(snapshot)) {
    if (mailbox != NULL) {
      ++mailbox->invalid_policy_count;
    }
    return false;
  }

  const uint8_t next_buffer = mailbox->published_buffer == 0U ? 1U : 0U;
  if (mailbox->pending) {
    ++mailbox->overwrite_count;
  }
  mailbox->buffers[next_buffer] = *snapshot;
  mailbox->published_buffer = next_buffer;
  mailbox->pending = true;
  return true;
}

bool control_policy_mailbox_take(control_policy_mailbox_t *mailbox,
    control_policy_snapshot_t *snapshot) {
  if (mailbox == NULL || snapshot == NULL || !mailbox->pending) {
    return false;
  }
  *snapshot = mailbox->buffers[mailbox->published_buffer];
  mailbox->pending = false;
  return true;
}

void control_policy_mailbox_clear(control_policy_mailbox_t *mailbox) {
  if (mailbox != NULL) {
    mailbox->pending = false;
  }
}
