#include "control_trajectory_mailbox.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

// publish 的缓冲翻转写入 published 的补集，按双缓冲硬编码；
// 变更容量须同步修改翻转逻辑，勿单独调整宏。
_Static_assert(CONTROL_TRAJECTORY_MAILBOX_BUFFER_COUNT == 2U,
    "publish buffer flip assumes exactly 2 buffers");

// 路点在 target_ids 中的下标；不在集合内返回 -1。
static int trajectory_motor_slot(const control_trajectory_t *trajectory,
    uint16_t motor_id) {
  for (uint16_t i = 0U; i < trajectory->target_count; ++i) {
    if (trajectory->target_ids[i] == motor_id) {
      return (int)i;
    }
  }
  return -1;
}

// 路点的共同合法性。分段校验允许一个分段只覆盖部分电机；完整轨迹校验
// 再要求 target_ids 中每台电机都至少出现一个路点。
//
// 这里使用已接收前缀的 O(n²) 重复检查，而不是只比较相邻点。不同电机可以
// 在同一时刻交错排列，因而同一电机的重复时间点不一定相邻；轨迹分段是低频
// 输入，明确拒绝这种数据比把零时间间隔带入 Hermite 分母更安全。
static bool trajectory_points_are_valid(const control_trajectory_t *trajectory,
    bool require_target_coverage) {
  if (trajectory == NULL ||
      trajectory->point_count > CONTROL_TRAJECTORY_MAILBOX_MAX_POINTS ||
      trajectory->target_count > CONTROL_TRAJECTORY_MAILBOX_MAX_MOTORS ||
      trajectory->point_count == 0U || trajectory->target_count == 0U) {
    return false;
  }

  for (uint16_t i = 0U; i < trajectory->target_count; ++i) {
    for (uint16_t j = (uint16_t)(i + 1U); j < trajectory->target_count; ++j) {
      if (trajectory->target_ids[i] == trajectory->target_ids[j]) {
        return false;
      }
    }
  }

  uint32_t point_mask = 0U;
  const bool need_mask = trajectory->target_count < 32U;
  uint32_t previous_time = 0U;
  for (uint16_t i = 0U; i < trajectory->point_count; ++i) {
    const control_trajectory_point_t *point = &trajectory->points[i];
    const int slot = trajectory_motor_slot(trajectory, point->motor_id);
    if (slot < 0 || !isfinite(point->position) ||
        !isfinite(point->velocity) || !isfinite(point->effort)) {
      return false;
    }
    if (i > 0U && point->time_from_start_ms < previous_time) {
      return false;
    }
    for (uint16_t j = 0U; j < i; ++j) {
      const control_trajectory_point_t *previous = &trajectory->points[j];
      if (previous->motor_id == point->motor_id &&
          previous->time_from_start_ms == point->time_from_start_ms) {
        return false; // 同一电机同一时刻重复路点
      }
    }
    previous_time = point->time_from_start_ms;
    if (need_mask) {
      point_mask |= (uint32_t)1U << (uint32_t)slot;
    }
  }

  if (!require_target_coverage) {
    return true;
  }
  if (need_mask) {
    return point_mask == ((1U << trajectory->target_count) - 1U);
  }
  for (uint16_t i = 0U; i < trajectory->target_count; ++i) {
    bool found = false;
    for (uint16_t j = 0U; j < trajectory->point_count && !found; ++j) {
      found = trajectory->points[j].motor_id == trajectory->target_ids[i];
    }
    if (!found) {
      return false;
    }
  }
  return true;
}

// 完整轨迹（单段输入，或分段拼接完成的整条时间线）。分段字段由调用方归一，
// 这里只判载荷。
static bool trajectory_is_valid(const control_trajectory_t *trajectory) {
  if (trajectory == NULL) {
    return false;
  }
  // 空轨迹 = 取消，合法。
  if (trajectory->point_count == 0U && trajectory->target_count == 0U) {
    return true;
  }
  return trajectory_points_are_valid(trajectory, true);
}

// 多段轨迹里的单个分段：允许只覆盖部分电机，完整性等最后一段拼完再判。空分段
// 不是取消（取消只能是单段空轨迹，见 publish 的分派），由载荷校验拒绝。
static bool trajectory_segment_is_valid(const control_trajectory_t *trajectory) {
  if (trajectory == NULL || trajectory->segment_count < 2U ||
      trajectory->segment_count > CONTROL_TRAJECTORY_MAILBOX_MAX_SEGMENTS ||
      trajectory->segment_index >= trajectory->segment_count) {
    return false;
  }
  return trajectory_points_are_valid(trajectory, false);
}

static bool trajectory_target_sets_equal(const control_trajectory_t *left,
    const control_trajectory_t *right) {
  if (left->target_count != right->target_count) {
    return false;
  }
  for (uint16_t i = 0U; i < left->target_count; ++i) {
    bool found = false;
    for (uint16_t j = 0U; j < right->target_count; ++j) {
      if (left->target_ids[i] == right->target_ids[j]) {
        found = true;
        break;
      }
    }
    if (!found) {
      return false;
    }
  }
  return true;
}

static bool trajectory_point_equal(const control_trajectory_point_t *left,
    const control_trajectory_point_t *right) {
  return left->time_from_start_ms == right->time_from_start_ms &&
         left->motor_id == right->motor_id &&
         left->position == right->position &&
         left->velocity == right->velocity && left->effort == right->effort;
}

static void clear_assembly(control_trajectory_mailbox_t *mailbox) {
  mailbox->assembly_active = false;
  mailbox->assembly_next_segment = 0U;
  mailbox->assembly.point_count = 0U;
  mailbox->assembly.target_count = 0U;
}

static void publish_complete(control_trajectory_mailbox_t *mailbox,
    const control_trajectory_t *trajectory) {
  // 对外发布的轨迹统一归一为单段：内核只认完整轨迹，分段元数据到此为止。
  const uint8_t next_buffer = mailbox->published_buffer == 0U ? 1U : 0U;
  if (mailbox->pending) {
    ++mailbox->overwrite_count;
  }
  mailbox->buffers[next_buffer] = *trajectory;
  mailbox->buffers[next_buffer].segment_index = 0U;
  mailbox->buffers[next_buffer].segment_count = 1U;
  mailbox->published_buffer = next_buffer;
  mailbox->pending = true;
}

static bool append_segment(control_trajectory_mailbox_t *mailbox,
    const control_trajectory_t *segment) {
  control_trajectory_t *assembly = &mailbox->assembly;
  if (assembly->lease_id != segment->lease_id ||
      assembly->trajectory_id != segment->trajectory_id ||
      assembly->segment_count != segment->segment_count ||
      !trajectory_target_sets_equal(assembly, segment)) {
    return false;
  }
  // 容量按**去重合并后**的拼接总量判（在下方真正写入时检查），不能按原始
  // 分段载荷之和判：上层允许边界行重叠，原始合计可超过 MAX_POINTS 而合并后
  // 恰好在容量内，提前按载荷判会把这类合法整条轨迹误拒。
  const uint32_t last_time =
      assembly->point_count == 0U
          ? 0U
          : assembly->points[assembly->point_count - 1U].time_from_start_ms;
  uint32_t previous_segment_time = 0U;
  for (uint16_t i = 0U; i < segment->point_count; ++i) {
    const control_trajectory_point_t *point = &segment->points[i];
    if (i > 0U && point->time_from_start_ms < previous_segment_time) {
      return false;
    }
    previous_segment_time = point->time_from_start_ms;
    if (point->time_from_start_ms < last_time) {
      return false; // 只允许在边界时刻重叠，不允许时间线回退
    }

    if (point->time_from_start_ms == last_time) {
      bool duplicate = false;
      for (uint16_t j = 0U; j < assembly->point_count; ++j) {
        const control_trajectory_point_t *existing = &assembly->points[j];
        if (existing->motor_id != point->motor_id ||
            existing->time_from_start_ms != point->time_from_start_ms) {
          continue;
        }
        if (!trajectory_point_equal(existing, point)) {
          return false; // 同一边界点的位姿/前馈冲突
        }
        duplicate = true;
        break;
      }
      if (duplicate) {
        continue;
      }
    }
    if (assembly->point_count >= CONTROL_TRAJECTORY_MAILBOX_MAX_POINTS) {
      return false; // 去重后仍超 256：整组非法，由调用方 clear_assembly
    }
    assembly->points[assembly->point_count++] = *point;
  }
  // 完成时间线后才有执行意义；诊断序号和接收 tick 取最后一段。没有共享边界点
  // 时也直接按绝对时间拼接，内核稍后在整条时间线上计算两侧内部切矢，不把分段
  // 端点误当成整条轨迹的零速端点。
  assembly->sequence = segment->sequence;
  assembly->received_tick_ms = segment->received_tick_ms;
  return true;
}

void control_trajectory_mailbox_init(control_trajectory_mailbox_t *mailbox) {
  if (mailbox == NULL) {
    return;
  }
  memset(mailbox, 0, sizeof(*mailbox));
}

bool control_trajectory_mailbox_publish(
    control_trajectory_mailbox_t *mailbox,
    const control_trajectory_t *trajectory) {
  if (mailbox == NULL) {
    return false;
  }
  // 单段输入就是完整轨迹（空单段 = 取消）：一次完整校验后直接发布，不走组装。
  if (trajectory != NULL && trajectory->segment_count == 1U &&
      trajectory->segment_index == 0U) {
    if (!trajectory_is_valid(trajectory)) {
      ++mailbox->invalid_trajectory_count;
      return false;
    }
    clear_assembly(mailbox);
    publish_complete(mailbox, trajectory);
    return true;
  }
  if (!trajectory_segment_is_valid(trajectory)) {
    ++mailbox->invalid_trajectory_count;
    return false;
  }

  if (trajectory->segment_index == 0U) {
    // 新的第 0 段开始一组新的组装；旧的完整轨迹仍在 published buffer
    // 或内核中运行，不会因半条新轨迹而改变。
    mailbox->assembly = *trajectory;
    mailbox->assembly_active = true;
    mailbox->assembly_next_segment = 1U;
    return true;
  }

  if (!mailbox->assembly_active ||
      trajectory->segment_index != mailbox->assembly_next_segment ||
      !append_segment(mailbox, trajectory)) {
    clear_assembly(mailbox);
    ++mailbox->invalid_trajectory_count;
    return false;
  }

  ++mailbox->assembly_next_segment;
  if (trajectory->segment_index + 1U != trajectory->segment_count) {
    return true;
  }

  // 最后一段：时间线已完整，按整条轨迹复验（分段校验不查每台电机的路点覆盖）。
  // publish_complete 会把分段元数据归一回去，这里不再单独拷一份完整轨迹。
  if (!trajectory_is_valid(&mailbox->assembly)) {
    clear_assembly(mailbox);
    ++mailbox->invalid_trajectory_count;
    return false;
  }
  publish_complete(mailbox, &mailbox->assembly);
  clear_assembly(mailbox);
  return true;
}

bool control_trajectory_mailbox_take(control_trajectory_mailbox_t *mailbox,
    control_trajectory_t *trajectory) {
  if (mailbox == NULL || trajectory == NULL || !mailbox->pending) {
    return false;
  }
  *trajectory = mailbox->buffers[mailbox->published_buffer];
  mailbox->pending = false;
  return true;
}

void control_trajectory_mailbox_clear(control_trajectory_mailbox_t *mailbox) {
  if (mailbox != NULL) {
    mailbox->pending = false;
    clear_assembly(mailbox);
  }
}
