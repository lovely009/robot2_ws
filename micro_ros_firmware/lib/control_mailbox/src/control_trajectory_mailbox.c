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

// 整条轨迹的合法性（ARD §15.5 的时序扩展）：非法即整条拒绝，绝不部分采纳。
//
// 约束：
//  - 计数域内有界；
//  - target_ids 内无重复，且每台至少有一个路点（声明了归属却没有时间线
//    的电机语义不明，与其让内核实现在运行期兜底，不如在入口拒绝）；
//  - 路点按 time_from_start_ms 非递减排列，同一电机内严格递增
//    （不同电机允许共用同一时刻——并行的关节本就同拍采样，强行要求全局
//    递增会逼上层去重时间轴）；
//  - 所有浮点为有限值（命令侧禁 NaN/Inf，TBD-ROS-01=A）。
//
// 复杂度 O(point_count × target_count)，由 mailbox 在临界区内调用。轨迹发布
// 是低频动作（段级，非 100Hz），实测量级见 TBD-RT-02；若台架显示关中断
// 窗口过长，再考虑把校验前移到 ROS 回调（非临界区）。
static bool trajectory_is_valid(const control_trajectory_t *trajectory) {
  if (trajectory == NULL ||
      trajectory->point_count > CONTROL_TRAJECTORY_MAILBOX_MAX_POINTS ||
      trajectory->target_count > CONTROL_TRAJECTORY_MAILBOX_MAX_MOTORS) {
    return false;
  }
  // 空轨迹 = 取消，合法。
  if (trajectory->point_count == 0U && trajectory->target_count == 0U) {
    return true;
  }
  // 有路点必须有归属电机，反之亦然（半条轨迹没有可执行语义）。
  if (trajectory->point_count == 0U || trajectory->target_count == 0U) {
    return false;
  }

  for (uint16_t i = 0U; i < trajectory->target_count; ++i) {
    for (uint16_t j = (uint16_t)(i + 1U); j < trajectory->target_count; ++j) {
      if (trajectory->target_ids[i] == trajectory->target_ids[j]) {
        return false;
      }
    }
  }

  uint32_t point_mask = 0U; // 每台「已有路点」位图（target_count ≤ 32 时够用）
  // target_count == 32 时 1U<<32 是未定义行为，故位图只覆盖 ≤31 台。
  const bool need_mask = trajectory->target_count < 32U;
  uint32_t previous_time = 0U;
  for (uint16_t i = 0U; i < trajectory->point_count; ++i) {
    const control_trajectory_point_t *point = &trajectory->points[i];
    const int slot = trajectory_motor_slot(trajectory, point->motor_id);
    if (slot < 0) {
      return false; // motor_id 不在声明集合内
    }
    if (!isfinite(point->position) || !isfinite(point->velocity) ||
        !isfinite(point->effort)) {
      return false;
    }
    if (i > 0U) {
      if (point->time_from_start_ms < previous_time) {
        return false; // 时间轴必须非递减
      }
      // 时间非递减下，同电机的同一时刻必然相邻——只比前一点即可。
      const control_trajectory_point_t *previous = &trajectory->points[i - 1U];
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

  if (need_mask && point_mask != ((1U << trajectory->target_count) - 1U)) {
    return false; // 有声明了归属却没有任何路点的电机
  }
  if (!need_mask) {
    // 归属电机数 > 32：退回逐台复查（只在超大轨迹时才走这条路径）。
    for (uint16_t i = 0U; i < trajectory->target_count; ++i) {
      bool found = false;
      for (uint16_t j = 0U; j < trajectory->point_count && !found; ++j) {
        found = trajectory->points[j].motor_id == trajectory->target_ids[i];
      }
      if (!found) {
        return false;
      }
    }
  }
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
  if (mailbox == NULL || !trajectory_is_valid(trajectory)) {
    if (mailbox != NULL) {
      ++mailbox->invalid_trajectory_count;
    }
    return false;
  }

  const uint8_t next_buffer = mailbox->published_buffer == 0U ? 1U : 0U;
  if (mailbox->pending) {
    ++mailbox->overwrite_count;
  }
  mailbox->buffers[next_buffer] = *trajectory;
  mailbox->published_buffer = next_buffer;
  mailbox->pending = true;
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
  }
}
