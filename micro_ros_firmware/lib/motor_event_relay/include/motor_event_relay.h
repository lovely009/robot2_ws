#ifndef MOTOR_EVENT_RELAY_H
#define MOTOR_EVENT_RELAY_H

// 内核事件中继环：motor_rt_task（生产侧，1ms 服务节拍）把 motor_control
// 内核事件搬入本环，ROS 桥任务（消费侧）轮询取走。元素复用
// motor_control_event_t，容量编译期固定且无动态内存（ARD §29）。
//
// 并发契约（与 control_mailbox 同款）：库内无锁、无原子字段、无 volatile——
// push 与 drain 的每次调用都必须位于调用方提供的临界区内，且生产/消费两侧
// 共用同一互斥域（单核下 FreeRTOS taskENTER_CRITICAL 或等效关中断即满足）。
// 「单侧持锁」不构成互斥。仅支持任务上下文，ISR 内临界区非法，本层不做检测。
//
// 满策略：覆盖 head 槽后前移，丢最旧保最新并累计 dropped_count，保持 FIFO。

#include <stddef.h>
#include <stdint.h>

#include "motor_control.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_EVENT_RELAY_CAPACITY 64U

typedef struct {
  motor_control_event_t slots[MOTOR_EVENT_RELAY_CAPACITY];
  size_t head; // 最旧元素下标（取模环）
  size_t count;
  uint32_t dropped_count;
} motor_event_relay_t;

// 复位为空环（head/count 清零；dropped_count 同步清零，仅初始化期调用）。
void motor_event_relay_init(motor_event_relay_t *relay);

// 入队一条事件；满则丢最旧（覆盖 head 槽后前移）并累计 dropped_count。
void motor_event_relay_push(motor_event_relay_t *relay,
                            const motor_control_event_t *event);

// FIFO 取走至多 capacity 条，返回实际条数（0 = 空）。
size_t motor_event_relay_drain(motor_event_relay_t *relay,
                               motor_control_event_t *out, size_t capacity);

// 生命周期累计的丢弃计数（诊断可见：丢事件必须留痕）。
uint32_t motor_event_relay_dropped_count(const motor_event_relay_t *relay);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_EVENT_RELAY_H
