// 逐电机策略的最新值 mailbox（enable + MIT 增益）：control_command_mailbox 是「运动目标」，
// 本条是「控制律参数与武装位」，二者独立、互不覆盖。**不走 config FIFO**：config 队列深 16、
// 面向低频逐台带设备事务的请求，而一条策略最多 FIRMWARE_MAX_MOTORS 台，按台提交会有一半以上被拒、策略变成
// 部分生效。并发契约与 control_command_mailbox 逐字相同（无锁、调用方临界区、写读同域、
// 仅任务上下文）；语义为双缓冲 latest-wins，空策略是合法载荷、等价于无操作。

#ifndef CONTROL_POLICY_MAILBOX_H
#define CONTROL_POLICY_MAILBOX_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CONTROL_POLICY_MAILBOX_BUFFER_COUNT 2U
#include "firmware_config.h"
#define CONTROL_POLICY_MAILBOX_MAX_TARGETS FIRMWARE_MAX_MOTORS

// 策略元素。三项一起送：内核一次落地，不存在「已使能但 Kd 还是旧的」中间态。
typedef struct {
  uint16_t motor_id;
  bool enabled;
  float kp;
  float kd;
  // 保护确知为关时的显式武装确认（每次都要），见 motor_control_fault_code_t
  // 的 UNPROTECTED_UNCONFIRMED。
  bool confirm_unprotected;
} control_policy_target_t;

typedef struct {
  uint64_t lease_id;
  uint32_t received_tick_ms;
  uint32_t valid_until_tick_ms;
  // 消息级请求号：逐台结果经 /micro_ros_up/events 的 CONFIG_RESULT 回，用它关联。
  uint32_t request_id;
  // 整条消息共用的代数（见 MotorPolicyArray.msg 的防复权说明）。
  uint32_t policy_generation;
  uint8_t target_count;
  control_policy_target_t targets[CONTROL_POLICY_MAILBOX_MAX_TARGETS];
} control_policy_snapshot_t;

typedef struct {
  control_policy_snapshot_t buffers[CONTROL_POLICY_MAILBOX_BUFFER_COUNT];
  uint8_t published_buffer;
  bool pending;
  uint32_t overwrite_count;
  uint32_t invalid_policy_count;
} control_policy_mailbox_t;

void control_policy_mailbox_init(control_policy_mailbox_t *mailbox);
bool control_policy_mailbox_publish(control_policy_mailbox_t *mailbox,
    const control_policy_snapshot_t *snapshot);
bool control_policy_mailbox_take(control_policy_mailbox_t *mailbox,
    control_policy_snapshot_t *snapshot);
void control_policy_mailbox_clear(control_policy_mailbox_t *mailbox);

#ifdef __cplusplus
}
#endif

#endif  // CONTROL_POLICY_MAILBOX_H
