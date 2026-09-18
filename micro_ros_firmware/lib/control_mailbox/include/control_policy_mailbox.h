// 逐电机策略的最新值 mailbox（enable + MIT 增益）。
//
// 与 control_command_mailbox 的关系：那条是「运动目标」，本条是「控制律参数
// 与武装位」。二者独立，互不覆盖。
//
// **为什么不走 config FIFO**：config 队列深 16，是给「低频、逐台、带设备事务」
// 的请求用的；一条策略消息最多 64 台，按台提交会有一半以上被 FIFO 拒掉，
// 策略变成部分生效。策略是「批量、本地立即生效」的类别，理应有自己的有界
// 通道（与命令/轨迹同构）。
//
// 并发契约：与 control_command_mailbox 逐字相同——本层无内部锁、无原子字段，
// publish/take/clear 的每一次调用都必须位于调用方提供的临界区内，且写侧与
// 读侧必须共用同一互斥域。仅支持任务上下文调用。
//
// 语义：双缓冲 latest-wins。空策略（target_count==0）是合法载荷（等价于
// 无操作），不是非法。

#ifndef CONTROL_POLICY_MAILBOX_H
#define CONTROL_POLICY_MAILBOX_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CONTROL_POLICY_MAILBOX_BUFFER_COUNT 2U
#define CONTROL_POLICY_MAILBOX_MAX_TARGETS 64U

// 策略元素。三项一起送：内核一次落地，不存在「已使能但 Kd 还是旧的」中间态。
typedef struct {
  uint16_t motor_id;
  bool enabled;
  float kp;
  float kd;
} control_policy_target_t;

typedef struct {
  uint32_t received_tick_ms;
  // 消息级请求号：逐台结果经 /micro_ros/events 的 CONFIG_RESULT 回，用它关联。
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
