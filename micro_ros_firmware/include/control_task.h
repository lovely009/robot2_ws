// 控制业务任务接口（官方 CtrBoard-H7_FDCAN 例程移植）。
// 当前承载三路 FDCAN 初始化、100 Hz 电机控制周期与反馈快照；固件只内置
// 两个默认测试电机（RP + vaxor），完整拓扑后续通过参数服务下发。

#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "motor_control.h"

#ifdef __cplusplus
extern "C" {
#endif

// 创建控制任务（只能调用一次；重复调用返回 true 且不重复建任务）
bool control_task_start(void);

// 按 joint_name 覆盖对应静态 mailbox；只复制命令，不触碰 FDCAN/HAL。
// received_ms 由 ROS 接收时刻提供；找不到默认测试电机时返回 false。
bool control_task_submit_command(const motor_control_command_t *command,
                                 uint32_t received_ms);

// 覆盖单槽 debug CAN mailbox；control task 每周期最多消费一帧。
bool control_task_submit_debug_frame(const motor_control_tx_action_t *frame);

// 请求控制内核改变人工授权；false 立即使旧命令失效，true 仅清除已消失的
// 可恢复故障，不会重放旧命令。
void control_task_set_authorization(bool authorized);

// 转发 micro-ROS agent 连接状态；断连会使旧命令失效并要求重新授权。
void control_task_set_agent_connected(bool connected);

// 关节反馈（sensor_msgs/JointState 同语义）
typedef struct {
  char joint_name[32];
  double position;
  double velocity;
  double effort;
} control_joint_state_t;

// 按默认电机表索引复制最新反馈；尚未收到该关节反馈或索引越界时返回 false。
bool control_task_get_joint_state(size_t index, control_joint_state_t *out);

// 调试计数：回调收到 JointState 次数 / 匹配到默认电机次数（GDB 可读）
extern volatile uint32_t g_control_motor_cmd_count;
extern volatile uint32_t g_control_motor_found_count;
extern volatile uint32_t g_control_debug_submit_count;
extern volatile uint32_t g_control_debug_overwrite_count;
extern volatile uint32_t g_control_debug_rejected_count;
extern volatile uint32_t g_control_debug_sent_count;

#ifdef __cplusplus
}
#endif

#endif  // CONTROL_TASK_H
