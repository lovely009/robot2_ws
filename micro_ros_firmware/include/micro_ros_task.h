// micro-ROS 连接管理任务：WAIT_AGENT/CONNECT/RUN/LOST 四态机（状态语义见
// micro_ros_link_state.h）。RUN 中周期调 ros_bridge_spin；断连整体销毁
// ROS 实体并回到 WAIT_AGENT 重连。由 main 创建，LED 任务读状态选颜色。
#ifndef MICRO_ROS_TASK_H
#define MICRO_ROS_TASK_H

#include <stdbool.h>

#include "micro_ros_link_state.h"

#ifdef __cplusplus
extern "C" {
#endif

bool micro_ros_task_start(void);

// 仅通信任务内、通过会话/序号/载荷校验并受理的入站帧可调用。
void micro_ros_note_agent_rx(void);

// 当前连接状态（LED/其他只读消费者用）。
micro_ros_link_state_t micro_ros_task_get_link_state(void);

// 是否曾完成过至少一次连接（区分"引导期未知"与"断线"）。
bool micro_ros_task_ever_connected(void);

#ifdef __cplusplus
}
#endif

#endif
