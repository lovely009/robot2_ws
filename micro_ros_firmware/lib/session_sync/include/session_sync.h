// 连接边沿与显式运动授权同步；无 FreeRTOS 依赖。
// 调用方在与 mailbox 共用的临界区内取得 events。
#ifndef SESSION_SYNC_H
#define SESSION_SYNC_H

#include <stdbool.h>
#include <stdint.h>
#include "control_mailbox.h"
#include "motor_control.h"

#ifdef __cplusplus
extern "C" {
#endif

// 连接边沿可撤权，在线电平不可创建授权；复权只能由新策略执行入口触发。
void session_sync_apply(motor_control_context_t *ctx,
    const control_mailbox_events_t *events,
    bool motion_authorized, uint32_t now_ms);

#ifdef __cplusplus
}
#endif
#endif
