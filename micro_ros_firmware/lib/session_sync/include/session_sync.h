// 会话/授权同步纯逻辑（2ms/500Hz 控制边界调用一次）：把 mailbox 边沿与
// 会话电平翻译成内核状态变更，无 FreeRTOS 依赖，host 可直测。
// 临界区由调用方负责（take_events 与 session 电平读取须在同一临界区内）。
#ifndef SESSION_SYNC_H
#define SESSION_SYNC_H

#include <stdbool.h>
#include <stdint.h>

#include "control_mailbox.h"
#include "motor_control.h"

#ifdef __cplusplus
extern "C" {
#endif

// events/level 由调用方在临界区内取得（见 control_boundary）。
//  - agent_state_pending：转发连接/断连（断连触发内核 P0 撤权路径）；
//  - 每边界按会话电平重立/撤销全局授权（未知=引导豁免，true）。
// 顺序固定：先转发连接边沿、再按电平设授权——断连边界因此产生两次
// AUTHORIZATION_LOST（set_agent + set_authorization 各一次），勿合并。
void session_sync_apply(motor_control_context_t *ctx,
    const control_mailbox_events_t *events,
    bool session_authorized, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif
