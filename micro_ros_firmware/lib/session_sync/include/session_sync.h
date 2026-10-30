// 会话/授权同步纯逻辑：把 mailbox 边沿与调用方核验后的运动许可
// 翻译成内核状态变更，无 FreeRTOS 依赖，host 可直测。
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
//  - session_authorized 必须是「会话在线 && MCU 本地租约有效」；连接本身
//    不代表许可，启动/重连若无新租约必须传 false。
// 顺序固定：先转发连接边沿、再按有效许可设授权；撤权事件由内核去重。
void session_sync_apply(motor_control_context_t *ctx,
    const control_mailbox_events_t *events,
    bool session_authorized, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif
