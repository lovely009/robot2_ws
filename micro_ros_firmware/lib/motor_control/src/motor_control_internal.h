// motor_control 内部共享接口（仅本库 src/ 内使用，不下沉到公共 API）。

#ifndef MOTOR_CONTROL_INTERNAL_H
#define MOTOR_CONTROL_INTERNAL_H

#include "motor_control.h"
#include "motor_control_profile.h"

#ifdef __cplusplus
extern "C" {
#endif

// 表项索引（tx_slot 的取值来源）：依赖"表项只增不减"不变量（见
// motor_control.h entries 注释）。P1 latest-wins 槽位与 P0 回告重入队
// 均按此索引定位表项。
static inline size_t motor_control_entry_index(
    const motor_control_context_t *ctx, const motor_control_entry_t *entry) {
  return (size_t)(entry - ctx->entries);
}

// P0 帧重试挂起类别（motor_control_entry_t::p0_kind）。
#define MOTOR_CONTROL_P0_NONE 0U
#define MOTOR_CONTROL_P0_SOFT_STOP 1U
#define MOTOR_CONTROL_P0_DISABLE 2U
#define MOTOR_CONTROL_P0_ENABLE 3U // 使能帧亦为 P0

// 追加事件（环满丢最旧并计数——保最新状态，ARD §18 精神；tick 为 MCU 单调时基）。
void motor_control_push_event(motor_control_context_t *ctx, uint32_t now_ms,
                              motor_control_event_type_t type, uint16_t motor_id,
                              uint8_t bus_index, uint16_t reason, uint32_t request_id);

// 追加待发送帧（满则丢弃并计数，不阻塞实时路径）。
bool motor_control_push_tx(motor_control_context_t *ctx,
                           const motor_protocol_tx_frame_t *frame);

// P0 安全帧推送：满时驱逐最旧的非 P0 帧腾位（对齐 ARD §22.1 抢占语义）；
// 仍失败则按 entry->last_p0_kind 挂 p0_pending，由 service 逐节拍重发
// （不静默丢安全帧）。调用方推送前必须已设置 last_p0_kind。
bool motor_control_push_tx_p0(motor_control_context_t *ctx,
                              motor_control_entry_t *entry,
                              const motor_protocol_tx_frame_t *frame);

// 统一 P0 安全帧生成 + 推送（kind 取 MOTOR_CONTROL_P0_DISABLE /
// MOTOR_CONTROL_P0_SOFT_STOP）。编码失败与队列满都不静默丢：挂起重试
// 并计数。撤权/离线/故障/冲突/软停五条路径共用，勿再各自复制编码逻辑。
bool motor_control_send_p0(motor_control_context_t *ctx,
                           motor_control_entry_t *entry, uint8_t kind);

// 全局键查找（线性扫描：40 台规模 + 64 容量下足够，确定性优于哈希）。
motor_control_entry_t *motor_control_find_entry(motor_control_context_t *ctx,
                                                uint16_t motor_id);

// 撤销运动授权的统一安全路径（断链 / 撤权 / 100ms 超时共用）：
// 对所有已使能电机发协议级停机/失能（P0），清目标与使能状态，
// 受控态降级 READY_DISABLED，置 require_new_snapshot（ARD §13.2/§13.3/§33）。
void motor_control_revoke_motion(motor_control_context_t *ctx, uint32_t now_ms);

// 单电机软停（P0 阻尼帧 + SAFE_STOP）。仅对 ACTIVE 态发帧（去重）。
void motor_control_soft_stop_entry(motor_control_context_t *ctx,
                                   motor_control_entry_t *entry, uint32_t now_ms);

// 绑定后首次设备影子填 Profile 默认（CERB；motor_control_params.c）。
void motor_control_set_default_device_shadow(motor_control_entry_t *entry);

// 配置事务：响应帧裁决（on_raw_frame 调用）与跨节拍步进（service 调用）。
bool motor_control_config_on_frame(motor_control_context_t *ctx,
                                   motor_control_entry_t *entry,
                                   const motor_protocol_raw_frame_t *frame);
void motor_control_config_step(motor_control_context_t *ctx, uint32_t now_ms);

// 身份读取发起（config_step 的不变量复检调用，每拍至多一台）：走事务机器
// 读设备自报型号串，读毕由内核按串精确匹配型号目录并绑定；协议无身份读
// 对象时直接用"协议内唯一型号"兜底。已在读（tx_active）或本轮已定论
// （identity_read_done）的表项直接返回。
void motor_control_start_identity_read(motor_control_context_t *ctx,
                                       motor_control_entry_t *entry,
                                       uint32_t now_ms);

// 配置事务统一收尾（CONFIG_RESULT 终态回告，清理 tx_active/tx_awaiting）。
// 只在事务自己走出裁决（响应、超时、阶段推进完毕）时调用。
void motor_control_finish_tx(motor_control_context_t *ctx,
                             motor_control_entry_t *entry, uint16_t result,
                             uint32_t now_ms);

// 事务中止（离线/冲突/厂家故障/撤权路径共用）：配置事务回 UNCONFIRMED 终态
// 避免上层请求悬挂；身份读取不判成败也不回事件，待表项回到可绑定态重发。
void motor_control_abort_tx(motor_control_context_t *ctx,
                            motor_control_entry_t *entry, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_CONTROL_INTERNAL_H
