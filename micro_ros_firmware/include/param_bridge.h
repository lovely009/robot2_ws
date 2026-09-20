// ROS 2 参数服务器桥（rclc_parameter）：固件运行时参数的唯一 ROS 入口。
//
// 参数契约：
//   每电机 7 个（电机发现后动态注册，离线不注销；容量见 kMaxMotors）：
//     m<id>.enable(bool) m<id>.kp/.kd(double) m<id>.pos_min/.pos_max(rad)
//     m<id>.vel_limit(rad/s) m<id>.torque_limit(A，峰值电流工程单位)
//   系统参数（建服时注册）：
//     system.save(bool，false→true 脉冲：保存全部参数到 W25)
//     system.refresh(int，写 motor_id：触发 CERB 设备参数 SDO 回读)
//     system.last_error / .last_error_motor（只读错误码，见 .cpp 码表）
//     system.heap_free（只读，台架观测 newlib 堆余量）
//
// 语义（ARD §19/§20）：set 同步返回仅表示「已受理」；设备类写最终成败经
// 内核配置结果事件异步给出（本桥接收后转 motor_events/参数码），失败自动
// 回滚影子。回调运行在 micro-ROS
// 任务上下文，不得直接发 FDCAN/擦 Flash——一律经 mailbox 或 storage 通知。
//
// rclc 硬约束：仅 BOOL/INT/DOUBLE；回调内禁 add/delete/set（get 不禁）；
// 内部写回（同步/回滚/脉冲复位）必须经 g_internal_write 守卫。
// low_mem_mode=true：兼容 ros2 param get/set/list，不兼容批量 get/dump。

#ifndef PARAM_BRIDGE_H
#define PARAM_BRIDGE_H

#include <stdbool.h>
#include <stddef.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>

#include "motor_control.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PARAM_BRIDGE_EXECUTOR_HANDLES 5U

// 容量（2026-09-20 定）：**72 槽 = 10 system + 10 电机 × 6 项**。
// （原 64 槽 = 10 system + 10 × 5；加 `m<id>.direction` 后按 6 项/台重算，
// 取 72 以保住 10 台契约：kMaxMotors = (72 − 10) / 6 = 10。）
//
// 唯一稀缺资源是 DTCM 堆池：**122,880 B**（= DTCM 128 KiB − 主栈 8 KiB；.data/.bss
// 落 AXI/RAM_D1，DTCM 只放堆/主栈，见 ldscript:178-213）。FreeRTOS
// heap_useNewlib_ST 与 newlib 同池，**物理不可扩**。非参数常驻 19,660 B
// （FreeRTOS 动态任务栈 11,264 + rcl/transport 启动分配约 8,396）。
// rclc 在 low_mem 模式下于 init 时**按 max_params 一次性全量分配**（Parameter
// 112 B + descriptor 68 B + 每槽名缓冲 50 B + 两个 range 序列 + list 响应的
// String 头 → 结构约 269 B，加分配器块开销后**实测边际 328 B/槽**，由 176/208
// 与 224/256 两对台架点各自算出同一斜率）。**与板上实际几台电机无关**。
//
//   72 槽  → 72 × 328 = 23,616 B → heap_free ≈ 122,880 − 19,660 − 23,616 ≈ 79.6 KB [推导]
//   416 槽 → 136,448 B > 122,880 B 池 → **必然分配失败**
//
// 416 是 2026-09-19 一次未复核的改动，而 rclc 在该路径**不对分配结果做 NULL
// 检查**（parameter_server.c:543/555 会写 data[i].name）——所以它不是"优雅失败"，
// 而是写非法地址。故 param_bridge_create() 建服前先做堆预算拦截（见 .cpp）。
//
// 每电机项数由 src/param_bridge.cpp 的 kParamMotorFields[] 显式列出（当前 6 项：
// 限位×2 / 限速 / 限流 / 保护开关 / 方向），不再由 param_field_t 枚举边界隐式决定。
// 目标规模 48 台时电机参数**整体撤出参数树**（配置改走块话题，见
// docs/plans/active/48-motor-scale-design.md §4 P2）；届时本值收到 16（只留 system）。
#define PARAM_BRIDGE_MAX_PARAMS 72U

bool param_bridge_create(rcl_node_t *node, rclc_executor_t *executor);
void param_bridge_destroy(rcl_node_t *node);
bool param_bridge_created(void);

// 10ms 桥定时器调用（排在 motor_bridge_poll 之后）：动态注册、影子同步、
// 脉冲动作（save/refresh）、错误码回写。
void param_bridge_poll(void);

// motor_bridge publish_events 的逐条事件钩子（同任务顺序调用）：
// 消费 CONFIG_RESULT 做 outstanding 收敛/回滚。
void param_bridge_on_config_result(const motor_control_event_t *event);

// 最近一次 param_bridge_poll 的参数视图快照（只读，条数写入 *out_count）。
// 设备参数报告（motor_bridge）复用这一份，而不是自己再拷一份：视图数组
// 64 台约 6 KB，拷两份就是白占一份 RAM_D1。两者同在本任务里顺序调用，无并发。
const motor_control_param_view_t *param_bridge_views(size_t *out_count);

#ifdef __cplusplus
}
#endif

#endif // PARAM_BRIDGE_H
