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

// 128 槽 = 5 系统参数 + 17 电机 × 7。ponytail: rclc 参数集在 DTCM newlib
// 堆（low-mem 约 0.3KB/槽），远期三总线 ~40 台时需改自定义 srv 参数接口。
// 槽位预算：每电机 4 项（限位/限速/限流）+ system.* 10 项。
// **2026-09-18 台架实测修正**：原设 208（覆盖 48 台）在真机上把堆吃到只剩
// 1,776 B——每槽约 269 B，208 槽比 128 槽多吃约 21.5 KB，而 DTCM 堆池的实际
// 余量远小于静态估算。降到 176：覆盖 (176-10)/4 = 41 台（实际部署约 40），
// 释放约 8.6 KB。再往上调之前必须先在台架上量 heap_free 的稳态最低位。
#define PARAM_BRIDGE_MAX_PARAMS 176U

bool param_bridge_create(rcl_node_t *node, rclc_executor_t *executor);
void param_bridge_destroy(rcl_node_t *node);
bool param_bridge_created(void);

// 10ms 桥定时器调用（排在 motor_bridge_poll 之后）：动态注册、影子同步、
// 脉冲动作（save/refresh）、错误码回写。
void param_bridge_poll(void);

// motor_bridge publish_events 的逐条事件钩子（同任务顺序调用）：
// 消费 CONFIG_RESULT 做 outstanding 收敛/回滚。
void param_bridge_on_config_result(const motor_control_event_t *event);

#ifdef __cplusplus
}
#endif

#endif // PARAM_BRIDGE_H
