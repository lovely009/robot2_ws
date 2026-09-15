// 编译期拓扑表（下行配置话题已删后的型号绑定来源）。
//
// 真机台架基线（2026-09-15）：FDCAN1 总线 1×R58 + 3×R48。换机器人/换拓扑
// 只改本表；表外 id 不绑定（保持未绑定不可控，安全侧）。增益/限位不在此
// 配置——继承型号目录（lib/motor_control/src/motor_control_profile.c）的
// default_kp/default_kd 与 MIT 限位。
#ifndef MOTOR_STATIC_TOPOLOGY_H
#define MOTOR_STATIC_TOPOLOGY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 返回 motor_id 对应的型号名（型号目录锚点）；表外返回 NULL。
const char *motor_static_topology_model_for(uint16_t motor_id);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_STATIC_TOPOLOGY_H
