// QoS 判据（TBD-ROS-05 默认表）的唯一出处：motor_bridge 与 param_bridge 共用。
//
// 选哪条看**消息性质**，不看它属于哪个业务域：
//   - 高频流、丢一帧无所谓（latest-wins 的电平/测量）→ qos_stream()
//   - 低频状态声明、不允许静默丢（命令、配置、或 >512 B 需分片）→ qos_reliable(n)
//
// 此前这两条只存在于 motor_bridge.cpp 里；param_bridge 改造时一并提出来，
// 避免两份判据各自漂移（同一个话题在两边用不同 QoS 会静默收不到）。
#ifndef ROS_QOS_H
#define ROS_QOS_H

#include <stddef.h>

#include <rmw/types.h>

// 高频流：best-effort + depth 1（latest-wins，无历史积压，ARD §34）。
static inline rmw_qos_profile_t qos_stream(void) {
  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  qos.depth = 1;
  return qos;
}

// 低频状态声明：reliable，历史上限 depth。
static inline rmw_qos_profile_t qos_reliable(size_t depth) {
  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  qos.depth = depth;
  return qos;
}

#endif // ROS_QOS_H
