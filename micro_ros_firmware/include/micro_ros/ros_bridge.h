// micro-ROS 桥接层接口（agent 通信生命周期 + rclc_lifecycle 状态机）。
#ifndef ROS_BRIDGE_H
#define ROS_BRIDGE_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool ros_bridge_create(void);
void ros_bridge_destroy(void);
void ros_bridge_spin(void);
bool ros_bridge_is_active(void);

#ifdef __cplusplus
}
#endif

#endif /* ROS_BRIDGE_H */
