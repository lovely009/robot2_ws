#ifndef MICRO_ROS_TASK_H
#define MICRO_ROS_TASK_H

#include <stdbool.h>

#include "micro_ros_link_state.h"

#ifdef __cplusplus
extern "C" {
#endif

bool micro_ros_task_start(void);
micro_ros_link_state_t micro_ros_task_get_link_state(void);
bool micro_ros_task_ever_connected(void);

#ifdef __cplusplus
}
#endif

#endif