#ifndef MICRO_ROS_LINK_STATE_H
#define MICRO_ROS_LINK_STATE_H

// micro-ROS Agent 连接状态（micro_ros 任务持有，LED 任务只读消费）。
// 注意这是 rclc 传输生命周期，与电机安全授权是两层不同状态：
// WAIT_AGENT/从未连接属"状态未知"，安全侧按引导豁免处理。
typedef enum {
  MICRO_ROS_LINK_WAIT_AGENT, // 等 Agent 上电（持续重连）
  MICRO_ROS_LINK_CONNECT,    // 传输已连，正在建 ROS 实体
  MICRO_ROS_LINK_RUN,        // 实体就绪，正常运行
  MICRO_ROS_LINK_LOST,       // 曾连接后断开（安全侧据此立即撤权）
} micro_ros_link_state_t;

#endif