// CAN1 调试桥接实现。
//
// 职责：把 FDCAN1 接收帧发布到 micro_ros/can1/receive，并把
// micro_ros/can1/send 的订阅帧写入 FDCAN1 TX FIFO。接收帧统一通过
// can1_rx_snapshot() 获取，不直接访问 can_bsp 内部缓冲。

#include "can_debug_bridge.h"

#include <Arduino.h>
#include <string.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <can_bridge_msgs/msg/can_frame.h>

#include "can_bsp.h"

#if defined(ENABLE_CAN1_DEBUG_BRIDGE) && ENABLE_CAN1_DEBUG_BRIDGE

static const char *kCan1SendTopic = "micro_ros/can1/send";
static const char *kCan1ReceiveTopic = "micro_ros/can1/receive";

static rcl_publisher_t g_can1_rx_pub;
static rcl_subscription_t g_can1_tx_sub;
static can_bridge_msgs__msg__CANFrame g_can1_rx_msg;
static can_bridge_msgs__msg__CANFrame g_can1_tx_msg;
static char g_can1_tx_frame_id_buf[8];  // 订阅反序列化需要预分配字符串
static uint32_t g_last_rx_seq = 0;
static bool g_ready = false;
static bool g_sub_added = false;

static void fill_can1_rx_msg(const can_rx_frame_t *frame) {
  // 时间戳沿用 IMU 的 millis 方案；frame_id 标记来源总线
  uint32_t ms = millis();
  g_can1_rx_msg.header.stamp.sec = (int32_t)(ms / 1000u);
  g_can1_rx_msg.header.stamp.nanosec = (uint32_t)((ms % 1000u) * 1000000u);
  g_can1_rx_msg.header.frame_id.data = (char *)"can1";
  g_can1_rx_msg.header.frame_id.size = 4;
  g_can1_rx_msg.header.frame_id.capacity = 5;

  g_can1_rx_msg.id = (uint16_t)frame->id;
  g_can1_rx_msg.dlc = frame->dlc;
  memset(g_can1_rx_msg.data, 0, sizeof(g_can1_rx_msg.data));
  memcpy(g_can1_rx_msg.data, frame->data, frame->dlc);
  g_can1_rx_msg.is_extended = frame->is_extended;
  g_can1_rx_msg.is_fd = frame->is_fd;
}

// 收到 ROS 侧 CANFrame 后写入 FDCAN1 TX FIFO（micro-ROS 任务上下文，可阻塞）
static void can1_tx_sub_cb(const void *msg) {
  const can_bridge_msgs__msg__CANFrame *frame =
      (const can_bridge_msgs__msg__CANFrame *)msg;
  fdcanx_send_frame(&hfdcan1, frame->id, frame->data, frame->dlc,
                    frame->is_extended, frame->is_fd);
}

bool can_debug_bridge_create(rcl_node_t *node, rclc_executor_t *executor) {
  if (rclc_publisher_init_default(&g_can1_rx_pub, node,
                                  ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg, CANFrame),
                                  kCan1ReceiveTopic) != RCL_RET_OK) {
    return false;
  }
  if (rclc_subscription_init_default(&g_can1_tx_sub, node,
                                     ROSIDL_GET_MSG_TYPE_SUPPORT(can_bridge_msgs, msg, CANFrame),
                                     kCan1SendTopic) != RCL_RET_OK) {
    rcl_ret_t rc = rcl_publisher_fini(&g_can1_rx_pub, node);
    (void)rc;
    return false;
  }
  // micro-ROS 不会为 NULL 字符串分配空间：订阅缓冲须预分配
  g_can1_tx_msg.header.frame_id.data = g_can1_tx_frame_id_buf;
  g_can1_tx_msg.header.frame_id.size = 0;
  g_can1_tx_msg.header.frame_id.capacity = sizeof(g_can1_tx_frame_id_buf);

  if (rclc_executor_add_subscription(executor, &g_can1_tx_sub, &g_can1_tx_msg,
                                     can1_tx_sub_cb, ON_NEW_DATA) != RCL_RET_OK) {
    rcl_ret_t rc = rcl_subscription_fini(&g_can1_tx_sub, node);
    rc = rcl_publisher_fini(&g_can1_rx_pub, node);
    (void)rc;
    return false;
  }
  g_sub_added = true;
  g_ready = true;
  return true;
}

void can_debug_bridge_destroy(rcl_node_t *node, rclc_executor_t *executor) {
  if (!g_ready && !g_sub_added) {
    return;
  }
  g_ready = false;
  if (g_sub_added) {
    rcl_ret_t rc = rclc_executor_remove_subscription(executor, &g_can1_tx_sub);
    (void)rc;
    g_sub_added = false;
  }
  rcl_ret_t rc = rcl_subscription_fini(&g_can1_tx_sub, node);
  rc = rcl_publisher_fini(&g_can1_rx_pub, node);
  (void)rc;
}

void can_debug_bridge_poll(void) {
  if (!g_ready) {
    return;
  }
  can_rx_frame_t frame = {0};
  uint32_t seq = can1_rx_snapshot(&frame);
  if (seq == g_last_rx_seq) {
    return;
  }
  g_last_rx_seq = seq;
  fill_can1_rx_msg(&frame);
  rcl_ret_t rc = rcl_publish(&g_can1_rx_pub, &g_can1_rx_msg, nullptr);
  (void)rc;
}

#else

bool can_debug_bridge_create(rcl_node_t *node, rclc_executor_t *executor) {
  (void)node;
  (void)executor;
  return true;
}

void can_debug_bridge_destroy(rcl_node_t *node, rclc_executor_t *executor) {
  (void)node;
  (void)executor;
}

void can_debug_bridge_poll(void) {}

#endif  // ENABLE_CAN1_DEBUG_BRIDGE
