# 固件级诊断（MCU → 上层，1Hz）。
#
# 计数器为单调递增的生命周期累计值，上层求差分。
#
# 计数含义：
#   rx_frame_count        收到的原始帧总数（含未归属的）
#   unknown_frame_count   无法归属到任何注册设备的帧
#   decode_error_count    归属到设备但解码失败的帧
#   snapshot_accepted / snapshot_rejected   完整命令快照的受理/整条拒绝
#   config_rejected       被内核门禁拒绝的配置请求
#   event_dropped         事件环满而挤掉的最旧事件（>0 表示上层消费太慢）
#   tx_dropped            内核待发缓冲满而丢弃的帧（P1 路径；P0 不丢，改重试）
#   p0_retry              P0 安全帧因缓冲满而挂起重试的次数
#   trajectory_accepted / trajectory_rejected  轨迹的受理/整条拒绝
#   ring_drop             每总线 RX ring 溢出丢弃数（索引 = bus_index）
uint64 sync_time_ns
uint32 rx_frame_count
uint32 unknown_frame_count
uint32 decode_error_count
uint32 snapshot_accepted
uint32 snapshot_rejected
uint32 config_rejected
uint32 event_dropped
uint32 tx_dropped
uint32 p0_retry
uint32 trajectory_accepted
uint32 trajectory_rejected
uint32 policy_accepted
uint32 policy_rejected
uint32[<=3] ring_drop
uint32 heap_free

# mailbox 层拒绝：尚未进入内核的非法快照、时间递减或越权轨迹等请求。
uint32 mailbox_rejected_command
uint32 mailbox_rejected_trajectory
uint32 mailbox_rejected_policy
# 出站发布失败总数，含 best-effort 单条超过传输上限的丢弃。
uint32 publish_fail
# 每总线硬件 RX FIFO0 的 MESSAGE_LOST 计数（帧不可恢复；FIFO 只有 8 深）。
uint32[<=3] rx_hw_lost
# 每总线成功提交硬件的帧数，在 HAL_FDCAN_AddMessageToTxFifoQ 成功处累加，含 SYNC。
uint32[<=3] tx_frame_count
