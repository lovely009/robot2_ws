# 固件级诊断（MCU → 上层，1Hz）。
#
# 这些计数器此前只存在于 MCU 的运行上下文里，一个都不对外——实时性与总线
# 容量的问题因此只能靠 JTAG 定位。全部是**单调递增的生命周期累计**，上层
# 自己求差分即可。
#
# 计数含义与「谁该看哪个」：
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
