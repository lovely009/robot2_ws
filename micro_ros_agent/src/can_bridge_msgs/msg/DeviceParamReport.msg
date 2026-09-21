# 参数核对用的双行快照全集（MCU → 上层，低频）。
#
# 为什么不塞进 motor_status_array：那是每台 20 Hz 的流，加 12 个 float 会让每条
# 状态消息涨约 50 B/台（48 台约 2.4 KB/条），而参数值几乎不变；"USB CDC 上行
# 带宽余量"本就是验证计划的开放项。低频专用话题也更贴现有先例
# （MotorTrajectoryState 同为「MCU → 上层、低频」）。
#
# 写侧对偶是 /micro_ros_do/param_set（ParamSetArray）——本话题的槽位序就是它的
# field 取值，两行 rec_* 也正是"只想改一边时另一边读回一起发"的来源。
#
# 发布时机：状态变化时（内核 CONFIG_RESULT / DEV_INIT 事件触发）即时一帧 + 1 Hz
# 心跳。**可靠 QoS**——48 台时本消息约 3 KB，超过一个 MTU(512 B)，best-effort
# 流不分片会被静默丢弃（与 motor_topology 同一处理）。
uint64 sync_time_ns
# 台数（= entries.size；不设它消费者会读到 0 条，口径同 MotorTrajectoryState.target_count）。
uint16 count
DeviceParamEntry[<=64] entries
