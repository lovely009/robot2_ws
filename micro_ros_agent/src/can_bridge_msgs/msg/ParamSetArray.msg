# 稀疏参数写（上层 → MCU，reliable）：只影响 targets 里列出的 (motor_id, field)，
# 未列出的一律保持原状——**不是**完整快照语义（同 MotorPolicyArray 的判据）。
#
# 上界 16 是**内核约束不是带宽约束**：每个目标（每对位置限位算一次）一次
# motor_rt_submit_config 提交，而 CONTROL_MAILBOX_CONFIG_QUEUE_CAPACITY = 16
# （firmware lib/control_mailbox/include/control_mailbox.h）。参数面是**唯一**
# 逐目标进配置 FIFO 的下行路径（motor_command_array / motor_trajectory /
# motor_policy 都是整条快照一次提交），所以只有本消息暴露这个问题。取 16 保证
# "一条消息不可能自己把队列打满"——有 static_assert 守着（param_bridge.cpp）。
#
# **整条受理或整条拒绝**：任一目标不合法则整条丢弃、一个字节都不下发，与
# motor_command_array / motor_trajectory / motor_policy 同口径（内核那三个
# rejected 计数器同名同义）。不做部分受理——位置限位出现"下限进去了、上限没
# 进去"的半边状态正是本设计要消掉的东西。
#
# 位置限位必须**成对出现在同一条消息里**：只给一边 → 整条拒绝。一条消息能带
# 两边，没有半边暂存的存在理由（参数面时期的暂存是 rclc 一次只能 set 一个标量
# 的产物）。只想改一边时，另一边从 device_param_report 的 rec_pos_* 读回一起发。
# 同一 (motor_id, field) 在一条消息里出现两次 → 整条拒绝（静默取一个会掩盖
# 调用方 bug）。
#
# request_id 是**关联 id 不是去重键**：0 = 固件代分配（配置 FIFO 拒 0）；bit30
# 是固件自发脉冲 id 的命名空间、置位会被本地拒绝（bit31 已被内核身份读取占用）；
# 其余逐字进内核，随 /micro_ros_up/motor_events 的 CONFIG_RESULT 原样回告。
uint64 sync_time_ns
uint32 request_id
# 目标条数（= targets.size；不设它消费者会读到 0 条，口径同 MotorPolicyArray.count）
uint16 count
ParamSetTarget[<=16] targets
