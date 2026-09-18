# 逐电机策略（上层 → MCU）：动态量 enable/kp/kd。
#
# **稀疏语义**：只影响 targets 里列出的电机，未列出的保持原状。这**不是**
# MotorCommandArray 的完整快照语义——「缺席即软停」（ARD §13.1）只适用于
# 命令快照，这里套用会造成「没提到就被失能」的意外。
#
# 为什么不放进 100Hz 命令快照：ARD §15.4/§19.4 明令禁止 Kp/Kd/Enable 进入
# MotorCommandArray——Policy 更新不得刷新运动 freshness。
#
# policy_generation 防「周期重发自动复权」：内核在表项进入 FAULT / OFFLINE /
# CONFIG_CONFLICT 时锁存当时的代数，enabled=true 且代数 <= 锁存值的一律拒绝。
# 于是 ARD §36 要求的「显式重新使能」必须是**新意图**，而不是同一条消息的
# 重复发布。上层每次真正想改变意图时递增本字段即可。
#
# 结果不在此话题回：每个电机的受理/拒绝经 /micro_ros/events 的 CONFIG_RESULT
# 事件给出，用 request_id 关联。
uint64 sync_time_ns
uint32 request_id
uint32 policy_generation
uint16 count
MotorPolicyTarget[<=64] targets
