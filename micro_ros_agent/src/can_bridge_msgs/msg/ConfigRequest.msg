# 配置/系统命令面：ParamSetArray 与 SystemCommand 合一，用 kind 区分。
# 两组字段互斥：kind 决定哪组有效，另一组按下面的固定值填，否则整条拒绝
# （沿用「整条受理或整条拒绝」的旧口径）。PARAM_SET 要求 command = COMMAND_SAVE(0)、
# motor_id = MOTOR_ID_NONE(65535)；SYSTEM_COMMAND 要求 count = 0、targets 为空。
# 取值不在此定义：command 见 SystemCommand.msg 的 COMMAND_*，motor_id 缺省见其 MOTOR_ID_NONE。
# targets 上界 16 是内核约束不是带宽约束（CONTROL_MAILBOX_CONFIG_QUEUE_CAPACITY = 16）。
uint64 sync_time_ns
# 关联 id 不是去重键：0 = 固件代分配；bit30 = 固件自发脉冲命名空间（置位被本地拒绝）；
# bit31 已被内核身份读取占用。
uint32 request_id
uint8 CONFIG_KIND_PARAM_SET=0
uint8 CONFIG_KIND_SYSTEM_COMMAND=1
uint8 kind
# kind = CONFIG_KIND_PARAM_SET 时有效
uint16 count
ParamSetTarget[<=16] targets
# kind = CONFIG_KIND_SYSTEM_COMMAND 时有效（取值 = SystemCommand.msg 的 COMMAND_*）
uint8 command
# MOTOR_ID_NONE(65535) = 不适用
uint16 motor_id
