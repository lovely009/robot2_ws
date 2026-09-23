# 全量设备拓扑（ARD-0007 §18）：发现窗口完成后整包发布，拓扑变化后刷新。
# 低频；事件级的即时变化走 MotorEvent。
#
# 本包承载电机身份与总线映射的权威：谁在、在哪条总线、什么协议、绑了哪个型号、
# 能力位、是否冲突。MotorStatusTarget 为便于按 20Hz 状态流显示也重复提供 bus_index，
# 但总线映射身份仍以此拓扑为准。生命周期/电气状态在 MotorStatusTarget，运动学在
# MotorMotionStateTarget。
#
# schema_version：消息面契约版本。上层据此检测漂移，而不是静默按旧布局解析
# （字段只增不改序，故版本只在语义变更时递增）。
uint64 sync_time_ns
uint16 schema_version
MotorTopologyEntry[<=64] entries
