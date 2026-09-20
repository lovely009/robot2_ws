# 多点轨迹（上层 → MCU）：100Hz 完整快照 MotorCommandArray 的**时序补充**
# 通道，不是替代品。MCU 在 20ms 控制边界对每条时间线确定性插值执行。
#
# 归属语义（与快照的仲裁，ARD §13.1 的补充）：target_ids 是本轨迹声明的
# 受控电机集合。轨迹在播期间是这些电机的**唯一**命令权威——同一时刻到达的
# 快照对它们无效（否则 100Hz 快照会立刻打死轨迹）。集合外的电机不受影响，
# 仍由 /motor_command_array 负责，"缺席即软停"语义不变。
#
# 空轨迹（count==0 && target_count==0）= 取消：释放全部所属电机并软停。
# 被新轨迹替换时，移出集合的电机立即解除归属。
#
# 时间基准：MCU 受理时刻即轨迹起点（本地单调时基）；sync_time_ns 仅用于
# 跨系统日志对齐，MCU 不读它做任何裁决（与 MotorCommandArray 同口径）。
# sequence：诊断/日志关联，不参与裁决。
#
# 校验：count<=256；time_from_start_ms 非递减（同一电机内严格递增）；每个
# 点的 motor_id 必须 ∈ target_ids；target_ids 内不得重复，且每台至少一个
# 路点；任何 NaN/Inf → 整条拒收。
#
# ⚠️ count 上限是**整条轨迹的总路点数**，不按电机分：4 台电机时只够 64 个
# 时刻。超出后下行链路是**静默丢弃**（ROS 桥在入口 return，不发事件也不
# 计数），上位机必须自己先判，别把"发出去了"当成"固件收到了"。
uint64 sync_time_ns
uint32 sequence
# 轨迹标识：上层分配，内核原样回带在 trajectory_state 与拒绝事件里。并发下发
# 多条轨迹时，没有它就无法回答「被拒的是哪一条、上一帧进度属于谁」。
uint32 trajectory_id
uint16 count                      # 路点数
uint16 target_count               # 本轨迹受控电机数
uint16[<=64] target_ids           # 受控电机集合
MotorTrajectoryPoint[<=256] points
