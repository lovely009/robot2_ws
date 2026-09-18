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
# 校验：count<=256；time_from_start_ms 严格递增；每个点的 motor_id 必须
# ∈ target_ids；target_ids 内不得重复；任何 NaN/Inf → 整条拒收。
uint64 sync_time_ns
uint32 sequence
# 轨迹标识：上层分配，内核原样回带在 trajectory_state 与拒绝事件里。并发下发
# 多条轨迹时，没有它就无法回答「被拒的是哪一条、上一帧进度属于谁」。
uint32 trajectory_id
uint16 count                      # 路点数
uint16 target_count               # 本轨迹受控电机数
uint16[<=64] target_ids           # 受控电机集合
MotorTrajectoryPoint[<=256] points
