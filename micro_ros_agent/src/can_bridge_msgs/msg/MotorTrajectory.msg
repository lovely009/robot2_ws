# 多点轨迹（上层 → MCU）：MotorCommandArray 的**时序补充**通道，不是替代品。
# 上层可以把同一条轨迹拆成多个可靠分段；固件在收到完整分段组后才原子替换
# 当前轨迹。MCU 在 5ms 控制边界对完整时间线确定性插值执行。
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
# segment_index：从 0 开始的分段序号；segment_count：本轨迹总分段数。
# 同一 trajectory_id 的分段必须按序到达，时间仍是相对整条轨迹起点的绝对偏移。
# 固件拼接后的整条轨迹总路点数上限仍为 256；分段未收齐时旧轨迹继续运行。
# 分段边界没有共享时刻也允许：固件按绝对时间把两侧路点直接拼成同一时间线，
# 再在整条时间线上计算内部切矢，不把每个分段端点错误地当成零速端点。
#
# 校验：count<=本消息 points 容量；segment_index<segment_count；
# time_from_start_ms 在整条轨迹内非递减（同一电机内严格递增）；每个
# 点的 motor_id 必须 ∈ target_ids；target_ids 内不得重复，且每台至少一个
# 路点；任何 NaN/Inf → 整条拒收。
#
# count 是当前分段的路点数；固件拼接后整条轨迹的总路点数不超过 256，
# 超出或分段不连续时整组拒收，当前正在执行的旧轨迹不受影响。
uint64 sync_time_ns
uint32 sequence
# 轨迹标识：上层分配，内核原样回带在 trajectory_state 与拒绝事件里。并发下发
# 多条轨迹时，没有它就无法回答「被拒的是哪一条、上一帧进度属于谁」。
uint32 trajectory_id
uint16 segment_index              # 0-based 分段序号
uint16 segment_count              # 总分段数，单段轨迹为 1
uint16 count                      # 当前分段路点数
uint16 target_count               # 本轨迹受控电机数
uint16[<=64] target_ids           # 受控电机集合
MotorTrajectoryPoint[<=256] points
