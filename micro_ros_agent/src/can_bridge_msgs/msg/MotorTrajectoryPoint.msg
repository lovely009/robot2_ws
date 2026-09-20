# 轨迹路点样本（ARD-0007 §15 的时序补充通道；同 §14.2 约束：固定容量、
# SI 单位、无 joint_name、无厂家协议字节）。
#
# time_from_start_ms：相对**本轨迹起点**的毫秒偏移。起点由 MCU 在受理时以
# 本地单调时基落定，绝不用 ROS 时间做执行判定（ARD §12.2）。本字段是排序
# 键：整表**非递减**，同一电机内**严格递增**（不同电机允许共用同一时刻——
# 并行的关节本就同拍采样，强行要求全局递增会逼上层去重时间轴）。
#
# motor_id 必须属于所属 MotorTrajectory.target_ids 声明的受控集合。
#
# 数值语义同 MotorCommandTarget：NaN/Inf → 整条轨迹拒收，不做占位语义。
# position 是执行量：段内**单调三次 Hermite**（Fritsch–Carlson 切矢，位置与
# 速度都连续）；velocity 仅供诊断与上层自检——执行侧速度由位置时间线解析
# 求导得来（上层填 0 是常见写法，直通会错误地让 Kd 刹车）；
# effort 为前馈 τ_ff，段内保持左端点值。
uint32 time_from_start_ms
uint16 motor_id
float32 position     # rad，关节侧
float32 velocity     # rad/s，关节侧（诊断/自检用）
float32 effort       # N·m，关节侧（= 前馈 τ_ff）
