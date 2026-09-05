# 命令快照元素（ARD-0007 §15.3）：uint16 motor_id + float32 SI 物理量。
# 高频物理量一律 float32（ARD §38.2）；缺失/占位语义不使用 NaN
# （命令侧 NaN/Inf = 整快照拒收，TBD-ROS-01=A）。
# effort 是唯一动态扭矩/前馈目标（TBD-ROS-03=A）；禁止 joint_name、
# Kp/Kd、Enable、raw CAN 字段（ARD §15.4）。
uint16 motor_id
float32 position     # rad，关节侧
float32 velocity     # rad/s，关节侧
float32 effort       # N·m，关节侧（= 前馈 τ_ff）
