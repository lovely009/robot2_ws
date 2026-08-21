# 单电机命令（每电机一条，joint_name 区分 RP/vaxor）
# 单位与 ros2_ws 一致：rad、rad/s、A/Nm；enable 变化时固件同步使能/失能
string joint_name   # 关节名，与 ros2_ws YAML 对齐（如 left_shoulder_yaw_joint）
float64 position    # 目标位置（rad）
float64 velocity    # 目标速度（rad/s）
float64 kp          # 位置增益
float64 kd          # 速度增益
float64 feedforward # 前馈（RP: Nm；vaxor: 电流 A）
bool enable         # true=使能，false=失能