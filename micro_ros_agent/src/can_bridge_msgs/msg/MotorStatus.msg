# DEPRECATED（Phase C）：joint_name/raw-CAN 依赖面，固件已停用；上层迁移完成后删除。

# 单台 vaxor 电机的 20 字节状态帧；joint_name 用于区分电机。
std_msgs/Header header
string joint_name
float32 voltage
uint32 fault
uint8 motor_state
uint8 calibration_state
uint8 mode
uint8 motor_type
