# MCU 运动反馈，频率由 FIRMWARE_MOTOR_REPORT_HZ 配置。
# 只含运动量；诊断/故障走 MotorSystemState。
std_msgs/Header header
MotorMotionStateTarget[<=37] states
uint64 session_id
uint32 tick_ms
uint32 seq
