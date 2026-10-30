# 只供主机监管入口使用，MCU 线协议保持不变。
std_msgs/Header header
string lease_id
uint64 epoch
uint64 session_id
uint64 sequence
can_bridge_msgs/MotorCommandArray command
