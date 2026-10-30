# 策略意图仍需 MCU 确认；监管者分配最终 policy_generation。
std_msgs/Header header
string lease_id
uint64 epoch
uint64 session_id
uint64 sequence
can_bridge_msgs/MotorPolicyArray policy
