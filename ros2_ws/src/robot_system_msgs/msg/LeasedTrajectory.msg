# 只供主机监管入口使用；取消语义沿用 MotorTrajectory 的空轨迹。
std_msgs/Header header
string lease_id
uint64 epoch
uint64 session_id
uint64 sequence
can_bridge_msgs/MotorTrajectory trajectory
