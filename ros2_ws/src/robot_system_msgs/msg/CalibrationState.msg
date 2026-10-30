# 启动核验报告；commissioned 表示存在装配验收记录，不代表硬件安全认证。
std_msgs/Header header
string robot_id
string model_name
string calibration_id
string config_hash
bool valid
bool commissioned
# 以下四组数组按电机一一对应；joint_names 是模型关节清单，不要求逐电机一一对应。
uint16[] motor_ids
uint8[] bus_indices
uint8[] protocols
string[] model_names
string[] joint_names
string[] blockers
