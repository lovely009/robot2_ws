# 设备身份绑定请求（TBD-PROFILE-01 过渡态：识别源关闭前由上层显式绑定型号）。
# 结果经 micro_ros/motor_events 的 EVENT_PROFILE_BOUND(1) / EVENT_PROFILE_REJECTED(2)
# 回告（以 motor_id 关联；绑定幂等，重发无害，故不带 request_id）。
# 断链保留绑定：绑定是身份事实，重连后无需重发。
# model_name 锚点：与 ROS 型号库 / 固件编译期型号目录同名（TBD-PROFILE-01）。
uint16 motor_id
string<=31 model_name
